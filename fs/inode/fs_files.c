/****************************************************************************
 * fs/inode/fs_files.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <execinfo.h>
#include <sched.h>
#include <errno.h>
#include <fcntl.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/cancelpt.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/spawn.h>
#include <nuttx/spinlock.h>
#include <nuttx/lib/lib.h>

#ifdef CONFIG_FDSAN
#  include <android/fdsan.h>
#endif

#ifdef CONFIG_FDCHECK
#  include <nuttx/fdcheck.h>
#endif

#include "sched/sched.h"
#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Local helper structure to atomically read flags from file descriptor */

struct fd_flags
{
  bool     f_cloexec;     /* Close on exec */
#ifdef CONFIG_FDCHECK
  uint8_t  f_tag_fdcheck; /* File owner fdcheck tag, init to 0 */
#endif
#ifdef CONFIG_FDSAN
  uint64_t f_tag_fdsan;   /* File owner fdsan tag, init to 0 */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fget_by_index
 ****************************************************************************/

static FAR struct file *fget_by_index(FAR struct fdlist *list,
                                      int l1, int l2,
                                      FAR struct fd_flags *fdflags)
{
  FAR struct file *filep;
  FAR struct fd *fdp;
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  /* The callers have ensured that no out of bounds access can happen here */

  fdp = &list->fl_fds[l1][l2];

  /* The file pointer is removed before the file is freed, so there is no
   * race condition here as long as we take a reference to the file before
   * releasing file list lock.
   */

  filep = fdp->f_file;
  if (filep != NULL)
    {
      fs_reffilep(filep);
      if (fdflags != NULL)
        {
          fdflags->f_cloexec     = fdp->f_cloexec;
#ifdef CONFIG_FDSAN
          fdflags->f_tag_fdsan   = fdp->f_tag_fdsan;
#endif
#ifdef CONFIG_FDCHECK
          fdflags->f_tag_fdcheck = fdp->f_tag_fdcheck;
#endif
        }
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

  return filep;
}

/****************************************************************************
 * Name: fget_by_fd
 ****************************************************************************/

static FAR struct file *fget_by_fd(FAR struct fdlist *list, int fd,
                                   FAR struct fd_flags *fdflags)
{
  return fget_by_index(list, fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK,
                       fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK, fdflags);
}

/****************************************************************************
 * Name: fdlist_finstall
 ****************************************************************************/

static FAR struct file *fdlist_finstall(FAR struct fdlist *list, int fd,
                                        FAR struct file *filep,
                                        FAR struct fd_flags *fdflags)
{
  int i = fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  int j = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  FAR struct file *oldfilep;
  irqstate_t flags;

  /* Atomically replace the file pointer */

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  oldfilep = list->fl_fds[i][j].f_file;
  list->fl_fds[i][j].f_file = filep;

  if (fdflags != NULL)
    {
      list->fl_fds[i][j].f_cloexec     = fdflags->f_cloexec;
#ifdef CONFIG_FDSAN
      list->fl_fds[i][j].f_tag_fdsan   = fdflags->f_tag_fdsan;
#endif
#ifdef CONFIG_FDCHECK
      list->fl_fds[i][j].f_tag_fdcheck = fdflags->f_tag_fdcheck;
#endif
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

  /* Return the old file pointer */

  return oldfilep;
}

/****************************************************************************
 * Name: fdlist_extend
 ****************************************************************************/

static int fdlist_extend(FAR struct fdlist *list, size_t row)
{
  FAR struct fd **fds;
  uint8_t orig_rows;
  FAR void *tmp;
  int flags;
  int i;
  int j;

  orig_rows = list->fl_rows;
  if (row <= orig_rows)
    {
      return 0;
    }

  if (CONFIG_NFILE_DESCRIPTORS_PER_BLOCK * orig_rows > OPEN_MAX)
    {
      fdlist_dump(list);
      return -EMFILE;
    }

  fds = fs_heap_malloc(sizeof(FAR struct fd *) * row);
  DEBUGASSERT(fds);
  if (fds == NULL)
    {
      return -ENFILE;
    }

  i = orig_rows;
  do
    {
      fds[i] = fs_heap_zalloc(sizeof(struct fd) *
                              CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
      if (fds[i] == NULL)
        {
          while (--i >= orig_rows)
            {
              fs_heap_free(fds[i]);
            }

          fs_heap_free(fds);
          return -ENFILE;
        }
    }
  while (++i < row);

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  /* To avoid race condition, if the file list is updated by other threads
   * and list rows is greater or equal than temp list,
   * release the obsolete buffers
   */

  if (orig_rows != list->fl_rows && list->fl_rows >= row)
    {
      spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

      for (j = orig_rows; j < i; j++)
        {
          fs_heap_free(fds[j]);
        }

      fs_heap_free(fds);

      return OK;
    }

  if (list->fl_fds != NULL)
    {
      memcpy(fds, list->fl_fds, list->fl_rows * sizeof(FAR struct fd *));
    }

  tmp = list->fl_fds;
  list->fl_fds = fds;
  list->fl_rows = row;

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

  if (tmp != NULL && tmp != &list->fl_prefd)
    {
      fs_heap_free(tmp);
    }

  return OK;
}

static void task_fssync(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct tcb_s *ctcb;
  FAR struct file *filep;
  int pid = tcb->pid;
  uint8_t rows;
  int i;
  int j;

  if (tcb->group == NULL)
    {
      return;
    }

  rows = tcb->group->tg_fdlist.fl_rows;

  for (i = 0; i < rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          ctcb = nxsched_get_tcb(pid);
          if (ctcb == NULL || ctcb->group == NULL || ctcb != tcb)
            {
              return;
            }

          filep = fget_by_index(&ctcb->group->tg_fdlist, i, j, NULL);
          if (filep != NULL)
            {
              file_fsync(filep);
              ctcb = nxsched_get_tcb(pid);
              if (ctcb != NULL && ctcb->group != NULL && ctcb == tcb)
                {
                  fs_putfilep(filep);
                }
            }
        }
    }
}

/****************************************************************************
 * Name: fs_closefilep
 *
 * Description:
 *   Close an existing file pointer and free the allocated memory.
 *
 * Input Parameters:
 *   filep - The file pointer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int fs_closefilep(FAR struct file *filep)
{
  int ret;

  ret = file_close(filep);
  if (ret >= 0)
    {
      fs_heap_free(filep);
    }

  return ret;
}

/****************************************************************************
 * Name: file_free_from_tcb
 *
 * Description:
 *   Remove a struct files instance from a file descriptor.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

static int file_free_from_tcb(FAR struct tcb_s *tcb, FAR struct file *filep,
                              int fd)
{
  int i = fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  int j = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  FAR struct fdlist *list;
  irqstate_t flags;

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_fdlist_from_tcb(tcb);

  /* Perform the protected close operation */

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  /* Unlink the file description from the descriptor atomically */

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  /* There is a race condition here: someone might have freed and installed
   * a new file for the descriptor, as the file list lock was released after
   * taking the file pointer. In this case, do not touch the file.
   */

  if (list->fl_fds[i][j].f_file == filep)
    {
      list->fl_fds[i][j].f_file = NULL;
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdlist_init
 *
 * Description: Initializes the list of file descriptors for a new task.
 *
 ****************************************************************************/

void fdlist_init(FAR struct fdlist *list)
{
  /* The first row will reuse pre-allocated files, which will avoid
   * unnecessary allocator accesses during file initialization.
   */

  list->fl_rows = 1;
  list->fl_fds = &list->fl_prefd;
  list->fl_prefd = list->fl_prefds;
  spin_lock_init(&list->fl_lock);
}

/****************************************************************************
 * Name: fdlist_dump
 *
 * Description:
 *   Dump the list of file descriptors.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_DUMP_ON_EXIT
void fdlist_dump(FAR struct fdlist *list)
{
  FAR char *path;
  int count = fdlist_count(list);
  int i;

  syslog(LOG_INFO, "%-4s%-4s%-8s%-5s%-10s%-14s"
#if CONFIG_FS_BACKTRACE > 0
        " BACKTRACE"
#endif
        "\n",
        "PID", "FD", "FLAGS", "TYPE", "POS", "PATH"
        );

  path = lib_get_pathbuffer();
  if (path == NULL)
    {
      return;
    }

  for (i = 0; i < count; i++)
    {
      FAR struct file *filep = fdlist_fget(list, i);

#if CONFIG_FS_BACKTRACE > 0
      char buf[BACKTRACE_BUFFER_SIZE(CONFIG_FS_BACKTRACE)];
      FAR struct fd *fdp = fdlist_fdget(list, i);
#endif

      /* Is there an inode associated with the file descriptor? */

      if (filep == NULL || filep->f_inode == NULL)
        {
          continue;
        }

      if (file_ioctl(filep, FIOC_FILEPATH, path) < 0)
        {
          path[0] = '\0';
        }

#if CONFIG_FS_BACKTRACE > 0
      backtrace_format(buf, sizeof(buf), fdp->f_backtrace,
                       CONFIG_FS_BACKTRACE);
#endif

      syslog(LOG_INFO, "%-4d%-4d%-8d%-5x%-10ld%-14s"
#if CONFIG_FS_BACKTRACE > 0
            " %s"
#endif
            "\n", getpid(), i, filep->f_oflags,
            INODE_GET_TYPE(filep->f_inode),
            (long)filep->f_pos, path
#if CONFIG_FS_BACKTRACE > 0
            , buf
#endif
            );
      fs_putfilep(filep);
    }

  lib_put_pathbuffer(path);
}
#endif

/****************************************************************************
 * Name: fdlist_put
 *
 * Description:
 *   Release the list of file descriptors.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.
 *
 ****************************************************************************/

void fdlist_put(FAR struct fdlist *list)
{
  int i;
  int j;

  /* Close each file descriptor .. Normally, you would need take the list
   * mutex, but it is safe to ignore the mutex in this context
   * because there should not be any references in this context.
   */

  for (i = list->fl_rows - 1; i >= 0; i--)
    {
      for (j = CONFIG_NFILE_DESCRIPTORS_PER_BLOCK - 1; j >= 0; j--)
        {
          if (list->fl_fds[i][j].f_file != NULL)
            {
              fs_putfilep(list->fl_fds[i][j].f_file);
            }
        }

      if (i != 0)
        {
          fs_heap_free(list->fl_fds[i]);
        }
    }

  if (list->fl_fds != &list->fl_prefd)
    {
      fs_heap_free(list->fl_fds);
    }
}

/****************************************************************************
 * Name: fdlist_count
 *
 * Description:
 *   Get file descriptor count from file list.
 *
 * Input Parameters:
 *   list - Pointer to the file descriptor list structure.
 *
 * Returned Value:
 *   file count of file list.
 *
 ****************************************************************************/

int fdlist_count(FAR struct fdlist *list)
{
  return list->fl_rows * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
}

/****************************************************************************
 * Name: fdlist_dup
 *
 * Description:
 *   Duplicate parent task's file descriptors.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int fdlist_dup(FAR struct fdlist *plist, FAR struct fdlist *clist,
               FAR const posix_spawn_file_actions_t *actions,
               bool cloexec)
{
  bool fcloexec;
  int ret;
  int fd;
  int i;
  int j;

  for (i = 0; i < plist->fl_rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          FAR struct file *filep;
          struct fd_flags fdflags;

          fd = i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;
#ifdef CONFIG_FDCLONE_STDIO

          /* Determine how many file descriptors to clone.  If
           * CONFIG_FDCLONE_DISABLE is set, no file descriptors will be
           * cloned.  If CONFIG_FDCLONE_STDIO is set, only the first
           * three descriptors (stdin, stdout, and stderr) will be
           * cloned.  Otherwise all file descriptors will be cloned.
           */

          if (fd >= 3)
            {
              return OK;
            }
#endif

          /* Find the file from the parent's list taking a reference to
           * filep and reading the descriptor flags atomically.
           */

          filep = fget_by_fd(plist, fd, &fdflags);
          if (filep == NULL)
            {
              continue;
            }

          /* Read the close on exec flag from the parent list */

          fcloexec = (cloexec && fdflags.f_cloexec);

          /* Skip file dup if file action is unnecessary to duplicate */

          if (actions != NULL)
            {
#ifdef CONFIG_FDCHECK
              fd = fdcheck_protect(fd);
#endif
              if (!spawn_file_is_duplicateable(actions, fd, fcloexec))
                {
                  fs_putfilep(filep);
                  continue;
                }
            }
          else if (fcloexec)
            {
              fs_putfilep(filep);
              continue;
            }

          ret = fdlist_extend(clist, i + 1);
          if (ret < 0)
            {
              fs_putfilep(filep);
              return ret;
            }

          /* Assign filep to the child's descriptor list. Omit the flags */

          fdlist_finstall(clist, fd, filep, NULL);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: fdlist_fget
 *
 * Description:
 *   Get the instance of struct file from file descriptor list by file
 *   descriptor.
 *
 * Input Parameters:
 *   list - The list of files for a task.
 *   fd   - A valid descriptor between 0 and fdlist_count(list).
 *
 * Returned Value:
 *   Pointer to file structure of list[fd].
 *
 ****************************************************************************/

FAR struct file *fdlist_fget(FAR struct fdlist *list, int fd)
{
  return fget_by_fd(list, fd, NULL);
}

/****************************************************************************
 * Name: fdlist_fget
 *
 * Description:
 *   Get the instance of struct fd from file descriptor list by file
 *   descriptor.
 *
 * Input Parameters:
 *   list - The list of files for a task.
 *   fd   - A valid descriptor between 0 and fdlist_count(list).
 *
 * Returned Value:
 *   Pointer to file structure of list[fd].
 *
 ****************************************************************************/

FAR struct fd *fdlist_fdget(FAR struct fdlist *list, int fd)
{
  return &list->fl_fds[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                      [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK];
}

/****************************************************************************
 * Name: fdlist_getfree
 *
 * Description:
 *   Get next free file descriptor number, starting from minfd.
 *
 * Input Parameters:
 *   list  - The list of files for a task.
 *   minfd - The lowest acceptable file descriptor number.
 *
 * Returned Value:
 *   Returns the file descriptor == index into the files array on success;
 *   a negated errno value is returned on any failure.
 *
 * Assumptions:
 *   File list lock (fl_lock) is taken and local interrupts are disabled
 *   when this is called.
 *
 ****************************************************************************/

int fdlist_getfree(FAR struct fdlist *list, int minfd)
{
  int i = minfd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  int j = minfd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  FAR struct fd *fdp;
  int ret;

  for (; ; i++, j = 0)
    {
      if (i >= list->fl_rows)
        {
          /* We know local interrupts are disabled, but we must temporarily
           * release the file list lock.
           */

          spin_unlock(&list->fl_lock);

          ret = fdlist_extend(list, i + 1);

          /* Re-acquire the lock */

          spin_lock(&list->fl_lock);

          if (ret < 0)
            {
              return ret;
            }
        }

      do
        {
          fdp = &list->fl_fds[i][j];
          if (fdp->f_file == NULL)
            {
              goto found;
            }
        }
      while (++j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
    }

found:
  return i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;
}

/****************************************************************************
 * Name: file_allocate_from_tcb
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode
 *   instance.
 *
 * Returned Value:
 *   Returns the file descriptor == index into the files array on success;
 *   a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int file_allocate_from_tcb(FAR struct tcb_s *tcb, FAR struct inode *inode,
                           int oflags, off_t pos, FAR void *priv, int minfd,
                           bool addref)
{
  FAR struct fdlist *list;
  FAR struct file *filep;
  FAR struct fd *fdp;
  irqstate_t flags;
  int ret;

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_fdlist_from_tcb(tcb);

  /* Allocate a new file pointer */

  filep = fs_heap_malloc(sizeof(struct file));
  if (filep == NULL)
    {
      return -ENFILE;
    }

  /* Find free file descriptor */

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  ret = fdlist_getfree(list, minfd);
  if (ret < 0)
    {
      goto errout_with_fp;
    }

  fdp = fdlist_fdget(list, ret);
  if (fdp->f_file == NULL)
    {
      fdp->f_file          = filep;
      filep->f_oflags      = oflags & ~O_CLOEXEC;
      filep->f_pos         = pos;
      filep->f_inode       = inode;
      filep->f_priv        = priv;
 #ifdef CONFIG_FS_REFCOUNT
      atomic_set(&filep->f_refs, 1);
 #endif
 #ifdef CONFIG_FDSAN
      fdp->f_tag_fdsan     = 0;
 #endif
 #ifdef CONFIG_FDCHECK
      fdp->f_tag_fdcheck   = 0;
 #endif
      fdp->f_cloexec       = oflags & O_CLOEXEC ? true : false;
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

  if (addref)
    {
      inode_addref(inode);
    }

  FS_ADD_BACKTRACE(fdp);

#ifdef CONFIG_FDCHECK
  return fdcheck_protect(ret);
#else
  return ret;
#endif

errout_with_fp:
  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  fs_heap_free(filep);
  return ret;
}

/****************************************************************************
 * Name: file_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode
 *   instance.
 *
 * Returned Value:
 *     Returns the file descriptor == index into the files array on success;
 *     a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int file_allocate(FAR struct inode *inode, int oflags, off_t pos,
                  FAR void *priv, int minfd, bool addref)
{
  return file_allocate_from_tcb(this_task(), inode, oflags,
                                pos, priv, minfd, addref);
}

/****************************************************************************
 * Name: nx_dup
 *
 * Description:
 *   Clone a file or socket descriptor to an arbitrary descriptor number
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int nx_dup(int fd, int minfd, int oflags)
{
  FAR struct fdlist *list;
  FAR struct file *filep;
  FAR struct fd *fdp;
  irqstate_t flags;
  int fd2;

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_fdlist();

  /* Perform some sanity checks */

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  /* Get the file structure corresponding to the file descriptor. */

  filep = fget_by_fd(list, fd, NULL);
  if (filep == NULL)
    {
      return -EBADF;
    }

  /* Find free file descriptor */

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  fd2 = fdlist_getfree(list, minfd);
  if (fd2 >= 0)
    {
      fdp            = fdlist_fdget(list, fd2);
      fdp->f_file    = filep;
      fdp->f_cloexec = oflags & O_CLOEXEC ? true : false;
    }
  else
    {
      fs_putfilep(filep);
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  return fd2;
}

/****************************************************************************
 * Name: fs_getfilep
 *
 * Description:
 *   Given a file descriptor, return the corresponding instance of struct
 *   file.
 *
 * Input Parameters:
 *   fd    - The file descriptor
 *   filep - The location to return the struct file instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int fs_getfilep(int fd, FAR struct file **filep)
{
  FAR struct fdlist *list;

#ifdef CONFIG_FDCHECK
  fd = fdcheck_restore(fd);
#endif

  *filep = NULL;

  list = nxsched_get_fdlist();

  /* The file list can be NULL under two cases:  (1) One is an obscure
   * cornercase:  When memory management debug output is enabled.  Then
   * there may be attempts to write to stdout from malloc before the group
   * data has been allocated.  The other other is (2) if this is a kernel
   * thread.  Kernel threads have no allocated file descriptors.
   */

  if (list == NULL)
    {
      return -EAGAIN;
    }

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  /* The descriptor is in a valid range to file descriptor... Get the
   * thread-specific file list.
   */

  *filep = fdlist_fget(list, fd);

  /* if *filep is NULL, fd was closed */

  if (*filep == NULL)
    {
      return -EBADF;
    }

  return OK;
}

/****************************************************************************
 * Name: fs_reffilep
 *
 * Description:
 *   To specify filep increase the reference count.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_REFCOUNT
void fs_reffilep(FAR struct file *filep)
{
  /* This interface is used to increase the reference count of filep */

  DEBUGASSERT(filep);
  atomic_fetch_add(&filep->f_refs, 1);
}

/****************************************************************************
 * Name: fs_putfilep
 *
 * Description:
 *   Handles reference counts for files, less than or equal to 0 and close
 *   the file
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 ****************************************************************************/

int fs_putfilep(FAR struct file *filep)
{
  int ret = 0;

  DEBUGASSERT(filep);

  /* If refs is zero, the close() had called, closing it now. */

  if (atomic_fetch_sub(&filep->f_refs, 1) == 1)
    {
      ret = fs_closefilep(filep);
      if (ret < 0)
        {
          ferr("ERROR: fs putfilep file_close() failed: %d\n", ret);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: fs_setcloseonexec
 *
 * Description:
 *   Given a file descriptor, set the close on exec flag.
 *
 * Input Parameters:
 *   fd   - The file descriptor
 *   flag - The close on exec flag
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int fs_setcloseonexec(int fd, int flag)
{
  int i = fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  int j = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  FAR struct fdlist *list;
  irqstate_t flags;
  int ret = OK;

  list = nxsched_get_fdlist();

  /* Perform some sanity checks */

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  if (list->fl_fds[i][j].f_file == NULL)
    {
      ret = -EBADF;
    }
  else if (flag == FD_CLOEXEC)
    {
      list->fl_fds[i][j].f_cloexec = true;
    }
  else
    {
      list->fl_fds[i][j].f_cloexec = false;
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: fs_getcloseonexec
 *
 * Description:
 *   Given a file descriptor, get the close on exec flag.
 *
 * Input Parameters:
 *   fd   - The file descriptor
 *
 * Returned Value:
 *   A non-negative value on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int fs_getcloseonexec(int fd)
{
  int i = fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  int j = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  FAR struct fdlist *list;
  irqstate_t flags;
  int ret;

  list = nxsched_get_fdlist();

  /* Perform some sanity checks */

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  if (list->fl_fds[i][j].f_file == NULL)
    {
      ret = -EBADF;
    }
  else
    {
      ret = list->fl_fds[i][j].f_cloexec ? FD_CLOEXEC : 0;
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: nx_dup2_from_tcb
 *
 * Description:
 *   nx_dup2_from_tcb() is similar to the standard 'dup2' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   nx_dup2_from_tcb() is an internal NuttX interface and should not be
 *   called from applications.
 *
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2_from_tcb(FAR struct tcb_s *tcb, int fd1, int fd2)
{
  /* If fd1 is a valid file descriptor, and fd1 == fd2, then dup2() does
   * nothing, and returns fd2.
   */

  if (fd1 == fd2)
    {
      FAR struct fdlist *list;
      FAR struct file *filep;

      /* Get the file descriptor list. */

      list  = nxsched_get_fdlist_from_tcb(tcb);

      /* Get file behind fd1, and taking a reference */

      filep = fget_by_fd(list, fd1, NULL);
      if (filep == NULL)
        {
          return -EBADF;
        }

      /* Release the reference */

      fs_putfilep(filep);
      return fd2;
    }

  return nx_dup3_from_tcb(tcb, fd1, fd2, 0);
}

/****************************************************************************
 * Name: nx_dup3_from_tcb
 *
 * Description:
 *   nx_dup3_from_tcb() is similar to the standard 'dup3' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   nx_dup3_from_tcb() is an internal NuttX interface and should not be
 *   called from applications.
 *
 *   Clone a file descriptor to a specific descriptor number and
 *   specific flags.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup3_from_tcb(FAR struct tcb_s *tcb, int fd1, int fd2, int flags)
{
  FAR struct fdlist *list;
  FAR struct file *filep1;
  FAR struct file *filep2;
  int count;
  int ret;

  if (fd1 == fd2)
    {
      return -EINVAL;
    }

#ifdef CONFIG_FDCHECK
  fd1 = fdcheck_restore(fd1);
  fd2 = fdcheck_restore(fd2);
#endif

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_fdlist_from_tcb(tcb);
  count = fdlist_count(list);

  if (fd1 < 0 || fd1 >= count || fd2 < 0)
    {
      return -EBADF;
    }

  if (fd2 >= count)
    {
      ret = fdlist_extend(list,
                          fd2 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Get file behind fd1, and keep the reference */

  filep1 = fget_by_fd(list, fd1, NULL);
  if (filep1 == NULL)
    {
      return -EBADF;
    }

  /* Atomically swap the file pointer behind fd2 */

  filep2 = fdlist_finstall(list, fd2, filep1, NULL);

  /* Set close on exec */

  fs_setcloseonexec(fd2, flags & FD_CLOEXEC);

  /* Close the old file, if any */

  if (filep2 != NULL)
    {
      fs_putfilep(filep2);
    }

  FS_ADD_BACKTRACE(fdlist_fdget(list, fd2));

#ifdef CONFIG_FDCHECK
  return fdcheck_protect(fd2);
#else
  return fd2;
#endif
}

/****************************************************************************
 * Name: nx_close_from_tcb
 *
 * Description:
 *   nx_close_from_tcb() is similar to the standard 'close' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   nx_close_from_tcb() is an internal NuttX interface and should not
 *   be called from applications.
 *
 *   Close an inode (if open)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   on any failure.
 *
 * Assumptions:
 *   Caller holds the list mutex because the file descriptor will be
 *   freed.
 *
 ****************************************************************************/

int nx_close_from_tcb(FAR struct tcb_s *tcb, int fd)
{
  FAR struct file   *filep;
  FAR struct fdlist *list;

#ifdef CONFIG_FDCHECK
  fd = fdcheck_restore(fd);
#endif

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_fdlist_from_tcb(tcb);

  /* Perform the protected close operation */

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  filep = fdlist_fget(list, fd);

  /* If the file was properly opened, there should be an inode assigned */

  if (filep == NULL)
    {
      return -EBADF;
    }

  /* Remove the file from the descriptor list */

  file_free_from_tcb(tcb, filep, fd);

#ifdef CONFIG_FS_REFCOUNT

  /* fdlist_fget will increase the reference count, there call fs_putfilep
   * reduce reference count.
   */

  fs_putfilep(filep);

  /* Undo the last reference count from file_allocate_from_tcb */

  return fs_putfilep(filep);
#else
  return fs_closefilep(filep);
#endif
}

/****************************************************************************
 * Name: sync
 *
 * Description:
 *   sync() causes all pending modifications to filesystem metadata and
 *   cached file data to be written to the underlying filesystems.
 *
 ****************************************************************************/

void sync(void)
{
  nxsched_foreach(task_fssync, NULL);
}
