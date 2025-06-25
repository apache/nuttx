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

#ifdef CONFIG_FDCHECK
#  include <nuttx/fdcheck.h>
#endif

#include "sched/sched.h"
#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdlist_get_by_index
 ****************************************************************************/

static void fdlist_get_by_index(FAR struct fdlist *list,
                                int l1, int l2,
                                FAR struct file **filep,
                                FAR struct fd **fdp)
{
  FAR struct fd *fdp1;
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(&list->fl_lock);
  fdp1 = &list->fl_fds[l1][l2];
  *filep = fdp1->f_file;
  if (*filep != NULL)
    {
      atomic_fetch_add(&(*filep)->f_refs, 1);
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  if (fdp != NULL)
    {
      *fdp = fdp1;
    }
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

/****************************************************************************
 * Name: fdlist_uninstall
 ****************************************************************************/

static void fdlist_uninstall(FAR struct fdlist *list, FAR struct fd *fdp)
{
  FAR struct file *filep = NULL;
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  if (fdp->f_file != NULL)
    {
#ifdef CONFIG_FDSAN
      fdp->f_tag_fdsan   = 0;
#endif
#ifdef CONFIG_FDCHECK
      fdp->f_tag_fdcheck = 0;
#endif
      filep              = fdp->f_file;
      fdp->f_file        = NULL;
    }

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  file_put(filep);
}

static void fdlist_install(FAR struct fdlist *list, int fd,
                           FAR struct file *filep, int oflags)
{
  FAR struct file *oldfilep;
  FAR struct fd *fdp;
  irqstate_t flags;
  int l1;
  int l2;

  l1 = fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  l2 = fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  fdp = &list->fl_fds[l1][l2];
  oldfilep = fdp->f_file;
  fdp->f_file = filep;
  file_ref(filep);
  fdp->f_cloexec = !!(oflags & O_CLOEXEC);
  FS_ADD_BACKTRACE(fdp);

  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);
  file_put(oldfilep);
}

/****************************************************************************
 * Name: task_fssync
 ****************************************************************************/

static void task_fssync(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct tcb_s *ctcb;
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
          FAR struct file *filep;

          ctcb = nxsched_get_tcb(pid);
          if (ctcb == NULL || ctcb->group == NULL || ctcb != tcb)
            {
              return;
            }

          fdlist_get_by_index(&ctcb->group->tg_fdlist, i, j, &filep, NULL);
          if (filep != NULL)
            {
              file_fsync(filep);
              file_put(filep);
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdlist_dup3
 *
 * Description:
 *   fdlist_dup3() is similar to the standard 'dup3' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   fdlist_dup3() is an internal NuttX interface and should not be
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

int fdlist_dup3(FAR struct fdlist *list, int fd1, int fd2, int flags)
{
  FAR struct file *filep1;
  int ret;

  if (fd1 == fd2)
    {
      return -EINVAL;
    }

#ifdef CONFIG_FDCHECK
  fd2 = fdcheck_restore(fd2);
#endif

  /* Get the file descriptor list.  It should not be NULL in this context. */

  if (fd2 < 0)
    {
      return -EBADF;
    }

  if (fd2 >= fdlist_count(list))
    {
      ret = fdlist_extend(list,
                          fd2 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = fdlist_get(list, fd1, &filep1);
  if (ret < 0)
    {
      return ret;
    }

  fdlist_install(list, fd2, filep1, flags);
  file_put(filep1);

#ifdef CONFIG_FDCHECK
  return fdcheck_protect(fd2);
#else
  return fd2;
#endif
}

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
      FAR struct fd *fdp;
      FAR struct file *filep;

#if CONFIG_FS_BACKTRACE > 0
      char buf[BACKTRACE_BUFFER_SIZE(CONFIG_FS_BACKTRACE)];
#endif

      /* Is there an inode associated with the file descriptor? */

      if (fdlist_get2(list, i, &filep, &fdp) < 0)
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

      file_put(filep);
    }

  lib_put_pathbuffer(path);
}
#endif

/****************************************************************************
 * Name: fdlist_free
 *
 * Description:
 *   Release the list of file descriptors.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.
 *
 ****************************************************************************/

void fdlist_free(FAR struct fdlist *list)
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
          fdlist_close(list, i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j);
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
 * Name: fdlist_get2
 *
 * Description:
 *   Given a file descriptor, return the corresponding instance of struct
 *   fd and filep.
 *
 * Input Parameters:
 *   list  - Pointer to the file descriptor list structure.
 *   fd    - The file descriptor
 *   filep - The location to return the struct file instance
 *   fdp   - The location to return the struct fd instance
 *
 * Returned Value:
 *   Return the pointer to file structure of list[fd] when list[fd].f_file
 *   is valid, otherwise, a null pointer is returned.
 *
 ****************************************************************************/

int fdlist_get2(FAR struct fdlist *list, int fd,
                FAR struct file **filep, FAR struct fd **fdp)
{
  if (list == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_FDCHECK
  fd = fdcheck_restore(fd);
#endif

  if (fd < 0 || fd >= fdlist_count(list))
    {
      return -EBADF;
    }

  fdlist_get_by_index(list,
                      fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK,
                      fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK,
                      filep, fdp);
  if (*filep == NULL)
    {
      return -EBADF;
    }

  return 0;
}

/****************************************************************************
 * Name: fdlist_dupfile
 *
 * Description:
 *   Allocate a struct fd instance and bind it to the corresponding file
 *   handle.
 *
 * Returned Value:
 *   Returns the file descriptor == index into the files array on success;
 *   a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int fdlist_dupfile(FAR struct fdlist *list, int oflags, int minfd,
                   FAR struct file *filep)
{
  FAR struct fd *fdp;
  irqstate_t flags;
  int ret;
  int i;
  int j;

  DEBUGASSERT(filep);

#ifdef CONFIG_FDCHECK
  minfd = fdcheck_restore(minfd);
#endif

  i = minfd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  j = minfd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;

  /* Find free file descriptor */

  flags = spin_lock_irqsave_notrace(&list->fl_lock);

  for (; ; i++, j = 0)
    {
      if (i >= list->fl_rows)
        {
          spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

          ret = fdlist_extend(list, i + 1);
          if (ret < 0)
            {
              return ret;
            }

          flags = spin_lock_irqsave_notrace(&list->fl_lock);
        }

      do
        {
          fdp = &list->fl_fds[i][j];
          if (fdp->f_file == NULL)
            {
              atomic_fetch_add(&filep->f_refs, 1);
              fdp->f_file        = filep;
              fdp->f_cloexec     = !!(oflags & O_CLOEXEC);
 #ifdef CONFIG_FDSAN
              fdp->f_tag_fdsan   = 0;
 #endif
 #ifdef CONFIG_FDCHECK
              fdp->f_tag_fdcheck = 0;
 #endif
              goto found;
            }
        }
      while (++j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
    }

found:
  spin_unlock_irqrestore_notrace(&list->fl_lock, flags);

  FS_ADD_BACKTRACE(fdp);

#ifdef CONFIG_FDCHECK
  return fdcheck_protect(i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j);
#else
  return i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;
#endif
}

/****************************************************************************
 * Name: fdlist_allocate
 *
 * Description:
 *   Allocate a struct fd instance and associate it with an empty file
 *   instance. The difference between this function and
 *   file_allocate_from_inode is that this function is only used for
 *   placeholder purposes. Later, the caller will initialize the file entity
 *   through the returned filep.
 *
 *   The fd allocated by this function can be released using fdlist_close.
 *
 *   After the function call is completed, it will hold a reference count
 *   for the filep. Therefore, when the filep is no longer in use, it is
 *   necessary to call file_put to release the reference count, in order
 *   to avoid a race condition where the file might be closed during
 *   this process.
 *
 * Returned Value:
 *   Returns the file descriptor == index into the files array on success;
 *   a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int fdlist_allocate(FAR struct fdlist *list, int oflags,
                    int minfd, FAR struct file **filep)
{
  int fd;

  *filep = fs_heap_zalloc(sizeof(struct file));
  if (*filep == NULL)
    {
      return -ENOMEM;
    }

  file_ref(*filep);
  fd = fdlist_dupfile(list, oflags, minfd, *filep);
  if (fd < 0)
    {
      file_put(*filep);
    }

  return fd;
}

/****************************************************************************
 * Name: file_allocate
 *
 * Description:
 *   Allocate a struct fd instance and associate it with an empty file
 *   instance. The difference between this function and
 *   file_allocate_from_inode is that this function is only used for
 *   placeholder purposes. Later, the caller will initialize the file entity
 *   through the returned filep.
 *
 *   The fd allocated by this function can be released using nx_close.
 *
 *   After the function call is completed, it will hold a reference count
 *   for the filep. Therefore, when the filep is no longer in use, it is
 *   necessary to call file_put to release the reference count, in order
 *   to avoid a race condition where the file might be closed during
 *   this process.
 *
 * Returned Value:
 *   Returns the file descriptor == index into the files array on success;
 *   a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int file_allocate(int oflags, int minfd, FAR struct file **filep)
{
  return fdlist_allocate(nxsched_get_fdlist_from_tcb(this_task()),
                         oflags, minfd, filep);
}

/****************************************************************************
 * Name: file_allocate_from_inode
 *
 * Description:
 *   Allocate a struct fd instance and associate it with an file instance.
 *   And initialize them with inode, oflags, pos and priv.
 *
 * Returned Value:
 *   Returns the file descriptor == index into the files array on success;
 *   a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int file_allocate_from_inode(FAR struct inode *inode, int oflags, off_t pos,
                             FAR void *priv, int minfd)
{
  FAR struct file *filep;
  int fd;

  fd = file_allocate(oflags, minfd, &filep);
  if (fd < 0)
    {
      return fd;
    }

  inode_addref(inode);
  filep->f_inode  = inode;
  filep->f_pos    = pos;
  filep->f_oflags = oflags & ~O_CLOEXEC;
  filep->f_priv   = priv;
#if CONFIG_FS_LOCK_BUCKET_SIZE > 0
  filep->f_locked = false;
#endif

  file_put(filep);

  return fd;
}

/****************************************************************************
 * Name: fdlist_copy
 *
 * Description:
 *   Copy parent task's file descriptors to child task.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int fdlist_copy(FAR struct fdlist *plist, FAR struct fdlist *clist,
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
          FAR struct fd *fdp;

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

          fdlist_get_by_index(plist, i, j, &filep, &fdp);
          if (filep == NULL)
            {
              continue;
            }

          fcloexec = cloexec && fdp->f_cloexec;

          /* Skip file dup if file action is unnecessary to duplicate */

          if (actions != NULL)
            {
#ifdef CONFIG_FDCHECK
              fd = fdcheck_protect(fd);
#endif
              if (!spawn_file_is_duplicateable(actions, fd, fcloexec))
                {
                  file_put(filep);
                  continue;
                }

#ifdef CONFIG_FDCHECK
              fd = fdcheck_restore(fd);
#endif
            }
          else if (fcloexec)
            {
              file_put(filep);
              continue;
            }

          ret = fdlist_extend(clist, i + 1);
          if (ret < 0)
            {
              file_put(filep);
              return ret;
            }

          /* Assign filep to the child's descriptor list. Omit the flags */

          fdlist_install(clist, fd, filep, 0);
          file_put(filep);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: file_get2
 *
 * Description:
 *   Given a file descriptor, return the corresponding instance of struct
 *   fd and filep
 *
 * Input Parameters:
 *   fd    - The file descriptor
 *   filep - The location to return the struct file instance
 *   fdp   - The location to return the struct fd instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int file_get2(int fd, FAR struct file **filep, FAR struct fd **fdp)
{
  return fdlist_get2(nxsched_get_fdlist(), fd, filep, fdp);
}

/****************************************************************************
 * Name: file_ref
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

void file_ref(FAR struct file *filep)
{
  /* This interface is used to increase the reference count of filep */

  DEBUGASSERT(filep);
  atomic_fetch_add(&filep->f_refs, 1);
}

/****************************************************************************
 * Name: file_put
 *
 * Description:
 *   Handles reference counts for files, less than or equal to 0 and close
 *   the file
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 ****************************************************************************/

int file_put(FAR struct file *filep)
{
  int ret = 0;

  if (filep == NULL)
    {
      return ret;
    }

  /* If refs is zero, the close() had called, closing it now. */

  if (atomic_fetch_sub(&filep->f_refs, 1) == 1)
    {
      ret = file_close(filep);
      if (ret < 0)
        {
          ferr("ERROR: fs putfilep file_close() failed: %d\n", ret);
        }

      fs_heap_free(filep);
    }

  return ret;
}

/****************************************************************************
 * Name: fdlist_dup2
 *
 * Description:
 *   fdlist_dup2() is similar to the standard 'dup2' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   fdlist_dup2() is an internal NuttX interface and should not be
 *   called from applications.
 *
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int fdlist_dup2(FAR struct fdlist *list, int fd1, int fd2)
{
  /* If fd1 is a valid file descriptor, and fd1 == fd2, then dup2() does
   * nothing, and returns fd2.
   */

  if (fd1 == fd2)
    {
      FAR struct file *filep;
      int ret;

      ret = file_get(fd1, &filep);
      if (ret < 0)
        {
          return ret;
        }

      /* Release the reference */

      file_put(filep);
      return fd2;
    }

  return fdlist_dup3(list, fd1, fd2, 0);
}

/****************************************************************************
 * Name: fdlist_close
 *
 * Description:
 *   fdlist_close() is similar to the standard 'close' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   fdlist_close() is an internal NuttX interface and should not
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

int fdlist_close(FAR struct fdlist *list, int fd)
{
  FAR struct file *filep;
  FAR struct fd   *fdp;
  int ret;

  ret = fdlist_get2(list, fd, &filep, &fdp);
  if (ret < 0)
    {
      return ret;
    }

  /* Perform the protected close operation */

  fdlist_uninstall(list, fdp);

  /* fdlist_get2 will increase the reference count, there call
   * file_put reduce reference count.
   */

  return file_put(filep);
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
