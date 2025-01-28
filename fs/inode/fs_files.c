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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: files_fget_by_index
 ****************************************************************************/

static FAR struct file *files_fget_by_index(FAR struct filelist *list,
                                            int l1, int l2, FAR bool *new)
{
  FAR struct file *filep;
  irqstate_t flags;

  flags = spin_lock_irqsave(&list->fl_lock);
  filep = &list->fl_files[l1][l2];
  spin_unlock_irqrestore(&list->fl_lock, flags);

#ifdef CONFIG_FS_REFCOUNT
  if (filep->f_inode != NULL)
    {
      /* When the reference count is zero but the inode has not yet been
       * released, At this point we should return a null pointer
       */

      int32_t refs = atomic_read(&filep->f_refs);
      do
        {
          if (refs == 0)
            {
              filep = NULL;
              break;
            }
        }
      while (!atomic_try_cmpxchg(&filep->f_refs, &refs, refs + 1));
    }
  else if (new == NULL)
    {
      filep = NULL;
    }
  else if (atomic_fetch_add(&filep->f_refs, 1) == 0)
    {
      atomic_fetch_add(&filep->f_refs, 1);
      *new = true;
    }

#else
  if (filep->f_inode == NULL && new == NULL)
    {
      filep = NULL;
    }
#endif

  return filep;
}

/****************************************************************************
 * Name: files_extend
 ****************************************************************************/

static int files_extend(FAR struct filelist *list, size_t row)
{
  FAR struct file **files;
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
      files_dumplist(list);
      return -EMFILE;
    }

  files = fs_heap_malloc(sizeof(FAR struct file *) * row);
  DEBUGASSERT(files);
  if (files == NULL)
    {
      return -ENFILE;
    }

  i = orig_rows;
  do
    {
      files[i] = fs_heap_zalloc(sizeof(struct file) *
                            CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
      if (files[i] == NULL)
        {
          while (--i >= orig_rows)
            {
              fs_heap_free(files[i]);
            }

          fs_heap_free(files);
          return -ENFILE;
        }
    }
  while (++i < row);

  flags = spin_lock_irqsave(&list->fl_lock);

  /* To avoid race condition, if the file list is updated by other threads
   * and list rows is greater or equal than temp list,
   * release the obsolete buffers
   */

  if (orig_rows != list->fl_rows && list->fl_rows >= row)
    {
      spin_unlock_irqrestore(&list->fl_lock, flags);

      for (j = orig_rows; j < i; j++)
        {
          fs_heap_free(files[j]);
        }

      fs_heap_free(files);

      return OK;
    }

  if (list->fl_files != NULL)
    {
      memcpy(files, list->fl_files,
             list->fl_rows * sizeof(FAR struct file *));
    }

  tmp = list->fl_files;
  list->fl_files = files;
  list->fl_rows = row;

  spin_unlock_irqrestore(&list->fl_lock, flags);

  if (tmp != NULL && tmp != &list->fl_prefile)
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

  rows = tcb->group->tg_filelist.fl_rows;

  for (i = 0; i < rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          ctcb = nxsched_get_tcb(pid);
          if (ctcb == NULL || ctcb->group == NULL || ctcb != tcb)
            {
              return;
            }

          filep = files_fget_by_index(&ctcb->group->tg_filelist,
                                      i, j, NULL);
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

static int nx_dup3_from_tcb(FAR struct tcb_s *tcb, int fd1, int fd2,
                            int flags)
{
  FAR struct filelist *list;
  FAR struct file *filep1;
  FAR struct file *filep;
#ifdef CONFIG_FDCHECK
  uint8_t f_tag_fdcheck;
#endif
#ifdef CONFIG_FDSAN
  uint64_t f_tag_fdsan;
#endif
  bool new = false;
  int count;
  int ret;

  if (fd1 == fd2)
    {
      return fd1;
    }

#ifdef CONFIG_FDCHECK
  fd1 = fdcheck_restore(fd1);
  fd2 = fdcheck_restore(fd2);
#endif

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_files_from_tcb(tcb);
  count = files_countlist(list);

  if (fd1 < 0 || fd1 >= count || fd2 < 0)
    {
      return -EBADF;
    }

  if (fd2 >= count)
    {
      ret = files_extend(list, fd2 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  filep1 = files_fget(list, fd1);
  if (filep1 == NULL)
    {
      return -EBADF;
    }

  filep = files_fget_by_index(list,
                              fd2 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK,
                              fd2 % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK,
                              &new);

#ifdef CONFIG_FDSAN
  f_tag_fdsan = filep->f_tag_fdsan;
#endif

#ifdef CONFIG_FDCHECK
  f_tag_fdcheck = filep->f_tag_fdcheck;
#endif

  /* Perform the dup3 operation */

  ret = file_dup3(filep1, filep, flags);
  fs_putfilep(filep1);
  fs_putfilep(filep);
  if (ret < 0)
    {
      if (new)
        {
          fs_putfilep(filep);
        }

      return ret;
    }

#ifdef CONFIG_FDSAN
  filep->f_tag_fdsan = f_tag_fdsan;
#endif

#ifdef CONFIG_FDCHECK
  filep->f_tag_fdcheck = f_tag_fdcheck;
#endif

#ifdef CONFIG_FDCHECK
  return fdcheck_protect(fd2);
#else
  return fd2;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: files_initlist
 *
 * Description: Initializes the list of files for a new task
 *
 ****************************************************************************/

void files_initlist(FAR struct filelist *list)
{
  /* The first row will reuse pre-allocated files, which will avoid
   * unnecessary allocator accesses during file initialization.
   */

  list->fl_rows = 1;
  list->fl_files = &list->fl_prefile;
  list->fl_prefile = list->fl_prefiles;
  spin_lock_init(&list->fl_lock);
}

/****************************************************************************
 * Name: files_dumplist
 *
 * Description:
 *   Dump the list of files.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_DUMP_ON_EXIT
void files_dumplist(FAR struct filelist *list)
{
  FAR char *path;
  int count = files_countlist(list);
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
      FAR struct file *filep = files_fget(list, i);

#if CONFIG_FS_BACKTRACE > 0
      char buf[BACKTRACE_BUFFER_SIZE(CONFIG_FS_BACKTRACE)];
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
      backtrace_format(buf, sizeof(buf), filep->f_backtrace,
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
 * Name: files_putlist
 *
 * Description:
 *   Release the list of files.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.
 *
 ****************************************************************************/

void files_putlist(FAR struct filelist *list)
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
          file_close(&list->fl_files[i][j]);
        }

      if (i != 0)
        {
          fs_heap_free(list->fl_files[i]);
        }
    }

  if (list->fl_files != &list->fl_prefile)
    {
      fs_heap_free(list->fl_files);
    }
}

/****************************************************************************
 * Name: files_countlist
 *
 * Description:
 *   Get file count from file list.
 *
 * Input Parameters:
 *   list - Pointer to the file list structure.
 *
 * Returned Value:
 *   file count of file list.
 *
 ****************************************************************************/

int files_countlist(FAR struct filelist *list)
{
  return list->fl_rows * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
}

/****************************************************************************
 * Name: files_fget
 *
 * Description:
 *   Get the instance of struct file from file list by file descriptor.
 *
 * Input Parameters:
 *   list - The list of files for a task.
 *   fd   - A valid descriptor between 0 and files_countlist(list).
 *
 * Returned Value:
 *   Pointer to file structure of list[fd].
 *
 ****************************************************************************/

FAR struct file *files_fget(FAR struct filelist *list, int fd)
{
  return files_fget_by_index(list, fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK,
                             fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK, NULL);
}

/****************************************************************************
 * Name: file_allocate_from_tcb
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

int file_allocate_from_tcb(FAR struct tcb_s *tcb, FAR struct inode *inode,
                           int oflags, off_t pos, FAR void *priv, int minfd,
                           bool addref)
{
  int i = minfd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  int j = minfd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  FAR struct filelist *list;
  FAR struct file *filep;
  irqstate_t flags;
  int ret;

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_files_from_tcb(tcb);

  /* Find free file */

  flags = spin_lock_irqsave(&list->fl_lock);

  for (; ; i++, j = 0)
    {
      if (i >= list->fl_rows)
        {
          spin_unlock_irqrestore(&list->fl_lock, flags);

          ret = files_extend(list, i + 1);
          if (ret < 0)
            {
              return ret;
            }

          flags = spin_lock_irqsave(&list->fl_lock);
        }

      do
        {
          filep = &list->fl_files[i][j];
          if (filep->f_inode == NULL)
            {
              filep->f_oflags      = oflags;
              filep->f_pos         = pos;
              filep->f_inode       = inode;
              filep->f_priv        = priv;
#ifdef CONFIG_FS_REFCOUNT
              atomic_set(&filep->f_refs, 1);
#endif
#ifdef CONFIG_FDSAN
              filep->f_tag_fdsan   = 0;
#endif
#ifdef CONFIG_FDCHECK
              filep->f_tag_fdcheck = 0;
#endif

              goto found;
            }
        }
      while (++j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
    }

found:
  spin_unlock_irqrestore(&list->fl_lock, flags);

  if (addref)
    {
      inode_addref(inode);
    }

  FS_ADD_BACKTRACE(filep);

#ifdef CONFIG_FDCHECK
  return fdcheck_protect(i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j);
#else
  return i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;
#endif
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
 * Name: files_duplist
 *
 * Description:
 *   Duplicate parent task's file descriptors.
 *
 ****************************************************************************/

int files_duplist(FAR struct filelist *plist, FAR struct filelist *clist,
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
          FAR struct file *filep2;
          FAR struct file *filep;
          bool new = false;

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

          filep = files_fget_by_index(plist, i, j, NULL);
          if (filep == NULL)
            {
              continue;
            }

          fcloexec = (cloexec && (filep->f_oflags & O_CLOEXEC));

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

          ret = files_extend(clist, i + 1);
          if (ret < 0)
            {
              fs_putfilep(filep);
              return ret;
            }

          /* Yes... duplicate it for the child, include O_CLOEXEC flag. */

          filep2 = files_fget_by_index(clist, i, j, &new);
          ret = file_dup2(filep, filep2);
          fs_putfilep(filep2);
          fs_putfilep(filep);
          if (ret < 0)
            {
              if (new)
                {
                  fs_putfilep(filep2);
                }

              return ret;
            }
        }
    }

  return OK;
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
  FAR struct filelist *list;

#ifdef CONFIG_FDCHECK
  fd = fdcheck_restore(fd);
#endif

  *filep = NULL;

  list = nxsched_get_files();

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

  if (fd < 0 || fd >= files_countlist(list))
    {
      return -EBADF;
    }

  /* The descriptor is in a valid range to file descriptor... Get the
   * thread-specific file list.
   */

  *filep = files_fget(list, fd);

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
      ret = file_close(filep);
      if (ret < 0)
        {
          ferr("ERROR: fs putfilep file_close() failed: %d\n", ret);
        }
    }

  return ret;
}
#endif

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
  return nx_dup3_from_tcb(tcb, fd1, fd2, 0);
}

/****************************************************************************
 * Name: nx_dup2
 *
 * Description:
 *   nx_dup2() is similar to the standard 'dup2' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_dup2() is an internal NuttX interface and should not be called from
 *   applications.
 *
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2(int fd1, int fd2)
{
  return nx_dup2_from_tcb(this_task(), fd1, fd2);
}

/****************************************************************************
 * Name: dup2
 *
 * Description:
 *   Clone a file descriptor or socket descriptor to a specific descriptor
 *   number
 *
 ****************************************************************************/

int dup2(int fd1, int fd2)
{
  int ret;

  ret = nx_dup2(fd1, fd2);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: dup3
 *
 * Description:
 *   Clone a file descriptor or socket descriptor to a specific descriptor
 *   number and specific flags.
 *
 ****************************************************************************/

int dup3(int fd1, int fd2, int flags)
{
  int ret;

  ret = nx_dup3_from_tcb(this_task(), fd1, fd2, flags);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
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
  FAR struct file     *filep;
  FAR struct filelist *list;

#ifdef CONFIG_FDCHECK
  fd = fdcheck_restore(fd);
#endif

  list = nxsched_get_files_from_tcb(tcb);

  /* Perform the protected close operation */

  if (fd < 0 || fd >= files_countlist(list))
    {
      return -EBADF;
    }

  filep = files_fget(list, fd);

  /* If the file was properly opened, there should be an inode assigned */

  if (filep == NULL)
    {
      return -EBADF;
    }

#ifdef CONFIG_FS_REFCOUNT

  /* files_fget will increase the reference count, there call fs_putfilep
   * reduce reference count.
   */

  fs_putfilep(filep);

  /* Undo the last reference count from file_allocate_from_tcb */

  return fs_putfilep(filep);
#else
  return file_close(filep);
#endif
}

/****************************************************************************
 * Name: nx_close
 *
 * Description:
 *   nx_close() is similar to the standard 'close' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_close() is an internal NuttX interface and should not be called from
 *   applications.
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

int nx_close(int fd)
{
  return nx_close_from_tcb(this_task(), fd);
}

/****************************************************************************
 * Name: close
 *
 * Description:
 *   close() closes a file descriptor, so that it no longer refers to any
 *   file and may be reused. Any record locks (see fcntl(2)) held on the file
 *   it was associated with, and owned by the process, are removed
 *   (regardless of the file descriptor that was used to obtain the lock).
 *
 *   If fd is the last copy of a particular file descriptor the resources
 *   associated with it are freed; if the descriptor was the last reference
 *   to a file which has been removed using unlink(2) the file is deleted.
 *
 * Input Parameters:
 *   fd   file descriptor to close
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int close(int fd)
{
  int ret;

#ifdef CONFIG_FDSAN
  android_fdsan_exchange_owner_tag(fd, 0, 0);
#endif

  /* close() is a cancellation point */

  enter_cancellation_point();

  ret = nx_close(fd);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
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
