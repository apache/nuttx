/****************************************************************************
 * fs/inode/fs_files.c
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
#include <sched.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/cancelpt.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/spawn.h>

#ifdef CONFIG_FDSAN
#  include <android/fdsan.h>
#endif

#ifdef CONFIG_FDCHECK
#  include <nuttx/fdcheck.h>
#endif

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: files_fget_by_index
 ****************************************************************************/

static FAR struct file *files_fget_by_index(FAR struct filelist *list,
                                            int l1, int l2)
{
  FAR struct file *filep;
  irqstate_t flags;

  flags = spin_lock_irqsave(&list->fl_lock);

  filep = &list->fl_files[l1][l2];

  spin_unlock_irqrestore(&list->fl_lock, flags);

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

  if (row <= list->fl_rows)
    {
      return 0;
    }

  if (files_countlist(list) > OPEN_MAX)
    {
      return -EMFILE;
    }

  orig_rows = list->fl_rows;

  files = kmm_malloc(sizeof(FAR struct file *) * row);
  DEBUGASSERT(files);
  if (files == NULL)
    {
      return -ENFILE;
    }

  i = list->fl_rows;
  do
    {
      files[i] = kmm_zalloc(sizeof(struct file) *
                            CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
      if (files[i] == NULL)
        {
          while (--i >= list->fl_rows)
            {
              kmm_free(files[i]);
            }

          kmm_free(files);
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
          kmm_free(files[i]);
        }

      kmm_free(files);

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
      kmm_free(tmp);
    }

  return OK;
}

static void task_fssync(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct filelist *list;
  int i;
  int j;

  list = &tcb->group->tg_filelist;

  for (i = 0; i < list->fl_rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          FAR struct file *filep;

          filep = files_fget_by_index(list, i, j);
          if (filep->f_inode != NULL)
            {
              file_fsync(filep);
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
  FAR struct file *filep;
#ifdef CONFIG_FDCHECK
  uint8_t f_tag_fdcheck;
#endif
#ifdef CONFIG_FDSAN
  uint64_t f_tag_fdsan;
#endif
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

  filep = files_fget(list, fd2);
  if (filep == NULL)
    {
      return -EBADF;
    }

#ifdef CONFIG_FDSAN
  f_tag_fdsan = filep->f_tag_fdsan;
#endif

#ifdef CONFIG_FDCHECK
  f_tag_fdcheck = filep->f_tag_fdcheck;
#endif

  /* Perform the dup3 operation */

  ret = file_dup3(files_fget(list, fd1), filep, flags);
  if (ret < 0)
    {
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
}

/****************************************************************************
 * Name: files_releaselist
 *
 * Description:
 *   Release a reference to the file list
 *
 ****************************************************************************/

void files_releaselist(FAR struct filelist *list)
{
  int i;
  int j;

  DEBUGASSERT(list);

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
          kmm_free(list->fl_files[i]);
        }
    }

  if (list->fl_files != &list->fl_prefile)
    {
      kmm_free(list->fl_files);
    }
}

/****************************************************************************
 * Name: files_countlist
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
                             fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
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
              filep->f_oflags = oflags;
              filep->f_pos    = pos;
              filep->f_inode  = inode;
              filep->f_priv   = priv;

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
  return file_allocate_from_tcb(nxsched_self(), inode, oflags,
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
          FAR struct file *filep;

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

          filep = files_fget_by_index(plist, i, j);
          if (filep->f_inode == NULL)
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
                  continue;
                }
            }
          else if (fcloexec)
            {
              continue;
            }

          ret = files_extend(clist, i + 1);
          if (ret < 0)
            {
              return ret;
            }

          /* Yes... duplicate it for the child, include O_CLOEXEC flag. */

          ret = file_dup2(filep, files_fget_by_index(clist, i, j));
          if (ret < 0)
            {
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

  /* if f_inode is NULL, fd was closed */

  if ((*filep)->f_inode == NULL)
    {
      *filep = NULL;
      return -EBADF;
    }

  return OK;
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
  return nx_dup2_from_tcb(nxsched_self(), fd1, fd2);
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

  ret = nx_dup3_from_tcb(nxsched_self(), fd1, fd2, flags);
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

  if (filep->f_inode == NULL)
    {
      return -EBADF;
    }

  return file_close(filep);
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
  return nx_close_from_tcb(nxsched_self(), fd);
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
