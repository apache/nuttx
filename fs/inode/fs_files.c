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

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: files_extend
 ****************************************************************************/

static int files_extend(FAR struct filelist *list, size_t row)
{
  FAR struct file **tmp;
  int i;

  if (row <= list->fl_rows)
    {
      return 0;
    }

  if (row * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK > OPEN_MAX)
    {
      return -EMFILE;
    }

  tmp = kmm_realloc(list->fl_files, sizeof(FAR struct file *) * row);
  DEBUGASSERT(tmp);
  if (tmp == NULL)
    {
      return -ENFILE;
    }

  i = list->fl_rows;
  do
    {
      tmp[i] = kmm_zalloc(sizeof(struct file) *
                          CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
      if (tmp[i] == NULL)
        {
          while (--i >= list->fl_rows)
            {
              kmm_free(tmp[i]);
            }

          kmm_free(tmp);
          return -ENFILE;
        }
    }
  while (++i < row);

  list->fl_files = tmp;
  list->fl_rows = row;

  /* Note: If assertion occurs, the fl_rows has a overflow.
   * And there may be file descriptors leak in system.
   */

  DEBUGASSERT(list->fl_rows == row);
  return 0;
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
  DEBUGASSERT(list);

  /* Initialize the list access mutex */

  nxmutex_init(&list->fl_lock);
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

      kmm_free(list->fl_files[i]);
    }

  kmm_free(list->fl_files);

  /* Destroy the mutex */

  nxmutex_destroy(&list->fl_lock);
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
  FAR struct filelist *list;
  int ret;
  int i;
  int j;

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  ret = nxmutex_lock(&list->fl_lock);
  if (ret < 0)
    {
      /* Probably canceled */

      return ret;
    }

  /* Calculate minfd whether is in list->fl_files.
   * if not, allocate a new filechunk.
   */

  i = minfd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  if (i >= list->fl_rows)
    {
      ret = files_extend(list, i + 1);
      if (ret < 0)
        {
          nxmutex_unlock(&list->fl_lock);
          return ret;
        }
    }

  /* Find free file */

  j = minfd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
  do
    {
      do
        {
          if (!list->fl_files[i][j].f_inode)
            {
              list->fl_files[i][j].f_oflags = oflags;
              list->fl_files[i][j].f_pos    = pos;
              list->fl_files[i][j].f_inode  = inode;
              list->fl_files[i][j].f_priv   = priv;
              nxmutex_unlock(&list->fl_lock);

              if (addref)
                {
                  inode_addref(inode);
                }

              return i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;
            }
        }
      while (++j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);

      j = 0;
    }
  while (++i < list->fl_rows);

  /* The space of file array isn't enough, allocate a new filechunk */

  ret = files_extend(list, i + 1);
  if (ret < 0)
    {
      nxmutex_unlock(&list->fl_lock);
      return ret;
    }

  list->fl_files[i][0].f_oflags = oflags;
  list->fl_files[i][0].f_pos    = pos;
  list->fl_files[i][0].f_inode  = inode;
  list->fl_files[i][0].f_priv   = priv;
  nxmutex_unlock(&list->fl_lock);

  if (addref)
    {
      inode_addref(inode);
    }

  return i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;
}

/****************************************************************************
 * Name: files_duplist
 *
 * Description:
 *   Duplicate parent task's file descriptors.
 *
 ****************************************************************************/

int files_duplist(FAR struct filelist *plist, FAR struct filelist *clist)
{
  int ret;
  int i;
  int j;

  DEBUGASSERT(clist);
  DEBUGASSERT(plist);

  ret = nxmutex_lock(&plist->fl_lock);
  if (ret < 0)
    {
      /* Probably canceled */

      return ret;
    }

  for (i = 0; i < plist->fl_rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          FAR struct file *filep;
#ifdef CONFIG_FDCLONE_STDIO

          /* Determine how many file descriptors to clone.  If
           * CONFIG_FDCLONE_DISABLE is set, no file descriptors will be
           * cloned.  If CONFIG_FDCLONE_STDIO is set, only the first
           * three descriptors (stdin, stdout, and stderr) will be
           * cloned.  Otherwise all file descriptors will be cloned.
           */

          if (i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j >= 3)
            {
              goto out;
            }
#endif

          filep = &plist->fl_files[i][j];
          DEBUGASSERT(filep);

          if (filep && (filep->f_inode == NULL ||
                       (filep->f_oflags & O_CLOEXEC) != 0))
            {
              continue;
            }

          ret = files_extend(clist, i + 1);
          if (ret < 0)
            {
              goto out;
            }

          /* Yes... duplicate it for the child */

          ret = file_dup2(filep, &clist->fl_files[i][j]);
          if (ret < 0)
            {
              goto out;
            }
        }
    }

out:
  nxmutex_unlock(&plist->fl_lock);
  return ret;
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
  int ret;

  DEBUGASSERT(filep != NULL);
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

  if (fd < 0 || fd >= list->fl_rows * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK)
    {
      return -EBADF;
    }

  /* The descriptor is in a valid range to file descriptor... Get the
   * thread-specific file list.
   */

  /* And return the file pointer from the list */

  ret = nxmutex_lock(&list->fl_lock);
  if (ret < 0)
    {
      return ret;
    }

  *filep = &list->fl_files[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                          [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK];

  /* if f_inode is NULL, fd was closed */

  if (!(*filep)->f_inode)
    {
      *filep = NULL;
      ret = -EBADF;
    }

  nxmutex_unlock(&list->fl_lock);
  return ret;
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
  FAR struct filelist *list;
  FAR struct file     *filep;
  FAR struct file      file;
  int ret;

  if (fd1 == fd2)
    {
      return fd1;
    }

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  if (fd1 < 0 || fd1 >= CONFIG_NFILE_DESCRIPTORS_PER_BLOCK * list->fl_rows ||
      fd2 < 0)
    {
      return -EBADF;
    }

  ret = nxmutex_lock(&list->fl_lock);
  if (ret < 0)
    {
      /* Probably canceled */

      return ret;
    }

  if (fd2 >= CONFIG_NFILE_DESCRIPTORS_PER_BLOCK * list->fl_rows)
    {
      ret = files_extend(list, fd2 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + 1);
      if (ret < 0)
        {
          nxmutex_unlock(&list->fl_lock);
          return ret;
        }
    }

  filep = &list->fl_files[fd2 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                         [fd2 % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK];
  memcpy(&file, filep, sizeof(struct file));
  memset(filep, 0,     sizeof(struct file));

  /* Perform the dup2 operation */

  ret = file_dup2(&list->fl_files[fd1 / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                                 [fd1 % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK],
                  filep);
  nxmutex_unlock(&list->fl_lock);

  file_close(&file);
  return ret < 0 ? ret : fd2;
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
  FAR struct filelist *list;
  FAR struct file     *filep;
  FAR struct file      file;
  int                  ret;

  /* Get the thread-specific file list.  It should never be NULL in this
   * context.
   */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  /* Perform the protected close operation */

  ret = nxmutex_lock(&list->fl_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* If the file was properly opened, there should be an inode assigned */

  if (fd < 0 || fd >= list->fl_rows * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK ||
      !list->fl_files[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                     [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK].f_inode)
    {
      nxmutex_unlock(&list->fl_lock);
      return -EBADF;
    }

  filep = &list->fl_files[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                         [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK];
  memcpy(&file, filep, sizeof(struct file));
  memset(filep, 0,     sizeof(struct file));

  nxmutex_unlock(&list->fl_lock);
  return file_close(&file);
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
