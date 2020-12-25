/****************************************************************************
 * fs/tmpfs/fs_tmpfs.c
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

#include <sys/stat.h>
#include <sys/statfs.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>

#include "fs_tmpfs.h"

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_FS_TMPFS_DIRECTORY_FREEGUARD <= CONFIG_FS_TMPFS_DIRECTORY_ALLOCGUARD
#  warning CONFIG_FS_TMPFS_DIRECTORY_FREEGUARD needs to be > ALLOCGUARD
#endif

#if CONFIG_FS_TMPFS_FILE_FREEGUARD <= CONFIG_FS_TMPFS_FILE_ALLOCGUARD
#  warning CONFIG_FS_TMPFS_FILE_FREEGUARD needs to be > ALLOCGUARD
#endif

#define tmpfs_lock_file(tfo) \
           (tmpfs_lock_object((FAR struct tmpfs_object_s *)tfo))
#define tmpfs_lock_directory(tdo) \
           (tmpfs_lock_object((FAR struct tmpfs_object_s *)tdo))
#define tmpfs_unlock_file(tfo) \
           (tmpfs_unlock_object((FAR struct tmpfs_object_s *)tfo))
#define tmpfs_unlock_directory(tdo) \
           (tmpfs_unlock_object((FAR struct tmpfs_object_s *)tdo))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TMPFS helpers */

static int  tmpfs_lock_reentrant(FAR struct tmpfs_sem_s *sem);
static int  tmpfs_lock(FAR struct tmpfs_s *fs);
static void tmpfs_unlock_reentrant(FAR struct tmpfs_sem_s *sem);
static void tmpfs_unlock(FAR struct tmpfs_s *fs);
static int  tmpfs_lock_object(FAR struct tmpfs_object_s *to);
static void tmpfs_unlock_object(FAR struct tmpfs_object_s *to);
static int  tmpfs_realloc_directory(FAR struct tmpfs_directory_s **tdo,
              unsigned int nentries);
static int  tmpfs_realloc_file(FAR struct tmpfs_file_s **tfo,
              size_t newsize);
static void tmpfs_release_lockedobject(FAR struct tmpfs_object_s *to);
static void tmpfs_release_lockedfile(FAR struct tmpfs_file_s *tfo);
static int  tmpfs_find_dirent(FAR struct tmpfs_directory_s *tdo,
              FAR const char *name);
static int  tmpfs_remove_dirent(FAR struct tmpfs_directory_s *tdo,
              FAR const char *name);
static int  tmpfs_add_dirent(FAR struct tmpfs_directory_s **tdo,
              FAR struct tmpfs_object_s *to, FAR const char *name);
static FAR struct tmpfs_file_s *tmpfs_alloc_file(void);
static int  tmpfs_create_file(FAR struct tmpfs_s *fs,
              FAR const char *relpath, FAR struct tmpfs_file_s **tfo);
static FAR struct tmpfs_directory_s *tmpfs_alloc_directory(void);
static int  tmpfs_create_directory(FAR struct tmpfs_s *fs,
              FAR const char *relpath, FAR struct tmpfs_directory_s **tdo);
static int  tmpfs_find_object(FAR struct tmpfs_s *fs,
              FAR const char *relpath, FAR struct tmpfs_object_s **object,
              FAR struct tmpfs_directory_s **parent);
static int  tmpfs_find_file(FAR struct tmpfs_s *fs,
              FAR const char *relpath,
              FAR struct tmpfs_file_s **tfo,
              FAR struct tmpfs_directory_s **parent);
static int  tmpfs_find_directory(FAR struct tmpfs_s *fs,
              FAR const char *relpath,
              FAR struct tmpfs_directory_s **tdo,
              FAR struct tmpfs_directory_s **parent);
static int  tmpfs_statfs_callout(FAR struct tmpfs_directory_s *tdo,
              unsigned int index, FAR void *arg);
static int  tmpfs_free_callout(FAR struct tmpfs_directory_s *tdo,
              unsigned int index, FAR void *arg);
static int  tmpfs_foreach(FAR struct tmpfs_directory_s *tdo,
              tmpfs_foreach_t callout, FAR void *arg);

/* File system operations */

static int  tmpfs_open(FAR struct file *filep, FAR const char *relpath,
              int oflags, mode_t mode);
static int  tmpfs_close(FAR struct file *filep);
static ssize_t tmpfs_read(FAR struct file *filep, FAR char *buffer,
              size_t buflen);
static ssize_t tmpfs_write(FAR struct file *filep, FAR const char *buffer,
              size_t buflen);
static off_t tmpfs_seek(FAR struct file *filep, off_t offset, int whence);
static int  tmpfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  tmpfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int  tmpfs_fstat(FAR const struct file *filep, FAR struct stat *buf);
static int  tmpfs_truncate(FAR struct file *filep, off_t length);

static int  tmpfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
              FAR struct fs_dirent_s *dir);
static int  tmpfs_closedir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  tmpfs_readdir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  tmpfs_rewinddir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  tmpfs_bind(FAR struct inode *blkdriver, FAR const void *data,
              FAR void **handle);
static int  tmpfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
              unsigned int flags);
static int  tmpfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf);
static int  tmpfs_unlink(FAR struct inode *mountpt, FAR const char *relpath);
static int  tmpfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
              mode_t mode);
static int  tmpfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath);
static int  tmpfs_rename(FAR struct inode *mountpt,
              FAR const char *oldrelpath, FAR const char *newrelpath);
static void tmpfs_stat_common(FAR struct tmpfs_object_s *to,
              FAR struct stat *buf);
static int  tmpfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
              FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations tmpfs_operations =
{
  tmpfs_open,       /* open */
  tmpfs_close,      /* close */
  tmpfs_read,       /* read */
  tmpfs_write,      /* write */
  tmpfs_seek,       /* seek */
  tmpfs_ioctl,      /* ioctl */

  NULL,             /* sync */
  tmpfs_dup,        /* dup */
  tmpfs_fstat,      /* fstat */
  tmpfs_truncate,   /* truncate */

  tmpfs_opendir,    /* opendir */
  tmpfs_closedir,   /* closedir */
  tmpfs_readdir,    /* readdir */
  tmpfs_rewinddir,  /* rewinddir */

  tmpfs_bind,       /* bind */
  tmpfs_unbind,     /* unbind */
  tmpfs_statfs,     /* statfs */

  tmpfs_unlink,     /* unlink */
  tmpfs_mkdir,      /* mkdir */
  tmpfs_rmdir,      /* rmdir */
  tmpfs_rename,     /* rename */
  tmpfs_stat,       /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmpfs_lock_reentrant
 ****************************************************************************/

static int tmpfs_lock_reentrant(FAR struct tmpfs_sem_s *sem)
{
  pid_t me;
  int ret = OK;

  /* Do we already hold the semaphore? */

  me = getpid();
  if (me == sem->ts_holder)
    {
      /* Yes... just increment the count */

      sem->ts_count++;
      DEBUGASSERT(sem->ts_count > 0);
    }

  /* Take the semaphore (perhaps waiting) */

  else
    {
      ret = nxsem_wait_uninterruptible(&sem->ts_sem);
      if (ret >= 0)
        {
          /* No we hold the semaphore */

          sem->ts_holder = me;
          sem->ts_count  = 1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tmpfs_lock
 ****************************************************************************/

static int tmpfs_lock(FAR struct tmpfs_s *fs)
{
  return tmpfs_lock_reentrant(&fs->tfs_exclsem);
}

/****************************************************************************
 * Name: tmpfs_lock_object
 ****************************************************************************/

static int tmpfs_lock_object(FAR struct tmpfs_object_s *to)
{
  return tmpfs_lock_reentrant(&to->to_exclsem);
}

/****************************************************************************
 * Name: tmpfs_unlock_reentrant
 ****************************************************************************/

static void tmpfs_unlock_reentrant(FAR struct tmpfs_sem_s *sem)
{
  DEBUGASSERT(sem->ts_holder == getpid());

  /* Is this our last count on the semaphore? */

  if (sem->ts_count > 1)
    {
      /* No.. just decrement the count */

      sem->ts_count--;
    }

  /* Yes.. then we can really release the semaphore */

  else
    {
      sem->ts_holder = TMPFS_NO_HOLDER;
      sem->ts_count  = 0;
      nxsem_post(&sem->ts_sem);
    }
}

/****************************************************************************
 * Name: tmpfs_unlock
 ****************************************************************************/

static void tmpfs_unlock(FAR struct tmpfs_s *fs)
{
  tmpfs_unlock_reentrant(&fs->tfs_exclsem);
}

/****************************************************************************
 * Name: tmpfs_unlock_object
 ****************************************************************************/

static void tmpfs_unlock_object(FAR struct tmpfs_object_s *to)
{
  tmpfs_unlock_reentrant(&to->to_exclsem);
}

/****************************************************************************
 * Name: tmpfs_realloc_directory
 ****************************************************************************/

static int tmpfs_realloc_directory(FAR struct tmpfs_directory_s **tdo,
                                   unsigned int nentries)
{
  FAR struct tmpfs_directory_s *oldtdo = *tdo;
  FAR struct tmpfs_directory_s *newtdo;
  size_t objsize;
  int ret = oldtdo->tdo_nentries;

  /* Get the new object size */

  objsize = SIZEOF_TMPFS_DIRECTORY(nentries);
  if (objsize <= oldtdo->tdo_alloc)
    {
      /* Already big enough.
       * REVISIT: Missing logic to shrink directory objects.
       */

      oldtdo->tdo_nentries = nentries;
      return ret;
    }

  /* Added some additional amount to the new size to account frequent
   * reallocations.
   */

  objsize += CONFIG_FS_TMPFS_DIRECTORY_ALLOCGUARD;

  /* Realloc the directory object */

  newtdo = (FAR struct tmpfs_directory_s *)kmm_realloc(oldtdo, objsize);
  if (newtdo == NULL)
    {
      return -ENOMEM;
    }

  /* Adjust the reference in the parent directory entry */

  DEBUGASSERT(newtdo->tdo_dirent);
  newtdo->tdo_dirent->tde_object = (FAR struct tmpfs_object_s *)newtdo;

  /* Return the new address of the reallocated directory object */

  newtdo->tdo_alloc    = objsize;
  newtdo->tdo_nentries = nentries;
  *tdo                 = newtdo;

  /* Return the index to the first, newly allocated directory entry */

  return ret;
}

/****************************************************************************
 * Name: tmpfs_realloc_file
 ****************************************************************************/

static int tmpfs_realloc_file(FAR struct tmpfs_file_s **tfo,
                              size_t newsize)
{
  FAR struct tmpfs_file_s *oldtfo = *tfo;
  FAR struct tmpfs_file_s *newtfo;
  size_t objsize;
  size_t allocsize;
  size_t delta;

  /* Check if the current allocation is sufficient */

  objsize = SIZEOF_TMPFS_FILE(newsize);

  /* Are we growing or shrinking the object? */

  if (objsize <= oldtfo->tfo_alloc)
    {
      /* Shrinking ... Shrink unconditionally if the size is shrinking to
       * zero.
       */

      if (newsize > 0)
        {
          /* Otherwise, don't realloc unless the object has shrunk by a
           * lot.
           */

          delta = oldtfo->tfo_alloc - objsize;
          if (delta <= CONFIG_FS_TMPFS_FILE_FREEGUARD)
            {
              /* Hasn't shrunk enough.. Return doing nothing for now */

              oldtfo->tfo_size = newsize;
              return OK;
            }
        }
    }

  /* Added some additional amount to the new size to account frequent
   * reallocations.
   */

  allocsize = objsize + CONFIG_FS_TMPFS_FILE_ALLOCGUARD;

  /* Realloc the file object */

  newtfo = (FAR struct tmpfs_file_s *)kmm_realloc(oldtfo, allocsize);
  if (newtfo == NULL)
    {
      return -ENOMEM;
    }

  /* Adjust the reference in the parent directory entry */

  DEBUGASSERT(newtfo->tfo_dirent);
  newtfo->tfo_dirent->tde_object = (FAR struct tmpfs_object_s *)newtfo;

  /* Return the new address of the reallocated file object */

  newtfo->tfo_alloc = allocsize;
  newtfo->tfo_size  = newsize;
  *tfo              = newtfo;
  return OK;
}

/****************************************************************************
 * Name: tmpfs_release_lockedobject
 ****************************************************************************/

static void tmpfs_release_lockedobject(FAR struct tmpfs_object_s *to)
{
  DEBUGASSERT(to && to->to_refs > 0);

  /* Is this a file object? */

  if (to->to_type == TMPFS_REGULAR)
    {
      tmpfs_release_lockedfile((FAR struct tmpfs_file_s *)to);
    }
  else
    {
      to->to_refs--;
      tmpfs_unlock_object(to);
    }
}

/****************************************************************************
 * Name: tmpfs_release_lockedfile
 ****************************************************************************/

static void tmpfs_release_lockedfile(FAR struct tmpfs_file_s *tfo)
{
  DEBUGASSERT(tfo && tfo->tfo_refs > 0);

  /* If there are no longer any references to the file and the file has been
   * unlinked from its parent directory, then free the file object now.
   */

  if (tfo->tfo_refs == 1 && (tfo->tfo_flags & TFO_FLAG_UNLINKED) != 0)
    {
      nxsem_destroy(&tfo->tfo_exclsem.ts_sem);
      kmm_free(tfo);
    }

  /* Otherwise, just decrement the reference count on the file object */

  else
    {
      tfo->tfo_refs--;
      tmpfs_unlock_file(tfo);
    }
}

/****************************************************************************
 * Name: tmpfs_find_dirent
 ****************************************************************************/

static int tmpfs_find_dirent(FAR struct tmpfs_directory_s *tdo,
                             FAR const char *name)
{
  int i;

  /* Search the list of directory entries for a match */

  for (i = 0;
       i < tdo->tdo_nentries &&
       strcmp(tdo->tdo_entry[i].tde_name, name) != 0;
       i++);

  /* Return what we found, if anything */

  return i < tdo->tdo_nentries ? i : -ENOENT;
}

/****************************************************************************
 * Name: tmpfs_remove_dirent
 ****************************************************************************/

static int tmpfs_remove_dirent(FAR struct tmpfs_directory_s *tdo,
                               FAR const char *name)
{
  int index;
  int last;

  /* Search the list of directory entries for a match */

  index = tmpfs_find_dirent(tdo, name);
  if (index < 0)
    {
      return index;
    }

  /* Free the object name */

  if (tdo->tdo_entry[index].tde_name != NULL)
    {
      kmm_free(tdo->tdo_entry[index].tde_name);
    }

  /* Remove by replacing this entry with the final directory entry */

  last = tdo->tdo_nentries - 1;
  if (index != last)
    {
      FAR struct tmpfs_dirent_s *newtde;
      FAR struct tmpfs_dirent_s *oldtde;
      FAR struct tmpfs_object_s *to;

      /* Move the directory entry */

      newtde             = &tdo->tdo_entry[index];
      oldtde             = &tdo->tdo_entry[last];
      to                 = oldtde->tde_object;

      newtde->tde_object = to;
      newtde->tde_name   = oldtde->tde_name;

      /* Reset the backward link to the directory entry */

      to->to_dirent      = newtde;
    }

  /* And decrement the count of directory entries */

  tdo->tdo_nentries = last;
  return OK;
}

/****************************************************************************
 * Name: tmpfs_add_dirent
 ****************************************************************************/

static int tmpfs_add_dirent(FAR struct tmpfs_directory_s **tdo,
                            FAR struct tmpfs_object_s *to,
                            FAR const char *name)
{
  FAR struct tmpfs_directory_s *oldtdo;
  FAR struct tmpfs_directory_s *newtdo;
  FAR struct tmpfs_dirent_s *tde;
  FAR char *newname;
  unsigned int nentries;
  int index;

  /* Copy the name string so that it will persist as long as the
   * directory entry.
   */

  newname = strdup(name);
  if (newname == NULL)
    {
      return -ENOMEM;
    }

  /* Get the new number of entries */

  oldtdo = *tdo;
  nentries = oldtdo->tdo_nentries + 1;

  /* Reallocate the directory object (if necessary) */

  index = tmpfs_realloc_directory(tdo, nentries);
  if (index < 0)
    {
      kmm_free(newname);
      return index;
    }

  /* Save the new object info in the new directory entry */

  newtdo          = *tdo;
  tde             = &newtdo->tdo_entry[index];
  tde->tde_object = to;
  tde->tde_name   = newname;

  /* Add backward link to the directory entry to the object */

  to->to_dirent  = tde;
  return OK;
}

/****************************************************************************
 * Name: tmpfs_alloc_file
 ****************************************************************************/

static FAR struct tmpfs_file_s *tmpfs_alloc_file(void)
{
  FAR struct tmpfs_file_s *tfo;
  size_t allocsize;

  /* Create a new zero length file object */

  allocsize = SIZEOF_TMPFS_FILE(CONFIG_FS_TMPFS_FILE_ALLOCGUARD);
  tfo = (FAR struct tmpfs_file_s *)kmm_malloc(allocsize);
  if (tfo == NULL)
    {
      return NULL;
    }

  /* Initialize the new file object.  NOTE that the initial state is
   * locked with one reference count.
   */

  tfo->tfo_alloc = allocsize;
  tfo->tfo_type  = TMPFS_REGULAR;
  tfo->tfo_refs  = 1;
  tfo->tfo_flags = 0;
  tfo->tfo_size  = 0;

  tfo->tfo_exclsem.ts_holder = getpid();
  tfo->tfo_exclsem.ts_count  = 1;
  nxsem_init(&tfo->tfo_exclsem.ts_sem, 0, 0);

  return tfo;
}

/****************************************************************************
 * Name: tmpfs_create_file
 ****************************************************************************/

static int tmpfs_create_file(FAR struct tmpfs_s *fs,
                             FAR const char *relpath,
                             FAR struct tmpfs_file_s **tfo)
{
  FAR struct tmpfs_directory_s *parent;
  FAR struct tmpfs_file_s *newtfo;
  FAR char *copy;
  FAR char *name;
  int ret;

  /* Duplicate the path variable so that we can modify it */

  copy = strdup(relpath);
  if (copy == NULL)
    {
      return -ENOMEM;
    }

  /* Separate the path into the file name and the path to the parent
   * directory.
   */

  name = strrchr(copy, '/');
  if (name == NULL)
    {
      /* No subdirectories... use the root directory */

      name   = copy;
      parent = (FAR struct tmpfs_directory_s *)fs->tfs_root.tde_object;

      /* Lock the root directory to emulate the behavior of
       * tmpfs_find_directory()
       */

      ret = tmpfs_lock_directory(parent);
      if (ret < 0)
        {
          kmm_free(copy);
          return ret;
        }

      parent->tdo_refs++;
    }
  else
    {
      /* Terminate the parent directory path */

      *name++ = '\0';

      /* Locate the parent directory that should contain this name.
       * On success, tmpfs_find_directory() will lock the parent
       * directory and increment the reference count.
       */

      ret = tmpfs_find_directory(fs, copy, &parent, NULL);
      if (ret < 0)
        {
          goto errout_with_copy;
        }
    }

  /* Verify that no object of this name already exists in the directory */

  ret = tmpfs_find_dirent(parent, name);
  if (ret != -ENOENT)
    {
      /* Something with this name already exists in the directory.
       * OR perhaps some fatal error occurred.
       */

      if (ret >= 0)
        {
          ret = -EEXIST;
        }

      goto errout_with_parent;
    }

  /* Allocate an empty file.  The initial state of the file is locked with
   * one reference count.
   */

  newtfo = tmpfs_alloc_file();
  if (newtfo == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_parent;
    }

  /* Then add the new, empty file to the directory */

  ret = tmpfs_add_dirent(&parent, (FAR struct tmpfs_object_s *)newtfo, name);
  if (ret < 0)
    {
      goto errout_with_file;
    }

  /* Release the reference and lock on the parent directory */

  parent->tdo_refs--;
  tmpfs_unlock_directory(parent);

  /* Free the copy of the relpath and return success */

  kmm_free(copy);
  *tfo = newtfo;
  return OK;

  /* Error exits */

errout_with_file:
  nxsem_destroy(&newtfo->tfo_exclsem.ts_sem);
  kmm_free(newtfo);

errout_with_parent:
  parent->tdo_refs--;
  tmpfs_unlock_directory(parent);

errout_with_copy:
  kmm_free(copy);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_alloc_directory
 ****************************************************************************/

static FAR struct tmpfs_directory_s *tmpfs_alloc_directory(void)
{
  FAR struct tmpfs_directory_s *tdo;
  size_t allocsize;
  unsigned int nentries;

  /* Convert the pre-allocated memory to a number of directory entries */

  nentries = (CONFIG_FS_TMPFS_DIRECTORY_ALLOCGUARD +
              sizeof(struct tmpfs_dirent_s) - 1) /
             sizeof(struct tmpfs_dirent_s);

  /* Create a new zero length directory object */

  allocsize = SIZEOF_TMPFS_DIRECTORY(nentries);
  tdo = (FAR struct tmpfs_directory_s *)kmm_malloc(allocsize);
  if (tdo == NULL)
    {
      return NULL;
    }

  /* Initialize the new directory object */

  tdo->tdo_alloc    = allocsize;
  tdo->tdo_type     = TMPFS_DIRECTORY;
  tdo->tdo_refs     = 0;
  tdo->tdo_nentries = 0;

  tdo->tdo_exclsem.ts_holder = TMPFS_NO_HOLDER;
  tdo->tdo_exclsem.ts_count  = 0;
  nxsem_init(&tdo->tdo_exclsem.ts_sem, 0, 1);

  return tdo;
}

/****************************************************************************
 * Name: tmpfs_create_directory
 ****************************************************************************/

static int tmpfs_create_directory(FAR struct tmpfs_s *fs,
                                  FAR const char *relpath,
                                  FAR struct tmpfs_directory_s **tdo)
{
  FAR struct tmpfs_directory_s *parent;
  FAR struct tmpfs_directory_s *newtdo;
  FAR char *copy;
  FAR char *name;
  int ret;

  /* Duplicate the path variable so that we can modify it */

  copy = strdup(relpath);
  if (copy == NULL)
    {
      return -ENOMEM;
    }

  /* Separate the path into the file name and the path to the parent
   * directory.
   */

  name = strrchr(copy, '/');
  if (name == NULL)
    {
      /* No subdirectories... use the root directory */

      name   = copy;
      parent = (FAR struct tmpfs_directory_s *)fs->tfs_root.tde_object;

      ret = tmpfs_lock_directory(parent);
      if (ret < 0)
        {
          kmm_free(copy);
          return ret;
        }

      parent->tdo_refs++;
    }
  else
    {
      /* Terminate the parent directory path */

      *name++ = '\0';

      /* Locate the parent directory that should contain this name.
       * On success, tmpfs_find_directory() will lockthe parent
       * directory and increment the reference count.
       */

      ret = tmpfs_find_directory(fs, copy, &parent, NULL);
      if (ret < 0)
        {
          goto errout_with_copy;
        }
    }

  /* Verify that no object of this name already exists in the directory */

  ret = tmpfs_find_dirent(parent, name);
  if (ret != -ENOENT)
    {
      /* Something with this name already exists in the directory.
       * OR perhaps some fatal error occurred.
       */

      if (ret >= 0)
        {
          ret = -EEXIST;
        }

      goto errout_with_parent;
    }

  /* Allocate an empty directory object.  NOTE that there is no reference on
   * the new directory and the object is not locked.
   */

  newtdo = tmpfs_alloc_directory();
  if (newtdo == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_parent;
    }

  /* Then add the new, empty file to the directory */

  ret = tmpfs_add_dirent(&parent, (FAR struct tmpfs_object_s *)newtdo, name);
  if (ret < 0)
    {
      goto errout_with_directory;
    }

  /* Free the copy of the relpath, release our reference to the parent
   * directory, and return success
   */

  parent->tdo_refs--;
  tmpfs_unlock_directory(parent);
  kmm_free(copy);

  /* Return the (unlocked, unreferenced) directory object to the caller */

  if (tdo != NULL)
    {
      *tdo = newtdo;
    }

  return OK;

  /* Error exits */

errout_with_directory:
  nxsem_destroy(&newtdo->tdo_exclsem.ts_sem);
  kmm_free(newtdo);

errout_with_parent:
  parent->tdo_refs--;
  tmpfs_unlock_directory(parent);

errout_with_copy:
  kmm_free(copy);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_find_object
 ****************************************************************************/

static int tmpfs_find_object(FAR struct tmpfs_s *fs,
                             FAR const char *relpath,
                             FAR struct tmpfs_object_s **object,
                             FAR struct tmpfs_directory_s **parent)
{
  FAR struct tmpfs_object_s *to = NULL;
  FAR struct tmpfs_directory_s *tdo = NULL;
  FAR struct tmpfs_directory_s *next_tdo;
  FAR char *segment;
  FAR char *next_segment;
  FAR char *tkptr;
  FAR char *copy;
  int index;
  int ret;

  /* Make a copy of the path (so that we can modify it via strtok) */

  copy = strdup(relpath);
  if (copy == NULL)
    {
      return -ENOMEM;
    }

  /* Traverse the file system for any object with the matching name */

  to       = fs->tfs_root.tde_object;
  next_tdo = (FAR struct tmpfs_directory_s *)fs->tfs_root.tde_object;

  for (segment =  strtok_r(copy, "/", &tkptr);
       segment != NULL;
       segment = next_segment)
    {
      /* Get the next segment after the one we are currently working on.
       * This will be NULL is we are working on the final segment of the
       * relpath.
       */

      next_segment = strtok_r(NULL, "/", &tkptr);

      /* Search the next directory. */

      tdo = next_tdo;

      /* Find the TMPFS object with the next segment name in the current
       * directory.
       */

      index = tmpfs_find_dirent(tdo, segment);
      if (index < 0)
        {
          /* No object with this name exists in the directory. */

          kmm_free(copy);
          return index;
        }

      to = tdo->tdo_entry[index].tde_object;

      /* Is this object another directory? */

      if (to->to_type != TMPFS_DIRECTORY)
        {
          /* No.  Was this the final segment in the path? */

          if (next_segment == NULL)
            {
              /* Then we can break out of the loop now */

               break;
            }

          /* No, this was not the final segment of the relpath.
           * We cannot continue the search if any of the intermediate
           * segments do no correspond to directories.
           */

          kmm_free(copy);
          return -ENOTDIR;
        }

      /* Search this directory for the next segment.  If we
       * exit the loop, tdo will still refer to the parent
       * directory of to.
       */

      next_tdo = (FAR struct tmpfs_directory_s *)to;
    }

  /* When we exit this loop (successfully), to will point to the TMPFS
   * object associated with the terminal segment of the relpath.
   * Increment the reference count on the located object.
   */

  /* Free the dup'ed string */

  kmm_free(copy);

  /* Return what we found */

  if (parent)
    {
      if (tdo != NULL)
        {
          /* Get exclusive access to the parent and increment the reference
           * count on the object.
           */

          ret = tmpfs_lock_directory(tdo);
          if (ret < 0)
            {
              return ret;
            }

          tdo->tdo_refs++;
        }

      *parent = tdo;
    }

  if (object)
    {
      if (to != NULL)
        {
          /* Get exclusive access to the object and increment the reference
           * count on the object.
           */

          ret = tmpfs_lock_object(to);
          if (ret < 0)
            {
              return ret;
            }

          to->to_refs++;
        }

      *object = to;
    }

  return OK;
}

/****************************************************************************
 * Name: tmpfs_find_file
 ****************************************************************************/

static int tmpfs_find_file(FAR struct tmpfs_s *fs,
                           FAR const char *relpath,
                           FAR struct tmpfs_file_s **tfo,
                           FAR struct tmpfs_directory_s **parent)
{
  FAR struct tmpfs_object_s *to;
  int ret;

  /* Find the object at this path.  If successful, tmpfs_find_object() will
   * lock both the object and the parent directory and will increment the
   * reference count on both.
   */

  ret = tmpfs_find_object(fs, relpath, &to, parent);
  if (ret >= 0)
    {
      /* We found it... but is it a regular file? */

      if (to->to_type != TMPFS_REGULAR)
        {
          /* No... unlock the object and its parent and return an error */

          tmpfs_release_lockedobject(to);

          if (parent)
            {
              FAR struct tmpfs_directory_s *tdo = *parent;

              tdo->tdo_refs--;
              tmpfs_unlock_directory(tdo);
            }

          ret = -EISDIR;
        }

      /* Return the verified file object */

      *tfo = (FAR struct tmpfs_file_s *)to;
    }

  return ret;
}

/****************************************************************************
 * Name: tmpfs_find_directory
 ****************************************************************************/

static int tmpfs_find_directory(FAR struct tmpfs_s *fs,
                           FAR const char *relpath,
                           FAR struct tmpfs_directory_s **tdo,
                           FAR struct tmpfs_directory_s **parent)
{
  FAR struct tmpfs_object_s *to;
  int ret;

  /* Find the object at this path */

  ret = tmpfs_find_object(fs, relpath, &to, parent);
  if (ret >= 0)
    {
      /* We found it... but is it a regular file? */

      if (to->to_type != TMPFS_DIRECTORY)
        {
          /* No... unlock the object and its parent and return an error */

          tmpfs_release_lockedobject(to);

          if (parent)
            {
              FAR struct tmpfs_directory_s *tmptdo = *parent;

              tmptdo->tdo_refs--;
              tmpfs_unlock_directory(tmptdo);
            }

          ret = -ENOTDIR;
        }

      /* Return the verified file object */

      *tdo = (FAR struct tmpfs_directory_s *)to;
    }

  return ret;
}

/****************************************************************************
 * Name: tmpfs_statfs_callout
 ****************************************************************************/

static int tmpfs_statfs_callout(FAR struct tmpfs_directory_s *tdo,
                                unsigned int index, FAR void *arg)
{
  FAR struct tmpfs_object_s *to;
  FAR struct tmpfs_statfs_s *tmpbuf;

  DEBUGASSERT(tdo != NULL && arg != NULL && index < tdo->tdo_nentries);

  to     = tdo->tdo_entry[index].tde_object;
  tmpbuf = (FAR struct tmpfs_statfs_s *)arg;

  DEBUGASSERT(to != NULL);

  /* Accumulate statistics.  Save the total memory allocated
   * for this object.
   */

  tmpbuf->tsf_alloc += to->to_alloc;

  /* Is this directory entry a file object? */

  if (to->to_type == TMPFS_REGULAR)
    {
      FAR struct tmpfs_file_s *tmptfo;

      /* It is a file object.  Increment the number of files and update the
       * amount of memory in use.
       */

      tmptfo             = (FAR struct tmpfs_file_s *)to;
      tmpbuf->tsf_inuse += tmptfo->tfo_size;
      tmpbuf->tsf_files++;
    }
  else /* if (to->to_type == TMPFS_DIRECTORY) */
    {
      FAR struct tmpfs_directory_s *tmptdo;
      size_t inuse;
      size_t avail;

      /* It is a directory object.  Update the amount of memory in use
       * for the directory and estimate the number of free directory nodes.
       */

      tmptdo = (FAR struct tmpfs_directory_s *)to;
      inuse  = SIZEOF_TMPFS_DIRECTORY(tmptdo->tdo_nentries);
      avail  = tmptdo->tdo_alloc - inuse;

      tmpbuf->tsf_inuse += inuse;
      tmpbuf->tsf_ffree += avail / sizeof(struct tmpfs_dirent_s);
    }

  return TMPFS_CONTINUE;
}

/****************************************************************************
 * Name: tmpfs_free_callout
 ****************************************************************************/

static int tmpfs_free_callout(FAR struct tmpfs_directory_s *tdo,
                              unsigned int index, FAR void *arg)
{
  FAR struct tmpfs_dirent_s *tde;
  FAR struct tmpfs_object_s *to;
  FAR struct tmpfs_file_s *tfo;
  unsigned int last;

  /* Free the object name */

  if (tdo->tdo_entry[index].tde_name != NULL)
    {
      kmm_free(tdo->tdo_entry[index].tde_name);
    }

  /* Remove by replacing this entry with the final directory entry */

  tde  = &tdo->tdo_entry[index];
  to   = tde->tde_object;
  last = tdo->tdo_nentries - 1;

  if (index != last)
    {
      FAR struct tmpfs_dirent_s *oldtde;
      FAR struct tmpfs_object_s *oldto;

      /* Move the directory entry */

      oldtde           = &tdo->tdo_entry[last];
      oldto            = oldtde->tde_object;

      tde->tde_object  = oldto;
      tde->tde_name    = oldtde->tde_name;

      /* Reset the backward link to the directory entry */

      oldto->to_dirent = tde;
    }

  /* And decrement the count of directory entries */

  tdo->tdo_nentries = last;

  /* Is this directory entry a file object? */

  if (to->to_type == TMPFS_REGULAR)
    {
      tfo = (FAR struct tmpfs_file_s *)to;

      /* Are there references to the file? */

      if (tfo->tfo_refs > 0)
        {
          /* Yes.. We cannot delete the file now. Just mark it as unlinked. */

          tfo->tfo_flags |= TFO_FLAG_UNLINKED;
          return TMPFS_UNLINKED;
        }
    }

  /* Free the object now */

  nxsem_destroy(&to->to_exclsem.ts_sem);
  kmm_free(to);
  return TMPFS_DELETED;
}

/****************************************************************************
 * Name: tmpfs_foreach
 ****************************************************************************/

static int tmpfs_foreach(FAR struct tmpfs_directory_s *tdo,
                         tmpfs_foreach_t callout, FAR void *arg)
{
  FAR struct tmpfs_object_s *to;
  unsigned int index;
  int ret;

  /* Visit each directory entry */

  for (index = 0; index < tdo->tdo_nentries; )
    {
      /* Lock the object and take a reference */

      to  = tdo->tdo_entry[index].tde_object;
      ret = tmpfs_lock_object(to);
      if (ret < 0)
        {
          return ret;
        }

      to->to_refs++;

      /* Is the next entry a directory? */

      if (to->to_type == TMPFS_DIRECTORY)
        {
          FAR struct tmpfs_directory_s *next =
            (FAR struct tmpfs_directory_s *)to;

          /* Yes.. traverse its children first in the case the final
           * action will be to delete the directory.
           */

          ret = tmpfs_foreach(next, callout, arg);
          if (ret < 0)
            {
              return -ECANCELED;
            }
        }

      /* Perform the callout */

      ret = callout(tdo, index, arg);
      switch (ret)
        {
         case TMPFS_CONTINUE:    /* Continue enumeration */

           /* Release the object and index to the next entry */

           tmpfs_release_lockedobject(to);
           index++;
           break;

         case TMPFS_HALT:        /* Stop enumeration */

           /* Release the object and cancel the traversal */

           tmpfs_release_lockedobject(to);
           return -ECANCELED;

         case TMPFS_UNLINKED:    /* Only the directory entry was deleted */

           /* Release the object and continue with the same index */

           tmpfs_release_lockedobject(to);

         case TMPFS_DELETED:     /* Object and directory entry deleted */
           break;                /* Continue with the same index */
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tmpfs_open
 ****************************************************************************/

static int tmpfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_file_s *tfo;
  off_t offset;
  int ret;

  finfo("filep: %p\n", filep);
  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Skip over any leading directory separators (shouldn't be any) */

  for (; *relpath == '/'; relpath++);

  /* Find the file object associated with this relative path.
   * If successful, this action will lock both the parent directory and
   * the file object, adding one to the reference count of both.
   * In the event that -ENOENT, there will still be a reference and
   * lock on the returned directory.
   */

  ret = tmpfs_find_file(fs, relpath, &tfo, NULL);
  if (ret >= 0)
    {
      /* The file exists.  We hold the lock and one reference count
       * on the file object.
       *
       * It would be an error if we are asked to create it exclusively
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          /* Already exists -- can't create it exclusively */

          ret = -EEXIST;
          goto errout_with_filelock;
        }

      /* Check if the caller has sufficient privileges to open the file.
       * REVISIT: No file protection implemented
       */

      /* If O_TRUNC is specified and the file is opened for writing,
       * then truncate the file.  This operation requires that the file is
       * writeable, but we have already checked that. O_TRUNC without write
       * access is ignored.
       */

      if ((oflags & (O_TRUNC | O_WRONLY)) == (O_TRUNC | O_WRONLY))
        {
          /* Truncate the file to zero length (if it is not already
           * zero length)
           */

          if (tfo->tfo_size > 0)
            {
              ret = tmpfs_realloc_file(&tfo, 0);
              if (ret < 0)
                {
                  goto errout_with_filelock;
                }
            }
        }
    }

  /* ENOENT would be returned by tmpfs_find_file() if the full directory
   * path was found, but the file was not found in the final directory.
   */

  else if (ret == -ENOENT)
    {
      /* The file does not exist.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* No.. then we fail with -ENOENT */

          ret = -ENOENT;
          goto errout_with_fslock;
        }

      /* Yes.. create the file object.  There will be a reference and a lock
       * on the new file object.
       */

      ret = tmpfs_create_file(fs, relpath, &tfo);
      if (ret < 0)
        {
          goto errout_with_fslock;
        }
    }

  /* Some other error occurred */

  else
    {
      goto errout_with_fslock;
    }

  /* Save the struct tmpfs_file_s instance as the file private data */

  filep->f_priv = tfo;

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  offset = 0;
  if ((oflags & (O_APPEND | O_WRONLY)) == (O_APPEND | O_WRONLY))
    {
      offset = tfo->tfo_size;
    }

  filep->f_pos = offset;

  /* Unlock the file file object, but retain the reference count */

  tmpfs_unlock_file(tfo);
  tmpfs_unlock(fs);
  return OK;

  /* Error exits */

errout_with_filelock:
  tmpfs_release_lockedfile(tfo);

errout_with_fslock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_close
 ****************************************************************************/

static int tmpfs_close(FAR struct file *filep)
{
  FAR struct tmpfs_file_s *tfo;
  int ret;

  finfo("filep: %p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  ret = tmpfs_lock_file(tfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count on the file */

  DEBUGASSERT(tfo->tfo_refs > 0);
  if (tfo->tfo_refs > 0)
    {
      tfo->tfo_refs--;
    }

  filep->f_priv = NULL;

  /* If the reference count decremented to zero and the file has been
   * unlinked, then free the file allocation now.
   */

  if (tfo->tfo_refs == 0 && (tfo->tfo_flags & TFO_FLAG_UNLINKED) != 0)
    {
      /* Free the file object while we hold the lock?  Weird but this
       * should be safe because the object is unlinked and could not
       * have any other references.
       */

      kmm_free(tfo);
      return OK;
    }

  /* Release the lock on the file */

  tmpfs_unlock_file(tfo);
  return OK;
}

/****************************************************************************
 * Name: tmpfs_read
 ****************************************************************************/

static ssize_t tmpfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct tmpfs_file_s *tfo;
  ssize_t nread;
  off_t startpos;
  off_t endpos;
  int ret;

  finfo("filep: %p buffer: %p buflen: %lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  ret = tmpfs_lock_file(tfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle attempts to read beyond the end of the file. */

  startpos = filep->f_pos;
  nread    = buflen;
  endpos   = startpos + buflen;

  if (endpos > tfo->tfo_size)
    {
      endpos = tfo->tfo_size;
      nread  = endpos - startpos;
    }

  /* Copy data from the memory object to the user buffer */

  memcpy(buffer, &tfo->tfo_data[startpos], nread);
  filep->f_pos += nread;

  /* Release the lock on the file */

  tmpfs_unlock_file(tfo);
  return nread;
}

/****************************************************************************
 * Name: tmpfs_write
 ****************************************************************************/

static ssize_t tmpfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct tmpfs_file_s *tfo;
  ssize_t nwritten;
  off_t startpos;
  off_t endpos;
  int ret;

  finfo("filep: %p buffer: %p buflen: %lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  ret = tmpfs_lock_file(tfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle attempts to write beyond the end of the file */

  startpos = filep->f_pos;
  nwritten = buflen;
  endpos   = startpos + buflen;

  if (endpos > tfo->tfo_size)
    {
      /* Reallocate the file to handle the write past the end of the file. */

      ret = tmpfs_realloc_file(&tfo, (size_t)endpos);
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      filep->f_priv = tfo;
    }

  /* Copy data from the memory object to the user buffer */

  memcpy(&tfo->tfo_data[startpos], buffer, nwritten);
  filep->f_pos += nwritten;

  /* Release the lock on the file */

  tmpfs_unlock_file(tfo);
  return nwritten;

errout_with_lock:
  tmpfs_unlock_file(tfo);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: tmpfs_seek
 ****************************************************************************/

static off_t tmpfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct tmpfs_file_s *tfo;
  off_t position;

  finfo("filep: %p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET: /* The offset is set to offset bytes. */
          position = offset;
          break;

      case SEEK_CUR: /* The offset is set to its current location plus
                      * offset bytes. */
          position = offset + filep->f_pos;
          break;

      case SEEK_END: /* The offset is set to the size of the file plus
                      * offset bytes. */
          position = offset + tfo->tfo_size;
          break;

      default:
          return -EINVAL;
    }

  /* Attempts to set the position beyond the end of file will
   * work if the file is open for write access.
   *
   * REVISIT: This simple implementation has no per-open storage that
   * would be needed to retain the open flags.
   */

#if 0
  if (position > tfo->tfo_size && (tfo->tfo_oflags & O_WROK) == 0)
    {
      /* Otherwise, the position is limited to the file size */

      position = tfo->tfo_size;
    }
#endif

  /* Save the new file position */

  filep->f_pos = position;
  return position;
}

/****************************************************************************
 * Name: tmpfs_ioctl
 ****************************************************************************/

static int tmpfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct tmpfs_file_s *tfo;
  FAR void **ppv = (FAR void**)arg;

  finfo("filep: %p cmd: %d arg: %08lx\n", filep, cmd, arg);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  DEBUGASSERT(tfo != NULL);

  /* Only one ioctl command is supported */

  if (cmd == FIOC_MMAP && ppv != NULL)
    {
      /* Return the address on the media corresponding to the start of
       * the file.
       */

      *ppv = (FAR void *)tfo->tfo_data;
      return OK;
    }

  ferr("ERROR: Invalid cmd: %d\n", cmd);
  return -ENOTTY;
}

/****************************************************************************
 * Name: tmpfs_dup
 ****************************************************************************/

static int tmpfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct tmpfs_file_s *tfo;
  int ret;

  finfo("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL &&
              newp->f_priv == NULL && newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = oldp->f_priv;
  DEBUGASSERT(tfo != NULL);

  /* Increment the reference count (atomically) */

  ret = tmpfs_lock_file(tfo);
  if (ret >= 0)
    {
      tfo->tfo_refs++;
      tmpfs_unlock_file(tfo);

      /* Save a copy of the file object as the dup'ed file.  This
       * simple implementation does not many any per-open data
       * structures so there is not really much to the dup operation.
       */

      newp->f_priv = tfo;
      ret = OK;
    }

  return OK;
}

/****************************************************************************
 * Name: tmpfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int tmpfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct tmpfs_file_s *tfo;
  int ret;

  finfo("Fstat %p\n", buf);
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  ret = tmpfs_lock_file(tfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Return information about the file in the stat buffer. */

  tmpfs_stat_common((FAR struct tmpfs_object_s *)tfo, buf);

  /* Release the lock on the file and return success. */

  tmpfs_unlock_file(tfo);
  return OK;
}

/****************************************************************************
 * Name: tmpfs_truncate
 ****************************************************************************/

static int tmpfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct tmpfs_file_s *tfo;
  size_t oldsize;
  int ret;

  finfo("filep: %p length: %ld\n", filep, (long)length);
  DEBUGASSERT(filep != NULL && length >= 0);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  ret = tmpfs_lock_file(tfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the old size of the file.  Do nothing if the file size is not
   * changing.
   */

  oldsize = tfo->tfo_size;
  if (oldsize != length)
    {
      /* The size is changing.. up or down.  Reallocate the file memory. */

      ret = tmpfs_realloc_file(&tfo, (size_t)length);
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      filep->f_priv = tfo;

      /* If the size has increased, then we need to zero the newly added
       * memory.
       */

      if (length > oldsize)
        {
          memset(&tfo->tfo_data[oldsize], 0, length - oldsize);
        }

      ret = OK;
    }

  /* Release the lock on the file */

errout_with_lock:
  tmpfs_unlock_file(tfo);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_opendir
 ****************************************************************************/

static int tmpfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *tdo;
  int ret;

  finfo("mountpt: %p relpath: %s dir: %p\n",
        mountpt, relpath, dir);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && dir != NULL);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Skip over any leading directory separators (shouldn't be any) */

  for (; *relpath == '/'; relpath++);

  /* Find the directory object associated with this relative path.
   * If successful, this action will lock both the parent directory and
   * the file object, adding one to the reference count of both.
   * In the event that -ENOENT, there will still be a reference and
   * lock on the returned directory.
   */

  ret = tmpfs_find_directory(fs, relpath, &tdo, NULL);
  if (ret >= 0)
    {
      dir->u.tmpfs.tf_tdo   = tdo;
      dir->u.tmpfs.tf_index = 0;

      tmpfs_unlock_directory(tdo);
    }

  /* Release the lock on the file system and return the result */

  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_closedir
 ****************************************************************************/

static int tmpfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_directory_s *tdo;

  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Get the directory structure from the dir argument */

  tdo = dir->u.tmpfs.tf_tdo;
  DEBUGASSERT(tdo != NULL);

  /* Decrement the reference count on the directory object */

  tmpfs_lock_directory(tdo);
  tdo->tdo_refs--;
  tmpfs_unlock_directory(tdo);
  return OK;
}

/****************************************************************************
 * Name: tmpfs_readdir
 ****************************************************************************/

static int tmpfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_directory_s *tdo;
  unsigned int index;
  int ret;

  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Get the directory structure from the dir argument and lock it */

  tdo = dir->u.tmpfs.tf_tdo;
  DEBUGASSERT(tdo != NULL);

  tmpfs_lock_directory(tdo);

  /* Have we reached the end of the directory? */

  index = dir->u.tmpfs.tf_index;
  if (index >= tdo->tdo_nentries)
    {
      /* We signal the end of the directory by returning the special error:
       * -ENOENT
       */

      finfo("End of directory\n");
      ret = -ENOENT;
    }
  else
    {
      FAR struct tmpfs_dirent_s *tde;
      FAR struct tmpfs_object_s *to;

      /* Does this entry refer to a file or a directory object? */

      tde = &tdo->tdo_entry[index];
      to  = tde->tde_object;
      DEBUGASSERT(to != NULL);

      if (to->to_type == TMPFS_DIRECTORY)
        {
          /* A directory */

           dir->fd_dir.d_type = DTYPE_DIRECTORY;
        }
      else /* to->to_type == TMPFS_REGULAR) */
        {
          /* A regular file */

           dir->fd_dir.d_type = DTYPE_FILE;
        }

      /* Copy the entry name */

      strncpy(dir->fd_dir.d_name, tde->tde_name, NAME_MAX + 1);

      /* Increment the index for next time */

      dir->u.tmpfs.tf_index = index + 1;
      ret = OK;
    }

  tmpfs_unlock_directory(tdo);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_rewinddir
 ****************************************************************************/

static int tmpfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Set the readdir index to zero */

  dir->u.tmpfs.tf_index = 0;
  return OK;
}

/****************************************************************************
 * Name: tmpfs_bind
 ****************************************************************************/

static int tmpfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct tmpfs_directory_s *tdo;
  FAR struct tmpfs_s *fs;

  finfo("blkdriver: %p data: %p handle: %p\n", blkdriver, data, handle);
  DEBUGASSERT(blkdriver == NULL && handle != NULL);

  /* Create an instance of the tmpfs file system */

  fs = (FAR struct tmpfs_s *)kmm_zalloc(sizeof(struct tmpfs_s));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* Create a root file system.  This is like a single directory entry in
   * the file system structure.
   */

  tdo = tmpfs_alloc_directory();
  if (tdo == NULL)
    {
      kmm_free(fs);
      return -ENOMEM;
    }

  fs->tfs_root.tde_object = (FAR struct tmpfs_object_s *)tdo;
  fs->tfs_root.tde_name   = "";

  /* Set up the backward link (to support reallocation) */

  tdo->tdo_dirent         = &fs->tfs_root;

  /* Initialize the file system state */

  fs->tfs_exclsem.ts_holder = TMPFS_NO_HOLDER;
  fs->tfs_exclsem.ts_count  = 0;
  nxsem_init(&fs->tfs_exclsem.ts_sem, 0, 1);

  /* Return the new file system handle */

  *handle = (FAR void *)fs;
  return OK;
}

/****************************************************************************
 * Name: tmpfs_unbind
 ****************************************************************************/

static int tmpfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags)
{
  FAR struct tmpfs_s *fs = (FAR struct tmpfs_s *)handle;
  FAR struct tmpfs_directory_s *tdo;
  int ret;

  finfo("handle: %p blkdriver: %p flags: %02x\n",
        handle, blkdriver, flags);
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Lock the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Traverse all directory entries (recursively), freeing all resources. */

  tdo = (FAR struct tmpfs_directory_s *)fs->tfs_root.tde_object;
  ret = tmpfs_foreach(tdo, tmpfs_free_callout, NULL);

  /* Now we can destroy the root file system and the file system itself. */

  nxsem_destroy(&tdo->tdo_exclsem.ts_sem);
  kmm_free(tdo);

  nxsem_destroy(&fs->tfs_exclsem.ts_sem);
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_statfs
 ****************************************************************************/

static int tmpfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *tdo;
  struct tmpfs_statfs_s tmpbuf;
  size_t inuse;
  size_t avail;
  off_t blkalloc;
  off_t blkused;
  int ret;

  finfo("mountpt: %p buf: %p\n", mountpt, buf);
  DEBUGASSERT(mountpt != NULL && buf != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Set up the memory use for the file system and root directory object */

  tdo              = (FAR struct tmpfs_directory_s *)fs->tfs_root.tde_object;
  inuse            = sizeof(struct tmpfs_s) +
                     SIZEOF_TMPFS_DIRECTORY(tdo->tdo_nentries);
  avail            = sizeof(struct tmpfs_s) +
                     tdo->tdo_alloc - inuse;

  tmpbuf.tsf_alloc = tdo->tdo_alloc;
  tmpbuf.tsf_inuse = inuse;
  tmpbuf.tsf_files = 0;
  tmpbuf.tsf_ffree = avail / sizeof(struct tmpfs_dirent_s);

  /* Traverse the file system to accurmulate statistics */

  ret = tmpfs_foreach(tdo, tmpfs_statfs_callout, (FAR void *)&tmpbuf);
  if (ret < 0)
    {
      return -ECANCELED;
    }

  /* Return something for the file system description */

  blkalloc        = (tmpbuf.tsf_alloc + CONFIG_FS_TMPFS_BLOCKSIZE - 1) /
                     CONFIG_FS_TMPFS_BLOCKSIZE;
  blkused         = (tmpbuf.tsf_inuse + CONFIG_FS_TMPFS_BLOCKSIZE - 1) /
                     CONFIG_FS_TMPFS_BLOCKSIZE;

  buf->f_type     = TMPFS_MAGIC;
  buf->f_namelen  = NAME_MAX;
  buf->f_bsize    = CONFIG_FS_TMPFS_BLOCKSIZE;
  buf->f_blocks   = blkalloc;
  buf->f_bfree    = blkalloc - blkused;
  buf->f_bavail   = blkalloc - blkused;
  buf->f_files    = tmpbuf.tsf_files;
  buf->f_ffree    = tmpbuf.tsf_ffree;

  /* Release the lock on the file system */

  tmpfs_unlock(fs);
  return OK;
}

/****************************************************************************
 * Name: tmpfs_unlink
 ****************************************************************************/

static int tmpfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *tdo;
  FAR struct tmpfs_file_s *tfo = NULL;
  FAR const char *name;
  int ret;

  finfo("mountpt: %p relpath: %s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the file object and parent directory associated with this relative
   * path.  If successful, tmpfs_find_file will lock both the file object
   * and the parent directory and take one reference count on each.
   */

  ret = tmpfs_find_file(fs, relpath, &tfo, &tdo);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  DEBUGASSERT(tfo != NULL);

  /* Get the file name from the relative path */

  name = strrchr(relpath, '/');
  if (name != NULL)
    {
      /* Skip over the file '/' character */

      name++;
    }
  else
    {
      /* The name must lie in the root directory */

      name = relpath;
    }

  /* Remove the file from parent directory */

  ret = tmpfs_remove_dirent(tdo, name);
  if (ret < 0)
    {
      goto errout_with_objects;
    }

  /* If the reference count is not one, then just mark the file as
   * unlinked
   */

  if (tfo->tfo_refs > 1)
    {
      /* Make the file object as unlinked */

      tfo->tfo_flags |= TFO_FLAG_UNLINKED;

      /* Release the reference count on the file object */

      tfo->tfo_refs--;
      tmpfs_unlock_file(tfo);
    }

  /* Otherwise we can free the object now */

  else
    {
      nxsem_destroy(&tfo->tfo_exclsem.ts_sem);
      kmm_free(tfo);
    }

  /* Release the reference and lock on the parent directory */

  tdo->tdo_refs--;
  tmpfs_unlock_directory(tdo);
  tmpfs_unlock(fs);

  return OK;

errout_with_objects:
  tmpfs_release_lockedfile(tfo);

  tdo->tdo_refs--;
  tmpfs_unlock_directory(tdo);

errout_with_lock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_mkdir
 ****************************************************************************/

static int tmpfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                       mode_t mode)
{
  FAR struct tmpfs_s *fs;
  int ret;

  finfo("mountpt: %p relpath: %s mode: %04x\n", mountpt, relpath, mode);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Create the directory. */

  ret = tmpfs_create_directory(fs, relpath, NULL);
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_rmdir
 ****************************************************************************/

static int tmpfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *parent;
  FAR struct tmpfs_directory_s *tdo;
  FAR const char *name;
  int ret;

  finfo("mountpt: %p relpath: %s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the directory object and parent directory associated with this
   * relative path.  If successful, tmpfs_find_file will lock both the
   * directory object and the parent directory and take one reference count
   * on each.
   */

  ret = tmpfs_find_directory(fs, relpath, &tdo, &parent);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  /* Is the directory empty?  We cannot remove directories that still
   * contain references to file system objects.  No can we remove the
   * directory if there are outstanding references on it (other than
   * our reference).
   */

  if (tdo->tdo_nentries > 0 || tdo->tdo_refs > 1)
    {
      ret = -EBUSY;
      goto errout_with_objects;
    }

  /* Get the directory name from the relative path */

  name = strrchr(relpath, '/');
  if (name != NULL)
    {
      /* Skip over the fidirectoryle '/' character */

      name++;
    }
  else
    {
      /* The name must lie in the root directory */

      name = relpath;
    }

  /* Remove the directory from parent directory */

  ret = tmpfs_remove_dirent(parent, name);
  if (ret < 0)
    {
      goto errout_with_objects;
    }

  /* Free the directory object */

  nxsem_destroy(&tdo->tdo_exclsem.ts_sem);
  kmm_free(tdo);

  /* Release the reference and lock on the parent directory */

  parent->tdo_refs--;
  tmpfs_unlock_directory(parent);
  tmpfs_unlock(fs);

  return OK;

errout_with_objects:
  tdo->tdo_refs--;
  tmpfs_unlock_directory(tdo);

  parent->tdo_refs--;
  tmpfs_unlock_directory(parent);

errout_with_lock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_rename
 ****************************************************************************/

static int tmpfs_rename(FAR struct inode *mountpt,
                        FAR const char *oldrelpath,
                        FAR const char *newrelpath)
{
  FAR struct tmpfs_directory_s *oldparent;
  FAR struct tmpfs_directory_s *newparent;
  FAR struct tmpfs_object_s *to;
  FAR struct tmpfs_s *fs;
  FAR const char *oldname;
  FAR char *newname;
  FAR char *copy;
  int ret;

  finfo("mountpt: %p oldrelpath: %s newrelpath: %s\n",
        mountpt, oldrelpath, newrelpath);
  DEBUGASSERT(mountpt != NULL && oldrelpath != NULL && newrelpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Duplicate the newpath variable so that we can modify it */

  copy = strdup(newrelpath);
  if (copy == NULL)
    {
      return -ENOMEM;
    }

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      kmm_free(copy);
      return ret;
    }

  /* Separate the new path into the new file name and the path to the new
   * parent directory.
   */

  newname = strrchr(copy, '/');
  if (newname == NULL)
    {
      /* No subdirectories... use the root directory */

      newname   = copy;
      newparent = (FAR struct tmpfs_directory_s *)fs->tfs_root.tde_object;

      tmpfs_lock_directory(newparent);
      newparent->tdo_refs++;
    }
  else
    {
      /* Terminate the parent directory path */

      *newname++ = '\0';

      /* Locate the parent directory that should contain this name.
       * On success, tmpfs_find_directory() will lockthe parent
       * directory and increment the reference count.
       */

      ret = tmpfs_find_directory(fs, copy, &newparent, NULL);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }

  /* Verify that no object of this name already exists in the destination
   * directory.
   */

  ret = tmpfs_find_dirent(newparent, newname);
  if (ret != -ENOENT)
    {
      /* Something with this name already exists in the directory.
       * OR perhaps some fatal error occurred.
       */

      if (ret >= 0)
        {
          ret = -EEXIST;
        }

      goto errout_with_newparent;
    }

  /* Find the old object at oldpath.  If successful, tmpfs_find_object()
   * will lock both the object and the parent directory and will increment
   * the reference count on both.
   */

  ret = tmpfs_find_object(fs, oldrelpath, &to, &oldparent);
  if (ret < 0)
    {
      goto errout_with_newparent;
    }

  /* Get the old file name from the relative path */

  oldname = strrchr(oldrelpath, '/');
  if (oldname != NULL)
    {
      /* Skip over the file '/' character */

      oldname++;
    }
  else
    {
      /* The name must lie in the root directory */

      oldname = oldrelpath;
    }

  /* Remove the entry from the parent directory */

  ret = tmpfs_remove_dirent(oldparent, oldname);
  if (ret < 0)
    {
      goto errout_with_oldparent;
    }

  /* Add an entry to the new parent directory. */

  ret = tmpfs_add_dirent(&newparent, to, newname);

errout_with_oldparent:
  oldparent->tdo_refs--;
  tmpfs_unlock_directory(oldparent);

  tmpfs_release_lockedobject(to);

errout_with_newparent:
  newparent->tdo_refs--;
  tmpfs_unlock_directory(newparent);

errout_with_lock:
  tmpfs_unlock(fs);
  kmm_free(copy);
  return ret;
}

/****************************************************************************
 * Name: tmpfs_stat_common
 ****************************************************************************/

static void tmpfs_stat_common(FAR struct tmpfs_object_s *to,
                              FAR struct stat *buf)
{
  size_t objsize;

  /* Is the tmpfs object a regular file? */

  memset(buf, 0, sizeof(struct stat));

  if (to->to_type == TMPFS_REGULAR)
    {
      FAR struct tmpfs_file_s *tfo =
        (FAR struct tmpfs_file_s *)to;

      /* -rwxrwxrwx */

      buf->st_mode = S_IRWXO | S_IRWXG | S_IRWXU | S_IFREG;

      /* Get the size of the object */

      objsize = tfo->tfo_size;
    }
  else /* if (to->to_type == TMPFS_DIRECTORY) */
    {
      FAR struct tmpfs_directory_s *tdo =
        (FAR struct tmpfs_directory_s *)to;

      /* drwxrwxrwx */

      buf->st_mode = S_IRWXO | S_IRWXG | S_IRWXU | S_IFDIR;

      /* Get the size of the object */

      objsize = SIZEOF_TMPFS_DIRECTORY(tdo->tdo_nentries);
    }

  /* Fake the rest of the information */

  buf->st_size    = objsize;
  buf->st_blksize = CONFIG_FS_TMPFS_BLOCKSIZE;
  buf->st_blocks  = (objsize + CONFIG_FS_TMPFS_BLOCKSIZE - 1) /
                    CONFIG_FS_TMPFS_BLOCKSIZE;
}

/****************************************************************************
 * Name: tmpfs_stat
 ****************************************************************************/

static int tmpfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_object_s *to;
  int ret;

  finfo("mountpt=%p relpath=%s buf=%p\n", mountpt, relpath, buf);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && buf != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL && fs->tfs_root.tde_object != NULL);

  /* Get exclusive access to the file system */

  ret = tmpfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the tmpfs object at the relpath.  If successful,
   * tmpfs_find_object() will lock the object and increment the
   * reference count on the object.
   */

  ret = tmpfs_find_object(fs, relpath, &to, NULL);
  if (ret < 0)
    {
      goto errout_with_fslock;
    }

  /* We found it... Return information about the file object in the stat
   * buffer.
   */

  DEBUGASSERT(to != NULL);
  tmpfs_stat_common(to, buf);

  /* Unlock the object and return success */

  tmpfs_release_lockedobject(to);
  ret = OK;

errout_with_fslock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_DISABLE_MOUNTPOINT */
