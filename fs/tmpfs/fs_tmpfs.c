/****************************************************************************
 * fs/tmpfs/fs_tmpfs.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>

#include <nuttx/fs/fs.h>

#include "fs_tmpfs.h"

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define tmpfs_unlock(fs)            (sem_post(&fs->tfs_exclsem))

#define tmpfs_lock_file(tfo) \
           (tmpfs_lock_object((FAR struct tmpfs_object_s *)tfo))
#define tmpfs_lock_directory(tdo) \
           (tmpfs_lock_object((FAR struct tmpfs_object_s *)tdo))

#define tmpfs_unlock_object(to)     (sem_post(&to->to_exclsem))
#define tmpfs_unlock_file(tfo)      (sem_post(&tfo->tfo_exclsem))
#define tmpfs_unlock_directory(tdo) (sem_post(&tdo->tdo_exclsem))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* TMPFS helpers */

static void tmpfs_lock(FAR struct tmpfs_s *fs);
static void tmpfs_lock_object(FAR struct tmpfs_object_s *to);
static int tmpfs_realloc_directory(FAR struct tmpfs_directory_s **tdo,
            unsigned int nentries);
static int tmpfs_realloc_file(FAR struct tmpfs_file_s **tfo,
            size_t newsize);
static int tmpfs_find_dirent(FAR struct tmpfs_directory_s *tdo,
            FAR const char *name);
static int tmpfs_remove_dirent(FAR struct tmpfs_directory_s *tdo,
            FAR const char *name);
static int tmpfs_add_dirent(FAR struct tmpfs_directory_s **tdo,
            FAR struct tmpfs_object_s *to, FAR const char *name);
static FAR struct tmpfs_file_s *tmpfs_alloc_file(void);
static int tmpfs_create_file(FAR struct tmpfs_s *fs,
            FAR const char *relpath, FAR struct tmpfs_file_s **tfo);
static int tmpfs_find_object(FAR struct tmpfs_s *fs,
            FAR const char *relpath, FAR struct tmpfs_object_s **object,
            FAR struct tmpfs_directory_s **parent);
static int tmpfs_find_file(FAR struct tmpfs_s *fs,
            FAR const char *relpath,
            FAR struct tmpfs_file_s **tfo,
            FAR struct tmpfs_directory_s **parent);
static int tmpfs_find_directory(FAR struct tmpfs_s *fs,
            FAR const char *relpath,
            FAR struct tmpfs_directory_s **tdo,
            FAR struct tmpfs_directory_s **parent);

/* File system operations */

static int tmpfs_open(FAR struct file *filep, FAR const char *relpath,
            int oflags, mode_t mode);
static int tmpfs_close(FAR struct file *filep);
static ssize_t tmpfs_read(FAR struct file *filep, FAR char *buffer,
            size_t buflen);
static ssize_t tmpfs_write(FAR struct file *filep, FAR const char *buffer,
            size_t buflen);
static off_t tmpfs_seek(FAR struct file *filep, off_t offset, int whence);
static int tmpfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int tmpfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int tmpfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
            FAR struct fs_dirent_s *dir);
static int tmpfs_closedir(FAR struct inode *mountpt,
            FAR struct fs_dirent_s *dir);
static int tmpfs_readdir(FAR struct inode *mountpt,
            FAR struct fs_dirent_s *dir);
static int tmpfs_rewinddir(FAR struct inode *mountpt,
            FAR struct fs_dirent_s *dir);
static int tmpfs_bind(FAR struct inode *blkdriver, FAR const void *data,
            FAR void **handle);
static int tmpfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
            unsigned int flags);
static int tmpfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf);
static int tmpfs_unlink(FAR struct inode *mountpt, FAR const char *relpath);
static int tmpfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
            mode_t mode);
static int tmpfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath);
static int tmpfs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
            FAR const char *newrelpath);
static int tmpfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
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
 * name: tmpfs_lock
 ****************************************************************************/

static void tmpfs_lock(FAR struct tmpfs_s *fs)
{
  int ret;

  while ((ret = sem_wait(&fs->tfs_exclsem)) < 0)
    {
      DEBUGASSERT(errno == EINTR)
    }
}

/****************************************************************************
 * name: tmpfs_lock_object
 ****************************************************************************/

static void tmpfs_lock_object(FAR struct tmpfs_object_s *to)
{
  int ret;

  while ((ret = sem_wait(&to->to_exclsem)) < 0)
    {
      DEBUGASSERT(errno == EINTR)
    }
}

/****************************************************************************
 * name: tmpfs_realloc_directory
 ****************************************************************************/

static int tmpfs_realloc_directory(FAR struct tmpfs_directory_s **tdo,
                                   unsigned int nentries)
{
  FAR struct tmpfs_directory_s *oldrdo = *tdo;
  FAR struct tmpfs_directory_s *newrdo;
  size_t objsize;
  int ret = oldrdo->tdo_nentries;

  /* Get the new object size */

  objsize = SIZEOF_TMPFS_DIRECTORY(nentries);
  if (objsize <= oldrdo->tdo_alloc)
    {
      /* Already big enough */

      *oldrdo->tdo_nentries = nentries;
      return OK;
    }

  /* Added some additional amount to the new size to account frequent
   * reallocations.
   */

  objsize += CONFIG_FS_TMPFS_DIRECTORY_ALLOCGUARD;

  /* Realloc the directory object */

  newrdo = (FAR struct tmpfs_directory_s *)kmm_realloc(oldrdo, objsize);
  if (newrdo == NULL)
    {
      return -ENOMEM;
    }

  /* Return the new address of the reallocating directory object */

  newrdo->tdo_alloc     = objsize;
  *newrdo->tdo_nentries = nentries;
  *tdo                  = newrdo;

  /* Return the index to the first, newly alloated directory entry */

  return ret;
}

/****************************************************************************
 * name: tmpfs_realloc_file
 ****************************************************************************/

static int tmpfs_realloc_file(FAR struct tmpfs_file_s **tfo,
                              size_t newsize)
{
  FAR struct tmpfs_file_s *oldtfo = *tfo;
  FAR struct tmpfs_file_s *newtfo;
  size_t objsize;
  size_t allocsize;
  size_t delta;

  /* Check if the current allocation is sufficent */

  objsize = SIZEOF_TMPFS_FILE(newsize);

  /* Added some additional amount to the new size to account frequent
   * reallocations.
   */

  allocsize = objsize + CONFIG_FS_TMPFS_FILE_ALLOCGUARD;

  /* Are we growing or shrinking the object? */

  if (allocsize <= oldtfo->tfo_alloc)
    {
      /* Shrinking ... Shrink unconditionally if the size is shrinking to
       * zero.
       */

      if (newsize > 0)
        {
          /* Otherwise, don't realloc unless the object has shrunk by a
           * lot.
           */

          delta = oldtfo->tfo_alloc - allocsize;
          if (delta <= CONFIG_FS_TMPFS_FILE_ALLOCGUARD)
            {
              /* Hasn't shrunk enough.. Return doing nothing for now */

              oldtfo->tfo_size = newsize;
              return OK;
            }
        }
    }

  /* Realloc the directory object */

  newtfo = (FAR struct tmpfs_file_s *)kmm_realloc(oldtfo, objsize);
  if (newtfo == NULL)
    {
      return -ENOMEM;
    }

  /* Return the new address of the reallocating directory object */

  newtfo->tfo_alloc = allocsize;
  newtfo->tfo_size  = objsize;
  *tfo              = newtfo;
  return OK;
}

/****************************************************************************
 * name: tmpfs_find_dirent
 ****************************************************************************/

static int tmpfs_find_dirent(FAR struct tmpfs_directory_s *tdo,
                             FAR const char *name)
{
  int i;

  /* Search the list of directory entries for a match */

  for (i = 0;
       i < tdo->tdo_nentries &&
       strcmp(tdo->tdo_entry[i].rde_name, name) != 0;
       i++);

  /* Return what we found, if anything */

  return i < tdo->tdo-nentries ? i : -ENOENT;
}

/****************************************************************************
 * name: tmpfs_remove_dirent
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

  if (tdo_entry[index].rde_name != NULL)
    {
      kmm_free(tdo_entry[index].rde_name);
    }

  /* Remove be replace this entry with the final directory entry */

  last = tdo->tdo_nentries - 1;
  if (index != last)
    {
      tdo->tdo_entry[index].rde_object = tdo->tdo_entry[last].rde_object;
      tdo->tdo_entry[index].rde_name = tdo->tdo_entry[last].rde_name;
    }

  /* And decrement the count of directory entries */

  tdo->tdo_entries = last;
  return OK;
}

/****************************************************************************
 * name: tmpfs_add_dirent
 ****************************************************************************/

static int tmpfs_add_dirent(FAR struct tmpfs_directory_s **tdo,
                            FAR struct tmpfs_object_s *to,
                            FAR const char *name)
{
  FAR struct tmpfs_directory_s *oldtdo;
  FAR struct tmpfs_directory_s *newtdo;
  FAR char newname;
  unsigned int nentries;
  int index;

  /* Copy the name string so that it will persist as long as the
   * directory entry.
   */

  newname = strdup(name)
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
      kmm_free(newname)
      return index;
    }

  /* Save the new object info in the new directory entry */

  newtdo = *tdo;
  newtdo->tdo_entry[index].tdo_object = to;
  newtdo->tdo_entry[index].tdo_name   = newname;

  kmm_free(newname);
  return OK;
}

/****************************************************************************
 * name: tmpfs_alloc_file
 ****************************************************************************/

static FAR struct tmpfs_file_s *tmpfs_alloc_file(void)
{
  FAR struct tmpfs_file_s *tfo;
  size_t allocsize;

  /* Create a new zero length file object */

  allocsize = SIZEOF_TMPFS_FILE(FS_TMPFS_FILE_ALLOCGUARD);
  tfo = (FAR struct tmpfs_file_s *)kmm_malloc(allocsize);
  if (nenewtfowfile == NULL)
    {
      return NULL;
    }

  /* Initialize the new file object */

  tfo->tfo_alloc = allocsize;
  tfo->tfo_type  = TMPFS_REGULAR;
  tfo->tfo_refs  = 1;
  tfo->tfo_flags = 0;
  tfo->tfo_size  = 0;
  sem_init(&tfo->tfo_exclsem, 0, 0);

  return tfo;
}

/****************************************************************************
 * name: tmpfs_create_file
 ****************************************************************************/

static int tmpfs_create_file(FAR struct tmpfs_s *fs,
                             FAR const char *relpath,
                             FAR struct tmpfs_file_s **tfo)
{
  FAR struct tmpfs_directory_s *tdo;
  FAR struct tmpfs_file_s *newtfo;
  FAR char *copy;
  FAR char *name;

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
      /* No subdirectories... the root directory */

      name = copy;
      tdo  = fs->tfs_root;
    }
  else
    {
      /* Terminate the parent directly path */

      *name++ = '\0';

      /* Locate the parent directory that should contain this name */

      ret = tmpfs_find_directory(fs, copy, &tdo, NULL);
      if (ret < 0)
        {
          goto errout_with_copy;
        }
    }

  /* Verify that no object of this name already exists in the directory */

  ret = tmpfs_find_dirent(tdo, name);
  if (ret != -ENOENT)
    {
      /* Something with this name already exists n the directory */

      if (ret >= 0)
        {
          ret = -EEXIST;
          goto errout_with_copy;
        }
    }

  /* Allocate an empty file */

  newtfo = tmpfs_alloc_file();
  if (newtfo == NULL)
    {
      ret = -ENOMEM:
      goto errout_with_copy;
    }

  /* Then add the new, empty file to the directory */

  ret = tmpfs_add_dirent(tdo, ()FAR struct tmpfs_object_s *)newtfo, name);
  if (ret < 0)
    {
      goto errout_with_file;
    }

  /* Free the copy of the relpath and return success */

  kmm_free(copy);
  *tfo = newtfo;
  return OK;

/* Error exits */

errout_with_file:
  sem_destroy(&tfo->tfo_exclsem);
  kmm_free(tfo);

errout_with_copy:
  kmm_free(copy);
  return ret;
}

/****************************************************************************
 * name: tmpfs_find_object
 ****************************************************************************/

static int tmpfs_find_object(FAR struct tmpfs_s *fs,
                             FAR const char *relpath,
                             FAR struct tmpfs_object_s **object,
                             FAR struct tmpfs_directory_s **parent)
{
  FAR struct tmpfs_object_s *to;
  FAR struct tmpfs_directory_s *tdo;
  FAR struct tmpfs_directory_s *next_rdo;
  FAR char *segment;
  FAR char *next_segment;
  FAR char *last;
  FAR char *tkptr;
  FAR char *copy;
  int index;

  /* Make a copy of the path (so that we can modify it via strtok) */

  copy = strdup(relpath);
  if (copy == NULL)
    {
      return NULL;
    }

  /* Traverse the file system for any object with the matching name */

  to       = (FAR struct tmpfs_object_s)fs->root;
  next_rdo = fs->tfs_root;

  for (segment =  strtok_r(copy, "/", &tkptr);
       segment != NULL;
       segment = next_segment)
    {
      /* Get the next segment after the one we are currently working on.
       * This will be NULL is we are working on the final segment of the
       * relpath.
       */

      next_segment = strtok_r(NULL, "/", &tkptr);

      /* Search the the next directory. */

      tdo = next_rdo;

      /* Find the TMPFS object with the next segment name in the current
       * directory.
       */

      index = tmpfs_find_dirent(tdo, segment);
      if (index < 0)
        {
          /* No object with this name exists in the directory. */

          return index;
        }

      to = tdo->tdo_entry[index];

      /* Is this object another directory? */

      if (to->to_type != TMPFS_DIRECTORY)
        {
          /* No.  Was this the final segment in the path? */

          if (next_segment == NULL)
            {
              /* Then we can break out of the loop now */

               break;
            }

          /* No, this was not the final segement of the relpath.
           * We cannot continue the search if any of the intermidate
           * segements do no correspond to directories.
           */

          return -ENOTDIR;
        }

      /* Search this directory for the next segement.  If we
       * exit the loop, tdo will still refer to the parent
       * directory of to.
       */

      next_rdo = (FAR struct tmpfs_directory_s *)to;
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
      /* Get exclusive access to the parent and increment the reference
       * count on the object.
       */

      tmpfs_lock_object(parent);
      parent->tdo_refs++;

      *parent = tdo;
    }

  if (object)
    {
      /* Get exclusive access to the object and increment the reference
       * count on the object.
       */

      tmpfs_lock_object(to);
      to->to_refs++;

      *object = to;
    }

  return OK;
}

/****************************************************************************
 * name: tmpfs_find_file
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

          to->to_refs++;
#warning Add check if the file system became unlinked
          tmpfs_unlock_object(to);

          if (parent)
            {
              tdo = *parent;
              tdo->tdo_refs--;
              tmpfs_unlock_object(to);
            }

          ret = -EISDIR;
        }

      /* Return the verified file object */

      *tfo = (FAR struct tmpfs_file_s *)to;
    }

  return ret;
}

/****************************************************************************
 * name: tmpfs_find_directory
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
          /* No... unlock it and return an error */

          to->to_refs++;
#warning Add check if the file system became unlinked
          tmpfs_unlock_object(to);

          if (parent)
            {
              tdo = *parent;
              tdo->tdo_refs--;
              tmpfs_unlock_object(to);
            }

          ret = -ENOTDIR;
        }

      /* Return the verified file object */

      *tdo = (FAR struct tmpfs_file_s *)to;
    }

  return ret;
}

/****************************************************************************
 * name: tmpfs_open
 ****************************************************************************/

static int tmpfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_file_s *tfo;
  int ret;

  fvdbg("filep: %p", filep);
  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  tmpfs_lock(fs);

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

      /* Check if the caller has sufficient privileges to open the file */
      /* REVISIT: No file protection implemented */

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
              ret = tmpfs_realloc_file(tfo, 0);
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

      /* Yes.. create the file object. */

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

  /* Unlock the file object, but retain the reference count */

  tmpfs_unlock_file(tfo);
  tmpfs_unlock(fs);
  return OK;

  /* Error exits */

errout_with_filelock:
  tfo->tfo_refs--;
  tmpfs_unlock_file(tfo);

errout_with_fslock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * name: tmpfs_close
 ****************************************************************************/

static int tmpfs_close(FAR struct file *filep)
{
  FAR struct tmpfs_file_s *tfo;

  fvdbg("filep: %p", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  tmpfs_lock_file(tfo);

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

  if (tfo->tfo_refs == 0 && (tfo->flags & TFO_FLAG_UNLINKED) != 0)
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
 * name: tmpfs_read
 ****************************************************************************/

static ssize_t tmpfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct tmpfs_file_s *tfo;
  ssize_t nread;
  off_t startpos;
  off_t endpos;

  fvdbg("filep: %p buffer: %p buflen: %lu",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  tmpfs_lock_file(tfo);

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

  memcpy(buffer, &tfo->tfo_data, nread)

  /* Release the lock on the file */

  tmpfs_file_unlock

  return nread;
}

/****************************************************************************
 * name: tmpfs_write
 ****************************************************************************/

static ssize_t tmpfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct tmpfs_file_s *tfo;
  ssize_t nwritten;
  off_t startpos;
  off_t endpos;
  int ret;

  fvdbg("filep: %p buffer: %p buflen: %lu",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_priv;

  /* Get exclusive access to the file */

  tmpfs_lock_file(tfo);

  /* Handle attempts to read beyond the end of the file */

  startpos = filep->f_pos;
  nwritten = buflen;
  endpos   = startpos + buflen;

  if (endpos > tfo->tfo_size)
    {
      /* Reallocate the file to handle the write past the end of the file. */

      ret = tmpfs_realloc_file(tfo, (size_t)endpos);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }

  /* Copy data from the memory object to the user buffer */

  memcpy(&tfo->tfo_data, buffer, nwritten)

  /* Release the lock on the file */

  tmpfs_file_unlock(tfo);
  return nwritten;

errout_with_lock:
  tmpfs_file_unlock(tfo);
  return (ssize_t)ret;
}

/****************************************************************************
 * name: tmpfs_seek
 ****************************************************************************/

static off_t tmpfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct tmpfs_file_s *tfo;
  off_t position;

  fvdbg("filep: %p", filep);
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

  /* Attempts to set the position beyound the end of file will
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
 * name: tmpfs_ioctl
 ****************************************************************************/

static int tmpfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct tmpfs_file_s *tfo;
  FAR void **ppv = (FAR void**)arg;

  fvdbg("filep: %p cmd: %d arg: %08lx\n", filep, cmd, arg);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = filep->f_inode->i_private;

  DEBUGASSERT(tfo != NULL);

  /* Only one ioctl command is supported */

  if (cmd == FIOC_MMAP && tfo->rm_xipbase && ppv)
    {
      /* Return the address on the media corresponding to the start of
       * the file.
       */

      *ppv = (FAR void *)tfo->tfo_data;
      return OK;
    }

  fdbg("Invalid cmd: %d \n", cmd);
  return -ENOTTY;
}

/****************************************************************************
 * name: tmpfs_dup
 ****************************************************************************/

static int tmpfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct tmpfs_file_s *tfo;

  fvdbg("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL &&
              oldp->f_inode != NULL != NULL &&
              newp->f_priv == NULL &&
              newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  tfo = oldp->f_inode->i_private;
  DEBUGASSERT(tfo != NULL);

  /* Increment the reference count */

  tmpfs_lock_file(tfo);
  tfo->tfo_refs++;
  tmpfs_lunock_file(tfo);

  /* Save a copy of the file object as the dup'ed file.  This
   * simple implementation does not many any per-open data
   * structures so there is not really much to the dup operation.
   */

  oldp->f_inode->i_private = tfo;
  return OK;
}

/****************************************************************************
 * name: tmpfs_opendir
 ****************************************************************************/

static int tmpfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *tdo;
  int ret;

  fvdbg("mountpt: %p relpath: %s dir: %p",
        mountpt, relpath, dir);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && dir != NULL);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  tmpfs_lock(fs);

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
    }

  /* Release the lock on the file system and return the result */

  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * name: tmpfs_closedir
 ****************************************************************************/

static int tmpfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_directory_s *tdo;

  fvdbg("mountpt: %p dir: %p",  mountpt, dir);
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
 * name: tmpfs_readdir
 ****************************************************************************/

static int tmpfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_directory_s *tdo;
  unsigned int index;
  int ret;

  fvdbg("mountpt: %p dir: %p",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Get the directory structure from the dir argument and lock it */

  tdo = dir->u.tmpfs.tf_tdo;
  DEBUGASSERT(tdo != NULL);

  tmpfs_lock_directory(tdo);

  /* Have we reached the end of the directory? */

  index = dir->u.tmpfs.tf_index;
  if (index > = tdo->tdo_nentries)
    {
      /* We signal the end of the directory by returning the special error:
       * -ENOENT
       */

      fvdbg("End of directory\n");
      ret = -ENOENT;
    }
  else
    {
      /* Does this entry refer to a file or a directory object? */

      if (tdo->tdo_entry[index].to_type == TMPFS_DIRECTORY)
        {
          /* A directory */

           dir->fd_dir.d_type = DTYPE_DIRECTORY;
        }
      else /* tdo->tdo_entry[index].to_type == TMPFS_REGULAR) */
        {
          /* A regular file */

           dir->fd_dir.d_type = DTYPE_FILE;
        }

      /* Copy the entry name */

      strncpy(d_name, tdo->tdo_entry[index].to_name, NAME_MAX + 1);

      /* Increment the index for next time */

      dir->u.tmpfs.tf_index = index + 1;
      ret = OK;
    }

  tmpfs_unlock_directory(tdo);
  return ret;
}

/****************************************************************************
 * name: tmpfs_rewinddir
 ****************************************************************************/

static int tmpfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct tmpfs_directory_s *tdo;
  int ret;

  fvdbg("mountpt: %p dir: %p",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Set the readdir index to zero */

  dir->u.tmpfs.tf_index = 0;
  return OK;
}

/****************************************************************************
 * name: tmpfs_bind
 ****************************************************************************/

static int tmpfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct tmpfs_s *fs;

  DEBUGSSERT(blkdriver == NULL && handle != NULL);

  /* Create an instance of the tmpfs file system */

  fs = (FAR struct tmpfs_s *)kmm_zalloc(sizeof(struct tmpfs_s));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the file system state */

  sem_init(&fs->tfs_exclsem, 0, 1);

  /* Return the new file system handle */

  *handle = (FAR void *)fs;
  return OK;
}

/****************************************************************************
 * name: tmpfs_unbind
 ****************************************************************************/

static int tmpfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags)
{
  FAR struct tmpfs_s *fs = (FAR struct tmpfs_s *fs)handle;

  DEBUGSSERT(blkdriver == NULL && handle != NULL);

  /* Lock the file system */

  tmpfs_lock(fs);

  /* Traverse all directory entries (recursively), freeing all resource */
#warning Missing logic

  /* Now we can destroy the filesystem itself. */

  sem_destroy(&fs->tfs_exclsem);
  kmm_free(fs);
  return OK;
}

/****************************************************************************
 * name: tmpfs_statfs
 ****************************************************************************/

static int tmpfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct inode *inode;
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_file_s *tfo;

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */
#warning Missing logic
  /* Traverse the file system to accurmulate statistics */
#warning Missing logic
  /* Release the lock on the file system */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * name: tmpfs_unlink
 ****************************************************************************/

static int tmpfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *tdo;
  FAR struct tmpfs_file_s *tfo;
  FAR const char *name;
  int ret;

  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = mountpt->i_private;

  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  tmpfs_lock(fs);

  /* Find the file object and parent directory associated with this relative
   * path.  If successful, tmpfs_find_file will lock both the file object
   * and the parent directory and take one reference count on each.
   */

  ret = tmpfs_find_file(fs, relpath, &tfo, &tdo);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

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

  if (tfo->tfo_refx > 1)
    {
      /* Make the file object as unlinked */

      tfo->tfo_flags |= TFO_FLAG_UNLINKED;

      /* Release the reference count on the file object */

      tfo->tfo_refs--;
      tmpfs_unlock_object(tfo);
    }

  /* Otherwise we can free the object now */

  else
    {
      sem_destroy(&tfo->tfo_exclsem);
      kmm_free(tfo);
    }

  /* Release the reference and lock on the parent directory */

  tdo->tdo_refs--;
  tmpfs_unlock_object(tdo);
  tmpfs_unlock(fs);

  return OK;

errout_with_objects:
  tfo->tfo_refs--;
#warning Add check if the file system became unlinked
  tmpfs_unlock_object(tfo);

  tdo->tdo_refs--;
  tmpfs_unlock_object(tdo);

errout_with_lock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * name: tmpfs_mkdir
 ****************************************************************************/

static int tmpfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                       mode_t mode)
{
  FAR struct inode *inode;
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *parent;
  FAR struct tmpfs_directory_s *tdo;

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */
#warning Missing logic
  /* Find the parent directory allocation associated with this relative path */
#warning Missing logic
  /* Does the directory already exist? */
#warning Missing logic
  /* Create the new, empty directory entry */
#warning Missing logic
  /* Add the nre directory entry to the parent directory */
#warning Missing logic
  /* Release the lock on the file system */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * name: tmpfs_rmdir
 ****************************************************************************/

static int tmpfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct inode *inode;
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_directory_s *parent;
  FAR struct tmpfs_directory_s *tdo;

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */
#warning Missing logic
  /* Find the directory allocation associated with this relative path */
#warning Missing logic
  /* Is the directory empty? */
#warning Missing logic
  /* Remove the directory entry from the parent structure */
#warning Missing logic
  /* Release the lock on the file system */
#warning Missing logic
  /* Free the directory object */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * name: tmpfs_rename
 ****************************************************************************/

static int tmpfs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                        FAR const char *newrelpath)
{
  FAR struct inode *inode;
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_file_s *tfo;

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */
#warning Missing logic
  /* Find the file allocation associated with this relative path */
#warning Missing logic
  /* If the file was found, free its name allocation and repace the name
   * with the new name.
   */
#warning Missing logic
  /* Release the lock on the file system */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * name: tmpfs_stat
 ****************************************************************************/

static int tmpfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct tmpfs_s *fs;
  FAR struct tmpfs_object_s *to;
  size_t objsize;
  int ret;

  DEBUGASSERT(mountpt != NULL && relpath != NULL && buf != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  tmpfs_lock(fs);

  /* Find the tmpfs object at the relpath.  If successful,
   * tmpfs_find_object() will lock the object and increment the
   * reference count on the object.
   */

  ret = tmpfs_find_object(fs, relpath, &to, NULL);
  if (ret < 0)
    {
      goto errout_with_fslock;
    }

  /* We found it... Is the object a regular file? */

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
  buf->st_blksize = TMPFS_BLOCKSIZE;
  buf->st_blocks  = (objsize + TMPFS_BLOCKSIZE - 1) / TMPFS_BLOCKSIZE;

  /* No... unlock the object and return and return success */

  ret = OK;

  to->to_refs--;
#warning Add check if the file system became unlinked
  tmpfs_unlock_object(to);

errout_with_fslock:
  tmpfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_DISABLE_MOUNTPOINT */
