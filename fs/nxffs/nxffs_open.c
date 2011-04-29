/****************************************************************************
 * fs/nxffs/nxffs_open.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
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

#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs.h>
#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* A singly-linked list of open files */

struct nxffs_ofile_s *g_ofiles;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_create
 *
 * Description:
 *   Create a file:  Verify the sufficient space exists at the end of the
 *   FLASH.  If so, then write then update entry in preparation for writing.
 *
 ****************************************************************************/

static inline int nxffs_create(FAR struct nxffs_volume_s *volume,
                               FAR const char *name, mode_t mode,
                               FAR struct nxffs_ofile_s **ppofile)
{
#warning "Check if too close to end of block for whole header"
  return OK;
}

/****************************************************************************
 * Name: nxffs_wropen
 *
 * Description:
 *   Handle opening for writing.  Only a single writer is permitted and only
 *   file creation is supported.
 *
 ****************************************************************************/

static inline int nxffs_wropen(FAR struct nxffs_volume_s *volume,
                               FAR const char *name, mode_t mode,
                               FAR struct nxffs_ofile_s **ppofile)
{
  FAR struct nxffs_wrfile_s *wrfile;
  FAR struct nxffs_entry_s entry;
  int ret;

  /* Limitation: Only a single writer is permitted.  Writing may involve
   * extension of the file system in FLASH.  Since files are contiguous
   * in FLASH, only a single file may be extending the FLASH region.
   */

  if (volume->wrbusy)
    {
      fdbg("There is already a file writer\n");
      return -ENOSYS;
    }

  /* Check if the file exists */

  ret = nxffs_findinode(volume, name, &entry);
  if (ret == OK)
    {
      /* It exists.  It would be an error if we are asked to create it
       * exclusively.
       */

      if ((mode & (O_CREAT|O_EXCL)) == (O_CREAT|O_EXCL))
        {
          fdbg("File exists, can't create O_EXCL\n");
          return -EEXIST;
        }

      /* Were we asked to truncate the file?  NOTE: Don't truncate the
       * file if we were not also asked to created it.  See below...
       * we will not re-create the file unless O_CREAT is also specified.
       */

      else if ((mode & (O_CREAT|O_TRUNC)) == (O_CREAT|O_TRUNC))
        {
          /* Just remove the file and fall through to re-create it */
#warning "Should defer file removal until new file successfully written"

          ret = nxffs_rminode(volume, name);
          if (ret < 0)
            {
              fdbg("nxffs_rminode failed: %d\n", -ret);
              return ret;
            }
        }

      /* The file exists and we were not asked to truncate (and recreate) it.
       * Limitation: Cannot write to existing files.
       */

      else
        {
          fdbg("File %s exists and we were not asked to truncate it\n");
          return -ENOSYS;
        }
    }

  /* Okay, the file is not open and does not exists (maybe because we deleted
   * it).  Now, make sure that we were asked to created it.
   */

  if ((mode & O_CREAT) == 0)
    {
      fdbg("Not asked to create the file\n");
      return -ENOENT;
    }

  /* Yes.. Create a new structure that will describe the state of this open
   * file.  NOTE that a special variant of the open file structure is used
   * that includes additional information to support the write operation.
   */

  wrfile = (FAR struct nxffs_wrfile_s *)kzalloc(sizeof(struct nxffs_wrfile_s));
  if (!wrfile)
    {
      return -ENOMEM;
    }

  /* Initialize the open file state structure */

  wrfile->ofile.crefs  = 1;

  /* Allocate FLASH memory for the file and set up for the write */

#warning "Missing Logic"

  /* Add the open file structure to the head of the list of open files */

  wrfile->ofile.flink = g_ofiles;
  g_ofiles            = &wrfile->ofile;

  /* Indicate that the volume is open for writing and return the open file
   * instance.
   */

  volume->wrbusy = 1;
  *ppofile = &wrfile->ofile;
  return OK;
}

/****************************************************************************
 * Name: nxffs_rdopen
 *
 * Description:
 *   Open an existing file for reading.
 *
 ****************************************************************************/

static inline int nxffs_rdopen(FAR struct nxffs_volume_s *volume,
                               FAR const char *name,
                               FAR struct nxffs_ofile_s **ppofile)
{
  FAR struct nxffs_ofile_s *ofile;
  int ret;

  /* Check if the file has already been opened (for reading) */

  ofile = nxffs_findofile(name);
  if (ofile)
    {
      /* The file is already open.
       * Limitation:  Files cannot be open both for reading and writing.
       */

      if ((ofile->mode & O_WROK) != 0)
        {
          fdbg("File is open for writing\n");
          return -ENOSYS;
        }

      /* Just increment the reference count on the ofile */

      ofile->crefs++;
      fdbg("crefs: %d\n", ofile->crefs);
    }

  /* The file has not yet been opened.
   * Limitation: The file must exist.  We do not support creation of files
   * read-only.
   */

  else
    {
      /* Not already open.. create a new open structure */
 
      ofile = (FAR struct nxffs_ofile_s *)kzalloc(sizeof(struct nxffs_ofile_s));
      if (!ofile)
        {
          fdbg("ofile allocation failed\n");
          return -ENOMEM;
        }

      /* Initialize the open file state structure */

      ofile->crefs  = 1;

      /* Find the file on this volume associated with this file name */

      ret = nxffs_findinode(volume, name, &ofile->entry);
      if (ret != OK)
        {
          fdbg("Inode '%s' not found: %d\n", name, -ret);
          kfree(ofile);
          return ret;
        }

      /* Add the open file structure to the head of the list of open files */

      ofile->flink = g_ofiles;
      g_ofiles     = ofile;
    }

  /* Return the open file state structure */

  *ppofile = ofile;
  return OK;
}

/****************************************************************************
 * Name: nxffs_freeofile
 *
 * Description:
 *   Free resources held by an open file.
 *
 ****************************************************************************/

static inline void nxffs_freeofile(FAR struct nxffs_ofile_s *ofile)
{
  FAR struct nxffs_ofile_s *prev;
  FAR struct nxffs_ofile_s *curr;

  /* Find the open file structure to be removed */

  for (prev = NULL, curr = g_ofiles;
       curr && curr != ofile;
       prev = curr, curr = curr->flink);

  /* Was it found? */

  if (curr)
    {
      /* Yes.. at the head of the list? */

      if (prev)
        {
          prev->flink = ofile->flink;
        }
      else
        {
          g_ofiles = ofile->flink;
        }

      /* Then free the open file */

      nxffs_freeentry(&ofile->entry);
      kfree(ofile);
    }
  else
    {
      fdbg("ERROR: Open inode %p not found\n", ofile);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_findofile
 *
 * Description:
 *   Search the list of already opened files to see if the inode of this
 *   name is one of the opened files.
 *
 * Input Parameters:
 *   name - The name of the inode to check.
 *
 * Returned Value:
 *   If an inode of this name is found in the list of opened inodes, then
 *   a reference to the open file structure is returned.  NULL is returned
 *   otherwise.
 *
 ****************************************************************************/

FAR struct nxffs_ofile_s *nxffs_findofile(FAR const char *name)
{
  FAR struct nxffs_ofile_s *ofile;

  /* Check every open file.  Note that the volume exclsem protects the
   * list of open files.
   */

  for (ofile = g_ofiles; ofile; ofile = ofile->flink)
    {
      /* Check for a name match */

      if (strcmp(name, ofile->entry.name) == 0)
        {
          return ofile;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: nxffs_open
 *
 * Description:
 *   This is the standard mountpoint open method.
 *
 ****************************************************************************/

int nxffs_open(FAR struct file *filep, FAR const char *relpath,
               int oflags, mode_t mode)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_ofile_s *ofile = NULL;
  int ret;

  fvdbg("Open '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint private data from the NuttX inode reference in the
   * file structure
   */

  volume = (FAR struct nxffs_volume_s*)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      ret = -errno;
      fdbg("sem_wait failed: %d\n", ret);
      goto errout;
    }

#ifdef CONFIG_FILE_MODE
#  warning "Missing check for privileges based on inode->i_mode"
#endif

  /* Limitation:  A file must be opened for reading or writing, but not both.
   * There is no general for extending the size of of a file.  Extending the
   * file size of possible if the file to be extended is the last in the
   * sequence on FLASH, but since that case is not the general case, no file
   * extension is supported.
   */

   switch (mode & (O_WROK|O_RDOK))
     {
       case 0:
       default:
         fdbg("One of O_WRONLY/O_RDONLY must be provided\n");
         ret = -EINVAL;
         goto errout_with_semaphore;

       case O_WROK:
         ret = nxffs_wropen(volume, relpath, mode, &ofile);
         break;

       case O_RDOK:
         ret = nxffs_rdopen(volume, relpath, &ofile);
         break;

       case O_WROK|O_RDOK:
         fdbg("O_RDWR is not supported\n");
         ret = -ENOSYS;
         goto errout_with_semaphore;
     }

  /* Save open-specific state in filep->f_priv */

  filep->f_priv = ofile;
  ret = OK;

errout_with_semaphore:
  sem_post(&volume->exclsem);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_close
 *
 * Description:
 *   This is the standard mountpoint close method.
 *
 ****************************************************************************/

int nxffs_close(FAR struct file *filep)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_ofile_s *ofile;
  int ret = -ENOSYS;

  fvdbg("Closing\n");

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */

  ofile = (FAR struct nxffs_ofile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      ret = -errno;
      fdbg("sem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Decrement the reference count on the open file */

  if (ofile->crefs == 1)
    {
      /* Decrementing the reference count would take it zero.  Time to
       * delete the open file state.
       */

      nxffs_freeofile(ofile);
    }
  else
    {
      /* Just decrement the reference count */

      ofile->crefs--;
    }

  filep->f_priv = NULL;
  ret = OK;

  sem_post(&volume->exclsem);
errout:
  return ret;
}
