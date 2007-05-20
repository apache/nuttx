/****************************************************************************
 * fs_fat32.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References:
 *   Microsoft FAT documentation
 *   FAT implementation 'Copyright (C) 2007, ChaN, all right reserved.'
 *     which has an unrestricted license.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>

#include "fs_internal.h"
#include "fs_fat32.h"

#if CONFIG_FS_FAT

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     fat_open(FAR struct file *filp, const char *rel_path,
                        int oflags, mode_t mode);
static int     fat_close(FAR struct file *filp);
static ssize_t fat_read(FAR struct file *filp, char *buffer, size_t buflen);
static ssize_t fat_write(FAR struct file *filp, const char *buffer,
                         size_t buflen);
static off_t   fat_seek(FAR struct file *filp, off_t offset, int whence);
static int     fat_ioctl(FAR struct file *filp, int cmd, unsigned long arg);
static int     fat_sync(FAR struct file *filp);
static int     fat_bind(FAR struct inode *blkdriver, const void *data,
                        void **handle);
static int     fat_unbind(void *handle);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations fat_operations =
{
  fat_open,
  fat_close,
  fat_read,
  fat_write,
  fat_seek,
  fat_ioctl,
  fat_sync,
  fat_bind,
  fat_unbind
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_open
 ****************************************************************************/

static int fat_open(FAR struct file *filp, const char *rel_path,
                    int oflags, mode_t mode)
{
  struct fat_dirinfo_s  dirinfo;
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv == NULL && filp->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Initialize the directory info structure */

  memset(&dirinfo, 0, sizeof(struct fat_dirinfo_s));
  dirinfo.fs = fs;

  /* Locate the directory entry for this path */

  ret = fat_finddirentry(&dirinfo, rel_path);

  /* Three possibililities: (1) a node exists for the rel_path and
   * dirinfo describes the directory entry of the entity, (2) the
   * node does not exist, or (3) some error occurred.
   */

  if (ret == OK)
    {
      boolean readonly;

      /* The name exists -- but is it a file or a directory? */

      if (dirinfo.fd_entry == NULL ||
         (DIR_GETATTRIBUTES(dirinfo.fd_entry) & FATATTR_DIRECTORY))
        {
          /* It is a directory */
          ret = -EISDIR;
          goto errout_with_semaphore;
        }

      /* It would be an error if we are asked to create it exclusively */

      if ((oflags & (O_CREAT|O_EXCL)) == (O_CREAT|O_EXCL))
        {
          /* Already exists -- can't create it exclusively */
          ret = -EEXIST;
          goto errout_with_semaphore;
        }

#ifdef CONFIG_FILE_MODE
# warning "Missing check for privileges based on inode->i_mode"
#endif

      /* Check if the caller has sufficient privileges to open the file */

      readonly = ((DIR_GETATTRIBUTES(dirinfo.fd_entry) & FATATTR_READONLY) != 0);
      if (((oflags && O_WRONLY) != 0) && readonly)
        {
          ret = -EACCES;
          goto errout_with_semaphore;
        }

      /* If O_TRUNC is specified and the file is opened for writing,
       * then truncate the file.  This operation requires that the file is
       * writable, but we have already checked that. O_TRUNC without write
       * access is ignored.
       */

      if ((oflags & (O_TRUNC|O_WRONLY)) == (O_TRUNC|O_WRONLY))
        {
          /* Truncate the file to zero length */

          ret = fat_dirtruncate(fs, &dirinfo);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }
        }

      /* fall through to finish the file open operations */

    }
  else if (ret == -ENOENT)
    {
      /* The file does not exist.  Were we asked to create it? */

      if ((oflags && O_CREAT) == 0)
        {
          /* No.. then we fail with -ENOENT */
          ret = -ENOENT;
          goto errout_with_semaphore;
        }

      /* Yes.. create the file */

      ret = fat_dircreate(fs, &dirinfo);
      if ( ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Fall through to finish the file open operation */
    }
  else
    {
      /* An error occurred while checking for file existence --
       * such as if an invalid path were provided.
       */

      goto errout_with_semaphore;
    }

  /* Create an instance of the file private date to describe the opened
   * file.
   */

  ff = (struct fat_file_s *)zalloc(sizeof(struct fat_file_s));
  if (!ff)
    {
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }
  
  /* Create a file buffer to support partial sector accesses */

  ff->ff_buffer = (ubyte*)malloc(fs->fs_hwsectorsize);
  if (!ff->ff_buffer)
    {
      ret = -ENOMEM;
      goto errout_with_struct;
    }

  /* Initialize the file private data (only need to initialize non-zero elements) */

  ff->ff_open             = TRUE;
  ff->ff_oflags           = oflags;
  ff->ff_sectorsincluster = 1;

  /* Save information that can be used later to recover the directory entry */

  ff->ff_dirsector        = fs->fs_currentsector;
  ff->ff_dirindex         = dirinfo.fd_index;

  /* File cluster/size info */

  ff->ff_startcluster     =
      ((uint32)DIR_GETFSTCLUSTHI(dirinfo.fd_entry) << 16) |
      DIR_GETFSTCLUSTLO(dirinfo.fd_entry);

  ff->ff_size             = DIR_GETFILESIZE(dirinfo.fd_entry);

  /* In write/append mode, we need to set the file pointer to the end of the file */

  if ((oflags && (O_APPEND|O_WRONLY)) == (O_APPEND|O_WRONLY))
    {
        ff->ff_position   = ff->ff_size;
    }

  /* Attach the private date to the struct file instance */

  filp->f_priv = ff;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  ff->ff_next = fs->fs_head;
  fs->fs_head = ff->ff_next;

  fat_semgive(fs);
  return OK;

  /* Error exits -- goto's are nasty things, but they sure can make error
   * handling a lot simpler.
   */

 errout_with_struct:
  free(ff);

 errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_close
 ****************************************************************************/

static int fat_close(FAR struct file *filp)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  int                   ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv != NULL && filp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filp->f_priv;
  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Do not check if the mount is healthy.  We must support closing of
   * the file even when there is healthy mount.
   */

  /* Synchronize the file buffers and disk content; update times */

  ret = fat_sync(filp);

  /* Then deallocate the memory structures created when the open method
   * was called.
   *
   * Free the sector buffer that was used to manage partial sector accesses.
   */

  if (ff->ff_buffer)
  {
      free(ff->ff_buffer);
  }

  /* Then free the file structure itself. */

  free(ff);
  filp->f_priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: fat_read
 ****************************************************************************/

static ssize_t fat_read(FAR struct file *filp, char *buffer, size_t buflen)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  uint32                cluster;
  unsigned int          bytesread;
  unsigned int          readsize;
  unsigned int          nsectors;
  size_t                readsector;
  size_t                bytesleft;
  ubyte                 *userbuffer = (ubyte*)buffer;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv != NULL && filp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filp->f_priv;
  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if the file was opened with read access */

  if ((ff->ff_oflags & O_RDOK) == 0)
    {
      ret= -EACCES;
      goto errout_with_semaphore;
    }

  /* Get the number of bytes left in the file */

  bytesleft = ff->ff_size - ff->ff_position;

  /* Truncate read count so that it does not exceed the number
   * of bytes left in the file.
   */

  if (buflen > bytesleft)
  {
      buflen = bytesleft;
  }

  /* Loop until either (1) all data has been transferred, or (2) an
   * error occurs.
   */

  readsize = 0;
  while (buflen > 0)
    {
      /* Get offset into the sector where we begin the read */

      int sectorindex = ff->ff_position & SEC_NDXMASK(fs);

      /* Check if the current read stream happens to lie on a
       * sector boundary.
       */

      if (sectorindex == 0)
        {
          /* Try to read another contiguous sector from the cluster */

          ff->ff_sectorsincluster--;

          /* Are there unread sectors remaining in the cluster? */

          if (ff->ff_sectorsincluster > 0)
            {
              /* Yes.. There are more sectors in this cluster to be read
               * just increment the current sector number and read.
               */

              readsector = ff->ff_currentsector + 1;
            }
          else
            {
              /* No.. Handle the case of the first sector of the file */

              if (ff->ff_position == 0)
                {
                  /* Get the first cluster of the file */

                  cluster = ff->ff_startcluster;
                }

              /* But in the general case, we have to find the next cluster
               * in the FAT.
               */

              else
                {
                  cluster = fat_getcluster(fs, ff->ff_currentcluster);
                }

              /* Verify the cluster number */

              if (cluster < 2 || cluster >= fs->fs_nclusters)
                {
                  ret = -EINVAL; /* Not the right error */
                  goto errout_with_semaphore;
                }

              /* Setup to read the first sector from the new cluster */

              ff->ff_currentcluster   = cluster;
              readsector              = fat_cluster2sector(fs, cluster);
              ff->ff_sectorsincluster = fs->fs_fatsecperclus;
            }

          /* Check if the user has provided a buffer large enough to
           * hold one or more complete sectors.
           */

          nsectors = buflen / fs->fs_hwsectorsize;
          if (nsectors > 0)
            {
              /* Read maximum contiguous sectors directly to the user's
               * buffer without using our tiny read buffer.
               *
               * Limit the number of sectors that we read on this time
               * through the loop to the remaining contiguous sectors
               * in this cluster
               */

              if (nsectors > ff->ff_sectorsincluster)
                {
                  nsectors = ff->ff_sectorsincluster;
                }

              /* We are not sure of the state of the file buffer so
               * the safest thing to do is just invalidate it
               */

              (void)fat_ffcacheinvalidate(fs, ff);

              /* Read all of the sectors directory into user memory */

              ret = fat_hwread(fs, userbuffer, readsector, nsectors);
              if (ret < 0)
                {
                  goto errout_with_semaphore;
                }

              ff->ff_sectorsincluster -= nsectors - 1;
              bytesread                = nsectors * fs->fs_hwsectorsize;
            }
          else
            {
              /* We are reading a partial sector.  First, read the whole sector
               * into the file data buffer.  This is a caching buffer so if
               * it is already there then all is well.
               */

              ret = fat_ffcacheread(fs, ff, readsector);
              if (ret < 0)
                {
                  goto errout_with_semaphore;
                }

              /* Copy the partial sector into the user buffer */

              bytesread = fs->fs_hwsectorsize - sectorindex;
              if (bytesread > buflen)
                {
                  bytesread = buflen;
                }

              memcpy(userbuffer, &ff->ff_buffer[sectorindex], bytesread);
            }

          /* Set up for the next sector read */

          userbuffer      += bytesread;
          ff->ff_position += bytesread;
          readsize        += bytesread;
          buflen          -= bytesread;
        }
    }

  fat_semgive(fs);
  return readsize;

 errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_write
 ****************************************************************************/

static ssize_t fat_write(FAR struct file *filp, const char *buffer,
                         size_t buflen)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  sint32                cluster;
  size_t                writesector;
  unsigned int          byteswritten;
  unsigned int          writesize;
  unsigned int          nsectors;
  ubyte                *userbuffer = (ubyte*)buffer;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv != NULL && filp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filp->f_priv;
  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if the file was opened for write access */

  if ((ff->ff_oflags & O_WROK) == 0)
    {
      ret= -EACCES;
      goto errout_with_semaphore;
    }

  /* Check if the file size would exceed the range of size_t */

  if (ff->ff_size + buflen < ff->ff_size)
    {
      ret = -EFBIG;
      goto errout_with_semaphore;
    }

  /* Loop until either (1) all data has been transferred, or (2) an
   * error occurs.
   */

  byteswritten = 0;
  while (buflen > 0)
    {
      /* Get offset into the sector where we begin the read */

      int sectorindex = ff->ff_position & SEC_NDXMASK(fs);

      /* Check if the current read stream happens to lie on a
       * sector boundary.
       */

      if (sectorindex == 0)
        {
          /* Decrement the number of sectors left in this cluster */

          ff->ff_sectorsincluster--;

          /* Are there unwritten sectors remaining in this cluster */

          if (ff->ff_sectorsincluster > 0)
            {
              /* Yes.. There are more sectors in this cluster to be written.
               * just increment the current sector number and write.
               */

              writesector = ff->ff_currentsector + 1;
            }
          else
            {
              /* No.. Handle the case of the first sector of the file */

              if (ff->ff_position == 0)
                {
                  /* Check the first cluster of the file.  Zero means that
                   * the file is empty -- perhaps the file was truncated or
                   * created when it was opened
                   */

                  if (ff->ff_startcluster == 0)
                    {
                      /* In this case, we have to create a new cluster chain */

                      ff->ff_startcluster = fat_createchain(fs);
                    }

                  /* Start writing at the first cluster of the file */

                  cluster = ff->ff_startcluster;
                }

              /* But in the general case, we have to extend the current
               * cluster by one (unless lseek was used to move the file
               * position back from the end of the file)
               */

              else
                {
                    /* Extend the chain by adding a new cluster after
                     * the last one
                     */

                    cluster = fat_extendchain(fs, ff->ff_currentcluster);
                }

              /* Verify the cluster number */

              if (cluster < 0)
                {
                  ret = cluster;
                  goto errout_with_semaphore;
                }
              else if (cluster < 2 || cluster >= fs->fs_nclusters)
                {
                  ret = -ENOSPC;
                  goto errout_with_semaphore;
                }

              /* Setup to write the first sector from the new cluster */

              ff->ff_currentcluster   = cluster;
              writesector             = fat_cluster2sector(fs, cluster);
              ff->ff_sectorsincluster = fs->fs_fatsecperclus;
            }
        }

      /* Check if there is unwritten data in the file buffer */

      ret = fat_ffcacheflush(fs, ff);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Check if the user has provided a buffer large enough to
       * hold one or more complete sectors.
       */

      nsectors = buflen / fs->fs_hwsectorsize;
      if (nsectors > 0)
        {
          /* Write maximum contiguous sectors directly from the user's
           * buffer without using our tiny read buffer.
           *
           * Limit the number of sectors that we write on this time
           * through the loop to the remaining contiguous sectors
           * in this cluster
           */

          if (nsectors > ff->ff_sectorsincluster)
            {
              nsectors = ff->ff_sectorsincluster;
            }

          /* We are not sure of the state of the file buffer so
           * the safest thing to do is just invalidate it
           */

          (void)fat_ffcacheinvalidate(fs, ff);

          /* Write all of the sectors directory from user memory */

          ret = fat_hwwrite(fs, userbuffer, writesector, nsectors);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }

          ff->ff_sectorsincluster -= nsectors - 1;
          writesize                = nsectors * fs->fs_hwsectorsize;
          ff->ff_bflags           |= FFBUFF_MODIFIED;
        }
      else
        {
          /* We are write a partial sector.  We will first have to
           * read the full sector in memory as part of a read-modify-write
           * operation.
           */

          if (ff->ff_position < ff->ff_size)
            {
              ff->ff_currentsector = writesector;
              ret = fat_ffcacheread(fs, ff, writesector);
              if (ret < 0)
                {
                  goto errout_with_semaphore;
                }
            }
          
          /* Copy the partial sector from the user buffer */

          writesize = fs->fs_hwsectorsize - sectorindex;
          if (writesize > buflen)
            {
              writesize = buflen;
            }

          memcpy(&ff->ff_buffer[sectorindex], userbuffer, writesize);
          ff->ff_currentsector = writesector;
          ff->ff_bflags |= (FFBUFF_DIRTY|FFBUFF_VALID|FFBUFF_MODIFIED);
        }

      /* Set up for the next write */

      userbuffer += writesize;
      ff->ff_position += writesize;
      byteswritten += writesize;
      buflen -= writesize;
    }

  /* The transfer has completed without error.  Update the file size */

  if (ff->ff_position > ff->ff_size)
    {
      ff->ff_size = ff->ff_position;
    }

  fat_semgive(fs);
  return byteswritten;

 errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_seek
 ****************************************************************************/

static off_t fat_seek(FAR struct file *filp, off_t offset, int whence)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv != NULL && filp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filp->f_priv;
  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      fat_semgive(fs);
      return ret;
    }

  fat_semgive(fs);
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_ioctl
 ****************************************************************************/

static int fat_ioctl(FAR struct file *filp, int cmd, unsigned long arg)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv != NULL && filp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filp->f_priv;
  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      fat_semgive(fs);
      return ret;
    }

  /* ioctl calls are just passed through to the contained block driver */

  fat_semgive(fs);
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int fat_sync(FAR struct file *filp)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  uint32                wrttime;
  ubyte                *direntry;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filp->f_priv != NULL && filp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filp->f_priv;
  inode = filp->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if the has been modified in any way */
  if ((ff->ff_bflags & FFBUFF_MODIFIED) != 0)
    {
      /* Flush any unwritten data in the file buffer */

      ret = fat_ffcacheflush(fs, ff);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Update the directory entry.  First read the directory
       * entry into the fs_buffer (preserving the ff_buffer)
       */

      ret = fat_fscacheread(fs, ff->ff_dirsector);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Recover a pointer to the specific directory entry
       * in the sector using the saved directory index.
       */

      direntry = &fs->fs_buffer[ff->ff_dirindex];

      /* Set the archive bit, set the write time, and update
       * anything that may have* changed in the directory
       * entry: the file size, and the start cluster
       */

      direntry[DIR_ATTRIBUTES] |= FATATTR_ARCHIVE;

      DIR_PUTFILESIZE(direntry, ff->ff_size);
      DIR_PUTFSTCLUSTLO(direntry, ff->ff_startcluster);
      DIR_PUTFSTCLUSTHI(direntry, ff->ff_startcluster >> 16);

      wrttime = fat_gettime();
      DIR_PUTWRTTIME(direntry, wrttime & 0xffff);
      DIR_PUTWRTDATE(direntry, wrttime >> 16);

      /* Clear the modified bit in the flags */

      ff->ff_bflags &= ~FFBUFF_MODIFIED;

      /* Flush these change to disk and update FSINFO (if
       * appropriate.
       */

      fs->fs_dirty = TRUE;
      ret          = fat_updatefsinfo(fs);
    }

 errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int fat_bind(FAR struct inode *blkdriver, const void *data,
                    void **handle)
{
  struct fat_mountpt_s *fs;
  int ret;

  /* Open the block driver */

  if (!blkdriver || !blkdriver->u.i_bops)
    {
      return -ENODEV;
    }

  if ( blkdriver->u.i_bops->open &&
       blkdriver->u.i_bops->open(blkdriver) != OK)
    {
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  fs = (struct fat_mountpt_s *)zalloc(sizeof(struct fat_mountpt_s));
  if ( !fs )
    {
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure */

  fs->fs_blkdriver = blkdriver;
  sem_init(&fs->fs_sem, 0, 0);

  /* Then get information about the FAT32 filesystem on the devices managed
   * by this block driver.
   */

  ret = fat_mount(fs, TRUE);
  if ( ret != 0 )
    {
      sem_destroy(&fs->fs_sem);
      free(fs);
      return ret;
    }

  *handle = (void*)fs;
  fat_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: fat_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int fat_unbind(void *handle)
{
  struct fat_mountpt_s *fs = (struct fat_mountpt_s*)handle;
  int ret;

  if ( !fs )
    {
      return -EINVAL;
    }

  /* Check if there are sill any files opened on the filesystem. */

  ret = OK; /* Assume success */
  fat_semtake(fs);
  if (fs->fs_head)
    {
      /* We cannot unmount now.. there are open files */

      ret = -EBUSY;
    }
  else
    {
       /* Unmount ... close the block driver */

      if (fs->fs_blkdriver)
        {
          struct inode *inode = fs->fs_blkdriver;
          if (inode && inode->u.i_bops && inode->u.i_bops->close)
            {
              (void)inode->u.i_bops->close(inode);
            }
        }

      /* Release the mountpoint private data */

      if (fs->fs_buffer)
        {
          free(fs->fs_buffer);
        }
      free(fs);
    }

  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_FS_FAT */
