/****************************************************************************
 * fs/romfs/fs_romfs.c
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "fs_romfs.h"

/****************************************************************************
 * Private Type
 ****************************************************************************/

/* This structure represents one entry node in the romfs file system */

struct romfs_dir_s
{
  struct fs_dirent_s base;                 /* Vfs directory structure */
#ifdef CONFIG_FS_ROMFS_CACHE_NODE
  FAR struct romfs_nodeinfo_s **firstnode; /* The address of first node in the directory */
  FAR struct romfs_nodeinfo_s **currnode;  /* The address of current node into the directory */
#else
  off_t firstoffset;                       /* Offset to the first entry in the directory */
  off_t curroffset;                        /* Current offset into the directory contents */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     romfs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int     romfs_close(FAR struct file *filep);
static ssize_t romfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static off_t   romfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     romfs_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

static int     romfs_dup(FAR const struct file *oldp,
                         FAR struct file *newp);
static int     romfs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf);

static int     romfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir);
static int     romfs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int     romfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry);
static int     romfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);

static int     romfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                          FAR void **handle);
static int     romfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                            unsigned int flags);
static int     romfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);

static int     romfs_stat_common(uint8_t type, uint32_t size,
                                 uint16_t sectorsize, FAR struct stat *buf);
static int     romfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations romfs_operations =
{
  romfs_open,      /* open */
  romfs_close,     /* close */
  romfs_read,      /* read */
  NULL,            /* write */
  romfs_seek,      /* seek */
  romfs_ioctl,     /* ioctl */

  NULL,            /* sync */
  romfs_dup,       /* dup */
  romfs_fstat,     /* fstat */
  NULL,            /* fchstat */
  NULL,            /* truncate */

  romfs_opendir,   /* opendir */
  romfs_closedir,  /* closedir */
  romfs_readdir,   /* readdir */
  romfs_rewinddir, /* rewinddir */

  romfs_bind,      /* bind */
  romfs_unbind,    /* unbind */
  romfs_statfs,    /* statfs */

  NULL,            /* unlink */
  NULL,            /* mkdir */
  NULL,            /* rmdir */
  NULL,            /* rename */
  romfs_stat,      /* stat */
  NULL             /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: romfs_open
 ****************************************************************************/

static int romfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  struct romfs_nodeinfo_s     nodeinfo;
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  int                         ret;

  finfo("Open '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get mountpoint private data from the inode reference from the file
   * structure
   */

  rm = (FAR struct romfs_mountpt_s *)filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Check if the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      ferr("ERROR: romfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* ROMFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      ret = -EACCES;
      goto errout_with_lock;
    }

  /* Locate the directory entry for this path */

  ret = romfs_finddirentry(rm, &nodeinfo, relpath);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find directory directory entry for '%s': %d\n",
           relpath, ret);
      goto errout_with_lock;
    }

  /* The full path exists -- but is the final component a file
   * or a directory?  Or some other Unix file type that is not
   * appropriate in this context.
   *
   * REVISIT: This logic should follow hard/soft link file
   * types.  At present, it returns the ENXIO.
   */

  if (IS_DIRECTORY(nodeinfo.rn_next))
    {
      /* It is a directory */

      ret = -EISDIR;
      ferr("ERROR: '%s' is a directory\n", relpath);
      goto errout_with_lock;
    }
  else if (!IS_FILE(nodeinfo.rn_next))
    {
      /* ENXIO indicates "The named file is a character special or
       * block special file, and the device associated with this
       * special file does not exist."
       *
       * Here we also return ENXIO if the file is not a directory
       * or a regular file.
       */

      ret = -ENXIO;
      ferr("ERROR: '%s' is a special file\n", relpath);
      goto errout_with_lock;
    }

  /* Create an instance of the file private data to describe the opened
   * file.
   */

  rf = kmm_zalloc(sizeof(struct romfs_file_s) + strlen(relpath));
  if (!rf)
    {
      ferr("ERROR: Failed to allocate private data\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Initialize the file private data (only need to initialize
   * non-zero elements)
   */

  rf->rf_size = nodeinfo.rn_size;
  rf->rf_type = (uint8_t)(nodeinfo.rn_next & RFNEXT_ALLMODEMASK);
  strcpy(rf->rf_path, relpath);

  /* Get the start of the file data */

  ret = romfs_datastart(rm, &nodeinfo, &rf->rf_startoffset);
  if (ret < 0)
    {
      ferr("ERROR: Failed to locate start of file data: %d\n", ret);
      kmm_free(rf);
      goto errout_with_lock;
    }

  /* Configure buffering to support access to this file */

  ret = romfs_fileconfigure(rm, rf);
  if (ret < 0)
    {
      ferr("ERROR: Failed configure buffering: %d\n", ret);
      kmm_free(rf);
      goto errout_with_lock;
    }

  /* Attach the private date to the struct file instance */

  filep->f_priv = rf;

  rm->rm_refs++;

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_close
 ****************************************************************************/

static int romfs_close(FAR struct file *filep)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  int                         ret;

  finfo("Closing\n");

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  rm->rm_refs--;
  nxmutex_unlock(&rm->rm_lock);

  /* Do not check if the mount is healthy.  We must support closing of
   * the file even when there is healthy mount.
   */

  /* Deallocate the memory structures created when the open method
   * was called.
   *
   * Free the sector buffer that was used to manage partial sector
   * accesses.
   */

  if (!rm->rm_xipbase && rf->rf_buffer)
    {
      kmm_free(rf->rf_buffer);
    }

  /* Then free the file structure itself. */

  kmm_free(rf);
  filep->f_priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: romfs_read
 ****************************************************************************/

static ssize_t romfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  unsigned int                bytesread;
  unsigned int                readsize = 0;
  unsigned int                nsectors;
  uint32_t                    offset;
  size_t                      bytesleft;
  off_t                       sector;
  FAR uint8_t                *userbuffer = (FAR uint8_t *)buffer;
  int                         sectorndx;
  int                         ret;

  finfo("Read %zu bytes from offset %jd\n", buflen, (intmax_t)filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Make sure that the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      ferr("ERROR: romfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Get the number of bytes left in the file */

  bytesleft = rf->rf_size - filep->f_pos;

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

  while (buflen > 0)
    {
      /* Get the first sector and index to read from. */

      offset    = rf->rf_startoffset + filep->f_pos;
      sector    = SEC_NSECTORS(rm, offset);
      sectorndx = offset & SEC_NDXMASK(rm);

      /* Check if the user has provided a buffer large enough to
       * hold one or more complete sectors -AND- the read is
       * aligned to a sector boundary.
       */

      nsectors = SEC_NSECTORS(rm, buflen);
      if (nsectors >= rf->rf_ncachesector && sectorndx == 0)
        {
          /* Read maximum contiguous sectors directly to the user's
           * buffer without using our tiny read buffer.
           */

          /* Read all of the sectors directly into user memory */

          finfo("Read %d sectors starting with %jd\n", nsectors,
                (intmax_t)sector);
          ret = romfs_hwread(rm, userbuffer, sector, nsectors);
          if (ret < 0)
            {
              ferr("ERROR: romfs_hwread failed: %d\n", ret);
              goto errout_with_lock;
            }

          bytesread = nsectors * rm->rm_hwsectorsize;
        }
      else
        {
          /* We are reading a partial sector.  First, read the whole sector
           * into the file data buffer.  This is a caching buffer so if
           * it is already there then all is well.
           */

          finfo("Read sector %jd\n", (intmax_t)sector);
          ret = romfs_filecacheread(rm, rf, sector);
          if (ret < 0)
            {
              ferr("ERROR: romfs_filecacheread failed: %d\n", ret);
              goto errout_with_lock;
            }

          /* Copy the partial sector into the user buffer */

          bytesread = (rf->rf_cachesector + rf->rf_ncachesector - sector) *
                      rm->rm_hwsectorsize - sectorndx;
          sectorndx = rf->rf_ncachesector * rm->rm_hwsectorsize - bytesread;
          if (bytesread > buflen)
            {
              /* We will not read to the end of the buffer */

              bytesread = buflen;
            }

          finfo("Return %d bytes from sector offset %d\n",
                bytesread, sectorndx);
          memcpy(userbuffer, &rf->rf_buffer[sectorndx], bytesread);
        }

      /* Set up for the next sector read */

      userbuffer   += bytesread;
      filep->f_pos += bytesread;
      readsize     += bytesread;
      buflen       -= bytesread;
    }

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret < 0 ? ret : readsize;
}

/****************************************************************************
 * Name: romfs_seek
 ****************************************************************************/

static off_t romfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  off_t                       position;
  int                         ret;

  finfo("Seek to offset: %jd whence: %d\n", (intmax_t)offset, whence);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

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

        position = offset + rf->rf_size;
        break;

    default:
        ferr("ERROR: Whence is invalid: %d\n", whence);
        return -EINVAL;
    }

  /* Make sure that the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return (off_t)ret;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
       ferr("ERROR: romfs_checkmount failed: %d\n", ret);
       goto errout_with_lock;
    }

  /* Limit positions to the end of the file. */

  if (position > rf->rf_size)
    {
      /* Otherwise, the position is limited to the file size */

      position = rf->rf_size;
    }

  /* Set file position and return success */

  filep->f_pos = position;
  finfo("New file position: %jd\n", (intmax_t)filep->f_pos);

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_ioctl
 ****************************************************************************/

static int romfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  FAR void                  **ppv = (FAR void**)arg;

  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Only one ioctl command is supported */

  if (cmd == FIOC_MMAP && rm->rm_xipbase && ppv)
    {
      /* Return the address on the media corresponding to the start of
       * the file.
       */

      *ppv = (FAR void *)(rm->rm_xipbase + rf->rf_startoffset);
      return OK;
    }
  else if (cmd == FIOC_FILEPATH)
    {
      FAR char *ptr = (FAR char *)((uintptr_t)arg);
      inode_getpath(filep->f_inode, ptr);
      strcat(ptr, rf->rf_path);
      return OK;
    }

  ferr("ERROR: Invalid cmd: %d\n", cmd);
  return -ENOTTY;
}

/****************************************************************************
 * Name: romfs_dup
 ****************************************************************************/

static int romfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s *oldrf;
  FAR struct romfs_file_s *newrf;
  int ret;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Sanity checks */

  DEBUGASSERT(oldp->f_priv != NULL &&
              newp->f_priv == NULL &&
              newp->f_inode != NULL);

  /* Get mountpoint private data from the inode reference from the file
   * structure
   */

  rm = newp->f_inode->i_private;
  DEBUGASSERT(rm != NULL);

  /* Check if the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      ferr("ERROR: romfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Recover the old private data from the old struct file instance */

  oldrf = oldp->f_priv;

  /* Create an new instance of the file private data to describe the new
   * dup'ed file.
   */

  newrf = kmm_malloc(sizeof(struct romfs_file_s));
  if (!newrf)
    {
      ferr("ERROR: Failed to allocate private data\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Copy all file private data (except for the buffer) */

  newrf->rf_startoffset = oldrf->rf_startoffset;
  newrf->rf_size        = oldrf->rf_size;

  /* Configure buffering to support access to this file */

  ret = romfs_fileconfigure(rm, newrf);
  if (ret < 0)
    {
      kmm_free(newrf);
      ferr("ERROR: Failed configure buffering: %d\n", ret);
      goto errout_with_lock;
    }

  /* Attach the new private date to the new struct file instance */

  newp->f_priv = newrf;

  rm->rm_refs++;

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int romfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s *rf;
  int ret;

  finfo("fstat\n");

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get mountpoint private data from the inode reference from the file
   * structure
   */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;
  DEBUGASSERT(rm != NULL);

  /* Check if the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret >= 0)
    {
      /* Return information about the directory entry */

      ret = romfs_stat_common(rf->rf_type, rf->rf_size,
                              rm->rm_hwsectorsize, buf);
    }

  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int romfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s **dir)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_dir_s     *rdir;
  struct romfs_nodeinfo_s     nodeinfo;
  int                         ret;

  finfo("relpath: '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  rm = mountpt->i_private;

  rdir = kmm_zalloc(sizeof(*rdir));
  if (rdir == NULL)
    {
      return -ENOMEM;
    }

  /* Make sure that the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      goto errout_with_rdir;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      ferr("ERROR: romfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Find the requested directory */

  ret = romfs_finddirentry(rm, &nodeinfo, relpath);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find directory '%s': %d\n", relpath, ret);
      goto errout_with_lock;
    }

  /* Verify that it is some kind of directory */

  if (!IS_DIRECTORY(nodeinfo.rn_next))
    {
      /* The entry is not a directory */

      ferr("ERROR: '%s' is not a directory\n", relpath);
      ret = -ENOTDIR;
      goto errout_with_lock;
    }

  /* The entry is a directory */

#ifdef CONFIG_FS_ROMFS_CACHE_NODE
  rdir->firstnode   = nodeinfo.rn_child;
  rdir->currnode    = nodeinfo.rn_child;
#else
  rdir->firstoffset = nodeinfo.rn_offset;
  rdir->curroffset  = nodeinfo.rn_offset;
#endif

  *dir = &rdir->base;
  nxmutex_unlock(&rm->rm_lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);

errout_with_rdir:
  kmm_free(rdir);
  return ret;
}

/****************************************************************************
 * Name: romfs_closedir
 *
 * Description: Close the directory
 *
 ****************************************************************************/

static int romfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return 0;
}

/****************************************************************************
 * Name: romfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int romfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir,
                         FAR struct dirent *entry)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_dir_s     *rdir;
#ifndef CONFIG_FS_ROMFS_CACHE_NODE
  uint32_t                    linkoffset;
  uint32_t                    info;
  uint32_t                    size;
#endif
  uint32_t                    next;
  int                         ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  rm = mountpt->i_private;
  rdir = (FAR struct romfs_dir_s *)dir;

  /* Make sure that the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      ferr("ERROR: omfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Loop, skipping over unsupported items in the file system */

  for (; ; )
    {
      /* Have we reached the end of the directory */

#ifdef CONFIG_FS_ROMFS_CACHE_NODE
      if (!rdir->currnode || !(*rdir->currnode))
#else
      if (!rdir->curroffset)
#endif
        {
          /* We signal the end of the directory by returning the
           * special error -ENOENT
           */

          finfo("End of directory\n");
          ret = -ENOENT;
          goto errout_with_lock;
        }

#ifdef CONFIG_FS_ROMFS_CACHE_NODE
      next = (*rdir->currnode)->rn_next;
      strlcpy(entry->d_name, (*rdir->currnode)->rn_name,
              sizeof(entry->d_name));
      rdir->currnode++;
#else
      /* Parse the directory entry */

      ret = romfs_parsedirentry(rm, rdir->curroffset, &linkoffset,
                                &next, &info, &size);
      if (ret < 0)
        {
          ferr("ERROR: romfs_parsedirentry failed: %d\n", ret);
          goto errout_with_lock;
        }

      /* Save the filename */

      ret = romfs_parsefilename(rm, rdir->curroffset,
                                entry->d_name);
      if (ret < 0)
        {
          ferr("ERROR: romfs_parsefilename failed: %d\n", ret);
          goto errout_with_lock;
        }

      /* Set up the next directory entry offset */

      rdir->curroffset = next & RFNEXT_OFFSETMASK;
#endif

      /* Check the file type */

      if (IS_DIRECTORY(next))
        {
          entry->d_type = DTYPE_DIRECTORY;
          break;
        }
      else if (IS_FILE(next))
        {
          entry->d_type = DTYPE_FILE;
          break;
        }
      else if (IS_SOFTLINK(next))
        {
          entry->d_type = DTYPE_LINK;
          break;
        }
    }

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int romfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_dir_s *rdir;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  rm = mountpt->i_private;
  rdir = (FAR struct romfs_dir_s *)dir;

  /* Make sure that the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret == OK)
    {
#ifdef CONFIG_FS_ROMFS_CACHE_NODE
      rdir->currnode = rdir->firstnode;
#else
      rdir->curroffset = rdir->firstoffset;
#endif
    }

  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int romfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle)
{
  struct romfs_mountpt_s *rm;
  int ret;

  finfo("Entry\n");

  /* Open the block driver */

  if (blkdriver == NULL)
    {
      ferr("ERROR: No block driver/ops\n");
      return -ENODEV;
    }

  if (INODE_IS_BLOCK(blkdriver) &&
      blkdriver->u.i_bops->open != NULL &&
      blkdriver->u.i_bops->open(blkdriver) != OK)
    {
      ferr("ERROR: No open method\n");
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  rm = kmm_zalloc(sizeof(struct romfs_mountpt_s));
  if (!rm)
    {
      ferr("ERROR: Failed to allocate mountpoint structure\n");
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  nxmutex_init(&rm->rm_lock);   /* Initialize the mutex that controls access */
  rm->rm_blkdriver = blkdriver; /* Save the block driver reference */

  /* Get the hardware configuration and setup buffering appropriately */

  ret = romfs_hwconfigure(rm);
  if (ret < 0)
    {
      ferr("ERROR: romfs_hwconfigure failed: %d\n", ret);
      goto errout;
    }

  /* Then complete the mount by getting the ROMFS configuratrion from
   * the ROMF header
   */

  ret = romfs_fsconfigure(rm);
  if (ret < 0)
    {
      ferr("ERROR: romfs_fsconfigure failed: %d\n", ret);
      goto errout_with_buffer;
    }

  /* Mounted! */

  *handle = (FAR void *)rm;
  return OK;

errout_with_buffer:
  if (!rm->rm_xipbase)
    {
      kmm_free(rm->rm_buffer);
    }

errout:
  kmm_free(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int romfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags)
{
  FAR struct romfs_mountpt_s *rm = handle;
  int ret;

  finfo("Entry\n");

#ifdef CONFIG_DEBUG_FEATURES
  if (!rm)
    {
      return -EINVAL;
    }
#endif

  /* Check if there are sill any files opened on the filesystem. */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (rm->rm_refs)
    {
      /* We cannot unmount now.. there are open files */

      fwarn("WARNING: There are open files\n");

      /* This implementation currently only supports unmounting if there are
       * no open file references.
       */

      ret = (flags != 0) ? -ENOSYS : -EBUSY;
    }
  else
    {
      /* Unmount ... close the block driver */

      if (rm->rm_blkdriver)
        {
          FAR struct inode *inode = rm->rm_blkdriver;
          if (inode)
            {
              if (INODE_IS_BLOCK(inode) && inode->u.i_bops->close != NULL)
                {
                  inode->u.i_bops->close(inode);
                }

              /* We hold a reference to the block driver but should
               * not but mucking with inodes in this context.  So, we will
               * just return our contained reference to the block driver
               * inode and let the umount logic dispose of it.
               */

              if (blkdriver)
                {
                  *blkdriver = inode;
                }
            }
        }

      /* Release the mountpoint private data */

      if (!rm->rm_xipbase && rm->rm_buffer)
        {
          kmm_free(rm->rm_buffer);
        }

#ifdef CONFIG_FS_ROMFS_CACHE_NODE
      romfs_freenode(rm->rm_root);
#endif
      nxmutex_destroy(&rm->rm_lock);
      kmm_free(rm);
      return OK;
    }

  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int romfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct romfs_mountpt_s *rm;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  rm = mountpt->i_private;

  /* Check if the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret < 0)
    {
      ferr("ERROR: romfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = ROMFS_MAGIC;

  /* We will claim that the optimal transfer size is the size of one sector */

  buf->f_bsize   = rm->rm_hwsectorsize;

  /* Everything else follows in units of sectors */

  buf->f_blocks  = SEC_NSECTORS(rm, rm->rm_volsize + SEC_NDXMASK(rm));
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Name: romfs_stat_common
 *
 * Description:
 *   Return information about a file or directory
 *
 ****************************************************************************/

static int romfs_stat_common(uint8_t type, uint32_t size,
                             uint16_t sectorsize, FAR struct stat *buf)
{
  memset(buf, 0, sizeof(struct stat));
  if (IS_DIRECTORY(type))
    {
      /* It's a read-execute directory name */

      buf->st_mode = S_IFDIR | S_IROTH | S_IXOTH | S_IRGRP | S_IXGRP |
                     S_IRUSR | S_IXUSR;
    }
  else if (IS_FILE(type) || IS_SOFTLINK(type))
    {
      if (IS_FILE(type))
        {
          buf->st_mode = S_IFREG;
        }
      else
        {
          buf->st_mode = S_IFLNK;
        }

      /* It's a read-only file name */

      buf->st_mode |= S_IROTH | S_IRGRP | S_IRUSR;
      if (IS_EXECUTABLE(type))
        {
          /* It's a read-execute file name */

          buf->st_mode |= S_IXOTH | S_IXGRP | S_IXUSR;
        }
    }
  else
    {
      /* Otherwise, pretend like the unsupported type does not exist */

      finfo("Unsupported type: %d\n", type);
      return -ENOENT;
    }

  /* File/directory size, access block size */

  buf->st_size    = size;
  buf->st_blksize = sectorsize;
  buf->st_blocks  = (buf->st_size + sectorsize - 1) / sectorsize;
  return OK;
}

/****************************************************************************
 * Name: romfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int romfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct romfs_mountpt_s *rm;
  struct romfs_nodeinfo_s nodeinfo;
  uint8_t type;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  rm = mountpt->i_private;

  /* Check if the mount is still healthy */

  ret = nxmutex_lock(&rm->rm_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      ferr("ERROR: romfs_checkmount failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Find the directory entry corresponding to relpath. */

  ret = romfs_finddirentry(rm, &nodeinfo, relpath);

  /* If nothing was found, then we fail with EEXIST */

  if (ret < 0)
    {
      finfo("Failed to find directory: %d\n", ret);
      goto errout_with_lock;
    }

  /* Return information about the directory entry */

  type = (uint8_t)(nodeinfo.rn_next & RFNEXT_ALLMODEMASK);
  ret  = romfs_stat_common(type, nodeinfo.rn_size, rm->rm_hwsectorsize, buf);

errout_with_lock:
  nxmutex_unlock(&rm->rm_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
