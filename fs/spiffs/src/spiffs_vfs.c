/****************************************************************************
 * fs/spiffs/src/spiffs_vfs.c
 * Interface between SPIFFS and the NuttX VFS
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes logic taken from 0.3.7 of SPIFFS by Peter Andersion.  That
 * version was originally released under the MIT license.
 *
 *   Copyright (c) 2013-2017 Peter Andersson (pelleplutt1976@gmail.com)
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

#include <sys/stat.h>
#include <sys/statfs.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "spiffs.h"
#include "spiffs_core.h"
#include "spiffs_cache.h"
#include "spiffs_gc.h"
#include "spiffs_check.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define spiffs_lock_volume(fs)       (nxrmutex_lock(&fs->lock))
#define spiffs_unlock_volume(fs)     (nxrmutex_unlock(&fs->lock))

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct spiffs_dir_s
{
  struct fs_dirent_s base;
  int16_t block;
  int entry;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system operations */

static int  spiffs_open(FAR struct file *filep, FAR const char *relpath,
              int oflags, mode_t mode);
static int  spiffs_close(FAR struct file *filep);
static ssize_t spiffs_read(FAR struct file *filep, FAR char *buffer,
              size_t buflen);
static ssize_t spiffs_write(FAR struct file *filep, FAR const char *buffer,
              size_t buflen);
static off_t spiffs_seek(FAR struct file *filep, off_t offset, int whence);
static int  spiffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int  spiffs_sync(FAR struct file *filep);
static int  spiffs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int  spiffs_fstat(FAR const struct file *filep, FAR struct stat *buf);
static int  spiffs_truncate(FAR struct file *filep, off_t length);

static int  spiffs_opendir(FAR struct inode *mountpt,
              FAR const char *relpath, FAR struct fs_dirent_s **dir);
static int  spiffs_closedir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  spiffs_readdir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir,
              FAR struct dirent *dentry);
static int  spiffs_rewinddir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  spiffs_bind(FAR struct inode *mtdinode, FAR const void *data,
              FAR void **handle);
static int  spiffs_unbind(FAR void *handle, FAR struct inode **mtdinode,
              unsigned int flags);
static int  spiffs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf);
static int  spiffs_unlink(FAR struct inode *mountpt,
              FAR const char *relpath);
static int  spiffs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
              mode_t mode);
static int  spiffs_rmdir(FAR struct inode *mountpt, FAR const char *relpath);
static int  spiffs_rename(FAR struct inode *mountpt,
              FAR const char *oldrelpath, FAR const char *newrelpath);
static int  spiffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
              FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations spiffs_operations =
{
  spiffs_open,       /* open */
  spiffs_close,      /* close */
  spiffs_read,       /* read */
  spiffs_write,      /* write */
  spiffs_seek,       /* seek */
  spiffs_ioctl,      /* ioctl */

  spiffs_sync,       /* sync */
  spiffs_dup,        /* dup */
  spiffs_fstat,      /* fstat */
  NULL,              /* fchstat */
  spiffs_truncate,   /* truncate */

  spiffs_opendir,    /* opendir */
  spiffs_closedir,   /* closedir */
  spiffs_readdir,    /* readdir */
  spiffs_rewinddir,  /* rewinddir */

  spiffs_bind,       /* bind */
  spiffs_unbind,     /* unbind */
  spiffs_statfs,     /* statfs */

  spiffs_unlink,     /* unlink */
  spiffs_mkdir,      /* mkdir */
  spiffs_rmdir,      /* rmdir */
  spiffs_rename,     /* rename */
  spiffs_stat,       /* stat */
  NULL               /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_map_errno
 ****************************************************************************/

static inline int spiffs_map_errno(int errcode)
{
  /* Don't return any or our internal error codes to the application */

  return errcode < SPIFFS_ERR_INTERNAL ? -EFTYPE : errcode;
}

/****************************************************************************
 * Name: spiffs_consistency_check
 ****************************************************************************/

static int spiffs_consistency_check(FAR struct spiffs_s *fs)
{
  int status;
  int ret = OK;

  status = spiffs_check_luconsistency(fs);
  if (status < 0)
    {
      fwarn("WARNING: spiffs_check_luconsistency failed: %d\n", status);
      if (ret >= 0)
        {
          ret = status;
        }
    }

  status = spiffs_check_objidconsistency(fs);
  if (status < 0)
    {
      fwarn("WARNING: spiffs_check_objidconsistency failed: %d\n", status);
      if (ret >= 0)
        {
          ret = status;
        }
    }

  status = spiffs_check_pgconsistency(fs);
  if (status < 0)
    {
      fwarn("WARNING: spiffs_check_pgconsistency failed: %d\n", status);
      if (ret >= 0)
        {
          ret = status;
        }
    }

  status = spiffs_objlu_scan(fs);
  if (status < 0)
    {
      fwarn("WARNING: spiffs_objlu_scan failed: %d\n", status);
      if (ret >= 0)
        {
          ret = status;
        }
    }

  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_readdir_callback
 ****************************************************************************/

static int spiffs_readdir_callback(FAR struct spiffs_s *fs,
                                   int16_t objid, int16_t blkndx, int entry,
                                   FAR const void *user_const,
                                   FAR void *user_var)
{
  struct spiffs_pgobj_ndxheader_s objhdr;
  int16_t pgndx;
  int ret;

  if (objid == SPIFFS_OBJID_FREE || objid == SPIFFS_OBJID_DELETED ||
      (objid & SPIFFS_OBJID_NDXFLAG) == 0)
    {
      return SPIFFS_VIS_COUNTINUE;
    }

  pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                          0, SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                          sizeof(struct spiffs_pgobj_ndxheader_s),
                          (FAR uint8_t *) & objhdr);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read failed: %d\n", ret);
      return spiffs_map_errno(ret);
    }

  if ((objid & SPIFFS_OBJID_NDXFLAG) &&
      objhdr.phdr.spndx == 0 &&
      (objhdr.phdr.flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL |
                                SPIFFS_PH_FLAG_NDXDELE)) ==
      (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_NDXDELE))
    {
      FAR struct dirent *entryp = user_var;

#ifdef CONFIG_SPIFFS_LEADING_SLASH
      /* Skip the leading '/'. */

      if (objhdr.name[0] != '/')
        {
          return -EINVAL; /* The filesystem is corrupted */
        }
#endif

      strlcpy(entryp->d_name,
              (FAR char *)objhdr.name + SPIFFS_LEADING_SLASH_SIZE,
              sizeof(entryp->d_name));
      entryp->d_type = objhdr.type;
      return OK;
    }

  return SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Name: spiffs_open
 ****************************************************************************/

static int spiffs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  off_t offset;
  int16_t pgndx;
  int ret;

  finfo("relpath=%s oflags; %04x\n", relpath, oflags);
  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Skip over any leading directory separators (shouldn't be any) */

  for (; *relpath == '/'; relpath++)
    {
    }

  /* Check the length of the relative path */

  if (strlen(relpath) > CONFIG_SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Allocate a new file object with a reference count of one. */

  fobj = (FAR struct spiffs_file_s *)
    kmm_zalloc(sizeof(struct spiffs_file_s));
  if (fobj == NULL)
    {
      ferr("ERROR: Failed to allocate fail object\n");
      return -ENOMEM;
    }

  fobj->crefs  = 1;
  fobj->oflags = oflags;

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      kmm_free(fobj);
      return ret;
    }

  /* Check of the file object already exists */

  ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)relpath, &pgndx);
  if (ret < 0 && (oflags & O_CREAT) == 0)
    {
      /* It does not exist and we were not asked to create it */

      fwarn("WARNING: File does not exist and O_CREAT not set\n");
      goto errout_with_fileobject;
    }
  else if (ret >= 0 && (oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
    {
      /* O_CREAT and O_EXCL and file exists - fail */

      fwarn("WARNING: File exists and O_CREAT|O_EXCL is selected\n");
      ret = -EEXIST;
      goto errout_with_fileobject;
    }
  else if ((oflags & O_CREAT) != 0 && ret == -ENOENT)
    {
      int16_t objid;

      /* The file does not exist.  We need to create the it. */

      ret = spiffs_objlu_find_free_objid(fs, &objid, 0);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_objlu_find_free_objid() failed: %d\n", ret);
          goto errout_with_fileobject;
        }

      ret = spiffs_fobj_create(fs, objid, (FAR const uint8_t *)relpath,
                               DTYPE_FILE, &pgndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_fobj_create() failed: %d\n", ret);
          goto errout_with_fileobject;
        }

      /* Since we created the file, we don't need to truncate it */

      oflags &= ~O_TRUNC;
    }
  else if (ret < 0)
    {
      ferr("ERROR: spiffs_find_objhdr_pgndx() failed: %d\n", ret);
      goto errout_with_fileobject;
    }

  /* Open the file */

  ret = spiffs_fobj_open_bypage(fs, pgndx, fobj);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_fobj_open_bypage() failed: %d\n", ret);
      goto errout_with_fileobject;
    }

  /* Truncate the file to zero length */

  if ((oflags & O_TRUNC) != 0)
    {
      ret = spiffs_fobj_truncate(fs, fobj, 0, false);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_fobj_truncate() failed: %d\n", ret);
          goto errout_with_fileobject;
        }
    }

  /* Save the struct spiffs_file_s instance as the file private data */

  filep->f_priv = fobj;

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  offset = 0;
  if ((oflags & (O_APPEND | O_WROK)) == (O_APPEND | O_WROK))
    {
      offset = fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size;
    }

  /* Save the file position */

  filep->f_pos = offset;

  /* Add the new file object to the tail of the open file list */

  finfo("Adding fobj for objid=%04x\n", fobj->objid);
  dq_addlast((FAR dq_entry_t *)fobj, &fs->objq);

  spiffs_unlock_volume(fs);
  return OK;

errout_with_fileobject:
  kmm_free(fobj);
  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_close
 ****************************************************************************/

static int spiffs_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("filep=%p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover our private data from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count on the file */

  DEBUGASSERT(fobj->crefs > 0);
  if (fobj->crefs > 0)
    {
      fobj->crefs--;
    }

  filep->f_priv = NULL;

  /* If the reference count decremented to zero then free resources related
   * to the open file.
   */

  if (fobj->crefs == 0)
    {
      ssize_t nflushed;

      /* Flush any cached writes for the file object being closed.
       * This could result in an ENOSPC error being reported if the
       * cache could not flushed to FALSH (and the file will appear to
       * to have been truncated).
       */

      nflushed = spiffs_fobj_flush(fs, fobj);
      if (nflushed < 0)
        {
          ferr("ERROR: spiffs_fobj_flush() failed: %d\n", ret);
          ret = (int)nflushed;
        }

      /* Free the file object while we hold the lock?  Weird but this
       * should be safe because the object is does not have any other
       * references.
       *
       * If the file was unlinked while it was opened, then now would be
       * the time to perform the unlink operation.
       */

      spiffs_fobj_free(fs, fobj, (fobj->flags & SFO_FLAG_UNLINKED) != 0);
    }

  /* Release the lock on the file system */

  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_read
 ****************************************************************************/

static ssize_t spiffs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nread;
  int ret;

  finfo("filep=%p buffer=%p buflen=%lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Read from FLASH */

  nread = spiffs_fobj_read(fs, fobj, buffer, buflen, filep->f_pos);
  if (nread > 0)
    {
      filep->f_pos += nread;
    }

  /* Release the lock on the file system */

  spiffs_unlock_volume(fs);
  return nread;
}

/****************************************************************************
 * Name: spiffs_write
 ****************************************************************************/

static ssize_t spiffs_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nwritten;
  off_t offset;
  int ret;

  finfo("filep=%p buffer=%p buflen=%zu\n",
        filep, buffer, buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Verify that the file was opened with write access */

  if ((fobj->oflags & O_WROK) == 0)
    {
      ret = -EACCES;
      goto errout_with_lock;
    }

  /* Write to FLASH (or cache) */

  offset = filep->f_pos;

  if (fobj->cache_page == 0)
    {
      /* See if object ID is associated with cache already */

      fobj->cache_page = spiffs_cache_page_get_byobjid(fs, fobj);
    }

  if ((fobj->oflags & O_DIRECT) == 0)
    {
      if (buflen < (size_t)SPIFFS_GEO_PAGE_SIZE(fs))
        {
          /* Small write, try to cache it */

          bool alloc_cpage = true;
          if (fobj->cache_page != NULL)
            {
              /* We have a cached page for this object already, check cache
               * page boundaries
               */

              if (offset < fobj->cache_page->offset ||
                  offset > fobj->cache_page->offset +
                           fobj->cache_page->size ||
                  offset + buflen > fobj->cache_page->offset +
                                    SPIFFS_GEO_PAGE_SIZE(fs))
                {
                  /* Boundary violation, write back cache first and allocate
                   * new
                   */

                  spiffs_cacheinfo("Cache page=%d for fobj ID=%d "
                         "Boundary violation, offset=%" PRIu32 " size=%d\n",
                                   fobj->cache_page->cpndx, fobj->objid,
                                   fobj->cache_page->offset,
                                   fobj->cache_page->size);

                  nwritten =
                    spiffs_fobj_write(fs, fobj,
                                      spiffs_get_cache_page(fs,
                                        spiffs_get_cache(fs),
                                        fobj->cache_page->cpndx),
                                      fobj->cache_page->offset,
                                      fobj->cache_page->size);
                  spiffs_cache_page_release(fs, fobj->cache_page);
                  if (nwritten < 0)
                    {
                      ret = (int)nwritten;
                      goto errout_with_lock;
                    }
                }
              else
                {
                  /* Writing within cache */

                  alloc_cpage = false;
                }
            }

          if (alloc_cpage)
            {
              fobj->cache_page =
                spiffs_cache_page_allocate_byobjid(fs, fobj);
              if (fobj->cache_page)
                {
                  fobj->cache_page->offset = offset;
                  fobj->cache_page->size   = 0;

                  spiffs_cacheinfo("Allocated cache page %d for fobj %d\n",
                                   fobj->cache_page->cpndx, fobj->objid);
                }
            }

          if (fobj->cache_page)
            {
              FAR struct spiffs_cache_s *cache;
              FAR uint8_t *cpage_data;
              off_t offset_in_cpage;

              offset_in_cpage = offset - fobj->cache_page->offset;

              spiffs_cacheinfo("Storing to cache page %d for fobj %d "
                               "offset=%jd:%jd buflen=%zu\n",
                               fobj->cache_page->cpndx, fobj->objid,
                               (intmax_t)offset,
                               (intmax_t)offset_in_cpage, buflen);

              cache      = spiffs_get_cache(fs);
              cpage_data = spiffs_get_cache_page(fs, cache,
                                                 fobj->cache_page->cpndx);

              memcpy(&cpage_data[offset_in_cpage], buffer, buflen);
              fobj->cache_page->size = MAX(fobj->cache_page->size,
                                           offset_in_cpage + buflen);

              nwritten = buflen;
              goto success_with_lock;
            }
          else
            {
              nwritten = spiffs_fobj_write(fs, fobj, buffer, offset, buflen);
              if (nwritten < 0)
                {
                  ret = (int)nwritten;
                  goto errout_with_lock;
                }

              goto success_with_lock;
            }
        }
      else
        {
          /* Big write, no need to cache it - but first check if there is a
           * cached write already
           */

          if (fobj->cache_page)
            {
              /* Write back cache first */

              spiffs_cacheinfo("Cache page=%d for fobj ID=%d "
                  "Boundary violation, offset=%" PRIu32 " size=%d\n",
                               fobj->cache_page->cpndx, fobj->objid,
                               fobj->cache_page->offset,
                               fobj->cache_page->size);

              nwritten =
                spiffs_fobj_write(fs, fobj,
                                  spiffs_get_cache_page(fs,
                                    spiffs_get_cache(fs),
                                    fobj->cache_page->cpndx),
                                  fobj->cache_page->offset,
                                  fobj->cache_page->size);
              spiffs_cache_page_release(fs, fobj->cache_page);

              if (nwritten < 0)
                {
                  ret = (int)nwritten;
                  goto errout_with_lock;
                }

              /* Data written below */
            }
        }
    }

  nwritten = spiffs_fobj_write(fs, fobj, buffer, offset, buflen);
  if (nwritten < 0)
    {
      ret = (int)nwritten;
      goto errout_with_lock;
    }

success_with_lock:

  /* Update the file position */

  filep->f_pos += nwritten;

  /* Release our access to the volume */

  spiffs_unlock_volume(fs);
  return nwritten;

errout_with_lock:
  spiffs_unlock_volume(fs);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: spiffs_seek
 ****************************************************************************/

static off_t spiffs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nflushed;
  int16_t data_spndx;
  int16_t objndx_spndx;
  off_t fsize;
  off_t pos;
  int ret;

  finfo("filep=%p offset=%ld whence=%d\n", filep, (long)offset, whence);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      return (off_t)ret;
    }

  /* Get the new file offset */

  nflushed = spiffs_fobj_flush(fs, fobj);
  if (nflushed < 0)
    {
      ferr("ERROR: spiffs_fobj_flush() failed: %d\n", ret);
    }

  fsize = fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size;

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET: /* The offset is set to offset bytes. */
          pos = offset;
          break;

      case SEEK_CUR: /* The offset is set to its current location plus
                      * offset bytes. */
          pos = offset + filep->f_pos;
          break;

      case SEEK_END: /* The offset is set to the size of the file plus
                      * offset bytes. */
          pos = fsize + offset;
          break;

      default:
          ret = -EINVAL;
          goto errout_with_lock;
    }

  /* Verify the resulting file position */

  if (pos < 0)
    {
      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* Attempts to set the position beyond the end of file should
   * work if the file is open for write access.
   *
   * REVISIT: This simple implementation has no per-open storage that
   * would be needed to retain the open flags.
   */

  if (pos > fsize)
    {
      filep->f_pos = fsize;
      ret = -ENOSYS;
      goto errout_with_lock;
    }

  /* Set up for the new file position */

  data_spndx  = (pos > 0 ? (pos - 1) : 0) / SPIFFS_DATA_PAGE_SIZE(fs);
  objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);

  if (fobj->objndx_spndx != objndx_spndx)
    {
      int16_t pgndx;

      ret =
        spiffs_objlu_find_id_and_span(fs, fobj->objid | SPIFFS_OBJID_NDXFLAG,
                                      objndx_spndx, 0, &pgndx);
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      fobj->objndx_spndx = objndx_spndx;
      fobj->objndx_pgndx  = pgndx;
    }

  filep->f_pos = pos;
  spiffs_unlock_volume(fs);
  return pos;

errout_with_lock:
  spiffs_unlock_volume(fs);
  return (off_t)ret;
}

/****************************************************************************
 * Name: spiffs_ioctl
 ****************************************************************************/

static int spiffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  int ret;

  finfo("filep=%p cmd=%d arg=%ld\n", filep, cmd, (long)arg);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle the IOCTL according to the command */

  switch (cmd)
    {
      /* Run a consistency check on the file system media.
       * IN:  None
       * OUT: None
       */

      case FIOC_INTEGRITY:
        {
          ret = spiffs_consistency_check(fs);
        }
        break;

      /* Force reformatting of media.  All data will be lost.
       * IN:  None
       * OUT: None
       */

      case FIOC_REFORMAT:
        {
          /* Check if the MTD driver supports the MTDIOC_BULKERASE command */

          ret = MTD_IOCTL(fs->mtd, MTDIOC_BULKERASE, 0);
          if (ret < 0)
            {
              /* No.. we will have to erase a block at a time */

              int16_t blkndx = 0;
              while (blkndx < SPIFFS_GEO_BLOCK_COUNT(fs))
                {
                  fs->max_erase_count = 0;
                  ret = spiffs_erase_block(fs, blkndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_erase_block() failed: %d\n", ret);
                      break;
                    }

                  blkndx++;
                }
            }
        }
        break;

      /* Run garbage collection.
       * IN:  On entry holds the number of bytes to be recovered.
       * OUT: None
       */

      case FIOC_OPTIMIZE:
        {
          ret = spiffs_gc_check(fs, (size_t)arg);
        }
        break;

      /* Dump logical content of FLASH.
       * IN:  None
       * OUT: None
       */

#ifdef CONFIG_SPIFFS_DUMP
      case FIOC_DUMP:
        {
          ret = spiffs_dump(fs);
        }
        break;
#endif

      default:

        /* Pass through to the contained MTD driver */

        ret = MTD_IOCTL(fs->mtd, cmd, arg);
        break;
    }

  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int spiffs_sync(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nflushed;
  int ret;

  finfo("filep=%p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  ret = spiffs_lock_volume(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Flush all cached write data */

  nflushed = spiffs_fobj_flush(fs, fobj);
  if (nflushed < 0)
    {
      ferr("ERROR: spiffs_fobj_flush() failed: %d\n", ret);
      ret = (int)nflushed;
    }

  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_dup
 ****************************************************************************/

static int spiffs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL &&
              newp->f_priv == NULL && newp->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = oldp->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover our private data from the struct file instance */

  fobj = oldp->f_priv;

  /* Increment the reference count (atomically) */

  ret = spiffs_lock_volume(fs);
  if (ret >= 0)
    {
      spiffs_lock_volume(fs);
      fobj->crefs++;
      spiffs_unlock_volume(fs);

      /* Save a copy of the file object as the dup'ed file. */

      newp->f_priv = fobj;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fobj', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int spiffs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nflushed;
  int ret;

  finfo("filep=%p buf=%p\n", filep, buf);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL &&
              buf != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Flush the cache and perform the common stat() operation */

  nflushed = spiffs_fobj_flush(fs, fobj);
  if (nflushed < 0)
    {
      ferr("ERROR: spiffs_fobj_flush() failed: %d\n", ret);
    }

  ret = spiffs_stat_pgndx(fs, fobj->objhdr_pgndx, fobj->objid, buf);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_stat_pgndx() failed: %d\n", ret);
    }

  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_truncate
 ****************************************************************************/

static int spiffs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  off_t fsize;
  int ret;

  finfo("filep=%p length=%ld\n", filep, (long)length);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL &&
              length >= 0);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* REVISIT:  spiffs_fobj_truncate() can only truncate to smaller sizes. */

  ret = spiffs_fobj_truncate(fs, fobj, length, false);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_fobj_truncate failed: %d/n", ret);
    }

  /* Check if we need to reset the file pointer.  Probably could use
   * 'length', but let's use the authoritative file file size for the
   * comparison.
   */

  fsize = fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size;

  if (ret >= 0 && fsize < filep->f_pos)
    {
      /* Reset the file pointer to the new end-of-file position */

      filep->f_pos = fsize;
    }

  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_opendir
 ****************************************************************************/

static int spiffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s **dir)
{
  FAR struct spiffs_dir_s *sdir;

  finfo("mountpt=%p relpath=%s dir=%p\n",
        mountpt, relpath, dir);

  DEBUGASSERT(mountpt != NULL && relpath != NULL && dir != NULL);

  sdir = kmm_zalloc(sizeof(*sdir));
  if (sdir == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize for traversal of the 'directory' */

  sdir->block = 0;
  sdir->entry = 0;
  *dir        = &sdir->base;
  return OK;
}

/****************************************************************************
 * Name: spiffs_closedir
 ****************************************************************************/

static int spiffs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  finfo("mountpt=%p dir=%p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: spiffs_readdir
 ****************************************************************************/

static int spiffs_readdir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          FAR struct dirent *dentry)
{
  FAR struct spiffs_dir_s *sdir;
  FAR struct spiffs_s *fs;
  int16_t blkndx;
  int entry;
  int ret;

  finfo("mountpt=%p dir=%p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  sdir = (FAR struct spiffs_dir_s *)dir;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Lock the SPIFFS volume */

  spiffs_lock_volume(fs);

  /* And visit the next file object */

  ret = spiffs_foreach_objlu(fs, sdir->block, sdir->entry,
                             SPIFFS_VIS_NO_WRAP, 0, spiffs_readdir_callback,
                             NULL, dentry, &blkndx, &entry);
  if (ret >= 0)
    {
      sdir->block = blkndx;
      sdir->entry = entry + 1;
    }

  /* Release the lock on the file system */

  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_rewinddir
 ****************************************************************************/

static int spiffs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct spiffs_dir_s *sdir;

  finfo("mountpt=%p dir=%p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  sdir = (FAR struct spiffs_dir_s *)dir;

  /* Reset as when opendir() was called. */

  sdir->block = 0;
  sdir->entry = 0;

  return OK;
}

/****************************************************************************
 * Name: spiffs_bind
 ****************************************************************************/

static int spiffs_bind(FAR struct inode *mtdinode, FAR const void *data,
                       FAR void **handle)
{
  FAR struct spiffs_s *fs;
  FAR struct mtd_dev_s *mtd;
  FAR uint8_t *work;
  size_t cache_size;
  size_t cache_max;
  size_t work_size;
  size_t addrmask;
  int ret;

  finfo("mtdinode=%p data=%p handle=%p\n", mtdinode, data, handle);
  DEBUGASSERT(mtdinode != NULL && handle != NULL);

  /* Extract the MTD interface reference */

  DEBUGASSERT(INODE_IS_MTD(mtdinode) && mtdinode->u.i_mtd != NULL);
  mtd = mtdinode->u.i_mtd;

  /* Create an instance of the SPIFFS file system */

  fs = (FAR struct spiffs_s *)kmm_zalloc(sizeof(struct spiffs_s));
  if (fs == NULL)
    {
      ferr("ERROR: Failed to allocate volume structure\n");
      return -ENOMEM;
    }

  fs->mtd = mtd;

  /* Get the MTD geometry */

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                  (unsigned long)((uintptr_t)&fs->geo));
  if (ret < 0)
    {
      ferr("ERROR: MTD_IOCTL(MTDIOC_GEOMETRY) failed: %d\n", ret);
      goto errout_with_volume;
    }

  fs->media_size      = SPIFFS_GEO_EBLOCK_COUNT(fs) *
                        SPIFFS_GEO_EBLOCK_SIZE(fs);
  fs->total_pages     = fs->media_size /
                        SPIFFS_GEO_PAGE_SIZE(fs);
  fs->pages_per_block = SPIFFS_GEO_EBLOCK_SIZE(fs) /
                        SPIFFS_GEO_PAGE_SIZE(fs);

  /* Get the aligned cache size */

  addrmask   = (sizeof(FAR void *) - 1);
  cache_size = (CONFIG_SPIFFS_CACHE_SIZE + addrmask) & ~addrmask;

  /* Don't let the cache size exceed the maximum that is needed */

  cache_max  = SPIFFS_GEO_PAGE_SIZE(fs) << 5;
  if (cache_size > cache_max)
    {
      cache_size = cache_max;
    }

  /* Allocate the cache */

  fs->cache_size = cache_size;
  fs->cache      = (FAR void *)kmm_malloc(cache_size);

  if (fs->cache == NULL)
    {
      ferr("ERROR: Failed to allocate volume structure\n");
      ret = -ENOMEM;
      goto errout_with_volume;
    }

  spiffs_cache_initialize(fs);

  /* Allocate the memory work buffer comprising 3*config->page_size bytes
   * used throughout all file system operations.
   *
   * NOTE: Currently page size is equivalent to block size.
   *
   * REVISIT:  The MTD work buffer was added.  With some careful analysis,
   * it should, however, be possible to get by with fewer page buffers.
   */

  work_size = 3 * SPIFFS_GEO_PAGE_SIZE(fs);
  work      = (FAR uint8_t *)kmm_malloc(work_size);

  if (work == NULL)
    {
      ferr("ERROR: Failed to allocate work buffer\n");
      ret = -ENOMEM;
      goto errout_with_cache;
    }

  fs->work      = &work[0];
  fs->lu_work   = &work[SPIFFS_GEO_PAGE_SIZE(fs)];
  fs->mtd_work  = &work[2 * SPIFFS_GEO_PAGE_SIZE(fs)];

  nxrmutex_init(&fs->lock);

  /* Check the file system */

  ret = spiffs_objlu_scan(fs);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_objlu_scan() failed: %d\n", ret);
      goto errout_with_work;
    }

  finfo("page index byte len:         %u\n",
        (unsigned int)SPIFFS_GEO_PAGE_SIZE(fs));
  finfo("object lookup pages:         %u\n",
        (unsigned int)SPIFFS_OBJ_LOOKUP_PAGES(fs));
  finfo("pages per block:             %u\n",
        (unsigned int)SPIFFS_GEO_PAGES_PER_BLOCK(fs));
  finfo("page header length:          %u\n",
        (unsigned int)sizeof(struct spiffs_page_header_s));
  finfo("object header index entries: %u\n",
        (unsigned int)SPIFFS_OBJHDR_NDXLEN(fs));
  finfo("object index entries:        %u\n",
        (unsigned int)SPIFFS_OBJNDX_LEN(fs));
  finfo("free blocks:                 %u\n",
        (unsigned int)fs->free_blocks);

#ifdef CONFIG_SPIFFS_CHECK_ONMOUNT
  /* Perform the full consistency check */

  ret = spiffs_consistency_check(fs);
  if (ret < 0)
    {
      fwarn("WARNING: File system is damaged: %d\n", ret);
    }
#endif

  /* Return the new file system handle */

  *handle = (FAR void *)fs;
  return OK;

errout_with_work:
  kmm_free(fs->work);

errout_with_cache:
  kmm_free(fs->cache);

errout_with_volume:
  kmm_free(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_unbind
 ****************************************************************************/

static int spiffs_unbind(FAR void *handle, FAR struct inode **mtdinode,
                         unsigned int flags)
{
  FAR struct spiffs_s *fs = (FAR struct spiffs_s *)handle;
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("handle=%p mtdinode=%p flags=%02x\n",
        handle, mtdinode, flags);
  DEBUGASSERT(fs != NULL);

  /* Lock the file system */

  spiffs_lock_volume(fs);

  /* Are there open file system?  If so, are we being forced to unmount? */

  if (!dq_empty(&fs->objq) && (flags & MNT_FORCE) == 0)
    {
      fwarn("WARNING: Open files and umount not forced\n");
      ret = -EBUSY;
      goto errout_with_lock;
    }

  /* Release all of the open file objects... Very scary stuff. */

  while ((fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq)) != NULL)
    {
      /* Free the file object */

      spiffs_fobj_free(fs, fobj, false);
    }

  /* Free allocated working buffers */

  if (fs->work != NULL)
    {
      kmm_free(fs->work);
    }

  if (fs->cache != NULL)
    {
      kmm_free(fs->cache);
    }

  /* Free the volume memory (note that the semaphore is now stale!) */

  nxrmutex_destroy(&fs->lock);
  kmm_free(fs);
  ret = OK;

errout_with_lock:
  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_statfs
 ****************************************************************************/

static int spiffs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  uint32_t pages_per_block;
  uint32_t blocks;
  uint32_t obj_lupages;
  uint32_t data_pgsize;
  uint32_t ndata_pages;
  uint32_t nfile_objs;

  finfo("mountpt=%p buf=%p\n", mountpt, buf);
  DEBUGASSERT(mountpt != NULL && buf != NULL);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Lock the SPIFFS volume */

  spiffs_lock_volume(fs);

  /* Collect some statistics */

  pages_per_block  = SPIFFS_GEO_PAGES_PER_BLOCK(fs);
  blocks           = SPIFFS_GEO_BLOCK_COUNT(fs);
  obj_lupages      = SPIFFS_OBJ_LOOKUP_PAGES(fs);
  data_pgsize      = SPIFFS_DATA_PAGE_SIZE(fs);

  /* -2 for  spare blocks, +1 for emergency page */

  ndata_pages      = (blocks - 2) * (pages_per_block - obj_lupages) + 1;

  /* Count the number of file objects */

  nfile_objs       = 0;
  for (fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq);
       fobj != NULL;
       fobj  = (FAR struct spiffs_file_s *)dq_next((FAR dq_entry_t *)fobj))
    {
      nfile_objs++;
    }

  /* Fill in the statfs structure */

  buf->f_type      = SPIFFS_SUPER_MAGIC;
  buf->f_namelen   = CONFIG_SPIFFS_NAME_MAX - 1;
  buf->f_bsize     = data_pgsize;
  buf->f_blocks    = ndata_pages;
  buf->f_bfree     = ndata_pages - fs->alloc_pages;
  buf->f_bavail    = buf->f_bfree;
  buf->f_files     = nfile_objs;
  buf->f_ffree     = buf->f_bfree;  /* SWAG */

  /* Release the lock on the file system */

  spiffs_unlock_volume(fs);
  return OK;
}

/****************************************************************************
 * Name: spiffs_unlink
 ****************************************************************************/

static int spiffs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int16_t pgndx;
  int ret;

  finfo("mountpt=%p relpath=%s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  if (strlen(relpath) > CONFIG_SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Find the page index to the object header associated with this path */

  ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)relpath, &pgndx);
  if (ret == -ENOENT)
    {
      fwarn("WARNING: No objhdr found for relpath '%s': %d\n", relpath, ret);
      goto errout_with_lock;
    }
  else if (ret < 0)
    {
      ferr("ERROR: spiffs_find_objhdr_pgndx failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Check to see if there is an open file reference for the object at this
   * page index.
   */

  ret = spiffs_find_fobj_bypgndx(fs, pgndx, &fobj);
  if (ret >= 0)
    {
      /* If so, then we cannot unlink the file now. Just mark the file as
       * 'unlinked' so that it can be removed when the file object is
       * released.
       */

      fobj->flags |= SFO_FLAG_UNLINKED;
    }
  else
    {
      /* Otherwise, we will need to re-open the file.
       * First, allocate  new file object.
       */

      fobj = (FAR struct spiffs_file_s *)
        kmm_zalloc(sizeof(struct spiffs_file_s));
      if (fobj == NULL)
        {
          fwarn("WARNING: Failed to allocate fobj\n");
          ret = -ENOMEM;
          goto errout_with_lock;
        }

      /* Use the page index to open the file */

      ret = spiffs_fobj_open_bypage(fs, pgndx, fobj);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_fobj_open_bypage failed: %d\n", ret);
          kmm_free(fobj);
          goto errout_with_lock;
        }

      /* Now we can remove the file by truncating it to zero length */

      ret = spiffs_fobj_truncate(fs, fobj, 0, true);
      kmm_free(fobj);

      if (ret < 0)
        {
          ferr("ERROR: spiffs_fobj_truncate failed: %d\n", ret);
          goto errout_with_lock;
        }
    }

  /* Release the lock on the volume */

  spiffs_unlock_volume(fs);
  return OK;

errout_with_lock:
  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_mkdir
 ****************************************************************************/

static int spiffs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                       mode_t mode)
{
  finfo("mountpt=%p relpath=%s mode=%04x\n", mountpt, relpath, mode);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Directories are not supported */

  return -ENOSYS;
}

/****************************************************************************
 * Name: spiffs_rmdir
 ****************************************************************************/

static int spiffs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  finfo("mountpt=%p relpath=%s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Directories are not supported */

  return -ENOSYS;
}

/****************************************************************************
 * Name: spiffs_rename
 ****************************************************************************/

static int spiffs_rename(FAR struct inode *mountpt,
                         FAR const char *oldrelpath,
                         FAR const char *newrelpath)
{
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int16_t oldpgndx;
  int16_t newpgndx;
  int ret;

  finfo("mountpt=%p oldrelpath=%s newrelpath=%s\n",
        mountpt, oldrelpath, newrelpath);
  DEBUGASSERT(mountpt != NULL && oldrelpath != NULL && newrelpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  if (strlen(newrelpath) > CONFIG_SPIFFS_NAME_MAX - 1 ||
      strlen(oldrelpath) > CONFIG_SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Get the page index of the object header for the oldrelpath */

  ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)oldrelpath,
                                 &oldpgndx);
  if (ret < 0)
    {
      fwarn("WARNING: spiffs_find_objhdr_pgndx failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Check if there is any file object corresponding to the newrelpath */

  ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)newrelpath,
                                 &newpgndx);
  if (ret == -ENOENT)
    {
      ret = OK;
    }
  else if (ret >= 0)
    {
      ret = -EEXIST;
    }

  if (ret < 0)
    {
      goto errout_with_lock;
    }

  /* Allocate new file object.  NOTE:  The file could already be open. */

  fobj = (FAR struct spiffs_file_s *)
    kmm_zalloc(sizeof(struct spiffs_file_s));
  if (fobj == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Use the page index to open the file */

  ret = spiffs_fobj_open_bypage(fs, oldpgndx, fobj);
  if (ret < 0)
    {
      goto errout_with_fobj;
    }

  /* Then update the file name */

  ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                  fobj->objhdr_pgndx, 0,
                                  (FAR const uint8_t *)newrelpath, 0,
                                  &newpgndx);

errout_with_fobj:
  kmm_free(fobj);

errout_with_lock:
  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Name: spiffs_stat
 ****************************************************************************/

static int spiffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct spiffs_s *fs;
  int16_t pgndx;
  int len;
  int ret;

  finfo("mountpt=%p relpath=%s buf=%p\n", mountpt, relpath, buf);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && buf != NULL);

  /* Skip over any leading directory separators (shouldn't be any) */

  for (; *relpath == '/'; relpath++)
    {
    }

  /* Handle long file names */

  len = strlen(relpath);
  if (len > CONFIG_SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Handle stat of the SPIFFS root directory */

  if (len == 0)
    {
      memset(buf, 0, sizeof(struct stat));

      buf->st_mode    = S_IFDIR | S_IRWXO | S_IRWXG | S_IRWXU;
      buf->st_blksize = fs->geo.blocksize;
      buf->st_blocks  = fs->media_size / fs->geo.blocksize;
      ret             = OK;
    }
  else
    {
      /* Find the object associated with this relative path */

      ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)relpath,
                                     &pgndx);
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      /* And get information about the object */

      ret = spiffs_stat_pgndx(fs, pgndx, 0, buf);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_stat_pgndx failed: %d\n", ret);
        }
    }

errout_with_lock:
  spiffs_unlock_volume(fs);
  return spiffs_map_errno(ret);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
