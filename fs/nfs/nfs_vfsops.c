/****************************************************************************
 * fs/nfs/nfs_vfsops.c
 *
 *   Copyright (C) 2012-2013, 2015, 2017-2018 Gregory Nutt. All rights
 *     reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1989, 1993, 1995
 *   The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Rick Macklem at The University of Guelph.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nfs.h>
#include <nuttx/net/netconfig.h>

#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "nfs.h"
#include "rpc.h"
#include "nfs_proto.h"
#include "nfs_node.h"
#include "nfs_mount.h"
#include "xdr_subs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIRENT_NFS_MAXHANDLE    64 /* Maximum length of an NFSv3 file handle */
#define DIRENT_NFS_VERFLEN      8  /* Length of the copy verifier */

/* include/nuttx/fs/dirent.h has its own version of these lengths.  They must
 * match the NFS versions.
 */

#if NFSX_V3FHMAX != DIRENT_NFS_MAXHANDLE
#  error "Length of file handle in fs_dirent_s is incorrect"
#endif

#if NFSX_V3COOKIEVERF != DIRENT_NFS_VERFLEN
#  error "Length of cookie verify in fs_dirent_s is incorrect"
#endif

#define CH_STAT_SIZE            (1 << 7)

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct nfs_dir_s
{
  struct fs_dirent_s nfs_base;                /* VFS diretory structure */
  uint8_t  nfs_fhsize;                        /* Length of the file handle */
  uint8_t  nfs_fhandle[DIRENT_NFS_MAXHANDLE]; /* File handle (max size allocated) */
  uint8_t  nfs_verifier[DIRENT_NFS_VERFLEN];  /* Cookie verifier */
  uint32_t nfs_cookie[2];                     /* Cookie */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t nfs_true;
static uint32_t nfs_false;
static uint32_t nfs_xdrneg1;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     nfs_filecreate(FAR struct nfsmount *nmp,
                   FAR struct nfsnode *np, FAR const char *relpath,
                   mode_t mode);
static int     nfs_filechstat(FAR struct nfsmount *nmp,
                   FAR struct nfsnode *np,
                   FAR const struct stat *buf, int flags);
static int     nfs_fileopen(FAR struct nfsmount *nmp,
                   FAR struct nfsnode *np, FAR const char *relpath,
                   int oflags, mode_t mode);

static int     nfs_open(FAR struct file *filep, FAR const char *relpath,
                   int oflags, mode_t mode);
static int     nfs_close(FAR struct file *filep);
static ssize_t nfs_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t nfs_write(FAR struct file *filep, FAR const char *buffer,
                   size_t buflen);
static int     nfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int     nfs_fsinfo(FAR struct nfsmount *nmp);
static int     nfs_fstat(FAR const struct file *filep, FAR struct stat *buf);
static int     nfs_fchstat(FAR const struct file *filep,
                   FAR const struct stat *buf, int flags);
static int     nfs_truncate(FAR struct file *filep, off_t length);
static int     nfs_opendir(FAR struct inode *mountpt,
                   FAR const char *relpath, FAR struct fs_dirent_s **dir);
static int     nfs_closedir(FAR struct inode *mountpt,
                   FAR struct fs_dirent_s *dir);
static int     nfs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry);
static int     nfs_rewinddir(FAR struct inode *mountpt,
                   FAR struct fs_dirent_s *dir);
static void    nfs_decode_args(FAR struct nfs_mount_parameters *nprmt,
                   FAR struct nfs_args *argp);
static int     nfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                   FAR void **handle);
static int     nfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                   unsigned int flags);
static int     nfs_statfs(FAR struct inode *mountpt,
                   FAR struct statfs *buf);
static int     nfs_remove(FAR struct inode *mountpt,
                   FAR const char *relpath);
static int     nfs_mkdir(FAR struct inode *mountpt,
                   FAR const char *relpath, mode_t mode);
static int     nfs_rmdir(FAR struct inode *mountpt,
                   FAR const char *relpath);
static int     nfs_rename(FAR struct inode *mountpt,
                   FAR const char *oldrelpath, FAR const char *newrelpath);
static mode_t  nfs_stat_mode(unsigned int mode, unsigned int type);
static int     nfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                   FAR struct stat *buf);
static int     nfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                   FAR const struct stat *buf, int flags);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_NFS_STATISTICS
struct nfsstats nfsstats;
#endif

/* nfs vfs operations. */

const struct mountpt_operations nfs_operations =
{
  nfs_open,                     /* open */
  nfs_close,                    /* close */
  nfs_read,                     /* read */
  nfs_write,                    /* write */
  NULL,                         /* seek */
  NULL,                         /* ioctl */
  NULL,                         /* mmap */
  nfs_truncate,                 /* truncate */

  NULL,                         /* sync */
  nfs_dup,                      /* dup */
  nfs_fstat,                    /* fstat */
  nfs_fchstat,                  /* fchstat */

  nfs_opendir,                  /* opendir */
  nfs_closedir,                 /* closedir */
  nfs_readdir,                  /* readdir */
  nfs_rewinddir,                /* rewinddir */

  nfs_bind,                     /* bind */
  nfs_unbind,                   /* unbind */
  nfs_statfs,                   /* statfs */

  nfs_remove,                   /* unlink */
  nfs_mkdir,                    /* mkdir */
  nfs_rmdir,                    /* rmdir */
  nfs_rename,                   /* rename */
  nfs_stat,                     /* stat */
  nfs_chstat                    /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nfs_filecreate
 *
 * Description:
 *   Create a file.  This is part of the file open logic that is executed if
 *   the user asks to create a file.
 *
 * Returned Value:
 *   0 on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int nfs_filecreate(FAR struct nfsmount *nmp, FAR struct nfsnode *np,
                          FAR const char *relpath, mode_t mode)
{
  struct file_handle  fhandle;
  struct nfs_fattr    fattr;
  char                filename[NAME_MAX + 1];
  FAR uint32_t       *ptr;
  uint32_t            tmp;
  int                 namelen;
  int                 reqlen;
  int                 ret;

  /* Find the NFS node of the directory containing the file to be created */

  ret = nfs_finddir(nmp, relpath, &fhandle, &fattr, filename);
  if (ret != OK)
    {
      ferr("ERROR: nfs_finddir returned: %d\n", ret);
      return ret;
    }

  /* Create the CREATE RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.create.create;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  reqlen += uint32_alignup(fhandle.length);
  ptr    += uint32_increment(fhandle.length);

  /* Copy the variable-length file name */

  namelen = strlen(filename);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, filename, namelen);
  ptr    += uint32_increment(namelen);
  reqlen += uint32_alignup(namelen);

  /* Set the creation mode */

  *ptr++  = HTONL(NFSV3CREATE_GUARDED);
  reqlen += sizeof(uint32_t);

  /* Set the mode.  NOTE: Here we depend on the fact that the NuttX and NFS
   * bit settings are the same (at least for the bits of interest).
   */

  *ptr++  = nfs_true; /* True: mode value follows */
  reqlen += sizeof(uint32_t);

  tmp = mode & (NFSMODE_IXOTH | NFSMODE_IWOTH | NFSMODE_IROTH |
                NFSMODE_IXGRP | NFSMODE_IWGRP | NFSMODE_IRGRP |
                NFSMODE_IXUSR | NFSMODE_IWUSR | NFSMODE_IRUSR |
                NFSMODE_SAVETEXT | NFSMODE_ISGID | NFSMODE_ISUID);
  *ptr++  = txdr_unsigned(tmp);
  reqlen += sizeof(uint32_t);

  /* Set the user ID to zero */

  *ptr++  = nfs_true;             /* True: Uid value follows */
  *ptr++  = 0;                    /* UID = 0 (nobody) */
  reqlen += 2*sizeof(uint32_t);

  /* Set the group ID to one */

  *ptr++  = nfs_true;            /* True: Gid value follows */
  *ptr++  = 0;                   /* GID = 0 (nogroup) */
  reqlen += 2*sizeof(uint32_t);

  /* Set the size to zero */

  *ptr++  = nfs_true;            /* True: Size value follows */
  *ptr++  = 0;                   /* Size = 0 */
  *ptr++  = 0;
  reqlen += 3*sizeof(uint32_t);

  /* Don't change times */

  *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change atime */
  *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change mtime */
  reqlen += 2*sizeof(uint32_t);

  /* Send the NFS request. */

  nfs_statistics(NFSPROC_CREATE);
  ret = nfs_request(nmp, NFSPROC_CREATE,
                    &nmp->nm_msgbuffer.create, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);

  /* Check for success */

  if (ret == OK)
    {
      /* Parse the returned data */

      ptr = (FAR uint32_t *)&((FAR struct rpc_reply_create *)
        nmp->nm_iobuffer)->create;

      /* Save the file handle in the file data structure */

      tmp = *ptr++;  /* handle_follows */
      if (!tmp)
        {
          ferr("ERROR: no file handle follows\n");
          return -EINVAL;
        }

      tmp = *ptr++;
      tmp = fxdr_unsigned(uint32_t, tmp);
      DEBUGASSERT(tmp <= NFSX_V3FHMAX);

      np->n_fhsize      = (uint8_t)tmp;
      memcpy(&np->n_fhandle, ptr, tmp);
      ptr += uint32_increment(tmp);

      /* Save the attributes in the file data structure */

      tmp = *ptr;  /* attributes_follows */
      if (!tmp)
        {
          fwarn("WARNING: no file attributes\n");
        }
      else
        {
          /* Initialize the file attributes */

          nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
        }

      /* Any following dir_wcc data is ignored for now */
    }

  return ret;
}

/****************************************************************************
 * Name: nfs_filechstat
 *
 * Description:
 *   Change the status of an open file.  This is part of the file open logic.
 *
 * Returned Value:
 *   0 on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int nfs_filechstat(FAR struct nfsmount *nmp, FAR struct nfsnode *np,
                          FAR const struct stat *buf, int flags)
{
  FAR uint32_t *ptr;
  int           reqlen;
  int           ret;

  finfo("Changing file status\n");

  /* Create the SETATTR RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.setattr.setattr;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(np->n_fhsize);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &np->n_fhandle, np->n_fhsize);
  reqlen += uint32_alignup(np->n_fhsize);
  ptr    += uint32_increment(np->n_fhsize);

  /* Copy the variable-length attributes */

  if (flags & CH_STAT_MODE)
    {
      *ptr++  = nfs_true;
      *ptr++  = txdr_unsigned(buf->st_mode);
      reqlen += 2 * sizeof(uint32_t);
    }
  else
    {
      *ptr++  = nfs_false;
      reqlen += sizeof(uint32_t);
    }

  if (flags & CH_STAT_UID)
    {
      *ptr++  = nfs_true;
      *ptr++  = txdr_unsigned(buf->st_uid);
      reqlen += 2 * sizeof(uint32_t);
    }
  else
    {
      *ptr++  = nfs_false;
      reqlen += sizeof(uint32_t);
    }

  if (flags & CH_STAT_GID)
    {
      *ptr++  = nfs_true;
      *ptr++  = txdr_unsigned(buf->st_gid);
      reqlen += 2 * sizeof(uint32_t);
    }
  else
    {
      *ptr++  = nfs_false;
      reqlen += sizeof(uint32_t);
    }

  if (flags & CH_STAT_SIZE)
    {
      *ptr++  = nfs_true;
      txdr_hyper(buf->st_size, ptr);
      ptr    += 2;
      reqlen += 3 * sizeof(uint32_t);
    }
  else
    {
      *ptr++  = nfs_false;
      reqlen += sizeof(uint32_t);
    }

  if (flags & CH_STAT_ATIME)
    {
      *ptr++  = nfs_true;
      txdr_nfsv3time(&buf->st_atim, ptr);
      ptr    += 2;
      reqlen += 3 * sizeof(uint32_t);
    }
  else
    {
      *ptr++  = nfs_false;
      reqlen += sizeof(uint32_t);
    }

  if (flags & CH_STAT_MTIME)
    {
      *ptr++  = nfs_true;
      txdr_nfsv3time(&buf->st_mtim, ptr);
      ptr    += 2;
      reqlen += 3 * sizeof(uint32_t);
    }
  else
    {
      *ptr++  = nfs_false;
      reqlen += sizeof(uint32_t);
    }

  *ptr++  = nfs_false; /* No guard value */
  reqlen += sizeof(uint32_t);

  /* Perform the SETATTR RPC */

  nfs_statistics(NFSPROC_SETATTR);
  ret = nfs_request(nmp, NFSPROC_SETATTR,
                    &nmp->nm_msgbuffer.setattr, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);
  if (ret != OK)
    {
      ferr("ERROR: nfs_request failed: %d\n", ret);
      return ret;
    }

  /* Get a pointer to the SETATTR reply data */

  ptr = (FAR uint32_t *)&((FAR struct rpc_reply_setattr *)
    nmp->nm_iobuffer)->setattr;

  /* Parse file_wcc.  First, check if WCC attributes follow. */

  if (*ptr++ != 0)
    {
      /* Yes.. WCC attributes follow.  But we just skip over them. */

      ptr += uint32_increment(sizeof(struct wcc_attr));
    }

  /* Check if normal file attributes follow */

  if (*ptr++ != 0)
    {
      /* Yes.. Update the cached file status in the file structure. */

      nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
    }

  return OK;
}

/****************************************************************************
 * Name: nfs_fileopen
 *
 * Description:
 *   Open a file.  This is part of the file open logic that attempts to open
 *   an existing file.
 *
 * Returned Value:
 *   0 on success; a negative errno value on failure.
 *
 ****************************************************************************/

static int nfs_fileopen(FAR struct nfsmount *nmp, FAR struct nfsnode *np,
                        FAR const char *relpath, int oflags, mode_t mode)
{
  struct file_handle fhandle;
  struct nfs_fattr   fattr;
  uint32_t           tmp;
  int                ret = 0;

  /* Find the NFS node associate with the path */

  ret = nfs_findnode(nmp, relpath, &fhandle, &fattr, NULL);
  if (ret != OK)
    {
      ferr("ERROR: nfs_findnode returned: %d\n", ret);
      return ret;
    }

  /* Check if the object is a directory */

  tmp = fxdr_unsigned(uint32_t, fattr.fa_type);
  if (tmp == NFDIR)
    {
      /* Exit with EISDIR if we attempt to open a directory */

      ferr("ERROR: Path is a directory\n");
      return -EISDIR;
    }

  /* Check if the caller has sufficient privileges to open the file */

  if ((oflags & O_WRONLY) != 0)
    {
      /* Check if anyone has privileges to write to the file -- owner,
       * group, or other (we are probably "other" and may still not be
       * able to write).
       */

      tmp = fxdr_unsigned(uint32_t, fattr.fa_mode);
      if ((tmp & (NFSMODE_IWOTH | NFSMODE_IWGRP | NFSMODE_IWUSR)) == 0)
        {
          ferr("ERROR: File is read-only: %08" PRIx32 "\n", tmp);
          return -EACCES;
        }
    }

  /* It would be an ret if we are asked to create the file exclusively */

  if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
    {
      /* Already exists -- can't create it exclusively */

      ferr("ERROR: File exists\n");
      return -EEXIST;
    }

  /* Initialize the file private data.
   *
   * Copy the file handle.
   */

  np->n_fhsize      = (uint8_t)fhandle.length;
  memcpy(&np->n_fhandle, &fhandle.handle, fhandle.length);

  /* Save the file attributes */

  nfs_attrupdate(np, &fattr);

  /* If O_TRUNC is specified and the file is opened for writing,
   * then truncate the file.  This operation requires that the file is
   * writable, but we have already checked that. O_TRUNC without write
   * access is ignored.
   */

  if ((oflags & (O_TRUNC | O_WRONLY)) == (O_TRUNC | O_WRONLY))
    {
      struct stat buf;

      /* Truncate the file to zero length.  I think we can do this with
       * the SETATTR call by setting the length to zero.
       */

      buf.st_size = 0;
      return nfs_filechstat(nmp, np, &buf, CH_STAT_SIZE);
    }

  return OK;
}

/****************************************************************************
 * Name: nfs_open
 *
 * Description:
 *   If oflags == O_CREAT it creates a file, if not it check to see if the
 *   type is ok and that deletion is not in progress.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_open(FAR struct file *filep, FAR const char *relpath,
                    int oflags, mode_t mode)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode *np;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  DEBUGASSERT(nmp != NULL);

  /* Pre-allocate the file private data to describe the opened file. */

  np = (FAR struct nfsnode *)kmm_zalloc(sizeof(struct nfsnode));
  if (!np)
    {
      ferr("ERROR: Failed to allocate private data\n");
      return -ENOMEM;
    }

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      kmm_free(np);
      return ret;
    }

  /* Try to open an existing file at that path */

  ret = nfs_fileopen(nmp, np, relpath, oflags, mode);
  if (ret != OK)
    {
      /* An ret occurred while trying to open the existing file. Check if
       * the open failed because the file does not exist.  That is not
       * necessarily an ret; that may only mean that we have to create the
       * file.
       */

      if (ret != -ENOENT)
        {
          ferr("ERROR: nfs_fileopen failed: %d\n", ret);
          goto errout_with_lock;
        }

      /* The file does not exist. Check if we were asked to create the file.
       * If the O_CREAT bit is set in the oflags then we should create the
       * file if it does not exist.
       */

      if ((oflags & O_CREAT) == 0)
        {
          /* Return ENOENT if the file does not exist and we were not asked
           * to create it.
           */

          ferr("ERROR: File does not exist\n");
          goto errout_with_lock;
        }

      /* Create the file */

      ret = nfs_filecreate(nmp, np, relpath, mode);
      if (ret != OK)
        {
          ferr("ERROR: nfs_filecreate failed: %d\n", ret);
          goto errout_with_lock;
        }
    }

  /* Initialize the file private data (only need to initialize
   * non-zero elements)
   */

  np->n_crefs = 1;

  /* Attach the private data to the struct file instance */

  filep->f_priv = np;

  /* Then insert the new instance at the head of the list in the mountpoint
   * structure. It needs to be there (1) to handle error conditions that
   * effect all files, and (2) to inform the umount logic that we are busy.
   * We cannot unmount the file system if this list is not empty!
   */

  np->n_next   = nmp->nm_head;
  nmp->nm_head = np;

  nxmutex_unlock(&nmp->nm_lock);
  return OK;

errout_with_lock:
  if (np)
    {
      kmm_free(np);
    }

  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_close
 *
 * Description:
 *   Close a file.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_close(FAR struct file *filep)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode  *np;
  FAR struct nfsnode  *prev;
  FAR struct nfsnode  *curr;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  np  = (FAR struct nfsnode *)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  /* Get exclusive access to the mount structure. */

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count.  If the reference count would not
   * decrement to zero, then that is all we have to do.
   */

  if (np->n_crefs > 1)
    {
      np->n_crefs--;
      ret = OK;
    }

  /* There are no more references to the file structure.  Now we need to
   * free up all resources associated with the open file.
   *
   * First, find our file structure in the list of file structures
   * contained in the mount structure.
   */

  else
    {
      /* Assume file structure won't be found. This should never happen. */

      ret = -EINVAL;

      for (prev = NULL, curr = nmp->nm_head;
           curr;
           prev = curr, curr = curr->n_next)
        {
          /* Check if this node is ours */

          if (np == curr)
            {
              /* Yes.. remove it from the list of file structures */

              if (prev)
                {
                  /* Remove from mid-list */

                  prev->n_next = np->n_next;
                }
              else
                {
                  /* Remove from the head of the list */

                  nmp->nm_head = np->n_next;
                }

              /* Then deallocate the file structure and return success */

              kmm_free(np);
              ret = OK;
              break;
            }
        }
    }

  filep->f_priv = NULL;
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_read
 *
 * Returned Value:
 *   The (non-negative) number of bytes read on success; a negated errno
 *   value on failure.
 *
 ****************************************************************************/

static ssize_t nfs_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct nfsmount       *nmp;
  FAR struct nfsnode        *np;
  ssize_t                    readsize;
  ssize_t                    tmp;
  ssize_t                    bytesread;
  size_t                     reqlen;
  FAR uint32_t              *ptr;
  int                        ret = 0;

  finfo("Read %zu bytes from offset %jd\n",
        buflen, (intmax_t)filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  np  = (FAR struct nfsnode *)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Get the number of bytes left in the file and truncate read count so that
   * it does not exceed the number of bytes left in the file.
   */

  tmp = np->n_size - filep->f_pos;
  if (buflen > tmp)
    {
      buflen = tmp;
      finfo("Read size truncated to %zu\n", buflen);
    }

  /* Now loop until we fill the user buffer (or hit the end of the file) */

  for (bytesread = 0; bytesread < buflen; )
    {
      /* Make sure that the attempted read size does not exceed the RPC
       * maximum
       */

      readsize = buflen - bytesread;
      if (readsize > nmp->nm_rsize)
        {
          readsize = nmp->nm_rsize;
        }

      /* Make sure that the attempted read size does not exceed the IO buffer
       * size
       */

      tmp = SIZEOF_rpc_reply_read(readsize);
      if (tmp > nmp->nm_buflen)
        {
          readsize -= (tmp - nmp->nm_buflen);
        }

      /* Initialize the request */

      ptr     = (FAR uint32_t *)&nmp->nm_msgbuffer.read.read;
      reqlen  = 0;

      /* Copy the variable length, file handle */

      *ptr++  = txdr_unsigned((uint32_t)np->n_fhsize);
      reqlen += sizeof(uint32_t);

      memcpy(ptr, &np->n_fhandle, np->n_fhsize);
      reqlen += uint32_alignup(np->n_fhsize);
      ptr    += uint32_increment(np->n_fhsize);

      /* Copy the file offset */

      txdr_hyper((uint64_t)filep->f_pos, ptr);
      ptr += 2;
      reqlen += 2*sizeof(uint32_t);

      /* Set the readsize */

      *ptr = txdr_unsigned(readsize);
      reqlen += sizeof(uint32_t);

      /* Perform the read */

      finfo("Reading %zu bytes\n", readsize);
      nfs_statistics(NFSPROC_READ);
      ret = nfs_request(nmp, NFSPROC_READ,
                        &nmp->nm_msgbuffer.read, reqlen,
                        nmp->nm_iobuffer, nmp->nm_buflen);
      if (ret)
        {
          ferr("ERROR: nfs_request failed: %d\n", ret);
          goto errout_with_lock;
        }

      /* The read was successful.  Get a pointer to the beginning of the NFS
       * response data.
       */

      ptr = (FAR uint32_t *)
        &((FAR struct rpc_reply_read *)nmp->nm_iobuffer)->read;

      /* Check if attributes are included in the responses */

      tmp = *ptr++;
      if (tmp != 0)
        {
          /* Yes.. Update the cached file status in the file structure. */

          nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
          ptr += uint32_increment(sizeof(struct nfs_fattr));
        }

      /* This is followed by the count of data read.  Isn't this
       * the same as the length that is included in the read data?
       *
       * Just skip over if for now.
       */

      ptr++;

      /* Next comes an EOF indication.  Save that in tmp for now. */

      tmp = *ptr++;

      /* Then the length of the read data followed by the read data itself */

      readsize = fxdr_unsigned(uint32_t, *ptr);
      ptr++;

      /* Copy the read data into the user buffer */

      memcpy(buffer, ptr, readsize);

      /* Update the read state data */

      filep->f_pos += readsize;
      bytesread    += readsize;
      buffer       += readsize;

      /* Check if we hit the end of file */

      if (tmp != 0)
        {
          break;
        }
    }

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return bytesread > 0 ? bytesread : ret;
}

/****************************************************************************
 * Name: nfs_write
 *
 * Returned Value:
 *   The (non-negative) number of bytes written on success; a negated errno
 *   value on failure.
 *
 ****************************************************************************/

static ssize_t nfs_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode  *np;
  ssize_t              writesize;
  ssize_t              bufsize;
  ssize_t              byteswritten = 0;
  size_t               reqlen;
  FAR uint32_t        *ptr;
  uint32_t             tmp;
  int                  commit = 0;
  int                  committed = NFSV3WRITE_FILESYNC;
  int                  ret;

  finfo("Write %zu bytes to offset %jd\n",
        buflen, (intmax_t)filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  np  = (FAR struct nfsnode *)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Check if the file size would exceed the range of off_t */

  if (np->n_size + buflen < np->n_size)
    {
      ret = -EFBIG;
      goto errout_with_lock;
    }

  /* Now loop until we send the entire user buffer */

  for (byteswritten = 0; byteswritten < buflen; )
    {
      /* Make sure that the attempted write size does not exceed the RPC
       * maximum.
       */

      writesize = buflen - byteswritten;
      if (writesize > nmp->nm_wsize)
        {
          writesize = nmp->nm_wsize;
        }

      /* Make sure that the attempted read size does not exceed the IO
       * buffer size.
       */

      bufsize = SIZEOF_rpc_call_write(writesize);
      if (bufsize > nmp->nm_buflen)
        {
          writesize -= (bufsize - nmp->nm_buflen);
        }

      /* Initialize the request.  Here we need an offset pointer to the write
       * arguments, skipping over the RPC header.  Write is unique among the
       * RPC calls in that the entry RPC calls message lies in the I/O buffer
       */

      ptr     = (FAR uint32_t *)&((FAR struct rpc_call_write *)
                  nmp->nm_iobuffer)->write;
      reqlen  = 0;

      /* Copy the variable length, file handle */

      *ptr++  = txdr_unsigned((uint32_t)np->n_fhsize);
      reqlen += sizeof(uint32_t);

      memcpy(ptr, &np->n_fhandle, np->n_fhsize);
      reqlen += uint32_alignup(np->n_fhsize);
      ptr    += uint32_increment(np->n_fhsize);

      /* Copy the file offset */

      txdr_hyper((uint64_t)filep->f_pos, ptr);
      ptr    += 2;
      reqlen += 2*sizeof(uint32_t);

      /* Copy the count and stable values */

      *ptr++  = txdr_unsigned(writesize);
      *ptr++  = txdr_unsigned(committed);
      reqlen += 2*sizeof(uint32_t);

      /* Copy a chunk of the user data into the I/O buffer */

      *ptr++  = txdr_unsigned(writesize);
      reqlen += sizeof(uint32_t);
      memcpy(ptr, buffer, writesize);
      reqlen += uint32_alignup(writesize);

      /* Perform the write */

      nfs_statistics(NFSPROC_WRITE);
      ret = nfs_request(nmp, NFSPROC_WRITE,
                        nmp->nm_iobuffer, reqlen,
                        &nmp->nm_msgbuffer.write,
                        sizeof(struct rpc_reply_write));
      if (ret)
        {
          ferr("ERROR: nfs_request failed: %d\n", ret);
          goto errout_with_lock;
        }

      /* Get a pointer to the WRITE reply data */

      ptr = (FAR uint32_t *)&nmp->nm_msgbuffer.write.write;

      /* Parse file_wcc.  First, check if WCC attributes follow. */

      tmp = *ptr++;
      if (tmp != 0)
        {
          /* Yes.. WCC attributes follow.  But we just skip over them. */

          ptr += uint32_increment(sizeof(struct wcc_attr));
        }

      /* Check if normal file attributes follow */

      tmp = *ptr++;
      if (tmp != 0)
        {
          /* Yes.. Update the cached file status in the file structure. */

          nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
          ptr += uint32_increment(sizeof(struct nfs_fattr));
        }

      /* Get the count of bytes actually written */

      tmp = fxdr_unsigned(uint32_t, *ptr);
      ptr++;

      if (tmp < 1 || tmp > writesize)
        {
          ret = -EIO;
          goto errout_with_lock;
        }

      writesize = tmp;

      /* Determine the lowest commitment level obtained by any of the RPCs. */

      commit = *ptr++;
      if (committed == NFSV3WRITE_FILESYNC)
        {
          committed = commit;
        }
      else if (committed == NFSV3WRITE_DATASYNC &&
               commit == NFSV3WRITE_UNSTABLE)
        {
          committed = commit;
        }

      /* Update the read state data */

      filep->f_pos += writesize;
      byteswritten += writesize;
      buffer       += writesize;
    }

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return byteswritten > 0 ? byteswritten : ret;
}

/****************************************************************************
 * Name: nfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int nfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode *np;
  int ret;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Sanity checks */

  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (FAR struct nfsmount *)oldp->f_inode->i_private;
  np  = (FAR struct nfsnode *)oldp->f_priv;

  DEBUGASSERT(nmp != NULL);

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count on the NFS node structure */

  DEBUGASSERT(np->n_crefs < 0xff);
  np->n_crefs++;

  /* And save this as the file data for the new node */

  newp->f_priv = np;

  nxmutex_unlock(&nmp->nm_lock);
  return OK;
}

/****************************************************************************
 * Name: nfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   structure 'filep', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int nfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode *np;
  int ret;

  finfo("Buf %p\n", buf);
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  np  = (FAR struct nfsnode *)filep->f_priv;

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  DEBUGASSERT(nmp != NULL);

  memset(buf, 0, sizeof(*buf));

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Extract the file mode, file type, and file size from the nfsnode
   * structure.
   */

  buf->st_mode  = nfs_stat_mode(np->n_mode, np->n_type);
  buf->st_size  = (off_t)np->n_size;

  /* Extract time values as type time_t in units of seconds. */

  buf->st_atim = np->n_atime;
  buf->st_mtim = np->n_mtime;
  buf->st_ctim = np->n_ctime;

  nxmutex_unlock(&nmp->nm_lock);
  return OK;
}

/****************************************************************************
 * Name: nfs_fchstat
 *
 * Description:
 *   Change information about an open file associated with the file
 *   structure 'filep'.
 *
 ****************************************************************************/

static int nfs_fchstat(FAR const struct file *filep,
                       FAR const struct stat *buf, int flags)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode *np;
  int ret;

  finfo("Buf %p\n", buf);
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  np  = (FAR struct nfsnode *)filep->f_priv;

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  DEBUGASSERT(nmp != NULL);

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Change the file mode, owner, group and time. */

  ret = nfs_filechstat(nmp, np, buf, flags);

  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_truncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int nfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode *np;
  int ret;

  finfo("Truncate to %ld bytes\n", (long)length);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (FAR struct nfsmount *)filep->f_inode->i_private;
  np  = (FAR struct nfsnode *)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret >= 0)
    {
      struct stat buf;

      /* Then perform the SETATTR RPC to set the new file size */

      buf.st_size = length;
      ret = nfs_filechstat(nmp, np, &buf, CH_STAT_SIZE);

      nxmutex_unlock(&nmp->nm_lock);
    }

  return ret;
}

/****************************************************************************
 * Name: nfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct fs_dirent_s **dir)
{
  FAR struct nfsmount *nmp;
  FAR struct nfs_dir_s *ndir;
  struct file_handle fhandle;
  struct nfs_fattr obj_attributes;
  uint32_t objtype;
  int ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && dir);

  /* Recover our private data from the inode instance */

  nmp = mountpt->i_private;
  ndir = kmm_zalloc(sizeof(*ndir));
  if (ndir == NULL)
    {
      return -ENOMEM;
    }

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      goto errout_with_ndir;
    }

  /* Find the NFS node associate with the path */

  ret = nfs_findnode(nmp, relpath, &fhandle, &obj_attributes, NULL);
  if (ret != OK)
    {
      ferr("ERROR: nfs_findnode failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* The entry is a directory */

  objtype = fxdr_unsigned(uint32_t, obj_attributes.fa_type);
  if (objtype != NFDIR)
    {
      ferr("ERROR:  Not a directory, type=%" PRId32 "\n", objtype);
      ret = -ENOTDIR;
      goto errout_with_lock;
    }

  /* Save the directory information in struct fs_dirent_s so that it can be
   * used later when readdir() is called.
   */

  ndir->nfs_fhsize = (uint8_t)fhandle.length;
  DEBUGASSERT(fhandle.length <= DIRENT_NFS_MAXHANDLE);

  memcpy(ndir->nfs_fhandle, &fhandle.handle, fhandle.length);
  *dir = &ndir->nfs_base;
  nxmutex_unlock(&nmp->nm_lock);
  return 0;

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
errout_with_ndir:
  kmm_free(ndir);
  return ret;
}

/****************************************************************************
 * Name: nfs_closedir
 *
 * Description:
 *   Close directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_closedir(FAR struct inode *mountpt,
                        FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return 0;
}

/****************************************************************************
 * Name: nfs_readdir
 *
 * Description: Read from directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_readdir(FAR struct inode *mountpt,
                       FAR struct fs_dirent_s *dir,
                       FAR struct dirent *entry)
{
  FAR struct nfsmount *nmp;
  FAR struct nfs_dir_s *ndir;
  struct file_handle fhandle;
  struct nfs_fattr obj_attributes;
  uint32_t readsize;
  uint32_t tmp;
  FAR uint32_t *ptr;
  FAR uint8_t *name;
  unsigned int length;
  int reqlen;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  nmp = mountpt->i_private;
  ndir = (FAR struct nfs_dir_s *)dir;

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

read_dir:
  /* Request a block directory entries, copying directory information from
   * the dirent structure.
   */

  ptr     = (FAR uint32_t *)&nmp->nm_msgbuffer.readdir.readdir;
  reqlen  = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned((uint32_t)ndir->nfs_fhsize);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, ndir->nfs_fhandle, ndir->nfs_fhsize);
  reqlen += uint32_alignup(ndir->nfs_fhsize);
  ptr    += uint32_increment(ndir->nfs_fhsize);

  /* Cookie and cookie verifier */

  ptr[0] = ndir->nfs_cookie[0];
  ptr[1] = ndir->nfs_cookie[1];
  ptr    += 2;
  reqlen += 2*sizeof(uint32_t);

  memcpy(ptr, ndir->nfs_verifier, DIRENT_NFS_VERFLEN);
  ptr    += uint32_increment(DIRENT_NFS_VERFLEN);
  reqlen += DIRENT_NFS_VERFLEN;

  /* Number of directory entries (We currently only process one entry at a
   * time)
   */

  readsize = nmp->nm_readdirsize;
  tmp      = SIZEOF_rpc_reply_readdir(readsize);
  if (tmp > nmp->nm_buflen)
    {
      readsize -= (tmp - nmp->nm_buflen);
    }

  *ptr     = txdr_unsigned(readsize);
  reqlen  += sizeof(uint32_t);

  /* And read the directory */

  nfs_statistics(NFSPROC_READDIR);
  ret = nfs_request(nmp, NFSPROC_READDIR,
                    &nmp->nm_msgbuffer.readdir, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);
  if (ret != OK)
    {
      ferr("ERROR: nfs_request failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* A new group of entries was successfully read.  Process the
   * information contained in the response header.  This information
   * includes:
   *
   * 1) Attributes follow indication - 4 bytes
   * 2) Directory attributes         - sizeof(struct nfs_fattr)
   * 3) Cookie verifier              - NFSX_V3COOKIEVERF bytes
   * 4) Values follows indication    - 4 bytes
   */

  ptr = (FAR uint32_t *)
    &((FAR struct rpc_reply_readdir *)nmp->nm_iobuffer)->readdir;

  /* Check if attributes follow, if 0 so Skip over the attributes */

  tmp = *ptr++;
  if (tmp != 0)
    {
      /* Attributes are not currently used */

      ptr += uint32_increment(sizeof(struct nfs_fattr));
    }

  /* Save the verification cookie */

  memcpy(ndir->nfs_verifier, ptr, DIRENT_NFS_VERFLEN);
  ptr += uint32_increment(DIRENT_NFS_VERFLEN);

next_entry:
  /* Check if values follow.  If no values follow, then the EOF indication
   * will appear next.
   */

  tmp = *ptr++;
  if (tmp == 0)
    {
      /* No values follow, then the reply should consist only of a 4-byte
       * end-of-directory indication.
       */

      tmp = *ptr++;
      if (tmp != 0)
        {
          finfo("End of directory\n");
          ret = -ENOENT;
          goto errout_with_lock;
        }

      /* What would it mean if there were not data and we not at the end of
       * file?
       */

       else
        {
          finfo("No data but not end of directory???\n");
          goto read_dir;
        }
    }

  /* If we are not at the end of the directory listing, then a set of entries
   * will follow the header.  Each entry is of the form:
   *
   *    File ID (8 bytes)
   *    Name length (4 bytes)
   *    Name string (variable size but in multiples of 4 bytes)
   *    Cookie (8 bytes)
   *    next entry (4 bytes)
   */

  /* There is an entry. Skip over the file ID and point to the length */

  ptr += 2;

  /* Get the length and point to the name */

  tmp    = *ptr++;
  length = fxdr_unsigned(uint32_t, tmp);
  name   = (FAR uint8_t *)ptr;

  /* Increment the pointer past the name (allowing for padding). ptr
   * now points to the cookie.
   */

  ptr += uint32_increment(length);

  /* Save the cookie and increment the pointer to the next entry */

  ndir->nfs_cookie[0] = *ptr++;
  ndir->nfs_cookie[1] = *ptr++;

  /* Return the name of the node to the caller */

  if (length > NAME_MAX)
    {
      length = NAME_MAX;
    }

  memcpy(entry->d_name, name, length);
  entry->d_name[length] = '\0';
  finfo("name: \"%s\"\n", entry->d_name);

  if (strcmp(entry->d_name, ".") == 0 ||
      strcmp(entry->d_name, "..") == 0)
    {
      goto next_entry; /* Skip . and .. */
    }

  /* Get the file attributes associated with this name and return
   * the file type.
   */

  fhandle.length = (uint32_t)ndir->nfs_fhsize;
  memcpy(&fhandle.handle, ndir->nfs_fhandle, fhandle.length);

  ret = nfs_lookup(nmp, entry->d_name, &fhandle, &obj_attributes, NULL);
  if (ret != OK)
    {
      ferr("ERROR: nfs_lookup failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Set the dirent file type */

  tmp = fxdr_unsigned(uint32_t, obj_attributes.fa_type);
  switch (tmp)
    {
    default:
    case NFNON:        /* Unknown type */
      break;

    case NFSOCK:       /* Socket */
      entry->d_type = DTYPE_SOCK;
      break;

    case NFLNK:        /* Symbolic link */
      entry->d_type = DTYPE_LINK;
      break;

    case NFREG:        /* Regular file */
      entry->d_type = DTYPE_FILE;
      break;

    case NFDIR:        /* Directory */
      entry->d_type = DTYPE_DIRECTORY;
      break;

    case NFBLK:        /* Block special device file */
      entry->d_type = DTYPE_BLK;
      break;

    case NFFIFO:       /* Named FIFO */
      entry->d_type = DTYPE_FIFO;
      break;

    case NFCHR:        /* Character special device file */
      entry->d_type = DTYPE_CHR;
      break;
    }

  finfo("type: %d->%d\n", (int)tmp, entry->d_type);

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_rewinddir
 *
 * Description:
 *  Reset the directory traversal logic to the first entry in the open
 *  directory.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_rewinddir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct nfs_dir_s *ndir;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && dir != NULL);

  ndir = (FAR struct nfs_dir_s *)dir;

  /* Reset the NFS-specific portions of dirent structure, retaining only the
   * file handle.
   */

  memset(&ndir->nfs_verifier, 0, DIRENT_NFS_VERFLEN);
  ndir->nfs_cookie[0] = 0;
  ndir->nfs_cookie[1] = 0;
  return OK;
}

/****************************************************************************
 * Name: nfs_decode_args
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nfs_decode_args(FAR struct nfs_mount_parameters *nprmt,
                            FAR struct nfs_args *argp)
{
  int maxio;

  /* Get the selected timeout value */

  if ((argp->flags & NFSMNT_TIMEO) != 0 && argp->timeo > 0)
    {
      if (argp->timeo < NFS_MINTIMEO)
        {
          nprmt->timeo = NFS_MINTIMEO;
        }
      else if (argp->timeo > NFS_MAXTIMEO)
        {
          nprmt->timeo = NFS_MAXTIMEO;
        }
      else
        {
          nprmt->timeo = argp->timeo;
        }
    }

  /* Get the selected retransmission count */

  if ((argp->flags & NFSMNT_RETRANS) != 0 && argp->retrans > 1)
    {
      if  (argp->retrans < NFS_MAXREXMIT)
        {
          nprmt->retry = argp->retrans;
        }
      else
        {
          nprmt->retry = NFS_MAXREXMIT;
        }
    }

  if ((argp->flags & NFSMNT_SOFT) == 0)
    {
      nprmt->retry = NFS_MAXREXMIT + 1;  /* Past clip limit */
    }

  /* Get the maximum amount of data that can be transferred in one packet */

  if (argp->sotype == SOCK_DGRAM)
    {
      maxio = NFS_MAXDGRAMDATA;
    }
  else
    {
      maxio = NFS_MAXDATA;
    }

  if (maxio > MAXBSIZE)
    {
      maxio = MAXBSIZE;
    }

  /* Get the maximum amount of data that can be transferred in one write
   * transfer
   */

  if ((argp->flags & NFSMNT_WSIZE) != 0 && argp->wsize > 0)
    {
      nprmt->wsize = argp->wsize;

      /* Round down to multiple of blocksize */

      nprmt->wsize &= ~(NFS_FABLKSIZE - 1);
      if (nprmt->wsize <= 0)
        {
          nprmt->wsize = NFS_FABLKSIZE;
        }
    }

  if (nprmt->wsize > maxio)
    {
      nprmt->wsize = maxio;
    }

  /* Get the maximum amount of data that can be transferred in one read
   * transfer
   */

  if ((argp->flags & NFSMNT_RSIZE) != 0 && argp->rsize > 0)
    {
      nprmt->rsize = argp->rsize;

      /* Round down to multiple of blocksize */

      nprmt->rsize &= ~(NFS_FABLKSIZE - 1);
      if (nprmt->rsize <= 0)
        {
          nprmt->rsize = NFS_FABLKSIZE;
        }
    }

  if (nprmt->rsize > maxio)
    {
      nprmt->rsize = maxio;
    }

  /* Get the maximum amount of data that can be transferred in directory
   * transfer
   */

  if ((argp->flags & NFSMNT_READDIRSIZE) != 0 && argp->readdirsize > 0)
    {
      nprmt->readdirsize = argp->readdirsize;

      /* Round down to multiple of blocksize */

      nprmt->readdirsize &= ~(NFS_DIRBLKSIZ - 1);
      if (nprmt->readdirsize < NFS_DIRBLKSIZ)
        {
          nprmt->readdirsize = NFS_DIRBLKSIZ;
        }
    }
  else if (argp->flags & NFSMNT_RSIZE)
    {
      nprmt->readdirsize = nprmt->rsize;
    }

  if (nprmt->readdirsize > maxio)
    {
      nprmt->readdirsize = maxio;
    }
}

/****************************************************************************
 * Name: nfs_bind
 *
 * Description:
 *  This implements a portion of the mount operation. This function allocates
 *  and initializes the mountpoint private data and gets mount information
 *  from the NFS server.  The final binding of the private data (containing
 *  NFS server mount information) to the  mountpoint is performed by mount().
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                    FAR void **handle)
{
  FAR struct nfs_args        *argp = (FAR struct nfs_args *)data;
  FAR struct nfsmount        *nmp;
  FAR struct rpcclnt         *rpc;
  struct nfs_mount_parameters nprmt;
  uint32_t                    buflen;
  uint32_t                    tmp;
  int                         ret = 0;

  DEBUGASSERT(data && handle);

  /* Set default values of the parameters.  These may be overridden by
   * settings in the argp->flags.
   */

  nprmt.timeo       = NFS_TIMEO;
  nprmt.retry       = NFS_RETRANS;
  nprmt.wsize       = NFS_WSIZE;
  nprmt.rsize       = NFS_RSIZE;
  nprmt.readdirsize = NFS_READDIRSIZE;

  nfs_decode_args(&nprmt, argp);

  /* Determine the size of a buffer that will hold one RPC data transfer.
   * First, get the maximum size of a read and a write transfer.
   */

  buflen = SIZEOF_rpc_call_write(nprmt.wsize);
  tmp    = SIZEOF_rpc_reply_read(nprmt.rsize);

  /* The buffer size will be the maximum of those two sizes */

  if (tmp > buflen)
    {
      buflen = tmp;
    }

  /* And consider the maximum size of a read dir transfer too */

  tmp = SIZEOF_rpc_reply_readdir(nprmt.readdirsize);
  if (tmp > buflen)
    {
      buflen = tmp;
    }

  /* But don't let the buffer size exceed the MSS of the socket type.
   *
   * In the case where there are multiple network devices with different
   * link layer protocols, each network device may support a different
   * UDP MSS value.  Here we arbitrarily select the minimum MSS for
   * that case.
   */

  if (argp->sotype == SOCK_DGRAM && buflen > MIN_UDP_MSS)
    {
      buflen = MIN_UDP_MSS;
    }

  /* Create an instance of the mountpt state structure */

  nmp = (FAR struct nfsmount *)kmm_zalloc(SIZEOF_nfsmount(buflen));
  if (!nmp)
    {
      ferr("ERROR: Failed to allocate mountpoint structure\n");
      return -ENOMEM;
    }

  /* Save the allocated I/O buffer size */

  nmp->nm_buflen = (uint16_t)buflen;

  /* Initialize the allocated mountpt state structure. */

  /* Initialize the mutex that controls access.
   * nxmutex_lock() is called at the completion of
   * initialization, incrementing the count to one.
   */

  nxmutex_init(&nmp->nm_lock);   /* Initialize the mutex that
                                  * controls access */

  /* Initialize NFS */

  nfs_true = txdr_unsigned(TRUE);
  nfs_false = txdr_unsigned(FALSE);
  nfs_xdrneg1 = txdr_unsigned(-1);

  rpcclnt_init();

  /* Set initial values of other fields */

  nmp->nm_wsize       = nprmt.wsize;
  nmp->nm_rsize       = nprmt.rsize;
  nmp->nm_readdirsize = nprmt.readdirsize;

  strlcpy(nmp->nm_path, argp->path, sizeof(nmp->nm_path));
  memcpy(&nmp->nm_nam, &argp->addr, argp->addrlen);

  /* Create an instance of the rpc state structure */

  rpc = (FAR struct rpcclnt *)kmm_zalloc(sizeof(struct rpcclnt));
  if (!rpc)
    {
      ferr("ERROR: Failed to allocate rpc structure\n");
      goto bad;
    }

  finfo("Connecting\n");

  /* Translate nfsmnt flags -> rpcclnt flags */

  rpc->rc_path   = nmp->nm_path;
  rpc->rc_name   = &nmp->nm_nam;
  rpc->rc_sotype = argp->sotype;
  rpc->rc_timeo  = nprmt.timeo;
  rpc->rc_retry  = nprmt.retry;

  nmp->nm_rpcclnt = rpc;

  ret = rpcclnt_connect(nmp->nm_rpcclnt);
  if (ret != OK)
    {
      ferr("ERROR: nfs_connect failed: %d\n", ret);
      goto bad;
    }

  nmp->nm_fhsize = nmp->nm_rpcclnt->rc_fhsize;
  nmp->nm_fh     = &nmp->nm_rpcclnt->rc_fh;

  /* Get the file system info */

  ret = nfs_fsinfo(nmp);
  if (ret)
    {
      ferr("ERROR: nfs_fsinfo failed: %d\n", ret);
      goto bad;
    }

  /* Mounted! */

  *handle = nmp;

  finfo("Successfully mounted\n");
  return OK;

bad:

  /* Disconnect from the server */

  if (nmp->nm_rpcclnt)
    {
      rpcclnt_disconnect(nmp->nm_rpcclnt);
      kmm_free(nmp->nm_rpcclnt);
    }

  /* Free connection-related resources */

  nxmutex_destroy(&nmp->nm_lock);
  kmm_free(nmp);

  return ret;
}

/****************************************************************************
 * Name: nfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                      unsigned int flags)
{
  FAR struct nfsmount *nmp = (FAR struct nfsmount *)handle;
  int ret;

  finfo("Entry\n");
  DEBUGASSERT(nmp);

  /* Get exclusive access to the mount structure */

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there any open files?  We can tell if there are open files by
   * looking at the list of file structures in the mount structure.  If this
   * list not empty, then there are open files and we cannot unmount now (or
   * a crash is sure to follow).
   */

  if (nmp->nm_head != NULL)
    {
      ferr("ERROR;  There are open files: %p\n", nmp->nm_head);

      /* This implementation currently only supports unmounting if there are
       * no open file references.
       */

      ret = (flags != 0) ? -ENOSYS : -EBUSY;
      goto errout_with_lock;
    }

  /* Disconnect from the server */

  rpcclnt_disconnect(nmp->nm_rpcclnt);

  /* And free any allocated resources */

  nxmutex_destroy(&nmp->nm_lock);
  kmm_free(nmp->nm_rpcclnt);
  kmm_free(nmp);

  return OK;

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_fsinfo
 *
 * Description:
 *   Return information about root directory.
 *
 * Returned Value:
 *   0 on success; negative errno value on failure
 *
 * Assumptions:
 *   The caller has exclusive access to the NFS mount structure
 *
 ****************************************************************************/

static int nfs_fsinfo(FAR struct nfsmount *nmp)
{
  FAR struct rpc_call_fs *fsinfo;
  FAR struct rpc_reply_getattr *attr;
  FAR uint32_t *ptr;
  uint32_t pref;
  uint32_t max;
  int ret = 0;

  fsinfo = &nmp->nm_msgbuffer.fsinfo;
  fsinfo->fs.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  memcpy(&fsinfo->fs.fsroot.handle, nmp->nm_fh, nmp->nm_fhsize);

  /* Request FSINFO from the server */

  nfs_statistics(NFSPROC_FSINFO);
  ret = nfs_request(nmp, NFSPROC_FSINFO,
                    fsinfo, sizeof(struct FS3args),
                    nmp->nm_iobuffer, nmp->nm_buflen);
  if (ret)
    {
      return ret;
    }

  /* Save the root file system attributes */

  ptr = (FAR uint32_t *)
    &((FAR struct rpc_reply_fsinfo *)nmp->nm_iobuffer)->fsinfo;
  if (*ptr++ != 0)
    {
      memcpy(&nmp->nm_fattr, ptr, sizeof(struct nfs_fattr));
      ptr += uint32_increment(sizeof(struct nfs_fattr));
    }

  max  = fxdr_unsigned(uint32_t, *ptr++);
  pref = fxdr_unsigned(uint32_t, *ptr++);
  ptr += 1; /* Skip fs_rtmult */

  if (pref < nmp->nm_rsize)
    {
      nmp->nm_rsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  if (max < nmp->nm_rsize)
    {
      nmp->nm_rsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize == 0)
        {
          nmp->nm_rsize = max;
        }
    }

  max  = fxdr_unsigned(uint32_t, *ptr++);
  pref = fxdr_unsigned(uint32_t, *ptr++);
  ptr += 1; /* Skip fs_wtmult */

  if (pref < nmp->nm_wsize)
    {
      nmp->nm_wsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  if (max < nmp->nm_wsize)
    {
      nmp->nm_wsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize == 0)
        {
          nmp->nm_wsize = max;
        }
    }

  pref = fxdr_unsigned(uint32_t, *ptr++);
  if (pref < nmp->nm_readdirsize)
    {
      nmp->nm_readdirsize = (pref + NFS_DIRBLKSIZ - 1) &
                             ~(NFS_DIRBLKSIZ - 1);
    }

  /* Get the file attributes if needed */

  if (nmp->nm_fattr.fa_type == 0)
    {
      nfs_statistics(NFSPROC_GETATTR);
      ret = nfs_request(nmp, NFSPROC_GETATTR,
                        fsinfo, sizeof(struct FS3args),
                        nmp->nm_iobuffer, nmp->nm_buflen);
      if (ret)
        {
          return ret;
        }

      attr = (FAR struct rpc_reply_getattr *)nmp->nm_iobuffer;
      memcpy(&nmp->nm_fattr, &attr->attr, sizeof(struct nfs_fattr));
    }

  return OK;
}

/****************************************************************************
 * Name: nfs_statfs
 *
 * Description:
 *   Return filesystem statistics
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_statfs(FAR struct inode *mountpt, FAR struct statfs *sbp)
{
  FAR struct nfsmount *nmp;
  FAR struct rpc_call_fs *fsstat;
  FAR struct rpc_reply_fsstat *sfp;
  uint64_t tquad;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Fill in the statfs info */

  sbp->f_type = NFS_SUPER_MAGIC;

  fsstat = &nmp->nm_msgbuffer.fsstat;
  fsstat->fs.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  memcpy(&fsstat->fs.fsroot.handle, nmp->nm_fh, nmp->nm_fhsize);

  nfs_statistics(NFSPROC_FSSTAT);
  ret = nfs_request(nmp, NFSPROC_FSSTAT,
                    fsstat, sizeof(struct FS3args),
                    nmp->nm_iobuffer, nmp->nm_buflen);
  if (ret)
    {
      goto errout_with_lock;
    }

  sfp                   = (FAR struct rpc_reply_fsstat *)nmp->nm_iobuffer;
  sbp->f_bsize          = NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_tbytes);
  sbp->f_blocks         = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_fbytes);
  sbp->f_bfree          = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_abytes);
  sbp->f_bavail         = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_tfiles);
  sbp->f_files          = tquad;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_ffiles);
  sbp->f_ffree          = tquad;
  sbp->f_namelen        = NAME_MAX;

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_remove
 *
 * Description:
 *   Remove a file
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_remove(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct nfsmount *nmp;
  struct file_handle   fhandle;
  struct nfs_fattr     fattr;
  char                 filename[NAME_MAX + 1];
  FAR uint32_t        *ptr;
  int                  namelen;
  int                  reqlen;
  int                  ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the NFS node of the directory containing the file to be deleted */

  ret = nfs_finddir(nmp, relpath, &fhandle, &fattr, filename);
  if (ret != OK)
    {
      ferr("ERROR: nfs_finddir returned: %d\n", ret);
      goto errout_with_lock;
    }

  /* Create the REMOVE RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.removef.remove;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  reqlen += uint32_alignup(fhandle.length);
  ptr    += uint32_increment(fhandle.length);

  /* Copy the variable-length file name */

  namelen = strlen(filename);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, filename, namelen);
  reqlen += uint32_alignup(namelen);

  /* Perform the REMOVE RPC call */

  nfs_statistics(NFSPROC_REMOVE);
  ret = nfs_request(nmp, NFSPROC_REMOVE,
                    &nmp->nm_msgbuffer.removef, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_mkdir
 *
 * Description:
 *   Create a directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                     mode_t mode)
{
  FAR struct nfsmount *nmp;
  struct file_handle   fhandle;
  struct nfs_fattr     fattr;
  char                 dirname[NAME_MAX + 1];
  FAR uint32_t        *ptr;
  uint32_t             tmp;
  int                  namelen;
  int                  reqlen;
  int                  ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the NFS node of the directory containing the directory to be
   * created
   */

  ret = nfs_finddir(nmp, relpath, &fhandle, &fattr, dirname);
  if (ret != OK)
    {
      ferr("ERROR: nfs_finddir returned: %d\n", ret);
      goto errout_with_lock;
    }

  /* Format the MKDIR call message arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.mkdir.mkdir;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  ptr    += uint32_increment(fhandle.length);
  reqlen += uint32_alignup(fhandle.length);

  /* Copy the variable-length directory name */

  namelen = strlen(dirname);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, dirname, namelen);
  ptr    += uint32_increment(namelen);
  reqlen += uint32_alignup(namelen);

  /* Set the mode.  NOTE: Here we depend on the fact that the NuttX and NFS
   * bit settings are the same (at least for the bits of interest).
   */

  *ptr++  = nfs_true; /* True: mode value follows */
  reqlen += sizeof(uint32_t);

  tmp = mode & (NFSMODE_IXOTH | NFSMODE_IWOTH | NFSMODE_IROTH |
                NFSMODE_IXGRP | NFSMODE_IWGRP | NFSMODE_IRGRP |
                NFSMODE_IXUSR | NFSMODE_IWUSR | NFSMODE_IRUSR |
                NFSMODE_SAVETEXT | NFSMODE_ISGID | NFSMODE_ISUID);
  *ptr++  = txdr_unsigned(tmp);
  reqlen += sizeof(uint32_t);

  /* Set the user ID to zero */

  *ptr++  = nfs_true;             /* True: Uid value follows */
  *ptr++  = 0;                    /* UID = 0 (nobody) */
  reqlen += 2*sizeof(uint32_t);

  /* Set the group ID to one */

  *ptr++  = nfs_true;            /* True: Gid value follows */
  *ptr++  = 0;                   /* GID = 0 (nogroup) */
  reqlen += 2*sizeof(uint32_t);

  /* No size */

  *ptr++  = nfs_false; /* False: No size value follows */
  reqlen += sizeof(uint32_t);

  /* Don't change times */

  *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change atime */
  *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change mtime */
  reqlen += 2*sizeof(uint32_t);

  /* Perform the MKDIR RPC */

  nfs_statistics(NFSPROC_MKDIR);
  ret = nfs_request(nmp, NFSPROC_MKDIR,
                    &nmp->nm_msgbuffer.mkdir, reqlen,
                    &nmp->nm_iobuffer, nmp->nm_buflen);
  if (ret)
    {
      ferr("ERROR: nfs_request failed: %d\n", ret);
    }

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_rmdir
 *
 * Description:
 *   Remove a directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct nfsmount *nmp;
  struct file_handle   fhandle;
  struct nfs_fattr     fattr;
  char                 dirname[NAME_MAX + 1];
  FAR uint32_t        *ptr;
  int                  namelen;
  int                  reqlen;
  int                  ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the NFS node of the directory containing the directory to be
   * removed
   */

  ret = nfs_finddir(nmp, relpath, &fhandle, &fattr, dirname);
  if (ret != OK)
    {
      ferr("ERROR: nfs_finddir returned: %d\n", ret);
      goto errout_with_lock;
    }

  /* Set up the RMDIR call message arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.rmdir.rmdir;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  reqlen += uint32_alignup(fhandle.length);
  ptr    += uint32_increment(fhandle.length);

  /* Copy the variable-length directory name */

  namelen = strlen(dirname);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, dirname, namelen);
  reqlen += uint32_alignup(namelen);

  /* Perform the RMDIR RPC */

  nfs_statistics(NFSPROC_RMDIR);
  ret = nfs_request(nmp, NFSPROC_RMDIR,
                    &nmp->nm_msgbuffer.rmdir, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_rename
 *
 * Description:
 *   Rename a file or directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                      FAR const char *newrelpath)
{
  FAR struct nfsmount *nmp;
  struct file_handle   from_handle;
  struct file_handle   to_handle;
  char                 from_name[NAME_MAX + 1];
  char                 to_name[NAME_MAX + 1];
  struct nfs_fattr     fattr;
  FAR uint32_t        *ptr;
  int                  namelen;
  int                  reqlen;
  int                  ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Find the NFS node of the directory containing the 'from' object */

  ret = nfs_finddir(nmp, oldrelpath, &from_handle, &fattr, from_name);
  if (ret != OK)
    {
      ferr("ERROR: nfs_finddir returned: %d\n", ret);
      goto errout_with_lock;
    }

  /* Find the NFS node of the directory containing the 'to' object */

  ret = nfs_finddir(nmp, newrelpath, &to_handle, &fattr, to_name);
  if (ret != OK)
    {
      ferr("ERROR: nfs_finddir returned: %d\n", ret);
      goto errout_with_lock;
    }

  /* Format the RENAME RPC arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.renamef.rename;
  reqlen = 0;

  /* Copy the variable length, 'from' directory file handle */

  *ptr++  = txdr_unsigned(from_handle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &from_handle.handle, from_handle.length);
  reqlen += uint32_alignup(from_handle.length);
  ptr    += uint32_increment(from_handle.length);

  /* Copy the variable-length 'from' object name */

  namelen = strlen(from_name);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, from_name, namelen);
  reqlen += uint32_alignup(namelen);
  ptr    += uint32_increment(namelen);

  /* Copy the variable length, 'to' directory file handle */

  *ptr++  = txdr_unsigned(to_handle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &to_handle.handle, to_handle.length);
  ptr    += uint32_increment(to_handle.length);
  reqlen += uint32_alignup(to_handle.length);

  /* Copy the variable-length 'to' object name */

  namelen = strlen(to_name);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, to_name, namelen);
  reqlen += uint32_alignup(namelen);

  /* Perform the RENAME RPC */

  nfs_statistics(NFSPROC_RENAME);
  ret = nfs_request(nmp, NFSPROC_RENAME,
                    &nmp->nm_msgbuffer.renamef, reqlen,
                    nmp->nm_iobuffer, nmp->nm_buflen);

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_stat_mode
 *
 * Description:
 *   Convert NFSv3's type and mode to NuttX's mode
 *
 * Returned Value:
 *   Return NuttX's mode
 *
 ****************************************************************************/

static mode_t nfs_stat_mode(unsigned int mode, unsigned int type)
{
  /* Here we exploit the fact all mode bits are the same in NuttX
   * as in the NFSv3 spec.
   */

  mode &= (NFSMODE_IXOTH | NFSMODE_IWOTH | NFSMODE_IROTH |
           NFSMODE_IXGRP | NFSMODE_IWGRP | NFSMODE_IRGRP |
           NFSMODE_IXUSR | NFSMODE_IWUSR | NFSMODE_IRUSR |
           NFSMODE_SAVETEXT | NFSMODE_ISGID | NFSMODE_ISUID);

  /* Now OR in the file type */

  switch (type)
    {
    default:
    case NFNON:   /* Unknown type */
      break;

    case NFREG:   /* Regular file */
      mode |= S_IFREG;
      break;

    case NFDIR:   /* Directory */
      mode |= S_IFDIR;
      break;

    case NFBLK:   /* Block special device file */
      mode |= S_IFBLK;
      break;

    case NFCHR:   /* Character special device file */
      mode |= S_IFCHR;
      break;

    case NFLNK:   /* Symbolic link */
      mode |= S_IFLNK;
      break;

    case NFSOCK:  /* Socket */
      mode |= S_IFSOCK;
      break;

    case NFFIFO:  /* Named pipe */
      mode |= S_IFIFO;
      break;
    }

  return mode;
}

/****************************************************************************
 * Name: nfs_stat
 *
 * Description:
 *   Return information about the file system object at 'relpath'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                    FAR struct stat *buf)
{
  FAR struct nfsmount *nmp;
  struct file_handle fhandle;
  struct nfs_fattr attributes;
  struct timespec ts;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;
  DEBUGASSERT(nmp && buf);

  memset(buf, 0, sizeof(*buf));

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the file handle attributes of the requested node */

  ret = nfs_findnode(nmp, relpath, &fhandle, &attributes, NULL);
  if (ret != OK)
    {
      ferr("ERROR: nfs_findnode failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Extract the file mode, file type, and file size. */

  buf->st_mode  = nfs_stat_mode(fxdr_unsigned(uint16_t, attributes.fa_mode),
                                fxdr_unsigned(uint8_t, attributes.fa_type));
  buf->st_size  = fxdr_hyper(&attributes.fa_size);

  /* Extract time values as type time_t in units of seconds */

  fxdr_nfsv3time(&attributes.fa_mtime, &ts);
  buf->st_mtime = ts.tv_sec;

  fxdr_nfsv3time(&attributes.fa_atime, &ts);
  buf->st_atime = ts.tv_sec;

  fxdr_nfsv3time(&attributes.fa_ctime, &ts);
  buf->st_ctime = ts.tv_sec;

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}

/****************************************************************************
 * Name: nfs_chstat
 *
 * Description:
 *   Change information about the file system object at 'relpath'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR const struct stat *buf, int flags)
{
  FAR struct nfsmount *nmp;
  struct file_handle fhandle;
  struct nfsnode np;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (FAR struct nfsmount *)mountpt->i_private;
  DEBUGASSERT(nmp && buf);

  ret = nxmutex_lock(&nmp->nm_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the file handle of the requested node */

  ret = nfs_findnode(nmp, relpath, &fhandle, NULL, NULL);
  if (ret != OK)
    {
      ferr("ERROR: nfs_findnode failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* Initialize the nfs node */

  np.n_fhsize = (uint8_t)fhandle.length;
  memcpy(&np.n_fhandle, &fhandle.handle, fhandle.length);

  /* Change the file mode, owner, group and time. */

  ret = nfs_filechstat(nmp, &np, buf, flags);

errout_with_lock:
  nxmutex_unlock(&nmp->nm_lock);
  return ret;
}
