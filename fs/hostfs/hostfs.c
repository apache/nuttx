/****************************************************************************
 * nuttx/fs/hostfs/hostfs.c
 *
 *   Copyright (C) 2015 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/hostfs.h>

#include "hostfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOSTFS_RETRY_DELAY_MS       10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     hostfs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode);
static int     hostfs_close(FAR struct file *filep);
static ssize_t hostfs_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t hostfs_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static off_t   hostfs_seek(FAR struct file *filep, off_t offset,
                        int whence);
static int     hostfs_ioctl(FAR struct file *filep, int cmd,
                        unsigned long arg);

static int     hostfs_sync(FAR struct file *filep);
static int     hostfs_dup(FAR const struct file *oldp,
                        FAR struct file *newp);
static int     hostfs_fstat(FAR const struct file *filep,
                        FAR struct stat *buf);
static int     hostfs_ftruncate(FAR struct file *filep,
                        off_t length);

static int     hostfs_opendir(FAR struct inode *mountpt,
                        FAR const char *relpath,
                        FAR struct fs_dirent_s *dir);
static int     hostfs_closedir(FAR struct inode *mountpt,
                        FAR struct fs_dirent_s *dir);
static int     hostfs_readdir(FAR struct inode *mountpt,
                        FAR struct fs_dirent_s *dir);
static int     hostfs_rewinddir(FAR struct inode *mountpt,
                        FAR struct fs_dirent_s *dir);

static int     hostfs_bind(FAR struct inode *blkdriver,
                        FAR const void *data, FAR void **handle);
static int     hostfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags);
static int     hostfs_statfs(FAR struct inode *mountpt,
                        FAR struct statfs *buf);

static int     hostfs_unlink(FAR struct inode *mountpt,
                        FAR const char *relpath);
static int     hostfs_mkdir(FAR struct inode *mountpt,
                        FAR const char *relpath, mode_t mode);
static int     hostfs_rmdir(FAR struct inode *mountpt, const char *relpath);
static int     hostfs_rename(FAR struct inode *mountpt,
                        FAR const char *oldrelpath,
                        FAR const char *newrelpath);
static int     hostfs_stat(FAR struct inode *mountpt,
                        FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t  g_seminitialized = FALSE;
static sem_t    g_sem;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations hostfs_operations =
{
  hostfs_open,          /* open */
  hostfs_close,         /* close */
  hostfs_read,          /* read */
  hostfs_write,         /* write */
  hostfs_seek,          /* seek */
  hostfs_ioctl,         /* ioctl */

  hostfs_sync,          /* sync */
  hostfs_dup,           /* dup */
  hostfs_fstat,         /* fstat */
  hostfs_ftruncate,     /* ftruncate */

  hostfs_opendir,       /* opendir */
  hostfs_closedir,      /* closedir */
  hostfs_readdir,       /* readdir */
  hostfs_rewinddir,     /* rewinddir */

  hostfs_bind,          /* bind */
  hostfs_unbind,        /* unbind */
  hostfs_statfs,        /* statfs */

  hostfs_unlink,        /* unlinke */
  hostfs_mkdir,         /* mkdir */
  hostfs_rmdir,         /* rmdir */
  hostfs_rename,        /* rename */
  hostfs_stat           /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hostfs_semtake
 ****************************************************************************/

void hostfs_semtake(FAR struct hostfs_mountpt_s *fs)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(fs->fs_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: hostfs_semgive
 ****************************************************************************/

void hostfs_semgive(FAR struct hostfs_mountpt_s *fs)
{
  nxsem_post(fs->fs_sem);
}

/****************************************************************************
 * Name: hostfs_mkpath
 *
 * Description: Build absolute host path from relative NuttX path.
 *
 ****************************************************************************/

static void hostfs_mkpath(FAR struct hostfs_mountpt_s  *fs,
                          FAR const char *relpath,
                          FAR char *path, int pathlen)
{
  int depth = 0;
  int first;
  int x;

  /* Copy base host path to output */

  strncpy(path, fs->fs_root, pathlen);

  /* Be sure we aren't trying to use ".." to display outside of our
   * mounted path.
   */

  x = 0;
  while (relpath[x] == '/')
    {
      x++;
    }

  first = x;

  while (relpath[x] != '\0')
    {
      /* Test for ".." occurance */

      if (strncmp(&relpath[x], "..", 2) == 0)
        {
          /* Reduce depth by 1 */

          depth--;
          x += 2;
        }

      else if (relpath[x] == '/' && relpath[x+1] != '/' && relpath[x+1] != '\0')
        {
          depth++;
          x++;
        }
      else
        {
          x++;
        }
    }

  if (depth >= 0)
    {
      strncat(path, &relpath[first], pathlen-strlen(path)-1);
    }
}

/****************************************************************************
 * Name: hostfs_open
 ****************************************************************************/

static int hostfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s  *hf;
  char path[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT((filep->f_priv == NULL) && (filep->f_inode != NULL));


  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Allocate memory for the open file */

  hf = (struct hostfs_ofile_s *) kmm_malloc(sizeof *hf);
  if (hf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Try to open the file in the host file system */

  hf->fd = host_open(path, oflags, mode);
  if (hf->fd < 0)
    {
      /* Error opening file */

      ret = -EBADF;
      goto errout_with_buffer;
    }

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  if ((oflags & (O_APPEND | O_WRONLY)) == (O_APPEND | O_WRONLY))
    {
      ret = host_lseek(hf->fd, 0, SEEK_END);
      if (ret >= 0)
        {
          filep->f_pos = ret;
        }
      else
        {
          goto errout_with_buffer;
        }
    }

  /* Attach the private date to the struct file instance */

  filep->f_priv = hf;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  hf->fnext = fs->fs_head;
  hf->crefs = 1;
  hf->oflags = oflags;
  fs->fs_head = hf;

  ret = OK;
  goto errout_with_semaphore;

errout_with_buffer:
  kmm_free(hf);

errout_with_semaphore:
  hostfs_semgive(fs);
  if (ret == -EINVAL)
    {
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: hostfs_close
 ****************************************************************************/

static int hostfs_close(FAR struct file *filep)
{
  FAR struct inode            *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s   *hf;
  FAR struct hostfs_ofile_s   *nextfile;
  FAR struct hostfs_ofile_s   *prevfile;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  inode = filep->f_inode;
  fs    = inode->i_private;
  hf    = filep->f_priv;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Check if we are the last one with a reference to the file and
   * only close if we are. */

  if (hf->crefs > 1)
    {
      /* The file is opened more than once.  Just decrement the
       * reference count and return. */

      hf->crefs--;
      goto okout;
    }

  /* Remove ourselves from the linked list */

  nextfile = fs->fs_head;
  prevfile = nextfile;
  while ((nextfile != hf) && (nextfile != NULL))
    {
      /* Save the previous file pointer too */

      prevfile = nextfile;
      nextfile = nextfile->fnext;
    }

  if (nextfile != NULL)
    {
      /* Test if we were the first entry */

      if (nextfile == fs->fs_head)
        {
          /* Assign a new head */

          fs->fs_head = nextfile->fnext;
        }
      else
        {
          /* Take ourselves out of the list */

          prevfile->fnext = nextfile->fnext;
        }
    }

  /* Close the host file */

  host_close(hf->fd);

  /* Now free the pointer */

  filep->f_priv = NULL;
  kmm_free(hf);

okout:
  hostfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: hostfs_read
 ****************************************************************************/

static ssize_t hostfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host to perform the read */

  ret = host_read(hf->fd, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_write
 ****************************************************************************/

static ssize_t hostfs_write(FAR struct file *filep, const char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  ssize_t ret;

  /* Sanity checks.  I have seen the following assertion misfire if
   * CONFIG_DEBUG_MM is enabled while re-directing output to a
   * file.  In this case, the debug output can get generated while
   * the file is being opened,  FAT data structures are being allocated,
   * and things are generally in a perverse state.
   */

#ifdef CONFIG_DEBUG_MM
  if (filep->f_priv == NULL || filep->f_inode == NULL)
    {
      return -ENXIO;
    }
#else
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
#endif

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Test the permissions.  Only allow write if the file was opened with
   * write flags.
   */

  if ((hf->oflags & O_WROK) == 0)
    {
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Call the host to perform the write */

  ret = host_write(hf->fd, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

errout_with_semaphore:
  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_seek
 ****************************************************************************/

static off_t hostfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  off_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call our internal routine to perform the seek */

  ret = host_lseek(hf->fd, offset, whence);
  if (ret >= 0)
    {
      filep->f_pos = ret;
    }

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_ioctl
 ****************************************************************************/

static int hostfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call our internal routine to perform the ioctl */

  ret = host_ioctl(hf->fd, cmd, arg);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int hostfs_sync(FAR struct file *filep)
{
  FAR struct inode            *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s   *hf;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  host_sync(hf->fd);

  hostfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: hostfs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int hostfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct hostfs_ofile_s *sf;

  /* Sanity checks */

  DEBUGASSERT(oldp->f_priv != NULL &&
              newp->f_priv == NULL &&
              newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sf = oldp->f_priv;

  DEBUGASSERT(sf != NULL);

  /* Just increment the reference count on the ofile */

  sf->crefs++;
  newp->f_priv = (FAR void *)sf;

  return OK;
}

/****************************************************************************
 * Name: hostfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int hostfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  hf    = filep->f_priv;
  inode = filep->f_inode;

  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host to perform the read */

  ret = host_fstat(hf->fd, buf);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_ftruncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int hostfs_ftruncate(FAR struct file *filep, off_t length)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  hf    = filep->f_priv;
  inode = filep->f_inode;

  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host to perform the truncate */

  ret = host_ftruncate(hf->fd, length);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int hostfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct hostfs_mountpt_s *fs;
  char path[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host's opendir function */

  dir->u.hostfs.fs_dir = host_opendir(path);
  if (dir->u.hostfs.fs_dir == NULL)
    {
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  ret = OK;

errout_with_semaphore:

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_closedir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int hostfs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  struct hostfs_mountpt_s  *fs;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host's closedir function */

  host_closedir(dir->u.hostfs.fs_dir);

  hostfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: hostfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int hostfs_readdir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct hostfs_mountpt_s *fs;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host OS's readdir function */

  ret = host_readdir(dir->u.hostfs.fs_dir, &dir->fd_dir);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int hostfs_rewinddir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Call the host and let it do all the work */

  host_rewinddir(dir->u.hostfs.fs_dir);

  return OK;
}

/****************************************************************************
 * Name: hostfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int hostfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                       FAR void **handle)
{
  FAR struct hostfs_mountpt_s  *fs;
  struct stat buf;
  FAR char *options;
  char *ptr, *saveptr;
  int len, timeout = 0;
  int ret;

  /* Validate the block driver is NULL */

  if (blkdriver || !data)
    {
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  fs = (FAR struct hostfs_mountpt_s *)kmm_zalloc(sizeof(struct hostfs_mountpt_s));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* The options we suppor are:
   *  "fs=whatever", remote dir
   *  "timeout=xx", bind timeout, unit (ms)
   */

  options = strdup(data);
  if (!options)
    {
      kmm_free(fs);
      return -ENOMEM;
    }

  ptr = strtok_r(options, ",", &saveptr);
  while(ptr != NULL)
    {
      if ((strncmp(ptr, "fs=", 3) == 0))
        {
          strncpy(fs->fs_root, &ptr[3], sizeof(fs->fs_root));
        }
      else if ((strncmp(ptr, "timeout=", 8) == 0))
        {
          timeout = atoi(&ptr[8]);
        }

      ptr = strtok_r(NULL, ",", &saveptr);
    }

  kmm_free(options);

  /* If the global semaphore hasn't been initialized, then
   * initialized it now. */

  fs->fs_sem = &g_sem;
  if (!g_seminitialized)
    {
      /* Initialize the semaphore that controls access */

      nxsem_init(&g_sem, 0, 0);
      g_seminitialized = TRUE;
    }
  else
    {
      /* Take the semaphore for the mount */

      hostfs_semtake(fs);
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  fs->fs_head = NULL;

  /* Now perform the mount.  */

  len = strlen(fs->fs_root);
  if (len > 1 && fs->fs_root[len - 1] == '/')
    {
      /* Remove trailing '/' */

      fs->fs_root[len-1] = '\0';
    }

  /* Try to stat the file in the host FS */

  while (1)
    {
      ret = host_stat(fs->fs_root, &buf);
      if ((ret != 0 && timeout <= 0) ||
              (ret == 0 && (buf.st_mode & S_IFDIR) == 0))
        {
          hostfs_semgive(fs);
          kmm_free(fs);
          return -ENOENT;
        }
      else if (ret == 0)
        {
          break;
        }

      usleep(HOSTFS_RETRY_DELAY_MS * 1000);
      timeout -= HOSTFS_RETRY_DELAY_MS;
    }

  /* Append a '/' to the name now */

  if (fs->fs_root[len-1] != '/')
    {
      strcat(fs->fs_root, "/");
    }

  *handle = (FAR void *)fs;
  hostfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: hostfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int hostfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                         unsigned int flags)
{
  FAR struct hostfs_mountpt_s *fs = (FAR struct hostfs_mountpt_s *)handle;
  int ret;

  if (!fs)
    {
      return -EINVAL;
    }

  /* Check if there are sill any files opened on the filesystem. */

  ret = OK; /* Assume success */
  hostfs_semtake(fs);
  if (fs->fs_head != NULL)
    {
      /* We cannot unmount now.. there are open files */

      hostfs_semgive(fs);

      /* This implementation currently only supports unmounting if there are
       * no open file references.
       */

      return (flags != 0) ? -ENOSYS : -EBUSY;
    }

  hostfs_semgive(fs);
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int hostfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct hostfs_mountpt_s *fs;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Call the host fs to perform the statfs */

  memset(buf, 0, sizeof(struct statfs));
  ret = host_statfs(fs->fs_root, buf);
  buf->f_type = HOSTFS_MAGIC;

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int hostfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct hostfs_mountpt_s *fs;
  char path[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host fs to perform the unlink */

  ret = host_unlink(path);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int hostfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                        mode_t mode)
{
  FAR struct hostfs_mountpt_s *fs;
  char path[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = host_mkdir(path, mode);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

int hostfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct hostfs_mountpt_s *fs;
  char path[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = host_rmdir(path);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

int hostfs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                  FAR const char *newrelpath)
{
  FAR struct hostfs_mountpt_s *fs;
  char oldpath[HOSTFS_MAX_PATH];
  char newpath[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  strncpy(oldpath, fs->fs_root, sizeof(oldpath));
  strncat(oldpath, oldrelpath, sizeof(oldpath)-strlen(oldpath)-1);
  strncpy(newpath, fs->fs_root, sizeof(newpath));
  strncat(newpath, newrelpath, sizeof(newpath)-strlen(newpath)-1);

  /* Call the host FS to do the mkdir */

  ret = host_rename(oldpath, newpath);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int hostfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  FAR struct hostfs_mountpt_s *fs;
  char path[HOSTFS_MAX_PATH];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the stat operation */

  ret = host_stat(path, buf);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
