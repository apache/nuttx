/****************************************************************************
 * net/procfs/net_procfs.c
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

#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <libgen.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "procfs/procfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_NET)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     netprocfs_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     netprocfs_close(FAR struct file *filep);
static ssize_t netprocfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     netprocfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     netprocfs_opendir(FAR const char *relpath,
                 FAR struct fs_dirent_s *dir);
static int     netprocfs_closedir(FAR struct fs_dirent_s *dir);
static int     netprocfs_readdir(FAR struct fs_dirent_s *dir);
static int     netprocfs_rewinddir(FAR struct fs_dirent_s *dir);

static int     netprocfs_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations net_procfsoperations =
{
  netprocfs_open,       /* open */
  netprocfs_close,      /* close */
  netprocfs_read,       /* read */
  NULL,                 /* write */
  netprocfs_dup,        /* dup */

  netprocfs_opendir,    /* opendir */
  netprocfs_closedir,   /* closedir */
  netprocfs_readdir,    /* readdir */
  netprocfs_rewinddir,  /* rewinddir */

  netprocfs_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_open
 ****************************************************************************/

static int netprocfs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode)
{
  FAR struct netprocfs_file_s *priv;
  FAR struct net_driver_s *dev;

  fvdbg("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0) &&
      (net_procfsoperations.write == NULL))
    {
      fdbg("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* "net/stat" is an acceptable value for the relpath only if network layer
   * statistics are enabled.
   */

#ifdef CONFIG_NET_STATISTICS
  if (strcmp(relpath, "net/stat") == 0)
    {
      /* A NULL network device reference is a clue that we are processing
       * the network statistics file.
       */

      dev = NULL;
    }
  else
#endif
    {
      FAR char *devname;
      FAR char *copy;

      /* Otherwise, we need to search the list of registered network devices
       * to determine if the name corresponds to a network device.
       */

      copy = strdup(relpath);
      if (copy == NULL)
        {
          fdbg("ERROR: strdup failed\n");
          return -ENOMEM;
        }

      devname = basename(copy);
      dev     = netdev_findbyname(devname);
      kmm_free(copy);

      if (dev == NULL)
        {
          fdbg("ERROR: relpath is '%s'\n", relpath);
          return -ENOENT;
        }
    }

  /* Allocate the open file structure */

  priv = (FAR struct netprocfs_file_s *)kmm_zalloc(sizeof(struct netprocfs_file_s));
  if (!priv)
    {
      fdbg("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Initialize the open-file structure */

  priv->dev = dev;

  /* Save the open file structure as the open-specific state in
   * filep->f_priv.
   */

  filep->f_priv = (FAR void *)priv;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_close
 ****************************************************************************/

static int netprocfs_close(FAR struct file *filep)
{
  FAR struct netprocfs_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct netprocfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_read
 ****************************************************************************/

static ssize_t netprocfs_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct netprocfs_file_s *priv;
  ssize_t nreturned;

  fvdbg("buffer=%p buflen=%lu\n", buffer, (unsigned long)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct netprocfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

#ifdef CONFIG_NET_STATISTICS
  /* A NULL device structure reference is the key that we are showing the
   * network statistics.
   */

  if (priv->dev == NULL)
    {
      /* Show the network layer statistics */

      nreturned = netprocfs_read_netstats(priv, buffer, buflen);
    }
  else
#endif
   {
      /* Otherwise, we are showing device-specific statistics */

      nreturned = netprocfs_read_devstats(priv, buffer, buflen);
   }

  /* Update the file offset */

  if (nreturned > 0)
    {
      filep->f_pos += nreturned;
    }

  return nreturned;
}

/****************************************************************************
 * Name: netprocfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int netprocfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct netprocfs_file_s *oldpriv;
  FAR struct netprocfs_file_s *newpriv;

  fvdbg("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (FAR struct netprocfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = (FAR struct netprocfs_file_s *)kmm_zalloc(sizeof(struct netprocfs_file_s));
  if (!newpriv)
    {
      fdbg("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribtes from the old attributes to the new */

  memcpy(newpriv, oldpriv, sizeof(struct netprocfs_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newpriv;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int netprocfs_opendir(FAR const char *relpath,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct netprocfs_level1_s *level1;
  int ndevs;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath && dir && !dir->u.procfs);

  /* "net" is the only value of relpath that is a directory */

  if (strcmp(relpath, "net") != 0)
    {
      /* REVISIT: We really need to check if the relpath refers to a network
       * device.  In that case, we need to return -ENOTDIR.  Otherwise, we
       * should return -ENOENT.
       */

      fdbg("ERROR: Bad relpath: %s\n", relpath);
      return -ENOTDIR;
    }

  /* The path refers to the 1st level sbdirectory.  Allocate the level1
   * dirent structure.
   */

  level1 = (FAR struct netprocfs_level1_s *)
     kmm_zalloc(sizeof(struct netprocfs_level1_s));

  if (!level1)
    {
      fdbg("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  /* Count the number of network devices */

  ndevs = netdev_count();

  /* Initialze base structure components */

  level1->base.level    = 1;
#ifdef CONFIG_NET_STATISTICS
  level1->base.nentries = ndevs + 1;
#else
  level1->base.nentries = ndevs;
#endif
  level1->base.index    = 0;

  dir->u.procfs = (FAR void *) level1;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int netprocfs_closedir(FAR struct fs_dirent_s *dir)
{
  FAR struct netprocfs_level1_s *priv;

  DEBUGASSERT(dir && dir->u.procfs);
  priv = dir->u.procfs;

  if (priv)
    {
      kmm_free(priv);
    }

  dir->u.procfs = NULL;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int netprocfs_readdir(FAR struct fs_dirent_s *dir)
{
  FAR struct netprocfs_level1_s *level1;
  FAR struct net_driver_s *dev;
  int index;

  DEBUGASSERT(dir && dir->u.procfs);
  level1 = dir->u.procfs;
  DEBUGASSERT(level1->base.level == 1);

  /* Have we reached the end of the directory */

  index = level1->base.index;
  DEBUGASSERT(index <= level1->base.nentries);

  if (index >= level1->base.nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT.
       */

      fvdbg("Entry %d: End of directory\n", index);
      return -ENOENT;
    }

#ifdef CONFIG_NET_STATISTICS
  else if (index == 0)
    {
      /* Copy the network statistics directory entry */

      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, "stat", NAME_MAX + 1);
    }
  else
#endif
    {
      int devndx = index;

#ifdef CONFIG_NET_STATISTICS
      /* Subtract one to account for index == 0 which is used for network
       * status.
       */

      devndx--;
#endif

      /* Find the device corresponding to this device index */

      dev = netdev_findbyindex(devndx);

      /* Copy the device statistics file entry */

      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, dev->d_ifname, NAME_MAX + 1);
    }

  /* Set up the next directory entry offset.  NOTE that we could use the
   * standard f_pos instead of our own private index.
   */

  level1->base.index = index + 1;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int netprocfs_rewinddir(FAR struct fs_dirent_s *dir)
{
  FAR struct netprocfs_level1_s *priv;

  DEBUGASSERT(dir && dir->u.procfs);
  priv = dir->u.procfs;

  priv->base.index = 0;
  return OK;
}

/****************************************************************************
 * Name: netprocfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int netprocfs_stat(FAR const char *relpath, FAR struct stat *buf)
{
  /* Check for the directory "net" */

  if (strcmp(relpath, "net") == 0)
    {
      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
    }
  else
#ifdef CONFIG_NET_STATISTICS
  /* Check for network statistics "net/stat" */

  if (strcmp(relpath, "net/stat") == 0)
    {
      buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
    }
  else
#endif
    {
      FAR struct net_driver_s *dev;
      FAR char *devname;
      FAR char *copy;

      /* Otherwise, we need to search the list of registered network devices
       * to determine if the name corresponds to a network device.
       */

      copy = strdup(relpath);
      if (copy == NULL)
        {
          fdbg("ERROR: strdup failed\n");
          return -ENOMEM;
        }

      devname = basename(copy);
      dev     = netdev_findbyname(devname);
      kmm_free(copy);

      if (dev == NULL)
        {
          fdbg("ERROR: relpath is '%s'\n", relpath);
          return -ENOENT;
        }

      buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
    }

  /* File/directory size, access block size */

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_read_linegen
 *
 * Description:
 *   Read and format procfs data using a line generation table.
 *
 * Input Parameters:
 *   priv   - A reference to the network procfs file structure
 *   buffer - The user-provided buffer into which device status will be
 *            returned.
 *   buflen - The size in bytes of the user provided buffer.
 *   gentab - Table of line generation functions
 *   nelems - The number of elements in the table
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

ssize_t netprocfs_read_linegen(FAR struct netprocfs_file_s *priv,
                               FAR char *buffer, size_t buflen,
                               FAR const linegen_t *gentab, int nelems)
{
  size_t xfrsize;
  ssize_t nreturned;

  fvdbg("buffer=%p buflen=%lu\n", buffer, (unsigned long)buflen);

  /* Is there line data already buffered? */

  nreturned = 0;
  if (priv->linesize > 0)
    {
      /* Yes, how much can we transfer now? */

      xfrsize = priv->linesize;
      if (xfrsize > buflen)
        {
          xfrsize = buflen;
        }

      /* Transfer the data to the user buffer */

      memcpy(buffer, &priv->line[priv->offset], xfrsize);

      /* Update pointers, sizes, and offsets */

      buffer         += xfrsize;
      buflen         -= xfrsize;

      priv->linesize -= xfrsize;
      priv->offset   += xfrsize;
      nreturned       = xfrsize;
    }

  /* Loop until the user buffer is full or until all of the network
   * statistics have been transferred.  At this point we know that
   * either:
   *
   * 1. The user buffer is full, and/or
   * 2. All of the current line data has been transferred.
   */

  while (buflen > 0 && priv->lineno < nelems)
    {
      int len;

      /* Read the next line into the working buffer */

      len = gentab[priv->lineno](priv);

      /* Update line-related information */

      priv->lineno++;
      priv->linesize = len;
      priv->offset = 0;

      /* Transfer data to the user buffer */

      xfrsize = priv->linesize;
      if (xfrsize > buflen)
        {
          xfrsize = buflen;
        }

      memcpy(buffer, &priv->line[priv->offset], xfrsize);

      /* Update pointers, sizes, and offsets */

      buffer         += xfrsize;
      buflen         -= xfrsize;

      priv->linesize -= xfrsize;
      priv->offset   += xfrsize;
      nreturned      += xfrsize;
    }

  return nreturned;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * !CONFIG_FS_PROCFS_EXCLUDE_NET */
