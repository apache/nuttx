/****************************************************************************
 * net/procfs/net_procfs.c
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
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <fnmatch.h>
#include <libgen.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "procfs/procfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_NET)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAY_SIZE(x)   (sizeof(x) / sizeof((x)[0]))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Read statistics function type */

typedef CODE ssize_t (*read_stat_t)(FAR struct netprocfs_file_s *priv,
                                    FAR char *buffer, size_t buflen);

struct netprocfs_entry_s
{
  uint8_t                                 type;  /* Type of file */
  FAR const char                         *name;  /* File name */
  union
    {
      read_stat_t                         stat;  /* Read statistics hook */
      FAR const struct procfs_operations *ops;
    } u;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     netprocfs_open(FAR struct file *filep,
                              FAR const char *relpath,
                              int oflags, mode_t mode);
static int     netprocfs_close(FAR struct file *filep);
static ssize_t netprocfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     netprocfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     netprocfs_opendir(FAR const char *relpath,
                 FAR struct fs_dirent_s **dir);
static int     netprocfs_closedir(FAR struct fs_dirent_s *dir);
static int     netprocfs_readdir(FAR struct fs_dirent_s *dir,
                                 FAR struct dirent *entry);
static int     netprocfs_rewinddir(FAR struct fs_dirent_s *dir);

static int     netprocfs_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern const struct procfs_operations net_procfs_routeoperations;

/* Netprocfs component mappings */

static const struct netprocfs_entry_s g_net_entries[] =
{
#ifdef CONFIG_NET_STATISTICS
  {
    DTYPE_FILE, "stat",
    {
      netprocfs_read_netstats
    }
  },
#  ifdef CONFIG_NET_MLD
  {
    DTYPE_FILE, "mld",
    {
      netprocfs_read_mldstats
    }
  },
#  endif
#endif
#ifdef CONFIG_NET_ROUTE
  {
    DTYPE_DIRECTORY, "route",
    {
      (FAR void *)&net_procfs_routeoperations
    }
  },
#endif
  {
    DTYPE_FILE, "",
    {
      netprocfs_read_devstats
    }
  }
};

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
  FAR struct net_driver_s *dev = NULL;
  FAR struct netprocfs_file_s *priv;
  int i;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0) &&
      (net_procfsoperations.write == NULL))
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* For each net entries */

  for (i = 0; i < ARRAY_SIZE(g_net_entries); i++)
    {
      if (strncmp(relpath + 4, g_net_entries[i].name,
                  strlen(g_net_entries[i].name)))
        {
          continue;
        }

      if (g_net_entries[i].type == DTYPE_DIRECTORY)
        {
          return g_net_entries[i].u.ops->open(filep, relpath, oflags, mode);
        }

      break;
    }

  if (i == ARRAY_SIZE(g_net_entries) - 1)
    {
      FAR char *devname;
      FAR char *copy;

      /* Otherwise, we need to search the list of registered network devices
       * to determine if the name corresponds to a network device.
       */

      copy = strdup(relpath);
      if (copy == NULL)
        {
          ferr("ERROR: strdup failed\n");
          return -ENOMEM;
        }

      devname = basename(copy);
      dev     = netdev_findbyname(devname);
      kmm_free(copy);

      if (dev == NULL)
        {
          ferr("ERROR: relpath is '%s'\n", relpath);
          return -ENOENT;
        }
    }

  /* Allocate the open file structure */

  priv = (FAR struct netprocfs_file_s *)
    kmm_zalloc(sizeof(struct netprocfs_file_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Initialize the open-file structure */

  priv->dev   = dev;
  priv->entry = i;

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
  ssize_t nreturned = -EINVAL;

  finfo("buffer=%p buflen=%lu\n", buffer, (unsigned long)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct netprocfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Read according to the sub-directory */

  nreturned = g_net_entries[priv->entry].u.stat(priv,
                                                buffer, buflen);

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

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (FAR struct netprocfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = (FAR struct netprocfs_file_s *)
    kmm_zalloc(sizeof(struct netprocfs_file_s));
  if (!newpriv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribute from the old attributes to the new */

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
                             FAR struct fs_dirent_s **dir)
{
  FAR struct netprocfs_level1_s *level1;
  int ndevs;
  int ret;
  int i;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath && dir);

  /* Subdirectory ?  */

  for (i = 0; i < ARRAY_SIZE(g_net_entries); i++)
    {
      if (strncmp(relpath + 4, g_net_entries[i].name,
                  strlen(g_net_entries[i].name)))
        {
          continue;
        }

      if (g_net_entries[i].type == DTYPE_DIRECTORY)
        {
          return g_net_entries[i].u.ops->opendir(relpath, dir);
        }

      break;
    }

  /* Assume that path refers to the 1st level subdirectory.  Allocate the
   * level1 the dirent structure before checking.
   */

  level1 = (FAR struct netprocfs_level1_s *)
     kmm_zalloc(sizeof(struct netprocfs_level1_s));

  if (level1 == NULL)
    {
      ferr("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  level1->base.level = 1;

  if (strcmp(relpath, "net") == 0 || strcmp(relpath, "net/") == 0)
    {
      /* Count the number of network devices */

      ndevs = netdev_count();

      /* Initialize base structure components */

      level1->base.nentries = ndevs;

      /* Add other enabled net components, except netdev */

      level1->base.nentries += ARRAY_SIZE(g_net_entries) - 1;
    }
  else
    {
      /* REVISIT: We really need to check if the relpath refers to a network
       * device.  In that case, we need to return -ENOTDIR.  Otherwise, we
       * should return -ENOENT.
       */

      ferr("ERROR: Bad relpath: %s\n", relpath);
      ret = -ENOTDIR;
      goto errout_with_alloc;
    }

  *dir = (FAR struct fs_dirent_s *)level1;
  return OK;

errout_with_alloc:
  kmm_free(level1);
  return ret;
}

/****************************************************************************
 * Name: netprocfs_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int netprocfs_closedir(FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: netprocfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int netprocfs_readdir(FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry)
{
  FAR struct netprocfs_level1_s *level1;
  FAR struct net_driver_s *dev;
  int index;
  int ret;

  DEBUGASSERT(dir);
  level1 = (FAR struct netprocfs_level1_s *)dir;
  DEBUGASSERT(level1->base.level > 0);

  /* Are we searching this directory?  Or is it just an intermediate on the
   * way to a sub-directory?
   */

  if (level1->base.level == 1)
    {
      /* This directory..  Have we reached the end of the directory? */

      index = level1->base.index;
      DEBUGASSERT(index <= level1->base.nentries);

      if (index >= level1->base.nentries)
        {
          /* We signal the end of the directory by returning the special
           * error -ENOENT.
           */

          finfo("Entry %d: End of directory\n", index);
          return -ENOENT;
        }

      /* Process other enabled net components, except netdev */

      if (index < ARRAY_SIZE(g_net_entries) - 1)
        {
          entry->d_type = g_net_entries[index].type;
          strlcpy(entry->d_name,
                  g_net_entries[index].name, sizeof(entry->d_name));
        }

      /* Net device entry */

      else
        {
          int ifindex;

#ifdef CONFIG_NETDEV_IFINDEX
          /* For the first network device, ifindex will be zero.  We have
           * to take some special action to get the correct starting
           * ifindex.
           */

          if (level1->ifindex == 0)
            {
              ifindex = netdev_nextindex(1);
            }
          else
            {
              ifindex = netdev_nextindex(level1->ifindex);
            }

          if (ifindex < 0)
            {
              /* There are no more... one must have been unregistered */

              return -ENOENT;
            }

          level1->ifindex = ifindex + 1;
#else
          /* Get the raw index */

          ifindex = index - ARRAY_SIZE(g_net_entries) + 1;
#endif
          /* Find the device corresponding to this device index */

          dev = netdev_findbyindex(ifindex);
          if (dev == NULL)
            {
              /* What happened? */

              return -ENOENT;
            }

          /* Copy the device statistics file entry */

          entry->d_type = DTYPE_FILE;
          strlcpy(entry->d_name, dev->d_ifname, sizeof(entry->d_name));
        }

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      level1->base.index = index + 1;
      ret = OK;
    }
  else
    {
      /* We are performing a directory search of one of the subdirectories
       * and we must let the handler perform the read.
       */

      DEBUGASSERT(level1->base.procfsentry != NULL &&
                  level1->base.procfsentry->ops->readdir != NULL);

      ret = level1->base.procfsentry->ops->readdir(dir, entry);
    }

  return ret;
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

  DEBUGASSERT(dir);
  priv = (FAR struct netprocfs_level1_s *)dir;

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
  int i;

  /* Check for the directory "net" */

  buf->st_mode = 0;

  if (strcmp(relpath, "net") == 0 || strcmp(relpath, "net/") == 0)
    {
      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
    }
  else
    {
      for (i = 0; i < ARRAY_SIZE(g_net_entries); i++)
        {
          if (strcmp(relpath + 4, g_net_entries[i].name) == 0)
            {
              if (g_net_entries[i].type == DTYPE_FILE)
                {
                  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
                }
              else if (g_net_entries[i].type == DTYPE_DIRECTORY)
                {
                  buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
                }

              break;
            }
        }
    }

  if (buf->st_mode == 0)
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
          ferr("ERROR: strdup failed\n");
          return -ENOMEM;
        }

      devname = basename(copy);
      dev     = netdev_findbyname(devname);
      kmm_free(copy);

      if (dev == NULL)
        {
          ferr("ERROR: relpath is '%s'\n", relpath);
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

  finfo("buffer=%p buflen=%lu\n", buffer, (unsigned long)buflen);

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
