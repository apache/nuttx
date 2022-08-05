/****************************************************************************
 * fs/smartfs/smartfs_procfs.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/smart.h>

#include <arch/irq.h>
#include "smartfs.h"

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_EXCLUDE_SMARTFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration identifies all of the thread attributes that can be
 * accessed via the procfs file system.
 */

/* Level 1 is the directory of attributes */

struct smartfs_level1_s
{
  struct procfs_dir_priv_s  base;    /* Base directory private data */

  /* Add context specific data types here for managing the directory
   * open / read / stat, etc.
   */

  FAR struct smartfs_mountpt_s *mount;
  uint8_t direntry;
};

/* This structure describes one open "file" */

struct smartfs_file_s
{
  struct procfs_file_s  base;        /* Base open file structure */

  /* Add context specific data types for managing an open file here */

  struct smartfs_level1_s level1;    /* Reference to item being accessed */
  uint16_t  offset;
};

struct smartfs_procfs_entry_s
{
  const char  *name;                 /* Name of the directory entry */
  size_t (*read)(FAR struct file *filep, FAR char *buffer, size_t buflen);
  ssize_t (*write)(FAR struct file *filep,
                   FAR const char *buffer, size_t buflen);
  uint8_t type;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int      smartfs_open(FAR struct file *filep, FAR const char *relpath,
                  int oflags, mode_t mode);
static int      smartfs_close(FAR struct file *filep);
static ssize_t  smartfs_read(FAR struct file *filep, FAR char *buffer,
                  size_t buflen);
static ssize_t  smartfs_write(FAR struct file *filep, FAR const char *buffer,
                  size_t buflen);

static int      smartfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int      smartfs_opendir(const char *relpath,
                  FAR struct fs_dirent_s **dir);
static int      smartfs_closedir(FAR struct fs_dirent_s *dir);
static int      smartfs_readdir(FAR struct fs_dirent_s *dir,
                                FAR struct dirent *entry);
static int      smartfs_rewinddir(FAR struct fs_dirent_s *dir);

static int      smartfs_stat(FAR const char *relpath, FAR struct stat *buf);

static ssize_t  smartfs_debug_write(FAR struct file *filep,
                  FAR const char *buffer, size_t buflen);
static size_t   smartfs_status_read(FAR struct file *filep, FAR char *buffer,
                  size_t buflen);
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
static size_t   smartfs_mem_read(FAR struct file *filep, FAR char *buffer,
                  size_t buflen);
#endif
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
static size_t   smartfs_erasemap_read(FAR struct file *filep,
                  FAR char *buffer, size_t buflen);
#endif
#ifdef CONFIG_SMARTFS_FILE_SECTOR_DEBUG
static size_t   smartfs_files_read(FAR struct file *filep, FAR char *buffer,
                  size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct smartfs_procfs_entry_s g_direntry[] =
{
  { "debuglevel", NULL, smartfs_debug_write, DTYPE_FILE },
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  { "erasemap",   smartfs_erasemap_read, NULL, DTYPE_FILE },
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  { "mem",        smartfs_mem_read, NULL, DTYPE_FILE },
#endif
  { "status",     smartfs_status_read, NULL, DTYPE_FILE }
};

static const uint8_t g_direntrycount = sizeof(g_direntry) /
                      sizeof(struct smartfs_procfs_entry_s);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations smartfs_procfsoperations =
{
  smartfs_open,       /* open */
  smartfs_close,      /* close */
  smartfs_read,       /* read */

  /* No write supported */

  smartfs_write,      /* write */

  smartfs_dup,        /* dup */

  smartfs_opendir,    /* opendir */
  smartfs_closedir,   /* closedir */
  smartfs_readdir,    /* readdir */
  smartfs_rewinddir,  /* rewinddir */

  smartfs_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartfs_find_dirref
 *
 * Description:
 *   Analyse relpath to find the directory reference entry it represents,
 *   if any.
 *
 ****************************************************************************/

static int smartfs_find_dirref(FAR const char *relpath,
            FAR struct smartfs_level1_s *level1)
{
  int         ret = -ENOENT;
  FAR struct  smartfs_mountpt_s *mount;
  uint16_t    x;
  FAR char *  str;

  mount = smartfs_get_first_mount();

  /* Skip the "fs/smartfs" portion of relpath */

  if (strncmp(relpath, "fs/smartfs", 10) == 0)
    {
      relpath += 10;
    }

  if (relpath[0] == '/')
    {
      relpath++;
    }

  /* Now test if doing a full dir listing of fs/smartfs */

  if (relpath[0] == '\0')
    {
      /* Save the mount as the first one to display */

      level1->mount = mount;
      level1->base.level    = 1;
      level1->base.nentries = 0;
      while (mount != NULL)
        {
          level1->base.nentries++;
          mount = mount->fs_next;
        }

      level1->base.index    = 0;
      ret = OK;
    }
  else
    {
      /* Search for the requested entry */

      str = strchr(relpath, '/');
      if (str)
        {
          x = str - relpath;
        }
      else
        {
          x = strlen(relpath);
        }

      while (mount)
        {
          if (strncmp(mount->fs_blkdriver->i_name, relpath, x) == 0)
            {
              /* Found the mount point.  Just break */

              break;
            }

          /* Try the next mount */

          mount = mount->fs_next;
        }

      if (mount)
        {
          /* Save the mount and skip it in the relpath */

          ret = OK;
          level1->mount = mount;
          relpath += strlen(mount->fs_blkdriver->i_name);
          if (relpath[0] == '/')
            {
              relpath++;
            }

          /* Test if a level 3 directory entry being requested or not */

          if (relpath[0] == '\0')
            {
              /* Requesting directory listing of a specific SMARTFS mount or
               * entry
               */

              level1->base.level    = 2;
              level1->base.nentries = g_direntrycount;
              level1->base.index    = 0;
            }
          else
            {
              /* Find the level 3 directory entry */

              level1->base.level    = 3;
              level1->base.nentries = 1;
              level1->base.index    = 0;
              level1->direntry      = 0;

              while (level1->direntry < g_direntrycount)
                {
                  /* Test if this entry matches */

                  if (!strcmp(relpath, g_direntry[level1->direntry].name))
                    {
                      break;
                    }

                  /* Advance to next entry */

                  level1->direntry++;
                }

              /* Test if entry found or not */

              if (level1->direntry == g_direntrycount)
                {
                  ret = -ENOENT;
                }
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_open
 ****************************************************************************/

static int smartfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct smartfs_file_s *priv;
  int   ret;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0) &&
      (smartfs_procfsoperations.write == NULL))
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate a container to hold the task and attribute selection */

  priv = kmm_malloc(sizeof(struct smartfs_file_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Find the directory entry being opened */

  ret = smartfs_find_dirref(relpath, &priv->level1);
  if (ret == -ENOENT)
    {
      /* Entry not found */

      kmm_free(priv);
      return ret;
    }

  priv->offset = 0;

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)priv;
  return OK;
}

/****************************************************************************
 * Name: smartfs_close
 ****************************************************************************/

static int smartfs_close(FAR struct file *filep)
{
  FAR struct smartfs_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct smartfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: smartfs_read
 ****************************************************************************/

static ssize_t smartfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct smartfs_file_s *priv;
  ssize_t ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct smartfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Perform the read based on the directory entry */

  ret = 0;

  if (priv->level1.base.level == 3)
    {
      if (priv->level1.direntry < g_direntrycount)
        {
          if (g_direntry[priv->level1.direntry].read)
            {
              ret = g_direntry[priv->level1.direntry].read(filep,
                                                           buffer, buflen);
            }
        }
    }

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_write
 ****************************************************************************/

static ssize_t smartfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct smartfs_file_s *priv;
  ssize_t ret;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct smartfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Perform the write based on the directory entry */

  ret = 0;

  if (priv->level1.base.level == 3)
    {
      if (priv->level1.direntry < g_direntrycount)
        {
          if (g_direntry[priv->level1.direntry].write)
            {
              ret = g_direntry[priv->level1.direntry].write(filep,
                                                            buffer, buflen);
            }
        }
    }

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int smartfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct smartfs_file_s *oldpriv;
  FAR struct smartfs_file_s *newpriv;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (FAR struct smartfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = kmm_malloc(sizeof(struct smartfs_file_s));
  if (!newpriv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribute from the old attributes to the new */

  memcpy(newpriv, oldpriv, sizeof(struct smartfs_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newpriv;
  return OK;
}

/****************************************************************************
 * Name: smartfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int smartfs_opendir(FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  FAR struct smartfs_level1_s *level1;
  int        ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath);

  /* The path refers to the 1st level subdirectory.  Allocate the level1
   * dirent structure.
   */

  level1 = (FAR struct smartfs_level1_s *)
     kmm_malloc(sizeof(struct smartfs_level1_s));

  if (!level1)
    {
      ferr("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  /* Initialize base structure components */

  ret = smartfs_find_dirref(relpath, level1);

  if (ret == OK)
    {
      *dir = (FAR struct fs_dirent_s *)level1;
    }
  else
    {
      kmm_free(level1);
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int smartfs_closedir(FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: smartfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int smartfs_readdir(FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  FAR struct smartfs_level1_s *level1;
  int ret;
  int index;

  DEBUGASSERT(dir);
  level1 = (FAR struct smartfs_level1_s *)dir;

  /* Have we reached the end of the directory */

  index = level1->base.index;
  if (index >= level1->base.nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      finfo("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }

  /* We are traversing a subdirectory of task attributes */

  else
    {
      DEBUGASSERT(level1->base.level >= 1);

      /* Test the type of directory listing */

      if (level1->base.level == 1)
        {
          /* Listing the top level (mounted smartfs volumes) */

          if (!level1->mount)
            {
              return -ENOENT;
            }

          entry->d_type = DTYPE_DIRECTORY;
          strlcpy(entry->d_name, level1->mount->fs_blkdriver->i_name,
                  sizeof(entry->d_name));

          /* Advance to next entry */

          level1->base.index++;
          level1->mount = level1->mount->fs_next;
        }
      else if (level1->base.level == 2)
        {
          /* Listing the contents of a specific mount */

          entry->d_type = g_direntry[level1->base.index].type;
          strlcpy(entry->d_name, g_direntry[level1->base.index++].name,
                  sizeof(entry->d_name));
        }
      else if (level1->base.level == 3)
        {
          /* Listing the contents of a specific entry */

          entry->d_type = g_direntry[level1->base.index].type;
          strlcpy(entry->d_name, g_direntry[level1->direntry].name,
                  sizeof(entry->d_name));
          level1->base.index++;
        }

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int smartfs_rewinddir(struct fs_dirent_s *dir)
{
  FAR struct smartfs_level1_s *priv;

  DEBUGASSERT(dir);
  priv = (FAR struct smartfs_level1_s *)dir;

  priv->base.index = 0;
  return OK;
}

/****************************************************************************
 * Name: smartfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int smartfs_stat(const char *relpath, struct stat *buf)
{
  int ret;
  struct smartfs_level1_s level1;

  /* Decide if the relpath is valid and if it is a file
   *        or a directory and set it's permissions.
   */

  ret = smartfs_find_dirref(relpath, &level1);

  buf->st_mode = S_IROTH | S_IRGRP | S_IRUSR;
  if (ret == OK)
    {
      if (level1.base.level < 3)
        {
          buf->st_mode |= S_IFDIR;
        }
      else
        {
          /* The entry being stat'ed is lowest level */

          if (g_direntry[level1.direntry].type == DTYPE_DIRECTORY)
            {
              buf->st_mode |= S_IFDIR;
            }
          else
            {
              buf->st_mode |= S_IFREG;
            }

          /* Test if the entry is writable */

          if (g_direntry[level1.direntry].write != NULL)
            {
              buf->st_mode |= S_IWOTH | S_IWGRP | S_IWUSR;
            }
        }
    }

  /* File/directory size, access block size */

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;

  return ret;
}

/****************************************************************************
 * Name: smartfs_debug_write
 *
 * Description: Performs the write operation for the "debug" file
 *
 ****************************************************************************/

static ssize_t smartfs_debug_write(FAR struct file *filep,
                                   FAR const char *buffer, size_t buflen)
{
  struct mtd_smart_debug_data_s debug_data;
  FAR struct smartfs_file_s *priv;

  priv = (FAR struct smartfs_file_s *) filep->f_priv;

  /* Populate the debug_data structure */

  debug_data.debugcmd = SMART_DEBUG_CMD_SET_DEBUG_LEVEL;
  debug_data.debugdata = atoi(buffer);

  priv->level1.mount->fs_blkdriver->u.i_bops->ioctl(
      priv->level1.mount->fs_blkdriver, BIOC_DEBUGCMD,
      (unsigned long) &debug_data);

  return buflen;
}

/****************************************************************************
 * Name: smartfs_status_read
 *
 * Description: Performs the read operation for the "status" dir entry.
 *
 ****************************************************************************/

static size_t smartfs_status_read(FAR struct file *filep, FAR char *buffer,
                                  size_t buflen)
{
  struct mtd_smart_procfs_data_s procfs_data;
  FAR struct smartfs_file_s *priv;
  int       ret;
  size_t    len;
  int       utilization;

  priv = (FAR struct smartfs_file_s *) filep->f_priv;

  /* Initialize the read length to zero and test if we are at the
   * end of the file (i.e. already read the data.
   */

  len = 0;
  if (priv->offset == 0)
    {
      /* Get the ProcFS data from the block driver */

      ret = priv->level1.mount->fs_blkdriver->u.i_bops->ioctl(
          priv->level1.mount->fs_blkdriver, BIOC_GETPROCFSD,
          (unsigned long) &procfs_data);

      if (ret == OK)
        {
          /* Calculate the sector utilization percentage */

          if (procfs_data.blockerases == 0)
            {
              utilization = 100;
            }
          else
            {
              utilization = 100 * (procfs_data.blockerases *
                            procfs_data.sectorsperblk -
                            procfs_data.unusedsectors) /
                            (procfs_data.blockerases *
                            procfs_data.sectorsperblk);
            }

          /* Format and return data in the buffer */

          len = snprintf(buffer, buflen,
                         "Format version:    %d\nName Len:          %d\n"
                         "Total Sectors:     %d\nSector Size:       %d\n"
                         "Format Sector:     %d\nDir Sector:        %d\n"
                         "Free Sectors:      %d\nReleased Sectors:  %d\n"
                         "Unused Sectors:    %" PRIu32 "\n"
                         "Block Erases:      %" PRIu32 "\n"
                         "Sectors Per Block: %d\nSector Utilization:%d%%\n"
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
                         "Uneven Wear Count: %" PRIu32 "\n"
#endif
                  ,
                  procfs_data.formatversion, procfs_data.namelen,
                  procfs_data.totalsectors, procfs_data.sectorsize,
                  procfs_data.formatsector, procfs_data.dirsector,
                  procfs_data.freesectors, procfs_data.releasesectors,
                  procfs_data.unusedsectors, procfs_data.blockerases,
                  procfs_data.sectorsperblk, utilization
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
                  , procfs_data.uneven_wearcount
#endif
           );
        }

      /* Indicate we have already provided all the data */

      priv->offset = 0xff;
    }

  return len;
}

/****************************************************************************
 * Name: smartfs_mem_read
 *
 * Description: Performs the read operation for the "mem" dir entry.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
static size_t   smartfs_mem_read(FAR struct file *filep, FAR char *buffer,
                  size_t buflen)
{
  struct mtd_smart_procfs_data_s procfs_data;
  FAR struct smartfs_file_s *priv;
  int       ret;
  uint16_t  x;
  size_t    len;
  size_t    total;

  priv = (FAR struct smartfs_file_s *) filep->f_priv;

  /* Initialize the read length to zero and test if we are at the
   * end of the file (i.e. already read the data.
   */

  len = 0;
  if (priv->offset == 0)
    {
      /* Get the ProcFS data from the block driver */

      ret = priv->level1.mount->fs_blkdriver->u.i_bops->ioctl(
          priv->level1.mount->fs_blkdriver, BIOC_GETPROCFSD,
          (unsigned long) &procfs_data);

      if (ret == OK)
        {
          /* Print the allocations to the buffer */

          total = 0;
          len = snprintf(buffer, buflen, "Allocations:\n");
          buflen -= len;
          for (x = 0; x < procfs_data.alloccount; x++)
            {
              /* Only print allocations with a non-NULL pointer */

              if (procfs_data.allocs[x].ptr != NULL)
                {
                  len += snprintf(&buffer[len], buflen - len, "   %s: %d\n",
                    procfs_data.allocs[x].name, procfs_data.allocs[x].size);
                  total += procfs_data.allocs[x].size;
                }
            }

          /* Add the total allocation amount to the buffer */

          len += snprintf(&buffer[len], buflen - len,
                          "\nTotal: %d\n", total);
        }

      /* Indicate we have done the read */

      priv->offset = 0xff;
    }

  return len;
}
#endif

/****************************************************************************
 * Name: smartfs_erasemap_read
 *
 * Description: Performs the read operation for the "erase" dir entry.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
static size_t   smartfs_erasemap_read(FAR struct file *filep,
                                      FAR char *buffer, size_t buflen)
{
  struct mtd_smart_procfs_data_s procfs_data;
  FAR struct smartfs_file_s *priv;
  int       ret;
  int       rows;
  int       cols;
  size_t    x;
  size_t    y;
  size_t    len;
  size_t    copylen;

  priv = (FAR struct smartfs_file_s *) filep->f_priv;

  /* Get the ProcFS data from the block driver */

  ret = priv->level1.mount->fs_blkdriver->u.i_bops->ioctl(
      priv->level1.mount->fs_blkdriver, BIOC_GETPROCFSD,
      (unsigned long) &procfs_data);
  if (ret != OK)
    {
      return 0;
    }

  /* Initialize the read length to zero and test if we are at the
   * end of the file (i.e. already read the data).
   */

  len = 0;
  rows = 32;
  cols = procfs_data.neraseblocks / rows;
  while (rows >= 4 && (cols < 64 || cols > 128))
    {
      rows >>= 1;
      cols = procfs_data.neraseblocks / rows;
    }

  /* Continue sending data until everything sent.  We add 'rows' below to
   * account for the \n at the end of each line.
   */

  if (priv->offset < procfs_data.neraseblocks + rows)
    {
      /* copylen keeps track of the current length.  When it is
       * equal to or greater than the offset, we start sending data
       * again.  Basically we are starting at the beginning each time
       * and only sending where we left off and discarding the rest.
       */

      copylen = 0;
      for (y = 0; y < rows; y++)
        {
          for (x = 0; x < cols; x++)
            {
              /* Copy data to the buffer */

              if (copylen >= priv->offset)
                {
                  buffer[len++] =
                    procfs_data.erasecounts[y * cols + x] + 'A';
                  priv->offset++;

                  if (len >= buflen)
                    {
                      return len;
                    }
                }

              copylen++;
            }

          /* Add a trailing \n */

          if (copylen >= priv->offset)
            {
              buffer[len++] = '\n';
              priv->offset++;
              if (len >= buflen)
                {
                  return len;
                }
            }

          /* Terminate the string */

          if (copylen >= priv->offset)
            {
              buffer[len++] = '\0';
              priv->offset++;
            }

          copylen++;
        }
    }

  return len;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_SMARTFS */
