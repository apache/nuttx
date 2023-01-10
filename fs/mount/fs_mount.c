/****************************************************************************
 * fs/mount/fs_mount.c
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

#include <sys/mount.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "driver/driver.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* In the canonical case, a file system is bound to a block driver.  However,
 * some less typical cases a block driver is not required.  Examples are
 * pseudo file systems (like BINFS or PROCFS) and MTD file systems (like
 * NXFFS).
 *
 * These file systems all require block drivers:
 */

#if defined(CONFIG_FS_FAT) || defined(CONFIG_FS_ROMFS) || \
    defined(CONFIG_FS_SMARTFS) || defined(CONFIG_FS_LITTLEFS) || \
    defined(CONFIG_FS_FATFS)
#  define BDFS_SUPPORT 1
#endif

/* These file systems require MTD drivers */

#if (defined(CONFIG_FS_SPIFFS) || defined(CONFIG_FS_LITTLEFS) || \
    defined(CONFIG_FS_YAFFS)) && defined(CONFIG_MTD)
#  define MDFS_SUPPORT 1
#endif

/* These file systems do not require block or MTD drivers */

#if defined(CONFIG_FS_NXFFS) || defined(CONFIG_FS_BINFS) || \
    defined(CONFIG_FS_PROCFS) || defined(CONFIG_NFS) || \
    defined(CONFIG_FS_TMPFS) || defined(CONFIG_FS_USERFS) || \
    defined(CONFIG_FS_CROMFS) || defined(CONFIG_FS_UNIONFS) || \
    defined(CONFIG_FS_HOSTFS) || defined(CONFIG_FS_ZIPFS)
#  define NODFS_SUPPORT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fsmap_t
{
  FAR const char                      *fs_filesystemtype;
  FAR const struct mountpt_operations *fs_mops;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef BDFS_SUPPORT
/* File systems that require block drivers */

#ifdef CONFIG_FS_FAT
extern const struct mountpt_operations fat_operations;
#endif
#ifdef CONFIG_FS_FATFS
extern const struct mountpt_operations fatfs_operations;
#endif
#ifdef CONFIG_FS_ROMFS
extern const struct mountpt_operations romfs_operations;
#endif
#ifdef CONFIG_FS_SMARTFS
extern const struct mountpt_operations smartfs_operations;
#endif
#ifdef CONFIG_FS_LITTLEFS
extern const struct mountpt_operations littlefs_operations;
#endif

static const struct fsmap_t g_bdfsmap[] =
{
#ifdef CONFIG_FS_FAT
    { "vfat", &fat_operations },
#endif
#ifdef CONFIG_FS_FATFS
    { "fatfs", &fatfs_operations },
#endif
#ifdef CONFIG_FS_ROMFS
    { "romfs", &romfs_operations },
#endif
#ifdef CONFIG_FS_SMARTFS
    { "smartfs", &smartfs_operations },
#endif
#ifdef CONFIG_FS_LITTLEFS
    { "littlefs", &littlefs_operations },
#endif
    { NULL,   NULL },
};
#endif /* BDFS_SUPPORT */

#ifdef MDFS_SUPPORT
/* File systems that require MTD drivers */

#ifdef CONFIG_FS_ROMFS
extern const struct mountpt_operations romfs_operations;
#endif
#ifdef CONFIG_FS_SPIFFS
extern const struct mountpt_operations spiffs_operations;
#endif
#ifdef CONFIG_FS_LITTLEFS
extern const struct mountpt_operations littlefs_operations;
#endif
#ifdef CONFIG_FS_YAFFS
extern const struct mountpt_operations yaffs_operations;
#endif

static const struct fsmap_t g_mdfsmap[] =
{
#ifdef CONFIG_FS_ROMFS
    { "romfs", &romfs_operations },
#endif
#ifdef CONFIG_FS_SPIFFS
    { "spiffs", &spiffs_operations },
#endif
#ifdef CONFIG_FS_LITTLEFS
    { "littlefs", &littlefs_operations },
#endif
#ifdef CONFIG_FS_YAFFS
    { "yaffs", &yaffs_operations },
#endif
    { NULL,   NULL },
};
#endif /* MDFS_SUPPORT */

#ifdef NODFS_SUPPORT
/* File systems that require neither block nor MTD drivers */

#ifdef CONFIG_FS_NXFFS
extern const struct mountpt_operations nxffs_operations;
#endif
#ifdef CONFIG_FS_TMPFS
extern const struct mountpt_operations tmpfs_operations;
#endif
#ifdef CONFIG_NFS
extern const struct mountpt_operations nfs_operations;
#endif
#ifdef CONFIG_FS_BINFS
extern const struct mountpt_operations binfs_operations;
#endif
#ifdef CONFIG_FS_PROCFS
extern const struct mountpt_operations procfs_operations;
#endif
#ifdef CONFIG_FS_USERFS
extern const struct mountpt_operations userfs_operations;
#endif
#ifdef CONFIG_FS_HOSTFS
extern const struct mountpt_operations hostfs_operations;
#endif
#ifdef CONFIG_FS_CROMFS
extern const struct mountpt_operations cromfs_operations;
#endif
#ifdef CONFIG_FS_UNIONFS
extern const struct mountpt_operations unionfs_operations;
#endif
#ifdef CONFIG_FS_RPMSGFS
extern const struct mountpt_operations rpmsgfs_operations;
#endif
#ifdef CONFIG_FS_ZIPFS
extern const struct mountpt_operations zipfs_operations;
#endif
static const struct fsmap_t g_nonbdfsmap[] =
{
#ifdef CONFIG_FS_NXFFS
    { "nxffs", &nxffs_operations },
#endif
#ifdef CONFIG_FS_TMPFS
    { "tmpfs", &tmpfs_operations },
#endif
#ifdef CONFIG_NFS
    { "nfs", &nfs_operations },
#endif
#ifdef CONFIG_FS_BINFS
    { "binfs", &binfs_operations },
#endif
#ifdef CONFIG_FS_PROCFS
    { "procfs", &procfs_operations },
#endif
#ifdef CONFIG_FS_USERFS
    { "userfs", &userfs_operations },
#endif
#ifdef CONFIG_FS_HOSTFS
    { "hostfs", &hostfs_operations },
#endif
#ifdef CONFIG_FS_CROMFS
    { "cromfs", &cromfs_operations },
#endif
#ifdef CONFIG_FS_UNIONFS
    { "unionfs", &unionfs_operations },
#endif
#ifdef CONFIG_FS_RPMSGFS
    { "rpmsgfs", &rpmsgfs_operations },
#endif
#ifdef CONFIG_FS_ZIPFS
    { "zipfs", &zipfs_operations},
#endif
    { NULL, NULL },
};
#endif /* NODFS_SUPPORT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_findfs
 *
 * Description:
 *    find the specified filesystem
 *
 ****************************************************************************/

#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT) || defined(NODFS_SUPPORT)
static FAR const struct mountpt_operations *
mount_findfs(FAR const struct fsmap_t *fstab, FAR const char *filesystemtype)
{
  FAR const struct fsmap_t *fsmap;

  for (fsmap = fstab; fsmap->fs_filesystemtype; fsmap++)
    {
      if (strcmp(filesystemtype, fsmap->fs_filesystemtype) == 0)
        {
          return fsmap->fs_mops;
        }
    }

  return NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_mount
 *
 * Description:
 *   nx_mount() is similar to the standard 'mount' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_mount() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

int nx_mount(FAR const char *source, FAR const char *target,
             FAR const char *filesystemtype, unsigned long mountflags,
             FAR const void *data)
{
#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT) || defined(NODFS_SUPPORT)
  FAR struct inode *drvr_inode = NULL;
  FAR struct inode *mountpt_inode;
  FAR const struct mountpt_operations *mops = NULL;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  struct inode_search_s desc;
#endif
  void *fshandle;
  int ret;

  /* Verify required pointer arguments */

  DEBUGASSERT(target && filesystemtype);

  /* Find the specified filesystem. Try the block driver filesystems first */

  if (source != NULL &&
      find_blockdriver(source, mountflags, &drvr_inode) >= 0)
    {
      /* Find the block based file system */

#ifdef BDFS_SUPPORT
      mops = mount_findfs(g_bdfsmap, filesystemtype);
#endif /* BDFS_SUPPORT */
      if (mops == NULL)
        {
          ferr("ERROR: Failed to find block based file system %s\n",
               filesystemtype);

          ret = -ENODEV;
          goto errout_with_inode;
        }
    }
  else if (source != NULL &&
           (ret = find_mtddriver(source, &drvr_inode)) >= 0)
    {
      /* Find the MTD based file system */

#ifdef MDFS_SUPPORT
      mops = mount_findfs(g_mdfsmap, filesystemtype);
#endif /* MDFS_SUPPORT */
      if (mops == NULL)
        {
          ferr("ERROR: Failed to find MTD based file system %s\n",
               filesystemtype);

          ret = -ENODEV;
          goto errout_with_inode;
        }
    }
  else
#ifdef NODFS_SUPPORT
  if ((mops = mount_findfs(g_nonbdfsmap, filesystemtype)) != NULL)
    {
    }
  else
#endif /* NODFS_SUPPORT */
    {
      ferr("ERROR: Failed to find block driver %s\n", source);

      ret = -ENOTBLK;
      goto errout;
    }

  ret = inode_lock();
  if (ret < 0)
    {
      goto errout_with_inode;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* Check if the inode already exists */

  SETUP_SEARCH(&desc, target, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Successfully found.  The reference count on the inode has been
       * incremented.
       */

      mountpt_inode = desc.node;
      DEBUGASSERT(mountpt_inode != NULL);

      /* But is it a directory node (i.e., not a driver or other special
       * node)?
       */

      if (!INODE_IS_PSEUDODIR(mountpt_inode))
        {
          ferr("ERROR: target %s exists and is a special node\n", target);
          ret = -ENOTDIR;
          inode_release(mountpt_inode);
          goto errout_with_lock;
        }
    }
  else
#endif

  /* Insert a dummy node -- we need to hold the inode semaphore
   * to do this because we will have a momentarily bad structure.
   * NOTE that the new inode will be created with an initial reference
   * count of zero.
   */

    {
      ret = inode_reserve(target, 0777, &mountpt_inode);
      if (ret < 0)
        {
          /* inode_reserve can fail for a couple of reasons, but the most
           * likely one is that the inode already exists. inode_reserve may
           * return:
           *
           *  -EINVAL - 'path' is invalid for this operation
           *  -EEXIST - An inode already exists at 'path'
           *  -ENOMEM - Failed to allocate in-memory resources for the
           *            operation
           */

          ferr("ERROR: Failed to reserve inode for target %s\n", target);
          goto errout_with_lock;
        }
    }

  /* Bind the block driver to an instance of the file system.  The file
   * system returns a reference to some opaque, fs-dependent structure
   * that encapsulates this binding.
   */

  if (mops->bind == NULL)
    {
      /* The filesystem does not support the bind operation ??? */

      ferr("ERROR: Filesystem does not support bind\n");
      ret = -EINVAL;
      goto errout_with_mountpt;
    }

  /* Increment reference count for the reference we pass to the file system */

#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT)
#ifdef NODFS_SUPPORT
  if (drvr_inode != NULL)
#endif
    {
      drvr_inode->i_crefs++;
    }
#endif

  /* On failure, the bind method returns -errorcode */

#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT)
  ret = mops->bind(drvr_inode, data, &fshandle);
#else
  ret = mops->bind(NULL, data, &fshandle);
#endif
  if (ret < 0)
    {
      /* The inode is unhappy with the driver for some reason.  Back out
       * the count for the reference we failed to pass and exit with an
       * error.
       */

      ferr("ERROR: Bind method failed: %d\n", ret);

#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT)
#ifdef NODFS_SUPPORT
      if (drvr_inode != NULL)
#endif
        {
          drvr_inode->i_crefs--;
        }
#endif

      goto errout_with_mountpt;
    }

  /* We have it, now populate it with driver specific information. */

  INODE_SET_MOUNTPT(mountpt_inode);

  mountpt_inode->u.i_mops  = mops;
  mountpt_inode->i_private = fshandle;
  inode_unlock();

  /* We can release our reference to the blkdrver_inode, if the filesystem
   * wants to retain the blockdriver inode (which it should), then it must
   * have called inode_addref().  There is one reference on mountpt_inode
   * that will persist until umount2() is called.
   */

#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT)
#ifdef NODFS_SUPPORT
  if (drvr_inode != NULL)
#endif
    {
      inode_release(drvr_inode);
    }
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  RELEASE_SEARCH(&desc);
#endif
  return OK;

  /* A lot of goto's!  But they make the error handling much simpler */

errout_with_mountpt:
  inode_release(mountpt_inode);
  inode_remove(target);

errout_with_lock:
  inode_unlock();
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  RELEASE_SEARCH(&desc);
#endif

errout_with_inode:
#if defined(BDFS_SUPPORT) || defined(MDFS_SUPPORT)
  if (drvr_inode != NULL)
    {
      inode_release(drvr_inode);
    }
#endif

errout:
  return ret;

#else
  ferr("ERROR: No filesystems enabled\n");
  return -ENOSYS;
#endif /* BDFS_SUPPORT || MDFS_SUPPORT || NODFS_SUPPORT */
}

/****************************************************************************
 * Name: mount
 *
 * Description:
 *   mount() attaches the filesystem specified by the 'source' block device
 *   name into the root file system at the path specified by 'target.'
 *
 * Returned Value:
 *   Zero is returned on success; -1 is returned on an error and errno is
 *   set appropriately:
 *
 *   EACCES A component of a path was not searchable or mounting a read-only
 *      filesystem was attempted without giving the MS_RDONLY flag.
 *   EBUSY 'source' is already  mounted.
 *   EFAULT One of the pointer arguments points outside the user address
 *      space.
 *   EINVAL 'source' had an invalid superblock.
 *   ENODEV 'filesystemtype' not configured
 *   ENOENT A pathname was empty or had a nonexistent component.
 *   ENOMEM Could not allocate a memory to copy filenames or data into.
 *   ENOTBLK 'source' is not a block device
 *
 ****************************************************************************/

int mount(FAR const char *source, FAR const char *target,
          FAR const char *filesystemtype, unsigned long mountflags,
          FAR const void *data)
{
  int ret;

  ret = nx_mount(source, target, filesystemtype, mountflags, data);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

