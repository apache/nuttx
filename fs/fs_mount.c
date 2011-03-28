/****************************************************************************
 * fs/fs_mount.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/mount.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/fs.h>

#ifdef CONFIG_APPS_BINDIR
#  include <apps/apps.h>
#endif

#include "fs_internal.h"

/* At least one filesystem must be defined, or this file will not compile.
 * It may be desire-able to make filesystems dynamically registered at
 * some time in the future, but at present, this file needs to know about
 * every configured filesystem.
 */

#ifdef CONFIG_FS_READABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fsmap_t
{
  const char                      *fs_filesystemtype;
  const struct mountpt_operations *fs_mops;
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#ifdef CONFIG_FS_FAT
extern const struct mountpt_operations fat_operations;
#endif
#ifdef CONFIG_FS_ROMFS
extern const struct mountpt_operations romfs_operations;
#endif

static const struct fsmap_t g_fsmap[] =
{
#ifdef CONFIG_FS_FAT
    { "vfat", &fat_operations },
#endif
#ifdef CONFIG_FS_ROMFS
    { "romfs", &romfs_operations },
#endif
#ifdef CONFIG_APPS_BINDIR
    { "binfs", &binfs_operations },
#endif
    { NULL,   NULL },
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

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

static FAR const struct mountpt_operations *mount_findfs(const char *filesystemtype )
{
  const struct fsmap_t *fsmap;
  for (fsmap = g_fsmap; fsmap->fs_filesystemtype; fsmap++)
    {
      if (strcmp(filesystemtype, fsmap->fs_filesystemtype) == 0)
        {
            return fsmap->fs_mops;
        }
    }
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount
 *
 * Description:
 *   mount() attaches the filesystem specified by the 'source' block device
 *   name into the root file system at the path specified by 'target.'
 *
 * Return:
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

int mount(const char *source, const char *target,
          const char *filesystemtype, unsigned long mountflags,
          const void *data)
{
  FAR struct inode *blkdrvr_inode;
  FAR struct inode *mountpt_inode;
  FAR const struct mountpt_operations *mops;
  void *fshandle;
  int errcode;
  int status;

  /* Verify required pointer arguments */

  if (!source || !target || !filesystemtype)
    {
      errcode = EFAULT;
      goto errout;
    }

  /* Find the specified filesystem */

  mops = mount_findfs(filesystemtype);
  if (!mops)
    {
      fdbg("Failed to find filsystem %s\n", filesystemtype);
      errcode = ENODEV;
      goto errout;
    }

  /* Find the inode of the block driver indentified by 'source' */

  status = find_blockdriver(source, mountflags, &blkdrvr_inode);
  if (status < 0)
    {
       fdbg("Failed to find block driver %s\n", source);
       errcode = -status;
       goto errout;
    }

  /* Insert a dummy node -- we need to hold the inode semaphore
   * to do this because we will have a momentarily bad structure.
   */

  inode_semtake();
  mountpt_inode = inode_reserve(target);
  if (!mountpt_inode)
    {
      /* inode_reserve can fail for a couple of reasons, but the most likely
       * one is that the inode already exists.
       */

      fdbg("Failed to reserve inode\n");
      errcode = EBUSY;
      goto errout_with_semaphore;
    }

  /* Bind the block driver to an instance of the file system.  The file
   * system returns a reference to some opaque, fs-dependent structure
   * that encapsulates this binding.
   */

  if (!mops->bind)
    {
      /* The filesystem does not support the bind operation ??? */

      fdbg("Filesystem does not support bind\n");
      errcode = EINVAL;
      goto errout_with_mountpt;
    }

  /* Increment reference count for the reference we pass to the file system */

  blkdrvr_inode->i_crefs++;

  /* On failure, the bind method returns -errorcode */

  status = mops->bind(blkdrvr_inode, data, &fshandle);
  if (status != 0)
  {
      /* The inode is unhappy with the blkdrvr for some reason.  Back out
       * the count for the reference we failed to pass and exit with an
       * error.
       */

      fdbg("Bind method failed: %d\n", status);
      blkdrvr_inode->i_crefs--;
      errcode = -status;
      goto errout_with_mountpt;
  }

  /* We have it, now populate it with driver specific information. */

  INODE_SET_MOUNTPT(mountpt_inode);

  mountpt_inode->u.i_mops  = mops;
#ifdef CONFIG_FILE_MODE
  mountpt_inode->i_mode    = mode;
#endif
  mountpt_inode->i_private = fshandle;
  inode_semgive();

 /* We can release our reference to the blkdrver_inode, if the filesystem
  * wants to retain the blockdriver inode (which it should), then it must
  * have called inode_addref().  There is one reference on mountpt_inode
  * that will persist until umount() is called.
  */

  inode_release(blkdrvr_inode);
  return OK;

  /* A lot of goto's!  But they make the error handling much simpler */

errout_with_mountpt:
  mountpt_inode->i_crefs = 0;
  inode_remove(target);
  inode_semgive();
  inode_release(blkdrvr_inode);
  inode_release(mountpt_inode);
  goto errout;

errout_with_semaphore:
  inode_semgive();
  inode_release(blkdrvr_inode);

errout:
  errno = errcode;
  return ERROR;
}

#endif /* Need at least one filesystem */

