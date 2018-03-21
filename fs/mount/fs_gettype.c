/****************************************************************************
 * fs/mount/fs_mount.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
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

#include <sys/statfs.h>

#include "mount/mount.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_gettype
 *
 * Description:
 *   Given the result of statfs(), return a string representing the type of
 *   the file system.
 *
 * Input Parameters:
 *   statbuf - The result of a previouis statbuf statfs on the file system.
 *
 * Returned Value:
 *   A reference to a string representing the type of the file system.
 *
 ****************************************************************************/

FAR const char *fs_gettype(FAR struct statfs *statbuf)
{
  FAR const char *fstype;

  /* Get the file system type */

  switch (statbuf->f_type)
    {
#ifdef CONFIG_FS_FAT
      case MSDOS_SUPER_MAGIC:
        fstype = "vfat";
        break;
#endif

#ifdef CONFIG_FS_ROMFS
      case ROMFS_MAGIC:
        fstype = "romfs";
        break;
#endif

#ifdef CONFIG_FS_CROMFS
      case CROMFS_MAGIC:
        fstype = "cromfs";
        break;
#endif

#ifdef CONFIG_FS_TMPFS
      case TMPFS_MAGIC:
        fstype = "tmpfs";
        break;
#endif

#ifdef CONFIG_FS_BINFS
      case BINFS_MAGIC:
        fstype = "binfs";
        break;
#endif

#ifdef CONFIG_FS_NXFFS
      case NXFFS_MAGIC:
        fstype = "nxffs";
        break;
#endif

#ifdef CONFIG_NFS
      case NFS_SUPER_MAGIC:
        fstype = "nfs";
        break;
#endif

#ifdef CONFIG_FS_SMARTFS
      case SMARTFS_MAGIC:
        fstype = "smartfs";
        break;
#endif

#ifdef CONFIG_FS_PROCFS
      case PROCFS_MAGIC:
        fstype = "procfs";
        break;
#endif

#ifdef CONFIG_FS_UNIONFS
      case UNIONFS_MAGIC:
        fstype = "unionfs";
        break;
#endif

#ifdef CONFIG_FS_HOSTFS
      case HOSTFS_MAGIC:
        fstype = "hostfs";
        break;
#endif

#ifdef CONFIG_FS_USERFS
      case USERFS_MAGIC:
        fstype = "userfs";
        break;
#endif

      default:
        fstype = "Unrecognized";
        break;
    }

  return fstype;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
