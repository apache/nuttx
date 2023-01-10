/****************************************************************************
 * fs/mount/fs_gettype.c
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

#ifdef CONFIG_FS_FATFS
      case FATFS_SUPER_MAGIC:
        fstype = "fatfs";
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

#ifdef CONFIG_FS_SPIFFS
      case SPIFFS_SUPER_MAGIC:
        fstype = "spiffs";
        break;
#endif

#ifdef CONFIG_FS_LITTLEFS
      case LITTLEFS_SUPER_MAGIC:
        fstype = "littlefs";
        break;
#endif

#ifdef CONFIG_FS_YAFFS
      case YAFFS_SUPER_MAGIC:
        fstype = "yaffs";
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

#ifdef CONFIG_FS_RPMSGFS
      case RPMSGFS_MAGIC:
        fstype = "rpmsgfs";
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
