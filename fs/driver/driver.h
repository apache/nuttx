/****************************************************************************
 * fs/driver/driver.h
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

#ifndef __FS_DRIVER_DRIVER_H
#define __FS_DRIVER_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: find_blockdriver
 *
 * Description:
 *   Return the inode of the block driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname   - The full path to the block driver to be located
 *   mountflags - If MS_RDONLY is not set, then driver must support write
 *                operations (see include/sys/mount.h)
 *   ppinode    - Address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   ENOENT  - No block driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not a block driver
 *   EACCESS - The MS_RDONLY option was not set but this driver does not
 *             support write access
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int find_blockdriver(FAR const char *pathname, int mountflags,
                     FAR struct inode **ppinode);
#endif

/****************************************************************************
 * Name: block_proxy
 *
 * Description:
 *   Create a temporary char driver using drivers/bch to mediate character
 *   oriented accessed to the block driver.
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 *   blkdev - The path to the block driver
 *   oflags - Character driver open flags
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int block_proxy(FAR struct file *filep, FAR const char *blkdev, int oflags);
#endif

/****************************************************************************
 * Name: mtd_proxy
 *
 * Description:
 *   Create a temporary block driver using drivers/mtd/ftl to mediate block
 *   oriented accessed to the mtd driver.
 *
 * Input Parameters:
 *   mtddev  - The path to the mtd driver
 *   mountflags - if MS_RDONLY is not set, then driver must support write
 *     operations (see include/sys/mount.h)
 *   ppinode - address of the location to return the inode reference
 *
 * Returned Value:
 *   If zero, non-zero inode pointer is returned on success.  This
 *   is the inode pointer of the nameless block driver that mediates
 *   accesses to the mtd driver. A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD
int mtd_proxy(FAR const char *mtddev, int mountflags,
              FAR struct inode **ppinode);
#endif

/****************************************************************************
 * Name: find_mtddriver
 *
 * Description:
 *   Return the inode of the named MTD driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname   - the full path to the named MTD driver to be located
 *   ppinode    - address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   ENOENT  - No MTD driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not an MTD driver
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int find_mtddriver(FAR const char *pathname, FAR struct inode **ppinode);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_DRIVER_DRIVER_H */
