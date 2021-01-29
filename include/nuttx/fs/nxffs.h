/****************************************************************************
 * include/nuttx/fs/nxffs.h
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

#ifndef __INCLUDE_NUTTX_FS_NXFFS_H
#define __INCLUDE_NUTTX_FS_NXFFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If the erased state of FLASH memory is anything other than 0xff, then this
 * configuration should be provided.
 */

#ifndef CONFIG_NXFFS_ERASEDSTATE
#  define CONFIG_NXFFS_ERASEDSTATE 0xff
#endif

#if CONFIG_NXFFS_ERASEDSTATE != 0xff && CONFIG_NXFFS_ERASEDSTATE != 0x00
#  error "CONFIG_NXFFS_ERASEDSTATE must be either 0x00 or 0xff"
#endif

/* Don't bother trying to pack things closer together than this. */

#ifndef CONFIG_NXFFS_PACKTHRESHOLD
#  define CONFIG_NXFFS_PACKTHRESHOLD 32
#endif

/* This is how big an inode name is permitted to be. */

#ifndef CONFIG_NXFFS_MAXNAMLEN
#  define CONFIG_NXFFS_MAXNAMLEN 255
#endif

/* Clean-up can either mean packing files together toward the end of the file
 * or, if file are deleted at the end of the file, clean up can simply mean
 * erasing the end of FLASH memory so that it can be re-used again.  However,
 * doing this can also harm the life of the FLASH part because it can mean
 * that the tail end of the FLASH is re-used too often.
 *
 * This threshold determines if/when it is worth erased the tail end of FLASH
 * and making it available for re-use (and possible over-wear).
 */

#ifndef CONFIG_NXFFS_TAILTHRESHOLD
#  define CONFIG_NXFFS_TAILTHRESHOLD (8*1024)
#endif

/* At present, only a single pre-allocated NXFFS volume is supported.  This
 * is because here can be only a single NXFFS volume mounted at any time.
 * This has to do with the fact that we bind to an MTD driver (instead of a
 * block driver) and bypass all of the normal mount operations.
 */

#undef CONFIG_NXFSS_PREALLOCATED
#define CONFIG_NXFSS_PREALLOCATED 1

/* If we were asked to scan the volume, then a re-formatting threshold must
 * also be provided.
 */

#ifdef CONFIG_NXFFS_SCAN_VOLUME
#  ifndef CONFIG_NXFFS_REFORMAT_THRESH
#    define CONFIG_NXFFS_REFORMAT_THRESH 20
#  endif
#  if CONFIG_NXFFS_REFORMAT_THRESH < 0 || CONFIG_NXFFS_REFORMAT_THRESH > 100
#    error CONFIG_NXFFS_REFORMAT_THRESH is not a valid percentage
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: nxffs_initialize
 *
 * Description:
 *   Initialize to provide NXFFS on an MTD interface
 *
 * Input Parameters:
 *   mtd - The MTD device that supports the FLASH interface.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

struct mtd_dev_s;
int nxffs_initialize(FAR struct mtd_dev_s *mtd);

/****************************************************************************
 * Name: nxffs_dump
 *
 * Description:
 *   Dump a summary of the contents of an NXFFS file system.
 *   CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_FS must be enabled for this
 *   function to do anything.
 *
 * Input Parameters:
 *   mtd - The MTD device that provides the interface to NXFFS-formatted
 *     media.
 *   verbose - FALSE: only show errors
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

struct mtd_dev_s;
int nxffs_dump(FAR struct mtd_dev_s *mtd, bool verbose);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_NXFFS_H */
