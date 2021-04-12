/****************************************************************************
 * include/nuttx/mtd/smart.h
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

#ifndef __INCLUDE_NUTTX_MTD_SMART_H
#define __INCLUDE_NUTTX_MTD_SMART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SMART_DEBUG_CMD_SET_DEBUG_LEVEL   1
#define SMART_DEBUG_CMD_SHOW_LOGMAP       2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure to track SMART allocations for debugging */

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
struct smart_alloc_s
{
  size_t                size;             /* Size of the allocation */
  void                 *ptr;              /* Pointer to the data */
  const char           *name;             /* Name of the allocation */
};
#endif

/* The following defines the procfs data passed from the SMART MTD layer
 * from a BIOC_GETPROCFSD ioctl command.
 */

struct mtd_smart_procfs_data_s
{
  uint16_t            totalsectors;     /* Total number of sectors on device */
  uint16_t            sectorsize;       /* Size of each sector */
  uint16_t            freesectors;      /* Number of free sectors */
  uint16_t            releasesectors;   /* Number of released sectors */
  uint16_t            sectorsperblk;    /* Number of sectors per erase block */
  uint16_t            formatsector;     /* Physical sector number for sector 0 */
  uint16_t            dirsector;        /* Physical sector number for sector 1 */
  uint8_t             namelen;          /* Length of names on the volume */
  uint8_t             formatversion;    /* Version of the volume format */
  uint32_t            unusedsectors;    /* Number of unused sectors (free when erased) */
  uint32_t            blockerases;      /* Number block erase operations */

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  FAR const uint8_t  *erasecounts;      /* Array of erase counts per erase block */
  size_t              neraseblocks;     /* Number of erase blocks */
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  FAR const struct smart_alloc_s  *allocs; /* Array of allocations */
  uint16_t            alloccount;          /* Number of items in the array */
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  uint32_t            uneven_wearcount; /* Number of uneven block erases */
#endif
};

/* The following defines debug command data passed from the procfs layer to
 *  the SMART MTD layer for debug purposes.
 */

struct mtd_smart_debug_data_s
{
  uint8_t             debugcmd;         /* Debug command */
  uint32_t            debugdata;        /* Debug data */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_MTD_SMART_H */
