/****************************************************************************
 * include/nuttx/mtd/smart.h
 * Memory Technology Device (MTD) SMART specific interfaces
 *
 *   Copyright (C) 2014 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef __INCLUDE_NUTTX_MTD_SMART_H
#define __INCLUDE_NUTTX_MTD_SMART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
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
  void*                 ptr;              /* Pointer to the data */
  const char*           name;             /* Name of the allocation */
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
  FAR const uint8_t*  erasecounts;      /* Array of erase counts per erase block */
  size_t              neraseblocks;     /* Number of erase blocks */
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  FAR const struct smart_alloc_s  *allocs; /* Array of allocations */ 
  uint16_t            alloccount;       /* Number of items in the array */
#endif
#ifdef CONFIG_MTD_SMART_WEAR_LEVEL
  uint32_t            uneven_wearcount; /* Number of uneven block erases */
#endif
};

/* The following defines debug command data passed from the procfs layer to
   the SMART MTD layer for debug purposes.
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
