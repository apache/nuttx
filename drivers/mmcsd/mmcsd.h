/****************************************************************************
 * drivers/mmcsd/mmcsd.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __DRIVERS_MMCSD_MMCSD_H
#define __DRIVERS_MMCSD_MMCSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>
#include <stdint.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enable excessive debug options */

#undef CONFIG_MMCSD_DUMPALL /* MUST BE DEFINED MANUALLY */

#if !defined(CONFIG_DEBUG_INFO) || !defined(CONFIG_DEBUG_FS)
#  undef CONFIG_MMCSD_DUMPALL
#endif

#define MMCSD_PART_COUNT             8

/* Card type */

#define MMCSD_CARDTYPE_UNKNOWN       0  /* Unknown card type */
#define MMCSD_CARDTYPE_MMC           1  /* Bit 0: MMC card */
#define MMCSD_CARDTYPE_SDV1          2  /* Bit 1: SD version 1.x */
#define MMCSD_CARDTYPE_SDV2          4  /* Bit 2: SD version 2.x with byte addressing */
#define MMCSD_CARDTYPE_BLOCK         8  /* Bit 3: SD version 2.x or MMC with block addressing */

#define IS_MMC(t)   (((t) & MMCSD_CARDTYPE_MMC) != 0)
#define IS_SD(t)    (((t) & (MMCSD_CARDTYPE_SDV1|MMCSD_CARDTYPE_SDV2)) != 0)
#define IS_SDV1(t)  (((t) & MMCSD_CARDTYPE_SDV1) != 0)
#define IS_SDV2(t)  (((t) & MMCSD_CARDTYPE_SDV2) != 0)
#define IS_BLOCK(t) (((t) & MMCSD_CARDTYPE_BLOCK) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mmcsd_part_s
{
  FAR struct mmcsd_state_s *priv;
  blkcnt_t nblocks; /* Number of blocks */
};

/* This structure is contains the unique state of the MMC/SD block driver */

struct mmcsd_state_s
{
  FAR struct sdio_dev_s *dev;                  /* The SDIO device bound to this instance */
  uint8_t  crefs;                              /* Open references on the driver */
  mutex_t  lock;                               /* Assures mutually exclusive access to the slot */
  int      minor;                              /* Device number */
  struct mmcsd_part_s part[MMCSD_PART_COUNT];  /* Partition data */
  uint32_t partnum;                            /* Partition number */

  /* Status flags */

  uint8_t probed:1;                /* true: mmcsd_probe() discovered a card */
  uint8_t widebus:1;               /* true: Wide 4-bit bus selected */
  uint8_t mediachanged:1;          /* true: Media changed since last check */
  uint8_t wrbusy:1;                /* true: Last transfer was a write, card may be busy */
  uint8_t wrprotect:1;             /* true: Card is write protected (from CSD) */
  uint8_t locked:1;                /* true: Media is locked (from R1) */
  uint8_t dsrimp:1;                /* true: card supports CMD4/DSR setting (from CSD) */
#ifdef CONFIG_SDIO_DMA
  uint8_t dma:1;                   /* true: hardware supports DMA */
#endif

  uint8_t mode:4;                  /* (See MMCSDMODE_* definitions) */
  uint8_t type:4;                  /* Card type (See MMCSD_CARDTYPE_* definitions) */
  uint8_t buswidth:4;              /* Bus widths supported (SD only) */
  uint8_t cmd23support:1;          /* CMD23 supported (SD only) */
  sdio_capset_t caps;              /* SDIO driver capabilities/limitations */
  uint32_t cid[4];                 /* CID register */
  uint32_t csd[4];                 /* CSD register */
  uint16_t selblocklen;            /* The currently selected block length */
  uint16_t rca;                    /* Relative Card Address (RCS) register */

  /* Memory card geometry (extracted from the CSD) */

  uint8_t  blockshift;             /* Log2 of blocksize */
  uint16_t blocksize;              /* Read block length (== block size) */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_MMCSD_PROCFS
void mmcsd_initialize_procfs(void);
#endif

#ifdef CONFIG_MMCSD_DUMPALL
#  define mmcsd_dumpbuffer(m,b,l) finfodumpbuffer(m,b,l)
#else
#  define mmcsd_dumpbuffer(m,b,l)
#endif

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
EXTERN void mmcsd_dmpcsd(FAR const uint8_t *csd, uint8_t cardtype);
#else
#  define mmcsd_dmpcsd(csd,cadtype)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_H */
