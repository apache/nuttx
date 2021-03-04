/****************************************************************************
 * drivers/mmcsd/mmcsd.h
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
