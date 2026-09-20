/*****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_caam.h
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_CAAM_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_CAAM_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include "hardware/imxrt_memorymap.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* The job rings are separate 16 KB pages above the general block. Only ring
 * zero is used here; one ring answers one request at a time, which is all a
 * random number generator needs.
 */

#define IMXRT_CAAM_GEN_BASE           0x40440000
#define IMXRT_CAAM_JR0_BASE           0x40450000

/* General block */

#define IMXRT_CAAM_MCFGR_OFFSET       0x0004
#define IMXRT_CAAM_SCFGR_OFFSET       0x000c
#define IMXRT_CAAM_JRSTART_OFFSET     0x005c

/* RNG4 block, inside the general block */

#define IMXRT_CAAM_RTMCTL_OFFSET      0x0600
#define IMXRT_CAAM_RTSDCTL_OFFSET     0x0610
#define IMXRT_CAAM_RTFRQMIN_OFFSET    0x0618
#define IMXRT_CAAM_RTFRQMAX_OFFSET    0x061c
#define IMXRT_CAAM_RDSTA_OFFSET       0x06c0

#define IMXRT_CAAM_MCFGR              (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_MCFGR_OFFSET)
#define IMXRT_CAAM_SCFGR              (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_SCFGR_OFFSET)
#define IMXRT_CAAM_JRSTART            (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_JRSTART_OFFSET)
#define IMXRT_CAAM_RTMCTL             (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_RTMCTL_OFFSET)
#define IMXRT_CAAM_RTSDCTL            (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_RTSDCTL_OFFSET)
#define IMXRT_CAAM_RTFRQMIN           (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_RTFRQMIN_OFFSET)
#define IMXRT_CAAM_RTFRQMAX           (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_RTFRQMAX_OFFSET)
#define IMXRT_CAAM_RDSTA              (IMXRT_CAAM_GEN_BASE + IMXRT_CAAM_RDSTA_OFFSET)

/* Job ring zero */

#define IMXRT_CAAM_IRBA_H_OFFSET      0x0000
#define IMXRT_CAAM_IRBA_L_OFFSET      0x0004
#define IMXRT_CAAM_IRS_OFFSET         0x000c
#define IMXRT_CAAM_IRSA_OFFSET        0x0014
#define IMXRT_CAAM_IRJA_OFFSET        0x001c
#define IMXRT_CAAM_ORBA_H_OFFSET      0x0020
#define IMXRT_CAAM_ORBA_L_OFFSET      0x0024
#define IMXRT_CAAM_ORS_OFFSET         0x002c
#define IMXRT_CAAM_ORJR_OFFSET        0x0034
#define IMXRT_CAAM_ORSF_OFFSET        0x003c
#define IMXRT_CAAM_JRSTA_OFFSET       0x0044
#define IMXRT_CAAM_JRINT_OFFSET       0x004c
#define IMXRT_CAAM_JRCFG1_OFFSET      0x0054
#define IMXRT_CAAM_JRCR_OFFSET        0x006c

#define IMXRT_CAAM_IRBA_H             (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_IRBA_H_OFFSET)
#define IMXRT_CAAM_IRBA_L             (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_IRBA_L_OFFSET)
#define IMXRT_CAAM_IRS                (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_IRS_OFFSET)
#define IMXRT_CAAM_IRSA               (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_IRSA_OFFSET)
#define IMXRT_CAAM_IRJA               (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_IRJA_OFFSET)
#define IMXRT_CAAM_ORBA_H             (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_ORBA_H_OFFSET)
#define IMXRT_CAAM_ORBA_L             (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_ORBA_L_OFFSET)
#define IMXRT_CAAM_ORS                (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_ORS_OFFSET)
#define IMXRT_CAAM_ORJR               (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_ORJR_OFFSET)
#define IMXRT_CAAM_ORSF               (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_ORSF_OFFSET)
#define IMXRT_CAAM_JRSTA              (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_JRSTA_OFFSET)
#define IMXRT_CAAM_JRINT              (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_JRINT_OFFSET)
#define IMXRT_CAAM_JRCFG1             (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_JRCFG1_OFFSET)
#define IMXRT_CAAM_JRCR               (IMXRT_CAAM_JR0_BASE + IMXRT_CAAM_JRCR_OFFSET)

/* MCFGR */

#define CAAM_MCFGR_SWRST              (1 << 31)  /* Software reset */
#define CAAM_MCFGR_DMA_RST            (1 << 28)  /* DMA reset */
#define CAAM_MCFGR_WDE                (1 << 30)  /* DECO watchdog enable */
#define CAAM_MCFGR_LARGE_BURST        (1 << 2)   /* 128/256-byte bursts */
#define CAAM_MCFGR_AWCACHE_MASK       (0xf << 8)
#define CAAM_MCFGR_AWCACHE_BUFF       (0x1 << 8)
#define CAAM_MCFGR_AWCACHE_CACH       (0x2 << 8)

#define CAAM_JRSTART_JR0              (1 << 0)   /* Start job ring zero */

/* JRCR, JRINT, JRCFG1 */

#define CAAM_JRCR_RESET               (1 << 0)
#define CAAM_JRINT_ERR_HALT_MASK      (3 << 2)
#define CAAM_JRINT_ERR_HALT_INPROG    (1 << 2)
#define CAAM_JRCFG1_IMSK              (1 << 0)   /* Mask the ring interrupt */

/* RTMCTL, RTSDCTL, RDSTA */

#define CAAM_RTMCTL_PRGM              (1 << 16)  /* Program, not run, mode */
#define CAAM_RTMCTL_ERR               (1 << 12)
#define CAAM_RTSDCTL_ENT_DLY_SHIFT    (16)
#define CAAM_RTSDCTL_ENT_DLY_MASK     (0xffff << CAAM_RTSDCTL_ENT_DLY_SHIFT)
#define CAAM_RTSDCTL_SAMP_SIZE_MASK   (0xffff)

#define CAAM_RDSTA_IF0                (1 << 0)   /* State handle 0 instantiated */
#define CAAM_RDSTA_PR0                (1 << 4)   /* With prediction resistance */
#define CAAM_RDSTA_SKVN               (1 << 30)  /* Secure keys already made */
#define CAAM_RDSTA_ERRMASK            (3 << 16)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_CAAM_H */
