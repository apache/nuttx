/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_vims.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a
 * compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_VIMS_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_VIMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VIMS Register Offsets ****************************************************/

#define TIVA_VIMS_STAT_OFFSET           0x0000  /* Status */
#define TIVA_VIMS_CTL_OFFSET            0x0004  /* Control */

/* VIMS Register Addresses **************************************************/

#define TIVA_VIMS_STAT                  (TIVA_VIMS_BASE + TIVA_VIMS_STAT_OFFSET)
#define TIVA_VIMS_CTL                   (TIVA_VIMS_BASE + TIVA_VIMS_CTL_OFFSET)

/* VIMS Bitfield Definitions ************************************************/

/* VIMS_STAT */

#define VIMS_STAT_MODE_SHIFT            (0)       /* Bits 0-1:  Current VMS mode */
#define VIMS_STAT_MODE_MASK             (3 << VIMS_STAT_MODE_SHIFT)
#  define VIMS_STAT_MODE_GPRAM          (0 << VIMS_STAT_MODE_SHIFT) /* VIMS GPRAM mode */
#  define VIMS_STAT_MODE_CACHE          (1 << VIMS_STAT_MODE_SHIFT) /* VIMS Cache mode */
#  define VIMS_STAT_MODE_OFF            (3 << VIMS_STAT_MODE_SHIFT) /* VIMS Off mode */

#define VIMS_STAT_INV                   (1 << 2)  /* Bit 2:  Invalidation of caching memory in-progress */
#define VIMS_STAT_MODE_CHANGING         (1 << 3)  /* Bit 3:  VIMS mode change status */
#define VIMS_STAT_SYSBUS_LB_DIS         (1 << 4)  /* Bit 4:  Sysbus flash line buffer control */
#define VIMS_STAT_IDCODE_LB_DIS         (1 << 5)  /* Bit 5:  Icode/Dcode flash line buffer status */

/* VIMS_CTL */

#define VIMS_CTL_MODE_SHIFT             (0)       /* Bits 0-1:  Current VMS mode */
#define VIMS_CTL_MODE_MASK              (3 << VIMS_CTL_MODE_SHIFT)
#  define VIMS_CTL_MODE_GPRAM           (0 << VIMS_CTL_MODE_SHIFT) /* VIMS GPRAM mode */
#  define VIMS_CTL_MODE_CACHE           (1 << VIMS_CTL_MODE_SHIFT) /* VIMS Cache mode */
#  define VIMS_CTL_MODE_OFF             (3 << VIMS_CTL_MODE_SHIFT) /* VIMS Off mode */

#define VIMS_CTL_PREF_EN                (1 << 2)  /* Bit 2:  Tag prefetch control */
#define VIMS_CTL_ARB_CFG                (1 << 3)  /* Bit 3:  Icode/Dcode and sysbus arbitation scheme */
#  define VIMS_CTL_ARB_STATIC           (0)
#  define VIMS_CTL_ARB_ROUNDROBIN       VIMS_CTL_ARB_CFG
#define VIMS_CTL_SYSBUS_LB_DIS          (1 << 4)  /* Bit 4:  Sysbus flash line buffer control */
#define VIMS_CTL_IDCODE_LB_DIS          (1 << 5)  /* Bit 5:  Icode/Dcode flash line buffer control */
#define VIMS_CTL_DYN_CG_EN              (1 << 29) /* Bit 29: Enable in-built clock gate */
#define VIMS_CTL_STATS_EN               (1 << 30) /* Bit 30: Enable statistic counters */
#define VIMS_CTL_STATS_CLR              (1 << 31) /* Bit 31: Clear statistics counters */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_VIMS_H */
