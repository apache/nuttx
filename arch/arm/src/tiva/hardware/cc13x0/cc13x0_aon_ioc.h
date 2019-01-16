/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_aon_ioc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
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
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_IOC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_IOC_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* AON IOC Register Offsets *****************************************************************************************/

#define TIVA_AON_IOC_IOSTRMIN_OFFSET      0x0000
#define TIVA_AON_IOC_IOSTRMED_OFFSET      0x0004
#define TIVA_AON_IOC_IOSTRMAX_OFFSET      0x0008
#define TIVA_AON_IOC_IOCLATCH_OFFSET      0x000c  /* IO Latch Control */
#define TIVA_AON_IOC_CLK32KCTL_OFFSET     0x0010  /* SCLK_LF External Output Control */
#define TIVA_AON_IOC_TCKCTL_OFFSET        0x0014  /* TCK IO Pin Control */

/* AON IOC Register Addresses ***************************************************************************************/

#define TIVA_AON_IOC_IOSTRMIN             (TIVA_AON_IOC_BASE + TIVA_AON_IOC_IOSTRMIN_OFFSET)
#define TIVA_AON_IOC_IOSTRMED             (TIVA_AON_IOC_BASE + TIVA_AON_IOC_IOSTRMED_OFFSET)
#define TIVA_AON_IOC_IOSTRMAX             (TIVA_AON_IOC_BASE + TIVA_AON_IOC_IOSTRMAX_OFFSET)
#define TIVA_AON_IOC_IOCLATCH             (TIVA_AON_IOC_BASE + TIVA_AON_IOC_IOCLATCH_OFFSET)
#define TIVA_AON_IOC_CLK32KCTL            (TIVA_AON_IOC_BASE + TIVA_AON_IOC_CLK32KCTL_OFFSET)

/* AON IOC Bitfield Definitions *************************************************************************************/

/* TIVA_AON_IOC_IOSTRMIN */

#define AON_IOC_IOSTRMIN_GRAY_CODE_SHIFT  (0)       /* Bits 0-2: Gray code */
#define AON_IOC_IOSTRMIN_GRAY_CODE_MASK   (7 << AON_IOC_IOSTRMIN_GRAY_CODE_SHIFT)
#  define AON_IOC_IOSTRMIN_GRAY_CODE(n)   ((uint32_t)(n) << xx)

/* TIVA_AON_IOC_IOSTRMED */

#define AON_IOC_IOSTRMED_GRAY_CODE_SHIFT  (0)       /* Bits 0-2: Gray code */
#define AON_IOC_IOSTRMED_GRAY_CODE_MASK   (7 << AON_IOC_IOSTRMED_GRAY_CODE_SHIFT)
#  define AON_IOC_IOSTRMED_GRAY_CODE(n)   ((uint32_t)(n) << xx)

/* TIVA_AON_IOC_IOSTRMAX */

#define AON_IOC_IOSTRMAX_GRAY_CODE_SHIFT  (0)       /* Bits 0-2: Gray code */
#define AON_IOC_IOSTRMAX_GRAY_CODE_MASK   (7 << AON_IOC_IOSTRMAX_GRAY_CODE_SHIFT)
#  define AON_IOC_IOSTRMAX_GRAY_CODE(n)   ((uint32_t)(n) << xx)

/* TIVA_AON_IOC_IOCLATCH */

#define AON_IOC_IOCLATCH_EN               (1 << 0)  /* Bit 0: Controls latches between MCU IOC and AON_IOC */
#  define AON_IOC_IOCLATCH_EN_STATIC      (0)
#  define AON_IOC_IOCLATCH_EN_TRANSP      AON_IOC_IOCLATCH_EN

/* TIVA_AON_IOC_CLK32KCTL */

#define AON_IOC_CLK32KCTL_OE_N            (1 << 0)  /* Bit 0: Output enable */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_IOC_H */
