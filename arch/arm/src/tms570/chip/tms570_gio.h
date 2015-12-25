/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_gio.h
 * Secondary System Control Register Definitions
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual, Texas
 *   Instruments, Literature Number: SPNU517A, September 2013
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_GIO_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_GIO_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/tms570_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define TMS570_GIOA                    0
#define TMS570_GIOB                    1
#define TMS570_GIOC                    2
#define TMS570_GIOD                    3
#define TMS570_GIOE                    4
#define TMS570_GIOF                    5
#define TMS570_GIOG                    6
#define TMS570_GIOH                    7

/* Register Offsets *********************************************************************************/

#define TMS570_GIO_GCR0_OFFSET         0x0000 /* GIO Global Control Register */
#define TMS570_GIO_INTDET_OFFSET       0x0008 /* GIO Interrupt Detect Register */
#define TMS570_GIO_POL_OFFSET          0x000c /* GIO Interrupt Polarity Register */
#define TMS570_GIO_ENASET_OFFSET       0x0010 /* GIO Interrupt Enable Set Register */
#define TMS570_GIO_ENACLR_OFFSET       0x0014 /* GIO Interrupt Enable Clear Register */
#define TMS570_GIO_LVLSET_OFFSET       0x0018 /* GIO Interrupt Priority Set Register */
#define TMS570_GIO_LVLCLR_OFFSET       0x001c /* GIO Interrupt Priority Clear Register */
#define TMS570_GIO_FLG_OFFSET          0x0020 /* GIO Interrupt Flag Register */
#define TMS570_GIO_OFF1_OFFSET         0x0024 /* GIO Offset 1 Register */
#define TMS570_GIO_OFF2_OFFSET         0x0028 /* GIO Offset 2 Register */
#define TMS570_GIO_EMU1_OFFSET         0x002c /* GIO Emulation 1 Register */
#define TMS570_GIO_EMU2_OFFSET         0x0030 /* GIO Emulation 2 Register */

#define TMS570_GIO_OFFSET(n)           (0x0034 + ((n) << 5))
#define TMS570_GIO_DIR_OFFSET          0x0000 /* GIO Data Direction Register */
#define TMS570_GIO_DIN_OFFSET          0x0004 /* GIO Data Input Register */
#define TMS570_GIO_DOUT_OFFSET         0x0008 /* GIO Data Output Register */
#define TMS570_GIO_DSET_OFFSET         0x000c /* GIO Data Set Register */
#define TMS570_GIO_DCLR_OFFSET         0x0010 /* GIO Data Clear Register */
#define TMS570_GIO_PDR_OFFSET          0x0014 /* GIO Open Drain Register */
#define TMS570_GIO_PULDIS_OFFSET       0x0018 /* GIO Pull Disable Register */
#define TMS570_GIO_PSL_OFFSET          0x001c /* GIO Pull Select Register */

#define TMS570_GIOA_DIR_OFFSET         0x0034 /* GIOA Data Direction Register */
#define TMS570_GIOA_DIN_OFFSET         0x0038 /* GIOA Data Input Register */
#define TMS570_GIOA_DOUT_OFFSET        0x003c /* GIOA Data Output Register */
#define TMS570_GIOA_DSET_OFFSET        0x0040 /* GIOA Data Set Register */
#define TMS570_GIOA_DCLR_OFFSET        0x0044 /* GIOA Data Clear Register */
#define TMS570_GIOA_PDR_OFFSET         0x0048 /* GIOA Open Drain Register */
#define TMS570_GIOA_PULDIS_OFFSET      0x004c /* GIOA Pull Disable Register */
#define TMS570_GIOA_PSL_OFFSET         0x0050 /* GIOA Pull Select Register */

#define TMS570_GIOB_DIR_OFFSET         0x0054 /* GIOB Data Direction Register */
#define TMS570_GIOB_DIN_OFFSET         0x0058 /* GIOB Data Input Register */
#define TMS570_GIOB_DOUT_OFFSET        0x005c /* GIOB Data Output Register */
#define TMS570_GIOB_DSET_OFFSET        0x0060 /* GIOB Data Set Register */
#define TMS570_GIOB_DCLR_OFFSET        0x0064 /* GIOB Data Clear Register */
#define TMS570_GIOB_PDR_OFFSET         0x0068 /* GIOB Open Drain Register */
#define TMS570_GIOB_PULDIS_OFFSET      0x006c /* GIOB Pull Disable Register */
#define TMS570_GIOB_PSL_OFFSET         0x0070 /* GIOB Pull Select Register */

#define TMS570_GIOC_DIR_OFFSET         0x0074 /* GIOC Data Direction Register */
#define TMS570_GIOC_DIN_OFFSET         0x0078 /* GIOC Data Input Register */
#define TMS570_GIOC_DOUT_OFFSET        0x007c /* GIOC Data Output Register */
#define TMS570_GIOC_DSET_OFFSET        0x0080 /* GIOC Data Set Register */
#define TMS570_GIOC_DCLR_OFFSET        0x0084 /* GIOC Data Clear Register */
#define TMS570_GIOC_PDR_OFFSET         0x0088 /* GIOC Open Drain Register */
#define TMS570_GIOC_PULDIS_OFFSET      0x008c /* GIOC Pull Disable Register */
#define TMS570_GIOC_PSL_OFFSET         0x0090 /* GIOC Pull Select Register */

#define TMS570_GIOD_DIR_OFFSET         0x0094 /* GIOD Data Direction Register */
#define TMS570_GIOD_DIN_OFFSET         0x0098 /* GIOD Data Input Register */
#define TMS570_GIOD_DOUT_OFFSET        0x009c /* GIOD Data Output Register */
#define TMS570_GIOD_DSET_OFFSET        0x00a0 /* GIOD Data Set Register */
#define TMS570_GIOD_DCLR_OFFSET        0x00a4 /* GIOD Data Clear Register */
#define TMS570_GIOD_PDR_OFFSET         0x00a8 /* GIOD Open Drain Register */
#define TMS570_GIOD_PULDIS_OFFSET      0x00ac /* GIOD Pull Disable Register */
#define TMS570_GIOD_PSL_OFFSET         0x00b0 /* GIOD Pull Select Register */

#define TMS570_GIOE_DIR_OFFSET         0x00b4 /* GIOE Data Direction Register */
#define TMS570_GIOE_DIN_OFFSET         0x00b8 /* GIOE Data Input Register */
#define TMS570_GIOE_DOUT_OFFSET        0x00bc /* GIOE Data Output Register */
#define TMS570_GIOE_DSET_OFFSET        0x00c0 /* GIOE Data Set Register */
#define TMS570_GIOE_DCLR_OFFSET        0x00c4 /* GIOE Data Clear Register */
#define TMS570_GIOE_PDR_OFFSET         0x00c8 /* GIOE Open Drain Register */
#define TMS570_GIOE_PULDIS_OFFSET      0x00cc /* GIOE Pull Disable Register */
#define TMS570_GIOE_PSL_OFFSET         0x00d0 /* GIOE Pull Select Register */

#define TMS570_GIOF_DIR_OFFSET         0x00d4 /* GIOF Data Direction Register */
#define TMS570_GIOF_DIN_OFFSET         0x00d8 /* GIOF Data Input Register */
#define TMS570_GIOF_DOUT_OFFSET        0x00dc /* GIOF Data Output Register */
#define TMS570_GIOF_DSET_OFFSET        0x00e0 /* GIOF Data Set Register */
#define TMS570_GIOF_DCLR_OFFSET        0x00e4 /* GIOF Data Clear Register */
#define TMS570_GIOF_PDR_OFFSET         0x00e8 /* GIOF Open Drain Register */
#define TMS570_GIOF_PULDIS_OFFSET      0x00ec /* GIOF Pull Disable Register */
#define TMS570_GIOF_PSL_OFFSET         0x00f0 /* GIOF Pull Select Register */

#define TMS570_GIOG_DIR_OFFSET         0x00f4 /* GIOG Data Direction Register */
#define TMS570_GIOG_DIN_OFFSET         0x00f8 /* GIOG Data Input Register */
#define TMS570_GIOG_DOUT_OFFSET        0x00fc /* GIOG Data Output Register */
#define TMS570_GIOG_DSET_OFFSET        0x0100 /* GIOG Data Set Register */
#define TMS570_GIOG_DCLR_OFFSET        0x0104 /* GIOG Data Clear Register */
#define TMS570_GIOG_PDR_OFFSET         0x0108 /* GIOG Open Drain Register */
#define TMS570_GIOG_PULDIS_OFFSET      0x010c /* GIOG Pull Disable Register */
#define TMS570_GIOG_PSL_OFFSET         0x0110 /* GIOG Pull Select Register */

#define TMS570_GIOH_DIR_OFFSET         0x0114 /* GIOH Data Direction Register */
#define TMS570_GIOH_DIN_OFFSET         0x0118 /* GIOH Data Input Register */
#define TMS570_GIOH_DOUT_OFFSET        0x011c /* GIOH Data Output Register */
#define TMS570_GIOH_DSET_OFFSET        0x0120 /* GIOH Data Set Register */
#define TMS570_GIOH_DCLR_OFFSET        0x0124 /* GIOH Data Clear Register */
#define TMS570_GIOH_PDR_OFFSET         0x0128 /* GIOH Open Drain Register */
#define TMS570_GIOH_PULDIS_OFFSET      0x012c /* GIOH Pull Disable Register */
#define TMS570_GIOH_PSL_OFFSET         0x0130 /* GIOH Pull Select Register */

/* Register Addresses *******************************************************************************/

#define TMS570_GIO_GCR0                (TMS570_GIO_BASE+TMS570_GIO_GCR0_OFFSET)
#define TMS570_GIO_INTDET              (TMS570_GIO_BASE+TMS570_GIO_INTDET_OFFSET)
#define TMS570_GIO_POL                 (TMS570_GIO_BASE+TMS570_GIO_POL_OFFSET)
#define TMS570_GIO_ENASET              (TMS570_GIO_BASE+TMS570_GIO_ENASET_OFFSET)
#define TMS570_GIO_ENACLR              (TMS570_GIO_BASE+TMS570_GIO_ENACLR_OFFSET)
#define TMS570_GIO_LVLSET              (TMS570_GIO_BASE+TMS570_GIO_LVLSET_OFFSET)
#define TMS570_GIO_LVLCLR              (TMS570_GIO_BASE+TMS570_GIO_LVLCLR_OFFSET)
#define TMS570_GIO_FLG                 (TMS570_GIO_BASE+TMS570_GIO_FLG_OFFSET)
#define TMS570_GIO_OFF1                (TMS570_GIO_BASE+TMS570_GIO_OFF1_OFFSET)
#define TMS570_GIO_OFF2                (TMS570_GIO_BASE+TMS570_GIO_OFF2_OFFSET)
#define TMS570_GIO_EMU1                (TMS570_GIO_BASE+TMS570_GIO_EMU1_OFFSET)
#define TMS570_GIO_EMU2                (TMS570_GIO_BASE+TMS570_GIO_EMU2_OFFSET)

#define TMS570_GIO(n)                  (TMS570_GIO_BASE+TMS570_GIO_OFFSET(n))
#define TMS570_GIO_DIR(n)              (TMS570_GIO(n)+TMS570_GIO_DIR_OFFSET)
#define TMS570_GIO_DIN(n)              (TMS570_GIO(n)+TMS570_GIO_DIN_OFFSET)
#define TMS570_GIO_DOUT(n)             (TMS570_GIO(n)+TMS570_GIO_DOUT_OFFSET)
#define TMS570_GIO_DSET(n)             (TMS570_GIO(n)+TMS570_GIO_DSET_OFFSET)
#define TMS570_GIO_DCLR(n)             (TMS570_GIO(n)+TMS570_GIO_DCLR_OFFSET)
#define TMS570_GIO_PDR(n)              (TMS570_GIO(n)+TMS570_GIO_PDR_OFFSET)
#define TMS570_GIO_PULDIS(n)           (TMS570_GIO(n)+TMS570_GIO_PULDIS_OFFSET)
#define TMS570_GIO_PSL(n)              (TMS570_GIO(n)+TMS570_GIO_PSL_OFFSET)

#define TMS570_GIOA_DIR                (TMS570_GIO_BASE+TMS570_GIOA_DIR_OFFSET)
#define TMS570_GIOA_DIN                (TMS570_GIO_BASE+TMS570_GIOA_DIN_OFFSET)
#define TMS570_GIOA_DOUT               (TMS570_GIO_BASE+TMS570_GIOA_DOUT_OFFSET)
#define TMS570_GIOA_DSET               (TMS570_GIO_BASE+TMS570_GIOA_DSET_OFFSET)
#define TMS570_GIOA_DCLR               (TMS570_GIO_BASE+TMS570_GIOA_DCLR_OFFSET)
#define TMS570_GIOA_PDR                (TMS570_GIO_BASE+TMS570_GIOA_PDR_OFFSET)
#define TMS570_GIOA_PULDIS             (TMS570_GIO_BASE+TMS570_GIOA_PULDIS_OFFSET)
#define TMS570_GIOA_PSL                (TMS570_GIO_BASE+TMS570_GIOA_PSL_OFFSET)

#define TMS570_GIOB_DIR                (TMS570_GIO_BASE+TMS570_GIOB_DIR_OFFSET)
#define TMS570_GIOB_DIN                (TMS570_GIO_BASE+TMS570_GIOB_DIN_OFFSET)
#define TMS570_GIOB_DOUT               (TMS570_GIO_BASE+TMS570_GIOB_DOUT_OFFSET)
#define TMS570_GIOB_DSET               (TMS570_GIO_BASE+TMS570_GIOB_DSET_OFFSET)
#define TMS570_GIOB_DCLR               (TMS570_GIO_BASE+TMS570_GIOB_DCLR_OFFSET)
#define TMS570_GIOB_PDR                (TMS570_GIO_BASE+TMS570_GIOB_PDR_OFFSET)
#define TMS570_GIOB_PULDIS             (TMS570_GIO_BASE+TMS570_GIOB_PULDIS_OFFSET)
#define TMS570_GIOB_PSL                (TMS570_GIO_BASE+TMS570_GIOB_PSL_OFFSET)

#define TMS570_GIOC_DIR                (TMS570_GIO_BASE+TMS570_GIOC_DIR_OFFSET)
#define TMS570_GIOC_DIN                (TMS570_GIO_BASE+TMS570_GIOC_DIN_OFFSET)
#define TMS570_GIOC_DOUT               (TMS570_GIO_BASE+TMS570_GIOC_DOUT_OFFSET)
#define TMS570_GIOC_DSET               (TMS570_GIO_BASE+TMS570_GIOC_DSET_OFFSET)
#define TMS570_GIOC_DCLR               (TMS570_GIO_BASE+TMS570_GIOC_DCLR_OFFSET)
#define TMS570_GIOC_PDR                (TMS570_GIO_BASE+TMS570_GIOC_PDR_OFFSET)
#define TMS570_GIOC_PULDIS             (TMS570_GIO_BASE+TMS570_GIOC_PULDIS_OFFSET)
#define TMS570_GIOC_PSL                (TMS570_GIO_BASE+TMS570_GIOC_PSL_OFFSET)

#define TMS570_GIOD_DIR                (TMS570_GIO_BASE+TMS570_GIOD_DIR_OFFSET)
#define TMS570_GIOD_DIN                (TMS570_GIO_BASE+TMS570_GIOD_DIN_OFFSET)
#define TMS570_GIOD_DOUT               (TMS570_GIO_BASE+TMS570_GIOD_DOUT_OFFSET)
#define TMS570_GIOD_DSET               (TMS570_GIO_BASE+TMS570_GIOD_DSET_OFFSET)
#define TMS570_GIOD_DCLR               (TMS570_GIO_BASE+TMS570_GIOD_DCLR_OFFSET)
#define TMS570_GIOD_PDR                (TMS570_GIO_BASE+TMS570_GIOD_PDR_OFFSET)
#define TMS570_GIOD_PULDIS             (TMS570_GIO_BASE+TMS570_GIOD_PULDIS_OFFSET)
#define TMS570_GIOD_PSL                (TMS570_GIO_BASE+TMS570_GIOD_PSL_OFFSET)

#define TMS570_GIOE_DIR                (TMS570_GIO_BASE+TMS570_GIOE_DIR_OFFSET)
#define TMS570_GIOE_DIN                (TMS570_GIO_BASE+TMS570_GIOE_DIN_OFFSET)
#define TMS570_GIOE_DOUT               (TMS570_GIO_BASE+TMS570_GIOE_DOUT_OFFSET)
#define TMS570_GIOE_DSET               (TMS570_GIO_BASE+TMS570_GIOE_DSET_OFFSET)
#define TMS570_GIOE_DCLR               (TMS570_GIO_BASE+TMS570_GIOE_DCLR_OFFSET)
#define TMS570_GIOE_PDR                (TMS570_GIO_BASE+TMS570_GIOE_PDR_OFFSET)
#define TMS570_GIOE_PULDIS             (TMS570_GIO_BASE+TMS570_GIOE_PULDIS_OFFSET)
#define TMS570_GIOE_PSL                (TMS570_GIO_BASE+TMS570_GIOE_PSL_OFFSET)

#define TMS570_GIOF_DIR                (TMS570_GIO_BASE+TMS570_GIOF_DIR_OFFSET)
#define TMS570_GIOF_DIN                (TMS570_GIO_BASE+TMS570_GIOF_DIN_OFFSET)
#define TMS570_GIOF_DOUT               (TMS570_GIO_BASE+TMS570_GIOF_DOUT_OFFSET)
#define TMS570_GIOF_DSET               (TMS570_GIO_BASE+TMS570_GIOF_DSET_OFFSET)
#define TMS570_GIOF_DCLR               (TMS570_GIO_BASE+TMS570_GIOF_DCLR_OFFSET)
#define TMS570_GIOF_PDR                (TMS570_GIO_BASE+TMS570_GIOF_PDR_OFFSET)
#define TMS570_GIOF_PULDIS             (TMS570_GIO_BASE+TMS570_GIOF_PULDIS_OFFSET)
#define TMS570_GIOF_PSL                (TMS570_GIO_BASE+TMS570_GIOF_PSL_OFFSET)

#define TMS570_GIOG_DIR                (TMS570_GIO_BASE+TMS570_GIOG_DIR_OFFSET)
#define TMS570_GIOG_DIN                (TMS570_GIO_BASE+TMS570_GIOG_DIN_OFFSET)
#define TMS570_GIOG_DOUT               (TMS570_GIO_BASE+TMS570_GIOG_DOUT_OFFSET)
#define TMS570_GIOG_DSET               (TMS570_GIO_BASE+TMS570_GIOG_DSET_OFFSET)
#define TMS570_GIOG_DCLR               (TMS570_GIO_BASE+TMS570_GIOG_DCLR_OFFSET)
#define TMS570_GIOG_PDR                (TMS570_GIO_BASE+TMS570_GIOG_PDR_OFFSET)
#define TMS570_GIOG_PULDIS             (TMS570_GIO_BASE+TMS570_GIOG_PULDIS_OFFSET)
#define TMS570_GIOG_PSL                (TMS570_GIO_BASE+TMS570_GIOG_PSL_OFFSET)

#define TMS570_GIOH_DIR                (TMS570_GIO_BASE+TMS570_GIOH_DIR_OFFSET)
#define TMS570_GIOH_DIN                (TMS570_GIO_BASE+TMS570_GIOH_DIN_OFFSET)
#define TMS570_GIOH_DOUT               (TMS570_GIO_BASE+TMS570_GIOH_DOUT_OFFSET)
#define TMS570_GIOH_DSET               (TMS570_GIO_BASE+TMS570_GIOH_DSET_OFFSET)
#define TMS570_GIOH_DCLR               (TMS570_GIO_BASE+TMS570_GIOH_DCLR_OFFSET)
#define TMS570_GIOH_PDR                (TMS570_GIO_BASE+TMS570_GIOH_PDR_OFFSET)
#define TMS570_GIOH_PULDIS             (TMS570_GIO_BASE+TMS570_GIOH_PULDIS_OFFSET)
#define TMS570_GIOH_PSL                (TMS570_GIO_BASE+TMS570_GIOH_PSL_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/

/* GIO Global Control Register */
#define GIO_GCR0_
/* GIO Interrupt Detect Register */
#define GIO_INTDET_
/* GIO Interrupt Polarity Register */
#define GIO_POL_
/* GIO Interrupt Enable Set Register */
#define GIO_ENASET_
/* GIO Interrupt Enable Clear Register */
#define GIO_ENACLR_
/* GIO Interrupt Priority Set Register */
#define GIO_LVLSET_
/* GIO Interrupt Priority Clear Register */
#define GIO_LVLCLR_
/* GIO Interrupt Flag Register */
#define GIO_FLG_
/* GIO Offset 1 Register */
#define GIO_OFF1_
/* GIO Offset 2 Register */
#define GIO_OFF2_
/* GIO Emulation 1 Register */
#define GIO_EMU1_
/* GIO Emulation 2 Register */
#define GIO_EMU2_

/* GIO Data Direction Register */
#define GIO_DIR_
/* GIO Data Input Register */
#define GIO_DIN_
/* GIO Data Output Register */
#define GIO_DOUT_
/* GIO Data Set Register */
#define GIO_DSET_
/* GIO Data Clear Register */
#define GIO_DCLR_
/* GIO Open Drain Register */
#define GIO_PDR_
/* GIO Pull Disable Register */
#define GIO_PULDIS_
/* GIO Pull Select Register */
#define GIO_PSL_

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_GIO_H */
