/************************************************************************************
 * arch/arm/src/stm32l4/chip/stm32l4_dfsdm.h
 *
 *   Copyright (C) 2017-2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4_DFSDM_H
#define __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4_DFSDM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/
/* Register Offsets for the DFSDM. */

/* DFSDM channel y registers (y=0..7 or y=0..3 on STM32L4X3) */

#define STM32L4_DFSDM_CHCFGR1_OFFSET(y)    (0x00 + 0x20 * (y))

#define STM32L4_DFSDM_CH0CFGR1_OFFSET      0x0000  /* DFSDM channel configuration 0 register */
#define STM32L4_DFSDM_CH1CFGR1_OFFSET      0x0020  /* DFSDM channel configuration 1 register */
#define STM32L4_DFSDM_CH2CFGR1_OFFSET      0x0040  /* DFSDM channel configuration 2 register */
#define STM32L4_DFSDM_CH3CFGR1_OFFSET      0x0060  /* DFSDM channel configuration 3 register */
#ifndef CONFIG_STM32L4_STM32L4X3
#  define STM32L4_DFSDM_CH4CFGR1_OFFSET    0x0080  /* DFSDM channel configuration 4 register */
#  define STM32L4_DFSDM_CH5CFGR1_OFFSET    0x00a0  /* DFSDM channel configuration 5 register */
#  define STM32L4_DFSDM_CH6CFGR1_OFFSET    0x00c0  /* DFSDM channel configuration 6 register */
#  define STM32L4_DFSDM_CH7CFGR1_OFFSET    0x00e0  /* DFSDM channel configuration 7 register */
#endif

#define STM32L4_DFSDM_CHCFGR2_OFFSET(y)    (0x04 + 0x20 * (y))

#define STM32L4_DFSDM_CH0CFGR2_OFFSET      0x0004  /* DFSDM channel configuration 0 register 2 */
#define STM32L4_DFSDM_CH1CFGR2_OFFSET      0x0024  /* DFSDM channel configuration 1 register 2 */
#define STM32L4_DFSDM_CH2CFGR2_OFFSET      0x0044  /* DFSDM channel configuration 2 register 2 */
#define STM32L4_DFSDM_CH3CFGR2_OFFSET      0x0064  /* DFSDM channel configuration 3 register 2 */
#ifndef CONFIG_STM32L4_STM32L4X3
#  define STM32L4_DFSDM_CH4CFGR2_OFFSET    0x0084  /* DFSDM channel configuration 4 register 2 */
#  define STM32L4_DFSDM_CH5CFGR2_OFFSET    0x00a4  /* DFSDM channel configuration 5 register 2 */
#  define STM32L4_DFSDM_CH6CFGR2_OFFSET    0x00c4  /* DFSDM channel configuration 6 register 2 */
#  define STM32L4_DFSDM_CH7CFGR2_OFFSET    0x00e4  /* DFSDM channel configuration 7 register 2 */
#endif

#define STM32L4_DFSDM_CHAWSCDR_OFFSET(y)   (0x08 + 0x20 * (y))

#define STM32L4_DFSDM_CH0AWSCDR_OFFSET     0x0008 /* DFSDM channel 0 analog watchdog and short-circuit detector register */
#define STM32L4_DFSDM_CH1AWSCDR_OFFSET     0x0028 /* DFSDM channel 1 analog watchdog and short-circuit detector register */
#define STM32L4_DFSDM_CH2AWSCDR_OFFSET     0x0048 /* DFSDM channel 2 analog watchdog and short-circuit detector register */
#define STM32L4_DFSDM_CH3AWSCDR_OFFSET     0x0068 /* DFSDM channel 3 analog watchdog and short-circuit detector register */
#ifndef CONFIG_STM32L4_STM32L4X3
#  define STM32L4_DFSDM_CH4AWSCDR_OFFSET   0x0088 /* DFSDM channel 4 analog watchdog and short-circuit detector register */
#  define STM32L4_DFSDM_CH5AWSCDR_OFFSET   0x00a8 /* DFSDM channel 5 analog watchdog and short-circuit detector register */
#  define STM32L4_DFSDM_CH6AWSCDR_OFFSET   0x00c8 /* DFSDM channel 6 analog watchdog and short-circuit detector register */
#  define STM32L4_DFSDM_CH7AWSCDR_OFFSET   0x00e8 /* DFSDM channel 7 analog watchdog and short-circuit detector register */
#endif

#define STM32L4_DFSDM_CHWDATR_OFFSET(y)    (0x0c + 0x20 * (y))

#define STM32L4_DFSDM_CH0WDATR_OFFSET      0x000c  /* DFSDM channel 0 watchdog filter data register */
#define STM32L4_DFSDM_CH1WDATR_OFFSET      0x002c  /* DFSDM channel 1 watchdog filter data register */
#define STM32L4_DFSDM_CH2WDATR_OFFSET      0x004c  /* DFSDM channel 2 watchdog filter data register */
#define STM32L4_DFSDM_CH3WDATR_OFFSET      0x006c  /* DFSDM channel 3 watchdog filter data register */
#ifndef CONFIG_STM32L4_STM32L4X3
  #define STM32L4_DFSDM_CH4WDATR_OFFSET    0x008c  /* DFSDM channel 4 watchdog filter data register */
  #define STM32L4_DFSDM_CH5WDATR_OFFSET    0x00ac  /* DFSDM channel 5 watchdog filter data register */
  #define STM32L4_DFSDM_CH6WDATR_OFFSET    0x00cc  /* DFSDM channel 6 watchdog filter data register */
  #define STM32L4_DFSDM_CH7WDATR_OFFSET    0x00ec  /* DFSDM channel 7 watchdog filter data register */
#endif

#define STM32L4_DFSDM_CHDATINR_OFFSET(ch)  (0x10 + 0x20 * (ch)) /* DFSDM channel data input register */

#define STM32L4_DFSDM_CH0DATINR_OFFSET     0x0010  /* DFSDM channel 0 data input register  */
#define STM32L4_DFSDM_CH1DATINR_OFFSET     0x0030  /* DFSDM channel 1 data input register  */
#define STM32L4_DFSDM_CH2DATINR_OFFSET     0x0050  /* DFSDM channel 2 data input register  */
#define STM32L4_DFSDM_CH3DATINR_OFFSET     0x0070  /* DFSDM channel 3 data input register  */
#ifndef CONFIG_STM32L4_STM32L4X3
#  define STM32L4_DFSDM_CH4DATINR_OFFSET   0x0090  /* DFSDM channel 4 data input register  */
#  define STM32L4_DFSDM_CH5DATINR_OFFSET   0x00b0  /* DFSDM channel 5 data input register  */
#  define STM32L4_DFSDM_CH6DATINR_OFFSET   0x00d0  /* DFSDM channel 6 data input register  */
#  define STM32L4_DFSDM_CH7DATINR_OFFSET   0x00f0  /* DFSDM channel 7 data input register  */
#endif

#ifdef CONFIG_STM32L4_STM32L4XR
#  define STM32L4_DFSDM_CHDLYR_OFFSET(ch)  (0x14 + 0x20 * (ch)) /* DFSDM channel delay register */

#  define STM32L4_DFSDM_CH0DLYR_OFFSET     0x0014  /* DFSDM channel 0 delay register  */
#  define STM32L4_DFSDM_CH1DLYR_OFFSET     0x0034  /* DFSDM channel 1 delay register  */
#  define STM32L4_DFSDM_CH2DLYR_OFFSET     0x0054  /* DFSDM channel 2 delay register  */
#  define STM32L4_DFSDM_CH3DLYR_OFFSET     0x0074  /* DFSDM channel 3 delay register  */
#  define STM32L4_DFSDM_CH4DLYR_OFFSET     0x0094  /* DFSDM channel 4 delay register  */
#  define STM32L4_DFSDM_CH5DLYR_OFFSET     0x00b4  /* DFSDM channel 5 delay register  */
#  define STM32L4_DFSDM_CH6DLYR_OFFSET     0x00d4  /* DFSDM channel 6 delay register  */
#  define STM32L4_DFSDM_CH7DLYR_OFFSET     0x00f4  /* DFSDM channel 7 delay register  */
#endif

/* DFSDM filter x module registers (x=0..3 or x=0..1 on STM32L4X3) */

#define STM32L4_DFSDM_FLTCR1_OFFSET(x)     (0x100 + 0x80 * (x)) /* DFSDM control register 1 */
#define STM32L4_DFSDM_FLTCR2_OFFSET(x)     (0x104 + 0x80 * (x)) /* DFSDM control register 2 */
#define STM32L4_DFSDM_FLTISR_OFFSET(x)     (0x108 + 0x80 * (x)) /* DFSDM interrupt and status register */
#define STM32L4_DFSDM_FLTICR_OFFSET(x)     (0x10c + 0x80 * (x)) /* DFSDM interrupt flag clear register */
#define STM32L4_DFSDM_FLTJCHGR_OFFSET(x)   (0x110 + 0x80 * (x)) /* DFSDM injected channel group selection register */
#define STM32L4_DFSDM_FLTFCR_OFFSET(x)     (0x114 + 0x80 * (x)) /* DFSDM filter control register */
#define STM32L4_DFSDM_FLTJDATAR_OFFSET(x)  (0x118 + 0x80 * (x)) /* DFSDM data register for injected group */
#define STM32L4_DFSDM_FLTRDATAR_OFFSET(x)  (0x11c + 0x80 * (x)) /* DFSDM data register for the regular channel */
#define STM32L4_DFSDM_FLTAWHTR_OFFSET(x)   (0x120 + 0x80 * (x)) /* DFSDM analog watchdog high threshold register */
#define STM32L4_DFSDM_FLTAWLTR_OFFSET(x)   (0x124 + 0x80 * (x)) /* DFSDM analog watchdog low threshold register */
#define STM32L4_DFSDM_FLTAWSR_OFFSET(x)    (0x128 + 0x80 * (x)) /* DFSDM analog watchdog status register */
#define STM32L4_DFSDM_FLTAWCFR_OFFSET(x)   (0x12c + 0x80 * (x)) /* DFSDM analog watchdog clear flag register */
#define STM32L4_DFSDM_FLTEXMAX_OFFSET(x)   (0x130 + 0x80 * (x)) /* DFSDM Extremes detector maximum register */
#define STM32L4_DFSDM_FLTEXMIN_OFFSET(x)   (0x134 + 0x80 * (x)) /* DFSDM Extremes detector minimum register */
#define STM32L4_DFSDM_FLTCNVTIMR_OFFSET(x) (0x138 + 0x80 * (x)) /* DFSDM conversion timer register */

/* Register Addresses ***************************************************************/

/* DFSDM channel y registers (y=0..7 or y=0..3 on STM32L4X3) */

#define STM32L4_DFSDM_CHCFGR1(y)         (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CHCFGR1_OFFSET(y))
#define STM32L4_DFSDM_CH0CFGR1           (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CH0CFGR1_OFFSET)

#define STM32L4_DFSDM_CHCFGR2(y)         (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CHCFGR2_OFFSET(y))
#define STM32L4_DFSDM_CHAWSCDR(y)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CHAWSCDR_OFFSET(y))
#define STM32L4_DFSDM_CHWDATR(y)         (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CHWDATR_OFFSET(y)
#define STM32L4_DFSDM_CHDATINR(y)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CHDATINR_OFFSET(y))
#ifdef CONFIG_STM32L4_STM32L4XR
#  define STM32L4_DFSDM_CHDLYR(y)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_CHDLYR_OFFSET(y))
#endif

/* DFSDM filter x module registers (x=0..3 or x=0..1 on STM32L4X3) */

#define STM32L4_DFSDM_FLTCR1(x)          (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTCR1_OFFSET(x))
#define STM32L4_DFSDM_FLTCR2(x)          (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTCR2_OFFSET(x))
#define STM32L4_DFSDM_FLTISR(x)          (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTISR_OFFSET(x))
#define STM32L4_DFSDM_FLTICR(x)          (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTICR_OFFSET(x))
#define STM32L4_DFSDM_FLTJCHGR(x)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTJCHGR_OFFSET(x))
#define STM32L4_DFSDM_FLTFCR(x)          (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTFCR_OFFSET(x))
#define STM32L4_DFSDM_FLTJDATAR(x)       (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTJDATAR_OFFSET(x))
#define STM32L4_DFSDM_FLTRDATAR(x)       (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTRDATAR_OFFSET(x))
#define STM32L4_DFSDM_FLTAWHTR(x)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTAWHTR_OFFSET(x))
#define STM32L4_DFSDM_FLTAWLTR(x)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTAWLTR_OFFSET(x))
#define STM32L4_DFSDM_FLTAWSR(x)         (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTAWSR_OFFSET(x))
#define STM32L4_DFSDM_FLTAWCFR(x)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTAWCFR_OFFSET(x))
#define STM32L4_DFSDM_FLTEXMAX(x)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTEXMAX_OFFSET(x))
#define STM32L4_DFSDM_FLTEXMIN(x)        (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTEXMIN_OFFSET(x))
#define STM32L4_DFSDM_FLTCNVTIMR(x)      (STM32L4_DFSDM_BASE + STM32L4_DFSDM_FLTCNVTIMR_OFFSET(x))

/* Register Bitfield Definitions ****************************************************/
/* DFSDM channel configuration y register (DFSDM_CHyCFGR1) */

/* Bits that are present only in DFSDM_CH0CFGR1 register (channel y=0) */

#define DFSDM_CH0CFGR1_CKOUTDIV_SHIFT    (16)      /* Bits 16-23: Output serial clock divider */
#define DFSDM_CH0CFGR1_CKOUTDIV_MASK     (0xff << DFSDM_CH0CFGR1_CKOUTDIV_SHIFT)
#  define DFSDM_CH0CFGR1_CKOUTDIV_NONE   (0 << DFSDM_CH0CFGR1_CKOUTDIV_SHIFT) /* Output clock generation disabled */
#  define DFSDM_CH0CFGR1_CKOUTDIV(n)     ((n) << DFSDM_CH0CFGR1_CKOUTDIV_SHIFT) /* Divider = CKOUTDIV+1, n=1..255 */
#define DFSDM_CH0CFGR1_CKOUTSRC          (1 << 30) /* Bit 30: Output serial clock source selection */
#define DFSDM_CH0CFGR1_DFSDMEN           (1 << 31) /* Bit 31: Global enable for DFSDM */

/* Bits that are present in all DFSDM_CHyCFGR1 registers */

#define DFSDM_CHCFGR1_SPICKSEL_SHIFT       (2)  /* Bits 2-3: SPI clock select for channel y */
#define DFSDM_CHCFGR1_SPICKSEL_MASK        (3 << DFSDM_CHCFGR1_SPICKSEL_SHIFT)
#  define DFSDM_CHCFGR1_SPICKSEL_EXT       (0 << DFSDM_CHCFGR1_SPICKSEL_SHIFT) /* clock coming from external CKINy input */
#  define DFSDM_CHCFGR1_SPICKSEL_CKOUT     (1 << DFSDM_CHCFGR1_SPICKSEL_SHIFT) /* clock coming from internal CKOUT output, sampling from SITP[1:0] */
#  define DFSDM_CHCFGR1_SPICKSEL_CKOUTFALL (2 << DFSDM_CHCFGR1_SPICKSEL_SHIFT) /* clock coming from internal CKOUT input, sampling on falling edge */
#  define DFSDM_CHCFGR1_SPICKSEL_CKOUTRISE (3 << DFSDM_CHCFGR1_SPICKSEL_SHIFT) /* clock coming from internal CKOOUT input, sampling on rising edge */
#define DFSDM_CHCFGR1_SITP_SHIFT           (0)   /* Bits 0-1: Serial interface type for channel y */
#define DFSDM_CHCFGR1_SITP_MASK            (3 << DFSDM_CHCFGR1_SITP_SHIFT)
#  define DFSDM_CHCFGR1_SITP_SPIRISE       (0 << DFSDM_CHCFGR1_SITP_SHIFT) /* SPI with rising edge to strobe data */
#  define DFSDM_CHCFGR1_SITP_SPIFALL       (1 << DFSDM_CHCFGR1_SITP_SHIFT) /* SPI with falling edge to strobe data */
#  define DFSDM_CHCFGR1_SITP_MANSEFALL     (2 << DFSDM_CHCFGR1_SITP_SHIFT) /* Manchester coded input on DATINy pin with falling edge as logic 1 */
#  define DFSDM_CHCFGR1_SITP_MANSERISE     (3 << DFSDM_CHCFGR1_SITP_SHIFT) /* Manchester coded input on DATINy pin with rising edge as logic 1 */

#define DFSDM_CHCFGR1_SCDEN              (1 << 5)  /* Bit 5: Short-circuit detector enable on channel y */
#define DFSDM_CHCFGR1_CKABEN             (1 << 6)  /* Bit 6: Clock absence detector enable on channel y */
#define DFSDM_CHCFGR1_CHEN               (1 << 7)  /* Bit 7: Channel y enable */
#define DFSDM_CHCFGR1_CHINSEL            (1 << 8)  /* Bit 8: Channel inputs selection */
#define DFSDM_CHCFGR1_DATMPX_SHIFT       (12)      /* Bits 12-13: Input data multiplexer for channel y */
#define DFSDM_CHCFGR1_DATMPX_MASK        (3 << DFSDM_CHCFGR1_DATMPX_SHIFT)
#  define DFSDM_CHCFGR1_DATMPX_EXT       (0 << DFSDM_CHCFGR1_DATMPX_SHIFT) /* External: Data to channel y are taken from external serial inputs as 1-bit values */
#  define DFSDM_CHCFGR1_DATMPX_ADC       (1 << DFSDM_CHCFGR1_DATMPX_SHIFT) /* ADC: Data to channel y are taken from internal analog to digital converter ADC */
#  define DFSDM_CHCFGR1_DATMPX_DATINR    (2 << DFSDM_CHCFGR1_DATMPX_SHIFT) /* DATINR: Data to channel y are taken from internal DFSDM_CHyDATINR register by direct CPU/DMA write */
                                                                           /* 3: Reserved */
#define DFSDM_CHCFGR1_DATPACK_SHIFT      (14)      /* Bits 14-15: Data packing mode in DFSDM_CHyDATINR register. */
#define DFSDM_CHCFGR1_DATPACK_MASK       (3 << DFSDM_CHCFGR1_DATPACK_SHIFT)
#  define DFSDM_CHCFGR1_DATPACK_STD      (0 << DFSDM_CHCFGR1_DATPACK_SHIFT) /* Standard: input data in DFSDM_CHyDATINR register are stored only in INDAT0[15:0] */
#  define DFSDM_CHCFGR1_DATPACK_INTER    (1 << DFSDM_CHCFGR1_DATPACK_SHIFT) /* Interleaved: input data in DFSDM_CHyDATINR register are stored as two samples INDAT0[15:0] and INDAT1[15:0] for channel y */
#  define DFSDM_CHCFGR1_DATPACK_DUAL     (2 << DFSDM_CHCFGR1_DATPACK_SHIFT) /* Dual: input data in DFSDM_CHyDATINR register are stored as two samples INDAT0[15:0] and INDAT1[15:0] for channels y and y+1 */
                                                                            /* 3: Reserved */

/* DFSDM channel configuration y register (DFSDM_CHyCFGR2) */

#define DFSDM_CHCFGR2_DTRBS_SHIFT        (3)   /* Bits  3-7: Data right bit-shift for channel y */
#define DFSDM_CHCFGR2_DTRBS_MASK         (0x1f << DFSDM_CHCFGR2_DTRBS_SHIFT)
#  define DFSDM_CHCFGR2_DTRBS(n)         ((n) << DFSDM_CHCFGR2_DTRBS_SHIFT) /* n = 0..31 */

#define DFSDM_CHCFGR2_OFFSET_SHIFT       (8)    /* Bits 8-31: 24-bit calibration offset for channel y */
#define DFSDM_CHCFGR2_OFFSET_MASK        (0xffffff << DFSDM_CHCFGR2_OFFSET_SHIFT)
#  define DFSDM_CHCFGR2_OFFSET(n)        ((n) << DFSDM_CHCFGR2_OFFSET_SHIFT)

/* DFSDM channel analog watchdog and short-circuit detector register (DFSDM_CHyAWSCDR) */

#define DFSDM_CHAWSCDR_SCDT_SHIFT        (0)   /* Bits 0-7: short-circuit detector threshold */
#define DFSDM_CHAWSCDR_SCDT_MASK         (0xff << DFSDM_CHAWSCDR_SCDT_SHIFT)
#  define DFSDM_CHAWSCDR_SCDT(n)         ((n) << DFSDM_CHAWSCDR_SCDT_SHIFT)
#define DFSDM_CHAWSCDR_BKSCD_SHIFT       (12)  /* Bits 12-15: Break signal assignment for short-circuit detector */
#define DFSDM_CHAWSCDR_BKSCD_MASK        (0xf << DFSDM_CHAWSCDR_BKSCD_SHIFT)
#define DFSDM_CHAWSCDR_AWFOSR_SHIFT      (16)  /* Bits 16-20: Analog watchdog filter oversampling ratio */
#define DFSDM_CHAWSCDR_AWFOSR_MASK       (0x1f << DFSDM_CHAWSCDR_AWFOSR_SHIFT)
#  define DFSDM_CHAWSCDR_AWFOSR(n)       ((n) << DFSDM_CHAWSCDR_AWFOSR_SHIFT)  /* n=0..31 */
#define DFSDM_CHAWSCDR_AWFORD_SHIFT      (22)  /* Bits 22-24: Analog watchdog Sinc filter order */
#define DFSDM_CHAWSCDR_AWFORD_MASK       (3 << DFSDM_CHAWSCDR_AWFORD_SHIFT)
#  define DFSDM_CHAWSCDR_AWFORD_FASTSINC (0 << DFSDM_CHAWSCDR_AWFORD_SHIFT)
#  define DFSDM_CHAWSCDR_AWFORD_SINC1    (1 << DFSDM_CHAWSCDR_AWFORD_SHIFT)
#  define DFSDM_CHAWSCDR_AWFORD_SINC2    (2 << DFSDM_CHAWSCDR_AWFORD_SHIFT)
#  define DFSDM_CHAWSCDR_AWFORD_SINC3    (3 << DFSDM_CHAWSCDR_AWFORD_SHIFT)

/* DFSDM channel watchdog filter data register (DFSDM_CHyWDATR) */

#define DFSDM_CHWDATR_WDATA_SHIFT        (0)   /* Bits 0-15: channel watchdog data */
#define DFSDM_CHWDATR_WDATA_MASK         (0xffff << DFSDM_CHWDATR_WDATA_SHIFT)

/* DFSDM channel data input register (DFSDM_CHyDATINR) */

#define DFSDM_CHDATINR_INDAT0_SHIFT      (0)   /* Bits 0-15: input data 0 */
#define DFSDM_CHDATINR_INDAT0_MASK       (0xffff << DFSDM_CHDATINR_INDAT0_SHIFT)
#  define DFSDM_CHDATINR_INDAT0(n)       ((n) << DFSDM_CHDATINR_INDAT0_SHIFT)
#define DFSDM_CHDATINR_INDAT1_SHIFT      (0)   /* Bits 15-31: input data 1 */
#define DFSDM_CHDATINR_INDAT1_MASK       (0xffff << DFSDM_CHDATINR_INDAT1_SHIFT)
#  define DFSDM_CHDATINR_INDAT1(n)       ((n) << DFSDM_CHDATINR_INDAT1_SHIFT)

/* DFSDM channel delay register (DFSDM_CHyDLYR) */

#define DFSDM_CHDLYR_PLSSKP_SHIFT        (0)   /* Bits 0-5: Pulses to skip for input data skipping function */
#define DFSDM_CHDLYR_PLSSKP_MASK         (0x1f << DFSDM_CHDLYR_PLSSKP_SHIFT)
#  define DFSDM_CHDLYR_PLSSKP(n)         ((n) << DFSDM_CHDLYR_PLSSKP_SHIFT) /* n=0..63 */

/* DFSDM filter x module registers */
/* DFSDM control register 1 (DFSDM_FLTxCR1) */

#define DFSDM_FLTCR1_DFEN                (1 << 0)  /* Bit 0: DFSDM_FLTx enable */
#define DFSDM_FLTCR1_JSWSTART            (1 << 1)  /* Bit 1: Start a conversion of the injected group of channels */
#define DFSDM_FLTCR1_JSYNC               (1 << 3)  /* Bit 3: Launch an injected conversion synchronously with the DFSDM_FLT0 JSWSTART trigger */
#define DFSDM_FLTCR1_JSCAN               (1 << 4)  /* Bit 4: Scanning conversion mode for injected conversions */
#define DFSDM_FLTCR1_JDMAEN              (1 << 5)  /* Bit 5: DMA channel enabled to read data for the injected channel group */
                                                   /* Bits 6-7: Reserved */
#define DFSDM_FLTCR1_JEXTSEL_SHIFT       (8)       /* Bits 8-10: External trigger selection for injected group */
#define DFSDM_FLTCR1_JEXTSEL_MASK        (0x7 << DFSDM_FLTCR1_JEXTSEL_SHIFT)
#  define DFSDM_FLTCR1_JEXTSEL(event)    ((event) << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* Event = 0..7 */

/* Trigger selections. Note: for RM0351 devices (STM32L4X6) these are valid for FLT0 and FLT1.
 * For subtle differences for FLT2 and FLT3, see the reference manual.
 */

#  define DFSDM_FLTCR1_JEXTSEL_T1TRGO    (0x00 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0000: Timer 1 TRGO event */
#  define DFSDM_FLTCR1_JEXTSEL_T1TRGO2   (0x01 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0001: Timer 1 TRGO2 event */
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define DFSDM_FLTCR1_JEXTSEL_T8TRGO  (0x02 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0010: Timer 8 TRGO event */
#  else
#    define DFSDM_FLTCR1_JEXTSEL_T3TRGO  (0x02 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0010: Timer 3 TRGO event */
#  endif
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define DFSDM_FLTCR1_JEXTSEL_T8TRGO2 (0x03 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0011: Timer 8 TRGO2 event */
#  else
#    define DFSDM_FLTCR1_JEXTSEL_T16CC1  (0x03 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0011: Timer 16 CC1 (or OC1) event */
#  endif
#  if !defined(CONFIG_STM32L4_STM32L4X3)
#    define DFSDM_FLTCR1_JEXTSEL_T4TRGO  (0x04 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0100: Timer 4 TRGO event */
#  endif
#  define DFSDM_FLTCR1_JEXTSEL_T6TRGO    (0x05 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0101: Timer 6 TRGO event */
#  define DFSDM_FLTCR1_JEXTSEL_EXTI11    (0x06 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0110: EXTI line 11 */
#  define DFSDM_FLTCR1_JEXTSEL_EXTI15    (0x07 << DFSDM_FLTCR1_JEXTSEL_SHIFT) /* 0111: EXTI line 15 */
                                                   /* Bits 11-12: Reserved */
#define DFSDM_FLTCR1_JEXTEN_SHIFT        (13)      /* Bits 13-14: Trigger enable and edge election for injected group */
#define DFSDM_FLTCR1_JEXTEN_MASK         (3 << DFSDM_FLTCR1_JEXTEN_SHIFT)
#  define DFSDM_FLTCR1_JEXTEN_NONE       (0 << DFSDM_FLTCR1_JEXTEN_SHIFT) /* 00: Trigger detection disabled */
#  define DFSDM_FLTCR1_JEXTEN_RISING     (1 << DFSDM_FLTCR1_JEXTEN_SHIFT) /* 01: Trigger detection on the rising edge */
#  define DFSDM_FLTCR1_JEXTEN_FALLING    (2 << DFSDM_FLTCR1_JEXTEN_SHIFT) /* 10: Trigger detection on the falling edge */
#  define DFSDM_FLTCR1_JEXTEN_BOTH       (3 << DFSDM_FLTCR1_JEXTEN_SHIFT) /* 11: Trigger detection on both the rising and falling edges */
#define DFSDM_FLTCR1_RSWSTART            (1 << 17) /* Bit 17: Software start of a conversion on the regular channel */
#define DFSDM_FLTCR1_RCONT               (1 << 18) /* Bit 18: Continuous mode selection for regular conversions */
#define DFSDM_FLTCR1_RSYNC               (1 << 19) /* Bit 19: Launch regular conversion synchronously with DFSDM_FLT0 */
#define DFSDM_FLTCR1_RDMAEN              (1 << 21) /* Bit 21: DMA channel enabled to read data for the regular conversion */
#define DFSDM_FLTCR1_RCH_SHIFT           (24)      /* Bits 24-26: Regular channel selection */
#define DFSDM_FLTCR1_RCH_MASK            (0x7 << DFSDM_FLTCR1_RCH_SHIFT)
#define DFSDM_FLTCR1_RCH(ch)             ((ch) << DFSDM_FLTCR1_RCH_SHIFT) /* Channel ch is selected as the regular channel */
#define DFSDM_FLTCR1_FAST                (1 << 29) /* Bit 29: Fast conversion mode selection for regular conversions */
#define DFSDM_FLTCR1_AWFSEL              (1 << 30) /* Bit 30: Analog watchdog fast mode select */

/* DFSDM control register 2 (DFSDM_FLTxCR2) */

/* Bits that are present only in DFSDM_FLT0CR2 register (filter x=0) */

#define DFSDM_FLT0CR2_SCDIE              (1 << 5)  /* Bit 5: Short-circuit detector interrupt enable */
#define DFSDM_FLT0CR2_CKABIE             (1 << 6)  /* Bit 6: Clock absence interrupt enable */

/* Bits that are present in all DFSDM_FLTxCR2 registers */

#define DFSDM_FLTCR2_JEOCIE              (1 << 0)  /* Bit 0: Injected end of conversion interrupt enable */
#define DFSDM_FLTCR2_REOCIE              (1 << 1)  /* Bit 1: Regular end of conversion interrupt enable */
#define DFSDM_FLTCR2_JOWRIE              (1 << 2)  /* Bit 2: Injected data overrun interrupt enable */
#define DFSDM_FLTCR2_ROWRIE              (1 << 3)  /* Bit 3: Regular data overrun interrupt enable
 */
#define DFSDM_FLTCR2_AWDIE               (1 << 4)  /* Bit 4: Analog watchdog interrupt enable */
#define DFSDM_FLTCR2_EXCH_SHIFT          (8)       /* Bits 8-15: Extremes detector channel selection */
#define DFSDM_FLTCR2_EXCH_MASK           (0xff << DFSDM_FLTCR2_EXCH_SHIFT)
#define DFSDM_FLTCR2_EXCH(ch)            ((1 << (ch)) << DFSDM_FLTCR2_EXCH_SHIFT)
#define DFSDM_FLTCR2_AWDCH_SHIFT         (16)      /* Bits 16-23: Analog watchdog channel selection */
#define DFSDM_FLTCR2_AWDCH_MASK          (0xff << DFSDM_FLTCR2_AWDCH_SHIFT)
#define DFSDM_FLTCR2_AWDCH(ch)           ((1 << (ch)) << DFSDM_FLTCR2_AWDCH_SHIFT)

/* DFSDM interrupt and status register (DFSDM_FLTxISR) */

/* Bits that are present only in DFSDM_FLT0ISR register (filter x=0) */

#define DFSDM_FLT0ISR_CKABF_SHIFT        (16)      /* Bits 16-23: short-circuit detector flag */
#define DFSDM_FLT0ISR_CKABF_MASK         (0xff << DFSDM_FLT0ISR_CKABF_SHIFT)
#define DFSDM_FLT0ISR_SCDF_SHIFT         (24)      /* Bits 24-31: clock absence flag */
#define DFSDM_FLT0ISR_SCDF_MASK          (0xff << DFSDM_FLT0ISR_SCDF_SHIFT)

/* Bits that are present in all DFSDM_FLTxISR registers */

#define DFSDM_FLTISR_JEOCF               (1 << 0) /* Bit 0: End of injected conversion flag */
#define DFSDM_FLTISR_REOCF               (1 << 1) /* Bit 1: End of regular conversion flag */
#define DFSDM_FLTISR_JOVRF               (1 << 2) /* Bit 2: Injected conversion overrun flag */
#define DFSDM_FLTISR_ROVRF               (1 << 3) /* Bit 3: Regular conversion overrun flag */
#define DFSDM_FLTISR_AWDF                (1 << 4) /* Bit 4: Analog watchdog */

#define DFSDM_FLTISR_JCIP                (1 << 13) /* Bit 13: Injected conversion in progress status */
#define DFSDM_FLTISR_RCIP                (1 << 14) /* Bit 14: Regular conversion in progress status */

/* DFSDM interrupt flag clear register (DFSDM_FLTxICR) */

/* Bits that are present only in DFSDM_FLT0ICR register (filter x=0) */

#define DFSDM_FLT0ISR_CLRCKABF_SHIFT     (16)      /* Bits 16-23: Clear the short-circuit detector flag */
#define DFSDM_FLT0ISR_CLRCKABF_MASK      (0xff << DFSDM_FLT0ISR_CLRCKABF_SHIFT)
#define DFSDM_FLT0ISR_CLRSCDF_SHIFT      (24)      /* Bits 24-31: clear the clock absence flag */
#define DFSDM_FLT0ISR_CLRSCDF_MASK       (0xff << DFSDM_FLT0ISR_CLRSCDF_SHIFT)

/* Bits that are present in all DFSDM_FLTxICR registers */

#define DFSDM_FLTICR_CLRJOVRF            (1 << 2)  /* Bit 2: Clear the injected conversion overrun flag */
#define DFSDM_FLTICR_CLRROVRF            (1 << 3)  /* Bit 3: Clear the regular conversion overrun flag */

/* DFSDM injected channel group selection register (DFSDM_FLTxJCHGR) */

#define DFSDM_FLTJCHGR_JCHG_SHIFT        (0)     /* Bits 0-7: Injected channel group selection */
#define DFSDM_FLTJCHGR_JCHG_MASK         (0xff << DFSDM_FLTJCHGR_JCHG_SHIFT)

/* DFSDM filter control register (DFSDM_FLTxFCR) */

#define DFSDM_FLTFCR_FOSR_SHIFT          (16)    /* Bits 16-25: Sinc filter oversampling ratio) */
#define DFSDM_FLTFCR_FOSR_MASK           (0x3ff << DFSDM_FLTFCR_FOSR_SHIFT)
#  define DFSDM_FLTFCR_FOSR(n)           ((n) << DFSDM_FLTFCR_FOSR_SHIFT)  /* n=0..1023 */
#define DFSDM_FLTFCR_FORD_SHIFT          (29)    /* Bits 29-31: Sinc filter order */
#define DFSDM_FLTFCR_FORD_MASK           (7 << DFSDM_FLTFCR_FORD_SHIFT)
#  define DFSDM_FLTFCR_FORD_FASTSINC     (0 << DFSDM_FLTFCR_FORD_SHIFT)
#  define DFSDM_FLTFCR_FORD_SINC1        (1 << DFSDM_FLTFCR_FORD_SHIFT)
#  define DFSDM_FLTFCR_FORD_SINC2        (2 << DFSDM_FLTFCR_FORD_SHIFT)
#  define DFSDM_FLTFCR_FORD_SINC3        (3 << DFSDM_FLTFCR_FORD_SHIFT)
#  define DFSDM_FLTFCR_FORD_SINC4        (4 << DFSDM_FLTFCR_FORD_SHIFT)
#  define DFSDM_FLTFCR_FORD_SINC5        (5 << DFSDM_FLTFCR_FORD_SHIFT)
#define DFSDM_FLTFCR_IOSR_SHIFT          (0)    /* Bits 0-7: Integrator oversampling ratio) */
#define DFSDM_FLTFCR_IOSR_MASK           (0xff << DFSDM_FLTFCR_IOSR_SHIFT)
#  define DFSDM_FLTFCR_IOSR(n)           ((n) << DFSDM_FLTFCR_IOSR_SHIFT)  /* n=0..255 */

/* DFSDM data register for injected group (DFSDM_FLTxJDATAR) */

#define DFSDM_FLTJDATAR_JDATACH_SHIFT    (0)     /* Bits  0-3: Injected channel most recently converted */
#define DFSDM_FLTJDATAR_JDATACH_MASK     (7 << DFSDM_FLTJDATAR_JDATACH_SHIFT)
#define DFSDM_FLTJDATAR_JDATA_SHIFT      (8)     /* Bits 8-23: Injected group conversion data */
#define DFSDM_FLTJDATAR_JDATA_MASK       (0xffffff << DFSDM_FLTJDATAR_JDATA_SHIFT)

/* DFSDM data register for the regular channel (DFSDM_FLTxRDATAR) */

#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L496XX) || \
    defined(CONFIG_STM32L4_STM32L4XR)
#  define DFSDM_FLTRDATAR_RDATACH_SHIFT  (0)     /* Bits  0-3: channel most recently converted */
#  define DFSDM_FLTRDATAR_RDATACH_MASK   (7 << DFSDM_FLTRDATAR_RDATACH_SHIFT)
#endif
#define DFSDM_FLTRDATAR_RPEND            (1 << 4 /* Bit 4: Regular channel has pending data */
#define DFSDM_FLTRDATAR_RDATA_SHIFT      (8)     /* Bits 8-23: channel conversion data */
#define DFSDM_FLTRDATAR_RDATA_MASK       (0xffffff << DFSDM_FLTRDATAR_RDATA_SHIFT)

/* DFSDM analog watchdog high threshold register (DFSDM_FLTxAWHTR) */

#define DFSDM_AWHTR_BKAWH_SHIFT          (0)      /* Bits  0-3: Break signal assignment to analog watchdog high threshold event */
#define DFSDM_AWHTR_BKAWH_MASK           (0xf << DFSDM_AWHTR_BKAWH_SHIFT)
                                                  /* Bits 4-7:  Reserved */
#define DFSDM_AWHTR_AWHT_SHIFT           (8)      /* Bits 8-31: Analog watchdog higher threshold */
#define DFSDM_AWHTR_AWHT_MASK            (0xffffff << DFSDM_AWHTR_AWHT_SHIFT)

/* DFSDM analog watchdog low threshold register (DFSDM_FLTxAWLTR) */

#define DFSDM_AWLTR_BKAWL_SHIFT          (0)      /* Bits 0-3: Break signal assignment to analog watchdog low threshold event */
#define DFSDM_AWLTR_BKAWL_MASK           (0xf << DFSDM_AWLTR_BKAWL_SHIFT)
                                                  /* Bits 4-7:  Reserved */
#define DFSDM_AWLTR_AWLT_SHIFT           (8)      /* Bits 8-31: Analog watchdog lower threshold */
#define DFSDM_AWLTR_AWLT_MASK            (0xffffff << DFSDM_AWLTR_AWLT_SHIFT)

/* DFSDM analog watchdog status register (DFSDM_FLTxAWSR) */

#define DFSDM_AWSR_AWLTF_SHIFT           (0)      /* Bits 0-7: Analog watchdog low threshold flag */
#define DFSDM_AWSR_AWLTF_MASK            (0xff << DFSDM_AWSR_AWLTF_SHIFT)
#define DFSDM_AWSR_AWHTF_SHIFT           (8)      /* Bits 8-15: Analog watchdog high threshold flag */
#define DFSDM_AWSR_AWHTF_MASK            (0xff << DFSDM_AWSR_AWHTF_SHIFT)

/* DFSDM analog watchdog clear flag register (DFSDM_FLTxAWCFR) */

#define DFSDM_AWCSR_CLRAWLTF_SHIFT       (0)      /* Bits 0-7: Clear analog watchdog low threshold flag */
#define DFSDM_AWCSR_CLRAWLTF_MASK        (0xff << DFSDM_AWCSR_CLRAWLTF_SHIFT)
#define DFSDM_AWCSR_CLRAWHTF_SHIFT       (8)      /* Bits 8-15: Clear analog watchdog high threshold flag */
#define DFSDM_AWCSR_CLRAWHTF_MASK        (0xff << DFSDM_AWCSR_CLRAWHTF_SHIFT)

/* DFSDM Extremes detector maximum register (DFSDM_FLTxEXMAX) */

#define DFSDM_EXMAX_EXMAXCH_SHIFT        (0)     /* Bits 0-1:  Extremes detector maximum data channel */
#define DFSDM_EXMAX_EXMAXCH_MASK         (7 << DFSDM_EXMAX_EXMAXCH_SHIFT)
#define DFSDM_EXMAX_EXMAX_SHIFT          (8)     /* Bits 8-31: Extremes detector maximum value */
#define DFSDM_EXMAX_EXMAX_MASK           (0xffffff << DFSDM_EXMAX_EXMAX_SHIFT)

/* DFSDM Extremes detector minimum register (DFSDM_FLTxEXMIN) */

#define DFSDM_EXMIN_EXMINCH_SHIFT        (0)     /* Bits 0-1:  Extremes detector minimum data channel */
#define DFSDM_EXMIN_EXMINCH_MASK         (7 << DFSDM_EXMIN_EXMINCH_SHIFT)
#define DFSDM_EXMIN_EXMIN_SHIFT          (8)     /* Bits 8-31: Extremes detector minimum value */
#define DFSDM_EXMIN_EXMIN_MASK           (0xffffff << DFSDM_EXMIN_EXMIN_SHIFT)

/* DFSDM conversion timer register (DFSDM_FLTxCNVTIMR) */

#define DFSDM_CNVTIMR_CNVCNT_SHIFT       (4)     /* Bits 4-31: 28-bit timer counting conversion time t = CNVCNT[27:0] / fDFSDMCLK */
#define DFSDM_CNVTIMR_CNVCNT_MASK        (~0xfu)

#endif /* __ARCH_ARM_SRC_STM32L4_CHIP_STM32L4_DFSDM_H */
