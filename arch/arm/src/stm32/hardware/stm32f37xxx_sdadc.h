/****************************************************************************************************
 * arch/arm/src/stm32/hardware/stm32f37xxx_sdadc.h
 *
 *   Copyright (C) 2009, 2011, 2013 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Studelec SA. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Marc Rechté <mrechte@studelec-sa.com>
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32F37XXX_SDADC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32F37XXX_SDADC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32_SDADC_CR1_OFFSET       0x0000  /* SDADC control register 1 */
#define STM32_SDADC_CR2_OFFSET       0x0004  /* SDADC control register 2 */
#define STM32_SDADC_ISR_OFFSET       0x0008  /* SDADC interrupt and status register */
#define STM32_SDADC_CLRISR_OFFSET    0x000c  /* SDADC interrupt and status clear register */
#define STM32_SDADC_JCHGR_OFFSET     0x0014  /* SDADC injected channel group selection register */
#define STM32_SDADC_CONF0R_OFFSET    0x0020  /* SDADC configuration 0 register */
#define STM32_SDADC_CONF1R_OFFSET    0x0024  /* SDADC configuration 1 register */
#define STM32_SDADC_CONF2R_OFFSET    0x0028  /* SDADC configuration 2 register */
#define STM32_SDADC_CONFCHR1_OFFSET  0x0040  /* SDADC channel configuration register 1 */
#define STM32_SDADC_CONFCHR2_OFFSET  0x0044  /* SDADC channel configuration register 2 */
#define STM32_SDADC_JDATAR_OFFSET    0x0060  /* SDADC data register for injected group */
#define STM32_SDADC_RDATAR_OFFSET    0x0064  /* SDADC data register for the regular channel */
#define STM32_SDADC_JDATA12R_OFFSET  0x0070  /* SDADC1 and SDADC2 injected data register */
#define STM32_SDADC_RDATA12R_OFFSET  0x0074  /* SDADC1 and SDADC2 regular data register */
#define STM32_SDADC_JDATA13R_OFFSET  0x0078  /* SDADC1 and SDADC3 injected data register */
#define STM32_SDADC_RDATA13R_OFFSET  0x007c  /* SDADC1 and SDADC3 regular data register */


/* Register Addresses *******************************************************************************/

#define STM32_SDADC1_CR1             (STM32_SDADC1_BASE+STM32_SDADC_CR1_OFFSET)
#define STM32_SDADC1_CR2             (STM32_SDADC1_BASE+STM32_SDADC_CR2_OFFSET)
#define STM32_SDADC1_ISR             (STM32_SDADC1_BASE+STM32_SDADC_ISR_OFFSET)
#define STM32_SDADC1_CLRISR          (STM32_SDADC1_BASE+STM32_SDADC_CLRISR_OFFSET)
#define STM32_SDADC1_JCHGR           (STM32_SDADC1_BASE+STM32_SDADC_JCHGR_OFFSET)
#define STM32_SDADC1_CONF0R          (STM32_SDADC1_BASE+STM32_SDADC_CONF0R_OFFSET)
#define STM32_SDADC1_CONF1R          (STM32_SDADC1_BASE+STM32_SDADC_CONF1R_OFFSET)
#define STM32_SDADC1_CONF2R          (STM32_SDADC1_BASE+STM32_SDADC_CONF2R_OFFSET)
#define STM32_SDADC1_CONFCHR1        (STM32_SDADC1_BASE+STM32_SDADC_CONFCHR1_OFFSET)
#define STM32_SDADC1_CONFCHR2        (STM32_SDADC1_BASE+STM32_SDADC_CONFCHR2_OFFSET)
#define STM32_SDADC1_JDATAR          (STM32_SDADC1_BASE+STM32_SDADC_JDATAR_OFFSET)
#define STM32_SDADC1_RDATAR          (STM32_SDADC1_BASE+STM32_SDADC_RDATAR_OFFSET)
#define STM32_SDADC1_JDATA12R        (STM32_SDADC1_BASE+STM32_SDADC_JDATA12R_OFFSET)
#define STM32_SDADC1_RDATA12R        (STM32_SDADC1_BASE+STM32_SDADC_RDATA12R_OFFSET)
#define STM32_SDADC1_JDATA13R        (STM32_SDADC1_BASE+STM32_SDADC_JDATA13R_OFFSET)
#define STM32_SDADC1_RDATA13R        (STM32_SDADC1_BASE+STM32_SDADC_RDATA13R_OFFSET)

#define STM32_SDADC2_CR1             (STM32_SDADC2_BASE+STM32_SDADC_CR1_OFFSET)
#define STM32_SDADC2_CR2             (STM32_SDADC2_BASE+STM32_SDADC_CR2_OFFSET)
#define STM32_SDADC2_ISR             (STM32_SDADC2_BASE+STM32_SDADC_ISR_OFFSET)
#define STM32_SDADC2_CLRISR          (STM32_SDADC2_BASE+STM32_SDADC_CLRISR_OFFSET)
#define STM32_SDADC2_JCHGR           (STM32_SDADC2_BASE+STM32_SDADC_JCHGR_OFFSET)
#define STM32_SDADC2_CONF0R          (STM32_SDADC2_BASE+STM32_SDADC_CONF0R_OFFSET)
#define STM32_SDADC2_CONF1R          (STM32_SDADC2_BASE+STM32_SDADC_CONF1R_OFFSET)
#define STM32_SDADC2_CONF2R          (STM32_SDADC2_BASE+STM32_SDADC_CONF2R_OFFSET)
#define STM32_SDADC2_CONFCHR1        (STM32_SDADC2_BASE+STM32_SDADC_CONFCHR1_OFFSET)
#define STM32_SDADC2_CONFCHR2        (STM32_SDADC2_BASE+STM32_SDADC_CONFCHR2_OFFSET)
#define STM32_SDADC2_JDATAR          (STM32_SDADC2_BASE+STM32_SDADC_JDATAR_OFFSET)
#define STM32_SDADC2_RDATAR          (STM32_SDADC2_BASE+STM32_SDADC_RDATAR_OFFSET)

#define STM32_SDADC3_CR1             (STM32_SDADC3_BASE+STM32_SDADC_CR1_OFFSET)
#define STM32_SDADC3_CR2             (STM32_SDADC3_BASE+STM32_SDADC_CR2_OFFSET)
#define STM32_SDADC3_ISR             (STM32_SDADC3_BASE+STM32_SDADC_ISR_OFFSET)
#define STM32_SDADC3_CLRISR          (STM32_SDADC3_BASE+STM32_SDADC_CLRISR_OFFSET)
#define STM32_SDADC3_JCHGR           (STM32_SDADC3_BASE+STM32_SDADC_JCHGR_OFFSET)
#define STM32_SDADC3_CONF0R          (STM32_SDADC3_BASE+STM32_SDADC_CONF0R_OFFSET)
#define STM32_SDADC3_CONF1R          (STM32_SDADC3_BASE+STM32_SDADC_CONF1R_OFFSET)
#define STM32_SDADC3_CONF2R          (STM32_SDADC3_BASE+STM32_SDADC_CONF2R_OFFSET)
#define STM32_SDADC3_CONFCHR1        (STM32_SDADC3_BASE+STM32_SDADC_CONFCHR1_OFFSET)
#define STM32_SDADC3_CONFCHR2        (STM32_SDADC3_BASE+STM32_SDADC_CONFCHR2_OFFSET)
#define STM32_SDADC3_JDATAR          (STM32_SDADC3_BASE+STM32_SDADC_JDATAR_OFFSET)
#define STM32_SDADC3_RDATAR          (STM32_SDADC3_BASE+STM32_SDADC_RDATAR_OFFSET)


/* Register Bitfield Definitions ********************************************************************/
/* SDADC control register 1 */

#define SDADC_CR1_EOCALIE            (1 << 0)  /* Bit 0: End of calibration interrupt enable */
#define SDADC_CR1_JEOCIE             (1 << 1)  /* Bit 1: Injected end of conversion interrupt enable */
#define SDADC_CR1_JOVRIE             (1 << 2)  /* Bit 2: Injected data overrun interrupt enable */
#define SDADC_CR1_REOCIE             (1 << 3)  /* Bit 3: Regular end of conversion interrupt enable */
#define SDADC_CR1_ROVRIE             (1 << 4)  /* Bit 4: Regular data overrun interrupt enable */
#define SDADC_CR1_REFV_SHIFT         (8)       /* Bits 8-9: Reference voltage selection */
#define SDADC_CR1_REFV_MASK          (0x3 << SDADC_CR1_REFV_SHIFT)
#  define SDADC_CR1_REFV_EXT         (0 << SDADC_CR1_REFV_SHIFT)
#  define SDADC_CR1_REFV_INT1p2      (1 << SDADC_CR1_REFV_SHIFT)
#  define SDADC_CR1_REFV_INT1p8      (2 << SDADC_CR1_REFV_SHIFT)
#  define SDADC_CR1_REFV_INT         (3 << SDADC_CR1_REFV_SHIFT)
#define SDADC_CR1_SLOWCK             (1 << 10) /* Bit 10: Slow clock mode enable */
#define SDADC_CR1_SBI                (1 << 11) /* Bit 11: Enter Standby mode when idle */
#define SDADC_CR1_PDI                (1 << 12) /* Bit 12: Enter power down mode when idle */
#define SDADC_CR1_JSYNC              (1 << 14) /* Bit 14: Launch a injected conversion synchronously with SDADC1 */
#define SDADC_CR1_RSYNC              (1 << 15) /* Bit 15: Launch regular conversion synchronously with SDADC1 */
#define SDADC_CR1_JDMAEN             (1 << 16) /* Bit 16: DMA channel enabled to read data for the injected channel group */
#define SDADC_CR1_RDMAEN             (1 << 17) /* Bit 17: DMA channel enabled to read data for the regular channel */
#define SDADC_CR1_INIT               (1 << 31) /* Bit 31: Initialization mode request */

/* SDADC control register 2 */

#define SDADC_CR2_ADON               (1 << 0)  /* Bit 0: SDADC enable */
#define SDADC_CR2_CALIBCNT_SHIFT     (1)       /* Bit 1-2: Number of calibration sequences to be performed (number of valid configurations) */
#define SDADC_CR2_CALIBCNT_MASK      (0x3 << SDADC_CR2_CALIBCNT_SHIFT)
#define SDADC_CR2_STARTCALIB         (1 << 4)  /* Bit 4: Start calibration */
#define SDADC_CR2_JCONT              (1 << 5)  /* Bit 5: Continuous mode selection for injected conversions */
#define SDADC_CR2_JDS                (1 << 6)  /* Bit 6: Delay start of injected conversions */
#define SDADC_CR2_JEXTSEL_SHIFT      (8)       /* Bit 8-10: Trigger signal selection for launching injected conversions */
#define SDADC_CR2_JEXTSEL_MASK          (0x7 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_TIM13_CH1  (0 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_TIM14_CH1  (1 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_TIM15_CH2  (2 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_TIM3_CH1   (3 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_TIM4_CH1   (4 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_TIM19_CH2  (5 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_EXTI15     (6 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC1_CR2_JEXTSEL_EXTI11     (7 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_TIM17_CH1  (0 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_TIM12_CH1  (1 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_TIM2_CH3   (2 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_TIM3_CH2   (3 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_TIM4_CH2   (4 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_TIM19_CH3  (5 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_EXTI15     (6 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC2_CR2_JEXTSEL_EXTI11     (7 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_TIM16_CH1  (0 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_TIM12_CH1  (1 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_TIM2_CH4   (2 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_TIM3_CH3   (3 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_TIM4_CH3   (4 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_TIM19_CH4  (5 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_EXTI15     (6 << SDADC_CR2_JEXTSEL_SHIFT)
#  define SDADC3_CR2_JEXTSEL_EXTI11     (7 << SDADC_CR2_JEXTSEL_SHIFT)
#define SDADC_CR2_JEXTEN_SHIFT          (13)   /* Bit 13-14: Trigger enable and trigger edge selection for injected conversions */
#define SDADC_CR2_JEXTEN_MASK        (0x3 << SDADC_CR2_JEXTEN_SHIFT)
#  define SDADC_CR2_JEXTEN_NONE      (0 << SDADC_CR2_JEXTEN_SHIFT)
#  define SDADC_CR2_JEXTEN_RISING    (1 << SDADC_CR2_JEXTEN_SHIFT)
#  define SDADC_CR2_JEXTEN_FALLING   (2 << SDADC_CR2_JEXTEN_SHIFT)
#  define SDADC_CR2_JEXTEN_BOTH      (3 << SDADC_CR2_JEXTEN_SHIFT)
#define SDADC_CR2_JSWSTART           (1 << 15) /* Bit 15: Start a conversion of the injected group of channels */
#define SDADC_CR2_RCH_SHIFT          (16)      /* Bit 16-19: Regular channel selection */
#define SDADC_CR2_RCH_MASK           (0xf << SDADC_CR2_RCH_SHIFT)
#define SDADC_CR2_RCONT              (1 << 22) /* Bit 22: Continuous mode selection for regular conversions */
#define SDADC_CR2_RSWSTART           (1 << 23) /* Bit 23: Software start of a conversion on the regular channel */
#define SDADC_CR2_FAST               (1 << 24) /* Bit 24: Fast conversion mode selection */

/* SDADC interrupt and status register */

#define SDADC_ISR_EOCALF             (1 << 0)  /* Bit 0: End of calibration flag */
#define SDADC_ISR_JEOCF              (1 << 1)  /* Bit 1: End of injected conversion flag */
#define SDADC_ISR_JOVRF              (1 << 2)  /* Bit 2: Injected conversion overrun flag */
#define SDADC_ISR_REOCF              (1 << 3)  /* Bit 3: End of regular conversion flag */
#define SDADC_ISR_ROVRF              (1 << 4)  /* Bit 4: Regular conversion overrun flag */
#define SDADC_ISR_CALIBIP            (1 << 12) /* Bit 12: Calibration in progress status */
#define SDADC_ISR_JCIP               (1 << 13) /* Bit 13: Injected conversion in progress status */
#define SDADC_ISR_RCIP               (1 << 14) /* Bit 14: Regular conversion in progress status */
#define SDADC_ISR_STABIP             (1 << 15) /* Bit 15: Stabilization in progress status */
#define SDADC_ISR_INITRDY            (1 << 31) /* Bit 31: Initialization mode is ready */

/* SDADC interrupt and status clear register */

#define SDADC_CLRISR_CLREOCALF       (1 << 0)  /* Bit 0: Clear the end of calibration flag */
#define SDADC_CLRISR_CLRJOVRF        (1 << 2)  /* Bit 2: Clear the injected conversion overrun flag */
#define SDADC_CLRISR_CLRROVRF        (1 << 4)  /* Bit 4: Clear the regular conversion overrun flag */

/* SDADC injected channel group selection register */

#define SDADC_JCHGR_JCHG_SHIFT       (0)       /* Bit 0-8: Injected channel group selection */
#define SDADC_JCHGR_JCHG_MASK        (0x1ff << SDADC_JCHGR_JCHG_SHIFT)
#define SDADC_JCHGR_JCHG_CH(n)       (1 << (n + SDADC_JCHGR_JCHG_SHIFT))

/* SDADC configuration 0-2 register */

#define SDADC_CONF0R                 0
#define SDADC_CONF1R                 1
#define SDADC_CONF2R                 2

#define SDADC_CONFR_OFFSET_SHIFT     (0)       /* Bit 0-11: Twelve-bit calibration offset for configuration 0-2 */
#define SDADC_CONFR_OFFSET_MASK      (0xfff << SDADC_CONFR_OFFSET_SHIFT)
#define SDADC_CONFR_GAIN_SHIFT       (20)      /* Bit 20-22: Gain setting for configuration 0-2 */
#define SDADC_CONFR_GAIN_MASK        (0x7 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_1X        (0 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_2X        (1 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_4X        (2 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_8X        (3 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_16X       (4 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_32X       (5 << SDADC_CONFR_GAIN_SHIFT)
#  define SDADC_CONFR_GAIN_0p5X      (7 << SDADC_CONFR_GAIN_SHIFT)
#define SDADC_CONFR_SE_SHIFT         (26)      /* Bit 27-26: Single-ended mode for configuration 0-2 */
#define SDADC_CONFR_SE_MASK          (0x3 << SDADC_CONFR_SE_SHIFT)
#  define SDADC_CONFR_SE_DIF         (0 << SDADC_CONFR_SE_SHIFT)
#  define SDADC_CONFR_SE_SE_OFFSET   (1 << SDADC_CONFR_SE_SHIFT)
#  define SDADC_CONFR_SE_SE_ZERO     (3 << SDADC_CONFR_SE_SHIFT)
#define SDADC_CONFR_COMMON_SHIFT     (30)      /* Bit 30-31: Common mode for configuration 0-2 */
#define SDADC_CONFR_COMMON_MASK      (0x3 << SDADC_CONFR_COMMON_SHIFT)
#  define SDADC_CONFR_COMMON_GND     (0 << SDADC_CONFR_COMMON_SHIFT)
#  define SDADC_CONFR_COMMON_VCM     (1 << SDADC_CONFR_COMMON_SHIFT)
#  define SDADC_CONFR_COMMON_VDD     (2 << SDADC_CONFR_COMMON_SHIFT)

/* SDADC channel configuration register 1 */

#define SDADC_CONFCHR1_CH_SHIFT(i)   (2*i) /* Bit 0-1: Channel i configuration 0-7 */
#define SDADC_CONFCHR1_CH_MASK(i)    (0x3 << SDADC_CONFCHR1_CH_SHIFT(i))

/* SDADC channel configuration register 2 */

#define SDADC_CONFCHR2_CH8_SHIFT     (0)       /* Bit 0-1: Channel 8 configuration */
#define SDADC_CONFCHR2_CH8_MASK      (0x3 << SDADC_CONFCHR2_CH8_SHIFT)

/* SDADC data register for injected group */

#define SDADC_JDATAR_JDATA_SHIFT     (0)       /* Bit 0-15: Injected group conversion data */
#define SDADC_JDATAR_JDATA_MASK      (0xffff << SDADC_JDATAR_JDATA_SHIFT)
#define SDADC_JDATAR_JDATACH_SHIFT   (24)      /* Bit 24-27: Injected channel most recently converted */
#define SDADC_JDATAR_JDATACH_MASK    (0xf << SDADC_JDATAR_JDATACH_SHIFT)

/* SDADC data register for the regular channel */

#define SDADC_RDATAR_RDATA_SHIFT     (0)       /* Bit 0-15: Regular channel conversion data */
#define SDADC_RDATAR_RDATA_MASK      (0xffff << SDADC_RDATAR_RDATA_SHIFT)

/* SDADC1 and SDADC2 injected data register */

#define SDADC_JDATA12R_JDATA1_SHIFT  (0)       /* Bit 0-15: Injected group conversion data for SDADC1 */
#define SDADC_JDATA12R_JDATA1_MASK   (0xffff << SDADC_JDATA12R_JDATA1_SHIFT)
#define SDADC_JDATA12R_JDATA2_SHIFT  (16)       /* Bit 16-31: Injected group conversion data for SDADC2 */
#define SDADC_JDATA12R_JDATA2_MASK   (0xffff << SDADC_JDATA12R_JDATA2_SHIFT)

/* SDADC1 and SDADC2 regular data register */

#define SDADC_RDATA12R_RDATA1_SHIFT  (0)       /* Bit 0-15: Regular conversion data for SDADC1 */
#define SDADC_RDATA12R_RDATA1_MASK   (0xffff << SDADC_RDATA12R_RDATA1_SHIFT)
#define SDADC_RDATA12R_RDATA2_SHIFT  (16)       /* Bit 16-31: Regular conversion data for SDADC2 */
#define SDADC_RDATA12R_RDATA2_MASK   (0xffff << SDADC_RDATA12R_RDATA2_SHIFT)

/* SDADC1 and SDADC3 injected data register */

#define SDADC_JDATA13R_JDATA1_SHIFT  (0)       /* Bit 0-15: Injected group conversion data for SDADC1 */
#define SDADC_JDATA13R_JDATA1_MASK   (0xffff << SDADC_JDATA13R_JDATA1_SHIFT)
#define SDADC_JDATA13R_JDATA3_SHIFT  (16)       /* Bit 16-31: Injected group conversion data for SDADC3 */
#define SDADC_JDATA13R_JDATA3_MASK   (0xffff << SDADC_JDATA13R_JDATA3_SHIFT)

/* SDADC1 and SDADC3 regular data register */

#define SDADC_RDATA13R_RDATA1_SHIFT  (0)       /* Bit 0-15: Regular conversion data for SDADC1 */
#define SDADC_RDATA13R_RDATA1_MASK   (0xffff << SDADC_RDATA13R_RDATA1_SHIFT)
#define SDADC_RDATA13R_RDATA3_SHIFT  (16)       /* Bit 16-31: Regular conversion data for SDADC3 */
#define SDADC_RDATA13R_RDATA3_MASK   (0xffff << SDADC_RDATA13R_RDATA3_SHIFT)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32F37XXX_SDADC_H */
