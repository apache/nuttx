/********************************************************************************************
 * arch/arm/src/samd2l2/chip/saml_evsys.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_EVSYS_H
#define __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_EVSYS_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* EVSYS register offsets *******************************************************************/

#define SAM_EVSYS_CTRLA_OFFSET             0x0000  /* Control register */
#define SAM_EVSYS_CHSTATUS_OFFSET          0x000c  /* Channel status register */
#define SAM_EVSYS_INTENCLR_OFFSET          0x0010  /* Interrupt enable clear register */
#define SAM_EVSYS_INTENSET_OFFSET          0x0014  /* Interrupt enable set register */
#define SAM_EVSYS_INTFLAG_OFFSET           0x0018  /* Interrupt flag status and clear register */
#define SAM_EVSYS_SWEVT_OFFSET             0x001c  /* Event user */
#define SAM_EVSYS_CHANNEL_OFFSET(n)  (0x0020 + ((n) << 2)) /* Channel registers */
#define SAM_EVSYS_USER_OFFSET(n)     (0x0080 + ((m) << 2)) /* User registers */

/* EVSYS register addresses *****************************************************************/

#define SAM_EVSYS_CTRLA                    (SAM_EVSYS_BASE+SAM_EVSYS_CTRLA_OFFSET)
#define SAM_EVSYS_CHSTATUS                 (SAM_EVSYS_BASE+SAM_EVSYS_CHSTATUS_OFFSET
#define SAM_EVSYS_INTENCLR                 (SAM_EVSYS_BASE+SAM_EVSYS_INTENCLR_OFFSET
#define SAM_EVSYS_INTENSET                 (SAM_EVSYS_BASE+SAM_EVSYS_INTENSET_OFFSET
#define SAM_EVSYS_INTFLAG                  (SAM_EVSYS_BASE+SAM_EVSYS_INTFLAG_OFFSET)
#define SAM_EVSYS_SWEVT                    (SAM_EVSYS_BASE+SAM_EVSYS_SWEVT_OFFSET)
#define SAM_EVSYS_CHANNEL_BASE(n)          (SAM_EVSYS_BASE+SAM_EVSYS_CHANNEL_OFFSET(n))
#define SAM_EVSYS_USER_BASE(n)             (SAM_EVSYS_BASE+SAM_EVSYS_USER_OFFSET(n))

/* EVSYS register bit definitions ***********************************************************/

/* Control register */

#define EVSYS_CTRLA_SWRST                   (1 << 0)  /* Bit 0: Software Reset */

/* Channel status register */

#define EVSYS_CHSTATUS_USRRDY_SHIFT        (0)       /* Bits 0-7: User Ready for Channel n, n=0-11 */
#define EVSYS_CHSTATUS_USRRDY_MASK         (0xfff << EVSYS_CHSTATUS_USRRDY_SHIFT)
#  define EVSYS_CHSTATUS_USRRDY(n)         ((uint32_t)(n) << EVSYS_CHSTATUS_USRRDY_SHIFT)
#define EVSYS_CHSTATUS_CHBUSY_SHIFT        (8)       /* Bits 8-15: Channel Busy n, n=0-11 */
#define EVSYS_CHSTATUS_CHBUSY_MASK         (0xfff << EVSYS_CHSTATUS_CHBUSY_SHIFT)
#  define EVSYS_CHSTATUS_CHBUSY(n)         ((uint32_t)(n) << EVSYS_CHSTATUS_CHBUSY_SHIFT)

/* Interrupt enable clear, interrupt enable set, and interrupt flag status and clear registers */

#define EVSYS_INT_OVR_SHIFT                (0)       /* Bits 0-7: Overrun channel n interrupt, n= 0-11 */
#define EVSYS_INT_OVR_MASK                 (0xfff << EVSYS_INT_OVR_SHIFT)
#  define EVSYS_INT_OVR(n)                 (1 << (n))
#define EVSYS_INT_EVD_SHIFT                (8)       /* Bits 8-15: Event detected channel n interrupt */
#define EVSYS_INT_EVD_MASK                 (0xff << EVSYS_INT_EVD_SHIFT)
#  define EVSYS_INT_EVD(n)                 (1 << ((n)+8))

/* Event user register */

#define EVSYS_SWEVT_CHANNEL_SHIFT          (0))      /* Bits 0-11: Channel n software selection, n=0-11 */
#define EVSYS_SWEVT_CHANNEL_MASK           (0xfff << EVSYS_SWEVT_CHANNEL_SHIFT)
#  define EVSYS_SWEVT_CHANNEL(n)           (1 << (n))

/* Channel registers */

#define EVSYS_CHANNEL_EVGEN_SHIFT          (0)       /* Bits 0-6: Event generator */
#define EVSYS_CHANNEL_EVGEN_MASK           (0x7f << EVSYS_CHANNEL_EVGEN_SHIFT)
#  define EVSYS_CHANNEL_EVGEN_NONE         (0x00 << EVSYS_CHANNEL_EVGEN_SHIFT) /* No event generator selected */
#  define EVSYS_CHANNEL_EVGEN_RTC_CMP0     (0x01 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Compare 0 or alarm 0 */
#  define EVSYS_CHANNEL_EVGEN_RTC_CMP1     (0x02 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Compare 1 */
#  define EVSYS_CHANNEL_EVGEN_RTC_OVF      (0x03 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Overflow */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER0     (0x04 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 0 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER1     (0x05 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 1 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER2     (0x06 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 2 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER3     (0x07 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 3 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER4     (0x08 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 4 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER5     (0x09 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 5 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER6     (0x0a << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 6 */
#  define EVSYS_CHANNEL_EVGEN_RTC_PER7     (0x0b << EVSYS_CHANNEL_EVGEN_SHIFT) /* Period 7 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT0  (0x0c << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 0 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT1  (0x0d << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 1 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT2  (0x0e << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 2 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT3  (0x0f << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 3 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT4  (0x10 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 4 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT5  (0x11 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 5 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT6  (0x12 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 6 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT7  (0x13 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 7 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT8  (0x14 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 8 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT9  (0x15 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 9 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT10 (0x16 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 10 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT11 (0x17 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 11 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT12 (0x18 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 12 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT13 (0x19 << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 13 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT14 (0x1a << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 14 */
#  define EVSYS_CHANNEL_EVGEN_EIC_EXTINT15 (0x1b << EVSYS_CHANNEL_EVGEN_SHIFT) /* External interrupt 15 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH0     (0x1c << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 0 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH1     (0x1d << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 1 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH2     (0x1e << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 2 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH3     (0x1f << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 3 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH4     (0x20 << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 4 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH5     (0x21 << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 5 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH6     (0x22 << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 6 */
#  define EVSYS_CHANNEL_EVGEN_DMAC_CH7     (0x23 << EVSYS_CHANNEL_EVGEN_SHIFT) /* DMA channel 7 */
#  define EVSYS_CHANNEL_EVGEN_TCCO_OVF     (0x24 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 overflow */
#  define EVSYS_CHANNEL_EVGEN_TCCO_TRG     (0x25 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 trigger */
#  define EVSYS_CHANNEL_EVGEN_TCCO_CNT     (0x26 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 counter */
#  define EVSYS_CHANNEL_EVGEN_TCCO_MCX0    (0x27 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TCCO_MCX1    (0x28 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TCCO_MCX2    (0x29 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 match/capture 2 */
#  define EVSYS_CHANNEL_EVGEN_TCCO_MCX3    (0x2a << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC0 match/capture 2 */
#  define EVSYS_CHANNEL_EVGEN_TCC1_OVF     (0x2b << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC1 overflow */
#  define EVSYS_CHANNEL_EVGEN_TCC1_TRG     (0x2c << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC1 trigger */
#  define EVSYS_CHANNEL_EVGEN_TCC1_CNT     (0x2d << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC1 counter */
#  define EVSYS_CHANNEL_EVGEN_TCC1_MCX0    (0x2e << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC01match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TCC1_MCX1    (0x2f << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC1 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TCC2_OVF     (0x30 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC2 overflow */
#  define EVSYS_CHANNEL_EVGEN_TCC2_TRG     (0x31 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC2 trigger */
#  define EVSYS_CHANNEL_EVGEN_TCC2_CNT     (0x32 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC2 counter */
#  define EVSYS_CHANNEL_EVGEN_TCC2_MCX0    (0x33 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC2 match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TCC2_MCX1    (0x34 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCC2 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TC0_OVF      (0x35 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TCO Overflow */
#  define EVSYS_CHANNEL_EVGEN_TC0_MC0      (0x36 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC0 match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TC0_MC1      (0x37 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC0 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TC1_OVF      (0x38 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC1 Overflow */
#  define EVSYS_CHANNEL_EVGEN_TC1_MC0      (0x39 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC1 match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TC1_MC1      (0x3a << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC1 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TC2_OVF      (0x3b << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC2 Overflow */
#  define EVSYS_CHANNEL_EVGEN_TC2_MC0      (0x3c << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC2 match/captue 0 */
#  define EVSYS_CHANNEL_EVGEN_TC2_MC1      (0x3d << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC2 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TC3_OVF      (0x3e << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC3 Overflow */
#  define EVSYS_CHANNEL_EVGEN_TC3_MC0      (0x3f << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC3 match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TC3_MC1      (0x40 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC3 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_TC4_OVF      (0x41 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC4 Overflow */
#  define EVSYS_CHANNEL_EVGEN_TC4_MC0      (0x42 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC4 match/capture 0 */
#  define EVSYS_CHANNEL_EVGEN_TC4_MC1      (0x43 << EVSYS_CHANNEL_EVGEN_SHIFT) /* TC4 match/capture 1 */
#  define EVSYS_CHANNEL_EVGEN_ADC_RESRDY   (0x44 << EVSYS_CHANNEL_EVGEN_SHIFT) /* ADC result ready */
#  define EVSYS_CHANNEL_EVGEN_ADC_WINMON   (0x45 << EVSYS_CHANNEL_EVGEN_SHIFT) /* ADC window monitor */
#  define EVSYS_CHANNEL_EVGEN_AC_COMP0     (0x46 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Analog comparator 0 */
#  define EVSYS_CHANNEL_EVGEN_AC_COMP1     (0x47 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Analog comparator 1 */
#  define EVSYS_CHANNEL_EVGEN_AC_WIN0      (0x48 << EVSYS_CHANNEL_EVGEN_SHIFT) /* Analog window comparator */
#  define EVSYS_CHANNEL_EVGEN_DAC_EMPTY0   (0x49 << EVSYS_CHANNEL_EVGEN_SHIFT) /* DAC data buffer 0 empty */
#  define EVSYS_CHANNEL_EVGEN_DAC_EMPTY1   (0x4a << EVSYS_CHANNEL_EVGEN_SHIFT) /* DAC data buffer 1 empty */
#  define EVSYS_CHANNEL_EVGEN_PTC_EOC      (0x4b << EVSYS_CHANNEL_EVGEN_SHIFT) /* PTC end of conversion */
#  define EVSYS_CHANNEL_EVGEN_PTC_WCOMPT   (0x4c << EVSYS_CHANNEL_EVGEN_SHIFT) /* PTC window comparator */
#  define EVSYS_CHANNEL_EVGEN_TRNG_READY   (0x4d << EVSYS_CHANNEL_EVGEN_SHIFT) /* TRNG data ready */
#  define EVSYS_CHANNEL_EVGEN_CCL_LUTOUT0  (0x4e << EVSYS_CHANNEL_EVGEN_SHIFT) /* CCL output 0 */
#  define EVSYS_CHANNEL_EVGEN_CCL_LUTOUT1  (0x4f << EVSYS_CHANNEL_EVGEN_SHIFT) /* CCL output 1 */
#  define EVSYS_CHANNEL_EVGEN_CCL_LUTOUT2  (0x50 << EVSYS_CHANNEL_EVGEN_SHIFT) /* CCL output 2 */
#  define EVSYS_CHANNEL_EVGEN_CCL_LUTOUT3  (0x51 << EVSYS_CHANNEL_EVGEN_SHIFT) /* CCL output 3 */
#  define EVSYS_CHANNEL_EVGEN_PAC_ACCERR   (0x52 << EVSYS_CHANNEL_EVGEN_SHIFT) /* PAC access error */
#define EVSYS_CHANNEL_PATH_SHIFT           (8)       /* Bits 8-9: Path selection */
#define EVSYS_CHANNEL_PATH_MASK            (3 << EVSYS_CHANNEL_PATH_SHIFT)
#  define EVSYS_CHANNEL_PATH_SYNCH         (0 << EVSYS_CHANNEL_PATH_SHIFT) /* Synchronized path */
#  define EVSYS_CHANNEL_PATH_RESYNCH       (1 << EVSYS_CHANNEL_PATH_SHIFT) /* Resynchronized path */
#  define EVSYS_CHANNEL_PATH_ASYNCH        (2 << EVSYS_CHANNEL_PATH_SHIFT) /* Asynchronous path */
#define EVSYS_CHANNEL_EDGESEL_SHIFT        (10)      /* Bits 10-11: Edge dection selection */
#define EVSYS_CHANNEL_EDGESEL_MASK         (3 << EVSYS_CHANNEL_EDGESEL_SHIFT)
#  define EVSYS_CHANNEL_EDGESEL_NONE       (0 << EVSYS_CHANNEL_EDGESEL_SHIFT) /* No event output */
#  define EVSYS_CHANNEL_EDGESEL_RISING     (1 << EVSYS_CHANNEL_EDGESEL_SHIFT) /* Detect on rising edge */
#  define EVSYS_CHANNEL_EDGESEL_FALLING    (2 << EVSYS_CHANNEL_EDGESEL_SHIFT) /* Detect on falling edge */
#  define EVSYS_CHANNEL_EDGESEL_BOTH       (3 << EVSYS_CHANNEL_EDGESEL_SHIFT) /* Detect on both edges */
#define EVSYS_CHANNEL_RUNSTDBY             (1 << 14) /* Bit 14: Run in standby */
#define EVSYS_CHANNEL_ONDEMAND             (1 << 15) /* Bit 15: Generic clock on-demand */

/* User registers */

#define EVSYS_USER_CHANNEL_SHIFT           (0)       /* Bits 0-5: Channel number */
#define EVSYS_USER_CHANNEL_MASK            (63 << EVSYS_USER_CHANNEL_SHIFT)
#  define EVSYS_USER_CHANNEL_NONE          (0 << EVSYS_USER_CHANNEL_SHIFT) /* No channel output selected */
#  define EVSYS_USER_CHANNEL(n)            ((uint32_t)((n)+1) << EVSYS_USER_CHANNEL_SHIFT) /* Channel n */

/* User multiplexer numbers  ****************************************************************/

#define EVSYS_USER_PORT_EV0                0         /* Event 0 */
#define EVSYS_USER_PORT_EV1                1         /* Event 1 */
#define EVSYS_USER_PORT_EV2                2         /* Event 2 */
#define EVSYS_USER_PORT_EV3                3         /* Event 3 */
#define EVSYS_USER_DMAC_CH0                4         /* DMAC Channel 0 */
#define EVSYS_USER_DMAC_CH1                5         /* DMAC Channel 1 */
#define EVSYS_USER_DMAC_CH2                6         /* DMAC Channel 2 */
#define EVSYS_USER_DMAC_CH3                7         /* DMAC Channel 3 */
#define EVSYS_USER_DMAC_CH4                8         /* DMAC Channel 4 */
#define EVSYS_USER_DMAC_CH5                9         /* DMAC Channel 5 */
#define EVSYS_USER_DMAC_CH6                10        /* DMAC Channel 6 */
#define EVSYS_USER_DMAC_CH7                11        /* DMAC Channel 7 */
#define EVSYS_USER_TCC0_EV0                12        /* TCC0 Event 0 */
#define EVSYS_USER_TCC0_EV1                13        /* TCC0 Event 1 */
#define EVSYS_USER_TCC0_MC0                14        /* TCC0 Match/Capture 0 */
#define EVSYS_USER_TCC0_MC1                15        /* TCC0 Match/Capture 1 */
#define EVSYS_USER_TCC0_MC2                16        /* TCC0 Match/Capture 2 */
#define EVSYS_USER_TCC0_MC3                17        /* TCC0 Match/Capture 3 */
#define EVSYS_USER_TCC1_EV0                18        /* TCC1 Event 0 */
#define EVSYS_USER_TCC1_EV1                19        /* TCC1 Event 1 */
#define EVSYS_USER_TCC1_MC0                20        /* TCC1 Match/Capture 0 */
#define EVSYS_USER_TCC1_MC1                21        /* TCC1 Match/Capture 1 */
#define EVSYS_USER_TCC2_EV0                22        /* TCC2 Event 0 */
#define EVSYS_USER_TCC2_EV1                23        /* TCC2 Event 1 */
#define EVSYS_USER_TCC2_MC0                24        /* TCC2 Match/Capture 0 */
#define EVSYS_USER_TCC2_MC1                25        /* TCC2 Match/Capture 1 */
#define EVSYS_USER_TC0                     26        /* TC0 */
#define EVSYS_USER_TC1                     27        /* TC1 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_CHIP_SAML_EVSYS_H */
