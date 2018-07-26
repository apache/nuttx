/********************************************************************************************
 * arch/arm/src/samd5e5/chip/sam_evsys.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_EVSYS_H
#define __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_EVSYS_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define SAM_EVSYS_NCHANNELS                32      /* 0-31 */
#define SAM_EVSYS_NUSER                    67      /* 0-66 */

/* EVSYS register offsets *******************************************************************/

#define SAM_EVSYS_CTRLA_OFFSET             0x0000  /* Control register */
#define SAM_EVSYS_SWEVT_OFFSET             0x0004  /* Software event register */
#define SAM_EVSYS_PRICTRL_OFFSET           0x0008  /* Software event register */
#define SAM_EVSYS_INTPEND_OFFSET           0x0010  /* Channel pending interrupt register */
#define SAM_EVSYS_INTSTATUS_OFFSET         0x0014  /* Channel pending interrupt register */
#define SAM_EVSYS_BUSYCH_OFFSET            0x0018  /* Busy channels register */
#define SAM_EVSYS_READYUSR_OFFSET          0x001c  /* Busy channels register */

#define SAM_EVSYS_CHOFFSET(n)              (0x0020 + ((n) << 3)) /* Channel registers */
#  define SAM_EVSYS_CHANNEL_OFFSET         0x0000  /* Channel control register */
#  define SAM_EVSYS_CHINTENCLR_OFFSET      0x0004  /* Channl interrupt clear register */
#  define SAM_EVSYS_CHINTENSET_OFFSET      0x0005  /* Channel interrupt enable register */
#  define SAM_EVSYS_CHINTFLAG_OFFSET       0x0006  /* Channel interrupt status register */
#  define SAM_EVSYS_CHSTATUS_OFFSET        0x0007  /* Channel status register */

#define SAM_EVSYS_USER_OFFSET(n)           (0x0120 + ((m)) /* User registers */

/* EVSYS register addresses *****************************************************************/

#define SAM_EVSYS_CTRLA                    (SAM_EVSYS_BASE + SAM_EVSYS_CTRLA_OFFSET)
#define SAM_EVSYS_SWEVT                    (SAM_EVSYS_BASE + SAM_EVSYS_SWEVT_OFFSET)
#define SAM_EVSYS_PRICTRL                  (SAM_EVSYS_BASE + SAM_EVSYS_PRICTRL_OFFSET)
#define SAM_EVSYS_INTPEND                  (SAM_EVSYS_BASE + SAM_EVSYS_INTPEND_OFFSET)
#define SAM_EVSYS_INTSTATUS                (SAM_EVSYS_BASE + SAM_EVSYS_INTSTATUS_OFFSET)
#define SAM_EVSYS_BUSYCH                   (SAM_EVSYS_BASE + SAM_EVSYS_BUSYCH_OFFSET)
#define SAM_EVSYS_READYUSR                 (SAM_EVSYS_BASE + SAM_EVSYS_READYUSR_OFFSET)

#define SAM_EVSYS_CHBASEBASE(n)            (SAM_EVSYS_BASE + SAM_EVSYS_CHOFFSET(n))
#  define SAM_EVSYS_CHANNEL(n)             (SAM_EVSYS_CHBASEBASE(n) + SAM_EVSYS_CHANNEL_OFFSET)
#  define SAM_EVSYS_CHINTENCLR(n)          (SAM_EVSYS_CHBASEBASE(n) + SAM_EVSYS_CHINTENCLR_OFFSET)
#  define SAM_EVSYS_CHINTENSET(n)          (SAM_EVSYS_CHBASEBASE(n) + SAM_EVSYS_CHINTENSET_OFFSET)
#  define SAM_EVSYS_CHINTFLAG(n)           (SAM_EVSYS_CHBASEBASE(n) + SAM_EVSYS_CHINTFLAG_OFFSET)
#  define SAM_EVSYS_CHSTATUS(n)            (SAM_EVSYS_CHBASEBASE(n) + SAM_EVSYS_CHSTATUS_OFFSET

#define SAM_EVSYS_USER(n)                  (SAM_EVSYS_BASE +  SAM_EVSYS_USER_OFFSET(n))

/* EVSYS register bit definitions ***********************************************************/

/* Control register */

#define EVSYS_CTRLA_SWRST                   (1 << 0)  /* Bit 0: Software reset */

/* Software event register */

#define EVSYS_SWEVT_CHAN(n)                 (1 << (n)) /* Bit n: Channel n software selection */

/* Software event register */

#define EVSYS_PRICTRL_PRI_SHIFT             (0)  /* Bist 0-4: Channel priority number */
#define EVSYS_PRICTRL_PRI_MASK              (31 << EVSYS_PRICTRL_PRI_SHIFT)
#  define EVSYS_PRICTRL_PRI(n)              ((uint8_t)(n) << EVSYS_PRICTRL_PRI_SHIFT)
#define EVSYS_PRICTRL_RREN                  (1 << 7)  /* Bit 7: Round-robin scheduling enable */

/* Channel pending interrupt register */

#define EVSYS_INTPEND_ID_SHIFT              (0)       /* Bits 0-4:  Channel ID */
#define EVSYS_INTPEND_ID_MASK               (31 << EVSYS_INTPEND_ID_SHIFT)
#  define EVSYS_INTPEND_ID(n)               ((uint16_t)(n) << EVSYS_INTPEND_ID_SHIFT)
#define EVSYS_INTPEND_OVR                   (1 << 8)  /* Bit 8:  Channel Overrun */
#define EVSYS_INTPEND_EVD                   (1 << 9)  /* Bit 9:  Channel Event Detected */
#define EVSYS_INTPEND_READY                 (1 << 14) /* Bit 14: Ready */
#define EVSYS_INTPEND_BUSY                  (1 << 15) /* Bit 15: Busy */

/* Channel pending interrupt register */

#define EVSYS_INTSTATUS_CHAN(n)             (1 << (n)) /* Bit n: Channel n pending interrupt */

/* Busy channels register */

#define EVSYS_BUSYCH_CHAN(n)                (1 << (n)) /* Bit n: Busy channel n */

/* Busy channels register */

#define EVSYS_READYUSR_CHAN(n)              (1 << (n)) /* Bit n: Ready user for channel n */

/* Channel control register (see event generator selections below) */

#define EVSYS_CHANNEL_EVGEN_SHIFT           (0)       /* Bits 0-7: Event generator selection */
#define EVSYS_CHANNEL_EVGEN_MASK            (0xff << EVSYS_CHANNEL_EVGEN_SHIFT)
#  define EVSYS_CHANNEL_EVGEN(n)            ((uint32_t)(n) << EVSYS_CHANNEL_EVGEN_SHIFT)
#define EVSYS_CHANNEL_PATH_SHIFT            (8)       /* Bits 8-9: Path Selection */
#define EVSYS_CHANNEL_PATH_MASK             (3 << EVSYS_CHANNEL_PATH_SHIFT)
#  define EVSYS_CHANNEL_PATH_SYNCH          (0 << EVSYS_CHANNEL_PATH_SHIFT) /* Synchronous path */
#  define EVSYS_CHANNEL_PATH_RESYNCH        (1 << EVSYS_CHANNEL_PATH_SHIFT) /* Resynchronized path */
#  define EVSYS_CHANNEL_PATH_ASYNCH         (2 << EVSYS_CHANNEL_PATH_SHIFT) /* Asynchronous path */
#define EVSYS_CHANNEL_EDGSEL_SHIFT          (10)      /* Bits 10-11: Edge detection selection */
#define EVSYS_CHANNEL_EDGSEL_MASK           (3 << EVSYS_CHANNEL_EDGSEL_SHIFT)
#  define EVSYS_CHANNEL_EDGSEL_NONE         (0 << EVSYS_CHANNEL_EDGSEL_SHIFT) /* No event output */
#  define EVSYS_CHANNEL_EDGSEL_ RISING      (1 << EVSYS_CHANNEL_EDGSEL_SHIFT) /* Event on rising edge */
#  define EVSYS_CHANNEL_EDGSEL_FALLING      (2 << EVSYS_CHANNEL_EDGSEL_SHIFT) /* Event on falling edge */
#  define EVSYS_CHANNEL_EDGSEL_BOTH         (3 << EVSYS_CHANNEL_EDGSEL_SHIFT) /* Event on both edges */
#define EVSYS_CHANNEL_RUNSTDBY              (1 << 14) /* Bit 14: Run in standby */
#define EVSYS_CHANNEL_ONDEMAND              (1 << 15) /* Bit 15: Generic clock on demand */

/* Channel interrupt clear register, Channel interrupt enable register, and Channel interrupt
 * status register
 */

#define EVSYS_CHINT_OVR                     (1 << 0)  /* Bit 0: Channel overrun */
#define EVSYS_CHINT_EVD                     (1 << 1)  /* Bit 1: Channel event detected */

/* Channel status register */

#define EVSYS_CHSTATUS_RDYUSR               (1 << 0)  /* Bit 0: Ready user */
#define EVSYS_CHSTATUS_BUSYCH               (1 << 1)  /* Bit 1: Busy channel */

/* User registers (8-bit channel number.  See user multiplexor numbers below */

/* Event generator channel event selection **************************************************/

#define EVSYS_EVENT_NONE                    0x00  /* No event generator selected */
#define EVSYS_EVENT_OSCCTRL_XOSC_FAIL0      0x01  /* XOSC fail detection 0 */
#define EVSYS_EVENT_OSCCTRL_XOSC_FAIL1      0x02  /* XOSC fail detection 1 */
#define EVSYS_EVENT_OSC32KCTRL_XOSC32K_FAIL 0x03  /* XOSC32K fail detection */
#define EVSYS_EVENT_RTC_PER0                0x04  /* RTC period 0 */
#define EVSYS_EVENT_RTC_PER1                0x05  /* RTC period 1 */
#define EVSYS_EVENT_RTC_PER2                0x06  /* RTC period 2 */
#define EVSYS_EVENT_RTC_PER3                0x07  /* RTC period 3 */
#define EVSYS_EVENT_RTC_PER4                0x08  /* RTC period 4 */
#define EVSYS_EVENT_RTC_PER5                0x09  /* RTC period 5 */
#define EVSYS_EVENT_RTC_PER6                0x0a  /* RTC period 6 */
#define EVSYS_EVENT_RTC_PER7                0x0b  /* RTC period 7 */
#define EVSYS_EVENT_RTC_CMP0                0x0c  /* RTC comparison 0 */
#define EVSYS_EVENT_RTC_CMP1                0x0d  /* RTC comparison 0 */
#define EVSYS_EVENT_RTC_CMP2                0x0e  /* RTC comparison 0 */
#define EVSYS_EVENT_RTC_CMP3                0x0f  /* RTC comparison 0 */
#define EVSYS_EVENT_RTC_TAMPER              0x10  /* RTC tamper detection */
#define EVSYS_EVENT_RTC_OVF                 0x11  /* RTC overflow */
#define EVSYS_EVENT_EIC_EXTINT0             0x12  /* EIC external interrupt 0 */
#define EVSYS_EVENT_EIC_EXTINT1             0x13  /* EIC external interrupt 1 */
#define EVSYS_EVENT_EIC_EXTINT2             0x14  /* EIC external interrupt 2 */
#define EVSYS_EVENT_EIC_EXTINT3             0x15  /* EIC external interrupt 3 */
#define EVSYS_EVENT_EIC_EXTINT4             0x16  /* EIC external interrupt 4 */
#define EVSYS_EVENT_EIC_EXTINT5             0x17  /* EIC external interrupt 5 */
#define EVSYS_EVENT_EIC_EXTINT6             0x18  /* EIC external interrupt 6 */
#define EVSYS_EVENT_EIC_EXTINT7             0x19  /* EIC external interrupt 7 */
#define EVSYS_EVENT_EIC_EXTINT8             0x1a  /* EIC external interrupt 8 */
#define EVSYS_EVENT_EIC_EXTINT9             0x1b  /* EIC external interrupt 9 */
#define EVSYS_EVENT_EIC_EXTINT10            0x1c  /* EIC external interrupt 10 */
#define EVSYS_EVENT_EIC_EXTINT11            0x1d  /* EIC external interrupt 11 */
#define EVSYS_EVENT_EIC_EXTINT12            0x1e  /* EIC external interrupt 12 */
#define EVSYS_EVENT_EIC_EXTINT13            0x1f  /* EIC external interrupt 13 */
#define EVSYS_EVENT_EIC_EXTINT14            0x20  /* EIC external interrupt 14 */
#define EVSYS_EVENT_EIC_EXTINT15            0x21  /* EIC external interrupt 15 */
#define EVSYS_EVENT_DMAC_CH0                0x22  /* DMA channel 0 */
#define EVSYS_EVENT_DMAC_CH1                0x23  /* DMA channel 0 */
#define EVSYS_EVENT_DMAC_CH2                0x24  /* DMA channel 0 */
#define EVSYS_EVENT_DMAC_CH3                0x25  /* DMA channel 0 */
#define EVSYS_EVENT_PAC_ACCERR              0x26  /* PAC Acc. error */
#define EVSYS_EVENT_TCC0_OVF                0x29  /* TCC0 Overflow */
#define EVSYS_EVENT_TCC0_TRG                0x2a  /* TCC0 Trigger Event */
#define EVSYS_EVENT_TCC0_CNT                0x2b  /* TCC0 Counter */
#define EVSYS_EVENT_TCC0_MC0                0x2c  /* TCC0 Match/Compare 0 */
#define EVSYS_EVENT_TCC0_MC1                0x2d  /* TCC0 Match/Compare 1 */
#define EVSYS_EVENT_TCC0_MC2                0x2e  /* TCC0 Match/Compare 2 */
#define EVSYS_EVENT_TCC0_MC3                0x2f  /* TCC0 Match/Compare 3 */
#define EVSYS_EVENT_TCC0_MC4                0x30  /* TCC0 Match/Compare 4 */
#define EVSYS_EVENT_TCC0_MC5                0x31  /* TCC0 Match/Compare 5 */
#define EVSYS_EVENT_TCC1_OVF                0x32  /* TCC1 Overflow */
#define EVSYS_EVENT_TCC1_TRG                0x33  /* TCC1 Trigger Event */
#define EVSYS_EVENT_TCC1_CNT                0x34  /* TCC1 Counter */
#define EVSYS_EVENT_TCC1_MC0                0x35  /* TCC1 Match/Compare 0 */
#define EVSYS_EVENT_TCC1_MC1                0x36  /* TCC1 Match/Compare 1 */
#define EVSYS_EVENT_TCC1_MC2                0x37  /* TCC1 Match/Compare 2 */
#define EVSYS_EVENT_TCC1_MC3                0x38  /* TCC1 Match/Compare 3 */
#define EVSYS_EVENT_TCC2_OVF                0x39  /* TCC2 Overflow */
#define EVSYS_EVENT_TCC2                    0x3a  /* TCC2_TRG  Trigger Event */
#define EVSYS_EVENT_TCC2_CNT                0x3b  /* TCC2 Counter */
#define EVSYS_EVENT_TCC2_MC0                0x3c  /* TCC2 Match/Compare 0 */
#define EVSYS_EVENT_TCC2_MC1                0x3d  /* TCC2 Match/Compare 1 */
#define EVSYS_EVENT_TCC2_MC2                0x3e  /* TCC2 Match/Compare 2 */
#define EVSYS_EVENT_TCC3_OVF                0x3f  /* TCC3 Overflow */
#define EVSYS_EVENT_TCC3_TRG                0x40  /* TCC3 Trigger Event */
#define EVSYS_EVENT_TCC3_CNT                0x41  /* TCC3 Counter */
#define EVSYS_EVENT_TCC3_MC0                0x42  /* TCC3 Match/Compare 0 */
#define EVSYS_EVENT_TCC3_MC1                0x43  /* TCC3 Match/Compare 1 */
#define EVSYS_EVENT_TCC4_OVF                0x44  /* TCC4 Overflow */
#define EVSYS_EVENT_TCC4_TRG                0x45  /* TCC4 Trigger Event */
#define EVSYS_EVENT_TCC4_CNT                0x46  /* TCC4 Counter */
#define EVSYS_EVENT_TCC4_MC0                0x47  /* TCC4 Match/Compare 0 */
#define EVSYS_EVENT_TCC4_MC1                0x48  /* TCC4 Match/Compare 1 */
#define EVSYS_EVENT_TC0_OVF                 0x49  /* TC0 Overflow */
#define EVSYS_EVENT_TC0_MC0                 0x4a  /* TC0 Match/Compare 0 */
#define EVSYS_EVENT_TC0_MC1                 0x4b  /* TC0 Match/Compare 1 */
#define EVSYS_EVENT_TC1_OVF                 0x4c  /* TC1 Overflow */
#define EVSYS_EVENT_TC1_MC0                 0x4d  /* TC1 Match/Compare 0 */
#define EVSYS_EVENT_TC1_MC1                 0x4e  /* TC1 Match/Compare 1 */
#define EVSYS_EVENT_TC2_OVF                 0x4f  /* TC2 Overflow */
#define EVSYS_EVENT_TC2_MC0                 0x50  /* TC2 Match/Compare 0 */
#define EVSYS_EVENT_TC2_MC1                 0x51  /* TC2 Match/Compare 1 */
#define EVSYS_EVENT_TC3_OVF                 0x52  /* TC3 Overflow */
#define EVSYS_EVENT_TC3_MC0                 0x53  /* TC3 Match/Compare 0 */
#define EVSYS_EVENT_TC3_MC1                 0x54  /* TC3 Match/Compare 1 */
#define EVSYS_EVENT_TC4_OVF                 0x55  /* TC4 Overflow */
#define EVSYS_EVENT_TC4_MC0                 0x56  /* TC4 Match/Compare 0 */
#define EVSYS_EVENT_TC4_MC1                 0x57  /* TC4 Match/Compare 1 */
#define EVSYS_EVENT_TC5_OVF                 0x58  /* TC5 Overflow */
#define EVSYS_EVENT_TC5_MC0                 0x59  /* TC5 Match/Compare 0 */
#define EVSYS_EVENT_TC5_MC1                 0x5a  /* TC5 Match/Compare 1 */
#define EVSYS_EVENT_TC6_OVF                 0x5b  /* TC6 Overflow */
#define EVSYS_EVENT_TC6_MC0                 0x5c  /* TC6 Match/Compare 0 */
#define EVSYS_EVENT_TC6_MC1                 0x5d  /* TC6 Match/Compare 1 */
#define EVSYS_EVENT_TC7_OVF                 0x5e  /* TC7 Overflow */
#define EVSYS_EVENT_TC7_MC0                 0x5f  /* TC7 Match/Compare 0 */
#define EVSYS_EVENT_TC7_MC1                 0x60  /* TC7 Match/Compare 1 */
#define EVSYS_EVENT_PDEC_OVF                0x61  /* PDEC Overflow */
#define EVSYS_EVENT_PDEC_ERR                0x62  /* PDEC Error */
#define EVSYS_EVENT_PDEC_DIR                0x63  /* PDEC Direction */
#define EVSYS_EVENT_PDEC_VLC                0x64  /* PDEC VLC */
#define EVSYS_EVENT_PDEC_MC0                0x65  /* PDEC MC0 */
#define EVSYS_EVENT_PDEC_MC1                0x66  /* PDEC MC1 */
#define EVSYS_EVENT_ADC0_RESRDY             0x67  /* ADC0 RESRDY */
#define EVSYS_EVENT_ADC0_WINMON             0x68  /* ADC0 Window Monitor */
#define EVSYS_EVENT_ADC1_RESRDY             0x69  /* ADC1 RESRDY */
#define EVSYS_EVENT_ADC1_WINMON             0x6a  /* ADC1 Window Monitor */
#define EVSYS_EVENT_AC_COMP0                0x6b  /* AC Comparator 0 */
#define EVSYS_EVENT_AC_COMP1                0x6c  /* AC Comparator 1 */
#define EVSYS_EVENT_AC_WIN                  0x6d  /* AC0 Window */
#define EVSYS_EVENT_DAC_EMPTY0              0x6e  /* DAC empty 0 */
#define EVSYS_EVENT_DAC_EMPTY1              0x6f  /* DAC empty 1 */
#define EVSYS_EVENT_DAC_RESRDY0             0x70  /* DAC RSRDY 0 */
#define EVSYS_EVENT_DAC_RESRDY1             0x71  /* DAC RSRDY 1 */
#define EVSYS_EVENT_GMAC_TSU_CMP            0x72  /* GMAC Timestamp CMP */
#define EVSYS_EVENT_TRNG_READY              0x73  /* TRNG ready */
#define EVSYS_EVENT_CCL_LUTOUT0             0x74  /* CCL LUTOUT 0 */
#define EVSYS_EVENT_CCL_LUTOUT1             0x75  /* CCL LUTOUT 1 */
#define EVSYS_EVENT_CCL_LUTOUT2             0x76  /* CCL LUTOUT 2 */
#define EVSYS_EVENT_CCL_LUTOUT3             0x77  /* CCL LUTOUT 3 */

/* User multiplexer numbers  ****************************************************************/
/* These are indices that may be used with the SAM_EVSYS_USER(n) macro to get the address of
 * the correct user register.
 */

#define EVSYS_USER_RTC_TAMPER               0   /* RTC Tamper A */
#define EVSYS_USER_PORT_EV0                 1   /* Port 0 event */
#define EVSYS_USER_PORT_EV1                 2   /* Port 1 event */
#define EVSYS_USER_PORT_EV2                 3   /* Port 2 event */
#define EVSYS_USER_PORT_EV3                 4   /* Port 3 event */
#define EVSYS_USER_DMAC_CH0                 5   /* DMA channel 0 event */
#define EVSYS_USER_DMAC_CH1                 6   /* DMA channel 1 event */
#define EVSYS_USER_DMAC_CH2                 7   /* DMA channel 2 event */
#define EVSYS_USER_DMAC_CH3                 8   /* DMA channel 3 event */
#define EVSYS_USER_DMAC_CH4                 9   /* DMA channel 4 event */
#define EVSYS_USER_DMAC_CH5                 10  /* DMA channel 5 event */
#define EVSYS_USER_DMAC_CH6                 11  /* DMA channel 6 event */
#define EVSYS_USER_DMAC_CH7                 12  /* DMA channel 7 event */
#define EVSYS_USER_CM4_TRACE_START          14  /* CM4 trace start */
#define EVSYS_USER_CM4_TRACE_STOP           15  /* CM4 trace stop */
#define EVSYS_USER_CM4_TRACE_TRIG           16  /* CM4 trace trigger */
#define EVSYS_USER_TCC0_EV0                 17  /* TCC0 EV0 */
#define EVSYS_USER_TCC0_EV1                 18  /* TCC0 EV1 */
#define EVSYS_USER_TCC0_MC0                 19  /* TCC0 MC0 */
#define EVSYS_USER_TCC0_MC1                 20  /* TCC0 MC1 */
#define EVSYS_USER_TCC0_MC2                 21  /* TCC0 MC2 */
#define EVSYS_USER_TCC0_MC3                 22  /* TCC0 MC3 */
#define EVSYS_USER_TCC0_MC4                 23  /* TCC0 MC4 */
#define EVSYS_USER_TCC0_MC5                 24  /* TCC0 MC5 */
#define EVSYS_USER_TCC1_EV0                 25  /* TCC1 EV0 */
#define EVSYS_USER_TCC1_EV1                 26  /* TCC1 EV1 */
#define EVSYS_USER_TCC1_MC0                 27  /* TCC1 MC0 */
#define EVSYS_USER_TCC1_MC1                 28  /* TCC1 MC1 */
#define EVSYS_USER_TCC1_MC2                 29  /* TCC1 MC2 */
#define EVSYS_USER_TCC1_MC3                 30  /* TCC1 MC3 */
#define EVSYS_USER_TCC2_EV0                 31  /* TCC2 EV0 */
#define EVSYS_USER_TCC2_EV1                 32  /* TCC2 EV1 */
#define EVSYS_USER_TCC2_MC0                 33  /* TCC2 MC0 */
#define EVSYS_USER_TCC2_MC1                 34  /* TCC2 MC1 */
#define EVSYS_USER_TCC2_MC2                 35  /* TCC2 MC2 */
#define EVSYS_USER_TCC3_EV0                 36  /* TCC3 EV0 */
#define EVSYS_USER_TCC3_EV1                 37  /* TCC3 EV1 */
#define EVSYS_USER_TCC3_MC0                 38  /* TCC3 MC0 */
#define EVSYS_USER_TCC3_MC1                 39  /* TCC3 MC1 */
#define EVSYS_USER_TCC4_EV0                 40  /* TCC4 EV0 */
#define EVSYS_USER_TCC4_EV1                 41  /* TCC4 EV1 */
#define EVSYS_USER_TCC4_MC0                 42  /* TCC4 MC0 */
#define EVSYS_USER_TCC4_MC1                 43  /* TCC4 MC1 */
#define EVSYS_USER_TC0_EVU                  44  /* TC0 EVU */
#define EVSYS_USER_TC1_EVU                  45  /* TC1 EVU */
#define EVSYS_USER_TC2_EVU                  46  /* TC2 EVU */
#define EVSYS_USER_TC3_EVU                  47  /* TC3 EVU */
#define EVSYS_USER_TC4_EVU                  48  /* TC4 EVU */
#define EVSYS_USER_TC5_EVU                  49  /* TC5 EVU */
#define EVSYS_USER_TC6_EVU                  50  /* TC6 EVU */
#define EVSYS_USER_TC7_EVU                  51  /* TC7 EVU */
#define EVSYS_USER_PDEC_EVU0                52  /* PDEC EVU 0 */
#define EVSYS_USER_PDEC_EVU1                53  /* PDEC EVU 1 */
#define EVSYS_USER_PDEC_EVU2                54  /* PDEC EVU 2 */
#define EVSYS_USER_ADC0_START               55  /* ADC0 start conversion */
#define EVSYS_USER_ADC0_SYNC                56  /* SYNC Flush ADC0 */
#define EVSYS_USER_ADC1_START               57  /* ADC1 start conversion */
#define EVSYS_USER_ADC1_SYNC                58  /* SYNC Flush ADC1 */
#define EVSYS_USER_AC_SOC0                  59  /* AC SOC 0 */
#define EVSYS_USER_AC_SOC1                  60  /* AC SOC 1 */
#define EVSYS_USER_DAC_START0               61  /* DAC0 start conversion */
#define EVSYS_USER_DAC_START1               62  /* DAC1 start conversion */
#define EVSYS_USER_CCL_LUTIN0               63  /* CCL input 0 */
#define EVSYS_USER_CCL_LUTIN1               64  /* CCL input 1 */
#define EVSYS_USER_CCL_LUTIN2               65  /* CCL input 2 */
#define EVSYS_USER_CCL_LUTIN3               66  /* CCL input 3 */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_EVSYS_H */
