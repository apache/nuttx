/****************************************************************************
 * arch/arm/src/lc823450/chip/lc823450_vectors.h
 *
 *   Copyright (C) 2014-2017 Sony Corporation. All rights reserved.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* This file is included by lc823450_vectors.S.  It provides the macro VECTOR
 * that supplies a LC823450 vector in terms of a (lower-case) ISR label and 
 * an (upper-case) IRQ number as defined in arch/arm/include/lc823450/irq.h.
 * lc823450_vectors.S will defined the VECTOR in different ways in order to
 * generate the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the
 * following definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 82


#else
VECTOR(lc823450_ctxm3_00, LC823450_IRQ_CTXM3_00)       /* Vector 16+0: CortexM3_00 interrupt */
VECTOR(lc823450_ctxm3_01, LC823450_IRQ_CTXM3_01)       /* Vector 16+1: CortexM3_01 interrupt */
VECTOR(lc823450_ctxm3_02, LC823450_IRQ_CTXM3_02)       /* Vector 16+2: CortexM3_02 interrupt */
VECTOR(lc823450_ctxm3_03, LC823450_IRQ_CTXM3_03)       /* Vector 16+3: CortexM3_03 interrupt */
VECTOR(lc823450_ctxm3_10, LC823450_IRQ_CTXM3_10)       /* Vector 16+4: CortexM3_00 interrupt */
VECTOR(lc823450_ctxm3_11, LC823450_IRQ_CTXM3_11)       /* Vector 16+5: CortexM3_01 interrupt */
VECTOR(lc823450_ctxm3_12, LC823450_IRQ_CTXM3_12)       /* Vector 16+6: CortexM3_02 interrupt */
VECTOR(lc823450_ctxm3_13, LC823450_IRQ_CTXM3_13)       /* Vector 16+7: CortexM3_03 interrupt */
VECTOR(lc823450_lpdsp0, LC823450_IRQ_LPDSP0)           /* Vector 16+8: LPDSP0 interrupt */
VECTOR(lc823450_lpdsp1, LC823450_IRQ_LPDSP1)           /* Vector 16+9: LPDSP1 interrupt */
VECTOR(lc823450_lpdsp2, LC823450_IRQ_LPDSP2)           /* Vector 16+10: LPDSP2 interrupt */
VECTOR(lc823450_lpdsp3, LC823450_IRQ_LPDSP3)           /* Vector 16+11: LPDSP3 interrupt */
VECTOR(lc823450_wdt0, LC823450_IRQ_WDT0)               /* Vector 16+12: WatchDogTimer0 interrupt */
VECTOR(lc823450_wdt1, LC823450_IRQ_WDT1)               /* Vector 16+13: WatchDogTimer1 interrupt */
VECTOR(lc823450_wdt2, LC823450_IRQ_WDT2)               /* Vector 16+14: WatchDogTimer2 interrupt */
VECTOR(lc823450_btimer0, LC823450_IRQ_BTIMER0)         /* Vector 16+15: BasicTimer0 interrupt */
VECTOR(lc823450_btimer1, LC823450_IRQ_BTIMER1)         /* Vector 16+16: BasicTimer0 interrupt */
VECTOR(lc823450_btimer2, LC823450_IRQ_BTIMER2)         /* Vector 16+17: BasicTimer0 interrupt */
VECTOR(lc823450_mtimer00, LC823450_IRQ_MTIMER00)       /* Vector 16+18: MultipleTimer00 interrupt */
VECTOR(lc823450_mtimer01, LC823450_IRQ_MTIMER01)       /* Vector 16+19: MultipleTimer01 interrupt */
VECTOR(lc823450_mtimer10, LC823450_IRQ_MTIMER10)       /* Vector 16+20: MultipleTimer10 interrupt */
VECTOR(lc823450_mtimer11, LC823450_IRQ_MTIMER11)       /* Vector 16+21: MultipleTimer11 interrupt */
VECTOR(lc823450_mtimer20, LC823450_IRQ_MTIMER20)       /* Vector 16+22: MultipleTimer20 interrupt */
VECTOR(lc823450_mtimer21, LC823450_IRQ_MTIMER21)       /* Vector 16+23: MultipleTimer21 interrupt */
VECTOR(lc823450_mtimer30, LC823450_IRQ_MTIMER30)       /* Vector 16+24: MultipleTimer30 interrupt */
VECTOR(lc823450_mtimer31, LC823450_IRQ_MTIMER31)       /* Vector 16+25: MultipleTimer31 interrupt */
VECTOR(lc823450_ehci, LC823450_IRQ_EHCI)               /* Vector 16+26: USB HOST EHCI interrupt */
VECTOR(lc823450_ohci, LC823450_IRQ_OHCI)               /* Vector 16+27: USB HOST OHCI interrupt */
VECTOR(lc823450_serflash, LC823450_IRQ_SERFLASH)       /* Vector 16+28: USB HOST OHCI interrupt */
VECTOR(lc823450_dmac, LC823450_IRQ_DMAC)               /* Vector 16+29: DMA Controller interrupt */
VECTOR(lc823450_sdcsync0, LC823450_IRQ_SDCSYNC0)       /* Vector 16+30: SDCardSync0 interrupt */
VECTOR(lc823450_sdcsync1, LC823450_IRQ_SDCSYNC1)       /* Vector 16+31: SDCardSync1 interrupt */
VECTOR(lc823450_sdcsync2, LC823450_IRQ_SDCSYNC2)       /* Vector 16+32: SDCardSync2 interrupt */
VECTOR(lc823450_sdcasync0, LC823450_IRQ_SDCASYNC0)     /* Vector 16+33: SDCardAsync0 interrupt */
VECTOR(lc823450_sdcasync1, LC823450_IRQ_SDCASYNC1)     /* Vector 16+34: SDCardAsync1 interrupt */
VECTOR(lc823450_sdcasync2, LC823450_IRQ_SDCASYNC2)     /* Vector 16+35: SDCardAsync2 interrupt */
VECTOR(lc823450_memstick, LC823450_IRQ_MEMSTICK)       /* Vector 16+36: MemoryStick interrupt */
VECTOR(lc823450_memstickins, LC823450_IRQ_MEMSTICKINS) /* Vector 16+37: MemoryStick ins interrupt */
VECTOR(lc823450_dspcmd, LC823450_IRQ_DSPCMD)           /* Vector 16+38: DSP cmd interface interrupt */
VECTOR(lc823450_adc, LC823450_IRQ_ADC)                 /* Vector 16+39: AD Converter interrupt */
VECTOR(lc823450_sio, LC823450_IRQ_SIO)                 /* Vector 16+40: SIO interrupt */
VECTOR(lc823450_i2c0, LC823450_IRQ_I2C0)               /* Vector 16+41: I2C0 interrupt */
VECTOR(lc823450_i2c1, LC823450_IRQ_I2C1)               /* Vector 16+42: I2C1 interrupt */
VECTOR(lc823450_uart0, LC823450_IRQ_UART0)             /* Vector 16+43: UART0 interrupt */
VECTOR(lc823450_uart1, LC823450_IRQ_UART1)             /* Vector 16+44: UART1 interrupt */
VECTOR(lc823450_uart2, LC823450_IRQ_UART2)             /* Vector 16+45: UART2 interrupt */
VECTOR(lc823450_rtc, LC823450_IRQ_RTC)                 /* Vector 16+46: RTC interrupt */
VECTOR(lc823450_rtckey, LC823450_IRQ_RTCKEY)           /* Vector 16+47: RTCKEY interrupt */
VECTOR(lc823450_audiobuf0, LC823450_IRQ_AUDIOBUF0)     /* Vector 16+48: AudioBuffer0 interrupt */
VECTOR(lc823450_audiobuf1, LC823450_IRQ_AUDIOBUF1)     /* Vector 16+49: AudioBuffer1 interrupt */
VECTOR(lc823450_audiobuf2, LC823450_IRQ_AUDIOBUF2)     /* Vector 16+50: AudioBuffer2 interrupt */
VECTOR(lc823450_audiobuf3, LC823450_IRQ_AUDIOBUF3)     /* Vector 16+51: AudioBuffer3 interrupt */
VECTOR(lc823450_audiostat0, LC823450_IRQ_AUDIOSTAT0)   /* Vector 16+52: AudioStatus0 interrupt */
VECTOR(lc823450_audiostat1, LC823450_IRQ_AUDIOSTAT1)   /* Vector 16+53: AudioStatus1 interrupt */
VECTOR(lc823450_audiostat2, LC823450_IRQ_AUDIOSTAT2)   /* Vector 16+54: AudioStatus2 interrupt */
VECTOR(lc823450_audiostat3, LC823450_IRQ_AUDIOSTAT3)   /* Vector 16+55: AudioStatus3 interrupt */
VECTOR(lc823450_audiotm0, LC823450_IRQ_AUDIOTM0)       /* Vector 16+56: AudioTimer0 interrupt */
VECTOR(lc823450_audiotm1, LC823450_IRQ_AUDIOTM1)       /* Vector 16+57: AudioTimer1 interrupt */
VECTOR(lc823450_usbdev, LC823450_IRQ_USBDEV)           /* Vector 16+58: USB Device interrupt */
VECTOR(lc823450_extint0, LC823450_IRQ_EXTINT0)         /* Vector 16+59: ExternalINT0 interrupt */
VECTOR(lc823450_extint1, LC823450_IRQ_EXTINT1)         /* Vector 16+60: ExternalINT1 interrupt */
VECTOR(lc823450_extint2, LC823450_IRQ_EXTINT2)         /* Vector 16+61: ExternalINT2 interrupt */
VECTOR(lc823450_extint3, LC823450_IRQ_EXTINT3)         /* Vector 16+62: ExternalINT3 interrupt */
VECTOR(lc823450_extint4, LC823450_IRQ_EXTINT4)         /* Vector 16+63: ExternalINT4 interrupt */
VECTOR(lc823450_extint5, LC823450_IRQ_EXTINT5)         /* Vector 16+64: ExternalINT5 interrupt */

#endif /* CONFIG_ARMV7M_CMNVECTOR */
