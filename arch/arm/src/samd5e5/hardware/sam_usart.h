/********************************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_usart.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_USART_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_USART_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* USART register offsets *******************************************************************/

#define SAM_USART_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_USART_CTRLB_OFFSET       0x0004  /* Control B register */
#define SAM_USART_CTRLC_OFFSET       0x0008  /* Control C register */
#define SAM_USART_BAUD_OFFSET        0x000c  /* Baud register */
#define SAM_USART_RXPL_OFFSET        0x000e  /* Receive pulse length register */
#define SAM_USART_INTENCLR_OFFSET    0x0014  /* Interrupt enable clear register */
#define SAM_USART_INTENSET_OFFSET    0x0016  /* Interrupt enable set register */
#define SAM_USART_INTFLAG_OFFSET     0x0018  /* Interrupt flag and status clear register */
#define SAM_USART_STATUS_OFFSET      0x001a  /* Status register */
#define SAM_USART_SYNCBUSY_OFFSET    0x001c  /* Synchronization busy register */
#define SAM_USART_RXERRCNT_OFFSET    0x0020  /* Receive error count register */
#define SAM_USART_LENGTH_OFFSET      0x0022  /* Length register */
#define SAM_USART_DATA_OFFSET        0x0028  /* Data register */
#define SAM_USART_DBGCTRL_OFFSET     0x0030  /* Debug control register */

/* USART register addresses *****************************************************************/

#define SAM_USART0_CTRLA             (SAM_SERCOM0_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART0_CTRLB             (SAM_SERCOM0_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART0_CTRLC             (SAM_SERCOM0_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART0_BAUD              (SAM_SERCOM0_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART0_RXPL              (SAM_SERCOM0_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART0_INTENCLR          (SAM_SERCOM0_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART0_INTENSET          (SAM_SERCOM0_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART0_INTFLAG           (SAM_SERCOM0_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART0_STATUS            (SAM_SERCOM0_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART0_SYNCBUSY          (SAM_SERCOM0_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART0_RXERRCNT          (SAM_SERCOM0_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART0_LENGTH            (SAM_SERCOM0_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART0_DATA              (SAM_SERCOM0_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART0_DBGCTRL           (SAM_SERCOM0_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART1_CTRLA             (SAM_SERCOM1_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART1_CTRLB             (SAM_SERCOM1_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART1_CTRLC             (SAM_SERCOM1_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART1_BAUD              (SAM_SERCOM1_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART1_RXPL              (SAM_SERCOM1_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART1_INTENCLR          (SAM_SERCOM1_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART1_INTENSET          (SAM_SERCOM1_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART1_INTFLAG           (SAM_SERCOM1_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART1_STATUS            (SAM_SERCOM1_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART1_SYNCBUSY          (SAM_SERCOM1_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART1_RXERRCNT          (SAM_SERCOM1_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART1_LENGTH            (SAM_SERCOM1_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART1_DATA              (SAM_SERCOM1_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART1_DBGCTRL           (SAM_SERCOM1_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART2_CTRLA             (SAM_SERCOM2_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART2_CTRLB             (SAM_SERCOM2_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART2_CTRLC             (SAM_SERCOM2_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART2_BAUD              (SAM_SERCOM2_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART2_RXPL              (SAM_SERCOM2_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART2_INTENCLR          (SAM_SERCOM2_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART2_INTENSET          (SAM_SERCOM2_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART2_INTFLAG           (SAM_SERCOM2_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART2_STATUS            (SAM_SERCOM2_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART2_SYNCBUSY          (SAM_SERCOM2_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART2_RXERRCNT          (SAM_SERCOM2_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART2_LENGTH            (SAM_SERCOM2_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART2_DATA              (SAM_SERCOM2_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART2_DBGCTRL           (SAM_SERCOM2_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART3_CTRLA             (SAM_SERCOM3_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART3_CTRLB             (SAM_SERCOM3_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART3_CTRLC             (SAM_SERCOM3_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART3_BAUD              (SAM_SERCOM3_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART3_RXPL              (SAM_SERCOM3_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART3_INTENCLR          (SAM_SERCOM3_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART3_INTENSET          (SAM_SERCOM3_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART3_INTFLAG           (SAM_SERCOM3_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART3_STATUS            (SAM_SERCOM3_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART3_SYNCBUSY          (SAM_SERCOM3_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART3_RXERRCNT          (SAM_SERCOM3_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART3_LENGTH            (SAM_SERCOM3_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART3_DATA              (SAM_SERCOM3_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART3_DBGCTRL           (SAM_SERCOM3_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART4_CTRLA             (SAM_SERCOM4_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART4_CTRLB             (SAM_SERCOM4_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART4_CTRLC             (SAM_SERCOM4_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART4_BAUD              (SAM_SERCOM4_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART4_RXPL              (SAM_SERCOM4_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART4_INTENCLR          (SAM_SERCOM4_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART4_INTENSET          (SAM_SERCOM4_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART4_INTFLAG           (SAM_SERCOM4_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART4_STATUS            (SAM_SERCOM4_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART4_SYNCBUSY          (SAM_SERCOM4_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART4_RXERRCNT          (SAM_SERCOM4_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART4_LENGTH            (SAM_SERCOM4_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART4_DATA              (SAM_SERCOM4_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART4_DBGCTRL           (SAM_SERCOM4_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART5_CTRLA             (SAM_SERCOM5_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART5_CTRLB             (SAM_SERCOM5_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART5_CTRLC             (SAM_SERCOM5_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART5_BAUD              (SAM_SERCOM5_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART5_RXPL              (SAM_SERCOM5_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART5_INTENCLR          (SAM_SERCOM5_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART5_INTENSET          (SAM_SERCOM5_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART5_INTFLAG           (SAM_SERCOM5_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART5_STATUS            (SAM_SERCOM5_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART5_SYNCBUSY          (SAM_SERCOM5_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART5_RXERRCNT          (SAM_SERCOM5_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART5_LENGTH            (SAM_SERCOM5_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART5_DATA              (SAM_SERCOM5_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART5_DBGCTRL           (SAM_SERCOM5_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART6_CTRLA             (SAM_SERCOM6_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART6_CTRLB             (SAM_SERCOM6_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART6_CTRLC             (SAM_SERCOM6_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART6_BAUD              (SAM_SERCOM6_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART6_RXPL              (SAM_SERCOM6_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART6_INTENCLR          (SAM_SERCOM6_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART6_INTENSET          (SAM_SERCOM6_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART6_INTFLAG           (SAM_SERCOM6_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART6_STATUS            (SAM_SERCOM6_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART6_SYNCBUSY          (SAM_SERCOM6_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART6_RXERRCNT          (SAM_SERCOM6_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART6_LENGTH            (SAM_SERCOM6_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART6_DATA              (SAM_SERCOM6_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART6_DBGCTRL           (SAM_SERCOM6_BASE + SAM_USART_DBGCTRL_OFFSET)

#define SAM_USART7_CTRLA             (SAM_SERCOM7_BASE + SAM_USART_CTRLA_OFFSET)
#define SAM_USART7_CTRLB             (SAM_SERCOM7_BASE + SAM_USART_CTRLB_OFFSET)
#define SAM_USART7_CTRLC             (SAM_SERCOM7_BASE + SAM_USART_CTRLC_OFFSET)
#define SAM_USART7_BAUD              (SAM_SERCOM7_BASE + SAM_USART_BAUD_OFFSET)
#define SAM_USART7_RXPL              (SAM_SERCOM7_BASE + SAM_USART_RXPL_OFFSET)
#define SAM_USART7_INTENCLR          (SAM_SERCOM7_BASE + SAM_USART_INTENCLR_OFFSET)
#define SAM_USART7_INTENSET          (SAM_SERCOM7_BASE + SAM_USART_INTENSET_OFFSET)
#define SAM_USART7_INTFLAG           (SAM_SERCOM7_BASE + SAM_USART_INTFLAG_OFFSET)
#define SAM_USART7_STATUS            (SAM_SERCOM7_BASE + SAM_USART_STATUS_OFFSET)
#define SAM_USART7_SYNCBUSY          (SAM_SERCOM7_BASE + SAM_USART_SYNCBUSY_OFFSET)
#define SAM_USART7_RXERRCNT          (SAM_SERCOM7_BASE + SAM_USART_RXERRCNT_OFFSET)
#define SAM_USART7_LENGTH            (SAM_SERCOM7_BASE + SAM_USART_LENGTH_OFFSET)
#define SAM_USART7_DATA              (SAM_SERCOM7_BASE + SAM_USART_DATA_OFFSET)
#define SAM_USART7_DBGCTRL           (SAM_SERCOM7_BASE + SAM_USART_DBGCTRL_OFFSET)

/* USART register bit definitions ***********************************************************/

/* Control A register */

#define USART_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define USART_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define USART_CTRLA_MODE_SHIFT       (2)       /* Bits 2-4: Operating Mode */
#define USART_CTRLA_MODE_MASK        (7 << USART_CTRLA_MODE_SHIFT)
#  define USART_CTRLA_MODE_EXTUSART  (0 << USART_CTRLA_MODE_SHIFT) /* USART with external
                                                                    * clock */
#  define USART_CTRLA_MODE_INTUSART  (1 << USART_CTRLA_MODE_SHIFT) /* USART with internal
                                                                    * clock */
#define USART_CTRLA_RUNSTDBY         (1 << 7)  /* Bit 7:  Run in standby */
#define USART_CTRLA_IBON             (1 << 8)  /* Bit 8:  Immediate BUFOVF notification */
#define USART_CTRLA_TXINV            (1 << 9)  /* Bit 9:  Transmit Data Invert */
#define USART_CTRLA_RXINV            (1 << 10) /* Bit 10: Receive Data Invert */
#define USART_CTRLA_SAMPR_SHIFT      (11)      /* Bits 13-15: Sample rate */
#define USART_CTRLA_SAMPR_MASK       (7 << USART_CTRLA_SAMPR_SHIFT)
#  define USART_CTRLA_SAMPR_16XA     (0 << USART_CTRLA_SAMPR_SHIFT) /* 16x oversampling;
                                                                     * arithmetic baud */
#  define USART_CTRLA_SAMPR_16XF     (1 << USART_CTRLA_SAMPR_SHIFT) /* 16x oversampling;
                                                                     * fractional baud */
#  define USART_CTRLA_SAMPR_8XA      (2 << USART_CTRLA_SAMPR_SHIFT) /* 8x oversampling;
                                                                     * arithmetic baud */
#  define USART_CTRLA_SAMPR_8XF      (3 << USART_CTRLA_SAMPR_SHIFT) /* 8x oversampling;
                                                                     * fractional baud */
#  define USART_CTRLA_SAMPR_3XA      (4 << USART_CTRLA_SAMPR_SHIFT) /* 3x oversampling;
                                                                     * arithmetic baud */
#define USART_CTRLA_TXPO_SHIFT       (16)      /* Bits 16-17: Transmit data pinout */
#define USART_CTRLA_TXPO_MASK        (3 << USART_CTRLA_TXPO_SHIFT)
#  define USART_CTRLA_TXPAD0_1       (0 << USART_CTRLA_TXPO_SHIFT) /* TxD=PAD0 XCK=PAD1
                                                                    * RTS/TE=N/A CTS=N/A */
#  define USART_CTRLA_TXPAD0_2       (2 << USART_CTRLA_TXPO_SHIFT) /* TxD=PAD0 XCK=N/A
                                                                    * RTS/TE=PAD2 CTS=PAD3 */
#  define USART_CTRLA_TXPAD0_3       (3 << USART_CTRLA_TXPO_SHIFT) /* TxD=PAD0 RTS=PAD1
                                                                    * RTS/PAD2 CTS=N/A */
#define USART_CTRLA_RXPO_SHIFT       (20)      /* Bits 20-21: Receive data pinout */
#define USART_CTRLA_RXPO_MASK        (3 << USART_CTRLA_RXPO_SHIFT)
#  define USART_CTRLA_RXPAD0         (0 << USART_CTRLA_RXPO_SHIFT) /* RxD=SERCOM PAD0 */
#  define USART_CTRLA_RXPAD1         (1 << USART_CTRLA_RXPO_SHIFT) /* RxD=SERCOM PAD1 */
#  define USART_CTRLA_RXPAD2         (2 << USART_CTRLA_RXPO_SHIFT) /* RxD=SERCOM PAD2 */
#  define USART_CTRLA_RXPAD3         (3 << USART_CTRLA_RXPO_SHIFT) /* RxD=SERCOM PAD3 */
#define USART_CTRLA_SAMPA_SHIFT      (22)      /* Bits 22-23: Sample adjustment */
#define USART_CTRLA_SAMPA_MASK       (3 << USART_CTRLA_SAMPA_SHIFT)
#  define USART_CTRLA_SAMPA_789      (0 << USART_CTRLA_SAMPA_SHIFT) /* 16x oversampling 7-8-9 */
#  define USART_CTRLA_SAMPA_91011    (1 << USART_CTRLA_SAMPA_SHIFT) /* 16x oversampling 9-10-11 */
#  define USART_CTRLA_SAMPA_111213   (2 << USART_CTRLA_SAMPA_SHIFT) /* 16x oversampling 11-12-13 */
#  define USART_CTRLA_SAMPA_131415   (3 << USART_CTRLA_SAMPA_SHIFT) /* 16x oversampling 13-14-15 */
#  define USART_CTRLA_SAMPA_345      (0 << USART_CTRLA_SAMPA_SHIFT) /* 8x oversampling 3-4-5 */
#  define USART_CTRLA_SAMPA_456      (1 << USART_CTRLA_SAMPA_SHIFT) /* 8x oversampling 4-5-6 */
#  define USART_CTRLA_SAMPA_567      (2 << USART_CTRLA_SAMPA_SHIFT) /* 8x oversampling 5-6-7 */
#  define USART_CTRLA_SAMPA_678      (3 << USART_CTRLA_SAMPA_SHIFT) /* 8x oversampling 6-7-8 */
#define USART_CTRLA_FORM_SHIFT       (24)      /* Bits 24-27: Frame format */
#define USART_CTRLA_FORM_MASK        (7 << USART_CTRLA_FORM_SHIFT)
#  define USART_CTRLA_FORM_NOPARITY  (0 << USART_CTRLA_FORM_SHIFT) /* USART frame (no parity) */
#  define USART_CTRLA_FORM_PARITY    (1 << USART_CTRLA_FORM_SHIFT) /* USART frame (w/parity) */
#  define USART_CTRLA_FORM_LINMSTR   (2 << USART_CTRLA_FORM_SHIFT) /* LIN master */
#  define USART_CTRLA_FORM_AUTOBAUD  (4 << USART_CTRLA_FORM_SHIFT) /* Lin slave; Auto-baud (no parity) */
#  define USART_CTRLA_FORM_AUTOBAUDP (5 << USART_CTRLA_FORM_SHIFT) /* Auto-baud (w/ parity) */
#  define USART_CTRLA_FORM_ISO7816   (7 << USART_CTRLA_FORM_SHIFT) /* ISO 7816 */
#define USART_CTRLA_CMODE            (1 << 28)  /* Bit 28: Communication mode */
#  define USART_CTRLA_ASYNCH         (0)
#  define USART_CTRLA_SYNCH          USART_CTRLA_CMODE
#define USART_CTRLA_CPOL             (1 << 29)  /* Bit 29: Clock polarity */
#  define USART_CTRLA_CPOL_NORMAL    (0)              /* Rising XCK edge Falling XCK edge */
#  define USART_CTRLA_CPOL_INVERTED  USART_CTRLA_CPOL /* Falling XCK edge Rising XCK edge */
#define USART_CTRLA_DORD             (1 << 30)  /* Bit 30: Data order */
#  define USART_CTRLA_MSBFIRST       (0)
#  define USART_CTRLA_LSBFIRST       USART_CTRLA_DORD

/* Control B register */

#define USART_CTRLB_CHSIZE_SHIFT     (0)       /* Bits 0-2: Character Size */
#define USART_CTRLB_CHSIZE_MASK      (7 << USART_CTRLB_CHSIZE_SHIFT)
#  define USART_CTRLB_CHSIZE_8BITS   (0 << USART_CTRLB_CHSIZE_SHIFT) /* 8 bits */
#  define USART_CTRLB_CHSIZE_9BITS   (1 << USART_CTRLB_CHSIZE_SHIFT) /* 9 bits */
#  define USART_CTRLB_CHSIZE_5BITS   (5 << USART_CTRLB_CHSIZE_SHIFT) /* 5 bits */
#  define USART_CTRLB_CHSIZE_6BITS   (6 << USART_CTRLB_CHSIZE_SHIFT) /* 6 bits */
#  define USART_CTRLB_CHSIZE_7BITS   (7 << USART_CTRLB_CHSIZE_SHIFT) /* 7 bits */
#define USART_CTRLB_SBMODE           (1 << 6)  /* Bit 6:  Stop bit mode */
#  define USART_CTRLB_SBMODE_1       (0)
#  define USART_CTRLB_SBMODE_2       USART_CTRLB_SBMODE
#define USART_CTRLB_COLDEN           (1 << 8)  /* Bit 8:  Collision detection enable */
#define USART_CTRLB_SFDE             (1 << 9)  /* Bit 9:  Start of frame detection enable */
#define USART_CTRLB_ENC              (1 << 10) /* Bit 10: Encoding format */
#  define USART_CTRLB_UNENCODED      (0)
#  define USART_CTRLB_IRDA           USART_CTRLB_ENC
#define USART_CTRLB_PMODE            (1 << 13) /* Bit 13: Parity mode */
#  define USART_CTRLB_PEVEN          (0)
#  define USART_CTRLB_PODD           USART_CTRLB_PMODE
#define USART_CTRLB_TXEN             (1 << 16) /* Bit 16: Transmitter enable */
#define USART_CTRLB_RXEN             (1 << 17) /* Bit 17: Receiver enable */
#define USART_CTRLB_LINCMD_SHIFT     (24)      /* Bits 24-25: Receiver enable */
#define USART_CTRLB_LINCMD_MASK      (3 << USART_CTRLB_LINCMD_SHIFT)
#  define USART_CTRLB_LINCMD_NORMAL  (0 << USART_CTRLB_LINCMD_SHIFT) /* Normal USART transmission */
#  define USART_CTRLB_LINCMD_BREAK   (1 << USART_CTRLB_LINCMD_SHIFT) /* Break field is transmitted */
#  define USART_CTRLB_LINCMD_BSI     (2 << USART_CTRLB_LINCMD_SHIFT) /* Break, sync and identifier
                                                                      * transmitted */

/* Control C register */

#define USART_CTRLC_GTIME_SHIFT      (0)       /* Bits 0-2: Guard Time */
#define USART_CTRLC_GTIME_MASK       (7 << USART_CTRLC_GTIME_SHIFT)
#  define USART_CTRLC_GTIME(n)       ((uint32_t)(n) << USART_CTRLC_GTIME_SHIFT)
#define USART_CTRLC_BRKLEN_SHIFT     (8)       /* Bits 8-9: LIN Master Break Length */
#define USART_CTRLC_BRKLEN_MASK      (3 << USART_CTRLC_BRKLEN_SHIFT)
#  define USART_CTRLC_BRKLEN_13BITS  (0 << USART_CTRLC_BRKLEN_SHIFT) /* 13 bit times */
#  define USART_CTRLC_BRKLEN_17BITS  (1 << USART_CTRLC_BRKLEN_SHIFT) /* 17 bit times */
#  define USART_CTRLC_BRKLEN_21BITS  (2 << USART_CTRLC_BRKLEN_SHIFT) /* 21 bit times */
#  define USART_CTRLC_BRKLEN_26BITS  (3 << USART_CTRLC_BRKLEN_SHIFT) /* 26 bit times */
#define USART_CTRLC_HDRDLY_SHIFT     (10)      /* Bits 10-11: LIN Master Header Delay */
#define USART_CTRLC_HDRDLY_MASK      (3 << USART_CTRLC_HDRDLY_SHIFT)
#  define USART_CTRLC_HDRDLY_1_1     (0 << USART_CTRLC_HDRDLY_SHIFT) /* 1, 1 bit times */
#  define USART_CTRLC_HDRDLY_4_4     (1 << USART_CTRLC_HDRDLY_SHIFT) /* 4, 4 bit times */
#  define USART_CTRLC_HDRDLY_8_4     (2 << USART_CTRLC_HDRDLY_SHIFT) /* 8, 4 bit times */
#  define USART_CTRLC_HDRDLY_14_4    (3 << USART_CTRLC_HDRDLY_SHIFT) /* 14, 4 bit times */
#define USART_CTRLC_INACK            (1 << 16) /* Bit 16: Inhibit Not Acknowledge */
#  define USART_CTRLC_NACK           (0)                /* 0=NACK transmitted */
#  define USART_CTRLC_NONACK         USART_CTRLC_INACK  /* 1=NACK not ransmitted */
#define USART_CTRLC_DSNACK           (1 << 17) /* Bit 17: Disable Successive Not Acknowledge */
#define USART_CTRLC_MAXITER_SHIFT    (20)      /* Bits 20-22: Data 32 Bit */
#define USART_CTRLC_MAXITER_MASK     (7 << USART_CTRLC_MAXITER_SHIFT)
#  define USART_CTRLC_MAXITER(n)     ((uint32_t)(n) << USART_CTRLC_MAXITER_SHIFT)
#define USART_CTRLC_DATA32B_SHIFT    (24)      /* Bits 24-25: Data 32 Bit */
#define USART_CTRLC_DATA32B_MASK     (3 << USART_CTRLC_DATA32B_SHIFT)
#  define USART_CTRLC_DATA32B_CHSIZE (0 << USART_CTRLC_DATA32B_SHIFT) /* Read/write per CHSIZE */
#  define USART_CTRLC_DATA32B_WRITE  (1 << USART_CTRLC_DATA32B_SHIFT) /* Read CHSIZE; Write 32b */
#  define USART_CTRLC_DATA32B_READ   (2 << USART_CTRLC_DATA32B_SHIFT) /* Write CHSIZE; Read 32b */
#  define USART_CTRLC_DATA32B_BOTH   (3 << USART_CTRLC_DATA32B_SHIFT) /* Both per 32-bit extension */

/* Baud register (16-bit baud value) */
/* Receive pulse length register (8-bit value) */

/* Interrupt enable clear, interrupt enable set, interrupt enable set, interrupt flag and
 * status clear registers.
 */

#define USART_INT_DRE                (1 << 0)  /* Bit 0:  Data register empty interrupt */
#define USART_INT_TXC                (1 << 1)  /* Bit 1:  Transmit complete interrupt */
#define USART_INT_RXC                (1 << 2)  /* Bit 2:  Receive complete interrupt */
#define USART_INT_RXS                (1 << 3)  /* Bit 3:  Receive start interrupt */
#define USART_INT_CTSIC              (1 << 4)  /* Bit 4:  Clear to send input change interrupt */
#define USART_INT_RXBRK              (1 << 5)  /* Bit 5:  Receive break interrupt */
#define USART_INT_ERROR              (1 << 6)  /* Bit 7:  Error interrupt */

#define USART_INT_ALL                (0xbf)

/* Status register */

#define USART_STATUS_PERR            (1 << 0)  /* Bit 0:  Parity error */
#define USART_STATUS_FERR            (1 << 1)  /* Bit 1:  Frame error */
#define USART_STATUS_BUFOVF          (1 << 2)  /* Bit 2:  Buffer overflow */
#define USART_STATUS_CTS             (1 << 3)  /* Bit 3:  Clear to send */
#define USART_STATUS_ISF             (1 << 4)  /* Bit 4:  Inconsistent sync field */
#define USART_STATUS_COLL            (1 << 5)  /* Bit 5:  Collision detected */
#define USART_STATUS_TXE             (1 << 6)  /* Bit 6:  Transmitter Empty */
#define USART_STATUS_ITER            (1 << 7)  /* Bit 7:  Maximum Number of Repetitions Reached */

/* Synchronization busy register */

#define USART_SYNCBUSY_SWRST         (1 << 0)  /* Bit 0:  Software reset synchronization busy */
#define USART_SYNCBUSY_ENABLE        (1 << 1)  /* Bit 1:  SERCOM enable synchronization busy */
#define USART_SYNCBUSY_CTRLB         (1 << 2)  /* Bit 2:  CTRLB synchronization busy */
#define USART_SYNCBUSY_RXERRCNT      (1 << 3)  /* Bit 3:  Receive error count synchronization busy */
#define USART_SYNCBUSY_LENGTH        (1 << 4)  /* Bit 4:  LENGTH synchronization busy */

#define USART_SYNCBUSY_ALL           0x001f

/* Receive error count register (8-bit value) */

/* Length register */

#define USART_LENGTH_LEN_SHIFT       (0)       /* Bit 0-7: Data length */
#define USART_LENGTH_LEN_MASK        (0xff << USART_LENGTH_LEN_SHIFT)
#  define USART_LENGTHLEN(n)         ((uint32_t)(n) << USART_LENGTH_LEN_SHIFT)
#define USART_LENGTH_LENEN_SHIFT     (8)       /* Bits 8-9: Data Length Enable */
#define USART_LENGTH_LENEN_MASK      (3 << USART_LENGTH_LENEN_SHIFT)
#  define USART_LENGTH_LENEN_DISABLE (0 << USART_LENGTH_LENEN_SHIFT) /* Length counter disabled */
#  define USART_LENGTH_LENEN_XMIT    (1 << USART_LENGTH_LENEN_SHIFT) /* Enabled for transmit */
#  define USART_LENGTH_LENEN_RECV    (2 << USART_LENGTH_LENEN_SHIFT) /* Enabled for receive */

/* Data register (8-, 9- or 32-bit data) */

#define USART_DATA_MASK9             (0x1ff)   /* Bits 0-8: 8- or 9-bit Data */

/* Debug control register */

#define USART_DBGCTRL_DBGSTOP        (1 << 0)  /* Bit 0: Debug stop mode */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_USART_H */
