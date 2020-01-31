/************************************************************************************
 * arch/arm/src/imx6/hardware/imx_uart.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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

#ifndef __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_UART_H
#define __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* UART Register Offsets ************************************************************/

#define UART_RXD_OFFSET        0x0000 /* UART receiver register */
#define UART_TXD_OFFSET        0x0040 /* UART receiver register */
#define UART_UCR1_OFFSET       0x0080 /* UART control register 1 */
#define UART_UCR2_OFFSET       0x0084 /* UART control register 2 */
#define UART_UCR3_OFFSET       0x0088 /* UART control register 3 */
#define UART_UCR4_OFFSET       0x008c /* UART control register 4 */
#define UART_UFCR_OFFSET       0x0090 /* UART FIFO control register */
#define UART_USR1_OFFSET       0x0094 /* UART status register 1 */
#define UART_USR2_OFFSET       0x0098 /* UART status register 2 */
#define UART_UESC_OFFSET       0x009c /* UART escape character register */
#define UART_UTIM_OFFSET       0x00a0 /* UART escape timer register */
#define UART_UBIR_OFFSET       0x00a4 /* UART BRM incremental register */
#define UART_UBMR_OFFSET       0x00a8 /* UART BRM modulator register */
#define UART_UBRC_OFFSET       0x00ac /* UART baud rate counter register */
#define UART_ONEMS_OFFSET      0x00b0 /* UART One Millisecond Register */
#define UART_UTS_OFFSET        0x00b4 /* UART test register */
#define UART_UMCR_OFFSET       0x00b8 /* UART RS-485 Mode Control Register */

/* UART Register Addresses **********************************************************/

#define UART1_RXD              (IMX_UART1_VBASE+UART_RXD_OFFSET)
#define UART1_TXD              (IMX_UART1_VBASE+UART_TXD_OFFSET)
#define UART1_UCR1             (IMX_UART1_VBASE+UART_UCR1_OFFSET)
#define UART1_UCR2             (IMX_UART1_VBASE+UART_UCR2_OFFSET)
#define UART1_UCR3             (IMX_UART1_VBASE+UART_UCR3_OFFSET)
#define UART1_UCR4             (IMX_UART1_VBASE+UART_UCR4_OFFSET)
#define UART1_UFCR             (IMX_UART1_VBASE+UART_UFCR_OFFSET)
#define UART1_USR1             (IMX_UART1_VBASE+UART_USR1_OFFSET)
#define UART1_USR2             (IMX_UART1_VBASE+UART_USR2_OFFSET)
#define UART1_UESC             (IMX_UART1_VBASE+UART_UESC_OFFSET)
#define UART1_UTIM             (IMX_UART1_VBASE+UART_UTIM_OFFSET)
#define UART1_UBIR             (IMX_UART1_VBASE+UART_UBIR_OFFSET)
#define UART1_UBMR             (IMX_UART1_VBASE+UART_UBMR_OFFSET)
#define UART1_UBRC             (IMX_UART1_VBASE+UART_UBRC_OFFSET)
#define UART1_ONEMS            (IMX_UART1_VBASE+UART_ONEMS_OFFSET)
#define UART1_UTS              (IMX_UART1_VBASE+UART_UTS_OFFSET)
#define UART1_UMCR             (IMX_UART1_VBASE+UART_UMCR_OFFSET)

#define UART2_RXD              (IMX_UART2_VBASE+UART_RXD_OFFSET)
#define UART2_TXD              (IMX_UART2_VBASE+UART_TXD_OFFSET)
#define UART2_UCR1             (IMX_UART2_VBASE+UART_UCR1_OFFSET)
#define UART2_UCR2             (IMX_UART2_VBASE+UART_UCR2_OFFSET)
#define UART2_UCR3             (IMX_UART2_VBASE+UART_UCR3_OFFSET)
#define UART2_UCR4             (IMX_UART2_VBASE+UART_UCR4_OFFSET)
#define UART2_UFCR             (IMX_UART2_VBASE+UART_UFCR_OFFSET)
#define UART2_USR1             (IMX_UART2_VBASE+UART_USR1_OFFSET)
#define UART2_USR2             (IMX_UART2_VBASE+UART_USR2_OFFSET)
#define UART2_UESC             (IMX_UART2_VBASE+UART_UESC_OFFSET)
#define UART2_UTIM             (IMX_UART2_VBASE+UART_UTIM_OFFSET)
#define UART2_UBIR             (IMX_UART2_VBASE+UART_UBIR_OFFSET)
#define UART2_UBMR             (IMX_UART2_VBASE+UART_UBMR_OFFSET)
#define UART2_UBRC             (IMX_UART2_VBASE+UART_UBRC_OFFSET)
#define UART2_ONEMS            (IMX_UART2_VBASE+UART_ONEMS_OFFSET)
#define UART2_UTS              (IMX_UART2_VBASE+UART_UTS_OFFSET)
#define UART2_UMCR             (IMX_UART2_VBASE+UART_UMCR_OFFSET)

#define UART3_RXD              (IMX_UART3_VBASE+UART_RXD_OFFSET)
#define UART3_TXD              (IMX_UART3_VBASE+UART_TXD_OFFSET)
#define UART3_UCR1             (IMX_UART3_VBASE+UART_UCR1_OFFSET)
#define UART3_UCR2             (IMX_UART3_VBASE+UART_UCR2_OFFSET)
#define UART3_UCR3             (IMX_UART3_VBASE+UART_UCR3_OFFSET)
#define UART3_UCR4             (IMX_UART3_VBASE+UART_UCR4_OFFSET)
#define UART3_UFCR             (IMX_UART3_VBASE+UART_UFCR_OFFSET)
#define UART3_USR1             (IMX_UART3_VBASE+UART_USR1_OFFSET)
#define UART3_USR2             (IMX_UART3_VBASE+UART_USR2_OFFSET)
#define UART3_UESC             (IMX_UART3_VBASE+UART_UESC_OFFSET)
#define UART3_UTIM             (IMX_UART3_VBASE+UART_UTIM_OFFSET)
#define UART3_UBIR             (IMX_UART3_VBASE+UART_UBIR_OFFSET)
#define UART3_UBMR             (IMX_UART3_VBASE+UART_UBMR_OFFSET)
#define UART3_UBRC             (IMX_UART3_VBASE+UART_UBRC_OFFSET)
#define UART3_ONEMS            (IMX_UART3_VBASE+UART_ONEMS_OFFSET)
#define UART3_UTS              (IMX_UART3_VBASE+UART_UTS_OFFSET)
#define UART3_UMCR             (IMX_UART3_VBASE+UART_UMCR_OFFSET)

#define UART4_RXD              (IMX_UART4_VBASE+UART_RXD_OFFSET)
#define UART4_TXD              (IMX_UART4_VBASE+UART_TXD_OFFSET)
#define UART4_UCR1             (IMX_UART4_VBASE+UART_UCR1_OFFSET)
#define UART4_UCR2             (IMX_UART4_VBASE+UART_UCR2_OFFSET)
#define UART4_UCR3             (IMX_UART4_VBASE+UART_UCR3_OFFSET)
#define UART4_UCR4             (IMX_UART4_VBASE+UART_UCR4_OFFSET)
#define UART4_UFCR             (IMX_UART4_VBASE+UART_UFCR_OFFSET)
#define UART4_USR1             (IMX_UART4_VBASE+UART_USR1_OFFSET)
#define UART4_USR2             (IMX_UART4_VBASE+UART_USR2_OFFSET)
#define UART4_UESC             (IMX_UART4_VBASE+UART_UESC_OFFSET)
#define UART4_UTIM             (IMX_UART4_VBASE+UART_UTIM_OFFSET)
#define UART4_UBIR             (IMX_UART4_VBASE+UART_UBIR_OFFSET)
#define UART4_UBMR             (IMX_UART4_VBASE+UART_UBMR_OFFSET)
#define UART4_UBRC             (IMX_UART4_VBASE+UART_UBRC_OFFSET)
#define UART4_ONEMS            (IMX_UART4_VBASE+UART_ONEMS_OFFSET)
#define UART4_UTS              (IMX_UART4_VBASE+UART_UTS_OFFSET)
#define UART4_UMCR             (IMX_UART4_VBASE+UART_UMCR_OFFSET)

#define UART5_RXD              (IMX_UART5_VBASE+UART_RXD_OFFSET)
#define UART5_TXD              (IMX_UART5_VBASE+UART_TXD_OFFSET)
#define UART5_UCR1             (IMX_UART5_VBASE+UART_UCR1_OFFSET)
#define UART5_UCR2             (IMX_UART5_VBASE+UART_UCR2_OFFSET)
#define UART5_UCR3             (IMX_UART5_VBASE+UART_UCR3_OFFSET)
#define UART5_UCR4             (IMX_UART5_VBASE+UART_UCR4_OFFSET)
#define UART5_UFCR             (IMX_UART5_VBASE+UART_UFCR_OFFSET)
#define UART5_USR1             (IMX_UART5_VBASE+UART_USR1_OFFSET)
#define UART5_USR2             (IMX_UART5_VBASE+UART_USR2_OFFSET)
#define UART5_UESC             (IMX_UART5_VBASE+UART_UESC_OFFSET)
#define UART5_UTIM             (IMX_UART5_VBASE+UART_UTIM_OFFSET)
#define UART5_UBIR             (IMX_UART5_VBASE+UART_UBIR_OFFSET)
#define UART5_UBMR             (IMX_UART5_VBASE+UART_UBMR_OFFSET)
#define UART5_UBRC             (IMX_UART5_VBASE+UART_UBRC_OFFSET)
#define UART5_ONEMS            (IMX_UART5_VBASE+UART_ONEMS_OFFSET)
#define UART5_UTS              (IMX_UART5_VBASE+UART_UTS_OFFSET)
#define UART5_UMCR             (IMX_UART5_VBASE+UART_UMCR_OFFSET)

/* UART Register Bit Definitions ****************************************************/

/* UART Receiver Register */

#define UART_RXD_DATA_SHIFT    0         /* Bits 0-7: Received Data */
#define UART_RXD_DATA_MASK     (0xff << UART_RXD_DATA_SHIFT)
#define UART_RXD_PRERR         (1 << 10) /* Bit 10: Parity Error */
#define UART_RXD_BRK           (1 << 11) /* Bit 11: Break Detect */
#define UART_RXD_FRMERR        (1 << 12) /* Bit 12: Frame Error */
#define UART_RXD_OVRRUN        (1 << 13) /* Bit 13: Receiver Overrun */
#define UART_RXD_ERR           (1 << 14) /* Bit 14: Error Detect */
#define UART_RXD_CHARRDY       (1 << 15) /* Bit 15: Character Ready */

/* UART Transmitter Register */

#define UART_TXDATA_SHIFT      0 /* Bits 0-7: Transmit Data */
#define UART_TXDATA_MASK       (0xff << UART_UCR4_TXDATA_SHIFT)

/* UART Control Register 1 */

#define UART_UCR1_UARTEN       (1 << 0)  /* Bit 0: Enable/disable UART */
#define UART_UCR1_DOZE         (1 << 1)  /* Bit 1: UART Doze enable */
#define UART_UCR1_ATDMAEN      (1 << 2)  /* Bit 2: Aging DMA Timer Enable */
#define UART_UCR1_TXDMAEN      (1 << 3)  /* Bit 3: Transmitter Ready DMA Enable */
#define UART_UCR1_SNDBRK       (1 << 4)  /* Bit 4: Send BREAK */
#define UART_UCR1_RTSDEN       (1 << 5)  /* Bit 5: RTS Delta interrupt enable */
#define UART_UCR1_TXEMPTYEN    (1 << 6)  /* Bit 6: Transmitter empty interrupt enable */
#define UART_UCR1_IREN         (1 << 7)  /* Bit 7: Infrared Interface enable */
#define UART_UCR1_RDMAEN       (1 << 8)  /* Bit 8: Receive ready DMA enable */
#define UART_UCR1_RRDYEN       (1 << 9)  /* Bit 9: Receiver ready interrupt enable */
#define UART_UCR1_ICD_SHIFT    10        /* Bit 10-11: Idle condition detect */
#define UART_UCR1_ICD_MASK     (3 << UART_UCR1_ICD_SHIFT)
#  define UART_UCR1_ICD_4FRMS  (0 << UART_UCR1_ICD_SHIFT) /* Idle for more than 4 frames */
#  define UART_UCR1_ICD_8FRMS  (1 << UART_UCR1_ICD_SHIFT) /* Idle for more than 8 frames */
#  define UART_UCR1_ICD_16FRMS (2 << UART_UCR1_ICD_SHIFT) /* Idle for more than 16 frames */
#  define UART_UCR1_ICD_32FRMS (3 << UART_UCR1_ICD_SHIFT) /* Idle for more than 32 frames */
#define UART_UCR1_IDEN         (1 << 12) /* Bit 12: Idle condition detected interrupt enable */
#define UART_UCR1_TRDYEN       (1 << 13) /* Bit 13: Transmitter ready interrupt enable */
#define UART_UCR1_ADBR         (1 << 14) /* Bit 14: Automatic detection of baud rate */
#define UART_UCR1_ADEN         (1 << 15) /* Bit 15: Automatic baud rate detection interrupt enable */

/* UART Control Register 2 */

#define UART_UCR2_SRST         (1 << 0)  /* Bit 0: Software reset */
#define UART_UCR2_RXEN         (1 << 1)  /* Bit 1: Receiver enable */
#define UART_UCR2_TXEN         (1 << 2)  /* Bit 2: Transmitter enable */
#define UART_UCR2_ATEN         (1 << 3)  /* Bit 2: Aging Timer Enable */
#define UART_UCR2_RTSEN        (1 << 4)  /* Bit 4: RTS interrupt enable/disable */
#define UART_UCR2_WS           (1 << 5)  /* Bit 5: Word size */
#define UART_UCR2_STPB         (1 << 6)  /* Bit 6: Controls number of stop bits */
#define UART_UCR2_PROE         (1 << 7)  /* Bit 7: Parity Odd/Even */
#define UART_UCR2_PREN         (1 << 8)  /* Bit 8: Parity enable */
#define UART_UCR2_RTEC_SHIFT   9         /* Bit 9-10: Request to send edge control */
#define UART_UCR2_RTEC_MASK    (3 << UART_UCR2_RTEC_SHIFT)
#  define UART_UCR2_RTEC_RISE  (0 << UART_UCR2_RTEC_SHIFT) /* Interrupt on rising edge */
#  define UART_UCR2_RTEC_FALL  (1 << UART_UCR2_RTEC_SHIFT) /* Interrupt on falling edge */
#  define UART_UCR2_RTEC_BOTH  (2 << UART_UCR2_RTEC_SHIFT) /* Interrupt on any edge */
#define UART_UCR2_ESCEN        (1 << 11) /* Bit 11: Escape enable */
#define UART_UCR2_CTS          (1 << 12) /* Bit 12: Clear To Send pin */
#define UART_UCR2_CTSC         (1 << 13) /* Bit 13: CTS Pin control */
#define UART_UCR2_IRTS         (1 << 14) /* Bit 14: Ignore RTS Pin */
#define UART_UCR2_ESCI         (1 << 15) /* Bit 15: Escape Sequence Interrupt Enable */

/* UART Control Register 3 */

#define UART_UCR3_ACIEN        (1 << 0)  /* Bit 0:  Autobaud Counter Interrupt Enable */
#define UART_UCR3_INVT         (1 << 1)  /* Bit 1:  Inverted output */
#define UART_UCR3_RXDMUXSEL    (1 << 2)  /* Bit 2:  RXD muxed input selected */
#define UART_UCR3_DTRDEN       (1 << 3)  /* Bit 3:  Data Terminal Ready delta enable */
#define UART_UCR3_AWAKEN       (1 << 4)  /* Bit 4:  Asynchronous wake interrupt enable */
#define UART_UCR3_AIRINTEN     (1 << 5)  /* Bit 5:  Asynchronous IR Wake interrupt enable */
#define UART_UCR3_RXDSEN       (1 << 6)  /* Bit 6:  Receive status interrupt enable */
#define UART_UCR3_ADNIMP       (1 << 7)  /* Bit 7:  Autobaud Detection Not Improved */
#define UART_UCR3_RI           (1 << 8)  /* Bit 8:  Ring Indicator */
#define UART_UCR3_DCD          (1 << 9)  /* Bit 9:  Data Carrier Detect */
#define UART_UCR3_DSR          (1 << 10) /* Bit 10: Data Set Ready */
#define UART_UCR3_FRAERREN     (1 << 11) /* Bit 11: Frame error interrupt enable */
#define UART_UCR3_PARERREN     (1 << 12) /* Bit 12: Parity error interrupt enable */
#define UART_UCR3_DTREN        (1 << 13) /* Bit 13: Data Terminal Ready interrupt enable */
#define UART_UCR3_DPEC_SHIFT   (14)      /* Bits 14-15: DTR/DSR interrupt edge control */
#define UART_UCR3_DPEC_MASK    (3 << UART_UCR3_DPEC_SHIFT)
#  define UART1_DPEC_RISING    (0 << UART_UCR3_DPEC_SHIFT) /* Interrupt on rising edge */
#  define UART1_DPEC_FALLING   (1 << UART_UCR3_DPEC_SHIFT) /* Interrupt on falling edge */
#  define UART1_DPEC_BOTH      (2 << UART_UCR3_DPEC_SHIFT) /* Interrupt on either edge */

/* UART Control Register 4 */

#define UART_UCR4_DREN         (1 << 0)  /* Bit 0:  Receive data ready interrupt enable */
#define UART_UCR4_OREN         (1 << 1)  /* Bit 1:  Receiver overrun interrupt enable */
#define UART_UCR4_BKEN         (1 << 2)  /* Bit 2:  Break condition detected interrupt enable */
#define UART_UCR4_TCEN         (1 << 3)  /* Bit 3:  Transmit complete interrupt enable */
#define UART_UCR4_LPBYP        (1 << 4)  /* Bit 4:  Low Power B */
#define UART_UCR4_IRSC         (1 << 5)  /* Bit 5:  IR special case */
#define UART_UCR4_IDDMAEN      (1 << 6)  /* Bit 6:  DMA IDLE Condition Detected interrupt enable */
#define UART_UCR4_WKEN         (1 << 7)  /* Bit 7:  Wake interrupt enable */
#define UART_UCR4_ENIRI        (1 << 8)  /* Bit 8:  Serial infrared interrupt enable */
#define UART_UCR4_INVR         (1 << 9)  /* Bit 9:  Inverted reception */
#define UART_UCR4_CTSTL_SHIFT  10        /* Bits 10-15: CTS trigger level */
#define UART_UCR4_CTSTL_MASK   (0x3f << UART_UCR4_CTSTL_SHIFT)
#  define UART_UCR4_CTSTL(n)   ((uint32_t)(n) << UART_UCR4_CTSTL_SHIFT)

/* UART FIFO Control Register */

#define UART_UFCR_RXTL_SHIFT   0         /* Bits 0-6: Receiver Trigger Level */
#define UART_UFCR_RXTL_MASK    (0x3f << UART_UFCR_RXTL_SHIFT)
#  define UART_UFCR_RXTL(n)    ((uint32_t)(n) << UART_UFCR_RXTL_SHIFT)
#define UART_UFCR_RFDIV_SHIFT  7         /* Bits 7-9: Reference Frequency Divider */
#define UART_UFCR_RFDIV_MASK   (7 << UART_UFCR_RFDIV_SHIFT)
#  define UART_UFCR_RFDIV6     (0 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 6 */
#  define UART_UFCR_RFDIV5     (1 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 5 */
#  define UART_UFCR_RFDIV4     (2 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 4 */
#  define UART_UFCR_RFDIV3     (3 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 3 */
#  define UART_UFCR_RFDIV2     (4 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 2 */
#  define UART_UFCR_RFDIV1     (5 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 1 */
#  define UART_UFCR_RFDIV7     (6 << UART_UFCR_RFDIV_SHIFT) /* Divide input clock by 7 */
#define UART_UFCR_TXTL_SHIFT   10        /* Bits 10-15: Transmitter Trigger Level */
#define UART_UFCR_TXTL_MASK    (0x3f << UART_UFCR_TXTL_SHIFT)
#  define UART_UFCR_TXTL(n)    ((uint32_t)(n) << UART_UFCR_TXTL_SHIFT)

/* UART Status 1 Register  */

#define UART_USR1_SAD          (1 << 3)  /* Bit 3:  RS-485 Slave Address Detected Interrupt Flag */
#define UART_USR1_AWAKE        (1 << 4)  /* Bit 4:  Asynchronous WAKE Interrupt Flag */
#define UART_USR1_AIRINT       (1 << 5)  /* Bit 5:  Asynchronous IR WAKE Interrupt Flag */
#define UART_USR1_RXDS         (1 << 6)  /* Bit 6:  Receiver IDLE Interrupt Flag */
#define UART_USR1_DTRD         (1 << 7)  /* Bit 7:  DTR Delta */
#define UART_USR1_AGTIM        (1 << 8)  /* Bit 8:  Aging Timer Interrupt Flag */
#define UART_USR1_RRDY         (1 << 9)  /* Bit 9:  RX Ready Interrupt/DMA Flag */
#define UART_USR1_FRAMERR      (1 << 10) /* Bit 10: Frame Error Interrupt Flag */
#define UART_USR1_ESCF         (1 << 11) /* Bit 11: Escape Sequence Interrupt Flag */
#define UART_USR1_RTSD         (1 << 12) /* Bit 12: RTS Delta */
#define UART_USR1_TRDY         (1 << 13) /* Bit 13: TX Ready Interrupt/DMA Flag */
#define UART_USR1_RTSS         (1 << 14) /* Bit 14: RTS_B Pin Status */
#define UART_USR1_PARITYERR    (1 << 15) /* Bit 15: Parity Error Interrupt Flag */

/* UART Status 2 Register */

#define UART_USR2_RDR          (1 << 0)  /* Bit 0:  Receive data ready  */
#define UART_USR2_ORE          (1 << 1)  /* Bit 1:  Overrun error  */
#define UART_USR2_BRCD         (1 << 2)  /* Bit 2:  Break condition detected */
#define UART_USR2_TXDC         (1 << 3)  /* Bit 3:  Transmitter complete */
#define UART_USR2_RTSF         (1 << 4)  /* Bit 4:  RTS Edge Triggered Interrupt flag */
#define UART_USR2_DCDIN        (1 << 5)  /* Bit 5:  Data Carrier Detect Input */
#define UART_USR2_DCDDELT      (1 << 6)  /* Bit 6:  Data Carrier Detect Delta */
#define UART_USR2_WAKE         (1 << 7)  /* Bit 7:  Wake */
#define UART_USR2_IRINT        (1 << 8)  /* Bit 8:  Serial infrared interrupt flag */
#define UART_USR2_RIIN         (1 << 9)  /* Bit 9:  Ring Indicator Input */
#define UART_USR2_RIDELT       (1 << 10) /* Bit 10: Ring Indicator */
#define UART_USR2_ACST         (1 << 11) /* Bit 11: Autobaud Counter Stopped */
#define UART_USR2_IDLE         (1 << 12) /* Bit 12: Idle condition */
#define UART_USR2_DTRF         (1 << 13) /* Bit 13: DTR edge triggered interrupt flag */
#define UART_USR2_TXFE         (1 << 14) /* Bit 14: Transmit Buffer FIFO empty */
#define UART_USR2_ADET         (1 << 15) /* Bit 15: Automatic baud rate detection complete */

/* UART Escape Character Register */

#define UART_UESC_MASK         0xff      /* Bits 0-7: UART Escape Character */

/* UART Escape Timer Register */

#define UART_UTIM_MASK         0xfff     /* Bits 0-11: UART Escape Timer */

/* UART BRM Incremental Register */

#define UART_UBIR_MASK         0xffff    /* Bits 0-15: Incremental Numerator */

/* UART BRM Modulator Register */

#define UART_UBMR_MASK         0xffff    /* Bits 0-15: Modulator Denominator */

/* UART Baud Rate Count Register */

#define UART_UBRC_MASK         0xffff    /* Bits 0-15: Baud Rate Count Register */

/* UART One Millisecond Register */

#define UART_ONEMS_MASK        0xffffff  /* Bits 0-23: One Millisecond Register */

/* UART Test Register */

#define UART_UTS_SOFTRST       (1 << 0)  /* Bit 0:  Software Reset */
#define UART_UTS_RXFULL        (1 << 3)  /* Bit 3:  RxFIFO FULL */
#define UART_UTS_TXFULL        (1 << 4)  /* Bit 4:  TxFIFO FULL */
#define UART_UTS_RXEMPTY       (1 << 5)  /* Bit 5:  RxFIFO Empty */
#define UART_UTS_TXEMPTY       (1 << 6)  /* Bit 6:  TxFIFO Empty */
#define UART_UTS_RXDBG         (1 << 9)  /* Bit 9:  RX FIFO debug mode */
#define UART_UTS_LOOPIR        (1 << 10) /* Bit 10: Loop TX and RX for IR Test (LOOPIR)*/
#define UART_UTS_DBGEN         (1 << 11) /* Bit 11: Debug enable B */
#define UART_UTS_LOOP          (1 << 12) /* Bit 12: Loop TX and RX for Test */
#define UART_UTS_FRCPERR       (1 << 13) /* Bit 13: Force Parity Error */

/* UART RS-485 Mode Control Register */

#define UART_UMCR_MDEN         (1 << 0)  /* Bit 0:  9-bit data or Multidrop Mode (RS-485) Enable */
#define UART_UMCR_SLAM         (1 << 1)  /* Bit 1:  RS-485 Slave Address Detect Mode Selection */
#define UART_UMCR_TXB8         (1 << 2)  /* Bit 2:  Transmit RS-485 bit 8 */
#define UART_UMCR_SADEN        (1 << 3)  /* Bit 3:  RS-485 Slave Address Detected Interrupt Enable */
#define UART_UMCR_SLADDR_SHIFT (8)       /* Bits 8-15: RS-485 Slave Address Character */
#define UART_UMCR_SLADDR_MASK  (0xff << UART_UMCR_SLADDR_SHIFT)
#  define UART_UMCR_SLADDR(n)  ((uint32_t)(n) << UART_UMCR_SLADDR_SHIFT)

#endif /* __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_UART_H */
