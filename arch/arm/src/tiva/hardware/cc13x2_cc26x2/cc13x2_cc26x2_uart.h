/************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_uart.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X20_CC26X2_CC13X20_CC26X2_UART_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X20_CC26X2_CC13X20_CC26X2_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* UART register offsets ************************************************************/

#define TIVA_UART_DR_OFFSET        0x0000 /* UART Data */
#define TIVA_UART_RSR_OFFSET       0x0004 /* UART Receive Status */
#define TIVA_UART_ECR_OFFSET       0x0004 /* UART Error Clear */
#define TIVA_UART_FR_OFFSET        0x0018 /* UART Flag */
#define TIVA_UART_IBRD_OFFSET      0x0024 /* UART Integer Baud-Rate Divisor*/
#define TIVA_UART_FBRD_OFFSET      0x0028 /* UART Fractional Baud-Rate Divisor */
#define TIVA_UART_LCRH_OFFSET      0x002c /* UART Line Control */
#define TIVA_UART_CTL_OFFSET       0x0030 /* UART Control */
#define TIVA_UART_IFLS_OFFSET      0x0034 /* UART Interrupt FIFO Level Select */
#define TIVA_UART_IM_OFFSET        0x0038 /* UART Interrupt Mask */
#define TIVA_UART_RIS_OFFSET       0x003c /* UART Raw Interrupt Status */
#define TIVA_UART_MIS_OFFSET       0x0040 /* UART Masked Interrupt Status */
#define TIVA_UART_ICR_OFFSET       0x0044 /* UART Interrupt Clear */
#define TIVA_UART_DMACTL_OFFSET    0x0048 /* UART DMA Control */

/* UART register addresses **********************************************************/

#define TIVA_UART_BASE(n)          (TIVA_UART0_BASE + (n)*0x01000)

#define TIVA_UART_DR(n)            (TIVA_UART_BASE(n) + TIVA_UART_DR_OFFSET)
#define TIVA_UART_RSR(n)           (TIVA_UART_BASE(n) + TIVA_UART_RSR_OFFSET)
#define TIVA_UART_ECR(n)           (TIVA_UART_BASE(n) + TIVA_UART_ECR_OFFSET)
#define TIVA_UART_FR(n)            (TIVA_UART_BASE(n) + TIVA_UART_FR_OFFSET)
#define TIVA_UART_IBRD(n)          (TIVA_UART_BASE(n) + TIVA_UART_IBRD_OFFSET)
#define TIVA_UART_FBRD(n)          (TIVA_UART_BASE(n) + TIVA_UART_FBRD_OFFSET)
#define TIVA_UART_LCRH(n)          (TIVA_UART_BASE(n) + TIVA_UART_LCRH_OFFSET)
#define TIVA_UART_CTL(n)           (TIVA_UART_BASE(n) + TIVA_UART_CTL_OFFSET)
#define TIVA_UART_IFLS(n)          (TIVA_UART_BASE(n) + TIVA_UART_IFLS_OFFSET)
#define TIVA_UART_IM(n)            (TIVA_UART_BASE(n) + TIVA_UART_IM_OFFSET)
#define TIVA_UART_RIS(n)           (TIVA_UART_BASE(n) + TIVA_UART_RIS_OFFSET)
#define TIVA_UART_MIS(n)           (TIVA_UART_BASE(n) + TIVA_UART_MIS_OFFSET)
#define TIVA_UART_ICR(n)           (TIVA_UART_BASE(n) + TIVA_UART_ICR_OFFSET)
#define TIVA_UART_DMACTL(n)        (TIVA_UART_BASE(n) + TIVA_UART_DMACTL_OFFSET)

#if TIVA_NUARTS > 0
#  define TIVA_UART0_DR            (TIVA_UART0_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART0_RSR           (TIVA_UART0_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART0_ECR           (TIVA_UART0_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART0_FR            (TIVA_UART0_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART0_IBRD          (TIVA_UART0_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART0_FBRD          (TIVA_UART0_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART0_LCRH          (TIVA_UART0_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART0_CTL           (TIVA_UART0_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART0_IFLS          (TIVA_UART0_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART0_IM            (TIVA_UART0_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART0_RIS           (TIVA_UART0_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART0_MIS           (TIVA_UART0_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART0_ICR           (TIVA_UART0_BASE + TIVA_UART_ICR_OFFSET)
#  define TIVA_UART0_DMACTL        (TIVA_UART0_BASE + TIVA_UART_DMACTL_OFFSET)
#endif

#if TIVA_NUARTS > 1
#  define TIVA_UART1_DR            (TIVA_UART1_BASE + TIVA_UART_DR_OFFSET)
#  define TIVA_UART1_RSR           (TIVA_UART1_BASE + TIVA_UART_RSR_OFFSET)
#  define TIVA_UART1_ECR           (TIVA_UART1_BASE + TIVA_UART_ECR_OFFSET)
#  define TIVA_UART1_FR            (TIVA_UART1_BASE + TIVA_UART_FR_OFFSET)
#  define TIVA_UART1_IBRD          (TIVA_UART1_BASE + TIVA_UART_IBRD_OFFSET)
#  define TIVA_UART1_FBRD          (TIVA_UART1_BASE + TIVA_UART_FBRD_OFFSET)
#  define TIVA_UART1_LCRH          (TIVA_UART1_BASE + TIVA_UART_LCRH_OFFSET)
#  define TIVA_UART1_CTL           (TIVA_UART1_BASE + TIVA_UART_CTL_OFFSET)
#  define TIVA_UART1_IFLS          (TIVA_UART1_BASE + TIVA_UART_IFLS_OFFSET)
#  define TIVA_UART1_IM            (TIVA_UART1_BASE + TIVA_UART_IM_OFFSET)
#  define TIVA_UART1_RIS           (TIVA_UART1_BASE + TIVA_UART_RIS_OFFSET)
#  define TIVA_UART1_MIS           (TIVA_UART1_BASE + TIVA_UART_MIS_OFFSET)
#  define TIVA_UART1_ICR           (TIVA_UART1_BASE + TIVA_UART_ICR_OFFSET)
#  define TIVA_UART1_DMACTL        (TIVA_UART1_BASE + TIVA_UART_DMACTL_OFFSET)
#endif

/* UART register bit settings *******************************************************/

/* UART Data (DR) */

#define UART_DR_DATA_SHIFT         0         /* Bits 7-0: Data Transmitted or Received */
#define UART_DR_DATA_MASK          (0xff << UART_DR_DATA_SHIFT)
#define UART_DR_FE                 (1 << 8)  /* Bit 8:  UART Framing Error */
#define UART_DR_PE                 (1 << 9)  /* Bit 9:  UART Parity Error */
#define UART_DR_BE                 (1 << 10) /* Bit 10: UART Break Error */
#define UART_DR_OE                 (1 << 11) /* Bit 11: UART Overrun Error */

/* UART Receive Status (RSR) */

#define UART_RSR_FE                (1 << 0)  /* Bit 0:  UART Framing Error */
#define UART_RSR_PE                (1 << 1)  /* Bit 1:  UART Parity Error */
#define UART_RSR_BE                (1 << 2)  /* Bit 2:  UART Break Error */
#define UART_RSR_OE                (1 << 3)  /* Bit 3:  UART Overrun Error */

/* UART Error Clear (ECR) */

#define UART_ECR_FE                (1 << 0)  /* Bit 0:  UART Clear Framing Error */
#define UART_ECR_PE                (1 << 1)  /* Bit 1:  UART Clear Parity Error */
#define UART_ECR_BE                (1 << 2)  /* Bit 2:  UART Clear Break Error */
#define UART_ECR_OE                (1 << 3)  /* Bit 3:  UART Clear Overrun Error */

/* UART Flag (FR) */

#define UART_FR_CTS                (1 << 0)  /* Bit 0:  Data Set Ready */
#define UART_FR_BUSY               (1 << 3)  /* Bit 3:  UART Busy */
#define UART_FR_RXFE               (1 << 4)  /* Bit 4:  UART Receive FIFO Empty */
#define UART_FR_TXFF               (1 << 5)  /* Bit 5:  UART Transmit FIFO Full */
#define UART_FR_RXFF               (1 << 6)  /* Bit 6:  UART Receive FIFO Full */
#define UART_FR_TXFE               (1 << 7)  /* Bit 7:  UART Transmit FIFO Empty */

/* UART Integer Baud-Rate Divisor (IBRD) */

#define UART_IBRD_DIVINT_MASK      (0xffff)  /* Bits 15-0: Integer Baud-Rate Divisor */

/* UART Fractional Baud-Rate Divisor (UARTFBRD) */

#define UART_FBRD_DIVFRAC_MASK     (0x3f)    /* Bits 5-0: Fractional Baud-Rate Divisor */

/* Register 7: UART Line Control (LCRH) */

#define UART_LCRH_BRK              (1 << 0)  /* Bit 0:  UART Send Break */
#define UART_LCRH_PEN              (1 << 1)  /* Bit 1:  UART Parity Enable */
#define UART_LCRH_EPS              (1 << 2)  /* Bit 2:  UART Even Parity Select */
#define UART_LCRH_STP2             (1 << 3)  /* Bit 3:  UART Two Stop Bits Select */
#define UART_LCRH_FEN              (1 << 4)  /* Bit 4:  UART Enable FIFOs */
#define UART_LCRH_WLEN_SHIFT       5         /* Bits 6-5: UART Word Length */

#define UART_LCRH_WLEN_MASK        (3 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_5BITS     (0 << UART_LCRH_WLEN_SHIFT) /* 5-bits (reset) */
#  define UART_LCRH_WLEN_6BITS     (1 << UART_LCRH_WLEN_SHIFT) /* 6-bits */
#  define UART_LCRH_WLEN_7BITS     (2 << UART_LCRH_WLEN_SHIFT) /* 7-bits */
#  define UART_LCRH_WLEN_8BITS     (3 << UART_LCRH_WLEN_SHIFT) /* 8-bits */

#define UART_LCRH_SPS              (1 << 7)  /* Bit 7:  UART Stick Parity Select */

/* UART Control (CTL) */

#define UART_CTL_UARTEN            (1 << 0)  /* Bit 0:  UART Enable */
#define UART_CTL_LBE               (1 << 7)  /* Bit 7:  UART Loop Back Enable */
#define UART_CTL_TXE               (1 << 8)  /* Bit 8:  UART Transmit Enable */
#define UART_CTL_RXE               (1 << 9)  /* Bit 9:  UART Receive Enable */
#define UART_CTL_DTR               (1 << 10) /* Bit 10: Data Terminal Ready */
#define UART_CTL_RTS               (1 << 11) /* Bit 11: Request to Send */
#define UART_CTL_RTSEN             (1 << 14) /* Bit 14: Enable Request to Send */
#define UART_CTL_CTSEN             (1 << 15) /* Bit 15: Enable Clear To Send */

/* UART Interrupt FIFO Level Select (IFLS) */

#define UART_IFLS_TXIFLSEL_SHIFT   0         /* Bits 0-2: UART Transmit Interrupt FIFO Level Select */
#define UART_IFLS_TXIFLSEL_MASK    (7 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TXIFLSEL_18TH  (0 << UART_IFLS_TXIFLSEL_SHIFT) /* 1/8th full */
#  define UART_IFLS_TXIFLSEL_14TH  (1 << UART_IFLS_TXIFLSEL_SHIFT) /* 1/4th full */
#  define UART_IFLS_TXIFLSEL_HALF  (2 << UART_IFLS_TXIFLSEL_SHIFT) /* half full */
#  define UART_IFLS_TXIFLSEL_34TH  (3 << UART_IFLS_TXIFLSEL_SHIFT) /* 3/4th full */
#  define UART_IFLS_TXIFLSEL_78TH  (4 << UART_IFLS_TXIFLSEL_SHIFT) /* 7/8th full */

#define UART_IFLS_RXIFLSEL_SHIFT   3         /* Bits 3-5: UART Receive Interrupt FIFO Level Select */
#define UART_IFLS_RXIFLSEL_MASK    (7 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RXIFLSEL_18TH  (0 << UART_IFLS_RXIFLSEL_SHIFT) /* 1/8th full */
#  define UART_IFLS_RXIFLSEL_14TH  (1 << UART_IFLS_RXIFLSEL_SHIFT) /* 1/4th full */
#  define UART_IFLS_RXIFLSEL_HALF  (2 << UART_IFLS_RXIFLSEL_SHIFT) /* half full */
#  define UART_IFLS_RXIFLSEL_34TH  (3 << UART_IFLS_RXIFLSEL_SHIFT) /* 3/4th full */
#  define UART_IFLS_RXIFLSEL_78TH  (4 << UART_IFLS_RXIFLSEL_SHIFT) /* 7/8th full */

/* UART Interrupt Mask (IM) */

#define UART_IM_CTSIM              (1 << 1)  /* Bit 1:  UART Clear to Send Modem Interrupt Mask */
#define UART_IM_RXIM               (1 << 4)  /* Bit 4:  UART Receive Interrupt Mask */
#define UART_IM_TXIM               (1 << 5)  /* Bit 5:  UART Transmit Interrupt Mask */
#define UART_IM_RTIM               (1 << 6)  /* Bit 6:  UART Receive Time-Out Interrupt Mask */
#define UART_IM_FEIM               (1 << 7)  /* Bit 7:  UART Framing Error Interrupt Mask */
#define UART_IM_PEIM               (1 << 8)  /* Bit 8:  UART Parity Error Interrupt Mask */
#define UART_IM_BEIM               (1 << 9)  /* Bit 9:  UART Break Error Interrupt Mask */
#define UART_IM_OEIM               (1 << 10) /* Bit 10: UART Overrun Error Interrupt Mask */
#define UART_IM_EOTIM              (1 << 11) /* Bit 11: End of Transmission Interrupt Mask */

/* UART Raw Interrupt Status (RIS) */

#define UART_RIS_CTSIS             (1 << 1)  /* Bit 1:  UART Clear to Send Modem Raw Interrupt Status */
#define UART_RIS_RXRIS             (1 << 4)  /* Bit 4:  UART Receive Raw Interrupt Status */
#define UART_RIS_TXRIS             (1 << 5)  /* Bit 5:  UART Transmit Raw Interrupt Status */
#define UART_RIS_RTRIS             (1 << 6)  /* Bit 6:  UART Receive Time-Out Raw Interrupt Status */
#define UART_RIS_FERIS             (1 << 7)  /* Bit 7:  UART Framing Error Raw Interrupt Status */
#define UART_RIS_PERIS             (1 << 8)  /* Bit 8:  UART Parity Error Raw Interrupt Status */
#define UART_RIS_BERIS             (1 << 9)  /* Bit 9:  UART Break Error Raw Interrupt Status */
#define UART_RIS_OERIS             (1 << 10) /* Bit 10: UART Overrun Error Raw Interrupt Status */
#define UART_RIS_EOTIS             (1 << 11) /* Bit 11: End of Transmission Raw Interrupt Status */

/* UART Masked Interrupt Status (MIS) */

#define UART_MIS_CTSIS             (1 << 1)  /* Bit 1:  UART Clear to Send Modem Masked Interrupt Status */
#define UART_MIS_RXMIS             (1 << 4)  /* Bit 4:  UART Receive Masked Interrupt Status */
#define UART_MIS_TXMIS             (1 << 5)  /* Bit 5:  UART Transmit Masked Interrupt Status */
#define UART_MIS_RTMIS             (1 << 6)  /* Bit 6:  UART Receive Time-Out Masked Interrupt Status */
#define UART_MIS_FEMIS             (1 << 7)  /* Bit 7:  UART Framing Error Masked Interrupt Status */
#define UART_MIS_PEMIS             (1 << 8)  /* Bit 8:  UART Parity Error Masked Interrupt Status */
#define UART_MIS_BEMIS             (1 << 9)  /* Bit 9:  UART Break Error Masked Interrupt Status */
#define UART_MIS_OEMIS             (1 << 10) /* Bit 10: UART Overrun Error Masked Interrupt Status */
#define UART_MIS_EOTIS             (1 << 11) /* Bit 11: End of Transmission Masked Interrupt Status */

/* UART Interrupt Clear (ICR) */

#define UART_ICR_CTSIC             (1 << 1)  /* Bit 1:  UART Clear to Send Modem Interrupt Clear */
#define UART_ICR_RXIC              (1 << 4)  /* Bit 4:  Receive Interrupt Clear */
#define UART_ICR_TXIC              (1 << 5)  /* Bit 5:  Transmit Interrupt Clear */
#define UART_ICR_RTIC              (1 << 6)  /* Bit 6:  Receive Time-Out Interrupt Clear */
#define UART_ICR_FEIC              (1 << 7)  /* Bit 7:  Framing Error Interrupt Clear */
#define UART_ICR_PEIC              (1 << 8)  /* Bit 8:  Parity Error Interrupt Clear */
#define UART_ICR_BEIC              (1 << 9)  /* Bit 9:  Break Error Interrupt Clear */
#define UART_ICR_OEIC              (1 << 10) /* Bit 10: Overrun Error Interrupt Clear */
#define UART_ICR_EOTIC             (1 << 11) /* Bit 11: End of Transmission Interrupt Clear */

/* UART DMA Control (DMACTL) */

#define UART_DMACTL_RXDMAE         (1 << 0)  /* Bit 0:  Receive DMA Enable */
#define UART_DMACTL_TXDMAE         (1 << 1)  /* Bit 1:  Transmit DMA Enable */
#define UART_DMACTL_DMAERR         (1 << 2)  /* Bit 2:  DMA on Error */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X20_CC26X2_CC13X20_CC26X2_UART_H */
