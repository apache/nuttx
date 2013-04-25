/************************************************************************************
 * arch/arm/src/kl/kl_uart.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KL_KL_UART_H
#define __ARCH_ARM_SRC_KL_KL_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "kl_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KL_UART_BDH_OFFSET          0x0000 /* UART Baud Rate Register High */
#define KL_UART_BDL_OFFSET          0x0001 /* UART Baud Rate Register Low */
#define KL_UART_C1_OFFSET           0x0002 /* UART Control Register 1 */
#define KL_UART_C2_OFFSET           0x0003 /* UART Control Register 2 */
#define KL_UART_S1_OFFSET           0x0004 /* UART Status Register 1 */
#define KL_UART_S2_OFFSET           0x0005 /* UART Status Register 2 */
#define KL_UART_C3_OFFSET           0x0006 /* UART Control Register 3 */
#define KL_UART_D_OFFSET            0x0007 /* UART Data Register */
#define KL_UART_MA1_OFFSET          0x0008 /* UART Match Address Registers 1 (UART0)*/
#define KL_UART_MA2_OFFSET          0x0009 /* UART Match Address Registers 2 (UART0)*/
#define KL_UART_C4_OFFSET           0x000a /* UART Control Register 4 */
#define KL_UART_C5_OFFSET           0x000b /* UART Control Register 5 (UART0) */

/* Register Addresses ***************************************************************/

#if (KL_NUART) > 0
#  define KL_UART0_BDH              (KL_UART0_BASE+KL_UART_BDH_OFFSET)
#  define KL_UART0_BDL              (KL_UART0_BASE+KL_UART_BDL_OFFSET)
#  define KL_UART0_C1               (KL_UART0_BASE+KL_UART_C1_OFFSET)
#  define KL_UART0_C2               (KL_UART0_BASE+KL_UART_C2_OFFSET)
#  define KL_UART0_S1               (KL_UART0_BASE+KL_UART_S1_OFFSET)
#  define KL_UART0_S2               (KL_UART0_BASE+KL_UART_S2_OFFSET)
#  define KL_UART0_C3               (KL_UART0_BASE+KL_UART_C3_OFFSET)
#  define KL_UART0_D                (KL_UART0_BASE+KL_UART_D_OFFSET)
#  define KL_UART0_MA1              (KL_UART0_BASE+KL_UART_MA1_OFFSET)
#  define KL_UART0_MA2              (KL_UART0_BASE+KL_UART_MA2_OFFSET)
#  define KL_UART0_C4               (KL_UART0_BASE+KL_UART_C4_OFFSET)
#  define KL_UART0_C5               (KL_UART0_BASE+KL_UART_C5_OFFSET)
#endif

#if (KL_NUART) > 1
#  define KL_UART1_BDH              (KL_UART1_BASE+KL_UART_BDH_OFFSET)
#  define KL_UART1_BDL              (KL_UART1_BASE+KL_UART_BDL_OFFSET)
#  define KL_UART1_C1               (KL_UART1_BASE+KL_UART_C1_OFFSET)
#  define KL_UART1_C2               (KL_UART1_BASE+KL_UART_C2_OFFSET)
#  define KL_UART1_S1               (KL_UART1_BASE+KL_UART_S1_OFFSET)
#  define KL_UART1_S2               (KL_UART1_BASE+KL_UART_S2_OFFSET)
#  define KL_UART1_C3               (KL_UART1_BASE+KL_UART_C3_OFFSET)
#  define KL_UART1_D                (KL_UART1_BASE+KL_UART_D_OFFSET)
#  define KL_UART1_C4               (KL_UART1_BASE+KL_UART_C4_OFFSET)
#endif

#if (KL_NUART) > 2
#  define KL_UART2_BDH              (KL_UART2_BASE+KL_UART_BDH_OFFSET)
#  define KL_UART2_BDL              (KL_UART2_BASE+KL_UART_BDL_OFFSET)
#  define KL_UART2_C1               (KL_UART2_BASE+KL_UART_C1_OFFSET)
#  define KL_UART2_C2               (KL_UART2_BASE+KL_UART_C2_OFFSET)
#  define KL_UART2_S1               (KL_UART2_BASE+KL_UART_S1_OFFSET)
#  define KL_UART2_S2               (KL_UART2_BASE+KL_UART_S2_OFFSET)
#  define KL_UART2_C3               (KL_UART2_BASE+KL_UART_C3_OFFSET)
#  define KL_UART2_D                (KL_UART2_BASE+KL_UART_D_OFFSET)
#  define KL_UART2_C4               (KL_UART2_BASE+KL_UART_C4_OFFSET)
#endif

/* Register Bit Definitions *********************************************************/
/* UART Baud Rate Register High */

#define UART_BDH_SBR_SHIFT          (0)       /* Bits 0-4: MS Bits 8-13 of the UART Baud Rate Bits */
#define UART_BDH_SBR_MASK           (31 << UART_BDH_SBR_SHIFT)
#  define UART_BDH_SBR(x)           (((uint8_t)(((uint8_t)(x))<<UART_BDH_SBR_SHIFT))&UART_BDH_SBR_MASK)
#define UART_BDH_SBNS               (1 << 5)  /* Bit 5: Stop Bit Number Select */
#define UART_BDH_RXEDGIE            (1 << 6)  /* Bit 6: RxD Input Active Edge Interrupt Enable */
#define UART_BDH_LBKDIE             (1 << 7)  /* Bit 7: LIN Break Detect Interrupt Enable */

/* UART Baud Rate Register Low.  Bits 0-7 of the UART baud rate bits. */

#define UART_BDL_SBR_MASK           0xffu
#define UART_BDL_SBR_SHIFT          0
#  define UART_BDL_SBR(x)           (((uint8_t)(((uint8_t)(x))<<UART_BDL_SBR_SHIFT))&UART_BDL_SBR_MASK)

/* UART Control Register 1 */

#define UART_C1_PT                  (1 << 0)  /* Bit 0: Parity Type */
#define UART_C1_PE                  (1 << 1)  /* Bit 1: Parity Enable */
#define UART_C1_ILT                 (1 << 2)  /* Bit 2: Idle Line Type Select */
#define UART_C1_WAKE                (1 << 3)  /* Bit 3: Receiver Wakeup Method Select */
#define UART_C1_M                   (1 << 4)  /* Bit 4: 9-bit or 8-bit Mode Select */
#define UART_C1_RSRC                (1 << 5)  /* Bit 5: Receiver Source Select */
#define UART_C1_DOZEEN              (1 << 6)  /* Bit 6: Doze Enable (UART0) */
#define UART_C1_UARTSWAI            (1 << 6)  /* Bit 6: UART Stops in Wait Mode (UART1 and 2) */
#define UART_C1_LOOPS               (1 << 7)  /* Bit 7: Loop Mode Select */

/* UART Control Register 2 */

#define UART_C2_SBK                 (1 << 0)  /* Bit 0: Send Break */
#define UART_C2_RWU                 (1 << 1)  /* Bit 1: Receiver Wakeup Control */
#define UART_C2_RE                  (1 << 2)  /* Bit 2: Receiver Enable */
#define UART_C2_TE                  (1 << 3)  /* Bit 3: Transmitter Enable */
#define UART_C2_ILIE                (1 << 4)  /* Bit 4: Idle Line Interruptor Enable */
#define UART_C2_RIE                 (1 << 5)  /* Bit 5: Receiver Full Interrupt or DMA Transfer Enable */
#define UART_C2_TCIE                (1 << 6)  /* Bit 6: Transmission Complete Interrupt Enable */
#define UART_C2_TIE                 (1 << 7)  /* Bit 7: Transmitter Interrupt or DMA Transfer Enable */
#define UART_C2_ALLINTS             (0xf0)

/* UART Status Register 1 */

#define UART_S1_PF                  (1 << 0)  /* Bit 0: Parity Error Flag */
#define UART_S1_FE                  (1 << 1)  /* Bit 1: Framing Error Flag */
#define UART_S1_NF                  (1 << 2)  /* Bit 2: Noise Flag */
#define UART_S1_OR                  (1 << 3)  /* Bit 3: Receiver Overrun Flag */
#define UART_S1_IDLE                (1 << 4)  /* Bit 4: Idle Line Flag */
#define UART_S1_RDRF                (1 << 5)  /* Bit 5: Receive Data Register Full Flag */
#define UART_S1_TC                  (1 << 6)  /* Bit 6: Transmit Complete Flag */
#define UART_S1_TDRE                (1 << 7)  /* Bit 7: Transmit Data Register Empty Flag */
#define UART_S1_ERRORS              (0x0f)

/* UART Status Register 2 */

#define UART_S2_RAF                 (1 << 0)  /* Bit 0: Receiver Active Flag */
#define UART_S2_LBKDE               (1 << 1)  /* Bit 1: LIN Break Detection Enable */
#define UART_S2_BRK13               (1 << 2)  /* Bit 2: Break Transmit Character Length */
#define UART_S2_RWUID               (1 << 3)  /* Bit 3: Receive Wakeup Idle Detect */
#define UART_S2_RXINV               (1 << 4)  /* Bit 4: Receive Data Inversion */
#define UART_S2_MSBF                (1 << 5)  /* Bit 5: Most Significant Bit First (UART0) */
#define UART_S2_RXEDGIF             (1 << 6)  /* Bit 6: RxD Pin Active Edge Interrupt Flag */
#define UART_S2_LBKDIF              (1 << 7)  /* Bit 7: LIN Break Detect Interrupt Flag */

/* UART Control Register 3 */

#define UART_C3_PEIE                (1 << 0)  /* Bit 0: Parity Error Interrupt Enable */
#define UART_C3_FEIE                (1 << 1)  /* Bit 1: Framing Error Interrupt Enable */
#define UART_C3_NEIE                (1 << 2)  /* Bit 2: Noise Error Interrupt Enable */
#define UART_C3_ORIE                (1 << 3)  /* Bit 3: Overrun Error Interrupt Enable */
#define UART_C3_TXINV               (1 << 4)  /* Bit 4: Transmit Data Inversion */
#define UART_C3_TXDIR               (1 << 5)  /* Bit 5: Transmitter Pin Data Direction in Single-Wire mode */
#define UART_C3_R9                  (1 << 6)  /* Bit 6: Receive Bit 9 (UART0) */
#define UART_C3_T8                  (1 << 6)  /* Bit 6: Transmit Bit 8 */
#define UART_C3_T9                  (1 << 7)  /* Bit 7: Transmit Bit 9 (UART0) */
#define UART_C3_R8                  (1 << 7)  /* Bit 7: Receive Bit 8 */

/* UART Data Register: 8-bit data register. */
/* UART Match Address Registers 1 & 2: 8-bit address registers */

/* UART Control Register 4 (UART0) */

#define UART_C4_OSR_SHIFT           (0)       /* Bits 0-4: Over Sampling Ratio */
#define UART_C4_OSR_MASK            (0x1f << UART_C4_OSR_SHIFT)
#define UART_C4_M10                 (1 << 5)  /* Bit 5: 10-bit Mode select */
#define UART_C4_MAEN2               (1 << 6)  /* Bit 6: Match Address Mode Enable 2 */
#define UART_C4_MAEN1               (1 << 7)  /* Bit 7: Match Address Mode Enable 1 */

/* UART Control Register 4 (UART1 and 2) */

                                              /* Bit 0-4: Reserved */
#define UART_C4_RDMAS               (1 << 5)  /* Bit 5: This field is reserved */
                                              /* Bit 6: Reserved */
#define UART_C4_TDMAS               (1 << 7)  /* Bit 7: Transmitter DMA Select */

/* UART Control Register 5 (UART0) */

#define UART_C5_RESYNCDIS           (1 << 0)  /* Bit 0: Resynchronization Disable */
#define UART_C5_BOTHEDGE            (1 << 1)  /* Bit 1: Both Edge Sampling */
                                              /* Bit 2-4: Reserved */
#define UART_C5_RDMAE               (1 << 5)  /* Bit 5: Receiver Full DMA Enable */
                                              /* Bit 6: Reserved */
#define UART_C5_TDMAE               (1 << 7)  /* Bit 7: Transmitter DMA Enable */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_KL_UART_H */
