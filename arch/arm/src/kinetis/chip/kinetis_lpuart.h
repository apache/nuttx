/****************************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_lpuart.h
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane<david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_LPUART_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_LPUART_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define KINETIS_LPUART_BAUD_OFFSET    0x0000 /* Low Power UART Baud Rate Register */
#define KINETIS_LPUART_STAT_OFFSET    0x0004 /* Low Power UART Status Register */
#define KINETIS_LPUART_CTRL_OFFSET    0x0008 /* Low Power UART Control Register */
#define KINETIS_LPUART_DATA_OFFSET    0x000c /* Low Power UART Data Register */
#define KINETIS_LPUART_MATCH_OFFSET   0x0010 /* Low Power UART Match Address Register */
#define KINETIS_LPUART_MODIR_OFFSET   0x0014 /* Low Power UART Modem IrDA Register */
#if defined(KINETIS_LPUART_HAS_FIFO)
#  define KINETIS_LPUART_FIFO_OFFSET  0x0018 /* Low Power UART FIFO Register */
#endif
#if defined(KINETIS_LPUART_HAS_WATER)
#  define KINETIS_LPUART_WATER_OFFSET 0x001c /* Low Power UART Watermark Register */
#endif

/* Register Addresses *******************************************************************************/

#define KINETIS_LPUART0_BAUD          (KINETIS_LPUART0_BASE + KINETIS_LPUART_BAUD_OFFSET)
#define KINETIS_LPUART0_STAT          (KINETIS_LPUART0_BASE + KINETIS_LPUART_STAT_OFFSET)
#define KINETIS_LPUART0_CTRL          (KINETIS_LPUART0_BASE + KINETIS_LPUART_CTRL_OFFSET)
#define KINETIS_LPUART0_DATA          (KINETIS_LPUART0_BASE + KINETIS_LPUART_DATA_OFFSET)
#define KINETIS_LPUART0_MATCH         (KINETIS_LPUART0_BASE + KINETIS_LPUART_MATCH_OFFSET)
#define KINETIS_LPUART0_MODIR         (KINETIS_LPUART0_BASE + KINETIS_LPUART_MODIR_OFFSET)
#if defined(KINETIS_LPUART_HAS_FIFO)
#  define KINETIS_LPUART0_FIFO        (KINETIS_LPUART0_BASE + KINETIS_LPUART_FIFO_OFFSET)
#endif
#if defined(KINETIS_LPUART_HAS_WATER)
#  define KINETIS_LPUART0_WATER       (KINETIS_LPUART0_BASE + KINETIS_LPUART_WATER_OFFSET)
#endif

#define KINETIS_LPUART1_BAUD          (KINETIS_LPUART1_BASE + KINETIS_LPUART_BAUD_OFFSET)
#define KINETIS_LPUART1_STAT          (KINETIS_LPUART1_BASE + KINETIS_LPUART_STAT_OFFSET)
#define KINETIS_LPUART1_CTRL          (KINETIS_LPUART1_BASE + KINETIS_LPUART_CTRL_OFFSET)
#define KINETIS_LPUART1_DATA          (KINETIS_LPUART1_BASE + KINETIS_LPUART_DATA_OFFSET)
#define KINETIS_LPUART1_MATCH         (KINETIS_LPUART1_BASE + KINETIS_LPUART_MATCH_OFFSET)
#define KINETIS_LPUART1_MODIR         (KINETIS_LPUART1_BASE + KINETIS_LPUART_MODIR_OFFSET)
#if defined(KINETIS_LPUART_HAS_FIFO)
#  define KINETIS_LPUART0_FIFO        (KINETIS_LPUART0_BASE + KINETIS_LPUART_FIFO_OFFSET)
#endif
#if defined(KINETIS_LPUART_HAS_WATER)
#  define KINETIS_LPUART0_WATER       (KINETIS_LPUART0_BASE + KINETIS_LPUART_WATER_OFFSET)
#endif

#define KINETIS_LPUART2_BAUD          (KINETIS_LPUART2_BASE + KINETIS_LPUART_BAUD_OFFSET)
#define KINETIS_LPUART2_STAT          (KINETIS_LPUART2_BASE + KINETIS_LPUART_STAT_OFFSET)
#define KINETIS_LPUART2_CTRL          (KINETIS_LPUART2_BASE + KINETIS_LPUART_CTRL_OFFSET)
#define KINETIS_LPUART2_DATA          (KINETIS_LPUART2_BASE + KINETIS_LPUART_DATA_OFFSET)
#define KINETIS_LPUART2_MATCH         (KINETIS_LPUART2_BASE + KINETIS_LPUART_MATCH_OFFSET)
#define KINETIS_LPUART2_MODIR         (KINETIS_LPUART2_BASE + KINETIS_LPUART_MODIR_OFFSET)
#if defined(KINETIS_LPUART_HAS_FIFO)
#  define KINETIS_LPUART2_FIFO        (KINETIS_LPUART2_BASE + KINETIS_LPUART_FIFO_OFFSET)
#endif
#if defined(KINETIS_LPUART_HAS_WATER)
#  define KINETIS_LPUART2_WATER       (KINETIS_LPUART2_BASE + KINETIS_LPUART_WATER_OFFSET)
#endif

#define KINETIS_LPUART3_BAUD          (KINETIS_LPUART3_BASE + KINETIS_LPUART_BAUD_OFFSET)
#define KINETIS_LPUART3_STAT          (KINETIS_LPUART3_BASE + KINETIS_LPUART_STAT_OFFSET)
#define KINETIS_LPUART3_CTRL          (KINETIS_LPUART3_BASE + KINETIS_LPUART_CTRL_OFFSET)
#define KINETIS_LPUART3_DATA          (KINETIS_LPUART3_BASE + KINETIS_LPUART_DATA_OFFSET)
#define KINETIS_LPUART3_MATCH         (KINETIS_LPUART3_BASE + KINETIS_LPUART_MATCH_OFFSET)
#define KINETIS_LPUART3_MODIR         (KINETIS_LPUART3_BASE + KINETIS_LPUART_MODIR_OFFSET)
#if defined(KINETIS_LPUART_HAS_FIFO)
#  define KINETIS_LPUART3_FIFO        (KINETIS_LPUART3_BASE + KINETIS_LPUART_FIFO_OFFSET)
#endif
#if defined(KINETIS_LPUART_HAS_WATER)
#  define KINETIS_LPUART3_WATER       (KINETIS_LPUART3_BASE + KINETIS_LPUART_WATER_OFFSET)
#endif

#define KINETIS_LPUART4_BAUD          (KINETIS_LPUART4_BASE + KINETIS_LPUART_BAUD_OFFSET)
#define KINETIS_LPUART4_STAT          (KINETIS_LPUART4_BASE + KINETIS_LPUART_STAT_OFFSET)
#define KINETIS_LPUART4_CTRL          (KINETIS_LPUART4_BASE + KINETIS_LPUART_CTRL_OFFSET)
#define KINETIS_LPUART4_DATA          (KINETIS_LPUART4_BASE + KINETIS_LPUART_DATA_OFFSET)
#define KINETIS_LPUART4_MATCH         (KINETIS_LPUART4_BASE + KINETIS_LPUART_MATCH_OFFSET)
#define KINETIS_LPUART4_MODIR         (KINETIS_LPUART4_BASE + KINETIS_LPUART_MODIR_OFFSET)
#if defined(KINETIS_LPUART_HAS_FIFO)
#  define KINETIS_LPUART4_FIFO        (KINETIS_LPUART4_BASE + KINETIS_LPUART_FIFO_OFFSET)
#endif
#if defined(KINETIS_LPUART_HAS_WATER)
#  define KINETIS_LPUART4_WATER       (KINETIS_LPUART4_BASE + KINETIS_LPUART_WATER_OFFSET)
#endif

/* Register Bit Definitions *************************************************************************/

/*  Low Power UART Baud Rate Register */

#define LPUART_BAUD_SBR_SHIFT         (0)       /* Bits 0-12:  Baud Rate Modulo Divisor */
#define LPUART_BAUD_SBR_MASK          (0x1fff << LPUART_BAUD_SBR_SHIFT)
#  define LPUART_BAUD_SBR(n)          (((n) & 0x1fff) << LPUART_BAUD_SBR_SHIFT) /* n= 1..8191*/
#define LPUART_BAUD_SBNS              (1 << 13) /* Bit 13:  Stop Bit Number Select */
#define LPUART_BAUD_RXEDGIE           (1 << 14) /* Bit 14:  RX Input Active Edge Interrupt Enable */
#define LPUART_BAUD_LBKDIE            (1 << 15) /* Bit 15:  LIN Break Detect Interrupt Enable */
#define LPUART_BAUD_RESYNCDIS         (1 << 16) /* Bit 16:  Resynchronizations Disable */
#define LPUART_BAUD_BOTHEDGE          (1 << 17) /* Bit 17:  Both Edge Sampling */
#define LPUART_BAUD_MATCFG_SHIFT      (18)      /* Bits 18-19:  Match Configuration */
#define LPUART_BAUD_MATCFG_MASK       (3 << LPUART_BAUD_MATCFG_SHIFT)
#  define LPUART_BAUD_MATCFG_AMW      (0 << LPUART_BAUD_MATCFG_SHIFT) /* Address Match Wakeup */
#  define LPUART_BAUD_MATCFG_IMW      (1 << LPUART_BAUD_MATCFG_SHIFT) /* Idle Match Wakeup */
#  define LPUART_BAUD_MATCFG_MONOFF   (2 << LPUART_BAUD_MATCFG_SHIFT) /* Match On and Match Off */
#  define LPUART_BAUD_MATCFG_RWU      (3 << LPUART_BAUD_MATCFG_SHIFT) /* Enables RWU on Data Match and Match On/Off for transmitter CTS input */
                                                /* Bit 20:  Reserved */
#define LPUART_BAUD_RDMAE             (1 << 21) /* Bit 21:  Receiver Full DMA Enable */
                                                /* Bit 22:  Reserved */
#define LPUART_BAUD_TDMAE             (1 << 23) /* Bit 23:  Transmitter DMA Enable */
#define LPUART_BAUD_OSR_SHIFT         (24)      /* Bits 24-28:  Over Sampling Ratio */
#define LPUART_BAUD_OSR_MASK          (0x1f << LPUART_BAUD_OSR_SHIFT)
#define   LPUART_BAUD_OSR(n)          ((((n)-1) & 0x1f) << LPUART_BAUD_OSR_SHIFT) /* n=4..32 */
#define LPUART_BAUD_M10               (1 << 29) /* Bit 29:  10-bit Mode select */
#define LPUART_BAUD_MAEN2             (1 << 30) /* Bit 30:  Match Address Mode Enable 2 */
#define LPUART_BAUD_MAEN1             (1 << 31) /* Bit 31:  Match Address Mode Enable 1 */

/* Low Power UART Status Register */

                                                /* Bits 0-13:  Reserved */
#define LPUART_STAT_MA2F              (1 << 14) /* Match 2 Flag */
#define LPUART_STAT_MA1F              (1 << 15) /* Match 1 Flag */
#define LPUART_STAT_PF                (1 << 16) /* Parity Error Flag */
#define LPUART_STAT_FE                (1 << 17) /* Framing Error Flag */
#define LPUART_STAT_NF                (1 << 18) /* Noise Flag */
#define LPUART_STAT_OR                (1 << 19) /* Receiver Overrun Flag */
#define LPUART_STAT_IDLE              (1 << 20) /* Idle Line Flag */
#define LPUART_STAT_RDRF              (1 << 21) /* Receive Data Register Full Flag */
#define LPUART_STAT_TC                (1 << 22) /* Transmission Complete Flag */
#define LPUART_STAT_TDRE              (1 << 23) /* Transmit Data Register Empty Flag */
#define LPUART_STAT_RAF               (1 << 24) /* Receiver Active Flag */
#define LPUART_STAT_LBKDE             (1 << 25) /* LIN Break Detection Enable */
#define LPUART_STAT_BRK13             (1 << 26) /* Break Character Generation Length */
#define LPUART_STAT_RWUID             (1 << 27) /* Receive Wake Up Idle Detect */
#define LPUART_STAT_RXINV             (1 << 28) /* Receive Data Inversion */
#define LPUART_STAT_MSBF              (1 << 29) /* MSB First */
#define LPUART_STAT_RXEDGIF           (1 << 30) /* LPUART_RX Pin Active Edge Interrupt Flag */
#define LPUART_STAT_LBKDIF            (1 << 31) /* LIN Break Detect Interrupt Flag */

/* Low Power UART Control Register */

#define LPUART_CTRL_PT                (1 << 0)   /* Bit 0:  Parity Type */
#define LPUART_CTRL_PE                (1 << 1)   /* Bit 1:  Parity Enable */
#define LPUART_CTRL_ILT               (1 << 2)   /* Bit 2:  Idle Line Type Select */
#define LPUART_CTRL_WAKE              (1 << 3)   /* Bit 3:  Receiver Wakeup Method Select */
#define LPUART_CTRL_M                 (1 << 4)   /* Bit 4:  9-Bit or 8-Bit Mode Select */
#define LPUART_CTRL_RSRC              (1 << 5)   /* Bit 5:  Receiver Source Select */
#define LPUART_CTRL_DOZEEN            (1 << 6)   /* Bit 6:  Doze Enable */
#define LPUART_CTRL_LOOPS             (1 << 7)   /* Bit 7:  Loop Mode Select */
#define LPUART_CTRL_IDLECFG_SHIFT     (8)        /* Bits 8-10:  Idle Configuration */
#define LPUART_CTRL_IDLECFG_MASK      (3 << LPUART_CTRL_IDLECFG_SHIFT)
#  define LPUART_CTRL_IDLECFG_1       (0 << LPUART_CTRL_IDLECFG_SHIFT) /* 1   idle character */
#  define LPUART_CTRL_IDLECFG_2       (1 << LPUART_CTRL_IDLECFG_SHIFT) /* 2   idle characters */
#  define LPUART_CTRL_IDLECFG_4       (2 << LPUART_CTRL_IDLECFG_SHIFT) /* 4   idle characters */
#  define LPUART_CTRL_IDLECFG_8       (3 << LPUART_CTRL_IDLECFG_SHIFT) /* 8   idle characters */
#  define LPUART_CTRL_IDLECFG_16      (4 << LPUART_CTRL_IDLECFG_SHIFT) /* 16  idle characters */
#  define LPUART_CTRL_IDLECFG_32      (5 << LPUART_CTRL_IDLECFG_SHIFT) /* 32  idle characters */
#  define LPUART_CTRL_IDLECFG_64      (6 << LPUART_CTRL_IDLECFG_SHIFT) /* 64  idle characters */
#  define LPUART_CTRL_IDLECFG_128     (7 << LPUART_CTRL_IDLECFG_SHIFT) /* 128 idle characters */
                                                 /* Bits 11-13:  Reserved */
#define LPUART_CTRL_MA2IE             (1 << 14)  /* Bit 14:  Match 2 Interrupt Enable */
#define LPUART_CTRL_MA1IE             (1 << 15)  /* Bit 15:  Match 1 Interrupt Enable */
#define LPUART_CTRL_SBK               (1 << 16)  /* Bit 16:  Send Break */
#define LPUART_CTRL_RWU               (1 << 17)  /* Bit 17:  Receiver Wakeup Control */
#define LPUART_CTRL_RE                (1 << 18)  /* Bit 18:  Receiver Enable */
#define LPUART_CTRL_TE                (1 << 19)  /* Bit 19:  Transmitter Enable */
#define LPUART_CTRL_ILIE              (1 << 20)  /* Bit 20:  Idle Line Interrupt Enable */
#define LPUART_CTRL_RIE               (1 << 21)  /* Bit 21:  Receiver Interrupt Enable */
#define LPUART_CTRL_TCIE              (1 << 22)  /* Bit 22:  Transmission Complete Interrupt Enable for */
#define LPUART_CTRL_TIE               (1 << 23)  /* Bit 23:  Transmit Interrupt Enable */
#define LPUART_CTRL_PEIE              (1 << 24)  /* Bit 24:  Parity Error Interrupt Enable */
#define LPUART_CTRL_FEIE              (1 << 25)  /* Bit 25:  Framing Error Interrupt Enable */
#define LPUART_CTRL_NEIE              (1 << 26)  /* Bit 26:  Noise Error Interrupt Enable */
#define LPUART_CTRL_ORIE              (1 << 27)  /* Bit 27:  Overrun Interrupt Enable */
#define LPUART_CTRL_TXINV             (1 << 28)  /* Bit 28:  Transmit Data Inversion */
#define LPUART_CTRL_TXDIR             (1 << 29)  /* Bit 29:  LPUART_TX Pin Direction in Single-Wire Mode */
#define LPUART_CTRL_R9T8              (1 << 30)  /* Bit 30:  Receive Bit 9 / Transmit Bit 8 */
#define LPUART_CTRL_R8T9              (1 << 31)  /* Bit 31:  Receive Bit 8 / Transmit Bit 9 */

/* Low Power UART Data Register */

#define LPUART_DATA_SHIFT             (0)        /* Bits 0-9:  Read receive/ write transmit data */
#define LPUART_DATA_MASK              (0x3ff << LPUART_DATA_SHIFT)
#  define LPUART_DATA8(n)             (((n) & 0xff) << LPUART_DATA_SHIFT)
#  define LPUART_DATA9(n)             (((n) & 0x1ff) << LPUART_DATA_SHIFT)
#  define LPUART_DATA10(n)            (((n) & 0x3ff) << LPUART_DATA_SHIFT)
#  define LPUART_DATA_R0T0            (1 << 0)   /* Bit 0:  Read receive data buffer 0 or write transmit data buffer 0 */
#  define LPUART_DATA_R1T1            (1 << 1)   /* Bit 1:  Read receive data buffer 1 or write transmit data buffer 1 */
#  define LPUART_DATA_R2T2            (1 << 2)   /* Bit 2:  Read receive data buffer 2 or write transmit data buffer 2 */
#  define LPUART_DATA_R3T3            (1 << 3)   /* Bit 3:  Read receive data buffer 3 or write transmit data buffer 3 */
#  define LPUART_DATA_R4T4            (1 << 4)   /* Bit 4:  Read receive data buffer 4 or write transmit data buffer 4 */
#  define LPUART_DATA_R5T5            (1 << 5)   /* Bit 5:  Read receive data buffer 5 or write transmit data buffer 5 */
#  define LPUART_DATA_R6T6            (1 << 6)   /* Bit 6:  Read receive data buffer 6 or write transmit data buffer 6 */
#  define LPUART_DATA_R7T7            (1 << 7)   /* Bit 7:  Read receive data buffer 7 or write transmit data buffer 7 */
#  define LPUART_DATA_R8T8            (1 << 8)   /* Bit 8:  Read receive data buffer 8 or write transmit data buffer 8 */
#  define LPUART_DATA_R9T9            (1 << 9)   /* Bit 9:  Read receive data buffer 9 or write transmit data buffer 9 */
                                                 /* Bit 10:  Reserved */
#define LPUART_DATA_IDLINE            (1 << 11)  /* Bit 11:  Idle Line */
#define LPUART_DATA_RXEMPT            (1 << 12)  /* Bit 12:  Receive Buffer Empty */
#define LPUART_DATA_FRETSC            (1 << 13)  /* Bit 13:  Frame Error / Transmit Special Character */
#define LPUART_DATA_PARITYE           (1 << 14)  /* Bit 14:  The current received dataword contained in DATA[R9:R0] was received with a parity error */
#define LPUART_DATA_NOISY             (1 << 15)  /* Bit 15:  The current received dataword contained in DATA[R9:R0] was received with noise */
                                               /* Bits 16-31:  This field is reserved */

/* Low Power UART Match Address Register */

#define LPUART_MATCH_MA1_SHIFT        (0)        /* Bits 0-9:  Match Address 1 */
#define LPUART_MATCH_MA1_MASK         (0x3ff << LPUART_MATCH_MA1_SHIFT)
                                                 /* Bits 10-15:  Reserved */
#define LPUART_MATCH_MA2_SHIFT        (16)       /* Bits 16-25:  Match Address 2 */
#define LPUART_MATCH_MA2_MASK         (0x3ff << LPUART_MATCH_MA2_SHIFT)
                                                 /* Bits 26-31:  Reserved */

/* Low Power UART Modem IrDA Register */

#define LPUART_MODIR_TXCTSE           (1 << 0)   /* Bit 0:  Transmitter clear-to-send enable */
#define LPUART_MODIR_TXRTSE           (1 << 1)   /* Bit 1:  Transmitter request-to-send enable */
#define LPUART_MODIR_TXRTSPOL         (1 << 2)   /* Bit 2:  Transmitter request-to-send polarity */
#define LPUART_MODIR_RXRTSE           (1 << 3)   /* Bit 3:  Receiver request-to-send enable */
#define LPUART_MODIR_TXCTSC           (1 << 4)   /* Bit 4:  Transmit CTS Configuration */
#define LPUART_MODIR_TXCTSSRC         (1 << 5)   /* Bit 5:  Transmit CTS Source */
                                                 /* Bits 6-7: Reserved */
#if defined(KINETIS_LPUART_HAS_MODIR_RTSWATER)
#  define LPUART_MODIR_RTSWATER_SHIFT (8)        /* Bits 8-15: Receive RTS Configuration */
#  define LPUART_MODIR_RTSWATER_MASK  (0xff << LPUART_MODIR_RTSWATER_SHIFT)
#    define LPUART_MODIR_RTSWATER(n)  ((uint32_t)(n) << LPUART_MODIR_TNP_SHIFT)
#endif
#define LPUART_MODIR_TNP_SHIFT        (16)       /* Bits 16-17:  Transmitter narrow pulse */
#define LPUART_MODIR_TNP_MASK         (3 << LPUART_MODIR_TNP_SHIFT)
#  define LPUART_MODIR_TNP(n)         ((uint32_t)((n)-1) << LPUART_MODIR_TNP_SHIFT) /* n=1-4 */
#define LPUART_MODIR_IREN             (1 << 18)  /* Bit 18:  Infrared enable */
                                                 /* Bits 19-31:  Reserved */

/* Low Power UART FIFO Register */

#if defined(KINETIS_LPUART_HAS_FIFO)
#  define LPUART_FIFO_RXFIFOSIZE_SHIFT    (0)      /* Bits 0-2: Transmit FIFO buffer depth */
#  define LPUART_FIFO_RXFIFOSIZE_MASK     (7 << LPUART_FIFO_RXFIFOSIZE_SHIFT)
#    define LPUART_FIFO_RXFIFOSIZE_1WD    (0 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 1 dataword */
#    define LPUART_FIFO_RXFIFOSIZE_4WDS   (1 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 4 datawords */
#    define LPUART_FIFO_RXFIFOSIZE_8WDS   (2 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 8 datawords */
#    define LPUART_FIFO_RXFIFOSIZE_16WDS  (3 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 16 datawords */
#    define LPUART_FIFO_RXFIFOSIZE_32WDS  (4 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 32 datawords */
#    define LPUART_FIFO_RXFIFOSIZE_64WDS  (5 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 64 datawords */
#    define LPUART_FIFO_RXFIFOSIZE_128WDS (6 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 128 datawords */
#    define LPUART_FIFO_RXFIFOSIZE_256WDS (7 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 256 datawords */
#  define LPUART_FIFO_RXFE            (1 << 3)   /* Bit 3:  Receive FIFO Enable */
#  define LPUART_FIFO_TXFIFOSIZE_SHIFT    (4)    /* Bits 4-6: Transmit FIFO buffer depth */
#  define LPUART_FIFO_TXFIFOSIZE_MASK     (7 << LPUART_FIFO_TXFIFOSIZE_SHIFT)
#    define LPUART_FIFO_TXFIFOSIZE_1WD    (0 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 1 dataword */
#    define LPUART_FIFO_TXFIFOSIZE_4WDS   (1 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 4 datawords */
#    define LPUART_FIFO_TXFIFOSIZE_8WDS   (2 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 8 datawords */
#    define LPUART_FIFO_TXFIFOSIZE_16WDS  (3 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 16 datawords */
#    define LPUART_FIFO_TXFIFOSIZE_32WDS  (4 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 32 datawords */
#    define LPUART_FIFO_TXFIFOSIZE_64WDS  (5 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 64 datawords */
#    define LPUART_FIFO_TXFIFOSIZE_128WDS (6 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 128 datawords */
#    define LPUART_FIFO_TXFIFOSIZE_256WDS (7 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* Transmit FIFO depth = 256 datawords */
#  define LPUART_FIFO_TXFE            (1 << 7)   /* Bit 7:  Transmit FIFO Enable */
#  define LPUART_FIFO_RXUFE           (1 << 8)   /* Bit 8:  Receive FIFO Underflow Interrupt Enable */
#  define LPUART_FIFO_TXOFE           (1 << 9)   /* Bit 9:  Transmit FIFO Overflow Interrupt Enable */
#  define LPUART_FIFO_RXIDEN_SHIFT    (10)       /* Bits 10-12: Receiver Idle Empty Enable */
#  define LPUART_FIFO_RXIDEN_MASK     (7 << LPUART_FIFO_RXIDEN_SHIFT)
#    define LPUART_FIFO_RXIDEN_DISAB  (0 << LPUART_FIFO_RXIDEN_SHIFT) /* Disable RDRF */
#    define LPUART_FIFO_RXIDEN_1CH    (1 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 1 character */
#    define LPUART_FIFO_RXIDEN_2CH    (2 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 2 characters */
#    define LPUART_FIFO_RXIDEN_4CH    (3 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 4 characters */
#    define LPUART_FIFO_RXIDEN_8CH    (4 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 8 characters */
#    define LPUART_FIFO_RXIDEN_16CH   (5 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 16 characters */
#    define LPUART_FIFO_RXIDEN_32CH   (6 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 32 characters */
#    define LPUART_FIFO_RXIDEN_64CH   (7 << LPUART_FIFO_RXIDEN_SHIFT) /* RDRF when receiver is idle for 64 characters */
                                                 /* Bit 13: Reserved */
#  define LPUART_FIFO_RXFLUSH         (1 << 14)  /* Bit 14: Receive FIFO/Buffer Flush */
#  define LPUART_FIFO_TXFLUSH         (1 << 15)  /* Bit 15: Transmit FIFO/Buffer Flush */
#  define LPUART_FIFO_RXUF            (1 << 16)  /* Bit 16: Receiver Buffer Underflow Flag */
#  define LPUART_FIFO_TXOF            (1 << 17)  /* Bit 17: Transmitter Buffer Overflow Flag */
                                                 /* Bits 18-21: Reserved */
#  define LPUART_FIFO_RXEMPT          (1 << 21)  /* Bit 22: Receive Buffer/FIFO Empty */
#  define LPUART_FIFO_TXEMPT          (1 << 23)  /* Bit 23: Transmit Buffer/FIFO Empty */
                                                 /* Bits 24-31: Reserved */
#endif

/* Low Power UART Watermark Register */

#if defined(KINETIS_LPUART_HAS_WATER)
#  define LPUART_WATER_TXWATER_SHIFT  (0)        /* Bits 0-7: Transmit Watermark */
#  define LPUART_WATER_TXWATER_MASK   (0xff << LPUART_WATER_TXWATER_SHIFT)
#    define LPUART_WATER_TXWATER(n)   ((uint32_t)(n) << LPUART_WATER_TXWATER_SHIFT)
#  define LPUART_WATER_TXCOUNT_SHIFT  (8)        /* Bits 8-15: Transmit Counter */
#  define LPUART_WATER_TXCOUNT_MASK   (0xff << LPUART_WATER_TXCOUNT_SHIFT)
#    define LPUART_WATER_TXCOUNT(n)   ((uint32_t)(n) << LPUART_WATER_TXCOUNT_SHIFT)
#  define LPUART_WATER_RXWATER_SHIFT  (16)       /* Bits 16-23: Receive Watermark */
#  define LPUART_WATER_RXWATER_MASK   (0xff << LPUART_WATER_RXWATER_SHIFT)
#    define LPUART_WATER_RXWATER(n)   ((uint32_t)(n) << LPUART_WATER_RXWATER_SHIFT)
#  define LPUART_WATER_RXCOUNT_SHIFT  (24)       /* Bits 24-31: Receive Counter */
#  define LPUART_WATER_RXCOUNT_MASK   (0xff << LPUART_WATER_RXCOUNT_SHIFT)
#    define LPUART_WATER_RXCOUNT(n)   ((uint32_t)(n) << LPUART_WATER_RXCOUNT_SHIFT)
#endif

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_LPUART_H */
