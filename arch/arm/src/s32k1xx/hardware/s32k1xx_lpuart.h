/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_lpuart.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_LPUART_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_LPUART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/s32k1xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define S32K1XX_LPUART_VERID_OFFSET      0x0000  /* Version ID Register */
#define S32K1XX_LPUART_PARAM_OFFSET      0x0004  /* Parameter Register */
#define S32K1XX_LPUART_GLOBAL_OFFSET     0x0008  /* LPUART Global Register */
#define S32K1XX_LPUART_PINCFG_OFFSET     0x000c  /* LPUART Pin Configuration Register */
#define S32K1XX_LPUART_BAUD_OFFSET       0x0010  /* LPUART Baud Rate Register */
#define S32K1XX_LPUART_STAT_OFFSET       0x0014  /* LPUART Status Register */
#define S32K1XX_LPUART_CTRL_OFFSET       0x0018  /* LPUART Control Register */
#define S32K1XX_LPUART_DATA_OFFSET       0x001c  /* LPUART Data Register */
#define S32K1XX_LPUART_MATCH_OFFSET      0x0020  /* LPUART Match Address Register */
#define S32K1XX_LPUART_MODIR_OFFSET      0x0024  /* LPUART Modem IrDA Register */
#define S32K1XX_LPUART_FIFO_OFFSET       0x0028  /* LPUART FIFO Register */
#define S32K1XX_LPUART_WATER_OFFSET      0x002c  /* LPUART Watermark Register */

/* Register addresses *******************************************************/

#define S32K1XX_LPUART0_VERID            (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_VERID_OFFSET)
#define S32K1XX_LPUART0_PARAM            (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_PARAM_OFFSET)
#define S32K1XX_LPUART0_GLOBAL           (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_GLOBAL_OFFSET)
#define S32K1XX_LPUART0_PINCFG           (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_PINCFG_OFFSET)
#define S32K1XX_LPUART0_BAUD             (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_BAUD_OFFSET)
#define S32K1XX_LPUART0_STAT             (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_STAT_OFFSET)
#define S32K1XX_LPUART0_CTRL             (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_CTRL_OFFSET)
#define S32K1XX_LPUART0_DATA             (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_DATA_OFFSET)
#define S32K1XX_LPUART0_MATCH            (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_MATCH_OFFSET)
#define S32K1XX_LPUART0_MODIR            (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_MODIR_OFFSET)
#define S32K1XX_LPUART0_FIFO             (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_FIFO_OFFSET)
#define S32K1XX_LPUART0_WATER            (S32K1XX_LPUART0_BASE + S32K1XX_LPUART_WATER_OFFSET)

#define S32K1XX_LPUART1_VERID            (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_VERID_OFFSET)
#define S32K1XX_LPUART1_PARAM            (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_PARAM_OFFSET)
#define S32K1XX_LPUART1_GLOBAL           (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_GLOBAL_OFFSET)
#define S32K1XX_LPUART1_PINCFG           (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_PINCFG_OFFSET)
#define S32K1XX_LPUART1_BAUD             (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_BAUD_OFFSET)
#define S32K1XX_LPUART1_STAT             (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_STAT_OFFSET)
#define S32K1XX_LPUART1_CTRL             (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_CTRL_OFFSET)
#define S32K1XX_LPUART1_DATA             (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_DATA_OFFSET)
#define S32K1XX_LPUART1_MATCH            (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_MATCH_OFFSET)
#define S32K1XX_LPUART1_MODIR            (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_MODIR_OFFSET)
#define S32K1XX_LPUART1_FIFO             (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_FIFO_OFFSET)
#define S32K1XX_LPUART1_WATER            (S32K1XX_LPUART1_BASE + S32K1XX_LPUART_WATER_OFFSET)

#define S32K1XX_LPUART2_VERID            (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_VERID_OFFSET)
#define S32K1XX_LPUART2_PARAM            (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_PARAM_OFFSET)
#define S32K1XX_LPUART2_GLOBAL           (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_GLOBAL_OFFSET)
#define S32K1XX_LPUART2_PINCFG           (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_PINCFG_OFFSET)
#define S32K1XX_LPUART2_BAUD             (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_BAUD_OFFSET)
#define S32K1XX_LPUART2_STAT             (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_STAT_OFFSET)
#define S32K1XX_LPUART2_CTRL             (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_CTRL_OFFSET)
#define S32K1XX_LPUART2_DATA             (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_DATA_OFFSET)
#define S32K1XX_LPUART2_MATCH            (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_MATCH_OFFSET)
#define S32K1XX_LPUART2_MODIR            (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_MODIR_OFFSET)
#define S32K1XX_LPUART2_FIFO             (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_FIFO_OFFSET)
#define S32K1XX_LPUART2_WATER            (S32K1XX_LPUART2_BASE + S32K1XX_LPUART_WATER_OFFSET)

/* Register bit definitions *************************************************/

/* Version ID Register */

#define LPUART_VERID_FEATURE_SHIFT     (0)        /* Bits 0-15: Feature Identification Number */
#define LPUART_VERID_FEATURE_MASK      (0xffff << LPUART_VERID_FEATURE_SHIFT)
#  define LPUART_VERID_FEATURE_STD     (1 << LPUART_VERID_FEATURE_SHIFT) /* Standard feature set */
#  define LPUART_VERID_FEATURE_MODEM   (3 << LPUART_VERID_FEATURE_SHIFT) /* MODEM/IrDA support */

#define LPUART_VERID_MINOR_SHIFT       (16)       /* Bits 16-23: Minor Version Number */
#define LPUART_VERID_MINOR_MASK        (0xff << LPUART_VERID_MINOR_SHIFT)
#define LPUART_VERID_MAJOR_SHIFT       (24)       /* Bits 24-31: Major Version Number */
#define LPUART_VERID_MAJOR_MASK        (0xff << LPUART_VERID_MAJOR_SHIFT)

/* Parameter Register */

#define LPUART_PARAM_TXFIFO_SHIFT      (0)       /* Bits 0-7: Transmit FIFO Size */
#define LPUART_PARAM_TXFIFO_MASK       (0xff << LPUART_PARAM_TXFIFO_SHIFT)
#define LPUART_PARAM_RXFIFO_SHIFT      (8)       /* Bits 8-15: Transmit FIFO Size */
#define LPUART_PARAM_RXFIFO_MASK       (0xff << LPUART_PARAM_RXFIFO_SHIFT)
                                                 /* Bits 16-31: Reserved */

/* LPUART Global Register */

                                                 /* Bit 0:  Reserved */
#define LPUART_GLOBAL_RST              (1 << 1)  /* Bit 1: Software Reset */
                                                 /* Bits 2-31:  Reserved */

/* LPUART Pin Configuration Register */

#define LPUART_PINCFG_TRGSEL_SHIFT     (0)       /* Bits 0-1:  Trigger Select */
#define LPUART_PINCFG_TRGSEL_MASK      (3 << LPUART_PINCFG_TRGSEL_SHIFT)
#  define LPUART_PINCFG_TRGSEL_DISABLE (0 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger disabled */
#  define LPUART_PINCFG_TRGSEL_RXD     (1 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger used instead of RXD pin */
#  define LPUART_PINCFG_TRGSEL_CTSB    (2 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger used instead of CTS_B pin */
#  define LPUART_PINCFG_TRGSEL_TXDMOD  (3 << LPUART_PINCFG_TRGSEL_SHIFT) /* Trigger used to modulate the TXD output */

                                                 /* Bits 2-31:  Reserved */

/* LPUART Baud Rate Register */

#define LPUART_BAUD_SBR_SHIFT          (0)       /* Bits 0-12: Baud Rate Modulo Divisor. */
#define LPUART_BAUD_SBR_MASK           (0x1fff << LPUART_BAUD_SBR_SHIFT)
#  define LPUART_BAUD_SBR(n)           ((uint32_t)(n) << LPUART_BAUD_SBR_SHIFT)
#define LPUART_BAUD_SBNS               (1 << 13) /* Bit 13: Stop Bit Number Select */
#define LPUART_BAUD_RXEDGIE            (1 << 14) /* Bit 14: RX Input Active Edge Interrupt Enable */
#define LPUART_BAUD_LBKDIE             (1 << 15) /* Bit 15: LIN Break Detect Interrupt Enable */
#define LPUART_BAUD_RESYNCDIS          (1 << 16) /* Bit 16: Resynchronization Disable */
#define LPUART_BAUD_BOTHEDGE           (1 << 17) /* Bit 17: Both Edge Sampling */
#define LPUART_BAUD_MATCFG_SHIFT       (18)      /* Bits 18-19: Match Configuration */
#define LPUART_BAUD_MATCFG_MASK        (3 << LPUART_BAUD_MATCFG_SHIFT)
#  define LPUART_BAUD_MATCFG_ADDR      (0 << LPUART_BAUD_MATCFG_SHIFT) /* Address Match Wakeup */
#  define LPUART_BAUD_MATCFG_IDLE      (1 << LPUART_BAUD_MATCFG_SHIFT) /* Idle Match Wakeup */
#  define LPUART_BAUD_MATCFG_ONOFF     (2 << LPUART_BAUD_MATCFG_SHIFT) /* Match On and Match Off */
#  define LPUART_BAUD_MATCFG_RWUENAB   (3 << LPUART_BAUD_MATCFG_SHIFT) /* Enables RWU on Data Match and Match
                                                                        * On/Off for transmitter CTS input */

                                                 /* Bit 20: Reserved */
#define LPUART_BAUD_RDMAE              (1 << 21) /* Bit 21: Receiver Full DMA Enable */
                                                 /* Bit 22: Reserved */
#define LPUART_BAUD_TDMAE              (1 << 23) /* Bit 23: Transmitter DMA Enable */
#define LPUART_BAUD_OSR_SHIFT          (24)      /* Bits 24-28: Oversampling Ratio */
#define LPUART_BAUD_OSR_MASK           (15 << LPUART_BAUD_OSR_SHIFT)
#  define LPUART_BAUD_OSR(n)           ((uint32_t)((n) - 1) << LPUART_BAUD_OSR_SHIFT) /* n=4..32 */

#define LPUART_BAUD_M10                (1 << 29) /* Bit 20: 10-bit Mode select */
#define LPUART_BAUD_MAEN2              (1 << 30) /* Bit 30: Match Address Mode Enable 2 */
#define LPUART_BAUD_MAEN1              (1 << 31) /* Bit 31: Match Address Mode Enable 1 */

/* LPUART Status Register */

                                                 /* Bits 0-13:  Reserved */
#define LPUART_STAT_MA2F               (1 << 14) /* Bit 14: Match 2 Flag */
#define LPUART_STAT_MA1F               (1 << 15) /* Bit 15: Match 1 Flag */
#define LPUART_STAT_PF                 (1 << 16) /* Bit 16: Parity Error Flag */
#define LPUART_STAT_FE                 (1 << 17) /* Bit 17: Framing Error Flag */
#define LPUART_STAT_NF                 (1 << 18) /* Bit 18: Noise Flag */
#define LPUART_STAT_OR                 (1 << 19) /* Bit 19: Receiver Overrun Flag */
#define LPUART_STAT_IDLE               (1 << 20) /* Bit 20: Idle Line Flag */
#define LPUART_STAT_RDRF               (1 << 21) /* Bit 21: Receive Data Register Full Flag */
#define LPUART_STAT_TC                 (1 << 22) /* Bit 22: Transmission Complete Flag */
#define LPUART_STAT_TDRE               (1 << 23) /* Bit 23: Transmit Data Register Empty Flag */
#define LPUART_STAT_RAF                (1 << 24) /* Bit 24: Receiver Active Flag */
#define LPUART_STAT_LBKDE              (1 << 25) /* Bit 25: LIN Break Detection Enable */
#define LPUART_STAT_BRK13              (1 << 26) /* Bit 26: Break Character Generation Length */
#define LPUART_STAT_RWUID              (1 << 27) /* Bit 27: Receive Wake Up Idle Detect */
#define LPUART_STAT_RXINV              (1 << 28) /* Bit 28: Receive Data Inversion */
#define LPUART_STAT_MSBF               (1 << 29) /* Bit 29: MSB First */
#define LPUART_STAT_RXEDGIF            (1 << 30) /* Bit 30: RXD Pin Active Edge Interrupt Flag */
#define LPUART_STAT_LBKDIF             (1 << 31) /* Bit 31: LIN Break Detect Interrupt Flag */

/* LPUART Control Register */

#define LPUART_CTRL_PT                 (1 << 0)  /* Bit 0:  Parity Type */
#  define LPUART_CTRL_PT_EVEN          (0 << 0)  /*         Even parity */
#  define LPUART_CTRL_PT_ODD           (1 << 0)  /*         Odd parity */
#define LPUART_CTRL_PE                 (1 << 1)  /* Bit 1:  Parity Enable */
#define LPUART_CTRL_ILT                (1 << 2)  /* Bit 2:  Idle Line Type Select */
#define LPUART_CTRL_WAKE               (1 << 3)  /* Bit 3:  Receiver Wakeup Method Select */
#define LPUART_CTRL_M                  (1 << 4)  /* Bit 4:  9-Bit or 8-Bit Mode Select */
#define LPUART_CTRL_RSRC               (1 << 5)  /* Bit 5:  Receiver Source Select */
#define LPUART_CTRL_DOZEEN             (1 << 6)  /* Bit 6:  Doze Enable */
#define LPUART_CTRL_LOOPS              (1 << 7)  /* Bit 7:  Loop Mode Select */
#define LPUART_CTRL_IDLECFG_SHIFT      (8)       /* Bits 8-10: Idle Configuration */
#define LPUART_CTRL_IDLECFG_MASK       (7 << LPUART_CTRL_IDLECFG_SHIFT)
#  define LPUART_CTRL_IDLECFG_1        (0 << LPUART_CTRL_IDLECFG_SHIFT) /* 1 idle character */
#  define LPUART_CTRL_IDLECFG_2        (1 << LPUART_CTRL_IDLECFG_SHIFT) /* 2 idle characters */
#  define LPUART_CTRL_IDLECFG_4        (2 << LPUART_CTRL_IDLECFG_SHIFT) /* 4 idle characters */
#  define LPUART_CTRL_IDLECFG_8        (3 << LPUART_CTRL_IDLECFG_SHIFT) /* 8 idle characters */
#  define LPUART_CTRL_IDLECFG_16       (4 << LPUART_CTRL_IDLECFG_SHIFT) /* 6 idle characters */
#  define LPUART_CTRL_IDLECFG_32       (5 << LPUART_CTRL_IDLECFG_SHIFT) /* 32 idle characters */
#  define LPUART_CTRL_IDLECFG_64       (6 << LPUART_CTRL_IDLECFG_SHIFT) /* 64 idle characters */
#  define LPUART_CTRL_IDLECFG_128      (7 << LPUART_CTRL_IDLECFG_SHIFT) /* 128 idle characters */

#define LPUART_CTRL_M7                 (1 << 11) /* Bit 11: 7-Bit Mode Select */
                                                 /* Bits 12-13:  Reserved */
#define LPUART_CTRL_MA2IE              (1 << 14) /* Bit 14: Match 2 Interrupt Enable */
#define LPUART_CTRL_MA1IE              (1 << 15) /* Bit 15: Match 1 Interrupt Enable */
#define LPUART_CTRL_SBK                (1 << 16) /* Bit 16: Send Break */
#define LPUART_CTRL_RWU                (1 << 17) /* Bit 17: Receiver Wakeup Control */
#define LPUART_CTRL_RE                 (1 << 18) /* Bit 18: Receiver Enable */
#define LPUART_CTRL_TE                 (1 << 19) /* Bit 19: Transmitter Enable */
#define LPUART_CTRL_ILIE               (1 << 20) /* Bit 20: Idle Line Interrupt Enable */
#define LPUART_CTRL_RIE                (1 << 21) /* Bit 21: Receiver Interrupt Enable */
#define LPUART_CTRL_TCIE               (1 << 22) /* Bit 22: Transmission Complete Interrupt Enable */
#define LPUART_CTRL_TIE                (1 << 23) /* Bit 23: Transmit Interrupt Enable */
#define LPUART_CTRL_PEIE               (1 << 24) /* Bit 24: Parity Error Interrupt Enable */
#define LPUART_CTRL_FEIE               (1 << 25) /* Bit 25: Framing Error Interrupt Enable */
#define LPUART_CTRL_NEIE               (1 << 26) /* Bit 26: Noise Error Interrupt Enable */
#define LPUART_CTRL_ORIE               (1 << 27) /* Bit 27: Overrun Interrupt Enable */
#define LPUART_CTRL_TXINV              (1 << 28) /* Bit 28: Transmit Data Inversion */
#define LPUART_CTRL_TXDIR              (1 << 29) /* Bit 29: TXD Pin Direction in Single-Wire Mode */
#define LPUART_CTRL_R9T8               (1 << 30) /* Bit 30: Receive Bit 9 / Transmit Bit 8 */
#define LPUART_CTRL_R8T9               (1 << 31) /* Bit 31: Receive Bit 8 / Transmit Bit 9 */

#define LPUART_ALL_INTS (LPUART_CTRL_ORIE | LPUART_CTRL_NEIE | LPUART_CTRL_FEIE |  \
                         LPUART_CTRL_PEIE | LPUART_CTRL_TIE  | LPUART_CTRL_TCIE |  \
                         LPUART_CTRL_RIE  | LPUART_CTRL_ILIE | LPUART_CTRL_MA1IE | \
                         LPUART_CTRL_MA2IE)

/* LPUART Data Register */

#define LPUART_DATA_SHIFT              (0)       /* Bits 0-9: Data bits 0-9 */
#define LPUART_DATA_MASK               (0x3ff << LPUART_DATA_SHIFT)
                                                 /* Bit 10:  Reserved */
#define LPUART_DATA_STATUS_SHIFT       (11)      /* Bit 11: Idle Line status */
#define LPUART_DATA_IDLINE             (1 << 11) /* Bit 11: Idle Line */
#define LPUART_DATA_RXEMPT             (1 << 12) /* Bit 12: Receive Buffer Empty */
#define LPUART_DATA_FRETSC             (1 << 13) /* Bit 13: Frame Error / Transmit Special Character */
#define LPUART_DATA_PARITYE            (1 << 14) /* Bit 14: Parity Error */
#define LPUART_DATA_NOISY              (1 << 15) /* Bit 15: Noisy */
                                                 /* Bits 16-31:  Reserved */

/* LPUART Match Address Register */

#define LPUART_MATCH_MA1_SHIFT         (0)       /* Bits 0-9: Match Address 1 */
#define LPUART_MATCH_MA1_MASK          (0x3ff << LPUART_MATCH_MA1_SHIFT)
#  define LPUART_MATCH_MA1(n)          ((uint32_t)(n) << LPUART_MATCH_MA1_SHIFT)
                                                 /* Bits 10-15:  Reserved */
#define LPUART_MATCH_MA2_SHIFT         (16)      /* Bits 16-25: Match Address 2 */
#define LPUART_MATCH_MA2_MASK          (0x3ff << LPUART_MATCH_MA2_SHIFT)
#  define LPUART_MATCH_MA2(n)          ((uint32_t)(n) << LPUART_MATCH_MA2_SHIFT)
                                                 /* Bits 26-31:  Reserved */

/* LPUART Modem IrDA Register */

#define LPUART_MODIR_TXCTSE            (1 << 0)  /* Bit nn:  Transmitter clear-to-send enable */
#define LPUART_MODIR_TXRTSE            (1 << 1)  /* Bit nn:  Transmitter request-to-send enable */
#define LPUART_MODIR_TXRTSPOL          (1 << 2)  /* Bit nn:  Transmitter request-to-send polarity */
#define LPUART_MODIR_RXRTSE            (1 << 3)  /* Bit nn:  Receiver request-to-send enable */
#define LPUART_MODIR_TXCTSC            (1 << 4)  /* Bit nn:  Transmit CTS Configuration */
#  define LPUART_MODIR_TXCTSC_START    (0 << 4)  /* CTS sampled at start of character */
#  define LPUART_MODIR_TXCTSC_IDLE     (1 << 4)  /* CTS sampled when transmitter idle */
#define LPUART_MODIR_TXCTSSRC          (1 << 5)  /* Bit nn:  Transmit CTS Source */
#  define LPUART_MODIR_TXCTSSRC_CTSB   (0 << 5)  /* Bit nn:  CTS input is CTS_B pin */
#  define LPUART_MODIR_TXCTSSRC_RXMAT  (1 << 5)  /* Bit nn:  Transmit CTS Source */
                                                 /* Bits 6-7:  Reserved */
#define LPUART_MODIR_RTSWATER_SHIFT    (8)       /* Bits 8-9: Receive RTS Configuration */
#define LPUART_MODIR_RTSWATER_MASK     (3 << LPUART_MODIR_RTSWATER_SHIFT)
#  define LPUART_MODIR_RTSWATER(n)     ((uint32_t)(n) << LPUART_MODIR_RTSWATER_SHIFT)
                                                 /* Bits 10-15:  Reserved */
#define LPUART_MODIR_TNP_SHIFT         (16)      /* Bits 16-17: Transmitter narrow pulse */
#define LPUART_MODIR_TNP_MASK          (3 << LPUART_MODIR_TNP_SHIFT)
#  define LPUART_MODIR_TNP(n)          ((uint32_t)((n) - 1) << LPUART_MODIR_TNP_SHIFT) /* n/OSR */

#define LPUART_MODIR_IREN              (1 << 18) /* Bit nn: Infrared enable */
                                                 /* Bits 19-31:  Reserved */

/* LPUART FIFO Register */

#define LPUART_FIFO_RXFIFOSIZE_SHIFT   (0)       /* Bits 0-2: Receive FIFO. Buffer Depth */
#define LPUART_FIFO_RXFIFOSIZE_MASK    (7 << LPUART_FIFO_RXFIFOSIZE_SHIFT)
#  define LPUART_FIFO_RXFIFOSIZE_1     (0 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 1 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_4     (1 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 4 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_8     (2 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 8 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_16    (3 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 16 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_32    (4 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 32 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_64    (5 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 64 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_128   (6 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 128 datawords */
#  define LPUART_FIFO_RXFIFOSIZE_256   (7 << LPUART_FIFO_RXFIFOSIZE_SHIFT) /* 256 datawords */

#define LPUART_FIFO_RXFE               (1 << 3)  /* Bit 3:  Receive FIFO Enable */
#define LPUART_FIFO_TXFIFOSIZE_SHIFT   (4)       /* Bits 4-6: Transmit FIFO. Buffer Depth */
#define LPUART_FIFO_TXFIFOSIZE_MASK    (7 << LPUART_FIFO_TXFIFOSIZE_SHIFT)
#  define LPUART_FIFO_TXFIFOSIZE_1     (0 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 1 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_4     (1 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 4 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_8     (2 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 8 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_16    (3 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 16 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_32    (4 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 32 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_64    (5 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 64 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_128   (6 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 128 datawords */
#  define LPUART_FIFO_TXFIFOSIZE_256   (7 << LPUART_FIFO_TXFIFOSIZE_SHIFT) /* 256 datawords */

#define LPUART_FIFO_TXFE               (1 << 7)  /* Bit 7:  Transmit FIFO Enable */
#define LPUART_FIFO_RXUFE              (1 << 8)  /* Bit 8:  Receive FIFO Underflow Interrupt Enable */
#define LPUART_FIFO_TXOFE              (1 << 9)  /* Bit 9:  Transmit FIFO Overflow Interrupt Enable */
#define LPUART_FIFO_RXIDEN_SHIFT       (10)      /* Bits 10-12:  Receiver Idle Empty Enable */
#define LPUART_FIFO_RXIDEN_MASK        (7 << LPUART_FIFO_RXIDEN_SHIFT)
#  define LPUART_FIFO_RXIDEN_DISABLE   (0 << LPUART_FIFO_RXIDEN_SHIFT) /* Disable RDRF assertion when receiver is idle */
#  define LPUART_FIFO_RXIDEN_1         (1 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 1 word */
#  define LPUART_FIFO_RXIDEN_2         (2 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 2 words */
#  define LPUART_FIFO_RXIDEN_4         (3 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 4 words */
#  define LPUART_FIFO_RXIDEN_8         (4 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 8 words */
#  define LPUART_FIFO_RXIDEN_16        (5 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 16 words */
#  define LPUART_FIFO_RXIDEN_32        (6 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 32 words */
#  define LPUART_FIFO_RXIDEN_64        (7 << LPUART_FIFO_RXIDEN_SHIFT) /* Enable RDRF assertion when receiver idle for 64 words */

#define LPUART_FIFO_RXFLUSH            (1 << 14) /* Bit 14: Receive FIFO/Buffer Flush */
#define LPUART_FIFO_TXFLUSH            (1 << 15) /* Bit 15: Transmit FIFO/Buffer Flush */
#define LPUART_FIFO_RXUF               (1 << 16) /* Bit 16: Receiver Buffer Underflow Flag */
#define LPUART_FIFO_TXOF               (1 << 17) /* Bit 17: Transmitter Buffer Overflow Flag */
                                                 /* Bits 18-21:  Reserved */
#define LPUART_FIFO_RXEMPT             (1 << 22) /* Bit 22: Receive Buffer/FIFO Empty */
#define LPUART_FIFO_TXEMPT             (1 << 23) /* Bit 23: Transmit Buffer/FIFO Empty */
                                                 /* Bits 24-31:  Reserved */

/* LPUART Watermark Register */

#define LPUART_WATER_TXWATER_SHIFT     (0)       /* Bits 0-1: Transmit Watermark */
#define LPUART_WATER_TXWATER_MASK      (3 << LPUART_WATER_TXWATER_SHIFT)
#  define LPUART_WATER_TXWATER(n)      ((uint32_t)(n) << LPUART_WATER_TXWATER_SHIFT)
                                                 /* Bits 2-7:  Reserved */
#define LPUART_WATER_TXCOUNT_SHIFT     (8)       /* Bits 8-10:Transmit Counter */
#define LPUART_WATER_TXCOUNT_MASK      (7 << LPUART_WATER_TXCOUNT_SHIFT)
#  define LPUART_WATER_TXCOUNT(n)      ((uint32_t)(n) << LPUART_WATER_TXCOUNT_SHIFT)
                                                 /* Bits 11-15:  Reserved */
#define LPUART_WATER_RXWATER_SHIFT     (16)      /* Bits 16-17: Receive Watermark */
#define LPUART_WATER_RXWATER_MASK      (3 << LPUART_WATER_RXWATER_SHIFT)
#  define LPUART_WATER_RXWATER(n)      ((uint32_t)(n) << LPUART_WATER_RXWATER_SHIFT)
                                                 /* Bits 18-23:  Reserved */
#define LPUART_WATER_RXCOUNT_SHIFT     (24)      /* Bits 24-26: Receive Counter */
#define LPUART_WATER_RXCOUNT_MASK      (7 << LPUART_WATER_RXCOUNT_SHIFT)
#  define LPUART_WATER_RXCOUNT(n)      ((uint32_t)(n) << LPUART_WATER_RXCOUNT_SHIFT)
                                                 /* Bits 27-31:  Reserved */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_LPUART_H */
