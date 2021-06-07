/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_lpuart.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_LPUART_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_LPUART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_LPUART_VERID_OFFSET      0x0000 /* Version ID */
#define RV32M1_LPUART_PARAM_OFFSET      0x0004 /* Parameter */
#define RV32M1_LPUART_GLOBAL_OFFSET     0x0008 /* Global */
#define RV32M1_LPUART_PINCFG_OFFSET     0x000c /* Pin Configuration */
#define RV32M1_LPUART_BAUD_OFFSET       0x0010 /* Baud Rate */
#define RV32M1_LPUART_STAT_OFFSET       0x0014 /* Status */
#define RV32M1_LPUART_CTRL_OFFSET       0x0018 /* Control */
#define RV32M1_LPUART_DATA_OFFSET       0x001c /* Data */
#define RV32M1_LPUART_MATCH_OFFSET      0x0020 /* Match Address */
#define RV32M1_LPUART_MODIR_OFFSET      0x0024 /* Modem IrDA */
#define RV32M1_LPUART_FIFO_OFFSET       0x0028 /* FIFO */
#define RV32M1_LPUART_WATER_OFFSET      0x002c /* Watermark */

/* Register Addresses *******************************************************/

/* Register Bitfield Definitions ********************************************/

#define LPUART_PARAM_RXFIFO_SHIFT       (8)
#define LPUART_PARAM_RXFIFO_MASK        (0xff << LPUART_PARAM_RXFIFO_SHIFT)

#define LPUART_PARAM_TXFIFO_SHIFT       (8)
#define LPUART_PARAM_TXFIFO_MASK        (0xff << LPUART_PARAM_TXFIFO_SHIFT)

#define LPUART_GLOBAL_RST               (1 << 1)

#define LPUART_PINCFG_TRGSEL_SHIFT      (0)
#define LPUART_PINCFG_TRGSEL_MASK       (3 << LPUART_PINCFG_TRGSEL_SHIFT)
#define LPUART_PINCFG_TRGSEL_DISABLED   (0 << LPUART_PINCFG_TRGSEL_SHIFT)
#define LPUART_PINCFG_TRGSEL_RXD        (1 << LPUART_PINCFG_TRGSEL_SHIFT)
#define LPUART_PINCFG_TRGSEL_CTS        (2 << LPUART_PINCFG_TRGSEL_SHIFT)
#define LPUART_PINCFG_TRGSEL_MTX        (3 << LPUART_PINCFG_TRGSEL_SHIFT)

#define LPUART_BAUD_MAEN1               (1 << 31) /* Bit31: Match Address Mode Enable 1 */
#define LPUART_BAUD_MAEN2               (1 << 30) /* Bit30: Match Address Mode Enable 2 */
#define LPUART_BAUD_M10                 (1 << 29) /* Bit29: 10-bit Mode select */

#define LPUART_BAUD_OSR_SHIFT           (24) /* Bit[28:24]: Oversampling Ratio */
#define LPUART_BAUD_OSR_MASK            (0x1f << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_DEFAULT         (0    << LPUART_BAUD_OSR_SHIFT) /* ratio of 16 */
#define LPUART_BAUD_OSR_1_RESERVED      (1    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_2_RESERVED      (2    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_4               (3    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_5               (4    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_6               (5    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_7               (6    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_8               (7    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_9               (8    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_10              (9    << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_11              (10   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_12              (11   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_13              (12   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_14              (13   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_15              (14   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_16              (15   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_17              (16   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_18              (17   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_19              (18   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_20              (19   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_21              (20   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_22              (21   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_23              (22   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_24              (23   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_25              (24   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_26              (25   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_27              (26   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_28              (27   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_29              (28   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_30              (29   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_31              (30   << LPUART_BAUD_OSR_SHIFT)
#define LPUART_BAUD_OSR_32              (31   << LPUART_BAUD_OSR_SHIFT)

#define LPUART_BAUD_TDMAE               (1 << 23) /* Bit23: Transmitter DMA Enable */
#define LPUART_BAUD_RDMAE               (1 << 21) /* Bit22: Receiver Full DMA Enable */
#define LPUART_BAUD_RIDMAE              (1 << 20) /* Bit20: Receiver Idle DMA Enable */
#define LPUART_BAUD_MATCFG_SHIFT        (18)      /* Bit[19:18]: Match Configuration */
#define LPUART_BAUD_MATCFG_MASK         (0x3 << LPUART_BAUD_MATCFG_SHIFT)
#define LPUART_BAUD_MATCFG_ADDR         (0   << LPUART_BAUD_MATCFG_SHIFT) /* Address Match Wakeup */
#define LPUART_BAUD_MATCFG_IDLE         (1   << LPUART_BAUD_MATCFG_SHIFT) /* Idle Match Wakeup */
#define LPUART_BAUD_MATCFG_ONOFF        (2   << LPUART_BAUD_MATCFG_SHIFT) /* Match On and Match Off */
#define LPUART_BAUD_MATCFG_RWU          (3   << LPUART_BAUD_MATCFG_SHIFT)

#define LPUART_BAUD_BOTHEDGE            (1 << 17) /* Bit17: Both Edge Sampling */
#define LPUART_BAUD_RESYNCDIS           (1 << 16) /* Bit16: Resynchronization Disable */
#define LPUART_BAUD_LBKDIE              (1 << 15) /* Bit15: LIN Break Detect Interrupt Enable */
#define LPUART_BAUD_RXEDGIE             (1 << 14) /* Bit14: RX Input Active Edge Interrupt Enable */
#define LPUART_BAUD_SBNS                (1 << 13) /* Bit13: Stop Bit Number Select */
#  define LPUART_BAUD_SBNS_MASK         (1 << 13)
#  define LPUART_BAUD_SBNS_1            (0 << 13)
#  define LPUART_BAUD_SBNS_2            (1 << 13)

#define LPUART_BAUD_SBR_SHIFT           (0) /* Bit[12:0]: Baud Rate Modulo Divisor */
#define LPUART_BAUD_SBR_MASK            (0x1fff << LPUART_BAUD_SBR_SHIFT)

#define LPUART_STAT_LBKDIF              (1 << 31) /* Bit31: LIN Break Detect Interrupt Flag */
#define LPUART_STAT_RXEDGIF             (1 << 30) /* Bit30: RXD Pin Active Edge Interrupt Flag */
#define LPUART_STAT_MSBF                (1 << 29) /* Bit29: MSB First */
#define LPUART_STAT_RXINV               (1 << 28) /* Bit28: Receive Data Inversion */
#define LPUART_STAT_RWUID               (1 << 27) /* Bit27: Receive Wake Up Idel Detect */
#define LPUART_STAT_BRK13               (1 << 26) /* Bit26: Break Character Generation Length */
#define LPUART_STAT_LBKDE               (1 << 25) /* Bit25: Lin Break Detection Enable */
#define LPUART_STAT_RAF                 (1 << 24) /* Bit24: Receiver Active Flag */
#define LPUART_STAT_TDRE                (1 << 23) /* Bit23: Transmit Data Register Empty Flag */
#define LPUART_STAT_TC                  (1 << 22) /* Bit22: Transmission Complete Flag */
#define LPUART_STAT_RDRF                (1 << 21) /* Bit21: Receive Data Register Full Flag */
#define LPUART_STAT_IDLE                (1 << 20) /* Bit20: Idle Line Flag */
#define LPUART_STAT_OR                  (1 << 19) /* Bit19: Receiver Overrun Flag */
#define LPUART_STAT_NF                  (1 << 18) /* Bit18: Noise Flag */
#define LPUART_STAT_FE                  (1 << 17) /* Bit17: Framing Error Flag */
#define LPUART_STAT_PF                  (1 << 16) /* Bit16: Parity Error Flag */
#define LPUART_STAT_MA1F                (1 << 15) /* Bit15: Match 1 Flag */
#define LPUART_STAT_MA2F                (1 << 14) /* Bit14: Match 2 Flag */

#define LPUART_CTRL_R8T9                (1 << 31) /* Bit31: Receive Bit 8/Transmit Bit 9 */
#define LPUART_CTRL_R9T8                (1 << 30) /* Bit30: Receive Bit 9/Transmit Bit 8 */
#define LPUART_CTRL_TXDIR               (1 << 29) /* Bit29: TXD Pin Direction In Single-Wire Mode */
#define LPUART_CTRL_TXINV               (1 << 28) /* Bit28: Transmit Data Inversion */
#define LPUART_CTRL_ORIE                (1 << 27) /* Bit27: Overrun Interrupt Enable */
#define LPUART_CTRL_NEIE                (1 << 26) /* Bit26: Noise Error Interrupt Enable */
#define LPUART_CTRL_FEIE                (1 << 25) /* Bit25: Framing Error Interrupt Enable */
#define LPUART_CTRL_PEIE                (1 << 24) /* Bit24: Parity Error Interrupt Enable */
#define LPUART_CTRL_TIE                 (1 << 23) /* Bit23: Enable Interrupt if TDRE is 1 */
#define LPUART_CTRL_TCIE                (1 << 22) /* Bit22: Enable Interrupt if TC is 1 */
#define LPUART_CTRL_RIE                 (1 << 21) /* Bit21: Receiver Interrupt Enable */
#define LPUART_CTRL_ILIE                (1 << 20) /* Bit20: Idle Line Interrupt Enable */
#define LPUART_CTRL_TE                  (1 << 19) /* Bit19: Transmitter Enable */
#define LPUART_CTRL_RE                  (1 << 18) /* Bit18: Receiver Enable */ 
#define LPUART_CTRL_RWU                 (1 << 17) /* Bit17: Receiver Wakeup Control */
#define LPUART_CTRL_SBK                 (1 << 16) /* Bit16: Send Break */
#define LPUART_CTRL_MA1IE               (1 << 15) /* Bit15: Match 1 Interrupt Enable */
#define LPUART_CTRL_MA21E               (1 << 14) /* Bit14: Match 2 Interrupt Enable */
#define LPUART_CTRL_M7                  (1 << 11) /* Bit11: 7-Bit Mode Select */

#define LPUART_CTRL_IDLECFG_SHIFT       (8) /* Bit[10:8]: Idle Configuration */
#define LPUART_CTRL_IDLECFG_MASK        (0x7 << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_1           (0   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_2           (1   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_4           (2   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_8           (3   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_16          (4   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_32          (5   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_64          (6   << LPUART_CTRL_IDLECFG_SHIFT)
#define LPUART_CTRL_IDLECFG_128         (7   << LPUART_CTRL_IDLECFG_SHIFT)

#define LPUART_CTRL_LOOPS               (1 << 7) /* Bit7: Loop Mode Select */
#define LPUART_CTRL_DOZEEN              (1 << 6) /* Bit6: Doze Enable */
#define LPUART_CTRL_RSRC                (1 << 5) /* Bit5: Receiver Source Select */
#define LPUART_CTRL_M                   (1 << 4) /* Bit4: 9-Bit or 8-Bit Mode Select */
#  define LPUART_CTRL_M_MASK            (1 << 4)
#  define LPUART_CTRL_M8                (0 << 4) /* 8-Bit Mode */
#  define LPUART_CTRL_M9                (1 << 4) /* 9-Bit Mode */

#define LPUART_CTRL_WAKE_SHIFT          (1 << 3) /* Bit3: Receiver Wakeup Method Select */
#define LPUART_CTRL_WAKE_MASK           (1 << LPUART_CTRL_WAKE_SHIFT)
#define LPUART_CTRL_WAKE_IDLE           (0 << LPUART_CTRL_WAKE_SHIFT)
#define LPUART_CTRL_WAKE_ADRM           (1 << LPUART_CTRL_WAKE_SHIFT)

#define LPUART_CTRL_ILT                 (1 << 2)
#define LPUART_CTRL_PE                  (1 << 1) /* Bit1: Parity Enable */

#define LPUART_CTRL_PT_SHIFT            (0) /* Bit0: Parity Type */
#define LPUART_CTRL_PT_MASK             (1 << LPUART_CTRL_PT_SHIFT)
#define LPUART_CTRL_PT_EVEN             (0 << LPUART_CTRL_PT_SHIFT)
#define LPUART_CTRL_PT_ODD              (1 << LPUART_CTRL_PT_SHIFT)

#define LPUART_DATA_NOISY               (1 << 15) /* Bit15: NOISY */
#define LPUART_DATA_PARITYE             (1 << 14) /* Bit14: Parity Error */
#define LPUART_DATA_FRETSC              (1 << 13) /* Bit13: Frame Error/Transmit Special Character */
#define LPUART_DATA_RXEMPT              (1 << 12) /* Bit12: Receive Buffer Empty */
#define LPUART_DATA_IDLINE              (1 << 11) /* Bit11: Idle Line */

#define LPUART_MATCH_MA2_SHIFT          (16)
#define LPUART_MATCH_MA2_MASK           (0x3ff << LPUART_MATCH_MA2_SHIFT)

#define LPUART_MATCH_MA1_SHIFT          (0)
#define LPUART_MATCH_MA1_MASK           (0x3ff << LPUART_MATCH_MA1_SHIFT)

/* REVISIT: MODIR */

#define LPUART_FIFO_TXEMPT              (1 << 23) /* Bit23: TX Buffer/FIFO Empty */
#define LPUART_FIFO_RXEMPT              (1 << 22) /* Bit22: RX Buffer/FIFO Empty */
#define LPUART_FIFO_TXOF                (1 << 17) /* Bit17: TX Buffer Overflow */
#define LPUART_FIFO_RXUF                (1 << 16) /* Bit16: RX Buffer Underflow */
#define LPUART_FIFO_TXFLUSH             (1 << 15) /* Bit15: TX Buffer/FIFO Flush */
#define LPUART_FIFO_RXFLUSH             (1 << 14) /* Bit14: RX Buffer/FIFO Flush */

#define LPUART_FIFO_RXIDEN_SHIFT        (10) /* Bit[12:10]: Receiver Idle Empty Enable */
#define LPUART_FIFO_RXIDEN_MASK         (0x7 << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_DISABLED     (0   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF1        (1   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF2        (2   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF4        (3   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF8        (4   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF16       (5   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF32       (6   << LPUART_FIFO_RXIDEN_SHIFT)
#define LPUART_FIFO_RXIDEN_RXRF64       (7   << LPUART_FIFO_RXIDEN_SHIFT)

#define LPUART_FIFO_TXOFE               (1 << 9) /* Bit9: TX FIFO Overflow Interrupt */
#define LPUART_FIFO_RXUFE               (1 << 8) /* Bit8: RXUF generate an Interrupt */
#define LPUART_FIFO_TXFE                (1 << 7) /* Bit7: TX FIFO Enable */

#define LPUART_FIFO_TXFIFOSIZE_SHIFT    (4) /* Bit[6:4]: TX FIFO Depth */
#define LPUART_FIFO_TXFIFOSIZE_MASK     (0x7 << LPUART_FIFO_TXFIFOSIZE_SHIFT)

#define LPUART_FIFO_RXFE                (1 << 3) /* Bit3: RX FIFO Enable */

#define LPUART_FIFO_RXFIFOSIZE_SHIFT    (0) /* Bit[2:0]: RX FIFO Depth */
#define LPUART_FIFO_RXFIFOSIZE_MASK     (0x7 << LPUART_FIFO_RXFIFOSIZE_SHIFT)

#define LPUART_WATER_RXCOUNT_SHIFT      (24) /* Bit[27:24]: Receive Counter */
#define LPUART_WATER_RXCOUNT_MASK       (0xf << LPUART_WATER_RXCOUNT_SHIFT)

#define LPUART_WATER_RXWATER_SHIFT      (16) /* Bit[18:16]: Receive Watermark */
#define LPUART_WATER_RXWATER_MASK       (0x7 << LPUART_WATER_RXWATER_SHIFT)

#define LPUART_WATER_TXCOUNT_SHIFT      (8) /* Bit[11:8]: Transmit Counter */
#define LPUART_WATER_TXCOUNT_MASK       (0xf << LPUART_WATER_TXCOUNT_SHIFT)

#define LPUART_WATER_TXWATER_SHIFT      (0) /* Bit[2:0]: Transmit Watermark */
#define LPUART_WATER_TXWATER_MASK       (0x7 << LPUART_WATER_TXWATER_SHIFT)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_LPUART_H */
