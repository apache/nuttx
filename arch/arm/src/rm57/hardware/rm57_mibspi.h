/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_mibspi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_MIBSPI_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_MIBSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The offsets below are those of SPNU562A Table 27-8, cross-checked against
 * the field order of HALCoGen's mibspiBASE_t.  The two
 * agree everywhere except SBERRADDR0, which HALCoGen pads out to an
 * unaligned 0x152 and Table 27-8 misprints the same way; the register's own
 * figure (27-85) places it at 0x14c, right after SBERRADDR1.
 */

#define RM57_NMIBSPI_TG            8      /* Usable transfer groups */
#define RM57_NMIBSPI_DMA           8      /* MibSPI DMA channels */
#define RM57_NMIBSPI_CS            6      /* Chip selects (2 on MibSPI2) */

/* Multi-buffer RAM depth (SPNS215C Section 7.11.2).  Only MibSPI1
 * implements the extended buffer mode that doubles it.
 */

#define RM57_MIBSPI_NBUF           128
#define RM57_MIBSPI_NBUF_EXTENDED  256

/* MibSPI Register Offsets **************************************************/

#define RM57_MIBSPI_GCR0_OFFSET         0x0000 /* Global Control Register 0 */
#define RM57_MIBSPI_GCR1_OFFSET         0x0004 /* Global Control Register 1 */
#define RM57_MIBSPI_INT0_OFFSET         0x0008 /* Interrupt Register */
#define RM57_MIBSPI_LVL_OFFSET          0x000c /* Interrupt Level Register */
#define RM57_MIBSPI_FLG_OFFSET          0x0010 /* Flag Register */
#define RM57_MIBSPI_PC0_OFFSET          0x0014 /* Pin Control 0: Function */
#define RM57_MIBSPI_PC1_OFFSET          0x0018 /* Pin Control 1: Direction */
#define RM57_MIBSPI_PC2_OFFSET          0x001c /* Pin Control 2: Data In */
#define RM57_MIBSPI_PC3_OFFSET          0x0020 /* Pin Control 3: Data Out */
#define RM57_MIBSPI_PC4_OFFSET          0x0024 /* Pin Control 4: Data Out Set */
#define RM57_MIBSPI_PC5_OFFSET          0x0028 /* Pin Control 5: Data Out Clear */
#define RM57_MIBSPI_PC6_OFFSET          0x002c /* Pin Control 6: Open Drain */
#define RM57_MIBSPI_PC7_OFFSET          0x0030 /* Pin Control 7: Pull Disable */
#define RM57_MIBSPI_PC8_OFFSET          0x0034 /* Pin Control 8: Pull Select */
#define RM57_MIBSPI_DAT0_OFFSET         0x0038 /* Transmit Data Register 0 */
#define RM57_MIBSPI_DAT1_OFFSET         0x003c /* Transmit Data Register 1 */
#define RM57_MIBSPI_BUF_OFFSET          0x0040 /* Receive Buffer Register */
#define RM57_MIBSPI_EMU_OFFSET          0x0044 /* Emulation Register */
#define RM57_MIBSPI_DELAY_OFFSET        0x0048 /* Delay Register */
#define RM57_MIBSPI_DEF_OFFSET          0x004c /* Default Chip Select Register */
#define RM57_MIBSPI_FMT_OFFSET(n)       (0x0050 + ((n) << 2))
#define RM57_MIBSPI_FMT0_OFFSET         0x0050 /* Data Format Register 0 */
#define RM57_MIBSPI_FMT1_OFFSET         0x0054 /* Data Format Register 1 */
#define RM57_MIBSPI_FMT2_OFFSET         0x0058 /* Data Format Register 2 */
#define RM57_MIBSPI_FMT3_OFFSET         0x005c /* Data Format Register 3 */
#define RM57_MIBSPI_INTVECT0_OFFSET     0x0060 /* Interrupt Vector 0 */
#define RM57_MIBSPI_INTVECT1_OFFSET     0x0064 /* Interrupt Vector 1 */
#define RM57_MIBSPI_PC9_OFFSET          0x0068 /* Pin Control 9: Slew Rate (MibSPI2) */
#define RM57_MIBSPI_PMCTRL_OFFSET       0x006c /* Parallel/Modulo Mode Control */

/* Everything from here on is only accessible in multi-buffer mode */

#define RM57_MIBSPI_MIBSPIE_OFFSET      0x0070 /* Multi-buffer Mode Enable */
#define RM57_MIBSPI_TGITENST_OFFSET     0x0074 /* TG Interrupt Enable Set */
#define RM57_MIBSPI_TGITENCR_OFFSET     0x0078 /* TG Interrupt Enable Clear */
#define RM57_MIBSPI_TGITLVST_OFFSET     0x007c /* TG Interrupt Level Set */
#define RM57_MIBSPI_TGITLVCR_OFFSET     0x0080 /* TG Interrupt Level Clear */
#define RM57_MIBSPI_TGINTFLG_OFFSET     0x0084 /* TG Interrupt Flag */
#define RM57_MIBSPI_TICKCNT_OFFSET      0x0090 /* Tick Count Register */
#define RM57_MIBSPI_LTGPEND_OFFSET      0x0094 /* Last TG End Pointer */

/* TG control n = 0..15, DMA channel control and count n = 0..7 */

#define RM57_MIBSPI_TGCTRL_OFFSET(n)    (0x0098 + ((n) << 2))
#define RM57_MIBSPI_DMACTRL_OFFSET(n)   (0x00d8 + ((n) << 2))
#define RM57_MIBSPI_DMACOUNT_OFFSET(n)  (0x00f8 + ((n) << 2))
#define RM57_MIBSPI_DMACNTLEN_OFFSET    0x0118 /* DMA Large Count */
#define RM57_MIBSPI_PARECCCTRL_OFFSET   0x0120 /* Parity/ECC Control */
#define RM57_MIBSPI_PARECCSTAT_OFFSET   0x0124 /* Parity/ECC Status */
#define RM57_MIBSPI_UERRADDR1_OFFSET    0x0128 /* Uncorrectable Error Address, RXRAM */
#define RM57_MIBSPI_UERRADDR0_OFFSET    0x012c /* Uncorrectable Error Address, TXRAM */
#define RM57_MIBSPI_RXOVRNADDR_OFFSET   0x0130 /* RXRAM Overrun Buffer Address */
#define RM57_MIBSPI_IOLPBKTSTCR_OFFSET  0x0134 /* I/O Loopback Test Control */
#define RM57_MIBSPI_EXTPRESCALE1_OFFSET 0x0138 /* Extended Prescale 1 (FMT0, FMT1) */
#define RM57_MIBSPI_EXTPRESCALE2_OFFSET 0x013c /* Extended Prescale 2 (FMT2, FMT3) */
#define RM57_MIBSPI_ECCDIAGCTRL_OFFSET  0x0140 /* ECC Diagnostic Control */
#define RM57_MIBSPI_ECCDIAGSTAT_OFFSET  0x0144 /* ECC Diagnostic Status */
#define RM57_MIBSPI_SBERRADDR1_OFFSET   0x0148 /* Single-Bit Error Address, RXRAM */
#define RM57_MIBSPI_SBERRADDR0_OFFSET   0x014c /* Single-Bit Error Address, TXRAM */

/* Multi-buffer RAM *********************************************************/

/* One 32-bit word per buffer in each bank.  The receive bank starts where
 * the transmit bank ends, so its offset doubles with the extended buffer
 * mode (SPNU562A Figures 27-90 and 27-91).
 */

#define RM57_MIBSPI_TXRAM_OFFSET(n)     ((n) << 2)
#define RM57_MIBSPI_RXRAM_OFFSET(n)     (0x0200 + ((n) << 2))
#define RM57_MIBSPI_RXRAM_EXT_OFFSET(n) (0x0400 + ((n) << 2))

/* MibSPI Register Bit Definitions ******************************************/

/* Global Control Register 0 */

#define MIBSPI_GCR0_NRESET              (1 << 0)  /* Bit 0: Out of reset */

/* Global Control Register 1 */

#define MIBSPI_GCR1_MASTER              (1 << 0)  /* Bit 0: SIMO out, SOMI in */
#define MIBSPI_GCR1_CLKMOD              (1 << 1)  /* Bit 1: Internal clock */
#define MIBSPI_GCR1_POWERDOWN           (1 << 8)  /* Bit 8: Local power-down */
#define MIBSPI_GCR1_LOOPBACK            (1 << 16) /* Bit 16: Internal loopback */
#define MIBSPI_GCR1_SPIEN               (1 << 24) /* Bit 24: Transfers enabled */

/* Interrupt Register, Interrupt Level Register and Flag Register share the
 * layout of their low 10 bits.
 */

#define MIBSPI_INT_DLENERR              (1 << 0)  /* Bit 0: Data length error */
#define MIBSPI_INT_TIMEOUT              (1 << 1)  /* Bit 1: ENA time-out */
#define MIBSPI_INT_PARERR               (1 << 2)  /* Bit 2: Parity error */
#define MIBSPI_INT_DESYNC               (1 << 3)  /* Bit 3: Slave desynchronized */
#define MIBSPI_INT_BITERR               (1 << 4)  /* Bit 4: Bit error */
#define MIBSPI_INT_RXOVRN               (1 << 6)  /* Bit 6: Receive overrun */
#define MIBSPI_INT_RX                   (1 << 8)  /* Bit 8: Receive buffer full */
#define MIBSPI_INT_TX                   (1 << 9)  /* Bit 9: Transmit buffer empty */

#define MIBSPI_INT_ALLERRORS            (MIBSPI_INT_DLENERR | MIBSPI_INT_TIMEOUT | \
                                         MIBSPI_INT_PARERR | MIBSPI_INT_DESYNC | \
                                         MIBSPI_INT_BITERR | MIBSPI_INT_RXOVRN)

#define MIBSPI_INT0_DMAREQEN            (1 << 16) /* Bit 16: DMA requests (INT0 only) */
#define MIBSPI_INT0_ENABLEHIGHZ         (1 << 24) /* Bit 24: ENA high-Z (INT0 only) */

#define MIBSPI_FLG_BUFINITACTIVE        (1 << 24) /* Bit 24: RAM init running (FLG only) */

/* Pin Control Registers 0-8.  All nine share this layout; PC9 uses the
 * CLK, SIMO0 and SOMI0 positions only.  Bits 16 and 24 mirror SIMO0 and
 * SOMI0, so on a 1-bit data path only bits 10 and 11 need to be written.
 */

#define MIBSPI_PC_SCS_SHIFT             (0)       /* Bits 0-7: Chip selects */
#define MIBSPI_PC_SCS_MASK              (0xff << MIBSPI_PC_SCS_SHIFT)
#  define MIBSPI_PC_SCS(n)              (1 << (n))
#define MIBSPI_PC_ENA                   (1 << 8)  /* Bit 8: SPIENA */
#define MIBSPI_PC_CLK                   (1 << 9)  /* Bit 9: SPICLK */
#define MIBSPI_PC_SIMO0                 (1 << 10) /* Bit 10: SPISIMO[0] */
#define MIBSPI_PC_SOMI0                 (1 << 11) /* Bit 11: SPISOMI[0] */
#define MIBSPI_PC_SIMO_SHIFT            (16)      /* Bits 16-23: SPISIMO[7:0] */
#define MIBSPI_PC_SIMO_MASK             (0xff << MIBSPI_PC_SIMO_SHIFT)
#  define MIBSPI_PC_SIMO(n)             (1 << (MIBSPI_PC_SIMO_SHIFT + (n)))
#define MIBSPI_PC_SOMI_SHIFT            (24)      /* Bits 24-31: SPISOMI[7:0] */
#define MIBSPI_PC_SOMI_MASK             (0xff << MIBSPI_PC_SOMI_SHIFT)
#  define MIBSPI_PC_SOMI(n)             (1 << (MIBSPI_PC_SOMI_SHIFT + (n)))

/* Transmit Data Register 0 */

#define MIBSPI_DAT0_TXDATA_MASK         (0xffff)

/* Transmit Data Register 1, and each word of the transmit RAM.  BUFMODE
 * and LOCK exist only in the RAM words.
 */

#define MIBSPI_DAT1_TXDATA_SHIFT        (0)       /* Bits 0-15: Transmit data */
#define MIBSPI_DAT1_TXDATA_MASK         (0xffff << MIBSPI_DAT1_TXDATA_SHIFT)
#define MIBSPI_DAT1_CSNR_SHIFT          (16)      /* Bits 16-23: Chip select mask */
#define MIBSPI_DAT1_CSNR_MASK           (0xff << MIBSPI_DAT1_CSNR_SHIFT)
#  define MIBSPI_DAT1_CSNR(n)           ((uint32_t)(n) << MIBSPI_DAT1_CSNR_SHIFT)
#define MIBSPI_DAT1_DFSEL_SHIFT         (24)      /* Bits 24-25: Data format */
#define MIBSPI_DAT1_DFSEL_MASK          (3 << MIBSPI_DAT1_DFSEL_SHIFT)
#  define MIBSPI_DAT1_DFSEL(n)          ((uint32_t)(n) << MIBSPI_DAT1_DFSEL_SHIFT)
#define MIBSPI_DAT1_WDEL                (1 << 26) /* Bit 26: Insert WDELAY */
#define MIBSPI_DAT1_CSHOLD              (1 << 28) /* Bit 28: Hold chip select */

#define MIBSPI_TXRAM_LOCK               (1 << 27) /* Bit 27: Lock next buffer */
#define MIBSPI_TXRAM_BUFMODE_SHIFT      (29)      /* Bits 29-31: Buffer mode */
#define MIBSPI_TXRAM_BUFMODE_MASK       (7 << MIBSPI_TXRAM_BUFMODE_SHIFT)
#  define MIBSPI_TXRAM_BUFMODE_DISABLED   (0 << MIBSPI_TXRAM_BUFMODE_SHIFT)
#  define MIBSPI_TXRAM_BUFMODE_SKIP_TX    (1 << MIBSPI_TXRAM_BUFMODE_SHIFT) /* Skip until TXFULL */
#  define MIBSPI_TXRAM_BUFMODE_SKIP_RX    (2 << MIBSPI_TXRAM_BUFMODE_SHIFT) /* Skip until RXEMPTY */
#  define MIBSPI_TXRAM_BUFMODE_SKIP_TXRX  (3 << MIBSPI_TXRAM_BUFMODE_SHIFT) /* Skip until both */
#  define MIBSPI_TXRAM_BUFMODE_CONTINUOUS (4 << MIBSPI_TXRAM_BUFMODE_SHIFT)
#  define MIBSPI_TXRAM_BUFMODE_WAIT_TX    (5 << MIBSPI_TXRAM_BUFMODE_SHIFT) /* Suspend until TXFULL */
#  define MIBSPI_TXRAM_BUFMODE_WAIT_RX    (6 << MIBSPI_TXRAM_BUFMODE_SHIFT) /* Suspend until RXEMPTY */
#  define MIBSPI_TXRAM_BUFMODE_WAIT_TXRX  (7 << MIBSPI_TXRAM_BUFMODE_SHIFT) /* Suspend until both */

/* Receive Buffer Register, Emulation Register and each word of the receive
 * RAM.
 */

#define MIBSPI_BUF_RXDATA_SHIFT         (0)       /* Bits 0-15: Receive data */
#define MIBSPI_BUF_RXDATA_MASK          (0xffff << MIBSPI_BUF_RXDATA_SHIFT)
#define MIBSPI_BUF_LCSNR_SHIFT          (16)      /* Bits 16-23: Last chip select */
#define MIBSPI_BUF_LCSNR_MASK           (0xff << MIBSPI_BUF_LCSNR_SHIFT)
#define MIBSPI_BUF_DLENERR              (1 << 24) /* Bit 24: Data length error */
#define MIBSPI_BUF_TIMEOUT              (1 << 25) /* Bit 25: ENA time-out */
#define MIBSPI_BUF_PARITYERR            (1 << 26) /* Bit 26: Parity error */
#define MIBSPI_BUF_DESYNC               (1 << 27) /* Bit 27: Slave desynchronized */
#define MIBSPI_BUF_BITERR               (1 << 28) /* Bit 28: Bit error */
#define MIBSPI_BUF_TXFULL               (1 << 29) /* Bit 29: Transmit buffer full */
#define MIBSPI_BUF_RXOVR                (1 << 30) /* Bit 30: Receive overrun */
#define MIBSPI_BUF_RXEMPTY              (1 << 31) /* Bit 31: Nothing received */

#define MIBSPI_BUF_ALLERRORS            (MIBSPI_BUF_DLENERR | MIBSPI_BUF_TIMEOUT | \
                                         MIBSPI_BUF_PARITYERR | MIBSPI_BUF_DESYNC | \
                                         MIBSPI_BUF_BITERR | MIBSPI_BUF_RXOVR)

/* Delay Register.  The first two count VCLK cycles, the last two SPICLK
 * cycles.
 */

#define MIBSPI_DELAY_C2EDELAY_SHIFT     (0)       /* Bits 0-7: CS active to ENA time-out */
#define MIBSPI_DELAY_C2EDELAY_MASK      (0xff << MIBSPI_DELAY_C2EDELAY_SHIFT)
#  define MIBSPI_DELAY_C2EDELAY(n)      ((uint32_t)(n) << MIBSPI_DELAY_C2EDELAY_SHIFT)
#define MIBSPI_DELAY_T2EDELAY_SHIFT     (8)       /* Bits 8-15: Transmit end to ENA time-out */
#define MIBSPI_DELAY_T2EDELAY_MASK      (0xff << MIBSPI_DELAY_T2EDELAY_SHIFT)
#  define MIBSPI_DELAY_T2EDELAY(n)      ((uint32_t)(n) << MIBSPI_DELAY_T2EDELAY_SHIFT)
#define MIBSPI_DELAY_T2CDELAY_SHIFT     (16)      /* Bits 16-23: Transmit end to CS inactive */
#define MIBSPI_DELAY_T2CDELAY_MASK      (0xff << MIBSPI_DELAY_T2CDELAY_SHIFT)
#  define MIBSPI_DELAY_T2CDELAY(n)      ((uint32_t)(n) << MIBSPI_DELAY_T2CDELAY_SHIFT)
#define MIBSPI_DELAY_C2TDELAY_SHIFT     (24)      /* Bits 24-31: CS active to transmit start */
#define MIBSPI_DELAY_C2TDELAY_MASK      (0xff << MIBSPI_DELAY_C2TDELAY_SHIFT)
#  define MIBSPI_DELAY_C2TDELAY(n)      ((uint32_t)(n) << MIBSPI_DELAY_C2TDELAY_SHIFT)

/* Default Chip Select Register */

#define MIBSPI_DEF_CSDEF_MASK           (0xff)

/* Data Format Registers 0-3.
 *
 * PHASE is the inverse of the usual CPHA: with PHASE = 0 data changes on
 * the first clock edge and is sampled on the second, which is CPHA = 1
 * (SPNU562A Table 27-3).  POLARITY matches CPOL.
 */

#define MIBSPI_FMT_CHARLEN_SHIFT        (0)       /* Bits 0-4: Word length, 2-16 */
#define MIBSPI_FMT_CHARLEN_MASK         (0x1f << MIBSPI_FMT_CHARLEN_SHIFT)
#  define MIBSPI_FMT_CHARLEN(n)         ((uint32_t)(n) << MIBSPI_FMT_CHARLEN_SHIFT)
#define MIBSPI_FMT_PRESCALE_SHIFT       (8)       /* Bits 8-15: SPICLK = VCLK / (n + 1) */
#define MIBSPI_FMT_PRESCALE_MASK        (0xff << MIBSPI_FMT_PRESCALE_SHIFT)
#  define MIBSPI_FMT_PRESCALE(n)        ((uint32_t)(n) << MIBSPI_FMT_PRESCALE_SHIFT)
#define MIBSPI_FMT_PHASE                (1 << 16) /* Bit 16: Clock delayed half a cycle */
#define MIBSPI_FMT_POLARITY             (1 << 17) /* Bit 17: Clock idles high */
#define MIBSPI_FMT_DISCSTIMERS          (1 << 18) /* Bit 18: No C2TDELAY/T2CDELAY */
#define MIBSPI_FMT_HDUPLEXENA           (1 << 19) /* Bit 19: Half duplex on SIMO */
#define MIBSPI_FMT_SHIFTDIR             (1 << 20) /* Bit 20: LSB first */
#define MIBSPI_FMT_WAITENA              (1 << 21) /* Bit 21: Master waits for ENA */
#define MIBSPI_FMT_PARITYENA            (1 << 22) /* Bit 22: Parity bit appended */
#define MIBSPI_FMT_PARPOL               (1 << 23) /* Bit 23: Odd parity */
#define MIBSPI_FMT_WDELAY_SHIFT         (24)      /* Bits 24-31: Delay between words */
#define MIBSPI_FMT_WDELAY_MASK          (0xff << MIBSPI_FMT_WDELAY_SHIFT)
#  define MIBSPI_FMT_WDELAY(n)          ((uint32_t)(n) << MIBSPI_FMT_WDELAY_SHIFT)

#define MIBSPI_FMT_CHARLEN_MIN          2
#define MIBSPI_FMT_CHARLEN_MAX          16

/* Interrupt Vector 0 and 1 */

#define MIBSPI_INTVECT_SUSPEND          (1 << 0)  /* Bit 0: TG suspended, not finished */
#define MIBSPI_INTVECT_SHIFT            (1)       /* Bits 1-5: Pending source */
#define MIBSPI_INTVECT_MASK             (0x1f << MIBSPI_INTVECT_SHIFT)
#  define MIBSPI_INTVECT_NONE           (0x00 << MIBSPI_INTVECT_SHIFT)
#  define MIBSPI_INTVECT_TG(n)          (((uint32_t)(n) + 1) << MIBSPI_INTVECT_SHIFT)
#  define MIBSPI_INTVECT_ERROR          (0x11 << MIBSPI_INTVECT_SHIFT)
#  define MIBSPI_INTVECT_RX             (0x12 << MIBSPI_INTVECT_SHIFT) /* Compatibility mode */
#  define MIBSPI_INTVECT_RXOVRN         (0x13 << MIBSPI_INTVECT_SHIFT)
#  define MIBSPI_INTVECT_TX             (0x14 << MIBSPI_INTVECT_SHIFT) /* Compatibility mode */

/* Parallel/Modulo Mode Control Register: one byte per data format.  Modulo
 * mode is not implemented on this device.
 */

#define MIBSPI_PMCTRL_PMODE_SHIFT(n)    ((n) << 3) /* Bits 0-1 of byte n */
#define MIBSPI_PMCTRL_PMODE_MASK(n)     (3 << MIBSPI_PMCTRL_PMODE_SHIFT(n))
#  define MIBSPI_PMCTRL_PMODE_1LINE(n)  (0 << MIBSPI_PMCTRL_PMODE_SHIFT(n))
#  define MIBSPI_PMCTRL_PMODE_2LINE(n)  (1 << MIBSPI_PMCTRL_PMODE_SHIFT(n))
#  define MIBSPI_PMCTRL_PMODE_4LINE(n)  (2 << MIBSPI_PMCTRL_PMODE_SHIFT(n))
#  define MIBSPI_PMCTRL_PMODE_8LINE(n)  (3 << MIBSPI_PMCTRL_PMODE_SHIFT(n))

/* Multi-buffer Mode Enable Register */

#define MIBSPI_MIBSPIE_MSPIENA          (1 << 0)  /* Bit 0: Multi-buffer mode */
#define MIBSPI_MIBSPIE_EXTBUF_SHIFT     (8)       /* Bits 8-11: Extended buffer key */
#define MIBSPI_MIBSPIE_EXTBUF_MASK      (0xf << MIBSPI_MIBSPIE_EXTBUF_SHIFT)
#  define MIBSPI_MIBSPIE_EXTBUF_DISABLE (0x5 << MIBSPI_MIBSPIE_EXTBUF_SHIFT) /* 128 buffers */
#  define MIBSPI_MIBSPIE_EXTBUF_ENABLE  (0xa << MIBSPI_MIBSPIE_EXTBUF_SHIFT) /* 256 buffers */
#define MIBSPI_MIBSPIE_RXRAMACCESS      (1 << 16)

/* TG Interrupt Enable Set/Clear, Level Set/Clear and Flag Registers: the
 * "suspended" bits for TG0-15 in the low half, the "finished" bits in the
 * high half.
 */

#define MIBSPI_TGINT_SUS(n)             (1 << (n))
#define MIBSPI_TGINT_RDY(n)             (1 << (16 + (n)))

/* Tick Count Register */

#define MIBSPI_TICKCNT_TICKVALUE_SHIFT  (0)       /* Bits 0-15: Reload value */
#define MIBSPI_TICKCNT_TICKVALUE_MASK   (0xffff << MIBSPI_TICKCNT_TICKVALUE_SHIFT)
#define MIBSPI_TICKCNT_CLKCTRL_SHIFT    (28)      /* Bits 28-29: SPICLK of format n */
#define MIBSPI_TICKCNT_CLKCTRL_MASK     (3 << MIBSPI_TICKCNT_CLKCTRL_SHIFT)
#  define MIBSPI_TICKCNT_CLKCTRL(n)     ((uint32_t)(n) << MIBSPI_TICKCNT_CLKCTRL_SHIFT)
#define MIBSPI_TICKCNT_RELOAD           (1 << 30) /* Bit 30: Reload now */
#define MIBSPI_TICKCNT_TICKENA          (1 << 31) /* Bit 31: Counter enabled */

/* Last TG End Pointer */

#define MIBSPI_LTGPEND_LPEND_SHIFT      (8)       /* Bits 8-15: Last buffer of last TG */
#define MIBSPI_LTGPEND_LPEND_MASK       (0xff << MIBSPI_LTGPEND_LPEND_SHIFT)
#  define MIBSPI_LTGPEND_LPEND(n)       ((uint32_t)(n) << MIBSPI_LTGPEND_LPEND_SHIFT)
#define MIBSPI_LTGPEND_TGINSERVICE_SHIFT (24)     /* Bits 24-28: TG in service + 1 */
#define MIBSPI_LTGPEND_TGINSERVICE_MASK (0x1f << MIBSPI_LTGPEND_TGINSERVICE_SHIFT)

/* TG Control Registers */

#define MIBSPI_TGCTRL_PCURRENT_SHIFT    (0)       /* Bits 0-7: Current buffer */
#define MIBSPI_TGCTRL_PCURRENT_MASK     (0xff << MIBSPI_TGCTRL_PCURRENT_SHIFT)
#define MIBSPI_TGCTRL_PSTART_SHIFT      (8)       /* Bits 8-15: First buffer */
#define MIBSPI_TGCTRL_PSTART_MASK       (0xff << MIBSPI_TGCTRL_PSTART_SHIFT)
#  define MIBSPI_TGCTRL_PSTART(n)       ((uint32_t)(n) << MIBSPI_TGCTRL_PSTART_SHIFT)
#define MIBSPI_TGCTRL_TRIGSRC_SHIFT     (16)      /* Bits 16-19: Trigger source */
#define MIBSPI_TGCTRL_TRIGSRC_MASK      (0xf << MIBSPI_TGCTRL_TRIGSRC_SHIFT)
#  define MIBSPI_TGCTRL_TRIGSRC_NONE    (0x0 << MIBSPI_TGCTRL_TRIGSRC_SHIFT)
#  define MIBSPI_TGCTRL_TRIGSRC_EXT(n)  (((uint32_t)(n) + 1) << MIBSPI_TGCTRL_TRIGSRC_SHIFT)
#  define MIBSPI_TGCTRL_TRIGSRC_TICK    (0xf << MIBSPI_TGCTRL_TRIGSRC_SHIFT)
#define MIBSPI_TGCTRL_TRIGEVT_SHIFT     (20)      /* Bits 20-23: Trigger event */
#define MIBSPI_TGCTRL_TRIGEVT_MASK      (0xf << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_NEVER   (0 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_RISING  (1 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_FALLING (2 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_BOTH    (3 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_HIGH    (5 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_LOW     (6 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#  define MIBSPI_TGCTRL_TRIGEVT_ALWAYS  (7 << MIBSPI_TGCTRL_TRIGEVT_SHIFT)
#define MIBSPI_TGCTRL_TGTD              (1 << 28) /* Bit 28: Triggered, pending or in service */
#define MIBSPI_TGCTRL_PRST              (1 << 29) /* Bit 29: Retrigger restarts the TG */
#define MIBSPI_TGCTRL_ONESHOT           (1 << 30) /* Bit 30: Disable after one pass */
#define MIBSPI_TGCTRL_TGENA             (1 << 31) /* Bit 31: TG enabled */

/* DMA Channel Control Registers */

#define MIBSPI_DMACTRL_COUNT_SHIFT      (0)       /* Bits 0-5: Remaining transfers */
#define MIBSPI_DMACTRL_COUNT_MASK       (0x3f << MIBSPI_DMACTRL_COUNT_SHIFT)
#define MIBSPI_DMACTRL_COUNTBIT17       (1 << 6)  /* Bit 6: Bit 16 of DMAxCOUNT.COUNT */
#define MIBSPI_DMACTRL_ICOUNT_SHIFT     (8)       /* Bits 8-12: Transfers - 1 */
#define MIBSPI_DMACTRL_ICOUNT_MASK      (0x1f << MIBSPI_DMACTRL_ICOUNT_SHIFT)
#  define MIBSPI_DMACTRL_ICOUNT(n)      ((uint32_t)(n) << MIBSPI_DMACTRL_ICOUNT_SHIFT)
#define MIBSPI_DMACTRL_NOBRK            (1 << 13) /* Bit 13: Do not interleave other buffers */
#define MIBSPI_DMACTRL_TXDMAENA         (1 << 14) /* Bit 14: Transmit requests */
#define MIBSPI_DMACTRL_RXDMAENA         (1 << 15) /* Bit 15: Receive requests */
#define MIBSPI_DMACTRL_TXDMAMAP_SHIFT   (16)      /* Bits 16-19: Module request line, TX */
#define MIBSPI_DMACTRL_TXDMAMAP_MASK    (0xf << MIBSPI_DMACTRL_TXDMAMAP_SHIFT)
#  define MIBSPI_DMACTRL_TXDMAMAP(n)    ((uint32_t)(n) << MIBSPI_DMACTRL_TXDMAMAP_SHIFT)
#define MIBSPI_DMACTRL_RXDMAMAP_SHIFT   (20)      /* Bits 20-23: Module request line, RX */
#define MIBSPI_DMACTRL_RXDMAMAP_MASK    (0xf << MIBSPI_DMACTRL_RXDMAMAP_SHIFT)
#  define MIBSPI_DMACTRL_RXDMAMAP(n)    ((uint32_t)(n) << MIBSPI_DMACTRL_RXDMAMAP_SHIFT)
#define MIBSPI_DMACTRL_BUFID_SHIFT      (24)      /* Bits 24-30: Buffer served */
#define MIBSPI_DMACTRL_BUFID_MASK       (0x7f << MIBSPI_DMACTRL_BUFID_SHIFT)
#  define MIBSPI_DMACTRL_BUFID(n)       ((uint32_t)(n) << MIBSPI_DMACTRL_BUFID_SHIFT)
#define MIBSPI_DMACTRL_ONESHOT          (1 << 31) /* Bit 31: Disable after ICOUNT+1 */

/* DMAxCOUNT Registers.  Used in place of the DMAxCTRL counters once
 * DMACNTLEN.LARGECOUNT is set.
 */

#define MIBSPI_DMACOUNT_COUNT_SHIFT     (0)       /* Bits 0-15: Remaining transfers */
#define MIBSPI_DMACOUNT_COUNT_MASK      (0xffff << MIBSPI_DMACOUNT_COUNT_SHIFT)
#define MIBSPI_DMACOUNT_ICOUNT_SHIFT    (16)      /* Bits 16-31: Transfers - 1 */
#define MIBSPI_DMACOUNT_ICOUNT_MASK     (0xffff << MIBSPI_DMACOUNT_ICOUNT_SHIFT)
#  define MIBSPI_DMACOUNT_ICOUNT(n)     ((uint32_t)(n) << MIBSPI_DMACOUNT_ICOUNT_SHIFT)

/* DMA Large Count Register */

#define MIBSPI_DMACNTLEN_LARGECOUNT     (1 << 0)  /* Bit 0: Use DMAxCOUNT */

/* Parity/ECC Control Register.
 *
 * EDEN is a key field whose only "off" value is 5h, which is also its
 * reset value: any other pattern - 0h included - turns detection on.
 * HALCoGen writes 5h under a comment that says "enable".
 */

#define MIBSPI_PARECCCTRL_EDEN_SHIFT    (0)       /* Bits 0-3: Error detection key */
#define MIBSPI_PARECCCTRL_EDEN_MASK     (0xf << MIBSPI_PARECCCTRL_EDEN_SHIFT)
#  define MIBSPI_PARECCCTRL_EDEN_DISABLE (0x5 << MIBSPI_PARECCCTRL_EDEN_SHIFT)
#  define MIBSPI_PARECCCTRL_EDEN_ENABLE  (0xa << MIBSPI_PARECCCTRL_EDEN_SHIFT)
#define MIBSPI_PARECCCTRL_PTESTEN       (1 << 8)  /* Bit 8: Parity/ECC bits memory mapped */
#define MIBSPI_PARECCCTRL_EDACMODE_SHIFT (16)     /* Bits 16-19: Single-bit correction key */
#define MIBSPI_PARECCCTRL_EDACMODE_MASK (0xf << MIBSPI_PARECCCTRL_EDACMODE_SHIFT)
#  define MIBSPI_PARECCCTRL_EDACMODE_DISABLE (0x5 << MIBSPI_PARECCCTRL_EDACMODE_SHIFT)
#  define MIBSPI_PARECCCTRL_EDACMODE_ENABLE  (0xa << MIBSPI_PARECCCTRL_EDACMODE_SHIFT)
#define MIBSPI_PARECCCTRL_SBEEVTEN_SHIFT (24)     /* Bits 24-27: Single-bit event key */
#define MIBSPI_PARECCCTRL_SBEEVTEN_MASK (0xf << MIBSPI_PARECCCTRL_SBEEVTEN_SHIFT)
#  define MIBSPI_PARECCCTRL_SBEEVTEN_DISABLE (0x5 << MIBSPI_PARECCCTRL_SBEEVTEN_SHIFT)
#  define MIBSPI_PARECCCTRL_SBEEVTEN_ENABLE  (0xa << MIBSPI_PARECCCTRL_SBEEVTEN_SHIFT)

/* Parity/ECC Status Register */

#define MIBSPI_PARECCSTAT_UERRFLG0      (1 << 0)  /* Bit 0: Uncorrectable error, TXRAM */
#define MIBSPI_PARECCSTAT_UERRFLG1      (1 << 1)  /* Bit 1: Uncorrectable error, RXRAM */
#define MIBSPI_PARECCSTAT_SBEFLG0       (1 << 8)  /* Bit 8: Single-bit error, TXRAM */
#define MIBSPI_PARECCSTAT_SBEFLG1       (1 << 9)  /* Bit 9: Single-bit error, RXRAM */

/* Error address registers (UERRADDR0/1, SBERRADDR0/1) */

#define MIBSPI_ERRADDR_MASK             (0x7ff)

/* RXRAM Overrun Buffer Address Register */

#define MIBSPI_RXOVRNADDR_MASK          (0x3ff)

/* I/O Loopback Test Control Register */

#define MIBSPI_IOLPBKTSTCR_RXPENA       (1 << 0)  /* Bit 0: Analog loopback via receive pin */
#define MIBSPI_IOLPBKTSTCR_LPBKTYPE     (1 << 1)  /* Bit 1: Analog rather than digital */
#define MIBSPI_IOLPBKTSTCR_CTRLSCSPINERR (1 << 2) /* Bit 2: Inject chip select error */
#define MIBSPI_IOLPBKTSTCR_ERRSCSPIN_SHIFT (3)    /* Bits 3-5: Chip select to corrupt */
#define MIBSPI_IOLPBKTSTCR_ERRSCSPIN_MASK (7 << MIBSPI_IOLPBKTSTCR_ERRSCSPIN_SHIFT)
#define MIBSPI_IOLPBKTSTCR_ENA_SHIFT    (8)       /* Bits 8-11: Loopback key */
#define MIBSPI_IOLPBKTSTCR_ENA_MASK     (0xf << MIBSPI_IOLPBKTSTCR_ENA_SHIFT)
#  define MIBSPI_IOLPBKTSTCR_ENA_DISABLE (0x5 << MIBSPI_IOLPBKTSTCR_ENA_SHIFT)
#  define MIBSPI_IOLPBKTSTCR_ENA_ENABLE  (0xa << MIBSPI_IOLPBKTSTCR_ENA_SHIFT)
#define MIBSPI_IOLPBKTSTCR_CTRLDLENERR  (1 << 16) /* Bit 16: Inject data length error */
#define MIBSPI_IOLPBKTSTCR_CTRLTIMEOUT  (1 << 17) /* Bit 17: Inject ENA time-out */
#define MIBSPI_IOLPBKTSTCR_CTRLPARERR   (1 << 18) /* Bit 18: Inject parity error */
#define MIBSPI_IOLPBKTSTCR_CTRLDESYNC   (1 << 19) /* Bit 19: Inject desync error */
#define MIBSPI_IOLPBKTSTCR_CTRLBITERR   (1 << 20) /* Bit 20: Inject bit error */
#define MIBSPI_IOLPBKTSTCR_SCSFAILFLG   (1 << 24) /* Bit 24: Chip select miscompare */

/* Extended Prescale Registers 1 and 2.  The low 8 bits of each field alias
 * the PRESCALE field of the matching SPIFMTn, and writing that field clears
 * the upper three bits here - so this has to be written after SPIFMTn.
 */

#define MIBSPI_EXTPRESCALE_LO_SHIFT     (0)       /* Bits 0-10: FMT0 (reg 1), FMT2 (reg 2) */
#define MIBSPI_EXTPRESCALE_LO_MASK      (0x7ff << MIBSPI_EXTPRESCALE_LO_SHIFT)
#  define MIBSPI_EXTPRESCALE_LO(n)      ((uint32_t)(n) << MIBSPI_EXTPRESCALE_LO_SHIFT)
#define MIBSPI_EXTPRESCALE_HI_SHIFT     (16)      /* Bits 16-26: FMT1 (reg 1), FMT3 (reg 2) */
#define MIBSPI_EXTPRESCALE_HI_MASK      (0x7ff << MIBSPI_EXTPRESCALE_HI_SHIFT)
#  define MIBSPI_EXTPRESCALE_HI(n)      ((uint32_t)(n) << MIBSPI_EXTPRESCALE_HI_SHIFT)

#define MIBSPI_PRESCALE_MAX             0xff      /* SPIFMTn alone */
#define MIBSPI_EXTPRESCALE_MAX          0x7ff     /* With the extended register */

/* ECC Diagnostic Control and Status Registers */

#define MIBSPI_ECCDIAGCTRL_EN_MASK      (0xf)
#  define MIBSPI_ECCDIAGCTRL_EN_ENABLE  (0x5)     /* Note: 5h enables here */
#  define MIBSPI_ECCDIAGCTRL_EN_DISABLE (0xa)

#define MIBSPI_ECCDIAGSTAT_SEFLG0       (1 << 0)  /* Bit 0: Single-bit error, TXRAM */
#define MIBSPI_ECCDIAGSTAT_SEFLG1       (1 << 1)  /* Bit 1: Single-bit error, RXRAM */
#define MIBSPI_ECCDIAGSTAT_DEFLG0       (1 << 16) /* Bit 16: Double-bit error, TXRAM */
#define MIBSPI_ECCDIAGSTAT_DEFLG1       (1 << 17) /* Bit 17: Double-bit error, RXRAM */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_MIBSPI_H */