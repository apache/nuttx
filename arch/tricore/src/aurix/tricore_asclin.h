/****************************************************************************
 * arch/tricore/src/aurix/tricore_asclin.h
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

#ifndef __ARCH_TRICORE_SRC_AURIX_TRICORE_ASCLIN_H
#define __ARCH_TRICORE_SRC_AURIX_TRICORE_ASCLIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/bits.h>
#include <stdint.h>

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
#  include "hardware/tc4x_asclin.h"
#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
#  include "hardware/tc3x_asclin.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ASCLIN_REG(base, off)       ((base) + (off))

#define ASCLIN_CLC_DISR             BIT(0)
#define ASCLIN_CLC_DISS             BIT(1)
#define ASCLIN_CLC_EDIS             BIT(3)

#define ASCLIN_IOCR_ALTI_SHIFT      0
#define ASCLIN_IOCR_ALTI_MASK       GENMASK(2, 0)
#define ASCLIN_IOCR_DEPTH_SHIFT     4
#define ASCLIN_IOCR_DEPTH_MASK      GENMASK(9, 4)
#define ASCLIN_IOCR_CTS_SHIFT       16
#define ASCLIN_IOCR_CTS_MASK        GENMASK(17, 16)
#define ASCLIN_IOCR_RCPOL           BIT(25)
#define ASCLIN_IOCR_CPOL            BIT(26)
#define ASCLIN_IOCR_SPOL            BIT(27)
#define ASCLIN_IOCR_LB              BIT(28)
#define ASCLIN_IOCR_CTSEN           BIT(29)
#define ASCLIN_IOCR_RXM             BIT(30)
#define ASCLIN_IOCR_TXM             BIT(31)

#define ASCLIN_IOCR_ALTI_A          0
#define ASCLIN_IOCR_ALTI_B          1
#define ASCLIN_IOCR_ALTI_C          2
#define ASCLIN_IOCR_ALTI_D          3
#define ASCLIN_IOCR_ALTI_E          4
#define ASCLIN_IOCR_ALTI_F          5
#define ASCLIN_IOCR_ALTI_G          6
#define ASCLIN_IOCR_ALTI_H          7

#define ASCLIN_TXFIFOCON_FLUSH      BIT(0)
#define ASCLIN_TXFIFOCON_ENO        BIT(1)
#define ASCLIN_TXFIFOCON_FM_SHIFT   4
#define ASCLIN_TXFIFOCON_FM_MASK    GENMASK(5, 4)
#define ASCLIN_TXFIFOCON_INW_SHIFT  6
#define ASCLIN_TXFIFOCON_INW_MASK   GENMASK(7, 6)
#define ASCLIN_TXFIFOCON_INTLEVEL_SHIFT 8
#define ASCLIN_TXFIFOCON_INTLEVEL_MASK  GENMASK(11, 8)
#define ASCLIN_TXFIFOCON_FILL_SHIFT 16
#define ASCLIN_TXFIFOCON_FILL_MASK  GENMASK(20, 16)

#define ASCLIN_TXFIFOCON_INW_1      (1 << ASCLIN_TXFIFOCON_INW_SHIFT)
#define ASCLIN_TXFIFOCON_FM_COMBINED  (0 << ASCLIN_TXFIFOCON_FM_SHIFT)

#define ASCLIN_RXFIFOCON_FLUSH      BIT(0)
#define ASCLIN_RXFIFOCON_ENI        BIT(1)
#define ASCLIN_RXFIFOCON_FM_SHIFT   4
#define ASCLIN_RXFIFOCON_FM_MASK    GENMASK(5, 4)
#define ASCLIN_RXFIFOCON_OUTW_SHIFT 6
#define ASCLIN_RXFIFOCON_OUTW_MASK  GENMASK(7, 6)
#define ASCLIN_RXFIFOCON_INTLEVEL_SHIFT 8
#define ASCLIN_RXFIFOCON_INTLEVEL_MASK  GENMASK(11, 8)
#define ASCLIN_RXFIFOCON_FILL_SHIFT 16
#define ASCLIN_RXFIFOCON_FILL_MASK  GENMASK(20, 16)
#define ASCLIN_RXFIFOCON_BUF        BIT(31)

#define ASCLIN_RXFIFOCON_OUTW_1     (1 << ASCLIN_RXFIFOCON_OUTW_SHIFT)

#define ASCLIN_BITCON_PRESCALER_SHIFT   0
#define ASCLIN_BITCON_PRESCALER_MASK    GENMASK(11, 0)
#define ASCLIN_BITCON_OVERSAMPLING_SHIFT 16
#define ASCLIN_BITCON_OVERSAMPLING_MASK GENMASK(19, 16)
#define ASCLIN_BITCON_SAMPLEPOINT_SHIFT 24
#define ASCLIN_BITCON_SAMPLEPOINT_MASK  GENMASK(27, 24)
#define ASCLIN_BITCON_SM                BIT(31)

#define ASCLIN_FRAMECON_IDLE_SHIFT  6
#define ASCLIN_FRAMECON_IDLE_MASK   GENMASK(8, 6)
#define ASCLIN_FRAMECON_STOP_SHIFT  9
#define ASCLIN_FRAMECON_STOP_MASK   GENMASK(11, 9)
#define ASCLIN_FRAMECON_LEAD_SHIFT  12
#define ASCLIN_FRAMECON_LEAD_MASK   GENMASK(14, 12)
#define ASCLIN_FRAMECON_MODE_SHIFT  16
#define ASCLIN_FRAMECON_MODE_MASK   GENMASK(17, 16)
#define ASCLIN_FRAMECON_MSB         BIT(28)
#define ASCLIN_FRAMECON_CEN         BIT(29)
#define ASCLIN_FRAMECON_PEN         BIT(30)
#define ASCLIN_FRAMECON_ODD         BIT(31)

#define ASCLIN_FRAMECON_MODE_INIT   (0 << ASCLIN_FRAMECON_MODE_SHIFT)
#define ASCLIN_FRAMECON_MODE_ASC    (1 << ASCLIN_FRAMECON_MODE_SHIFT)
#define ASCLIN_FRAMECON_MODE_SPI    (2 << ASCLIN_FRAMECON_MODE_SHIFT)
#define ASCLIN_FRAMECON_MODE_LIN    (3 << ASCLIN_FRAMECON_MODE_SHIFT)

#define ASCLIN_FRAMECON_STOP_1      (1 << ASCLIN_FRAMECON_STOP_SHIFT)
#define ASCLIN_FRAMECON_STOP_2      (2 << ASCLIN_FRAMECON_STOP_SHIFT)

#define ASCLIN_DATCON_DATLEN_SHIFT  0
#define ASCLIN_DATCON_DATLEN_MASK   GENMASK(3, 0)
#define ASCLIN_DATCON_TRGSRC_SHIFT  8
#define ASCLIN_DATCON_TRGSRC_MASK   GENMASK(9, 8)

#define ASCLIN_DATCON_DATLEN(n)     (((n) - 1) & 0xf)

#define ASCLIN_BRG_DENOMINATOR_SHIFT  0
#define ASCLIN_BRG_DENOMINATOR_MASK   GENMASK(11, 0)
#define ASCLIN_BRG_NUMERATOR_SHIFT    16
#define ASCLIN_BRG_NUMERATOR_MASK     GENMASK(27, 16)

#define ASCLIN_BRG_VAL(num, den)     ((((num) & 0xfff) << 16) | ((den) & 0xfff))

#define ASCLIN_FLAGS_TH             BIT(0)
#define ASCLIN_FLAGS_TR             BIT(1)
#define ASCLIN_FLAGS_RH             BIT(2)
#define ASCLIN_FLAGS_RR             BIT(3)
#define ASCLIN_FLAGS_FED            BIT(5)
#define ASCLIN_FLAGS_RED            BIT(6)
#define ASCLIN_FLAGS_TFE            BIT(10)
#define ASCLIN_FLAGS_PE             BIT(16)
#define ASCLIN_FLAGS_TC             BIT(17)
#define ASCLIN_FLAGS_FE             BIT(18)
#define ASCLIN_FLAGS_HT             BIT(19)
#define ASCLIN_FLAGS_RT             BIT(20)
#define ASCLIN_FLAGS_BD             BIT(21)
#define ASCLIN_FLAGS_CE             BIT(25)
#define ASCLIN_FLAGS_RFO            BIT(26)
#define ASCLIN_FLAGS_RFU            BIT(27)
#define ASCLIN_FLAGS_RFL            BIT(28)
#define ASCLIN_FLAGS_TFO            BIT(30)
#define ASCLIN_FLAGS_TFL            BIT(31)

#define ASCLIN_FLAGSCLEAR_THC       BIT(0)
#define ASCLIN_FLAGSCLEAR_TRC       BIT(1)
#define ASCLIN_FLAGSCLEAR_RHC       BIT(2)
#define ASCLIN_FLAGSCLEAR_RRC       BIT(3)
#define ASCLIN_FLAGSCLEAR_FEDC      BIT(5)
#define ASCLIN_FLAGSCLEAR_REDC      BIT(6)
#define ASCLIN_FLAGSCLEAR_TFEC      BIT(10)
#define ASCLIN_FLAGSCLEAR_PEC       BIT(16)
#define ASCLIN_FLAGSCLEAR_TCC       BIT(17)
#define ASCLIN_FLAGSCLEAR_FEC       BIT(18)
#define ASCLIN_FLAGSCLEAR_HTC       BIT(19)
#define ASCLIN_FLAGSCLEAR_RTC       BIT(20)
#define ASCLIN_FLAGSCLEAR_BDC       BIT(21)
#define ASCLIN_FLAGSCLEAR_CEC       BIT(25)
#define ASCLIN_FLAGSCLEAR_RFOC      BIT(26)
#define ASCLIN_FLAGSCLEAR_RFUC      BIT(27)
#define ASCLIN_FLAGSCLEAR_RFLC      BIT(28)
#define ASCLIN_FLAGSCLEAR_TFOC      BIT(30)
#define ASCLIN_FLAGSCLEAR_TFLC      BIT(31)

#define ASCLIN_FLAGSCLEAR_ALL_ERRS  (ASCLIN_FLAGSCLEAR_PEC  | \
                                     ASCLIN_FLAGSCLEAR_FEC  | \
                                     ASCLIN_FLAGSCLEAR_BDC  | \
                                     ASCLIN_FLAGSCLEAR_CEC  | \
                                     ASCLIN_FLAGSCLEAR_RFOC | \
                                     ASCLIN_FLAGSCLEAR_RFUC | \
                                     ASCLIN_FLAGSCLEAR_TFOC)

#define ASCLIN_FLAGSCLEAR_ALL       (ASCLIN_FLAGSCLEAR_THC  | \
                                     ASCLIN_FLAGSCLEAR_TRC  | \
                                     ASCLIN_FLAGSCLEAR_RHC  | \
                                     ASCLIN_FLAGSCLEAR_RRC  | \
                                     ASCLIN_FLAGSCLEAR_FEDC | \
                                     ASCLIN_FLAGSCLEAR_REDC | \
                                     ASCLIN_FLAGSCLEAR_TFEC | \
                                     ASCLIN_FLAGSCLEAR_PEC  | \
                                     ASCLIN_FLAGSCLEAR_TCC  | \
                                     ASCLIN_FLAGSCLEAR_FEC  | \
                                     ASCLIN_FLAGSCLEAR_HTC  | \
                                     ASCLIN_FLAGSCLEAR_RTC  | \
                                     ASCLIN_FLAGSCLEAR_BDC  | \
                                     ASCLIN_FLAGSCLEAR_CEC  | \
                                     ASCLIN_FLAGSCLEAR_RFOC | \
                                     ASCLIN_FLAGSCLEAR_RFUC | \
                                     ASCLIN_FLAGSCLEAR_RFLC | \
                                     ASCLIN_FLAGSCLEAR_TFOC | \
                                     ASCLIN_FLAGSCLEAR_TFLC)

#define ASCLIN_FLAGSSET_THS         BIT(0)
#define ASCLIN_FLAGSSET_TFLC        BIT(31)

#define ASCLIN_FLAGSENABLE_THE      BIT(0)
#define ASCLIN_FLAGSENABLE_TRE      BIT(1)
#define ASCLIN_FLAGSENABLE_RHE      BIT(2)
#define ASCLIN_FLAGSENABLE_RRE      BIT(3)
#define ASCLIN_FLAGSENABLE_FEDE     BIT(5)
#define ASCLIN_FLAGSENABLE_REDE     BIT(6)
#define ASCLIN_FLAGSENABLE_TFEE     BIT(10)
#define ASCLIN_FLAGSENABLE_PEE      BIT(16)
#define ASCLIN_FLAGSENABLE_TCE      BIT(17)
#define ASCLIN_FLAGSENABLE_FEE      BIT(18)
#define ASCLIN_FLAGSENABLE_BDE      BIT(21)
#define ASCLIN_FLAGSENABLE_CEE      BIT(25)
#define ASCLIN_FLAGSENABLE_RFOE     BIT(26)
#define ASCLIN_FLAGSENABLE_RFUE     BIT(27)
#define ASCLIN_FLAGSENABLE_RFLE     BIT(28)
#define ASCLIN_FLAGSENABLE_TFOE     BIT(30)
#define ASCLIN_FLAGSENABLE_TFLE     BIT(31)

#define ASCLIN_CSR_CLKSEL_SHIFT     0
#define ASCLIN_CSR_CLKSEL_MASK      GENMASK(4, 0)
#define ASCLIN_CSR_CON              BIT(31)

#define ASCLIN_CSR_CLKSEL_NONE      0x0
#define ASCLIN_CSR_CLKSEL_FASCLINF  0x2
#define ASCLIN_CSR_CLKSEL_FASCLINS  0x4

#define ASCLIN_FIFO_DEPTH           16

#define ASCLIN_TX_FILL(txfifocon)   (((txfifocon) & ASCLIN_TXFIFOCON_FILL_MASK) >> \
                                     ASCLIN_TXFIFOCON_FILL_SHIFT)

#define ASCLIN_RX_FILL(rxfifocon)   (((rxfifocon) & ASCLIN_RXFIFOCON_FILL_MASK) >> \
                                     ASCLIN_RXFIFOCON_FILL_SHIFT)

#endif /* __ARCH_TRICORE_SRC_AURIX_TRICORE_ASCLIN_H */
