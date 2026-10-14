/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57l843_pinmux.h
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

/* References:
 * RM57Lx 16/32-Bit RISC Flash Microcontroller, Technical Reference Manual,
 * Texas Instruments, Literature Number: SPNU562
 *
 * The pin function encodings below were converted from TI's HALCoGen
 * output for RM57L843 for the 337-ball ZWT
 * package.  HALCoGen's header supplies only the bit position of each field;
 * the PINMMR register index of every field comes from the assignments in
 * its muxInit().
 *
 * Each definition below describes one function of one ball (or one input
 * signal source, or one special-function setting) as the triple needed by
 * struct rm57_pinmux_s:
 *
 *   _PINMMR  Index of the PINMMR register holding the field
 *   _SHIFT   Bit position of the field within that register
 *   _VALUE   Field value selecting this function
 *   _PIN     Brace initializer combining the three
 *
 * A ball is left in its reset (default) function unless a definition for
 * one of its alternate functions is placed in the board pin-mux table, so
 * only the pins a board actually re-purposes need to be listed there.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_PINMUX_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Output Pin Multiplexing (PINMMR0-37) *************************************/

/* One 8-bit field per multiplexed ball.  The functions available on each
 * ball are listed above its definitions, ordered by field value; the
 * function with value 0x01 is the reset default for most balls.
 */

/* PINMMR0: N19[0], D4[8], D5[16], C4[24] */

/* Ball N19: AD1EVT (0x01), MII_RX_ER (0x04), RMII_RX_ER (0x08), nTZ1_1
 * (0x20)
 */

#define PINMUX_N19_AD1EVT_PINMMR           0
#define PINMUX_N19_AD1EVT_SHIFT            0
#define PINMUX_N19_AD1EVT_VALUE            0x01
#define PINMUX_N19_AD1EVT_PIN              {0, 0, 0x01}

#define PINMUX_N19_MII_RX_ER_PINMMR        0
#define PINMUX_N19_MII_RX_ER_SHIFT         0
#define PINMUX_N19_MII_RX_ER_VALUE         0x04
#define PINMUX_N19_MII_RX_ER_PIN           {0, 0, 0x04}

#define PINMUX_N19_RMII_RX_ER_PINMMR       0
#define PINMUX_N19_RMII_RX_ER_SHIFT        0
#define PINMUX_N19_RMII_RX_ER_VALUE        0x08
#define PINMUX_N19_RMII_RX_ER_PIN          {0, 0, 0x08}

#define PINMUX_N19_NTZ1_1_PINMMR           0
#define PINMUX_N19_NTZ1_1_SHIFT            0
#define PINMUX_N19_NTZ1_1_VALUE            0x20
#define PINMUX_N19_NTZ1_1_PIN              {0, 0, 0x20}

/* Ball D4: EMIF_ADDR_00 (0x01), N2HET2_01 (0x04) */

#define PINMUX_D4_EMIF_ADDR_00_PINMMR      0
#define PINMUX_D4_EMIF_ADDR_00_SHIFT       8
#define PINMUX_D4_EMIF_ADDR_00_VALUE       0x01
#define PINMUX_D4_EMIF_ADDR_00_PIN         {0, 8, 0x01}

#define PINMUX_D4_N2HET2_01_PINMMR         0
#define PINMUX_D4_N2HET2_01_SHIFT          8
#define PINMUX_D4_N2HET2_01_VALUE          0x04
#define PINMUX_D4_N2HET2_01_PIN            {0, 8, 0x04}

/* Ball D5: EMIF_ADDR_01 (0x01), N2HET2_03 (0x04) */

#define PINMUX_D5_EMIF_ADDR_01_PINMMR      0
#define PINMUX_D5_EMIF_ADDR_01_SHIFT       16
#define PINMUX_D5_EMIF_ADDR_01_VALUE       0x01
#define PINMUX_D5_EMIF_ADDR_01_PIN         {0, 16, 0x01}

#define PINMUX_D5_N2HET2_03_PINMMR         0
#define PINMUX_D5_N2HET2_03_SHIFT          16
#define PINMUX_D5_N2HET2_03_VALUE          0x04
#define PINMUX_D5_N2HET2_03_PIN            {0, 16, 0x04}

/* Ball C4: EMIF_ADDR_06 (0x01), RTP_DATA_13 (0x02), N2HET2_11 (0x04) */

#define PINMUX_C4_EMIF_ADDR_06_PINMMR      0
#define PINMUX_C4_EMIF_ADDR_06_SHIFT       24
#define PINMUX_C4_EMIF_ADDR_06_VALUE       0x01
#define PINMUX_C4_EMIF_ADDR_06_PIN         {0, 24, 0x01}

#define PINMUX_C4_RTP_DATA_13_PINMMR       0
#define PINMUX_C4_RTP_DATA_13_SHIFT        24
#define PINMUX_C4_RTP_DATA_13_VALUE        0x02
#define PINMUX_C4_RTP_DATA_13_PIN          {0, 24, 0x02}

#define PINMUX_C4_N2HET2_11_PINMMR         0
#define PINMUX_C4_N2HET2_11_SHIFT          24
#define PINMUX_C4_N2HET2_11_VALUE          0x04
#define PINMUX_C4_N2HET2_11_PIN            {0, 24, 0x04}

/* PINMMR1: C5[0], C6[8], C7[16], C8[24] */

/* Ball C5: EMIF_ADDR_07 (0x01), RTP_DATA_12 (0x02), N2HET2_13 (0x04) */

#define PINMUX_C5_EMIF_ADDR_07_PINMMR      1
#define PINMUX_C5_EMIF_ADDR_07_SHIFT       0
#define PINMUX_C5_EMIF_ADDR_07_VALUE       0x01
#define PINMUX_C5_EMIF_ADDR_07_PIN         {1, 0, 0x01}

#define PINMUX_C5_RTP_DATA_12_PINMMR       1
#define PINMUX_C5_RTP_DATA_12_SHIFT        0
#define PINMUX_C5_RTP_DATA_12_VALUE        0x02
#define PINMUX_C5_RTP_DATA_12_PIN          {1, 0, 0x02}

#define PINMUX_C5_N2HET2_13_PINMMR         1
#define PINMUX_C5_N2HET2_13_SHIFT          0
#define PINMUX_C5_N2HET2_13_VALUE          0x04
#define PINMUX_C5_N2HET2_13_PIN            {1, 0, 0x04}

/* Ball C6: EMIF_ADDR_08 (0x01), RTP_DATA_11 (0x02), N2HET2_15 (0x04) */

#define PINMUX_C6_EMIF_ADDR_08_PINMMR      1
#define PINMUX_C6_EMIF_ADDR_08_SHIFT       8
#define PINMUX_C6_EMIF_ADDR_08_VALUE       0x01
#define PINMUX_C6_EMIF_ADDR_08_PIN         {1, 8, 0x01}

#define PINMUX_C6_RTP_DATA_11_PINMMR       1
#define PINMUX_C6_RTP_DATA_11_SHIFT        8
#define PINMUX_C6_RTP_DATA_11_VALUE        0x02
#define PINMUX_C6_RTP_DATA_11_PIN          {1, 8, 0x02}

#define PINMUX_C6_N2HET2_15_PINMMR         1
#define PINMUX_C6_N2HET2_15_SHIFT          8
#define PINMUX_C6_N2HET2_15_VALUE          0x04
#define PINMUX_C6_N2HET2_15_PIN            {1, 8, 0x04}

/* Ball C7: EMIF_ADDR_09 (0x01), RTP_DATA_10 (0x02) */

#define PINMUX_C7_EMIF_ADDR_09_PINMMR      1
#define PINMUX_C7_EMIF_ADDR_09_SHIFT       16
#define PINMUX_C7_EMIF_ADDR_09_VALUE       0x01
#define PINMUX_C7_EMIF_ADDR_09_PIN         {1, 16, 0x01}

#define PINMUX_C7_RTP_DATA_10_PINMMR       1
#define PINMUX_C7_RTP_DATA_10_SHIFT        16
#define PINMUX_C7_RTP_DATA_10_VALUE        0x02
#define PINMUX_C7_RTP_DATA_10_PIN          {1, 16, 0x02}

/* Ball C8: EMIF_ADDR_10 (0x01), RTP_DATA_09 (0x02) */

#define PINMUX_C8_EMIF_ADDR_10_PINMMR      1
#define PINMUX_C8_EMIF_ADDR_10_SHIFT       24
#define PINMUX_C8_EMIF_ADDR_10_VALUE       0x01
#define PINMUX_C8_EMIF_ADDR_10_PIN         {1, 24, 0x01}

#define PINMUX_C8_RTP_DATA_09_PINMMR       1
#define PINMUX_C8_RTP_DATA_09_SHIFT        24
#define PINMUX_C8_RTP_DATA_09_VALUE        0x02
#define PINMUX_C8_RTP_DATA_09_PIN          {1, 24, 0x02}

/* PINMMR2: C9[0], C10[8], C11[16], C12[24] */

/* Ball C9: EMIF_ADDR_11 (0x01), RTP_DATA_08 (0x02) */

#define PINMUX_C9_EMIF_ADDR_11_PINMMR      2
#define PINMUX_C9_EMIF_ADDR_11_SHIFT       0
#define PINMUX_C9_EMIF_ADDR_11_VALUE       0x01
#define PINMUX_C9_EMIF_ADDR_11_PIN         {2, 0, 0x01}

#define PINMUX_C9_RTP_DATA_08_PINMMR       2
#define PINMUX_C9_RTP_DATA_08_SHIFT        0
#define PINMUX_C9_RTP_DATA_08_VALUE        0x02
#define PINMUX_C9_RTP_DATA_08_PIN          {2, 0, 0x02}

/* Ball C10: EMIF_ADDR_12 (0x01), RTP_DATA_06 (0x02) */

#define PINMUX_C10_EMIF_ADDR_12_PINMMR     2
#define PINMUX_C10_EMIF_ADDR_12_SHIFT      8
#define PINMUX_C10_EMIF_ADDR_12_VALUE      0x01
#define PINMUX_C10_EMIF_ADDR_12_PIN        {2, 8, 0x01}

#define PINMUX_C10_RTP_DATA_06_PINMMR      2
#define PINMUX_C10_RTP_DATA_06_SHIFT       8
#define PINMUX_C10_RTP_DATA_06_VALUE       0x02
#define PINMUX_C10_RTP_DATA_06_PIN         {2, 8, 0x02}

/* Ball C11: EMIF_ADDR_13 (0x01), RTP_DATA_05 (0x02) */

#define PINMUX_C11_EMIF_ADDR_13_PINMMR     2
#define PINMUX_C11_EMIF_ADDR_13_SHIFT      16
#define PINMUX_C11_EMIF_ADDR_13_VALUE      0x01
#define PINMUX_C11_EMIF_ADDR_13_PIN        {2, 16, 0x01}

#define PINMUX_C11_RTP_DATA_05_PINMMR      2
#define PINMUX_C11_RTP_DATA_05_SHIFT       16
#define PINMUX_C11_RTP_DATA_05_VALUE       0x02
#define PINMUX_C11_RTP_DATA_05_PIN         {2, 16, 0x02}

/* Ball C12: EMIF_ADDR_14 (0x01), RTP_DATA_04 (0x02) */

#define PINMUX_C12_EMIF_ADDR_14_PINMMR     2
#define PINMUX_C12_EMIF_ADDR_14_SHIFT      24
#define PINMUX_C12_EMIF_ADDR_14_VALUE      0x01
#define PINMUX_C12_EMIF_ADDR_14_PIN        {2, 24, 0x01}

#define PINMUX_C12_RTP_DATA_04_PINMMR      2
#define PINMUX_C12_RTP_DATA_04_SHIFT       24
#define PINMUX_C12_RTP_DATA_04_VALUE       0x02
#define PINMUX_C12_RTP_DATA_04_PIN         {2, 24, 0x02}

/* PINMMR3: C13[0], D14[8], C14[16], D15[24] */

/* Ball C13: EMIF_ADDR_15 (0x01), RTP_DATA_03 (0x02) */

#define PINMUX_C13_EMIF_ADDR_15_PINMMR     3
#define PINMUX_C13_EMIF_ADDR_15_SHIFT      0
#define PINMUX_C13_EMIF_ADDR_15_VALUE      0x01
#define PINMUX_C13_EMIF_ADDR_15_PIN        {3, 0, 0x01}

#define PINMUX_C13_RTP_DATA_03_PINMMR      3
#define PINMUX_C13_RTP_DATA_03_SHIFT       0
#define PINMUX_C13_RTP_DATA_03_VALUE       0x02
#define PINMUX_C13_RTP_DATA_03_PIN         {3, 0, 0x02}

/* Ball D14: EMIF_ADDR_16 (0x01), RTP_DATA_02 (0x02) */

#define PINMUX_D14_EMIF_ADDR_16_PINMMR     3
#define PINMUX_D14_EMIF_ADDR_16_SHIFT      8
#define PINMUX_D14_EMIF_ADDR_16_VALUE      0x01
#define PINMUX_D14_EMIF_ADDR_16_PIN        {3, 8, 0x01}

#define PINMUX_D14_RTP_DATA_02_PINMMR      3
#define PINMUX_D14_RTP_DATA_02_SHIFT       8
#define PINMUX_D14_RTP_DATA_02_VALUE       0x02
#define PINMUX_D14_RTP_DATA_02_PIN         {3, 8, 0x02}

/* Ball C14: EMIF_ADDR_17 (0x01), RTP_DATA_01 (0x02) */

#define PINMUX_C14_EMIF_ADDR_17_PINMMR     3
#define PINMUX_C14_EMIF_ADDR_17_SHIFT      16
#define PINMUX_C14_EMIF_ADDR_17_VALUE      0x01
#define PINMUX_C14_EMIF_ADDR_17_PIN        {3, 16, 0x01}

#define PINMUX_C14_RTP_DATA_01_PINMMR      3
#define PINMUX_C14_RTP_DATA_01_SHIFT       16
#define PINMUX_C14_RTP_DATA_01_VALUE       0x02
#define PINMUX_C14_RTP_DATA_01_PIN         {3, 16, 0x02}

/* Ball D15: EMIF_ADDR_18 (0x01), RTP_DATA_00 (0x02) */

#define PINMUX_D15_EMIF_ADDR_18_PINMMR     3
#define PINMUX_D15_EMIF_ADDR_18_SHIFT      24
#define PINMUX_D15_EMIF_ADDR_18_VALUE      0x01
#define PINMUX_D15_EMIF_ADDR_18_PIN        {3, 24, 0x01}

#define PINMUX_D15_RTP_DATA_00_PINMMR      3
#define PINMUX_D15_RTP_DATA_00_SHIFT       24
#define PINMUX_D15_RTP_DATA_00_VALUE       0x02
#define PINMUX_D15_RTP_DATA_00_PIN         {3, 24, 0x02}

/* PINMMR4: C15[0], C16[8], C17[16] */

/* Ball C15: EMIF_ADDR_19 (0x01), RTP_nENA (0x02) */

#define PINMUX_C15_EMIF_ADDR_19_PINMMR     4
#define PINMUX_C15_EMIF_ADDR_19_SHIFT      0
#define PINMUX_C15_EMIF_ADDR_19_VALUE      0x01
#define PINMUX_C15_EMIF_ADDR_19_PIN        {4, 0, 0x01}

#define PINMUX_C15_RTP_NENA_PINMMR         4
#define PINMUX_C15_RTP_NENA_SHIFT          0
#define PINMUX_C15_RTP_NENA_VALUE          0x02
#define PINMUX_C15_RTP_NENA_PIN            {4, 0, 0x02}

/* Ball C16: EMIF_ADDR_20 (0x01), RTP_nSYNC (0x02) */

#define PINMUX_C16_EMIF_ADDR_20_PINMMR     4
#define PINMUX_C16_EMIF_ADDR_20_SHIFT      8
#define PINMUX_C16_EMIF_ADDR_20_VALUE      0x01
#define PINMUX_C16_EMIF_ADDR_20_PIN        {4, 8, 0x01}

#define PINMUX_C16_RTP_NSYNC_PINMMR        4
#define PINMUX_C16_RTP_NSYNC_SHIFT         8
#define PINMUX_C16_RTP_NSYNC_VALUE         0x02
#define PINMUX_C16_RTP_NSYNC_PIN           {4, 8, 0x02}

/* Ball C17: EMIF_ADDR_21 (0x01), RTP_CLK (0x02) */

#define PINMUX_C17_EMIF_ADDR_21_PINMMR     4
#define PINMUX_C17_EMIF_ADDR_21_SHIFT      16
#define PINMUX_C17_EMIF_ADDR_21_VALUE      0x01
#define PINMUX_C17_EMIF_ADDR_21_PIN        {4, 16, 0x01}

#define PINMUX_C17_RTP_CLK_PINMMR          4
#define PINMUX_C17_RTP_CLK_SHIFT           16
#define PINMUX_C17_RTP_CLK_VALUE           0x02
#define PINMUX_C17_RTP_CLK_PIN             {4, 16, 0x02}

/* PINMMR8: D16[24] */

/* Ball D16: EMIF_BA_1 (0x01), 8_25 (0x02), N2HET2_05 (0x04) */

#define PINMUX_D16_EMIF_BA_1_PINMMR        8
#define PINMUX_D16_EMIF_BA_1_SHIFT         24
#define PINMUX_D16_EMIF_BA_1_VALUE         0x01
#define PINMUX_D16_EMIF_BA_1_PIN           {8, 24, 0x01}

#define PINMUX_D16_8_25_PINMMR             8
#define PINMUX_D16_8_25_SHIFT              24
#define PINMUX_D16_8_25_VALUE              0x02
#define PINMUX_D16_8_25_PIN                {8, 24, 0x02}

#define PINMUX_D16_N2HET2_05_PINMMR        8
#define PINMUX_D16_N2HET2_05_SHIFT         24
#define PINMUX_D16_N2HET2_05_VALUE         0x04
#define PINMUX_D16_N2HET2_05_PIN           {8, 24, 0x04}

/* PINMMR9: K3[0], R4[8], N17[16], L17[24] */

/* Ball K3: RESERVED (0x01), EMIF_CLK (0x02), ECLK2 (0x04) */

#define PINMUX_K3_RESERVED_PINMMR          9
#define PINMUX_K3_RESERVED_SHIFT           0
#define PINMUX_K3_RESERVED_VALUE           0x01
#define PINMUX_K3_RESERVED_PIN             {9, 0, 0x01}

#define PINMUX_K3_EMIF_CLK_PINMMR          9
#define PINMUX_K3_EMIF_CLK_SHIFT           0
#define PINMUX_K3_EMIF_CLK_VALUE           0x02
#define PINMUX_K3_EMIF_CLK_PIN             {9, 0, 0x02}

#define PINMUX_K3_ECLK2_PINMMR             9
#define PINMUX_K3_ECLK2_SHIFT              0
#define PINMUX_K3_ECLK2_VALUE              0x04
#define PINMUX_K3_ECLK2_PIN                {9, 0, 0x04}

/* Ball R4: EMIF_nCAS (0x01), GIOB_3 (0x04) */

#define PINMUX_R4_EMIF_NCAS_PINMMR         9
#define PINMUX_R4_EMIF_NCAS_SHIFT          8
#define PINMUX_R4_EMIF_NCAS_VALUE          0x01
#define PINMUX_R4_EMIF_NCAS_PIN            {9, 8, 0x01}

#define PINMUX_R4_GIOB_3_PINMMR            9
#define PINMUX_R4_GIOB_3_SHIFT             8
#define PINMUX_R4_GIOB_3_VALUE             0x04
#define PINMUX_R4_GIOB_3_PIN               {9, 8, 0x04}

/* Ball N17: EMIF_nCS_0 (0x01), RTP_DATA_15 (0x02), N2HET2_07 (0x04) */

#define PINMUX_N17_EMIF_NCS_0_PINMMR       9
#define PINMUX_N17_EMIF_NCS_0_SHIFT        16
#define PINMUX_N17_EMIF_NCS_0_VALUE        0x01
#define PINMUX_N17_EMIF_NCS_0_PIN          {9, 16, 0x01}

#define PINMUX_N17_RTP_DATA_15_PINMMR      9
#define PINMUX_N17_RTP_DATA_15_SHIFT       16
#define PINMUX_N17_RTP_DATA_15_VALUE       0x02
#define PINMUX_N17_RTP_DATA_15_PIN         {9, 16, 0x02}

#define PINMUX_N17_N2HET2_07_PINMMR        9
#define PINMUX_N17_N2HET2_07_SHIFT         16
#define PINMUX_N17_N2HET2_07_VALUE         0x04
#define PINMUX_N17_N2HET2_07_PIN           {9, 16, 0x04}

/* Ball L17: EMIF_nCS_2 (0x01), GIOB_4 (0x04) */

#define PINMUX_L17_EMIF_NCS_2_PINMMR       9
#define PINMUX_L17_EMIF_NCS_2_SHIFT        24
#define PINMUX_L17_EMIF_NCS_2_VALUE        0x01
#define PINMUX_L17_EMIF_NCS_2_PIN          {9, 24, 0x01}

#define PINMUX_L17_GIOB_4_PINMMR           9
#define PINMUX_L17_GIOB_4_SHIFT            24
#define PINMUX_L17_GIOB_4_VALUE            0x04
#define PINMUX_L17_GIOB_4_PIN              {9, 24, 0x04}

/* PINMMR10: K17[0], M17[8], R3[16], P3[24] */

/* Ball K17: EMIF_nCS_3 (0x01), RTP_DATA_14 (0x02), N2HET2_09 (0x04) */

#define PINMUX_K17_EMIF_NCS_3_PINMMR       10
#define PINMUX_K17_EMIF_NCS_3_SHIFT        0
#define PINMUX_K17_EMIF_NCS_3_VALUE        0x01
#define PINMUX_K17_EMIF_NCS_3_PIN          {10, 0, 0x01}

#define PINMUX_K17_RTP_DATA_14_PINMMR      10
#define PINMUX_K17_RTP_DATA_14_SHIFT       0
#define PINMUX_K17_RTP_DATA_14_VALUE       0x02
#define PINMUX_K17_RTP_DATA_14_PIN         {10, 0, 0x02}

#define PINMUX_K17_N2HET2_09_PINMMR        10
#define PINMUX_K17_N2HET2_09_SHIFT         0
#define PINMUX_K17_N2HET2_09_VALUE         0x04
#define PINMUX_K17_N2HET2_09_PIN           {10, 0, 0x04}

/* Ball M17: EMIF_nCS_4 (0x01), RTP_DATA_07 (0x02), GIOB_5 (0x04) */

#define PINMUX_M17_EMIF_NCS_4_PINMMR       10
#define PINMUX_M17_EMIF_NCS_4_SHIFT        8
#define PINMUX_M17_EMIF_NCS_4_VALUE        0x01
#define PINMUX_M17_EMIF_NCS_4_PIN          {10, 8, 0x01}

#define PINMUX_M17_RTP_DATA_07_PINMMR      10
#define PINMUX_M17_RTP_DATA_07_SHIFT       8
#define PINMUX_M17_RTP_DATA_07_VALUE       0x02
#define PINMUX_M17_RTP_DATA_07_PIN         {10, 8, 0x02}

#define PINMUX_M17_GIOB_5_PINMMR           10
#define PINMUX_M17_GIOB_5_SHIFT            8
#define PINMUX_M17_GIOB_5_VALUE            0x04
#define PINMUX_M17_GIOB_5_PIN              {10, 8, 0x04}

/* Ball R3: EMIF_nRAS (0x01), GIOB_6 (0x04) */

#define PINMUX_R3_EMIF_NRAS_PINMMR         10
#define PINMUX_R3_EMIF_NRAS_SHIFT          16
#define PINMUX_R3_EMIF_NRAS_VALUE          0x01
#define PINMUX_R3_EMIF_NRAS_PIN            {10, 16, 0x01}

#define PINMUX_R3_GIOB_6_PINMMR            10
#define PINMUX_R3_GIOB_6_SHIFT             16
#define PINMUX_R3_GIOB_6_VALUE             0x04
#define PINMUX_R3_GIOB_6_PIN               {10, 16, 0x04}

/* Ball P3: EMIF_nWAIT (0x01), GIOB_7 (0x04) */

#define PINMUX_P3_EMIF_NWAIT_PINMMR        10
#define PINMUX_P3_EMIF_NWAIT_SHIFT         24
#define PINMUX_P3_EMIF_NWAIT_VALUE         0x01
#define PINMUX_P3_EMIF_NWAIT_PIN           {10, 24, 0x01}

#define PINMUX_P3_GIOB_7_PINMMR            10
#define PINMUX_P3_GIOB_7_SHIFT             24
#define PINMUX_P3_GIOB_7_VALUE             0x04
#define PINMUX_P3_GIOB_7_PIN               {10, 24, 0x04}

/* PINMMR11: D17[0], E9[8], E8[16], E7[24] */

/* Ball D17: EMIF_nWE (0x01), EMIF_RNW (0x02) */

#define PINMUX_D17_EMIF_NWE_PINMMR         11
#define PINMUX_D17_EMIF_NWE_SHIFT          0
#define PINMUX_D17_EMIF_NWE_VALUE          0x01
#define PINMUX_D17_EMIF_NWE_PIN            {11, 0, 0x01}

#define PINMUX_D17_EMIF_RNW_PINMMR         11
#define PINMUX_D17_EMIF_RNW_SHIFT          0
#define PINMUX_D17_EMIF_RNW_VALUE          0x02
#define PINMUX_D17_EMIF_RNW_PIN            {11, 0, 0x02}

/* Ball E9: ETMDATA_08 (0x01), EMIF_ADDR_05 (0x02) */

#define PINMUX_E9_ETMDATA_08_PINMMR        11
#define PINMUX_E9_ETMDATA_08_SHIFT         8
#define PINMUX_E9_ETMDATA_08_VALUE         0x01
#define PINMUX_E9_ETMDATA_08_PIN           {11, 8, 0x01}

#define PINMUX_E9_EMIF_ADDR_05_PINMMR      11
#define PINMUX_E9_EMIF_ADDR_05_SHIFT       8
#define PINMUX_E9_EMIF_ADDR_05_VALUE       0x02
#define PINMUX_E9_EMIF_ADDR_05_PIN         {11, 8, 0x02}

/* Ball E8: ETMDATA_09 (0x01), EMIF_ADDR_04 (0x02) */

#define PINMUX_E8_ETMDATA_09_PINMMR        11
#define PINMUX_E8_ETMDATA_09_SHIFT         16
#define PINMUX_E8_ETMDATA_09_VALUE         0x01
#define PINMUX_E8_ETMDATA_09_PIN           {11, 16, 0x01}

#define PINMUX_E8_EMIF_ADDR_04_PINMMR      11
#define PINMUX_E8_EMIF_ADDR_04_SHIFT       16
#define PINMUX_E8_EMIF_ADDR_04_VALUE       0x02
#define PINMUX_E8_EMIF_ADDR_04_PIN         {11, 16, 0x02}

/* Ball E7: ETMDATA_10 (0x01), EMIF_ADDR_03 (0x02) */

#define PINMUX_E7_ETMDATA_10_PINMMR        11
#define PINMUX_E7_ETMDATA_10_SHIFT         24
#define PINMUX_E7_ETMDATA_10_VALUE         0x01
#define PINMUX_E7_ETMDATA_10_PIN           {11, 24, 0x01}

#define PINMUX_E7_EMIF_ADDR_03_PINMMR      11
#define PINMUX_E7_EMIF_ADDR_03_SHIFT       24
#define PINMUX_E7_EMIF_ADDR_03_VALUE       0x02
#define PINMUX_E7_EMIF_ADDR_03_PIN         {11, 24, 0x02}

/* PINMMR12: E6[0], E13[8], E12[16], E11[24] */

/* Ball E6: ETMDATA_11 (0x01), EMIF_ADDR_02 (0x02) */

#define PINMUX_E6_ETMDATA_11_PINMMR        12
#define PINMUX_E6_ETMDATA_11_SHIFT         0
#define PINMUX_E6_ETMDATA_11_VALUE         0x01
#define PINMUX_E6_ETMDATA_11_PIN           {12, 0, 0x01}

#define PINMUX_E6_EMIF_ADDR_02_PINMMR      12
#define PINMUX_E6_EMIF_ADDR_02_SHIFT       0
#define PINMUX_E6_EMIF_ADDR_02_VALUE       0x02
#define PINMUX_E6_EMIF_ADDR_02_PIN         {12, 0, 0x02}

/* Ball E13: ETMDATA_12 (0x01), EMIF_BA_0 (0x02) */

#define PINMUX_E13_ETMDATA_12_PINMMR       12
#define PINMUX_E13_ETMDATA_12_SHIFT        8
#define PINMUX_E13_ETMDATA_12_VALUE        0x01
#define PINMUX_E13_ETMDATA_12_PIN          {12, 8, 0x01}

#define PINMUX_E13_EMIF_BA_0_PINMMR        12
#define PINMUX_E13_EMIF_BA_0_SHIFT         8
#define PINMUX_E13_EMIF_BA_0_VALUE         0x02
#define PINMUX_E13_EMIF_BA_0_PIN           {12, 8, 0x02}

/* Ball E12: ETMDATA_13 (0x01), EMIF_nOE (0x02) */

#define PINMUX_E12_ETMDATA_13_PINMMR       12
#define PINMUX_E12_ETMDATA_13_SHIFT        16
#define PINMUX_E12_ETMDATA_13_VALUE        0x01
#define PINMUX_E12_ETMDATA_13_PIN          {12, 16, 0x01}

#define PINMUX_E12_EMIF_NOE_PINMMR         12
#define PINMUX_E12_EMIF_NOE_SHIFT          16
#define PINMUX_E12_EMIF_NOE_VALUE          0x02
#define PINMUX_E12_EMIF_NOE_PIN            {12, 16, 0x02}

/* Ball E11: ETMDATA_14 (0x01), EMIF_nDQM_1 (0x02) */

#define PINMUX_E11_ETMDATA_14_PINMMR       12
#define PINMUX_E11_ETMDATA_14_SHIFT        24
#define PINMUX_E11_ETMDATA_14_VALUE        0x01
#define PINMUX_E11_ETMDATA_14_PIN          {12, 24, 0x01}

#define PINMUX_E11_EMIF_NDQM_1_PINMMR      12
#define PINMUX_E11_EMIF_NDQM_1_SHIFT       24
#define PINMUX_E11_EMIF_NDQM_1_VALUE       0x02
#define PINMUX_E11_EMIF_NDQM_1_PIN         {12, 24, 0x02}

/* PINMMR13: E10[0], K15[8], L15[16], M15[24] */

/* Ball E10: ETMDATA_15 (0x01), EMIF_nDQM_0 (0x02) */

#define PINMUX_E10_ETMDATA_15_PINMMR       13
#define PINMUX_E10_ETMDATA_15_SHIFT        0
#define PINMUX_E10_ETMDATA_15_VALUE        0x01
#define PINMUX_E10_ETMDATA_15_PIN          {13, 0, 0x01}

#define PINMUX_E10_EMIF_NDQM_0_PINMMR      13
#define PINMUX_E10_EMIF_NDQM_0_SHIFT       0
#define PINMUX_E10_EMIF_NDQM_0_VALUE       0x02
#define PINMUX_E10_EMIF_NDQM_0_PIN         {13, 0, 0x02}

/* Ball K15: ETMDATA_16 (0x01), EMIF_DATA_00 (0x02) */

#define PINMUX_K15_ETMDATA_16_PINMMR       13
#define PINMUX_K15_ETMDATA_16_SHIFT        8
#define PINMUX_K15_ETMDATA_16_VALUE        0x01
#define PINMUX_K15_ETMDATA_16_PIN          {13, 8, 0x01}

#define PINMUX_K15_EMIF_DATA_00_PINMMR     13
#define PINMUX_K15_EMIF_DATA_00_SHIFT      8
#define PINMUX_K15_EMIF_DATA_00_VALUE      0x02
#define PINMUX_K15_EMIF_DATA_00_PIN        {13, 8, 0x02}

/* Ball L15: ETMDATA_17 (0x01), EMIF_DATA_01 (0x02) */

#define PINMUX_L15_ETMDATA_17_PINMMR       13
#define PINMUX_L15_ETMDATA_17_SHIFT        16
#define PINMUX_L15_ETMDATA_17_VALUE        0x01
#define PINMUX_L15_ETMDATA_17_PIN          {13, 16, 0x01}

#define PINMUX_L15_EMIF_DATA_01_PINMMR     13
#define PINMUX_L15_EMIF_DATA_01_SHIFT      16
#define PINMUX_L15_EMIF_DATA_01_VALUE      0x02
#define PINMUX_L15_EMIF_DATA_01_PIN        {13, 16, 0x02}

/* Ball M15: ETMDATA_18 (0x01), EMIF_DATA_02 (0x02) */

#define PINMUX_M15_ETMDATA_18_PINMMR       13
#define PINMUX_M15_ETMDATA_18_SHIFT        24
#define PINMUX_M15_ETMDATA_18_VALUE        0x01
#define PINMUX_M15_ETMDATA_18_PIN          {13, 24, 0x01}

#define PINMUX_M15_EMIF_DATA_02_PINMMR     13
#define PINMUX_M15_EMIF_DATA_02_SHIFT      24
#define PINMUX_M15_EMIF_DATA_02_VALUE      0x02
#define PINMUX_M15_EMIF_DATA_02_PIN        {13, 24, 0x02}

/* PINMMR14: N15[0], E5[8], F5[16], G5[24] */

/* Ball N15: ETMDATA_19 (0x01), EMIF_DATA_03 (0x02) */

#define PINMUX_N15_ETMDATA_19_PINMMR       14
#define PINMUX_N15_ETMDATA_19_SHIFT        0
#define PINMUX_N15_ETMDATA_19_VALUE        0x01
#define PINMUX_N15_ETMDATA_19_PIN          {14, 0, 0x01}

#define PINMUX_N15_EMIF_DATA_03_PINMMR     14
#define PINMUX_N15_EMIF_DATA_03_SHIFT      0
#define PINMUX_N15_EMIF_DATA_03_VALUE      0x02
#define PINMUX_N15_EMIF_DATA_03_PIN        {14, 0, 0x02}

/* Ball E5: ETMDATA_20 (0x01), EMIF_DATA_04 (0x02) */

#define PINMUX_E5_ETMDATA_20_PINMMR        14
#define PINMUX_E5_ETMDATA_20_SHIFT         8
#define PINMUX_E5_ETMDATA_20_VALUE         0x01
#define PINMUX_E5_ETMDATA_20_PIN           {14, 8, 0x01}

#define PINMUX_E5_EMIF_DATA_04_PINMMR      14
#define PINMUX_E5_EMIF_DATA_04_SHIFT       8
#define PINMUX_E5_EMIF_DATA_04_VALUE       0x02
#define PINMUX_E5_EMIF_DATA_04_PIN         {14, 8, 0x02}

/* Ball F5: ETMDATA_21 (0x01), EMIF_DATA_05 (0x02) */

#define PINMUX_F5_ETMDATA_21_PINMMR        14
#define PINMUX_F5_ETMDATA_21_SHIFT         16
#define PINMUX_F5_ETMDATA_21_VALUE         0x01
#define PINMUX_F5_ETMDATA_21_PIN           {14, 16, 0x01}

#define PINMUX_F5_EMIF_DATA_05_PINMMR      14
#define PINMUX_F5_EMIF_DATA_05_SHIFT       16
#define PINMUX_F5_EMIF_DATA_05_VALUE       0x02
#define PINMUX_F5_EMIF_DATA_05_PIN         {14, 16, 0x02}

/* Ball G5: ETMDATA_22 (0x01), EMIF_DATA_06 (0x02) */

#define PINMUX_G5_ETMDATA_22_PINMMR        14
#define PINMUX_G5_ETMDATA_22_SHIFT         24
#define PINMUX_G5_ETMDATA_22_VALUE         0x01
#define PINMUX_G5_ETMDATA_22_PIN           {14, 24, 0x01}

#define PINMUX_G5_EMIF_DATA_06_PINMMR      14
#define PINMUX_G5_EMIF_DATA_06_SHIFT       24
#define PINMUX_G5_EMIF_DATA_06_VALUE       0x02
#define PINMUX_G5_EMIF_DATA_06_PIN         {14, 24, 0x02}

/* PINMMR15: K5[0], L5[8], M5[16], N5[24] */

/* Ball K5: ETMDATA_23 (0x01), EMIF_DATA_07 (0x02) */

#define PINMUX_K5_ETMDATA_23_PINMMR        15
#define PINMUX_K5_ETMDATA_23_SHIFT         0
#define PINMUX_K5_ETMDATA_23_VALUE         0x01
#define PINMUX_K5_ETMDATA_23_PIN           {15, 0, 0x01}

#define PINMUX_K5_EMIF_DATA_07_PINMMR      15
#define PINMUX_K5_EMIF_DATA_07_SHIFT       0
#define PINMUX_K5_EMIF_DATA_07_VALUE       0x02
#define PINMUX_K5_EMIF_DATA_07_PIN         {15, 0, 0x02}

/* Ball L5: ETMDATA_24 (0x01), EMIF_DATA_08 (0x02), N2HET2_24 (0x04),
 * MIBSPI5NCS_4 (0x08)
 */

#define PINMUX_L5_ETMDATA_24_PINMMR        15
#define PINMUX_L5_ETMDATA_24_SHIFT         8
#define PINMUX_L5_ETMDATA_24_VALUE         0x01
#define PINMUX_L5_ETMDATA_24_PIN           {15, 8, 0x01}

#define PINMUX_L5_EMIF_DATA_08_PINMMR      15
#define PINMUX_L5_EMIF_DATA_08_SHIFT       8
#define PINMUX_L5_EMIF_DATA_08_VALUE       0x02
#define PINMUX_L5_EMIF_DATA_08_PIN         {15, 8, 0x02}

#define PINMUX_L5_N2HET2_24_PINMMR         15
#define PINMUX_L5_N2HET2_24_SHIFT          8
#define PINMUX_L5_N2HET2_24_VALUE          0x04
#define PINMUX_L5_N2HET2_24_PIN            {15, 8, 0x04}

#define PINMUX_L5_MIBSPI5NCS_4_PINMMR      15
#define PINMUX_L5_MIBSPI5NCS_4_SHIFT       8
#define PINMUX_L5_MIBSPI5NCS_4_VALUE       0x08
#define PINMUX_L5_MIBSPI5NCS_4_PIN         {15, 8, 0x08}

/* Ball M5: ETMDATA_25 (0x01), EMIF_DATA_09 (0x02), N2HET2_25 (0x04),
 * MIBSPI5NCS_5 (0x08)
 */

#define PINMUX_M5_ETMDATA_25_PINMMR        15
#define PINMUX_M5_ETMDATA_25_SHIFT         16
#define PINMUX_M5_ETMDATA_25_VALUE         0x01
#define PINMUX_M5_ETMDATA_25_PIN           {15, 16, 0x01}

#define PINMUX_M5_EMIF_DATA_09_PINMMR      15
#define PINMUX_M5_EMIF_DATA_09_SHIFT       16
#define PINMUX_M5_EMIF_DATA_09_VALUE       0x02
#define PINMUX_M5_EMIF_DATA_09_PIN         {15, 16, 0x02}

#define PINMUX_M5_N2HET2_25_PINMMR         15
#define PINMUX_M5_N2HET2_25_SHIFT          16
#define PINMUX_M5_N2HET2_25_VALUE          0x04
#define PINMUX_M5_N2HET2_25_PIN            {15, 16, 0x04}

#define PINMUX_M5_MIBSPI5NCS_5_PINMMR      15
#define PINMUX_M5_MIBSPI5NCS_5_SHIFT       16
#define PINMUX_M5_MIBSPI5NCS_5_VALUE       0x08
#define PINMUX_M5_MIBSPI5NCS_5_PIN         {15, 16, 0x08}

/* Ball N5: ETMDATA_26 (0x01), EMIF_DATA_10 (0x02), N2HET2_26 (0x04) */

#define PINMUX_N5_ETMDATA_26_PINMMR        15
#define PINMUX_N5_ETMDATA_26_SHIFT         24
#define PINMUX_N5_ETMDATA_26_VALUE         0x01
#define PINMUX_N5_ETMDATA_26_PIN           {15, 24, 0x01}

#define PINMUX_N5_EMIF_DATA_10_PINMMR      15
#define PINMUX_N5_EMIF_DATA_10_SHIFT       24
#define PINMUX_N5_EMIF_DATA_10_VALUE       0x02
#define PINMUX_N5_EMIF_DATA_10_PIN         {15, 24, 0x02}

#define PINMUX_N5_N2HET2_26_PINMMR         15
#define PINMUX_N5_N2HET2_26_SHIFT          24
#define PINMUX_N5_N2HET2_26_VALUE          0x04
#define PINMUX_N5_N2HET2_26_PIN            {15, 24, 0x04}

/* PINMMR16: P5[0], R5[8], R6[16], R7[24] */

/* Ball P5: ETMDATA_27 (0x01), EMIF_DATA_11 (0x02), N2HET2_27 (0x04) */

#define PINMUX_P5_ETMDATA_27_PINMMR        16
#define PINMUX_P5_ETMDATA_27_SHIFT         0
#define PINMUX_P5_ETMDATA_27_VALUE         0x01
#define PINMUX_P5_ETMDATA_27_PIN           {16, 0, 0x01}

#define PINMUX_P5_EMIF_DATA_11_PINMMR      16
#define PINMUX_P5_EMIF_DATA_11_SHIFT       0
#define PINMUX_P5_EMIF_DATA_11_VALUE       0x02
#define PINMUX_P5_EMIF_DATA_11_PIN         {16, 0, 0x02}

#define PINMUX_P5_N2HET2_27_PINMMR         16
#define PINMUX_P5_N2HET2_27_SHIFT          0
#define PINMUX_P5_N2HET2_27_VALUE          0x04
#define PINMUX_P5_N2HET2_27_PIN            {16, 0, 0x04}

/* Ball R5: ETMDATA_28 (0x01), EMIF_DATA_12 (0x02), N2HET2_28 (0x04), GIOA_0
 * (0x08)
 */

#define PINMUX_R5_ETMDATA_28_PINMMR        16
#define PINMUX_R5_ETMDATA_28_SHIFT         8
#define PINMUX_R5_ETMDATA_28_VALUE         0x01
#define PINMUX_R5_ETMDATA_28_PIN           {16, 8, 0x01}

#define PINMUX_R5_EMIF_DATA_12_PINMMR      16
#define PINMUX_R5_EMIF_DATA_12_SHIFT       8
#define PINMUX_R5_EMIF_DATA_12_VALUE       0x02
#define PINMUX_R5_EMIF_DATA_12_PIN         {16, 8, 0x02}

#define PINMUX_R5_N2HET2_28_PINMMR         16
#define PINMUX_R5_N2HET2_28_SHIFT          8
#define PINMUX_R5_N2HET2_28_VALUE          0x04
#define PINMUX_R5_N2HET2_28_PIN            {16, 8, 0x04}

#define PINMUX_R5_GIOA_0_PINMMR            16
#define PINMUX_R5_GIOA_0_SHIFT             8
#define PINMUX_R5_GIOA_0_VALUE             0x08
#define PINMUX_R5_GIOA_0_PIN               {16, 8, 0x08}

/* Ball R6: ETMDATA_29 (0x01), EMIF_DATA_13 (0x02), N2HET2_29 (0x04), GIOA_1
 * (0x08)
 */

#define PINMUX_R6_ETMDATA_29_PINMMR        16
#define PINMUX_R6_ETMDATA_29_SHIFT         16
#define PINMUX_R6_ETMDATA_29_VALUE         0x01
#define PINMUX_R6_ETMDATA_29_PIN           {16, 16, 0x01}

#define PINMUX_R6_EMIF_DATA_13_PINMMR      16
#define PINMUX_R6_EMIF_DATA_13_SHIFT       16
#define PINMUX_R6_EMIF_DATA_13_VALUE       0x02
#define PINMUX_R6_EMIF_DATA_13_PIN         {16, 16, 0x02}

#define PINMUX_R6_N2HET2_29_PINMMR         16
#define PINMUX_R6_N2HET2_29_SHIFT          16
#define PINMUX_R6_N2HET2_29_VALUE          0x04
#define PINMUX_R6_N2HET2_29_PIN            {16, 16, 0x04}

#define PINMUX_R6_GIOA_1_PINMMR            16
#define PINMUX_R6_GIOA_1_SHIFT             16
#define PINMUX_R6_GIOA_1_VALUE             0x08
#define PINMUX_R6_GIOA_1_PIN               {16, 16, 0x08}

/* Ball R7: ETMDATA_30 (0x01), EMIF_DATA_14 (0x02), N2HET2_30 (0x04), GIOA_3
 * (0x08)
 */

#define PINMUX_R7_ETMDATA_30_PINMMR        16
#define PINMUX_R7_ETMDATA_30_SHIFT         24
#define PINMUX_R7_ETMDATA_30_VALUE         0x01
#define PINMUX_R7_ETMDATA_30_PIN           {16, 24, 0x01}

#define PINMUX_R7_EMIF_DATA_14_PINMMR      16
#define PINMUX_R7_EMIF_DATA_14_SHIFT       24
#define PINMUX_R7_EMIF_DATA_14_VALUE       0x02
#define PINMUX_R7_EMIF_DATA_14_PIN         {16, 24, 0x02}

#define PINMUX_R7_N2HET2_30_PINMMR         16
#define PINMUX_R7_N2HET2_30_SHIFT          24
#define PINMUX_R7_N2HET2_30_VALUE          0x04
#define PINMUX_R7_N2HET2_30_PIN            {16, 24, 0x04}

#define PINMUX_R7_GIOA_3_PINMMR            16
#define PINMUX_R7_GIOA_3_SHIFT             24
#define PINMUX_R7_GIOA_3_VALUE             0x08
#define PINMUX_R7_GIOA_3_PIN               {16, 24, 0x08}

/* PINMMR17: R8[0], R9[8], R10[16], R11[24] */

/* Ball R8: ETMDATA_31 (0x01), EMIF_DATA_15 (0x02), N2HET2_31 (0x04), GIOA_4
 * (0x08)
 */

#define PINMUX_R8_ETMDATA_31_PINMMR        17
#define PINMUX_R8_ETMDATA_31_SHIFT         0
#define PINMUX_R8_ETMDATA_31_VALUE         0x01
#define PINMUX_R8_ETMDATA_31_PIN           {17, 0, 0x01}

#define PINMUX_R8_EMIF_DATA_15_PINMMR      17
#define PINMUX_R8_EMIF_DATA_15_SHIFT       0
#define PINMUX_R8_EMIF_DATA_15_VALUE       0x02
#define PINMUX_R8_EMIF_DATA_15_PIN         {17, 0, 0x02}

#define PINMUX_R8_N2HET2_31_PINMMR         17
#define PINMUX_R8_N2HET2_31_SHIFT          0
#define PINMUX_R8_N2HET2_31_VALUE          0x04
#define PINMUX_R8_N2HET2_31_PIN            {17, 0, 0x04}

#define PINMUX_R8_GIOA_4_PINMMR            17
#define PINMUX_R8_GIOA_4_SHIFT             0
#define PINMUX_R8_GIOA_4_VALUE             0x08
#define PINMUX_R8_GIOA_4_PIN               {17, 0, 0x08}

/* Ball R9: ETMTRACECLKIN (0x01), EXTCLKIN2 (0x02), GIOA_5 (0x08) */

#define PINMUX_R9_ETMTRACECLKIN_PINMMR     17
#define PINMUX_R9_ETMTRACECLKIN_SHIFT      8
#define PINMUX_R9_ETMTRACECLKIN_VALUE      0x01
#define PINMUX_R9_ETMTRACECLKIN_PIN        {17, 8, 0x01}

#define PINMUX_R9_EXTCLKIN2_PINMMR         17
#define PINMUX_R9_EXTCLKIN2_SHIFT          8
#define PINMUX_R9_EXTCLKIN2_VALUE          0x02
#define PINMUX_R9_EXTCLKIN2_PIN            {17, 8, 0x02}

#define PINMUX_R9_GIOA_5_PINMMR            17
#define PINMUX_R9_GIOA_5_SHIFT             8
#define PINMUX_R9_GIOA_5_VALUE             0x08
#define PINMUX_R9_GIOA_5_PIN               {17, 8, 0x08}

/* Ball R10: ETMTRACECLKOUT (0x01), GIOA_6 (0x08) */

#define PINMUX_R10_ETMTRACECLKOUT_PINMMR   17
#define PINMUX_R10_ETMTRACECLKOUT_SHIFT    16
#define PINMUX_R10_ETMTRACECLKOUT_VALUE    0x01
#define PINMUX_R10_ETMTRACECLKOUT_PIN      {17, 16, 0x01}

#define PINMUX_R10_GIOA_6_PINMMR           17
#define PINMUX_R10_GIOA_6_SHIFT            16
#define PINMUX_R10_GIOA_6_VALUE            0x08
#define PINMUX_R10_GIOA_6_PIN              {17, 16, 0x08}

/* Ball R11: ETMTRACECTL (0x01), GIOA_7 (0x08) */

#define PINMUX_R11_ETMTRACECTL_PINMMR      17
#define PINMUX_R11_ETMTRACECTL_SHIFT       24
#define PINMUX_R11_ETMTRACECTL_VALUE       0x01
#define PINMUX_R11_ETMTRACECTL_PIN         {17, 24, 0x01}

#define PINMUX_R11_GIOA_7_PINMMR           17
#define PINMUX_R11_GIOA_7_SHIFT            24
#define PINMUX_R11_GIOA_7_VALUE            0x08
#define PINMUX_R11_GIOA_7_PIN              {17, 24, 0x08}

/* PINMMR18: B15[0], B8[8], B16[16], B9[24] */

/* Ball B15: FRAYTX1 (0x01), GIOA_2 (0x08) */

#define PINMUX_B15_FRAYTX1_PINMMR          18
#define PINMUX_B15_FRAYTX1_SHIFT           0
#define PINMUX_B15_FRAYTX1_VALUE           0x01
#define PINMUX_B15_FRAYTX1_PIN             {18, 0, 0x01}

#define PINMUX_B15_GIOA_2_PINMMR           18
#define PINMUX_B15_GIOA_2_SHIFT            0
#define PINMUX_B15_GIOA_2_VALUE            0x08
#define PINMUX_B15_GIOA_2_PIN              {18, 0, 0x08}

/* Ball B8: FRAYTX2 (0x01), GIOB_0 (0x08) */

#define PINMUX_B8_FRAYTX2_PINMMR           18
#define PINMUX_B8_FRAYTX2_SHIFT            8
#define PINMUX_B8_FRAYTX2_VALUE            0x01
#define PINMUX_B8_FRAYTX2_PIN              {18, 8, 0x01}

#define PINMUX_B8_GIOB_0_PINMMR            18
#define PINMUX_B8_GIOB_0_SHIFT             8
#define PINMUX_B8_GIOB_0_VALUE             0x08
#define PINMUX_B8_GIOB_0_PIN               {18, 8, 0x08}

/* Ball B16: FRAYTXEN1 (0x01), GIOB_1 (0x08) */

#define PINMUX_B16_FRAYTXEN1_PINMMR        18
#define PINMUX_B16_FRAYTXEN1_SHIFT         16
#define PINMUX_B16_FRAYTXEN1_VALUE         0x01
#define PINMUX_B16_FRAYTXEN1_PIN           {18, 16, 0x01}

#define PINMUX_B16_GIOB_1_PINMMR           18
#define PINMUX_B16_GIOB_1_SHIFT            16
#define PINMUX_B16_GIOB_1_VALUE            0x08
#define PINMUX_B16_GIOB_1_PIN              {18, 16, 0x08}

/* Ball B9: FRAYTXEN2 (0x01), GIOB_2 (0x08) */

#define PINMUX_B9_FRAYTXEN2_PINMMR         18
#define PINMUX_B9_FRAYTXEN2_SHIFT          24
#define PINMUX_B9_FRAYTXEN2_VALUE          0x01
#define PINMUX_B9_FRAYTXEN2_PIN            {18, 24, 0x01}

#define PINMUX_B9_GIOB_2_PINMMR            18
#define PINMUX_B9_GIOB_2_SHIFT             24
#define PINMUX_B9_GIOB_2_VALUE             0x08
#define PINMUX_B9_GIOB_2_PIN               {18, 24, 0x08}

/* PINMMR19: C1[0], E1[8], B5[16], H3[24] */

/* Ball C1: GIOA_2 (0x01), N2HET2_00 (0x04), eQEP2I (0x20) */

#define PINMUX_C1_GIOA_2_PINMMR            19
#define PINMUX_C1_GIOA_2_SHIFT             0
#define PINMUX_C1_GIOA_2_VALUE             0x01
#define PINMUX_C1_GIOA_2_PIN               {19, 0, 0x01}

#define PINMUX_C1_N2HET2_00_PINMMR         19
#define PINMUX_C1_N2HET2_00_SHIFT          0
#define PINMUX_C1_N2HET2_00_VALUE          0x04
#define PINMUX_C1_N2HET2_00_PIN            {19, 0, 0x04}

#define PINMUX_C1_EQEP2I_PINMMR            19
#define PINMUX_C1_EQEP2I_SHIFT             0
#define PINMUX_C1_EQEP2I_VALUE             0x20
#define PINMUX_C1_EQEP2I_PIN               {19, 0, 0x20}

/* Ball E1: GIOA_3 (0x01), N2HET2_02 (0x04) */

#define PINMUX_E1_GIOA_3_PINMMR            19
#define PINMUX_E1_GIOA_3_SHIFT             8
#define PINMUX_E1_GIOA_3_VALUE             0x01
#define PINMUX_E1_GIOA_3_PIN               {19, 8, 0x01}

#define PINMUX_E1_N2HET2_02_PINMMR         19
#define PINMUX_E1_N2HET2_02_SHIFT          8
#define PINMUX_E1_N2HET2_02_VALUE          0x04
#define PINMUX_E1_N2HET2_02_PIN            {19, 8, 0x04}

/* Ball B5: GIOA_5 (0x01), EXTCLKIN (0x08), eTPWM1A (0x20) */

#define PINMUX_B5_GIOA_5_PINMMR            19
#define PINMUX_B5_GIOA_5_SHIFT             16
#define PINMUX_B5_GIOA_5_VALUE             0x01
#define PINMUX_B5_GIOA_5_PIN               {19, 16, 0x01}

#define PINMUX_B5_EXTCLKIN_PINMMR          19
#define PINMUX_B5_EXTCLKIN_SHIFT           16
#define PINMUX_B5_EXTCLKIN_VALUE           0x08
#define PINMUX_B5_EXTCLKIN_PIN             {19, 16, 0x08}

#define PINMUX_B5_ETPWM1A_PINMMR           19
#define PINMUX_B5_ETPWM1A_SHIFT            16
#define PINMUX_B5_ETPWM1A_VALUE            0x20
#define PINMUX_B5_ETPWM1A_PIN              {19, 16, 0x20}

/* Ball H3: GIOA_6 (0x01), N2HET2_04 (0x04), eTPWM1B (0x20) */

#define PINMUX_H3_GIOA_6_PINMMR            19
#define PINMUX_H3_GIOA_6_SHIFT             24
#define PINMUX_H3_GIOA_6_VALUE             0x01
#define PINMUX_H3_GIOA_6_PIN               {19, 24, 0x01}

#define PINMUX_H3_N2HET2_04_PINMMR         19
#define PINMUX_H3_N2HET2_04_SHIFT          24
#define PINMUX_H3_N2HET2_04_VALUE          0x04
#define PINMUX_H3_N2HET2_04_PIN            {19, 24, 0x04}

#define PINMUX_H3_ETPWM1B_PINMMR           19
#define PINMUX_H3_ETPWM1B_SHIFT            24
#define PINMUX_H3_ETPWM1B_VALUE            0x20
#define PINMUX_H3_ETPWM1B_PIN              {19, 24, 0x20}

/* PINMMR20: M1[0], F2[8], W10[16], J2[24] */

/* Ball M1: GIOA_7 (0x01), N2HET2_06 (0x04), eTPWM2A (0x20) */

#define PINMUX_M1_GIOA_7_PINMMR            20
#define PINMUX_M1_GIOA_7_SHIFT             0
#define PINMUX_M1_GIOA_7_VALUE             0x01
#define PINMUX_M1_GIOA_7_PIN               {20, 0, 0x01}

#define PINMUX_M1_N2HET2_06_PINMMR         20
#define PINMUX_M1_N2HET2_06_SHIFT          0
#define PINMUX_M1_N2HET2_06_VALUE          0x04
#define PINMUX_M1_N2HET2_06_PIN            {20, 0, 0x04}

#define PINMUX_M1_ETPWM2A_PINMMR           20
#define PINMUX_M1_ETPWM2A_SHIFT            0
#define PINMUX_M1_ETPWM2A_VALUE            0x20
#define PINMUX_M1_ETPWM2A_PIN              {20, 0, 0x20}

/* Ball F2: GIOB_2 (0x01), DCAN4TX (0x08) */

#define PINMUX_F2_GIOB_2_PINMMR            20
#define PINMUX_F2_GIOB_2_SHIFT             8
#define PINMUX_F2_GIOB_2_VALUE             0x01
#define PINMUX_F2_GIOB_2_PIN               {20, 8, 0x01}

#define PINMUX_F2_DCAN4TX_PINMMR           20
#define PINMUX_F2_DCAN4TX_SHIFT            8
#define PINMUX_F2_DCAN4TX_VALUE            0x08
#define PINMUX_F2_DCAN4TX_PIN              {20, 8, 0x08}

/* Ball W10: GIOB_3 (0x01), DCAN4RX (0x08) */

#define PINMUX_W10_GIOB_3_PINMMR           20
#define PINMUX_W10_GIOB_3_SHIFT            16
#define PINMUX_W10_GIOB_3_VALUE            0x01
#define PINMUX_W10_GIOB_3_PIN              {20, 16, 0x01}

#define PINMUX_W10_DCAN4RX_PINMMR          20
#define PINMUX_W10_DCAN4RX_SHIFT           16
#define PINMUX_W10_DCAN4RX_VALUE           0x08
#define PINMUX_W10_DCAN4RX_PIN             {20, 16, 0x08}

/* Ball J2: GIOB_6 (0x01), nERROR1 (0x02) */

#define PINMUX_J2_GIOB_6_PINMMR            20
#define PINMUX_J2_GIOB_6_SHIFT             24
#define PINMUX_J2_GIOB_6_VALUE             0x01
#define PINMUX_J2_GIOB_6_PIN               {20, 24, 0x01}

#define PINMUX_J2_NERROR1_PINMMR           20
#define PINMUX_J2_NERROR1_SHIFT            24
#define PINMUX_J2_NERROR1_VALUE            0x02
#define PINMUX_J2_NERROR1_PIN              {20, 24, 0x02}

/* PINMMR21: F1[0], R2[8], F3[16], G3[24] */

/* Ball F1: GIOB_7 (0x01), nERROR2 (0x02), nTZ1_2 (0x20) */

#define PINMUX_F1_GIOB_7_PINMMR            21
#define PINMUX_F1_GIOB_7_SHIFT             0
#define PINMUX_F1_GIOB_7_VALUE             0x01
#define PINMUX_F1_GIOB_7_PIN               {21, 0, 0x01}

#define PINMUX_F1_NERROR2_PINMMR           21
#define PINMUX_F1_NERROR2_SHIFT            0
#define PINMUX_F1_NERROR2_VALUE            0x02
#define PINMUX_F1_NERROR2_PIN              {21, 0, 0x02}

#define PINMUX_F1_NTZ1_2_PINMMR            21
#define PINMUX_F1_NTZ1_2_SHIFT             0
#define PINMUX_F1_NTZ1_2_VALUE             0x20
#define PINMUX_F1_NTZ1_2_PIN               {21, 0, 0x20}

/* Ball R2: MIBSPI1NCS_0 (0x01), MIBSPI1SOMI_1 (0x02), MII_TXD_2 (0x04),
 * ECAP6 (0x20)
 */

#define PINMUX_R2_MIBSPI1NCS_0_PINMMR      21
#define PINMUX_R2_MIBSPI1NCS_0_SHIFT       8
#define PINMUX_R2_MIBSPI1NCS_0_VALUE       0x01
#define PINMUX_R2_MIBSPI1NCS_0_PIN         {21, 8, 0x01}

#define PINMUX_R2_MIBSPI1SOMI_1_PINMMR     21
#define PINMUX_R2_MIBSPI1SOMI_1_SHIFT      8
#define PINMUX_R2_MIBSPI1SOMI_1_VALUE      0x02
#define PINMUX_R2_MIBSPI1SOMI_1_PIN        {21, 8, 0x02}

#define PINMUX_R2_MII_TXD_2_PINMMR         21
#define PINMUX_R2_MII_TXD_2_SHIFT          8
#define PINMUX_R2_MII_TXD_2_VALUE          0x04
#define PINMUX_R2_MII_TXD_2_PIN            {21, 8, 0x04}

#define PINMUX_R2_ECAP6_PINMMR             21
#define PINMUX_R2_ECAP6_SHIFT              8
#define PINMUX_R2_ECAP6_VALUE              0x20
#define PINMUX_R2_ECAP6_PIN                {21, 8, 0x20}

/* Ball F3: MIBSPI1NCS_1 (0x01), MII_COL (0x04), N2HET1_17 (0x08), eQEP1S
 * (0x20)
 */

#define PINMUX_F3_MIBSPI1NCS_1_PINMMR      21
#define PINMUX_F3_MIBSPI1NCS_1_SHIFT       16
#define PINMUX_F3_MIBSPI1NCS_1_VALUE       0x01
#define PINMUX_F3_MIBSPI1NCS_1_PIN         {21, 16, 0x01}

#define PINMUX_F3_MII_COL_PINMMR           21
#define PINMUX_F3_MII_COL_SHIFT            16
#define PINMUX_F3_MII_COL_VALUE            0x04
#define PINMUX_F3_MII_COL_PIN              {21, 16, 0x04}

#define PINMUX_F3_N2HET1_17_PINMMR         21
#define PINMUX_F3_N2HET1_17_SHIFT          16
#define PINMUX_F3_N2HET1_17_VALUE          0x08
#define PINMUX_F3_N2HET1_17_PIN            {21, 16, 0x08}

#define PINMUX_F3_EQEP1S_PINMMR            21
#define PINMUX_F3_EQEP1S_SHIFT             16
#define PINMUX_F3_EQEP1S_VALUE             0x20
#define PINMUX_F3_EQEP1S_PIN               {21, 16, 0x20}

/* Ball G3: MIBSPI1NCS_2 (0x01), MDIO (0x04), N2HET1_19 (0x08) */

#define PINMUX_G3_MIBSPI1NCS_2_PINMMR      21
#define PINMUX_G3_MIBSPI1NCS_2_SHIFT       24
#define PINMUX_G3_MIBSPI1NCS_2_VALUE       0x01
#define PINMUX_G3_MIBSPI1NCS_2_PIN         {21, 24, 0x01}

#define PINMUX_G3_MDIO_PINMMR              21
#define PINMUX_G3_MDIO_SHIFT               24
#define PINMUX_G3_MDIO_VALUE               0x04
#define PINMUX_G3_MDIO_PIN                 {21, 24, 0x04}

#define PINMUX_G3_N2HET1_19_PINMMR         21
#define PINMUX_G3_N2HET1_19_SHIFT          24
#define PINMUX_G3_N2HET1_19_VALUE          0x08
#define PINMUX_G3_N2HET1_19_PIN            {21, 24, 0x08}

/* PINMMR22: J3[0], G19[8], V9[16], V10[24] */

/* Ball J3: MIBSPI1NCS_3 (0x01), N2HET1_21 (0x08), nTZ1_3 (0x20) */

#define PINMUX_J3_MIBSPI1NCS_3_PINMMR      22
#define PINMUX_J3_MIBSPI1NCS_3_SHIFT       0
#define PINMUX_J3_MIBSPI1NCS_3_VALUE       0x01
#define PINMUX_J3_MIBSPI1NCS_3_PIN         {22, 0, 0x01}

#define PINMUX_J3_N2HET1_21_PINMMR         22
#define PINMUX_J3_N2HET1_21_SHIFT          0
#define PINMUX_J3_N2HET1_21_VALUE          0x08
#define PINMUX_J3_N2HET1_21_PIN            {22, 0, 0x08}

#define PINMUX_J3_NTZ1_3_PINMMR            22
#define PINMUX_J3_NTZ1_3_SHIFT             0
#define PINMUX_J3_NTZ1_3_VALUE             0x20
#define PINMUX_J3_NTZ1_3_PIN               {22, 0, 0x20}

/* Ball G19: MIBSPI1NENA (0x01), MII_RXD_2 (0x04), N2HET1_23 (0x08), ECAP4
 * (0x20)
 */

#define PINMUX_G19_MIBSPI1NENA_PINMMR      22
#define PINMUX_G19_MIBSPI1NENA_SHIFT       8
#define PINMUX_G19_MIBSPI1NENA_VALUE       0x01
#define PINMUX_G19_MIBSPI1NENA_PIN         {22, 8, 0x01}

#define PINMUX_G19_MII_RXD_2_PINMMR        22
#define PINMUX_G19_MII_RXD_2_SHIFT         8
#define PINMUX_G19_MII_RXD_2_VALUE         0x04
#define PINMUX_G19_MII_RXD_2_PIN           {22, 8, 0x04}

#define PINMUX_G19_N2HET1_23_PINMMR        22
#define PINMUX_G19_N2HET1_23_SHIFT         8
#define PINMUX_G19_N2HET1_23_VALUE         0x08
#define PINMUX_G19_N2HET1_23_PIN           {22, 8, 0x08}

#define PINMUX_G19_ECAP4_PINMMR            22
#define PINMUX_G19_ECAP4_SHIFT             8
#define PINMUX_G19_ECAP4_VALUE             0x20
#define PINMUX_G19_ECAP4_PIN               {22, 8, 0x20}

/* Ball V9: MIBSPI3CLK (0x01), EXT_SEL_01 (0x02), eQEP1A (0x20) */

#define PINMUX_V9_MIBSPI3CLK_PINMMR        22
#define PINMUX_V9_MIBSPI3CLK_SHIFT         16
#define PINMUX_V9_MIBSPI3CLK_VALUE         0x01
#define PINMUX_V9_MIBSPI3CLK_PIN           {22, 16, 0x01}

#define PINMUX_V9_EXT_SEL_01_PINMMR        22
#define PINMUX_V9_EXT_SEL_01_SHIFT         16
#define PINMUX_V9_EXT_SEL_01_VALUE         0x02
#define PINMUX_V9_EXT_SEL_01_PIN           {22, 16, 0x02}

#define PINMUX_V9_EQEP1A_PINMMR            22
#define PINMUX_V9_EQEP1A_SHIFT             16
#define PINMUX_V9_EQEP1A_VALUE             0x20
#define PINMUX_V9_EQEP1A_PIN               {22, 16, 0x20}

/* Ball V10: MIBSPI3NCS_0 (0x01), AD2EVT (0x02), eQEP1I (0x20) */

#define PINMUX_V10_MIBSPI3NCS_0_PINMMR     22
#define PINMUX_V10_MIBSPI3NCS_0_SHIFT      24
#define PINMUX_V10_MIBSPI3NCS_0_VALUE      0x01
#define PINMUX_V10_MIBSPI3NCS_0_PIN        {22, 24, 0x01}

#define PINMUX_V10_AD2EVT_PINMMR           22
#define PINMUX_V10_AD2EVT_SHIFT            24
#define PINMUX_V10_AD2EVT_VALUE            0x02
#define PINMUX_V10_AD2EVT_PIN              {22, 24, 0x02}

#define PINMUX_V10_EQEP1I_PINMMR           22
#define PINMUX_V10_EQEP1I_SHIFT            24
#define PINMUX_V10_EQEP1I_VALUE            0x20
#define PINMUX_V10_EQEP1I_PIN              {22, 24, 0x20}

/* PINMMR23: V5[0], B2[8], C3[16], W9[24] */

/* Ball V5: MIBSPI3NCS_1 (0x01), MDCLK (0x04), N2HET1_25 (0x08) */

#define PINMUX_V5_MIBSPI3NCS_1_PINMMR      23
#define PINMUX_V5_MIBSPI3NCS_1_SHIFT       0
#define PINMUX_V5_MIBSPI3NCS_1_VALUE       0x01
#define PINMUX_V5_MIBSPI3NCS_1_PIN         {23, 0, 0x01}

#define PINMUX_V5_MDCLK_PINMMR             23
#define PINMUX_V5_MDCLK_SHIFT              0
#define PINMUX_V5_MDCLK_VALUE              0x04
#define PINMUX_V5_MDCLK_PIN                {23, 0, 0x04}

#define PINMUX_V5_N2HET1_25_PINMMR         23
#define PINMUX_V5_N2HET1_25_SHIFT          0
#define PINMUX_V5_N2HET1_25_VALUE          0x08
#define PINMUX_V5_N2HET1_25_PIN            {23, 0, 0x08}

/* Ball B2: MIBSPI3NCS_2 (0x01), I2C1_SDA (0x02), N2HET1_27 (0x08), nTZ1_2
 * (0x20)
 */

#define PINMUX_B2_MIBSPI3NCS_2_PINMMR      23
#define PINMUX_B2_MIBSPI3NCS_2_SHIFT       8
#define PINMUX_B2_MIBSPI3NCS_2_VALUE       0x01
#define PINMUX_B2_MIBSPI3NCS_2_PIN         {23, 8, 0x01}

#define PINMUX_B2_I2C1_SDA_PINMMR          23
#define PINMUX_B2_I2C1_SDA_SHIFT           8
#define PINMUX_B2_I2C1_SDA_VALUE           0x02
#define PINMUX_B2_I2C1_SDA_PIN             {23, 8, 0x02}

#define PINMUX_B2_N2HET1_27_PINMMR         23
#define PINMUX_B2_N2HET1_27_SHIFT          8
#define PINMUX_B2_N2HET1_27_VALUE          0x08
#define PINMUX_B2_N2HET1_27_PIN            {23, 8, 0x08}

#define PINMUX_B2_NTZ1_2_PINMMR            23
#define PINMUX_B2_NTZ1_2_SHIFT             8
#define PINMUX_B2_NTZ1_2_VALUE             0x20
#define PINMUX_B2_NTZ1_2_PIN               {23, 8, 0x20}

/* Ball C3: MIBSPI3NCS_3 (0x01), I2C1_SCL (0x02), N2HET1_29 (0x08), nTZ1_1
 * (0x20)
 */

#define PINMUX_C3_MIBSPI3NCS_3_PINMMR      23
#define PINMUX_C3_MIBSPI3NCS_3_SHIFT       16
#define PINMUX_C3_MIBSPI3NCS_3_VALUE       0x01
#define PINMUX_C3_MIBSPI3NCS_3_PIN         {23, 16, 0x01}

#define PINMUX_C3_I2C1_SCL_PINMMR          23
#define PINMUX_C3_I2C1_SCL_SHIFT           16
#define PINMUX_C3_I2C1_SCL_VALUE           0x02
#define PINMUX_C3_I2C1_SCL_PIN             {23, 16, 0x02}

#define PINMUX_C3_N2HET1_29_PINMMR         23
#define PINMUX_C3_N2HET1_29_SHIFT          16
#define PINMUX_C3_N2HET1_29_VALUE          0x08
#define PINMUX_C3_N2HET1_29_PIN            {23, 16, 0x08}

#define PINMUX_C3_NTZ1_1_PINMMR            23
#define PINMUX_C3_NTZ1_1_SHIFT             16
#define PINMUX_C3_NTZ1_1_VALUE             0x20
#define PINMUX_C3_NTZ1_1_PIN               {23, 16, 0x20}

/* Ball W9: MIBSPI3NENA (0x01), MIBSPI3NCS_5 (0x02), N2HET1_31 (0x08), eQEP1B
 * (0x20)
 */

#define PINMUX_W9_MIBSPI3NENA_PINMMR       23
#define PINMUX_W9_MIBSPI3NENA_SHIFT        24
#define PINMUX_W9_MIBSPI3NENA_VALUE        0x01
#define PINMUX_W9_MIBSPI3NENA_PIN          {23, 24, 0x01}

#define PINMUX_W9_MIBSPI3NCS_5_PINMMR      23
#define PINMUX_W9_MIBSPI3NCS_5_SHIFT       24
#define PINMUX_W9_MIBSPI3NCS_5_VALUE       0x02
#define PINMUX_W9_MIBSPI3NCS_5_PIN         {23, 24, 0x02}

#define PINMUX_W9_N2HET1_31_PINMMR         23
#define PINMUX_W9_N2HET1_31_SHIFT          24
#define PINMUX_W9_N2HET1_31_VALUE          0x08
#define PINMUX_W9_N2HET1_31_PIN            {23, 24, 0x08}

#define PINMUX_W9_EQEP1B_PINMMR            23
#define PINMUX_W9_EQEP1B_SHIFT             24
#define PINMUX_W9_EQEP1B_VALUE             0x20
#define PINMUX_W9_EQEP1B_PIN               {23, 24, 0x20}

/* PINMMR24: W8[0], V8[8], H19[16], E19[24] */

/* Ball W8: MIBSPI3SIMO (0x01), EXT_SEL_00 (0x02), ECAP3 (0x20) */

#define PINMUX_W8_MIBSPI3SIMO_PINMMR       24
#define PINMUX_W8_MIBSPI3SIMO_SHIFT        0
#define PINMUX_W8_MIBSPI3SIMO_VALUE        0x01
#define PINMUX_W8_MIBSPI3SIMO_PIN          {24, 0, 0x01}

#define PINMUX_W8_EXT_SEL_00_PINMMR        24
#define PINMUX_W8_EXT_SEL_00_SHIFT         0
#define PINMUX_W8_EXT_SEL_00_VALUE         0x02
#define PINMUX_W8_EXT_SEL_00_PIN           {24, 0, 0x02}

#define PINMUX_W8_ECAP3_PINMMR             24
#define PINMUX_W8_ECAP3_SHIFT              0
#define PINMUX_W8_ECAP3_VALUE              0x20
#define PINMUX_W8_ECAP3_PIN                {24, 0, 0x20}

/* Ball V8: MIBSPI3SOMI (0x01), EXT_ENA (0x02), ECAP2 (0x20) */

#define PINMUX_V8_MIBSPI3SOMI_PINMMR       24
#define PINMUX_V8_MIBSPI3SOMI_SHIFT        8
#define PINMUX_V8_MIBSPI3SOMI_VALUE        0x01
#define PINMUX_V8_MIBSPI3SOMI_PIN          {24, 8, 0x01}

#define PINMUX_V8_EXT_ENA_PINMMR           24
#define PINMUX_V8_EXT_ENA_SHIFT            8
#define PINMUX_V8_EXT_ENA_VALUE            0x02
#define PINMUX_V8_EXT_ENA_PIN              {24, 8, 0x02}

#define PINMUX_V8_ECAP2_PINMMR             24
#define PINMUX_V8_ECAP2_SHIFT              8
#define PINMUX_V8_ECAP2_VALUE              0x20
#define PINMUX_V8_ECAP2_PIN                {24, 8, 0x20}

/* Ball H19: MIBSPI5CLK (0x01), DMM_DATA_04 (0x02), MII_TXEN (0x04),
 * RMII_TXEN (0x08)
 */

#define PINMUX_H19_MIBSPI5CLK_PINMMR       24
#define PINMUX_H19_MIBSPI5CLK_SHIFT        16
#define PINMUX_H19_MIBSPI5CLK_VALUE        0x01
#define PINMUX_H19_MIBSPI5CLK_PIN          {24, 16, 0x01}

#define PINMUX_H19_DMM_DATA_04_PINMMR      24
#define PINMUX_H19_DMM_DATA_04_SHIFT       16
#define PINMUX_H19_DMM_DATA_04_VALUE       0x02
#define PINMUX_H19_DMM_DATA_04_PIN         {24, 16, 0x02}

#define PINMUX_H19_MII_TXEN_PINMMR         24
#define PINMUX_H19_MII_TXEN_SHIFT          16
#define PINMUX_H19_MII_TXEN_VALUE          0x04
#define PINMUX_H19_MII_TXEN_PIN            {24, 16, 0x04}

#define PINMUX_H19_RMII_TXEN_PINMMR        24
#define PINMUX_H19_RMII_TXEN_SHIFT         16
#define PINMUX_H19_RMII_TXEN_VALUE         0x08
#define PINMUX_H19_RMII_TXEN_PIN           {24, 16, 0x08}

/* Ball E19: MIBSPI5NCS_0 (0x01), DMM_DATA_05 (0x02), eTPWM4A (0x20) */

#define PINMUX_E19_MIBSPI5NCS_0_PINMMR     24
#define PINMUX_E19_MIBSPI5NCS_0_SHIFT      24
#define PINMUX_E19_MIBSPI5NCS_0_VALUE      0x01
#define PINMUX_E19_MIBSPI5NCS_0_PIN        {24, 24, 0x01}

#define PINMUX_E19_DMM_DATA_05_PINMMR      24
#define PINMUX_E19_DMM_DATA_05_SHIFT       24
#define PINMUX_E19_DMM_DATA_05_VALUE       0x02
#define PINMUX_E19_DMM_DATA_05_PIN         {24, 24, 0x02}

#define PINMUX_E19_ETPWM4A_PINMMR          24
#define PINMUX_E19_ETPWM4A_SHIFT           24
#define PINMUX_E19_ETPWM4A_VALUE           0x20
#define PINMUX_E19_ETPWM4A_PIN             {24, 24, 0x20}

/* PINMMR25: B6[0], W6[8], T12[16], H18[24] */

/* Ball B6: MIBSPI5NCS_1 (0x01), DMM_DATA_06 (0x02) */

#define PINMUX_B6_MIBSPI5NCS_1_PINMMR      25
#define PINMUX_B6_MIBSPI5NCS_1_SHIFT       0
#define PINMUX_B6_MIBSPI5NCS_1_VALUE       0x01
#define PINMUX_B6_MIBSPI5NCS_1_PIN         {25, 0, 0x01}

#define PINMUX_B6_DMM_DATA_06_PINMMR       25
#define PINMUX_B6_DMM_DATA_06_SHIFT        0
#define PINMUX_B6_DMM_DATA_06_VALUE        0x02
#define PINMUX_B6_DMM_DATA_06_PIN          {25, 0, 0x02}

/* Ball W6: MIBSPI5NCS_2 (0x01), DMM_DATA_02 (0x02) */

#define PINMUX_W6_MIBSPI5NCS_2_PINMMR      25
#define PINMUX_W6_MIBSPI5NCS_2_SHIFT       8
#define PINMUX_W6_MIBSPI5NCS_2_VALUE       0x01
#define PINMUX_W6_MIBSPI5NCS_2_PIN         {25, 8, 0x01}

#define PINMUX_W6_DMM_DATA_02_PINMMR       25
#define PINMUX_W6_DMM_DATA_02_SHIFT        8
#define PINMUX_W6_DMM_DATA_02_VALUE        0x02
#define PINMUX_W6_DMM_DATA_02_PIN          {25, 8, 0x02}

/* Ball T12: MIBSPI5NCS_3 (0x01), DMM_DATA_03 (0x02) */

#define PINMUX_T12_MIBSPI5NCS_3_PINMMR     25
#define PINMUX_T12_MIBSPI5NCS_3_SHIFT      16
#define PINMUX_T12_MIBSPI5NCS_3_VALUE      0x01
#define PINMUX_T12_MIBSPI5NCS_3_PIN        {25, 16, 0x01}

#define PINMUX_T12_DMM_DATA_03_PINMMR      25
#define PINMUX_T12_DMM_DATA_03_SHIFT       16
#define PINMUX_T12_DMM_DATA_03_VALUE       0x02
#define PINMUX_T12_DMM_DATA_03_PIN         {25, 16, 0x02}

/* Ball H18: MIBSPI5NENA (0x01), DMM_DATA_07 (0x02), MII_RXD_3 (0x04), ECAP5
 * (0x20)
 */

#define PINMUX_H18_MIBSPI5NENA_PINMMR      25
#define PINMUX_H18_MIBSPI5NENA_SHIFT       24
#define PINMUX_H18_MIBSPI5NENA_VALUE       0x01
#define PINMUX_H18_MIBSPI5NENA_PIN         {25, 24, 0x01}

#define PINMUX_H18_DMM_DATA_07_PINMMR      25
#define PINMUX_H18_DMM_DATA_07_SHIFT       24
#define PINMUX_H18_DMM_DATA_07_VALUE       0x02
#define PINMUX_H18_DMM_DATA_07_PIN         {25, 24, 0x02}

#define PINMUX_H18_MII_RXD_3_PINMMR        25
#define PINMUX_H18_MII_RXD_3_SHIFT         24
#define PINMUX_H18_MII_RXD_3_VALUE         0x04
#define PINMUX_H18_MII_RXD_3_PIN           {25, 24, 0x04}

#define PINMUX_H18_ECAP5_PINMMR            25
#define PINMUX_H18_ECAP5_SHIFT             24
#define PINMUX_H18_ECAP5_VALUE             0x20
#define PINMUX_H18_ECAP5_PIN               {25, 24, 0x20}

/* PINMMR26: J19[0], E16[8], H17[16], G17[24] */

/* Ball J19: MIBSPI5SIMO_0 (0x01), DMM_DATA_08 (0x02), MII_TXD_1 (0x04),
 * RMII_TXD_1 (0x08)
 */

#define PINMUX_J19_MIBSPI5SIMO_0_PINMMR    26
#define PINMUX_J19_MIBSPI5SIMO_0_SHIFT     0
#define PINMUX_J19_MIBSPI5SIMO_0_VALUE     0x01
#define PINMUX_J19_MIBSPI5SIMO_0_PIN       {26, 0, 0x01}

#define PINMUX_J19_DMM_DATA_08_PINMMR      26
#define PINMUX_J19_DMM_DATA_08_SHIFT       0
#define PINMUX_J19_DMM_DATA_08_VALUE       0x02
#define PINMUX_J19_DMM_DATA_08_PIN         {26, 0, 0x02}

#define PINMUX_J19_MII_TXD_1_PINMMR        26
#define PINMUX_J19_MII_TXD_1_SHIFT         0
#define PINMUX_J19_MII_TXD_1_VALUE         0x04
#define PINMUX_J19_MII_TXD_1_PIN           {26, 0, 0x04}

#define PINMUX_J19_RMII_TXD_1_PINMMR       26
#define PINMUX_J19_RMII_TXD_1_SHIFT        0
#define PINMUX_J19_RMII_TXD_1_VALUE        0x08
#define PINMUX_J19_RMII_TXD_1_PIN          {26, 0, 0x08}

/* Ball E16: MIBSPI5SIMO_1 (0x01), DMM_DATA_09 (0x02), EXT_SEL_00 (0x10) */

#define PINMUX_E16_MIBSPI5SIMO_1_PINMMR    26
#define PINMUX_E16_MIBSPI5SIMO_1_SHIFT     8
#define PINMUX_E16_MIBSPI5SIMO_1_VALUE     0x01
#define PINMUX_E16_MIBSPI5SIMO_1_PIN       {26, 8, 0x01}

#define PINMUX_E16_DMM_DATA_09_PINMMR      26
#define PINMUX_E16_DMM_DATA_09_SHIFT       8
#define PINMUX_E16_DMM_DATA_09_VALUE       0x02
#define PINMUX_E16_DMM_DATA_09_PIN         {26, 8, 0x02}

#define PINMUX_E16_EXT_SEL_00_PINMMR       26
#define PINMUX_E16_EXT_SEL_00_SHIFT        8
#define PINMUX_E16_EXT_SEL_00_VALUE        0x10
#define PINMUX_E16_EXT_SEL_00_PIN          {26, 8, 0x10}

/* Ball H17: MIBSPI5SIMO_2 (0x01), DMM_DATA_10 (0x02), EXT_SEL_01 (0x10) */

#define PINMUX_H17_MIBSPI5SIMO_2_PINMMR    26
#define PINMUX_H17_MIBSPI5SIMO_2_SHIFT     16
#define PINMUX_H17_MIBSPI5SIMO_2_VALUE     0x01
#define PINMUX_H17_MIBSPI5SIMO_2_PIN       {26, 16, 0x01}

#define PINMUX_H17_DMM_DATA_10_PINMMR      26
#define PINMUX_H17_DMM_DATA_10_SHIFT       16
#define PINMUX_H17_DMM_DATA_10_VALUE       0x02
#define PINMUX_H17_DMM_DATA_10_PIN         {26, 16, 0x02}

#define PINMUX_H17_EXT_SEL_01_PINMMR       26
#define PINMUX_H17_EXT_SEL_01_SHIFT        16
#define PINMUX_H17_EXT_SEL_01_VALUE        0x10
#define PINMUX_H17_EXT_SEL_01_PIN          {26, 16, 0x10}

/* Ball G17: MIBSPI5SIMO_3 (0x01), DMM_DATA_11 (0x02), I2C2_SDA (0x04),
 * EXT_SEL_02 (0x10)
 */

#define PINMUX_G17_MIBSPI5SIMO_3_PINMMR    26
#define PINMUX_G17_MIBSPI5SIMO_3_SHIFT     24
#define PINMUX_G17_MIBSPI5SIMO_3_VALUE     0x01
#define PINMUX_G17_MIBSPI5SIMO_3_PIN       {26, 24, 0x01}

#define PINMUX_G17_DMM_DATA_11_PINMMR      26
#define PINMUX_G17_DMM_DATA_11_SHIFT       24
#define PINMUX_G17_DMM_DATA_11_VALUE       0x02
#define PINMUX_G17_DMM_DATA_11_PIN         {26, 24, 0x02}

#define PINMUX_G17_I2C2_SDA_PINMMR         26
#define PINMUX_G17_I2C2_SDA_SHIFT          24
#define PINMUX_G17_I2C2_SDA_VALUE          0x04
#define PINMUX_G17_I2C2_SDA_PIN            {26, 24, 0x04}

#define PINMUX_G17_EXT_SEL_02_PINMMR       26
#define PINMUX_G17_EXT_SEL_02_SHIFT        24
#define PINMUX_G17_EXT_SEL_02_VALUE        0x10
#define PINMUX_G17_EXT_SEL_02_PIN          {26, 24, 0x10}

/* PINMMR27: J18[0], E17[8], H16[16], G16[24] */

/* Ball J18: MIBSPI5SOMI_0 (0x01), DMM_DATA_12 (0x02), MII_TXD_0 (0x04),
 * RMII_TXD_0 (0x08)
 */

#define PINMUX_J18_MIBSPI5SOMI_0_PINMMR    27
#define PINMUX_J18_MIBSPI5SOMI_0_SHIFT     0
#define PINMUX_J18_MIBSPI5SOMI_0_VALUE     0x01
#define PINMUX_J18_MIBSPI5SOMI_0_PIN       {27, 0, 0x01}

#define PINMUX_J18_DMM_DATA_12_PINMMR      27
#define PINMUX_J18_DMM_DATA_12_SHIFT       0
#define PINMUX_J18_DMM_DATA_12_VALUE       0x02
#define PINMUX_J18_DMM_DATA_12_PIN         {27, 0, 0x02}

#define PINMUX_J18_MII_TXD_0_PINMMR        27
#define PINMUX_J18_MII_TXD_0_SHIFT         0
#define PINMUX_J18_MII_TXD_0_VALUE         0x04
#define PINMUX_J18_MII_TXD_0_PIN           {27, 0, 0x04}

#define PINMUX_J18_RMII_TXD_0_PINMMR       27
#define PINMUX_J18_RMII_TXD_0_SHIFT        0
#define PINMUX_J18_RMII_TXD_0_VALUE        0x08
#define PINMUX_J18_RMII_TXD_0_PIN          {27, 0, 0x08}

/* Ball E17: MIBSPI5SOMI_1 (0x01), DMM_DATA_13 (0x02), EXT_SEL_03 (0x10) */

#define PINMUX_E17_MIBSPI5SOMI_1_PINMMR    27
#define PINMUX_E17_MIBSPI5SOMI_1_SHIFT     8
#define PINMUX_E17_MIBSPI5SOMI_1_VALUE     0x01
#define PINMUX_E17_MIBSPI5SOMI_1_PIN       {27, 8, 0x01}

#define PINMUX_E17_DMM_DATA_13_PINMMR      27
#define PINMUX_E17_DMM_DATA_13_SHIFT       8
#define PINMUX_E17_DMM_DATA_13_VALUE       0x02
#define PINMUX_E17_DMM_DATA_13_PIN         {27, 8, 0x02}

#define PINMUX_E17_EXT_SEL_03_PINMMR       27
#define PINMUX_E17_EXT_SEL_03_SHIFT        8
#define PINMUX_E17_EXT_SEL_03_VALUE        0x10
#define PINMUX_E17_EXT_SEL_03_PIN          {27, 8, 0x10}

/* Ball H16: MIBSPI5SOMI_2 (0x01), DMM_DATA_14 (0x02), EXT_SEL_04 (0x10) */

#define PINMUX_H16_MIBSPI5SOMI_2_PINMMR    27
#define PINMUX_H16_MIBSPI5SOMI_2_SHIFT     16
#define PINMUX_H16_MIBSPI5SOMI_2_VALUE     0x01
#define PINMUX_H16_MIBSPI5SOMI_2_PIN       {27, 16, 0x01}

#define PINMUX_H16_DMM_DATA_14_PINMMR      27
#define PINMUX_H16_DMM_DATA_14_SHIFT       16
#define PINMUX_H16_DMM_DATA_14_VALUE       0x02
#define PINMUX_H16_DMM_DATA_14_PIN         {27, 16, 0x02}

#define PINMUX_H16_EXT_SEL_04_PINMMR       27
#define PINMUX_H16_EXT_SEL_04_SHIFT        16
#define PINMUX_H16_EXT_SEL_04_VALUE        0x10
#define PINMUX_H16_EXT_SEL_04_PIN          {27, 16, 0x10}

/* Ball G16: MIBSPI5SOMI_3 (0x01), DMM_DATA_15 (0x02), I2C2_SCL (0x04),
 * EXT_ENA (0x10)
 */

#define PINMUX_G16_MIBSPI5SOMI_3_PINMMR    27
#define PINMUX_G16_MIBSPI5SOMI_3_SHIFT     24
#define PINMUX_G16_MIBSPI5SOMI_3_VALUE     0x01
#define PINMUX_G16_MIBSPI5SOMI_3_PIN       {27, 24, 0x01}

#define PINMUX_G16_DMM_DATA_15_PINMMR      27
#define PINMUX_G16_DMM_DATA_15_SHIFT       24
#define PINMUX_G16_DMM_DATA_15_VALUE       0x02
#define PINMUX_G16_DMM_DATA_15_PIN         {27, 24, 0x02}

#define PINMUX_G16_I2C2_SCL_PINMMR         27
#define PINMUX_G16_I2C2_SCL_SHIFT          24
#define PINMUX_G16_I2C2_SCL_VALUE          0x04
#define PINMUX_G16_I2C2_SCL_PIN            {27, 24, 0x04}

#define PINMUX_G16_EXT_ENA_PINMMR          27
#define PINMUX_G16_EXT_ENA_SHIFT           24
#define PINMUX_G16_EXT_ENA_VALUE           0x10
#define PINMUX_G16_EXT_ENA_PIN             {27, 24, 0x10}

/* PINMMR28: K18[0], V2[8], W5[16], U1[24] */

/* Ball K18: N2HET1_00 (0x01), MIBSPI4CLK (0x02), eTPWM2B (0x20) */

#define PINMUX_K18_N2HET1_00_PINMMR        28
#define PINMUX_K18_N2HET1_00_SHIFT         0
#define PINMUX_K18_N2HET1_00_VALUE         0x01
#define PINMUX_K18_N2HET1_00_PIN           {28, 0, 0x01}

#define PINMUX_K18_MIBSPI4CLK_PINMMR       28
#define PINMUX_K18_MIBSPI4CLK_SHIFT        0
#define PINMUX_K18_MIBSPI4CLK_VALUE        0x02
#define PINMUX_K18_MIBSPI4CLK_PIN          {28, 0, 0x02}

#define PINMUX_K18_ETPWM2B_PINMMR          28
#define PINMUX_K18_ETPWM2B_SHIFT           0
#define PINMUX_K18_ETPWM2B_VALUE           0x20
#define PINMUX_K18_ETPWM2B_PIN             {28, 0, 0x20}

/* Ball V2: N2HET1_01 (0x01), MIBSPI4NENA (0x02), N2HET2_08 (0x08), eQEP2A
 * (0x20)
 */

#define PINMUX_V2_N2HET1_01_PINMMR         28
#define PINMUX_V2_N2HET1_01_SHIFT          8
#define PINMUX_V2_N2HET1_01_VALUE          0x01
#define PINMUX_V2_N2HET1_01_PIN            {28, 8, 0x01}

#define PINMUX_V2_MIBSPI4NENA_PINMMR       28
#define PINMUX_V2_MIBSPI4NENA_SHIFT        8
#define PINMUX_V2_MIBSPI4NENA_VALUE        0x02
#define PINMUX_V2_MIBSPI4NENA_PIN          {28, 8, 0x02}

#define PINMUX_V2_N2HET2_08_PINMMR         28
#define PINMUX_V2_N2HET2_08_SHIFT          8
#define PINMUX_V2_N2HET2_08_VALUE          0x08
#define PINMUX_V2_N2HET2_08_PIN            {28, 8, 0x08}

#define PINMUX_V2_EQEP2A_PINMMR            28
#define PINMUX_V2_EQEP2A_SHIFT             8
#define PINMUX_V2_EQEP2A_VALUE             0x20
#define PINMUX_V2_EQEP2A_PIN               {28, 8, 0x20}

/* Ball W5: N2HET1_02 (0x01), MIBSPI4SIMO (0x02), eTPWM3A (0x20) */

#define PINMUX_W5_N2HET1_02_PINMMR         28
#define PINMUX_W5_N2HET1_02_SHIFT          16
#define PINMUX_W5_N2HET1_02_VALUE          0x01
#define PINMUX_W5_N2HET1_02_PIN            {28, 16, 0x01}

#define PINMUX_W5_MIBSPI4SIMO_PINMMR       28
#define PINMUX_W5_MIBSPI4SIMO_SHIFT        16
#define PINMUX_W5_MIBSPI4SIMO_VALUE        0x02
#define PINMUX_W5_MIBSPI4SIMO_PIN          {28, 16, 0x02}

#define PINMUX_W5_ETPWM3A_PINMMR           28
#define PINMUX_W5_ETPWM3A_SHIFT            16
#define PINMUX_W5_ETPWM3A_VALUE            0x20
#define PINMUX_W5_ETPWM3A_PIN              {28, 16, 0x20}

/* Ball U1: N2HET1_03 (0x01), MIBSPI4NCS_0 (0x02), N2HET2_10 (0x08), eQEP2B
 * (0x20)
 */

#define PINMUX_U1_N2HET1_03_PINMMR         28
#define PINMUX_U1_N2HET1_03_SHIFT          24
#define PINMUX_U1_N2HET1_03_VALUE          0x01
#define PINMUX_U1_N2HET1_03_PIN            {28, 24, 0x01}

#define PINMUX_U1_MIBSPI4NCS_0_PINMMR      28
#define PINMUX_U1_MIBSPI4NCS_0_SHIFT       24
#define PINMUX_U1_MIBSPI4NCS_0_VALUE       0x02
#define PINMUX_U1_MIBSPI4NCS_0_PIN         {28, 24, 0x02}

#define PINMUX_U1_N2HET2_10_PINMMR         28
#define PINMUX_U1_N2HET2_10_SHIFT          24
#define PINMUX_U1_N2HET2_10_VALUE          0x08
#define PINMUX_U1_N2HET2_10_PIN            {28, 24, 0x08}

#define PINMUX_U1_EQEP2B_PINMMR            28
#define PINMUX_U1_EQEP2B_SHIFT             24
#define PINMUX_U1_EQEP2B_VALUE             0x20
#define PINMUX_U1_EQEP2B_PIN               {28, 24, 0x20}

/* PINMMR29: B12[0], V6[8], W3[16], T1[24] */

/* Ball B12: N2HET1_04 (0x01), MIBSPI4NCS_1 (0x02), eTPWM4B (0x20) */

#define PINMUX_B12_N2HET1_04_PINMMR        29
#define PINMUX_B12_N2HET1_04_SHIFT         0
#define PINMUX_B12_N2HET1_04_VALUE         0x01
#define PINMUX_B12_N2HET1_04_PIN           {29, 0, 0x01}

#define PINMUX_B12_MIBSPI4NCS_1_PINMMR     29
#define PINMUX_B12_MIBSPI4NCS_1_SHIFT      0
#define PINMUX_B12_MIBSPI4NCS_1_VALUE      0x02
#define PINMUX_B12_MIBSPI4NCS_1_PIN        {29, 0, 0x02}

#define PINMUX_B12_ETPWM4B_PINMMR          29
#define PINMUX_B12_ETPWM4B_SHIFT           0
#define PINMUX_B12_ETPWM4B_VALUE           0x20
#define PINMUX_B12_ETPWM4B_PIN             {29, 0, 0x20}

/* Ball V6: N2HET1_05 (0x01), MIBSPI4SOMI (0x02), N2HET2_12 (0x08), eTPWM3B
 * (0x20)
 */

#define PINMUX_V6_N2HET1_05_PINMMR         29
#define PINMUX_V6_N2HET1_05_SHIFT          8
#define PINMUX_V6_N2HET1_05_VALUE          0x01
#define PINMUX_V6_N2HET1_05_PIN            {29, 8, 0x01}

#define PINMUX_V6_MIBSPI4SOMI_PINMMR       29
#define PINMUX_V6_MIBSPI4SOMI_SHIFT        8
#define PINMUX_V6_MIBSPI4SOMI_VALUE        0x02
#define PINMUX_V6_MIBSPI4SOMI_PIN          {29, 8, 0x02}

#define PINMUX_V6_N2HET2_12_PINMMR         29
#define PINMUX_V6_N2HET2_12_SHIFT          8
#define PINMUX_V6_N2HET2_12_VALUE          0x08
#define PINMUX_V6_N2HET2_12_PIN            {29, 8, 0x08}

#define PINMUX_V6_ETPWM3B_PINMMR           29
#define PINMUX_V6_ETPWM3B_SHIFT            8
#define PINMUX_V6_ETPWM3B_VALUE            0x20
#define PINMUX_V6_ETPWM3B_PIN              {29, 8, 0x20}

/* Ball W3: N2HET1_06 (0x01), SCI3RX (0x02), eTPWM5A (0x20) */

#define PINMUX_W3_N2HET1_06_PINMMR         29
#define PINMUX_W3_N2HET1_06_SHIFT          16
#define PINMUX_W3_N2HET1_06_VALUE          0x01
#define PINMUX_W3_N2HET1_06_PIN            {29, 16, 0x01}

#define PINMUX_W3_SCI3RX_PINMMR            29
#define PINMUX_W3_SCI3RX_SHIFT             16
#define PINMUX_W3_SCI3RX_VALUE             0x02
#define PINMUX_W3_SCI3RX_PIN               {29, 16, 0x02}

#define PINMUX_W3_ETPWM5A_PINMMR           29
#define PINMUX_W3_ETPWM5A_SHIFT            16
#define PINMUX_W3_ETPWM5A_VALUE            0x20
#define PINMUX_W3_ETPWM5A_PIN              {29, 16, 0x20}

/* Ball T1: N2HET1_07 (0x01), MIBSPI4NCS_2 (0x02), N2HET2_14 (0x08), eTPWM7B
 * (0x20)
 */

#define PINMUX_T1_N2HET1_07_PINMMR         29
#define PINMUX_T1_N2HET1_07_SHIFT          24
#define PINMUX_T1_N2HET1_07_VALUE          0x01
#define PINMUX_T1_N2HET1_07_PIN            {29, 24, 0x01}

#define PINMUX_T1_MIBSPI4NCS_2_PINMMR      29
#define PINMUX_T1_MIBSPI4NCS_2_SHIFT       24
#define PINMUX_T1_MIBSPI4NCS_2_VALUE       0x02
#define PINMUX_T1_MIBSPI4NCS_2_PIN         {29, 24, 0x02}

#define PINMUX_T1_N2HET2_14_PINMMR         29
#define PINMUX_T1_N2HET2_14_SHIFT          24
#define PINMUX_T1_N2HET2_14_VALUE          0x08
#define PINMUX_T1_N2HET2_14_PIN            {29, 24, 0x08}

#define PINMUX_T1_ETPWM7B_PINMMR           29
#define PINMUX_T1_ETPWM7B_SHIFT            24
#define PINMUX_T1_ETPWM7B_VALUE            0x20
#define PINMUX_T1_ETPWM7B_PIN              {29, 24, 0x20}

/* PINMMR30: E18[0], V7[8], D19[16], E3[24] */

/* Ball E18: N2HET1_08 (0x01), MIBSPI1SIMO_1 (0x02), MII_TXD_3 (0x04) */

#define PINMUX_E18_N2HET1_08_PINMMR        30
#define PINMUX_E18_N2HET1_08_SHIFT         0
#define PINMUX_E18_N2HET1_08_VALUE         0x01
#define PINMUX_E18_N2HET1_08_PIN           {30, 0, 0x01}

#define PINMUX_E18_MIBSPI1SIMO_1_PINMMR    30
#define PINMUX_E18_MIBSPI1SIMO_1_SHIFT     0
#define PINMUX_E18_MIBSPI1SIMO_1_VALUE     0x02
#define PINMUX_E18_MIBSPI1SIMO_1_PIN       {30, 0, 0x02}

#define PINMUX_E18_MII_TXD_3_PINMMR        30
#define PINMUX_E18_MII_TXD_3_SHIFT         0
#define PINMUX_E18_MII_TXD_3_VALUE         0x04
#define PINMUX_E18_MII_TXD_3_PIN           {30, 0, 0x04}

/* Ball V7: N2HET1_09 (0x01), MIBSPI4NCS_3 (0x02), N2HET2_16 (0x08), eTPWM7A
 * (0x20)
 */

#define PINMUX_V7_N2HET1_09_PINMMR         30
#define PINMUX_V7_N2HET1_09_SHIFT          8
#define PINMUX_V7_N2HET1_09_VALUE          0x01
#define PINMUX_V7_N2HET1_09_PIN            {30, 8, 0x01}

#define PINMUX_V7_MIBSPI4NCS_3_PINMMR      30
#define PINMUX_V7_MIBSPI4NCS_3_SHIFT       8
#define PINMUX_V7_MIBSPI4NCS_3_VALUE       0x02
#define PINMUX_V7_MIBSPI4NCS_3_PIN         {30, 8, 0x02}

#define PINMUX_V7_N2HET2_16_PINMMR         30
#define PINMUX_V7_N2HET2_16_SHIFT          8
#define PINMUX_V7_N2HET2_16_VALUE          0x08
#define PINMUX_V7_N2HET2_16_PIN            {30, 8, 0x08}

#define PINMUX_V7_ETPWM7A_PINMMR           30
#define PINMUX_V7_ETPWM7A_SHIFT            8
#define PINMUX_V7_ETPWM7A_VALUE            0x20
#define PINMUX_V7_ETPWM7A_PIN              {30, 8, 0x20}

/* Ball D19: N2HET1_10 (0x01), MIBSPI4NCS_4 (0x02), MII_TX_CLK (0x04),
 * MII_TX_AVCLK4 (0x08), nTZ1_3 (0x20)
 */

#define PINMUX_D19_N2HET1_10_PINMMR        30
#define PINMUX_D19_N2HET1_10_SHIFT         16
#define PINMUX_D19_N2HET1_10_VALUE         0x01
#define PINMUX_D19_N2HET1_10_PIN           {30, 16, 0x01}

#define PINMUX_D19_MIBSPI4NCS_4_PINMMR     30
#define PINMUX_D19_MIBSPI4NCS_4_SHIFT      16
#define PINMUX_D19_MIBSPI4NCS_4_VALUE      0x02
#define PINMUX_D19_MIBSPI4NCS_4_PIN        {30, 16, 0x02}

#define PINMUX_D19_MII_TX_CLK_PINMMR       30
#define PINMUX_D19_MII_TX_CLK_SHIFT        16
#define PINMUX_D19_MII_TX_CLK_VALUE        0x04
#define PINMUX_D19_MII_TX_CLK_PIN          {30, 16, 0x04}

#define PINMUX_D19_MII_TX_AVCLK4_PINMMR    30
#define PINMUX_D19_MII_TX_AVCLK4_SHIFT     16
#define PINMUX_D19_MII_TX_AVCLK4_VALUE     0x08
#define PINMUX_D19_MII_TX_AVCLK4_PIN       {30, 16, 0x08}

#define PINMUX_D19_NTZ1_3_PINMMR           30
#define PINMUX_D19_NTZ1_3_SHIFT            16
#define PINMUX_D19_NTZ1_3_VALUE            0x20
#define PINMUX_D19_NTZ1_3_PIN              {30, 16, 0x20}

/* Ball E3: N2HET1_11 (0x01), MIBSPI3NCS_4 (0x02), N2HET2_18 (0x08),
 * ETPWM1SYNCO (0x20)
 */

#define PINMUX_E3_N2HET1_11_PINMMR         30
#define PINMUX_E3_N2HET1_11_SHIFT          24
#define PINMUX_E3_N2HET1_11_VALUE          0x01
#define PINMUX_E3_N2HET1_11_PIN            {30, 24, 0x01}

#define PINMUX_E3_MIBSPI3NCS_4_PINMMR      30
#define PINMUX_E3_MIBSPI3NCS_4_SHIFT       24
#define PINMUX_E3_MIBSPI3NCS_4_VALUE       0x02
#define PINMUX_E3_MIBSPI3NCS_4_PIN         {30, 24, 0x02}

#define PINMUX_E3_N2HET2_18_PINMMR         30
#define PINMUX_E3_N2HET2_18_SHIFT          24
#define PINMUX_E3_N2HET2_18_VALUE          0x08
#define PINMUX_E3_N2HET2_18_PIN            {30, 24, 0x08}

#define PINMUX_E3_ETPWM1SYNCO_PINMMR       30
#define PINMUX_E3_ETPWM1SYNCO_SHIFT        24
#define PINMUX_E3_ETPWM1SYNCO_VALUE        0x20
#define PINMUX_E3_ETPWM1SYNCO_PIN          {30, 24, 0x20}

/* PINMMR31: B4[0], N2[8], N1[16], A4[24] */

/* Ball B4: N2HET1_12 (0x01), MIBSPI4NCS_5 (0x02), MII_CRS (0x04),
 * RMII_CRS_DV (0x08)
 */

#define PINMUX_B4_N2HET1_12_PINMMR         31
#define PINMUX_B4_N2HET1_12_SHIFT          0
#define PINMUX_B4_N2HET1_12_VALUE          0x01
#define PINMUX_B4_N2HET1_12_PIN            {31, 0, 0x01}

#define PINMUX_B4_MIBSPI4NCS_5_PINMMR      31
#define PINMUX_B4_MIBSPI4NCS_5_SHIFT       0
#define PINMUX_B4_MIBSPI4NCS_5_VALUE       0x02
#define PINMUX_B4_MIBSPI4NCS_5_PIN         {31, 0, 0x02}

#define PINMUX_B4_MII_CRS_PINMMR           31
#define PINMUX_B4_MII_CRS_SHIFT            0
#define PINMUX_B4_MII_CRS_VALUE            0x04
#define PINMUX_B4_MII_CRS_PIN              {31, 0, 0x04}

#define PINMUX_B4_RMII_CRS_DV_PINMMR       31
#define PINMUX_B4_RMII_CRS_DV_SHIFT        0
#define PINMUX_B4_RMII_CRS_DV_VALUE        0x08
#define PINMUX_B4_RMII_CRS_DV_PIN          {31, 0, 0x08}

/* Ball N2: N2HET1_13 (0x01), SCI3TX (0x02), N2HET2_20 (0x08), eTPWM5B (0x20)
 */

#define PINMUX_N2_N2HET1_13_PINMMR         31
#define PINMUX_N2_N2HET1_13_SHIFT          8
#define PINMUX_N2_N2HET1_13_VALUE          0x01
#define PINMUX_N2_N2HET1_13_PIN            {31, 8, 0x01}

#define PINMUX_N2_SCI3TX_PINMMR            31
#define PINMUX_N2_SCI3TX_SHIFT             8
#define PINMUX_N2_SCI3TX_VALUE             0x02
#define PINMUX_N2_SCI3TX_PIN               {31, 8, 0x02}

#define PINMUX_N2_N2HET2_20_PINMMR         31
#define PINMUX_N2_N2HET2_20_SHIFT          8
#define PINMUX_N2_N2HET2_20_VALUE          0x08
#define PINMUX_N2_N2HET2_20_PIN            {31, 8, 0x08}

#define PINMUX_N2_ETPWM5B_PINMMR           31
#define PINMUX_N2_ETPWM5B_SHIFT            8
#define PINMUX_N2_ETPWM5B_VALUE            0x20
#define PINMUX_N2_ETPWM5B_PIN              {31, 8, 0x20}

/* Ball N1: N2HET1_15 (0x01), MIBSPI1NCS_4 (0x02), N2HET2_22 (0x08), ECAP1
 * (0x20)
 */

#define PINMUX_N1_N2HET1_15_PINMMR         31
#define PINMUX_N1_N2HET1_15_SHIFT          16
#define PINMUX_N1_N2HET1_15_VALUE          0x01
#define PINMUX_N1_N2HET1_15_PIN            {31, 16, 0x01}

#define PINMUX_N1_MIBSPI1NCS_4_PINMMR      31
#define PINMUX_N1_MIBSPI1NCS_4_SHIFT       16
#define PINMUX_N1_MIBSPI1NCS_4_VALUE       0x02
#define PINMUX_N1_MIBSPI1NCS_4_PIN         {31, 16, 0x02}

#define PINMUX_N1_N2HET2_22_PINMMR         31
#define PINMUX_N1_N2HET2_22_SHIFT          16
#define PINMUX_N1_N2HET2_22_VALUE          0x08
#define PINMUX_N1_N2HET2_22_PIN            {31, 16, 0x08}

#define PINMUX_N1_ECAP1_PINMMR             31
#define PINMUX_N1_ECAP1_SHIFT              16
#define PINMUX_N1_ECAP1_VALUE              0x20
#define PINMUX_N1_ECAP1_PIN                {31, 16, 0x20}

/* Ball A4: N2HET1_16 (0x01), ETPWM1SYNCI (0x08), ETPWM1SYNCO (0x20) */

#define PINMUX_A4_N2HET1_16_PINMMR         31
#define PINMUX_A4_N2HET1_16_SHIFT          24
#define PINMUX_A4_N2HET1_16_VALUE          0x01
#define PINMUX_A4_N2HET1_16_PIN            {31, 24, 0x01}

#define PINMUX_A4_ETPWM1SYNCI_PINMMR       31
#define PINMUX_A4_ETPWM1SYNCI_SHIFT        24
#define PINMUX_A4_ETPWM1SYNCI_VALUE        0x08
#define PINMUX_A4_ETPWM1SYNCI_PIN          {31, 24, 0x08}

#define PINMUX_A4_ETPWM1SYNCO_PINMMR       31
#define PINMUX_A4_ETPWM1SYNCO_SHIFT        24
#define PINMUX_A4_ETPWM1SYNCO_VALUE        0x20
#define PINMUX_A4_ETPWM1SYNCO_PIN          {31, 24, 0x20}

/* PINMMR32: A13[0], J1[8], B13[16], P2[24] */

/* Ball A13: N2HET1_17 (0x01), EMIF_nOE (0x02), SCI4RX (0x04) */

#define PINMUX_A13_N2HET1_17_PINMMR        32
#define PINMUX_A13_N2HET1_17_SHIFT         0
#define PINMUX_A13_N2HET1_17_VALUE         0x01
#define PINMUX_A13_N2HET1_17_PIN           {32, 0, 0x01}

#define PINMUX_A13_EMIF_NOE_PINMMR         32
#define PINMUX_A13_EMIF_NOE_SHIFT          0
#define PINMUX_A13_EMIF_NOE_VALUE          0x02
#define PINMUX_A13_EMIF_NOE_PIN            {32, 0, 0x02}

#define PINMUX_A13_SCI4RX_PINMMR           32
#define PINMUX_A13_SCI4RX_SHIFT            0
#define PINMUX_A13_SCI4RX_VALUE            0x04
#define PINMUX_A13_SCI4RX_PIN              {32, 0, 0x04}

/* Ball J1: N2HET1_18 (0x01), EMIF_RNW (0x02), eTPWM6A (0x20) */

#define PINMUX_J1_N2HET1_18_PINMMR         32
#define PINMUX_J1_N2HET1_18_SHIFT          8
#define PINMUX_J1_N2HET1_18_VALUE          0x01
#define PINMUX_J1_N2HET1_18_PIN            {32, 8, 0x01}

#define PINMUX_J1_EMIF_RNW_PINMMR          32
#define PINMUX_J1_EMIF_RNW_SHIFT           8
#define PINMUX_J1_EMIF_RNW_VALUE           0x02
#define PINMUX_J1_EMIF_RNW_PIN             {32, 8, 0x02}

#define PINMUX_J1_ETPWM6A_PINMMR           32
#define PINMUX_J1_ETPWM6A_SHIFT            8
#define PINMUX_J1_ETPWM6A_VALUE            0x20
#define PINMUX_J1_ETPWM6A_PIN              {32, 8, 0x20}

/* Ball B13: N2HET1_19 (0x01), EMIF_nDQM_0 (0x02), SCI4TX (0x04) */

#define PINMUX_B13_N2HET1_19_PINMMR        32
#define PINMUX_B13_N2HET1_19_SHIFT         16
#define PINMUX_B13_N2HET1_19_VALUE         0x01
#define PINMUX_B13_N2HET1_19_PIN           {32, 16, 0x01}

#define PINMUX_B13_EMIF_NDQM_0_PINMMR      32
#define PINMUX_B13_EMIF_NDQM_0_SHIFT       16
#define PINMUX_B13_EMIF_NDQM_0_VALUE       0x02
#define PINMUX_B13_EMIF_NDQM_0_PIN         {32, 16, 0x02}

#define PINMUX_B13_SCI4TX_PINMMR           32
#define PINMUX_B13_SCI4TX_SHIFT            16
#define PINMUX_B13_SCI4TX_VALUE            0x04
#define PINMUX_B13_SCI4TX_PIN              {32, 16, 0x04}

/* Ball P2: N2HET1_20 (0x01), EMIF_nDQM_1 (0x02), eTPWM6B (0x20) */

#define PINMUX_P2_N2HET1_20_PINMMR         32
#define PINMUX_P2_N2HET1_20_SHIFT          24
#define PINMUX_P2_N2HET1_20_VALUE          0x01
#define PINMUX_P2_N2HET1_20_PIN            {32, 24, 0x01}

#define PINMUX_P2_EMIF_NDQM_1_PINMMR       32
#define PINMUX_P2_EMIF_NDQM_1_SHIFT        24
#define PINMUX_P2_EMIF_NDQM_1_VALUE        0x02
#define PINMUX_P2_EMIF_NDQM_1_PIN          {32, 24, 0x02}

#define PINMUX_P2_ETPWM6B_PINMMR           32
#define PINMUX_P2_ETPWM6B_SHIFT            24
#define PINMUX_P2_ETPWM6B_VALUE            0x20
#define PINMUX_P2_ETPWM6B_PIN              {32, 24, 0x20}

/* PINMMR33: H4[0], B3[8], J4[16], P1[24] */

/* Ball H4: N2HET1_21 (0x01), EMIF_nDQM_2 (0x02) */

#define PINMUX_H4_N2HET1_21_PINMMR         33
#define PINMUX_H4_N2HET1_21_SHIFT          0
#define PINMUX_H4_N2HET1_21_VALUE          0x01
#define PINMUX_H4_N2HET1_21_PIN            {33, 0, 0x01}

#define PINMUX_H4_EMIF_NDQM_2_PINMMR       33
#define PINMUX_H4_EMIF_NDQM_2_SHIFT        0
#define PINMUX_H4_EMIF_NDQM_2_VALUE        0x02
#define PINMUX_H4_EMIF_NDQM_2_PIN          {33, 0, 0x02}

/* Ball B3: N2HET1_22 (0x01), EMIF_nDQM_3 (0x02) */

#define PINMUX_B3_N2HET1_22_PINMMR         33
#define PINMUX_B3_N2HET1_22_SHIFT          8
#define PINMUX_B3_N2HET1_22_VALUE          0x01
#define PINMUX_B3_N2HET1_22_PIN            {33, 8, 0x01}

#define PINMUX_B3_EMIF_NDQM_3_PINMMR       33
#define PINMUX_B3_EMIF_NDQM_3_SHIFT        8
#define PINMUX_B3_EMIF_NDQM_3_VALUE        0x02
#define PINMUX_B3_EMIF_NDQM_3_PIN          {33, 8, 0x02}

/* Ball J4: N2HET1_23 (0x01), EMIF_BA_0 (0x02) */

#define PINMUX_J4_N2HET1_23_PINMMR         33
#define PINMUX_J4_N2HET1_23_SHIFT          16
#define PINMUX_J4_N2HET1_23_VALUE          0x01
#define PINMUX_J4_N2HET1_23_PIN            {33, 16, 0x01}

#define PINMUX_J4_EMIF_BA_0_PINMMR         33
#define PINMUX_J4_EMIF_BA_0_SHIFT          16
#define PINMUX_J4_EMIF_BA_0_VALUE          0x02
#define PINMUX_J4_EMIF_BA_0_PIN            {33, 16, 0x02}

/* Ball P1: N2HET1_24 (0x01), MIBSPI1NCS_5 (0x02), MII_RXD_0 (0x04),
 * RMII_RXD_0 (0x08)
 */

#define PINMUX_P1_N2HET1_24_PINMMR         33
#define PINMUX_P1_N2HET1_24_SHIFT          24
#define PINMUX_P1_N2HET1_24_VALUE          0x01
#define PINMUX_P1_N2HET1_24_PIN            {33, 24, 0x01}

#define PINMUX_P1_MIBSPI1NCS_5_PINMMR      33
#define PINMUX_P1_MIBSPI1NCS_5_SHIFT       24
#define PINMUX_P1_MIBSPI1NCS_5_VALUE       0x02
#define PINMUX_P1_MIBSPI1NCS_5_PIN         {33, 24, 0x02}

#define PINMUX_P1_MII_RXD_0_PINMMR         33
#define PINMUX_P1_MII_RXD_0_SHIFT          24
#define PINMUX_P1_MII_RXD_0_VALUE          0x04
#define PINMUX_P1_MII_RXD_0_PIN            {33, 24, 0x04}

#define PINMUX_P1_RMII_RXD_0_PINMMR        33
#define PINMUX_P1_RMII_RXD_0_SHIFT         24
#define PINMUX_P1_RMII_RXD_0_VALUE         0x08
#define PINMUX_P1_RMII_RXD_0_PIN           {33, 24, 0x08}

/* PINMMR34: A14[0], K19[8], B11[16], D8[24] */

/* Ball A14: N2HET1_26 (0x01), MII_RXD_1 (0x04), RMII_RXD_1 (0x08) */

#define PINMUX_A14_N2HET1_26_PINMMR        34
#define PINMUX_A14_N2HET1_26_SHIFT         0
#define PINMUX_A14_N2HET1_26_VALUE         0x01
#define PINMUX_A14_N2HET1_26_PIN           {34, 0, 0x01}

#define PINMUX_A14_MII_RXD_1_PINMMR        34
#define PINMUX_A14_MII_RXD_1_SHIFT         0
#define PINMUX_A14_MII_RXD_1_VALUE         0x04
#define PINMUX_A14_MII_RXD_1_PIN           {34, 0, 0x04}

#define PINMUX_A14_RMII_RXD_1_PINMMR       34
#define PINMUX_A14_RMII_RXD_1_SHIFT        0
#define PINMUX_A14_RMII_RXD_1_VALUE        0x08
#define PINMUX_A14_RMII_RXD_1_PIN          {34, 0, 0x08}

/* Ball K19: N2HET1_28 (0x01), MII_RXCLK (0x04), RMII_REFCLK (0x08),
 * MII_RX_AVCLK4 (0x10)
 */

#define PINMUX_K19_N2HET1_28_PINMMR        34
#define PINMUX_K19_N2HET1_28_SHIFT         8
#define PINMUX_K19_N2HET1_28_VALUE         0x01
#define PINMUX_K19_N2HET1_28_PIN           {34, 8, 0x01}

#define PINMUX_K19_MII_RXCLK_PINMMR        34
#define PINMUX_K19_MII_RXCLK_SHIFT         8
#define PINMUX_K19_MII_RXCLK_VALUE         0x04
#define PINMUX_K19_MII_RXCLK_PIN           {34, 8, 0x04}

#define PINMUX_K19_RMII_REFCLK_PINMMR      34
#define PINMUX_K19_RMII_REFCLK_SHIFT       8
#define PINMUX_K19_RMII_REFCLK_VALUE       0x08
#define PINMUX_K19_RMII_REFCLK_PIN         {34, 8, 0x08}

#define PINMUX_K19_MII_RX_AVCLK4_PINMMR    34
#define PINMUX_K19_MII_RX_AVCLK4_SHIFT     8
#define PINMUX_K19_MII_RX_AVCLK4_VALUE     0x10
#define PINMUX_K19_MII_RX_AVCLK4_PIN       {34, 8, 0x10}

/* Ball B11: N2HET1_30 (0x01), MII_RX_DV (0x04), eQEP2S (0x20) */

#define PINMUX_B11_N2HET1_30_PINMMR        34
#define PINMUX_B11_N2HET1_30_SHIFT         16
#define PINMUX_B11_N2HET1_30_VALUE         0x01
#define PINMUX_B11_N2HET1_30_PIN           {34, 16, 0x01}

#define PINMUX_B11_MII_RX_DV_PINMMR        34
#define PINMUX_B11_MII_RX_DV_SHIFT         16
#define PINMUX_B11_MII_RX_DV_VALUE         0x04
#define PINMUX_B11_MII_RX_DV_PIN           {34, 16, 0x04}

#define PINMUX_B11_EQEP2S_PINMMR           34
#define PINMUX_B11_EQEP2S_SHIFT            16
#define PINMUX_B11_EQEP2S_VALUE            0x20
#define PINMUX_B11_EQEP2S_PIN              {34, 16, 0x20}

/* Ball D8: N2HET2_01 (0x01), N2HET1_NDIS (0x02) */

#define PINMUX_D8_N2HET2_01_PINMMR         34
#define PINMUX_D8_N2HET2_01_SHIFT          24
#define PINMUX_D8_N2HET2_01_VALUE          0x01
#define PINMUX_D8_N2HET2_01_PIN            {34, 24, 0x01}

#define PINMUX_D8_N2HET1_NDIS_PINMMR       34
#define PINMUX_D8_N2HET1_NDIS_SHIFT        24
#define PINMUX_D8_N2HET1_NDIS_VALUE        0x02
#define PINMUX_D8_N2HET1_NDIS_PIN          {34, 24, 0x02}

/* PINMMR35: D7[0], D3[8], D2[16], D1[24] */

/* Ball D7: N2HET2_02 (0x01), N2HET2_NDIS (0x02) */

#define PINMUX_D7_N2HET2_02_PINMMR         35
#define PINMUX_D7_N2HET2_02_SHIFT          0
#define PINMUX_D7_N2HET2_02_VALUE          0x01
#define PINMUX_D7_N2HET2_02_PIN            {35, 0, 0x01}

#define PINMUX_D7_N2HET2_NDIS_PINMMR       35
#define PINMUX_D7_N2HET2_NDIS_SHIFT        0
#define PINMUX_D7_N2HET2_NDIS_VALUE        0x02
#define PINMUX_D7_N2HET2_NDIS_PIN          {35, 0, 0x02}

/* Ball D3: N2HET2_12 (0x01), MIBSPI2NENA (0x10), MIBSPI2NCS_1 (0x20) */

#define PINMUX_D3_N2HET2_12_PINMMR         35
#define PINMUX_D3_N2HET2_12_SHIFT          8
#define PINMUX_D3_N2HET2_12_VALUE          0x01
#define PINMUX_D3_N2HET2_12_PIN            {35, 8, 0x01}

#define PINMUX_D3_MIBSPI2NENA_PINMMR       35
#define PINMUX_D3_MIBSPI2NENA_SHIFT        8
#define PINMUX_D3_MIBSPI2NENA_VALUE        0x10
#define PINMUX_D3_MIBSPI2NENA_PIN          {35, 8, 0x10}

#define PINMUX_D3_MIBSPI2NCS_1_PINMMR      35
#define PINMUX_D3_MIBSPI2NCS_1_SHIFT       8
#define PINMUX_D3_MIBSPI2NCS_1_VALUE       0x20
#define PINMUX_D3_MIBSPI2NCS_1_PIN         {35, 8, 0x20}

/* Ball D2: N2HET2_13 (0x01), MIBSPI2SOMI (0x10) */

#define PINMUX_D2_N2HET2_13_PINMMR         35
#define PINMUX_D2_N2HET2_13_SHIFT          16
#define PINMUX_D2_N2HET2_13_VALUE          0x01
#define PINMUX_D2_N2HET2_13_PIN            {35, 16, 0x01}

#define PINMUX_D2_MIBSPI2SOMI_PINMMR       35
#define PINMUX_D2_MIBSPI2SOMI_SHIFT        16
#define PINMUX_D2_MIBSPI2SOMI_VALUE        0x10
#define PINMUX_D2_MIBSPI2SOMI_PIN          {35, 16, 0x10}

/* Ball D1: N2HET2_14 (0x01), MIBSPI2SIMO (0x10) */

#define PINMUX_D1_N2HET2_14_PINMMR         35
#define PINMUX_D1_N2HET2_14_SHIFT          24
#define PINMUX_D1_N2HET2_14_VALUE          0x01
#define PINMUX_D1_N2HET2_14_PIN            {35, 24, 0x01}

#define PINMUX_D1_MIBSPI2SIMO_PINMMR       35
#define PINMUX_D1_MIBSPI2SIMO_SHIFT        24
#define PINMUX_D1_MIBSPI2SIMO_VALUE        0x10
#define PINMUX_D1_MIBSPI2SIMO_PIN          {35, 24, 0x10}

/* PINMMR36: P4[0], T5[8], T4[16], U7[24] */

/* Ball P4: N2HET2_19 (0x01), LIN2RX (0x02) */

#define PINMUX_P4_N2HET2_19_PINMMR         36
#define PINMUX_P4_N2HET2_19_SHIFT          0
#define PINMUX_P4_N2HET2_19_VALUE          0x01
#define PINMUX_P4_N2HET2_19_PIN            {36, 0, 0x01}

#define PINMUX_P4_LIN2RX_PINMMR            36
#define PINMUX_P4_LIN2RX_SHIFT             0
#define PINMUX_P4_LIN2RX_VALUE             0x02
#define PINMUX_P4_LIN2RX_PIN               {36, 0, 0x02}

/* Ball T5: N2HET2_20 (0x01), LIN2TX (0x02) */

#define PINMUX_T5_N2HET2_20_PINMMR         36
#define PINMUX_T5_N2HET2_20_SHIFT          8
#define PINMUX_T5_N2HET2_20_VALUE          0x01
#define PINMUX_T5_N2HET2_20_PIN            {36, 8, 0x01}

#define PINMUX_T5_LIN2TX_PINMMR            36
#define PINMUX_T5_LIN2TX_SHIFT             8
#define PINMUX_T5_LIN2TX_VALUE             0x02
#define PINMUX_T5_LIN2TX_PIN               {36, 8, 0x02}

/* Ball T4: MII_RXCLK (0x01), MII_RX_AVCLK4 (0x10) */

#define PINMUX_T4_MII_RXCLK_PINMMR         36
#define PINMUX_T4_MII_RXCLK_SHIFT          16
#define PINMUX_T4_MII_RXCLK_VALUE          0x01
#define PINMUX_T4_MII_RXCLK_PIN            {36, 16, 0x01}

#define PINMUX_T4_MII_RX_AVCLK4_PINMMR     36
#define PINMUX_T4_MII_RX_AVCLK4_SHIFT      16
#define PINMUX_T4_MII_RX_AVCLK4_VALUE      0x10
#define PINMUX_T4_MII_RX_AVCLK4_PIN        {36, 16, 0x10}

/* Ball U7: MII_TX_CLK (0x01), MII_TX_AVCLK4 (0x10) */

#define PINMUX_U7_MII_TX_CLK_PINMMR        36
#define PINMUX_U7_MII_TX_CLK_SHIFT         24
#define PINMUX_U7_MII_TX_CLK_VALUE         0x01
#define PINMUX_U7_MII_TX_CLK_PIN           {36, 24, 0x01}

#define PINMUX_U7_MII_TX_AVCLK4_PINMMR     36
#define PINMUX_U7_MII_TX_AVCLK4_SHIFT      24
#define PINMUX_U7_MII_TX_AVCLK4_VALUE      0x10
#define PINMUX_U7_MII_TX_AVCLK4_PIN        {36, 24, 0x10}

/* PINMMR37: E2[0], N3[8] */

/* Ball E2: N2HET2_03 (0x01), MIBSPI2CLK (0x10) */

#define PINMUX_E2_N2HET2_03_PINMMR         37
#define PINMUX_E2_N2HET2_03_SHIFT          0
#define PINMUX_E2_N2HET2_03_VALUE          0x01
#define PINMUX_E2_N2HET2_03_PIN            {37, 0, 0x01}

#define PINMUX_E2_MIBSPI2CLK_PINMMR        37
#define PINMUX_E2_MIBSPI2CLK_SHIFT         0
#define PINMUX_E2_MIBSPI2CLK_VALUE         0x10
#define PINMUX_E2_MIBSPI2CLK_PIN           {37, 0, 0x10}

/* Ball N3: N2HET2_07 (0x01), MIBSPI2NCS_0 (0x10) */

#define PINMUX_N3_N2HET2_07_PINMMR         37
#define PINMUX_N3_N2HET2_07_SHIFT          8
#define PINMUX_N3_N2HET2_07_VALUE          0x01
#define PINMUX_N3_N2HET2_07_PIN            {37, 8, 0x01}

#define PINMUX_N3_MIBSPI2NCS_0_PINMMR      37
#define PINMUX_N3_MIBSPI2NCS_0_SHIFT       8
#define PINMUX_N3_MIBSPI2NCS_0_VALUE       0x10
#define PINMUX_N3_MIBSPI2NCS_0_PIN         {37, 8, 0x10}

/* Input Pin Multiplexing (PINMMR80-99) *************************************/

/* These fields select which ball drives an input signal when the same signal
 * is routed to more than one ball.  The corresponding ball must also be
 * multiplexed to that function above.
 */

/* PINMMR80: AD2EVT[0] */

/* Signal AD2EVT driven from: T10 (0x01), V10 (0x02) */

#define PINMUX_SIG_AD2EVT_T10_PINMMR       80
#define PINMUX_SIG_AD2EVT_T10_SHIFT        0
#define PINMUX_SIG_AD2EVT_T10_VALUE        0x01
#define PINMUX_SIG_AD2EVT_T10_PIN          {80, 0, 0x01}

#define PINMUX_SIG_AD2EVT_V10_PINMMR       80
#define PINMUX_SIG_AD2EVT_V10_SHIFT        0
#define PINMUX_SIG_AD2EVT_V10_VALUE        0x02
#define PINMUX_SIG_AD2EVT_V10_PIN          {80, 0, 0x02}

/* PINMMR83: GIOA_0[24] */

/* Signal GIOA_0 driven from: A5 (0x01), R5 (0x02) */

#define PINMUX_SIG_GIOA_0_A5_PINMMR        83
#define PINMUX_SIG_GIOA_0_A5_SHIFT         24
#define PINMUX_SIG_GIOA_0_A5_VALUE         0x01
#define PINMUX_SIG_GIOA_0_A5_PIN           {83, 24, 0x01}

#define PINMUX_SIG_GIOA_0_R5_PINMMR        83
#define PINMUX_SIG_GIOA_0_R5_SHIFT         24
#define PINMUX_SIG_GIOA_0_R5_VALUE         0x02
#define PINMUX_SIG_GIOA_0_R5_PIN           {83, 24, 0x02}

/* PINMMR84: GIOA_1[0], GIOA_2[8], GIOA_3[16], GIOA_4[24] */

/* Signal GIOA_1 driven from: C2 (0x01), R6 (0x02) */

#define PINMUX_SIG_GIOA_1_C2_PINMMR        84
#define PINMUX_SIG_GIOA_1_C2_SHIFT         0
#define PINMUX_SIG_GIOA_1_C2_VALUE         0x01
#define PINMUX_SIG_GIOA_1_C2_PIN           {84, 0, 0x01}

#define PINMUX_SIG_GIOA_1_R6_PINMMR        84
#define PINMUX_SIG_GIOA_1_R6_SHIFT         0
#define PINMUX_SIG_GIOA_1_R6_VALUE         0x02
#define PINMUX_SIG_GIOA_1_R6_PIN           {84, 0, 0x02}

/* Signal GIOA_2 driven from: C1 (0x01), B15 (0x02) */

#define PINMUX_SIG_GIOA_2_C1_PINMMR        84
#define PINMUX_SIG_GIOA_2_C1_SHIFT         8
#define PINMUX_SIG_GIOA_2_C1_VALUE         0x01
#define PINMUX_SIG_GIOA_2_C1_PIN           {84, 8, 0x01}

#define PINMUX_SIG_GIOA_2_B15_PINMMR       84
#define PINMUX_SIG_GIOA_2_B15_SHIFT        8
#define PINMUX_SIG_GIOA_2_B15_VALUE        0x02
#define PINMUX_SIG_GIOA_2_B15_PIN          {84, 8, 0x02}

/* Signal GIOA_3 driven from: E1 (0x01), R7 (0x02) */

#define PINMUX_SIG_GIOA_3_E1_PINMMR        84
#define PINMUX_SIG_GIOA_3_E1_SHIFT         16
#define PINMUX_SIG_GIOA_3_E1_VALUE         0x01
#define PINMUX_SIG_GIOA_3_E1_PIN           {84, 16, 0x01}

#define PINMUX_SIG_GIOA_3_R7_PINMMR        84
#define PINMUX_SIG_GIOA_3_R7_SHIFT         16
#define PINMUX_SIG_GIOA_3_R7_VALUE         0x02
#define PINMUX_SIG_GIOA_3_R7_PIN           {84, 16, 0x02}

/* Signal GIOA_4 driven from: A6 (0x01), R8 (0x02) */

#define PINMUX_SIG_GIOA_4_A6_PINMMR        84
#define PINMUX_SIG_GIOA_4_A6_SHIFT         24
#define PINMUX_SIG_GIOA_4_A6_VALUE         0x01
#define PINMUX_SIG_GIOA_4_A6_PIN           {84, 24, 0x01}

#define PINMUX_SIG_GIOA_4_R8_PINMMR        84
#define PINMUX_SIG_GIOA_4_R8_SHIFT         24
#define PINMUX_SIG_GIOA_4_R8_VALUE         0x02
#define PINMUX_SIG_GIOA_4_R8_PIN           {84, 24, 0x02}

/* PINMMR85: GIOA_5[0], GIOA_6[8], GIOA_7[16], GIOB_0[24] */

/* Signal GIOA_5 driven from: B5 (0x01), R9 (0x02) */

#define PINMUX_SIG_GIOA_5_B5_PINMMR        85
#define PINMUX_SIG_GIOA_5_B5_SHIFT         0
#define PINMUX_SIG_GIOA_5_B5_VALUE         0x01
#define PINMUX_SIG_GIOA_5_B5_PIN           {85, 0, 0x01}

#define PINMUX_SIG_GIOA_5_R9_PINMMR        85
#define PINMUX_SIG_GIOA_5_R9_SHIFT         0
#define PINMUX_SIG_GIOA_5_R9_VALUE         0x02
#define PINMUX_SIG_GIOA_5_R9_PIN           {85, 0, 0x02}

/* Signal GIOA_6 driven from: H3 (0x01), R10 (0x02) */

#define PINMUX_SIG_GIOA_6_H3_PINMMR        85
#define PINMUX_SIG_GIOA_6_H3_SHIFT         8
#define PINMUX_SIG_GIOA_6_H3_VALUE         0x01
#define PINMUX_SIG_GIOA_6_H3_PIN           {85, 8, 0x01}

#define PINMUX_SIG_GIOA_6_R10_PINMMR       85
#define PINMUX_SIG_GIOA_6_R10_SHIFT        8
#define PINMUX_SIG_GIOA_6_R10_VALUE        0x02
#define PINMUX_SIG_GIOA_6_R10_PIN          {85, 8, 0x02}

/* Signal GIOA_7 driven from: M1 (0x01), R11 (0x02) */

#define PINMUX_SIG_GIOA_7_M1_PINMMR        85
#define PINMUX_SIG_GIOA_7_M1_SHIFT         16
#define PINMUX_SIG_GIOA_7_M1_VALUE         0x01
#define PINMUX_SIG_GIOA_7_M1_PIN           {85, 16, 0x01}

#define PINMUX_SIG_GIOA_7_R11_PINMMR       85
#define PINMUX_SIG_GIOA_7_R11_SHIFT        16
#define PINMUX_SIG_GIOA_7_R11_VALUE        0x02
#define PINMUX_SIG_GIOA_7_R11_PIN          {85, 16, 0x02}

/* Signal GIOB_0 driven from: M2 (0x01), B8 (0x02) */

#define PINMUX_SIG_GIOB_0_M2_PINMMR        85
#define PINMUX_SIG_GIOB_0_M2_SHIFT         24
#define PINMUX_SIG_GIOB_0_M2_VALUE         0x01
#define PINMUX_SIG_GIOB_0_M2_PIN           {85, 24, 0x01}

#define PINMUX_SIG_GIOB_0_B8_PINMMR        85
#define PINMUX_SIG_GIOB_0_B8_SHIFT         24
#define PINMUX_SIG_GIOB_0_B8_VALUE         0x02
#define PINMUX_SIG_GIOB_0_B8_PIN           {85, 24, 0x02}

/* PINMMR86: GIOB_1[0], GIOB_2[8], GIOB_3[16], GIOB_4[24] */

/* Signal GIOB_1 driven from: K2 (0x01), B16 (0x02) */

#define PINMUX_SIG_GIOB_1_K2_PINMMR        86
#define PINMUX_SIG_GIOB_1_K2_SHIFT         0
#define PINMUX_SIG_GIOB_1_K2_VALUE         0x01
#define PINMUX_SIG_GIOB_1_K2_PIN           {86, 0, 0x01}

#define PINMUX_SIG_GIOB_1_B16_PINMMR       86
#define PINMUX_SIG_GIOB_1_B16_SHIFT        0
#define PINMUX_SIG_GIOB_1_B16_VALUE        0x02
#define PINMUX_SIG_GIOB_1_B16_PIN          {86, 0, 0x02}

/* Signal GIOB_2 driven from: F2 (0x01), B9 (0x02) */

#define PINMUX_SIG_GIOB_2_F2_PINMMR        86
#define PINMUX_SIG_GIOB_2_F2_SHIFT         8
#define PINMUX_SIG_GIOB_2_F2_VALUE         0x01
#define PINMUX_SIG_GIOB_2_F2_PIN           {86, 8, 0x01}

#define PINMUX_SIG_GIOB_2_B9_PINMMR        86
#define PINMUX_SIG_GIOB_2_B9_SHIFT         8
#define PINMUX_SIG_GIOB_2_B9_VALUE         0x02
#define PINMUX_SIG_GIOB_2_B9_PIN           {86, 8, 0x02}

/* Signal GIOB_3 driven from: W10 (0x01), R4 (0x02) */

#define PINMUX_SIG_GIOB_3_W10_PINMMR       86
#define PINMUX_SIG_GIOB_3_W10_SHIFT        16
#define PINMUX_SIG_GIOB_3_W10_VALUE        0x01
#define PINMUX_SIG_GIOB_3_W10_PIN          {86, 16, 0x01}

#define PINMUX_SIG_GIOB_3_R4_PINMMR        86
#define PINMUX_SIG_GIOB_3_R4_SHIFT         16
#define PINMUX_SIG_GIOB_3_R4_VALUE         0x02
#define PINMUX_SIG_GIOB_3_R4_PIN           {86, 16, 0x02}

/* Signal GIOB_4 driven from: G1 (0x01), L17 (0x02) */

#define PINMUX_SIG_GIOB_4_G1_PINMMR        86
#define PINMUX_SIG_GIOB_4_G1_SHIFT         24
#define PINMUX_SIG_GIOB_4_G1_VALUE         0x01
#define PINMUX_SIG_GIOB_4_G1_PIN           {86, 24, 0x01}

#define PINMUX_SIG_GIOB_4_L17_PINMMR       86
#define PINMUX_SIG_GIOB_4_L17_SHIFT        24
#define PINMUX_SIG_GIOB_4_L17_VALUE        0x02
#define PINMUX_SIG_GIOB_4_L17_PIN          {86, 24, 0x02}

/* PINMMR87: GIOB_5[0], GIOB_6[8], GIOB_7[16], MDIO[24] */

/* Signal GIOB_5 driven from: G2 (0x01), M17 (0x02) */

#define PINMUX_SIG_GIOB_5_G2_PINMMR        87
#define PINMUX_SIG_GIOB_5_G2_SHIFT         0
#define PINMUX_SIG_GIOB_5_G2_VALUE         0x01
#define PINMUX_SIG_GIOB_5_G2_PIN           {87, 0, 0x01}

#define PINMUX_SIG_GIOB_5_M17_PINMMR       87
#define PINMUX_SIG_GIOB_5_M17_SHIFT        0
#define PINMUX_SIG_GIOB_5_M17_VALUE        0x02
#define PINMUX_SIG_GIOB_5_M17_PIN          {87, 0, 0x02}

/* Signal GIOB_6 driven from: J2 (0x01), R3 (0x02) */

#define PINMUX_SIG_GIOB_6_J2_PINMMR        87
#define PINMUX_SIG_GIOB_6_J2_SHIFT         8
#define PINMUX_SIG_GIOB_6_J2_VALUE         0x01
#define PINMUX_SIG_GIOB_6_J2_PIN           {87, 8, 0x01}

#define PINMUX_SIG_GIOB_6_R3_PINMMR        87
#define PINMUX_SIG_GIOB_6_R3_SHIFT         8
#define PINMUX_SIG_GIOB_6_R3_VALUE         0x02
#define PINMUX_SIG_GIOB_6_R3_PIN           {87, 8, 0x02}

/* Signal GIOB_7 driven from: F1 (0x01), P3 (0x02) */

#define PINMUX_SIG_GIOB_7_F1_PINMMR        87
#define PINMUX_SIG_GIOB_7_F1_SHIFT         16
#define PINMUX_SIG_GIOB_7_F1_VALUE         0x01
#define PINMUX_SIG_GIOB_7_F1_PIN           {87, 16, 0x01}

#define PINMUX_SIG_GIOB_7_P3_PINMMR        87
#define PINMUX_SIG_GIOB_7_P3_SHIFT         16
#define PINMUX_SIG_GIOB_7_P3_VALUE         0x02
#define PINMUX_SIG_GIOB_7_P3_PIN           {87, 16, 0x02}

/* Signal MDIO driven from: F4 (0x01), G3 (0x02) */

#define PINMUX_SIG_MDIO_F4_PINMMR          87
#define PINMUX_SIG_MDIO_F4_SHIFT           24
#define PINMUX_SIG_MDIO_F4_VALUE           0x01
#define PINMUX_SIG_MDIO_F4_PIN             {87, 24, 0x01}

#define PINMUX_SIG_MDIO_G3_PINMMR          87
#define PINMUX_SIG_MDIO_G3_SHIFT           24
#define PINMUX_SIG_MDIO_G3_VALUE           0x02
#define PINMUX_SIG_MDIO_G3_PIN             {87, 24, 0x02}

/* PINMMR88: MIBSPI1NCS_4[0], MIBSPI1NCS_5[8] */

/* Signal MIBSPI1NCS_4 driven from: U10 (0x01), N1 (0x02) */

#define PINMUX_SIG_MIBSPI1NCS_4_U10_PINMMR  88
#define PINMUX_SIG_MIBSPI1NCS_4_U10_SHIFT   0
#define PINMUX_SIG_MIBSPI1NCS_4_U10_VALUE   0x01
#define PINMUX_SIG_MIBSPI1NCS_4_U10_PIN     {88, 0, 0x01}

#define PINMUX_SIG_MIBSPI1NCS_4_N1_PINMMR  88
#define PINMUX_SIG_MIBSPI1NCS_4_N1_SHIFT   0
#define PINMUX_SIG_MIBSPI1NCS_4_N1_VALUE   0x02
#define PINMUX_SIG_MIBSPI1NCS_4_N1_PIN     {88, 0, 0x02}

/* Signal MIBSPI1NCS_5 driven from: U9 (0x01), P1 (0x02) */

#define PINMUX_SIG_MIBSPI1NCS_5_U9_PINMMR  88
#define PINMUX_SIG_MIBSPI1NCS_5_U9_SHIFT   8
#define PINMUX_SIG_MIBSPI1NCS_5_U9_VALUE   0x01
#define PINMUX_SIG_MIBSPI1NCS_5_U9_PIN     {88, 8, 0x01}

#define PINMUX_SIG_MIBSPI1NCS_5_P1_PINMMR  88
#define PINMUX_SIG_MIBSPI1NCS_5_P1_SHIFT   8
#define PINMUX_SIG_MIBSPI1NCS_5_P1_VALUE   0x02
#define PINMUX_SIG_MIBSPI1NCS_5_P1_PIN     {88, 8, 0x02}

/* PINMMR89: MII_COL[16], MII_CRS[24] */

/* Signal MII_COL driven from: W4 (0x01), F3 (0x02) */

#define PINMUX_SIG_MII_COL_W4_PINMMR       89
#define PINMUX_SIG_MII_COL_W4_SHIFT        16
#define PINMUX_SIG_MII_COL_W4_VALUE        0x01
#define PINMUX_SIG_MII_COL_W4_PIN          {89, 16, 0x01}

#define PINMUX_SIG_MII_COL_F3_PINMMR       89
#define PINMUX_SIG_MII_COL_F3_SHIFT        16
#define PINMUX_SIG_MII_COL_F3_VALUE        0x02
#define PINMUX_SIG_MII_COL_F3_PIN          {89, 16, 0x02}

/* Signal MII_CRS driven from: V4 (0x01), B4 (0x02) */

#define PINMUX_SIG_MII_CRS_V4_PINMMR       89
#define PINMUX_SIG_MII_CRS_V4_SHIFT        24
#define PINMUX_SIG_MII_CRS_V4_VALUE        0x01
#define PINMUX_SIG_MII_CRS_V4_PIN          {89, 24, 0x01}

#define PINMUX_SIG_MII_CRS_B4_PINMMR       89
#define PINMUX_SIG_MII_CRS_B4_SHIFT        24
#define PINMUX_SIG_MII_CRS_B4_VALUE        0x02
#define PINMUX_SIG_MII_CRS_B4_PIN          {89, 24, 0x02}

/* PINMMR90: MII_RX_DV[0], MII_RX_ER[8], MII_RXCLK[16], MII_RXD_0[24] */

/* Signal MII_RX_DV driven from: U6 (0x01), B11 (0x02) */

#define PINMUX_SIG_MII_RX_DV_U6_PINMMR     90
#define PINMUX_SIG_MII_RX_DV_U6_SHIFT      0
#define PINMUX_SIG_MII_RX_DV_U6_VALUE      0x01
#define PINMUX_SIG_MII_RX_DV_U6_PIN        {90, 0, 0x01}

#define PINMUX_SIG_MII_RX_DV_B11_PINMMR    90
#define PINMUX_SIG_MII_RX_DV_B11_SHIFT     0
#define PINMUX_SIG_MII_RX_DV_B11_VALUE     0x02
#define PINMUX_SIG_MII_RX_DV_B11_PIN       {90, 0, 0x02}

/* Signal MII_RX_ER driven from: U5 (0x01), N19 (0x02) */

#define PINMUX_SIG_MII_RX_ER_U5_PINMMR     90
#define PINMUX_SIG_MII_RX_ER_U5_SHIFT      8
#define PINMUX_SIG_MII_RX_ER_U5_VALUE      0x01
#define PINMUX_SIG_MII_RX_ER_U5_PIN        {90, 8, 0x01}

#define PINMUX_SIG_MII_RX_ER_N19_PINMMR    90
#define PINMUX_SIG_MII_RX_ER_N19_SHIFT     8
#define PINMUX_SIG_MII_RX_ER_N19_VALUE     0x02
#define PINMUX_SIG_MII_RX_ER_N19_PIN       {90, 8, 0x02}

/* Signal MII_RXCLK driven from: T4 (0x01), K19 (0x02) */

#define PINMUX_SIG_MII_RXCLK_T4_PINMMR     90
#define PINMUX_SIG_MII_RXCLK_T4_SHIFT      16
#define PINMUX_SIG_MII_RXCLK_T4_VALUE      0x01
#define PINMUX_SIG_MII_RXCLK_T4_PIN        {90, 16, 0x01}

#define PINMUX_SIG_MII_RXCLK_K19_PINMMR    90
#define PINMUX_SIG_MII_RXCLK_K19_SHIFT     16
#define PINMUX_SIG_MII_RXCLK_K19_VALUE     0x02
#define PINMUX_SIG_MII_RXCLK_K19_PIN       {90, 16, 0x02}

/* Signal MII_RXD_0 driven from: U4 (0x01), P1 (0x02) */

#define PINMUX_SIG_MII_RXD_0_U4_PINMMR     90
#define PINMUX_SIG_MII_RXD_0_U4_SHIFT      24
#define PINMUX_SIG_MII_RXD_0_U4_VALUE      0x01
#define PINMUX_SIG_MII_RXD_0_U4_PIN        {90, 24, 0x01}

#define PINMUX_SIG_MII_RXD_0_P1_PINMMR     90
#define PINMUX_SIG_MII_RXD_0_P1_SHIFT      24
#define PINMUX_SIG_MII_RXD_0_P1_VALUE      0x02
#define PINMUX_SIG_MII_RXD_0_P1_PIN        {90, 24, 0x02}

/* PINMMR91: MII_RXD_1[0], MII_RXD_2[8], MII_RXD_3[16], MII_TX_CLK[24] */

/* Signal MII_RXD_1 driven from: T3 (0x01), A14 (0x02) */

#define PINMUX_SIG_MII_RXD_1_T3_PINMMR     91
#define PINMUX_SIG_MII_RXD_1_T3_SHIFT      0
#define PINMUX_SIG_MII_RXD_1_T3_VALUE      0x01
#define PINMUX_SIG_MII_RXD_1_T3_PIN        {91, 0, 0x01}

#define PINMUX_SIG_MII_RXD_1_A14_PINMMR    91
#define PINMUX_SIG_MII_RXD_1_A14_SHIFT     0
#define PINMUX_SIG_MII_RXD_1_A14_VALUE     0x02
#define PINMUX_SIG_MII_RXD_1_A14_PIN       {91, 0, 0x02}

/* Signal MII_RXD_2 driven from: U3 (0x01), G19 (0x02) */

#define PINMUX_SIG_MII_RXD_2_U3_PINMMR     91
#define PINMUX_SIG_MII_RXD_2_U3_SHIFT      8
#define PINMUX_SIG_MII_RXD_2_U3_VALUE      0x01
#define PINMUX_SIG_MII_RXD_2_U3_PIN        {91, 8, 0x01}

#define PINMUX_SIG_MII_RXD_2_G19_PINMMR    91
#define PINMUX_SIG_MII_RXD_2_G19_SHIFT     8
#define PINMUX_SIG_MII_RXD_2_G19_VALUE     0x02
#define PINMUX_SIG_MII_RXD_2_G19_PIN       {91, 8, 0x02}

/* Signal MII_RXD_3 driven from: V3 (0x01), H18 (0x02) */

#define PINMUX_SIG_MII_RXD_3_V3_PINMMR     91
#define PINMUX_SIG_MII_RXD_3_V3_SHIFT      16
#define PINMUX_SIG_MII_RXD_3_V3_VALUE      0x01
#define PINMUX_SIG_MII_RXD_3_V3_PIN        {91, 16, 0x01}

#define PINMUX_SIG_MII_RXD_3_H18_PINMMR    91
#define PINMUX_SIG_MII_RXD_3_H18_SHIFT     16
#define PINMUX_SIG_MII_RXD_3_H18_VALUE     0x02
#define PINMUX_SIG_MII_RXD_3_H18_PIN       {91, 16, 0x02}

/* Signal MII_TX_CLK driven from: U7 (0x01), D19 (0x02) */

#define PINMUX_SIG_MII_TX_CLK_U7_PINMMR    91
#define PINMUX_SIG_MII_TX_CLK_U7_SHIFT     24
#define PINMUX_SIG_MII_TX_CLK_U7_VALUE     0x01
#define PINMUX_SIG_MII_TX_CLK_U7_PIN       {91, 24, 0x01}

#define PINMUX_SIG_MII_TX_CLK_D19_PINMMR   91
#define PINMUX_SIG_MII_TX_CLK_D19_SHIFT    24
#define PINMUX_SIG_MII_TX_CLK_D19_VALUE    0x02
#define PINMUX_SIG_MII_TX_CLK_D19_PIN      {91, 24, 0x02}

/* PINMMR92: N2HET1_17[0], N2HET1_19[8], N2HET1_21[16], N2HET1_23[24] */

/* Signal N2HET1_17 driven from: A13 (0x01), F3 (0x02) */

#define PINMUX_SIG_N2HET1_17_A13_PINMMR    92
#define PINMUX_SIG_N2HET1_17_A13_SHIFT     0
#define PINMUX_SIG_N2HET1_17_A13_VALUE     0x01
#define PINMUX_SIG_N2HET1_17_A13_PIN       {92, 0, 0x01}

#define PINMUX_SIG_N2HET1_17_F3_PINMMR     92
#define PINMUX_SIG_N2HET1_17_F3_SHIFT      0
#define PINMUX_SIG_N2HET1_17_F3_VALUE      0x02
#define PINMUX_SIG_N2HET1_17_F3_PIN        {92, 0, 0x02}

/* Signal N2HET1_19 driven from: B13 (0x01), G3 (0x02) */

#define PINMUX_SIG_N2HET1_19_B13_PINMMR    92
#define PINMUX_SIG_N2HET1_19_B13_SHIFT     8
#define PINMUX_SIG_N2HET1_19_B13_VALUE     0x01
#define PINMUX_SIG_N2HET1_19_B13_PIN       {92, 8, 0x01}

#define PINMUX_SIG_N2HET1_19_G3_PINMMR     92
#define PINMUX_SIG_N2HET1_19_G3_SHIFT      8
#define PINMUX_SIG_N2HET1_19_G3_VALUE      0x02
#define PINMUX_SIG_N2HET1_19_G3_PIN        {92, 8, 0x02}

/* Signal N2HET1_21 driven from: H4 (0x01), J3 (0x02) */

#define PINMUX_SIG_N2HET1_21_H4_PINMMR     92
#define PINMUX_SIG_N2HET1_21_H4_SHIFT      16
#define PINMUX_SIG_N2HET1_21_H4_VALUE      0x01
#define PINMUX_SIG_N2HET1_21_H4_PIN        {92, 16, 0x01}

#define PINMUX_SIG_N2HET1_21_J3_PINMMR     92
#define PINMUX_SIG_N2HET1_21_J3_SHIFT      16
#define PINMUX_SIG_N2HET1_21_J3_VALUE      0x02
#define PINMUX_SIG_N2HET1_21_J3_PIN        {92, 16, 0x02}

/* Signal N2HET1_23 driven from: J4 (0x01), G19 (0x02) */

#define PINMUX_SIG_N2HET1_23_J4_PINMMR     92
#define PINMUX_SIG_N2HET1_23_J4_SHIFT      24
#define PINMUX_SIG_N2HET1_23_J4_VALUE      0x01
#define PINMUX_SIG_N2HET1_23_J4_PIN        {92, 24, 0x01}

#define PINMUX_SIG_N2HET1_23_G19_PINMMR    92
#define PINMUX_SIG_N2HET1_23_G19_SHIFT     24
#define PINMUX_SIG_N2HET1_23_G19_VALUE     0x02
#define PINMUX_SIG_N2HET1_23_G19_PIN       {92, 24, 0x02}

/* PINMMR93: N2HET1_25[0], N2HET1_27[8], N2HET1_29[16], N2HET1_31[24] */

/* Signal N2HET1_25 driven from: M3 (0x01), V5 (0x02) */

#define PINMUX_SIG_N2HET1_25_M3_PINMMR     93
#define PINMUX_SIG_N2HET1_25_M3_SHIFT      0
#define PINMUX_SIG_N2HET1_25_M3_VALUE      0x01
#define PINMUX_SIG_N2HET1_25_M3_PIN        {93, 0, 0x01}

#define PINMUX_SIG_N2HET1_25_V5_PINMMR     93
#define PINMUX_SIG_N2HET1_25_V5_SHIFT      0
#define PINMUX_SIG_N2HET1_25_V5_VALUE      0x02
#define PINMUX_SIG_N2HET1_25_V5_PIN        {93, 0, 0x02}

/* Signal N2HET1_27 driven from: A9 (0x01), B2 (0x02) */

#define PINMUX_SIG_N2HET1_27_A9_PINMMR     93
#define PINMUX_SIG_N2HET1_27_A9_SHIFT      8
#define PINMUX_SIG_N2HET1_27_A9_VALUE      0x01
#define PINMUX_SIG_N2HET1_27_A9_PIN        {93, 8, 0x01}

#define PINMUX_SIG_N2HET1_27_B2_PINMMR     93
#define PINMUX_SIG_N2HET1_27_B2_SHIFT      8
#define PINMUX_SIG_N2HET1_27_B2_VALUE      0x02
#define PINMUX_SIG_N2HET1_27_B2_PIN        {93, 8, 0x02}

/* Signal N2HET1_29 driven from: A3 (0x01), C3 (0x02) */

#define PINMUX_SIG_N2HET1_29_A3_PINMMR     93
#define PINMUX_SIG_N2HET1_29_A3_SHIFT      16
#define PINMUX_SIG_N2HET1_29_A3_VALUE      0x01
#define PINMUX_SIG_N2HET1_29_A3_PIN        {93, 16, 0x01}

#define PINMUX_SIG_N2HET1_29_C3_PINMMR     93
#define PINMUX_SIG_N2HET1_29_C3_SHIFT      16
#define PINMUX_SIG_N2HET1_29_C3_VALUE      0x02
#define PINMUX_SIG_N2HET1_29_C3_PIN        {93, 16, 0x02}

/* Signal N2HET1_31 driven from: J17 (0x01), W9 (0x02) */

#define PINMUX_SIG_N2HET1_31_J17_PINMMR    93
#define PINMUX_SIG_N2HET1_31_J17_SHIFT     24
#define PINMUX_SIG_N2HET1_31_J17_VALUE     0x01
#define PINMUX_SIG_N2HET1_31_J17_PIN       {93, 24, 0x01}

#define PINMUX_SIG_N2HET1_31_W9_PINMMR     93
#define PINMUX_SIG_N2HET1_31_W9_SHIFT      24
#define PINMUX_SIG_N2HET1_31_W9_VALUE      0x02
#define PINMUX_SIG_N2HET1_31_W9_PIN        {93, 24, 0x02}

/* PINMMR94: N2HET2_00[0], N2HET2_01[8], N2HET2_02[16], N2HET2_03[24] */

/* Signal N2HET2_00 driven from: D6 (0x01), C1 (0x02) */

#define PINMUX_SIG_N2HET2_00_D6_PINMMR     94
#define PINMUX_SIG_N2HET2_00_D6_SHIFT      0
#define PINMUX_SIG_N2HET2_00_D6_VALUE      0x01
#define PINMUX_SIG_N2HET2_00_D6_PIN        {94, 0, 0x01}

#define PINMUX_SIG_N2HET2_00_C1_PINMMR     94
#define PINMUX_SIG_N2HET2_00_C1_SHIFT      0
#define PINMUX_SIG_N2HET2_00_C1_VALUE      0x02
#define PINMUX_SIG_N2HET2_00_C1_PIN        {94, 0, 0x02}

/* Signal N2HET2_01 driven from: D8 (0x01), D4 (0x02) */

#define PINMUX_SIG_N2HET2_01_D8_PINMMR     94
#define PINMUX_SIG_N2HET2_01_D8_SHIFT      8
#define PINMUX_SIG_N2HET2_01_D8_VALUE      0x01
#define PINMUX_SIG_N2HET2_01_D8_PIN        {94, 8, 0x01}

#define PINMUX_SIG_N2HET2_01_D4_PINMMR     94
#define PINMUX_SIG_N2HET2_01_D4_SHIFT      8
#define PINMUX_SIG_N2HET2_01_D4_VALUE      0x02
#define PINMUX_SIG_N2HET2_01_D4_PIN        {94, 8, 0x02}

/* Signal N2HET2_02 driven from: D7 (0x01), E1 (0x02) */

#define PINMUX_SIG_N2HET2_02_D7_PINMMR     94
#define PINMUX_SIG_N2HET2_02_D7_SHIFT      16
#define PINMUX_SIG_N2HET2_02_D7_VALUE      0x01
#define PINMUX_SIG_N2HET2_02_D7_PIN        {94, 16, 0x01}

#define PINMUX_SIG_N2HET2_02_E1_PINMMR     94
#define PINMUX_SIG_N2HET2_02_E1_SHIFT      16
#define PINMUX_SIG_N2HET2_02_E1_VALUE      0x02
#define PINMUX_SIG_N2HET2_02_E1_PIN        {94, 16, 0x02}

/* Signal N2HET2_03 driven from: E2 (0x01), D5 (0x02) */

#define PINMUX_SIG_N2HET2_03_E2_PINMMR     94
#define PINMUX_SIG_N2HET2_03_E2_SHIFT      24
#define PINMUX_SIG_N2HET2_03_E2_VALUE      0x01
#define PINMUX_SIG_N2HET2_03_E2_PIN        {94, 24, 0x01}

#define PINMUX_SIG_N2HET2_03_D5_PINMMR     94
#define PINMUX_SIG_N2HET2_03_D5_SHIFT      24
#define PINMUX_SIG_N2HET2_03_D5_VALUE      0x02
#define PINMUX_SIG_N2HET2_03_D5_PIN        {94, 24, 0x02}

/* PINMMR95: N2HET2_04[0], N2HET2_05[8], N2HET2_06[16], N2HET2_07[24] */

/* Signal N2HET2_04 driven from: D13 (0x01), H3 (0x02) */

#define PINMUX_SIG_N2HET2_04_D13_PINMMR    95
#define PINMUX_SIG_N2HET2_04_D13_SHIFT     0
#define PINMUX_SIG_N2HET2_04_D13_VALUE     0x01
#define PINMUX_SIG_N2HET2_04_D13_PIN       {95, 0, 0x01}

#define PINMUX_SIG_N2HET2_04_H3_PINMMR     95
#define PINMUX_SIG_N2HET2_04_H3_SHIFT      0
#define PINMUX_SIG_N2HET2_04_H3_VALUE      0x02
#define PINMUX_SIG_N2HET2_04_H3_PIN        {95, 0, 0x02}

/* Signal N2HET2_05 driven from: D12 (0x01), D16 (0x02) */

#define PINMUX_SIG_N2HET2_05_D12_PINMMR    95
#define PINMUX_SIG_N2HET2_05_D12_SHIFT     8
#define PINMUX_SIG_N2HET2_05_D12_VALUE     0x01
#define PINMUX_SIG_N2HET2_05_D12_PIN       {95, 8, 0x01}

#define PINMUX_SIG_N2HET2_05_D16_PINMMR    95
#define PINMUX_SIG_N2HET2_05_D16_SHIFT     8
#define PINMUX_SIG_N2HET2_05_D16_VALUE     0x02
#define PINMUX_SIG_N2HET2_05_D16_PIN       {95, 8, 0x02}

/* Signal N2HET2_06 driven from: D11 (0x01), M1 (0x02) */

#define PINMUX_SIG_N2HET2_06_D11_PINMMR    95
#define PINMUX_SIG_N2HET2_06_D11_SHIFT     16
#define PINMUX_SIG_N2HET2_06_D11_VALUE     0x01
#define PINMUX_SIG_N2HET2_06_D11_PIN       {95, 16, 0x01}

#define PINMUX_SIG_N2HET2_06_M1_PINMMR     95
#define PINMUX_SIG_N2HET2_06_M1_SHIFT      16
#define PINMUX_SIG_N2HET2_06_M1_VALUE      0x02
#define PINMUX_SIG_N2HET2_06_M1_PIN        {95, 16, 0x02}

/* Signal N2HET2_07 driven from: N3 (0x01), N17 (0x02) */

#define PINMUX_SIG_N2HET2_07_N3_PINMMR     95
#define PINMUX_SIG_N2HET2_07_N3_SHIFT      24
#define PINMUX_SIG_N2HET2_07_N3_VALUE      0x01
#define PINMUX_SIG_N2HET2_07_N3_PIN        {95, 24, 0x01}

#define PINMUX_SIG_N2HET2_07_N17_PINMMR    95
#define PINMUX_SIG_N2HET2_07_N17_SHIFT     24
#define PINMUX_SIG_N2HET2_07_N17_VALUE     0x02
#define PINMUX_SIG_N2HET2_07_N17_PIN       {95, 24, 0x02}

/* PINMMR96: N2HET2_08[0], N2HET2_09[8], N2HET2_10[16], N2HET2_11[24] */

/* Signal N2HET2_08 driven from: K16 (0x01), V2 (0x02) */

#define PINMUX_SIG_N2HET2_08_K16_PINMMR    96
#define PINMUX_SIG_N2HET2_08_K16_SHIFT     0
#define PINMUX_SIG_N2HET2_08_K16_VALUE     0x01
#define PINMUX_SIG_N2HET2_08_K16_PIN       {96, 0, 0x01}

#define PINMUX_SIG_N2HET2_08_V2_PINMMR     96
#define PINMUX_SIG_N2HET2_08_V2_SHIFT      0
#define PINMUX_SIG_N2HET2_08_V2_VALUE      0x02
#define PINMUX_SIG_N2HET2_08_V2_PIN        {96, 0, 0x02}

/* Signal N2HET2_09 driven from: L16 (0x01), K17 (0x02) */

#define PINMUX_SIG_N2HET2_09_L16_PINMMR    96
#define PINMUX_SIG_N2HET2_09_L16_SHIFT     8
#define PINMUX_SIG_N2HET2_09_L16_VALUE     0x01
#define PINMUX_SIG_N2HET2_09_L16_PIN       {96, 8, 0x01}

#define PINMUX_SIG_N2HET2_09_K17_PINMMR    96
#define PINMUX_SIG_N2HET2_09_K17_SHIFT     8
#define PINMUX_SIG_N2HET2_09_K17_VALUE     0x02
#define PINMUX_SIG_N2HET2_09_K17_PIN       {96, 8, 0x02}

/* Signal N2HET2_10 driven from: M16 (0x01), U1 (0x02) */

#define PINMUX_SIG_N2HET2_10_M16_PINMMR    96
#define PINMUX_SIG_N2HET2_10_M16_SHIFT     16
#define PINMUX_SIG_N2HET2_10_M16_VALUE     0x01
#define PINMUX_SIG_N2HET2_10_M16_PIN       {96, 16, 0x01}

#define PINMUX_SIG_N2HET2_10_U1_PINMMR     96
#define PINMUX_SIG_N2HET2_10_U1_SHIFT      16
#define PINMUX_SIG_N2HET2_10_U1_VALUE      0x02
#define PINMUX_SIG_N2HET2_10_U1_PIN        {96, 16, 0x02}

/* Signal N2HET2_11 driven from: N16 (0x01), C4 (0x02) */

#define PINMUX_SIG_N2HET2_11_N16_PINMMR    96
#define PINMUX_SIG_N2HET2_11_N16_SHIFT     24
#define PINMUX_SIG_N2HET2_11_N16_VALUE     0x01
#define PINMUX_SIG_N2HET2_11_N16_PIN       {96, 24, 0x01}

#define PINMUX_SIG_N2HET2_11_C4_PINMMR     96
#define PINMUX_SIG_N2HET2_11_C4_SHIFT      24
#define PINMUX_SIG_N2HET2_11_C4_VALUE      0x02
#define PINMUX_SIG_N2HET2_11_C4_PIN        {96, 24, 0x02}

/* PINMMR97: N2HET2_12[0], N2HET2_13[8], N2HET2_14[16], N2HET2_15[24] */

/* Signal N2HET2_12 driven from: D3 (0x01), V6 (0x02) */

#define PINMUX_SIG_N2HET2_12_D3_PINMMR     97
#define PINMUX_SIG_N2HET2_12_D3_SHIFT      0
#define PINMUX_SIG_N2HET2_12_D3_VALUE      0x01
#define PINMUX_SIG_N2HET2_12_D3_PIN        {97, 0, 0x01}

#define PINMUX_SIG_N2HET2_12_V6_PINMMR     97
#define PINMUX_SIG_N2HET2_12_V6_SHIFT      0
#define PINMUX_SIG_N2HET2_12_V6_VALUE      0x02
#define PINMUX_SIG_N2HET2_12_V6_PIN        {97, 0, 0x02}

/* Signal N2HET2_13 driven from: D2 (0x01), C5 (0x02) */

#define PINMUX_SIG_N2HET2_13_D2_PINMMR     97
#define PINMUX_SIG_N2HET2_13_D2_SHIFT      8
#define PINMUX_SIG_N2HET2_13_D2_VALUE      0x01
#define PINMUX_SIG_N2HET2_13_D2_PIN        {97, 8, 0x01}

#define PINMUX_SIG_N2HET2_13_C5_PINMMR     97
#define PINMUX_SIG_N2HET2_13_C5_SHIFT      8
#define PINMUX_SIG_N2HET2_13_C5_VALUE      0x02
#define PINMUX_SIG_N2HET2_13_C5_PIN        {97, 8, 0x02}

/* Signal N2HET2_14 driven from: D1 (0x01), T1 (0x02) */

#define PINMUX_SIG_N2HET2_14_D1_PINMMR     97
#define PINMUX_SIG_N2HET2_14_D1_SHIFT      16
#define PINMUX_SIG_N2HET2_14_D1_VALUE      0x01
#define PINMUX_SIG_N2HET2_14_D1_PIN        {97, 16, 0x01}

#define PINMUX_SIG_N2HET2_14_T1_PINMMR     97
#define PINMUX_SIG_N2HET2_14_T1_SHIFT      16
#define PINMUX_SIG_N2HET2_14_T1_VALUE      0x02
#define PINMUX_SIG_N2HET2_14_T1_PIN        {97, 16, 0x02}

/* Signal N2HET2_15 driven from: K4 (0x01), C6 (0x02) */

#define PINMUX_SIG_N2HET2_15_K4_PINMMR     97
#define PINMUX_SIG_N2HET2_15_K4_SHIFT      24
#define PINMUX_SIG_N2HET2_15_K4_VALUE      0x01
#define PINMUX_SIG_N2HET2_15_K4_PIN        {97, 24, 0x01}

#define PINMUX_SIG_N2HET2_15_C6_PINMMR     97
#define PINMUX_SIG_N2HET2_15_C6_SHIFT      24
#define PINMUX_SIG_N2HET2_15_C6_VALUE      0x02
#define PINMUX_SIG_N2HET2_15_C6_PIN        {97, 24, 0x02}

/* PINMMR98: N2HET2_16[0], N2HET2_18[8], N2HET2_20[16], N2HET2_22[24] */

/* Signal N2HET2_16 driven from: L4 (0x01), V7 (0x02) */

#define PINMUX_SIG_N2HET2_16_L4_PINMMR     98
#define PINMUX_SIG_N2HET2_16_L4_SHIFT      0
#define PINMUX_SIG_N2HET2_16_L4_VALUE      0x01
#define PINMUX_SIG_N2HET2_16_L4_PIN        {98, 0, 0x01}

#define PINMUX_SIG_N2HET2_16_V7_PINMMR     98
#define PINMUX_SIG_N2HET2_16_V7_SHIFT      0
#define PINMUX_SIG_N2HET2_16_V7_VALUE      0x02
#define PINMUX_SIG_N2HET2_16_V7_PIN        {98, 0, 0x02}

/* Signal N2HET2_18 driven from: N4 (0x01), E3 (0x02) */

#define PINMUX_SIG_N2HET2_18_N4_PINMMR     98
#define PINMUX_SIG_N2HET2_18_N4_SHIFT      8
#define PINMUX_SIG_N2HET2_18_N4_VALUE      0x01
#define PINMUX_SIG_N2HET2_18_N4_PIN        {98, 8, 0x01}

#define PINMUX_SIG_N2HET2_18_E3_PINMMR     98
#define PINMUX_SIG_N2HET2_18_E3_SHIFT      8
#define PINMUX_SIG_N2HET2_18_E3_VALUE      0x02
#define PINMUX_SIG_N2HET2_18_E3_PIN        {98, 8, 0x02}

/* Signal N2HET2_20 driven from: T5 (0x01), N2 (0x02) */

#define PINMUX_SIG_N2HET2_20_T5_PINMMR     98
#define PINMUX_SIG_N2HET2_20_T5_SHIFT      16
#define PINMUX_SIG_N2HET2_20_T5_VALUE      0x01
#define PINMUX_SIG_N2HET2_20_T5_PIN        {98, 16, 0x01}

#define PINMUX_SIG_N2HET2_20_N2_PINMMR     98
#define PINMUX_SIG_N2HET2_20_N2_SHIFT      16
#define PINMUX_SIG_N2HET2_20_N2_VALUE      0x02
#define PINMUX_SIG_N2HET2_20_N2_PIN        {98, 16, 0x02}

/* Signal N2HET2_22 driven from: T7 (0x01), N1 (0x02) */

#define PINMUX_SIG_N2HET2_22_T7_PINMMR     98
#define PINMUX_SIG_N2HET2_22_T7_SHIFT      24
#define PINMUX_SIG_N2HET2_22_T7_VALUE      0x01
#define PINMUX_SIG_N2HET2_22_T7_PIN        {98, 24, 0x01}

#define PINMUX_SIG_N2HET2_22_N1_PINMMR     98
#define PINMUX_SIG_N2HET2_22_N1_SHIFT      24
#define PINMUX_SIG_N2HET2_22_N1_VALUE      0x02
#define PINMUX_SIG_N2HET2_22_N1_PIN        {98, 24, 0x02}

/* PINMMR99: nTZ1_1[0], nTZ1_2[8], nTZ1_3[16] */

/* Signal nTZ1_1 driven from: N19 (0x01), C3 (0x02) */

#define PINMUX_SIG_NTZ1_1_N19_PINMMR       99
#define PINMUX_SIG_NTZ1_1_N19_SHIFT        0
#define PINMUX_SIG_NTZ1_1_N19_VALUE        0x01
#define PINMUX_SIG_NTZ1_1_N19_PIN          {99, 0, 0x01}

#define PINMUX_SIG_NTZ1_1_C3_PINMMR        99
#define PINMUX_SIG_NTZ1_1_C3_SHIFT         0
#define PINMUX_SIG_NTZ1_1_C3_VALUE         0x02
#define PINMUX_SIG_NTZ1_1_C3_PIN           {99, 0, 0x02}

/* Signal nTZ1_2 driven from: F1 (0x01), B2 (0x02) */

#define PINMUX_SIG_NTZ1_2_F1_PINMMR        99
#define PINMUX_SIG_NTZ1_2_F1_SHIFT         8
#define PINMUX_SIG_NTZ1_2_F1_VALUE         0x01
#define PINMUX_SIG_NTZ1_2_F1_PIN           {99, 8, 0x01}

#define PINMUX_SIG_NTZ1_2_B2_PINMMR        99
#define PINMUX_SIG_NTZ1_2_B2_SHIFT         8
#define PINMUX_SIG_NTZ1_2_B2_VALUE         0x02
#define PINMUX_SIG_NTZ1_2_B2_PIN           {99, 8, 0x02}

/* Signal nTZ1_3 driven from: J3 (0x01), D19 (0x02) */

#define PINMUX_SIG_NTZ1_3_J3_PINMMR        99
#define PINMUX_SIG_NTZ1_3_J3_SHIFT         16
#define PINMUX_SIG_NTZ1_3_J3_VALUE         0x01
#define PINMUX_SIG_NTZ1_3_J3_PIN           {99, 16, 0x01}

#define PINMUX_SIG_NTZ1_3_D19_PINMMR       99
#define PINMUX_SIG_NTZ1_3_D19_SHIFT        16
#define PINMUX_SIG_NTZ1_3_D19_VALUE        0x02
#define PINMUX_SIG_NTZ1_3_D19_PIN          {99, 16, 0x02}

/* Special Functionality Control (PINMMR160-179) ****************************/

/* These fields do not select a ball function; they enable or steer on-chip
 * functions.  GATE_EMIF_CLK is the exception: it lies in the output
 * multiplexing range and shares its field with ball K3, where selecting the
 * reserved function gates the EMIF clock off.  The eTPWM TBCLK_SYNC field is
 * omitted because HALCoGen never writes it, leaving its register index
 * unrecoverable; see SPNU562 if it is ever needed.
 */

/* GATE_EMIF_CLK (PINMMR9[0]): OFF (0x01), ON (0x02) */

#define PINMUX_GATE_EMIF_CLK_OFF_PINMMR    9
#define PINMUX_GATE_EMIF_CLK_OFF_SHIFT     0
#define PINMUX_GATE_EMIF_CLK_OFF_VALUE     0x01
#define PINMUX_GATE_EMIF_CLK_OFF_PIN       {9, 0, 0x01}

#define PINMUX_GATE_EMIF_CLK_ON_PINMMR     9
#define PINMUX_GATE_EMIF_CLK_ON_SHIFT      0
#define PINMUX_GATE_EMIF_CLK_ON_VALUE      0x02
#define PINMUX_GATE_EMIF_CLK_ON_PIN        {9, 0, 0x02}

/* ETHERNET (PINMMR160[24]): MII (0x00), RMII (0x01) */

#define PINMUX_ETHERNET_MII_PINMMR         160
#define PINMUX_ETHERNET_MII_SHIFT          24
#define PINMUX_ETHERNET_MII_VALUE          0x00
#define PINMUX_ETHERNET_MII_PIN            {160, 24, 0x00}

#define PINMUX_ETHERNET_RMII_PINMMR        160
#define PINMUX_ETHERNET_RMII_SHIFT         24
#define PINMUX_ETHERNET_RMII_VALUE         0x01
#define PINMUX_ETHERNET_RMII_PIN           {160, 24, 0x01}

/* ALT_ADC_TRIGGER (PINMMR161[0]): 1 (0x01), 2 (0x02) */

#define PINMUX_ALT_ADC_TRIGGER_1_PINMMR    161
#define PINMUX_ALT_ADC_TRIGGER_1_SHIFT     0
#define PINMUX_ALT_ADC_TRIGGER_1_VALUE     0x01
#define PINMUX_ALT_ADC_TRIGGER_1_PIN       {161, 0, 0x01}

#define PINMUX_ALT_ADC_TRIGGER_2_PINMMR    161
#define PINMUX_ALT_ADC_TRIGGER_2_SHIFT     0
#define PINMUX_ALT_ADC_TRIGGER_2_VALUE     0x02
#define PINMUX_ALT_ADC_TRIGGER_2_PIN       {161, 0, 0x02}

/* ETPWM_SOC1A (PINMMR164[0]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC1A_OFF_PINMMR      164
#define PINMUX_ETPWM_SOC1A_OFF_SHIFT       0
#define PINMUX_ETPWM_SOC1A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC1A_OFF_PIN         {164, 0, 0x00}

#define PINMUX_ETPWM_SOC1A_ON_PINMMR       164
#define PINMUX_ETPWM_SOC1A_ON_SHIFT        0
#define PINMUX_ETPWM_SOC1A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC1A_ON_PIN          {164, 0, 0x01}

/* ETPWM_SOC2A (PINMMR164[8]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC2A_OFF_PINMMR      164
#define PINMUX_ETPWM_SOC2A_OFF_SHIFT       8
#define PINMUX_ETPWM_SOC2A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC2A_OFF_PIN         {164, 8, 0x00}

#define PINMUX_ETPWM_SOC2A_ON_PINMMR       164
#define PINMUX_ETPWM_SOC2A_ON_SHIFT        8
#define PINMUX_ETPWM_SOC2A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC2A_ON_PIN          {164, 8, 0x01}

/* ETPWM_SOC3A (PINMMR164[16]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC3A_OFF_PINMMR      164
#define PINMUX_ETPWM_SOC3A_OFF_SHIFT       16
#define PINMUX_ETPWM_SOC3A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC3A_OFF_PIN         {164, 16, 0x00}

#define PINMUX_ETPWM_SOC3A_ON_PINMMR       164
#define PINMUX_ETPWM_SOC3A_ON_SHIFT        16
#define PINMUX_ETPWM_SOC3A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC3A_ON_PIN          {164, 16, 0x01}

/* ETPWM_SOC4A (PINMMR164[24]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC4A_OFF_PINMMR      164
#define PINMUX_ETPWM_SOC4A_OFF_SHIFT       24
#define PINMUX_ETPWM_SOC4A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC4A_OFF_PIN         {164, 24, 0x00}

#define PINMUX_ETPWM_SOC4A_ON_PINMMR       164
#define PINMUX_ETPWM_SOC4A_ON_SHIFT        24
#define PINMUX_ETPWM_SOC4A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC4A_ON_PIN          {164, 24, 0x01}

/* ETPWM_SOC5A (PINMMR165[0]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC5A_OFF_PINMMR      165
#define PINMUX_ETPWM_SOC5A_OFF_SHIFT       0
#define PINMUX_ETPWM_SOC5A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC5A_OFF_PIN         {165, 0, 0x00}

#define PINMUX_ETPWM_SOC5A_ON_PINMMR       165
#define PINMUX_ETPWM_SOC5A_ON_SHIFT        0
#define PINMUX_ETPWM_SOC5A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC5A_ON_PIN          {165, 0, 0x01}

/* ETPWM_SOC6A (PINMMR165[8]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC6A_OFF_PINMMR      165
#define PINMUX_ETPWM_SOC6A_OFF_SHIFT       8
#define PINMUX_ETPWM_SOC6A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC6A_OFF_PIN         {165, 8, 0x00}

#define PINMUX_ETPWM_SOC6A_ON_PINMMR       165
#define PINMUX_ETPWM_SOC6A_ON_SHIFT        8
#define PINMUX_ETPWM_SOC6A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC6A_ON_PIN          {165, 8, 0x01}

/* ETPWM_SOC7A (PINMMR165[16]): OFF (0x00), ON (0x01) */

#define PINMUX_ETPWM_SOC7A_OFF_PINMMR      165
#define PINMUX_ETPWM_SOC7A_OFF_SHIFT       16
#define PINMUX_ETPWM_SOC7A_OFF_VALUE       0x00
#define PINMUX_ETPWM_SOC7A_OFF_PIN         {165, 16, 0x00}

#define PINMUX_ETPWM_SOC7A_ON_PINMMR       165
#define PINMUX_ETPWM_SOC7A_ON_SHIFT        16
#define PINMUX_ETPWM_SOC7A_ON_VALUE        0x01
#define PINMUX_ETPWM_SOC7A_ON_PIN          {165, 16, 0x01}

/* ETPWM_TIME_BASE_SYNC (PINMMR165[24]): OFF (0x01), ON (0x02) */

#define PINMUX_ETPWM_TIME_BASE_SYNC_OFF_PINMMR  165
#define PINMUX_ETPWM_TIME_BASE_SYNC_OFF_SHIFT   24
#define PINMUX_ETPWM_TIME_BASE_SYNC_OFF_VALUE   0x01
#define PINMUX_ETPWM_TIME_BASE_SYNC_OFF_PIN     {165, 24, 0x01}

#define PINMUX_ETPWM_TIME_BASE_SYNC_ON_PINMMR  165
#define PINMUX_ETPWM_TIME_BASE_SYNC_ON_SHIFT   24
#define PINMUX_ETPWM_TIME_BASE_SYNC_ON_VALUE   0x02
#define PINMUX_ETPWM_TIME_BASE_SYNC_ON_PIN     {165, 24, 0x02}

/* ETPWM1 (PINMMR167[0]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM1_EQEPERR12_PINMMR     167
#define PINMUX_ETPWM1_EQEPERR12_SHIFT      0
#define PINMUX_ETPWM1_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM1_EQEPERR12_PIN        {167, 0, 0x01}

#define PINMUX_ETPWM1_EQEPERR1_PINMMR      167
#define PINMUX_ETPWM1_EQEPERR1_SHIFT       0
#define PINMUX_ETPWM1_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM1_EQEPERR1_PIN         {167, 0, 0x02}

#define PINMUX_ETPWM1_EQEPERR2_PINMMR      167
#define PINMUX_ETPWM1_EQEPERR2_SHIFT       0
#define PINMUX_ETPWM1_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM1_EQEPERR2_PIN         {167, 0, 0x04}

/* ETPWM2 (PINMMR167[8]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM2_EQEPERR12_PINMMR     167
#define PINMUX_ETPWM2_EQEPERR12_SHIFT      8
#define PINMUX_ETPWM2_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM2_EQEPERR12_PIN        {167, 8, 0x01}

#define PINMUX_ETPWM2_EQEPERR1_PINMMR      167
#define PINMUX_ETPWM2_EQEPERR1_SHIFT       8
#define PINMUX_ETPWM2_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM2_EQEPERR1_PIN         {167, 8, 0x02}

#define PINMUX_ETPWM2_EQEPERR2_PINMMR      167
#define PINMUX_ETPWM2_EQEPERR2_SHIFT       8
#define PINMUX_ETPWM2_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM2_EQEPERR2_PIN         {167, 8, 0x04}

/* ETPWM3 (PINMMR167[16]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM3_EQEPERR12_PINMMR     167
#define PINMUX_ETPWM3_EQEPERR12_SHIFT      16
#define PINMUX_ETPWM3_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM3_EQEPERR12_PIN        {167, 16, 0x01}

#define PINMUX_ETPWM3_EQEPERR1_PINMMR      167
#define PINMUX_ETPWM3_EQEPERR1_SHIFT       16
#define PINMUX_ETPWM3_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM3_EQEPERR1_PIN         {167, 16, 0x02}

#define PINMUX_ETPWM3_EQEPERR2_PINMMR      167
#define PINMUX_ETPWM3_EQEPERR2_SHIFT       16
#define PINMUX_ETPWM3_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM3_EQEPERR2_PIN         {167, 16, 0x04}

/* ETPWM4 (PINMMR167[24]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM4_EQEPERR12_PINMMR     167
#define PINMUX_ETPWM4_EQEPERR12_SHIFT      24
#define PINMUX_ETPWM4_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM4_EQEPERR12_PIN        {167, 24, 0x01}

#define PINMUX_ETPWM4_EQEPERR1_PINMMR      167
#define PINMUX_ETPWM4_EQEPERR1_SHIFT       24
#define PINMUX_ETPWM4_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM4_EQEPERR1_PIN         {167, 24, 0x02}

#define PINMUX_ETPWM4_EQEPERR2_PINMMR      167
#define PINMUX_ETPWM4_EQEPERR2_SHIFT       24
#define PINMUX_ETPWM4_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM4_EQEPERR2_PIN         {167, 24, 0x04}

/* ETPWM5 (PINMMR168[0]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM5_EQEPERR12_PINMMR     168
#define PINMUX_ETPWM5_EQEPERR12_SHIFT      0
#define PINMUX_ETPWM5_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM5_EQEPERR12_PIN        {168, 0, 0x01}

#define PINMUX_ETPWM5_EQEPERR1_PINMMR      168
#define PINMUX_ETPWM5_EQEPERR1_SHIFT       0
#define PINMUX_ETPWM5_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM5_EQEPERR1_PIN         {168, 0, 0x02}

#define PINMUX_ETPWM5_EQEPERR2_PINMMR      168
#define PINMUX_ETPWM5_EQEPERR2_SHIFT       0
#define PINMUX_ETPWM5_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM5_EQEPERR2_PIN         {168, 0, 0x04}

/* ETPWM6 (PINMMR168[8]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM6_EQEPERR12_PINMMR     168
#define PINMUX_ETPWM6_EQEPERR12_SHIFT      8
#define PINMUX_ETPWM6_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM6_EQEPERR12_PIN        {168, 8, 0x01}

#define PINMUX_ETPWM6_EQEPERR1_PINMMR      168
#define PINMUX_ETPWM6_EQEPERR1_SHIFT       8
#define PINMUX_ETPWM6_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM6_EQEPERR1_PIN         {168, 8, 0x02}

#define PINMUX_ETPWM6_EQEPERR2_PINMMR      168
#define PINMUX_ETPWM6_EQEPERR2_SHIFT       8
#define PINMUX_ETPWM6_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM6_EQEPERR2_PIN         {168, 8, 0x04}

/* ETPWM7 (PINMMR168[16]): EQEPERR12 (0x01), EQEPERR1 (0x02), EQEPERR2 (0x04)
 */

#define PINMUX_ETPWM7_EQEPERR12_PINMMR     168
#define PINMUX_ETPWM7_EQEPERR12_SHIFT      16
#define PINMUX_ETPWM7_EQEPERR12_VALUE      0x01
#define PINMUX_ETPWM7_EQEPERR12_PIN        {168, 16, 0x01}

#define PINMUX_ETPWM7_EQEPERR1_PINMMR      168
#define PINMUX_ETPWM7_EQEPERR1_SHIFT       16
#define PINMUX_ETPWM7_EQEPERR1_VALUE       0x02
#define PINMUX_ETPWM7_EQEPERR1_PIN         {168, 16, 0x02}

#define PINMUX_ETPWM7_EQEPERR2_PINMMR      168
#define PINMUX_ETPWM7_EQEPERR2_SHIFT       16
#define PINMUX_ETPWM7_EQEPERR2_VALUE       0x04
#define PINMUX_ETPWM7_EQEPERR2_PIN         {168, 16, 0x04}

/* ECAP1_FILTER (PINMMR169[0]): OFF (0x00), ON (0x02) */

#define PINMUX_ECAP1_FILTER_OFF_PINMMR     169
#define PINMUX_ECAP1_FILTER_OFF_SHIFT      0
#define PINMUX_ECAP1_FILTER_OFF_VALUE      0x00
#define PINMUX_ECAP1_FILTER_OFF_PIN        {169, 0, 0x00}

#define PINMUX_ECAP1_FILTER_ON_PINMMR      169
#define PINMUX_ECAP1_FILTER_ON_SHIFT       0
#define PINMUX_ECAP1_FILTER_ON_VALUE       0x02
#define PINMUX_ECAP1_FILTER_ON_PIN         {169, 0, 0x02}

/* ECAP2_FILTER (PINMMR169[8]): OFF (0x00), ON (0x02) */

#define PINMUX_ECAP2_FILTER_OFF_PINMMR     169
#define PINMUX_ECAP2_FILTER_OFF_SHIFT      8
#define PINMUX_ECAP2_FILTER_OFF_VALUE      0x00
#define PINMUX_ECAP2_FILTER_OFF_PIN        {169, 8, 0x00}

#define PINMUX_ECAP2_FILTER_ON_PINMMR      169
#define PINMUX_ECAP2_FILTER_ON_SHIFT       8
#define PINMUX_ECAP2_FILTER_ON_VALUE       0x02
#define PINMUX_ECAP2_FILTER_ON_PIN         {169, 8, 0x02}

/* ECAP3_FILTER (PINMMR169[16]): OFF (0x00), ON (0x02) */

#define PINMUX_ECAP3_FILTER_OFF_PINMMR     169
#define PINMUX_ECAP3_FILTER_OFF_SHIFT      16
#define PINMUX_ECAP3_FILTER_OFF_VALUE      0x00
#define PINMUX_ECAP3_FILTER_OFF_PIN        {169, 16, 0x00}

#define PINMUX_ECAP3_FILTER_ON_PINMMR      169
#define PINMUX_ECAP3_FILTER_ON_SHIFT       16
#define PINMUX_ECAP3_FILTER_ON_VALUE       0x02
#define PINMUX_ECAP3_FILTER_ON_PIN         {169, 16, 0x02}

/* ECAP4_FILTER (PINMMR169[24]): OFF (0x00), ON (0x02) */

#define PINMUX_ECAP4_FILTER_OFF_PINMMR     169
#define PINMUX_ECAP4_FILTER_OFF_SHIFT      24
#define PINMUX_ECAP4_FILTER_OFF_VALUE      0x00
#define PINMUX_ECAP4_FILTER_OFF_PIN        {169, 24, 0x00}

#define PINMUX_ECAP4_FILTER_ON_PINMMR      169
#define PINMUX_ECAP4_FILTER_ON_SHIFT       24
#define PINMUX_ECAP4_FILTER_ON_VALUE       0x02
#define PINMUX_ECAP4_FILTER_ON_PIN         {169, 24, 0x02}

/* ECAP5_FILTER (PINMMR170[0]): OFF (0x00), ON (0x02) */

#define PINMUX_ECAP5_FILTER_OFF_PINMMR     170
#define PINMUX_ECAP5_FILTER_OFF_SHIFT      0
#define PINMUX_ECAP5_FILTER_OFF_VALUE      0x00
#define PINMUX_ECAP5_FILTER_OFF_PIN        {170, 0, 0x00}

#define PINMUX_ECAP5_FILTER_ON_PINMMR      170
#define PINMUX_ECAP5_FILTER_ON_SHIFT       0
#define PINMUX_ECAP5_FILTER_ON_VALUE       0x02
#define PINMUX_ECAP5_FILTER_ON_PIN         {170, 0, 0x02}

/* ECAP6_FILTER (PINMMR170[8]): OFF (0x00), ON (0x02) */

#define PINMUX_ECAP6_FILTER_OFF_PINMMR     170
#define PINMUX_ECAP6_FILTER_OFF_SHIFT      8
#define PINMUX_ECAP6_FILTER_OFF_VALUE      0x00
#define PINMUX_ECAP6_FILTER_OFF_PIN        {170, 8, 0x00}

#define PINMUX_ECAP6_FILTER_ON_PINMMR      170
#define PINMUX_ECAP6_FILTER_ON_SHIFT       8
#define PINMUX_ECAP6_FILTER_ON_VALUE       0x02
#define PINMUX_ECAP6_FILTER_ON_PIN         {170, 8, 0x02}

/* EQEP1A_FILTER (PINMMR170[16]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP1A_FILTER_OFF_PINMMR    170
#define PINMUX_EQEP1A_FILTER_OFF_SHIFT     16
#define PINMUX_EQEP1A_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP1A_FILTER_OFF_PIN       {170, 16, 0x00}

#define PINMUX_EQEP1A_FILTER_ON_PINMMR     170
#define PINMUX_EQEP1A_FILTER_ON_SHIFT      16
#define PINMUX_EQEP1A_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP1A_FILTER_ON_PIN        {170, 16, 0x02}

/* EQEP1B_FILTER (PINMMR170[24]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP1B_FILTER_OFF_PINMMR    170
#define PINMUX_EQEP1B_FILTER_OFF_SHIFT     24
#define PINMUX_EQEP1B_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP1B_FILTER_OFF_PIN       {170, 24, 0x00}

#define PINMUX_EQEP1B_FILTER_ON_PINMMR     170
#define PINMUX_EQEP1B_FILTER_ON_SHIFT      24
#define PINMUX_EQEP1B_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP1B_FILTER_ON_PIN        {170, 24, 0x02}

/* EQEP1I_FILTER (PINMMR171[0]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP1I_FILTER_OFF_PINMMR    171
#define PINMUX_EQEP1I_FILTER_OFF_SHIFT     0
#define PINMUX_EQEP1I_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP1I_FILTER_OFF_PIN       {171, 0, 0x00}

#define PINMUX_EQEP1I_FILTER_ON_PINMMR     171
#define PINMUX_EQEP1I_FILTER_ON_SHIFT      0
#define PINMUX_EQEP1I_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP1I_FILTER_ON_PIN        {171, 0, 0x02}

/* EQEP1S_FILTER (PINMMR171[8]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP1S_FILTER_OFF_PINMMR    171
#define PINMUX_EQEP1S_FILTER_OFF_SHIFT     8
#define PINMUX_EQEP1S_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP1S_FILTER_OFF_PIN       {171, 8, 0x00}

#define PINMUX_EQEP1S_FILTER_ON_PINMMR     171
#define PINMUX_EQEP1S_FILTER_ON_SHIFT      8
#define PINMUX_EQEP1S_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP1S_FILTER_ON_PIN        {171, 8, 0x02}

/* EQEP2A_FILTER (PINMMR171[16]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP2A_FILTER_OFF_PINMMR    171
#define PINMUX_EQEP2A_FILTER_OFF_SHIFT     16
#define PINMUX_EQEP2A_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP2A_FILTER_OFF_PIN       {171, 16, 0x00}

#define PINMUX_EQEP2A_FILTER_ON_PINMMR     171
#define PINMUX_EQEP2A_FILTER_ON_SHIFT      16
#define PINMUX_EQEP2A_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP2A_FILTER_ON_PIN        {171, 16, 0x02}

/* EQEP2B_FILTER (PINMMR171[24]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP2B_FILTER_OFF_PINMMR    171
#define PINMUX_EQEP2B_FILTER_OFF_SHIFT     24
#define PINMUX_EQEP2B_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP2B_FILTER_OFF_PIN       {171, 24, 0x00}

#define PINMUX_EQEP2B_FILTER_ON_PINMMR     171
#define PINMUX_EQEP2B_FILTER_ON_SHIFT      24
#define PINMUX_EQEP2B_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP2B_FILTER_ON_PIN        {171, 24, 0x02}

/* EQEP2I_FILTER (PINMMR172[0]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP2I_FILTER_OFF_PINMMR    172
#define PINMUX_EQEP2I_FILTER_OFF_SHIFT     0
#define PINMUX_EQEP2I_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP2I_FILTER_OFF_PIN       {172, 0, 0x00}

#define PINMUX_EQEP2I_FILTER_ON_PINMMR     172
#define PINMUX_EQEP2I_FILTER_ON_SHIFT      0
#define PINMUX_EQEP2I_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP2I_FILTER_ON_PIN        {172, 0, 0x02}

/* EQEP2S_FILTER (PINMMR172[8]): OFF (0x00), ON (0x02) */

#define PINMUX_EQEP2S_FILTER_OFF_PINMMR    172
#define PINMUX_EQEP2S_FILTER_OFF_SHIFT     8
#define PINMUX_EQEP2S_FILTER_OFF_VALUE     0x00
#define PINMUX_EQEP2S_FILTER_OFF_PIN       {172, 8, 0x00}

#define PINMUX_EQEP2S_FILTER_ON_PINMMR     172
#define PINMUX_EQEP2S_FILTER_ON_SHIFT      8
#define PINMUX_EQEP2S_FILTER_ON_VALUE      0x02
#define PINMUX_EQEP2S_FILTER_ON_PIN        {172, 8, 0x02}

/* TZ1 (PINMMR172[16]): ASYNC (0x01), SYNC (0x02), FILTERED (0x04) */

#define PINMUX_TZ1_ASYNC_PINMMR            172
#define PINMUX_TZ1_ASYNC_SHIFT             16
#define PINMUX_TZ1_ASYNC_VALUE             0x01
#define PINMUX_TZ1_ASYNC_PIN               {172, 16, 0x01}

#define PINMUX_TZ1_SYNC_PINMMR             172
#define PINMUX_TZ1_SYNC_SHIFT              16
#define PINMUX_TZ1_SYNC_VALUE              0x02
#define PINMUX_TZ1_SYNC_PIN                {172, 16, 0x02}

#define PINMUX_TZ1_FILTERED_PINMMR         172
#define PINMUX_TZ1_FILTERED_SHIFT          16
#define PINMUX_TZ1_FILTERED_VALUE          0x04
#define PINMUX_TZ1_FILTERED_PIN            {172, 16, 0x04}

/* TZ2 (PINMMR172[24]): ASYNC (0x01), SYNC (0x02), FILTERED (0x04) */

#define PINMUX_TZ2_ASYNC_PINMMR            172
#define PINMUX_TZ2_ASYNC_SHIFT             24
#define PINMUX_TZ2_ASYNC_VALUE             0x01
#define PINMUX_TZ2_ASYNC_PIN               {172, 24, 0x01}

#define PINMUX_TZ2_SYNC_PINMMR             172
#define PINMUX_TZ2_SYNC_SHIFT              24
#define PINMUX_TZ2_SYNC_VALUE              0x02
#define PINMUX_TZ2_SYNC_PIN                {172, 24, 0x02}

#define PINMUX_TZ2_FILTERED_PINMMR         172
#define PINMUX_TZ2_FILTERED_SHIFT          24
#define PINMUX_TZ2_FILTERED_VALUE          0x04
#define PINMUX_TZ2_FILTERED_PIN            {172, 24, 0x04}

/* TZ3 (PINMMR173[0]): ASYNC (0x01), SYNC (0x02), FILTERED (0x04) */

#define PINMUX_TZ3_ASYNC_PINMMR            173
#define PINMUX_TZ3_ASYNC_SHIFT             0
#define PINMUX_TZ3_ASYNC_VALUE             0x01
#define PINMUX_TZ3_ASYNC_PIN               {173, 0, 0x01}

#define PINMUX_TZ3_SYNC_PINMMR             173
#define PINMUX_TZ3_SYNC_SHIFT              0
#define PINMUX_TZ3_SYNC_VALUE              0x02
#define PINMUX_TZ3_SYNC_PIN                {173, 0, 0x02}

#define PINMUX_TZ3_FILTERED_PINMMR         173
#define PINMUX_TZ3_FILTERED_SHIFT          0
#define PINMUX_TZ3_FILTERED_VALUE          0x04
#define PINMUX_TZ3_FILTERED_PIN            {173, 0, 0x04}

/* EPWM1SYNCI (PINMMR173[8]): ASYNC (0x01), SYNC (0x02), FILTERED (0x04) */

#define PINMUX_EPWM1SYNCI_ASYNC_PINMMR     173
#define PINMUX_EPWM1SYNCI_ASYNC_SHIFT      8
#define PINMUX_EPWM1SYNCI_ASYNC_VALUE      0x01
#define PINMUX_EPWM1SYNCI_ASYNC_PIN        {173, 8, 0x01}

#define PINMUX_EPWM1SYNCI_SYNC_PINMMR      173
#define PINMUX_EPWM1SYNCI_SYNC_SHIFT       8
#define PINMUX_EPWM1SYNCI_SYNC_VALUE       0x02
#define PINMUX_EPWM1SYNCI_SYNC_PIN         {173, 8, 0x02}

#define PINMUX_EPWM1SYNCI_FILTERED_PINMMR  173
#define PINMUX_EPWM1SYNCI_FILTERED_SHIFT   8
#define PINMUX_EPWM1SYNCI_FILTERED_VALUE   0x04
#define PINMUX_EPWM1SYNCI_FILTERED_PIN     {173, 8, 0x04}

/* TEMP1_ENABLE (PINMMR173[16]): OFF (0x01), ON (0x02) */

#define PINMUX_TEMP1_ENABLE_OFF_PINMMR     173
#define PINMUX_TEMP1_ENABLE_OFF_SHIFT      16
#define PINMUX_TEMP1_ENABLE_OFF_VALUE      0x01
#define PINMUX_TEMP1_ENABLE_OFF_PIN        {173, 16, 0x01}

#define PINMUX_TEMP1_ENABLE_ON_PINMMR      173
#define PINMUX_TEMP1_ENABLE_ON_SHIFT       16
#define PINMUX_TEMP1_ENABLE_ON_VALUE       0x02
#define PINMUX_TEMP1_ENABLE_ON_PIN         {173, 16, 0x02}

/* TEMP2_ENABLE (PINMMR173[24]): OFF (0x01), ON (0x02) */

#define PINMUX_TEMP2_ENABLE_OFF_PINMMR     173
#define PINMUX_TEMP2_ENABLE_OFF_SHIFT      24
#define PINMUX_TEMP2_ENABLE_OFF_VALUE      0x01
#define PINMUX_TEMP2_ENABLE_OFF_PIN        {173, 24, 0x01}

#define PINMUX_TEMP2_ENABLE_ON_PINMMR      173
#define PINMUX_TEMP2_ENABLE_ON_SHIFT       24
#define PINMUX_TEMP2_ENABLE_ON_VALUE       0x02
#define PINMUX_TEMP2_ENABLE_ON_PIN         {173, 24, 0x02}

/* TEMP3_ENABLE (PINMMR174[0]): OFF (0x01), ON (0x02) */

#define PINMUX_TEMP3_ENABLE_OFF_PINMMR     174
#define PINMUX_TEMP3_ENABLE_OFF_SHIFT      0
#define PINMUX_TEMP3_ENABLE_OFF_VALUE      0x01
#define PINMUX_TEMP3_ENABLE_OFF_PIN        {174, 0, 0x01}

#define PINMUX_TEMP3_ENABLE_ON_PINMMR      174
#define PINMUX_TEMP3_ENABLE_ON_SHIFT       0
#define PINMUX_TEMP3_ENABLE_ON_VALUE       0x02
#define PINMUX_TEMP3_ENABLE_ON_PIN         {174, 0, 0x02}

/* TEMP_PWRDN (PINMMR174[24]): OFF (0x00), ON (0x01) */

#define PINMUX_TEMP_PWRDN_OFF_PINMMR       174
#define PINMUX_TEMP_PWRDN_OFF_SHIFT        24
#define PINMUX_TEMP_PWRDN_OFF_VALUE        0x00
#define PINMUX_TEMP_PWRDN_OFF_PIN          {174, 24, 0x00}

#define PINMUX_TEMP_PWRDN_ON_PINMMR        174
#define PINMUX_TEMP_PWRDN_ON_SHIFT         24
#define PINMUX_TEMP_PWRDN_ON_VALUE         0x01
#define PINMUX_TEMP_PWRDN_ON_PIN           {174, 24, 0x01}

/* EMIF_OUTPUT_ENABLE (PINMMR174[8]): OFF (0x01), ON (0x02) */

#define PINMUX_EMIF_OUTPUT_ENABLE_OFF_PINMMR  174
#define PINMUX_EMIF_OUTPUT_ENABLE_OFF_SHIFT   8
#define PINMUX_EMIF_OUTPUT_ENABLE_OFF_VALUE   0x01
#define PINMUX_EMIF_OUTPUT_ENABLE_OFF_PIN     {174, 8, 0x01}

#define PINMUX_EMIF_OUTPUT_ENABLE_ON_PINMMR  174
#define PINMUX_EMIF_OUTPUT_ENABLE_ON_SHIFT   8
#define PINMUX_EMIF_OUTPUT_ENABLE_ON_VALUE   0x02
#define PINMUX_EMIF_OUTPUT_ENABLE_ON_PIN     {174, 8, 0x02}

/* GIOA0_DMA (PINMMR175[0]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA0_DMA_ON_PINMMR         175
#define PINMUX_GIOA0_DMA_ON_SHIFT          0
#define PINMUX_GIOA0_DMA_ON_VALUE          0x00
#define PINMUX_GIOA0_DMA_ON_PIN            {175, 0, 0x00}

#define PINMUX_GIOA0_DMA_OFF_PINMMR        175
#define PINMUX_GIOA0_DMA_OFF_SHIFT         0
#define PINMUX_GIOA0_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA0_DMA_OFF_PIN           {175, 0, 0x01}

/* GIOA1_DMA (PINMMR175[8]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA1_DMA_ON_PINMMR         175
#define PINMUX_GIOA1_DMA_ON_SHIFT          8
#define PINMUX_GIOA1_DMA_ON_VALUE          0x00
#define PINMUX_GIOA1_DMA_ON_PIN            {175, 8, 0x00}

#define PINMUX_GIOA1_DMA_OFF_PINMMR        175
#define PINMUX_GIOA1_DMA_OFF_SHIFT         8
#define PINMUX_GIOA1_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA1_DMA_OFF_PIN           {175, 8, 0x01}

/* GIOA2_DMA (PINMMR175[16]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA2_DMA_ON_PINMMR         175
#define PINMUX_GIOA2_DMA_ON_SHIFT          16
#define PINMUX_GIOA2_DMA_ON_VALUE          0x00
#define PINMUX_GIOA2_DMA_ON_PIN            {175, 16, 0x00}

#define PINMUX_GIOA2_DMA_OFF_PINMMR        175
#define PINMUX_GIOA2_DMA_OFF_SHIFT         16
#define PINMUX_GIOA2_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA2_DMA_OFF_PIN           {175, 16, 0x01}

/* GIOA3_DMA (PINMMR175[24]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA3_DMA_ON_PINMMR         175
#define PINMUX_GIOA3_DMA_ON_SHIFT          24
#define PINMUX_GIOA3_DMA_ON_VALUE          0x00
#define PINMUX_GIOA3_DMA_ON_PIN            {175, 24, 0x00}

#define PINMUX_GIOA3_DMA_OFF_PINMMR        175
#define PINMUX_GIOA3_DMA_OFF_SHIFT         24
#define PINMUX_GIOA3_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA3_DMA_OFF_PIN           {175, 24, 0x01}

/* GIOA4_DMA (PINMMR176[0]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA4_DMA_ON_PINMMR         176
#define PINMUX_GIOA4_DMA_ON_SHIFT          0
#define PINMUX_GIOA4_DMA_ON_VALUE          0x00
#define PINMUX_GIOA4_DMA_ON_PIN            {176, 0, 0x00}

#define PINMUX_GIOA4_DMA_OFF_PINMMR        176
#define PINMUX_GIOA4_DMA_OFF_SHIFT         0
#define PINMUX_GIOA4_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA4_DMA_OFF_PIN           {176, 0, 0x01}

/* GIOA5_DMA (PINMMR176[8]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA5_DMA_ON_PINMMR         176
#define PINMUX_GIOA5_DMA_ON_SHIFT          8
#define PINMUX_GIOA5_DMA_ON_VALUE          0x00
#define PINMUX_GIOA5_DMA_ON_PIN            {176, 8, 0x00}

#define PINMUX_GIOA5_DMA_OFF_PINMMR        176
#define PINMUX_GIOA5_DMA_OFF_SHIFT         8
#define PINMUX_GIOA5_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA5_DMA_OFF_PIN           {176, 8, 0x01}

/* GIOA6_DMA (PINMMR176[16]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA6_DMA_ON_PINMMR         176
#define PINMUX_GIOA6_DMA_ON_SHIFT          16
#define PINMUX_GIOA6_DMA_ON_VALUE          0x00
#define PINMUX_GIOA6_DMA_ON_PIN            {176, 16, 0x00}

#define PINMUX_GIOA6_DMA_OFF_PINMMR        176
#define PINMUX_GIOA6_DMA_OFF_SHIFT         16
#define PINMUX_GIOA6_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA6_DMA_OFF_PIN           {176, 16, 0x01}

/* GIOA7_DMA (PINMMR176[24]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOA7_DMA_ON_PINMMR         176
#define PINMUX_GIOA7_DMA_ON_SHIFT          24
#define PINMUX_GIOA7_DMA_ON_VALUE          0x00
#define PINMUX_GIOA7_DMA_ON_PIN            {176, 24, 0x00}

#define PINMUX_GIOA7_DMA_OFF_PINMMR        176
#define PINMUX_GIOA7_DMA_OFF_SHIFT         24
#define PINMUX_GIOA7_DMA_OFF_VALUE         0x01
#define PINMUX_GIOA7_DMA_OFF_PIN           {176, 24, 0x01}

/* GIOB0_DMA (PINMMR177[0]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB0_DMA_ON_PINMMR         177
#define PINMUX_GIOB0_DMA_ON_SHIFT          0
#define PINMUX_GIOB0_DMA_ON_VALUE          0x00
#define PINMUX_GIOB0_DMA_ON_PIN            {177, 0, 0x00}

#define PINMUX_GIOB0_DMA_OFF_PINMMR        177
#define PINMUX_GIOB0_DMA_OFF_SHIFT         0
#define PINMUX_GIOB0_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB0_DMA_OFF_PIN           {177, 0, 0x01}

/* GIOB1_DMA (PINMMR177[8]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB1_DMA_ON_PINMMR         177
#define PINMUX_GIOB1_DMA_ON_SHIFT          8
#define PINMUX_GIOB1_DMA_ON_VALUE          0x00
#define PINMUX_GIOB1_DMA_ON_PIN            {177, 8, 0x00}

#define PINMUX_GIOB1_DMA_OFF_PINMMR        177
#define PINMUX_GIOB1_DMA_OFF_SHIFT         8
#define PINMUX_GIOB1_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB1_DMA_OFF_PIN           {177, 8, 0x01}

/* GIOB2_DMA (PINMMR177[16]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB2_DMA_ON_PINMMR         177
#define PINMUX_GIOB2_DMA_ON_SHIFT          16
#define PINMUX_GIOB2_DMA_ON_VALUE          0x00
#define PINMUX_GIOB2_DMA_ON_PIN            {177, 16, 0x00}

#define PINMUX_GIOB2_DMA_OFF_PINMMR        177
#define PINMUX_GIOB2_DMA_OFF_SHIFT         16
#define PINMUX_GIOB2_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB2_DMA_OFF_PIN           {177, 16, 0x01}

/* GIOB3_DMA (PINMMR177[24]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB3_DMA_ON_PINMMR         177
#define PINMUX_GIOB3_DMA_ON_SHIFT          24
#define PINMUX_GIOB3_DMA_ON_VALUE          0x00
#define PINMUX_GIOB3_DMA_ON_PIN            {177, 24, 0x00}

#define PINMUX_GIOB3_DMA_OFF_PINMMR        177
#define PINMUX_GIOB3_DMA_OFF_SHIFT         24
#define PINMUX_GIOB3_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB3_DMA_OFF_PIN           {177, 24, 0x01}

/* GIOB4_DMA (PINMMR178[0]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB4_DMA_ON_PINMMR         178
#define PINMUX_GIOB4_DMA_ON_SHIFT          0
#define PINMUX_GIOB4_DMA_ON_VALUE          0x00
#define PINMUX_GIOB4_DMA_ON_PIN            {178, 0, 0x00}

#define PINMUX_GIOB4_DMA_OFF_PINMMR        178
#define PINMUX_GIOB4_DMA_OFF_SHIFT         0
#define PINMUX_GIOB4_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB4_DMA_OFF_PIN           {178, 0, 0x01}

/* GIOB5_DMA (PINMMR178[8]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB5_DMA_ON_PINMMR         178
#define PINMUX_GIOB5_DMA_ON_SHIFT          8
#define PINMUX_GIOB5_DMA_ON_VALUE          0x00
#define PINMUX_GIOB5_DMA_ON_PIN            {178, 8, 0x00}

#define PINMUX_GIOB5_DMA_OFF_PINMMR        178
#define PINMUX_GIOB5_DMA_OFF_SHIFT         8
#define PINMUX_GIOB5_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB5_DMA_OFF_PIN           {178, 8, 0x01}

/* GIOB6_DMA (PINMMR178[16]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB6_DMA_ON_PINMMR         178
#define PINMUX_GIOB6_DMA_ON_SHIFT          16
#define PINMUX_GIOB6_DMA_ON_VALUE          0x00
#define PINMUX_GIOB6_DMA_ON_PIN            {178, 16, 0x00}

#define PINMUX_GIOB6_DMA_OFF_PINMMR        178
#define PINMUX_GIOB6_DMA_OFF_SHIFT         16
#define PINMUX_GIOB6_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB6_DMA_OFF_PIN           {178, 16, 0x01}

/* GIOB7_DMA (PINMMR178[24]): ON (0x00), OFF (0x01) */

#define PINMUX_GIOB7_DMA_ON_PINMMR         178
#define PINMUX_GIOB7_DMA_ON_SHIFT          24
#define PINMUX_GIOB7_DMA_ON_VALUE          0x00
#define PINMUX_GIOB7_DMA_ON_PIN            {178, 24, 0x00}

#define PINMUX_GIOB7_DMA_OFF_PINMMR        178
#define PINMUX_GIOB7_DMA_OFF_SHIFT         24
#define PINMUX_GIOB7_DMA_OFF_VALUE         0x01
#define PINMUX_GIOB7_DMA_OFF_PIN           {178, 24, 0x01}

/* GIOB_DISABLE_HET2 (PINMMR179[0]): OFF (0x01), ON (0x02) */

#define PINMUX_GIOB_DISABLE_HET2_OFF_PINMMR  179
#define PINMUX_GIOB_DISABLE_HET2_OFF_SHIFT   0
#define PINMUX_GIOB_DISABLE_HET2_OFF_VALUE   0x01
#define PINMUX_GIOB_DISABLE_HET2_OFF_PIN     {179, 0, 0x01}

#define PINMUX_GIOB_DISABLE_HET2_ON_PINMMR  179
#define PINMUX_GIOB_DISABLE_HET2_ON_SHIFT   0
#define PINMUX_GIOB_DISABLE_HET2_ON_VALUE   0x02
#define PINMUX_GIOB_DISABLE_HET2_ON_PIN     {179, 0, 0x02}

/* GIOA_DISABLE_HET1 (PINMMR179[8]): OFF (0x01), ON (0x02) */

#define PINMUX_GIOA_DISABLE_HET1_OFF_PINMMR  179
#define PINMUX_GIOA_DISABLE_HET1_OFF_SHIFT   8
#define PINMUX_GIOA_DISABLE_HET1_OFF_VALUE   0x01
#define PINMUX_GIOA_DISABLE_HET1_OFF_PIN     {179, 8, 0x01}

#define PINMUX_GIOA_DISABLE_HET1_ON_PINMMR  179
#define PINMUX_GIOA_DISABLE_HET1_ON_SHIFT   8
#define PINMUX_GIOA_DISABLE_HET1_ON_VALUE   0x02
#define PINMUX_GIOA_DISABLE_HET1_ON_PIN     {179, 8, 0x02}

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57L843_PINMUX_H */