/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmtw.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_CMTW_H
#define __ARCH_RENESAS_SRC_RX65N_CMTW_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer Start Register (CMWSTR) */

#define _0000_CMTW_CMWSTR_COUNTER_STOP   (0x0000U) /* Stop counter count */
#define _0001_CMTW_CMWSTR_COUNTER_START  (0x0001U) /* Start counter count */

/* Timer Control Register (CMWCR) */

/* Clock select (CKS[1:0]) */

#define _0000_CMTW_CMWCR_CLOCK_PCLK8              (0x0000U) /* PCLK/8 */
#define _0001_CMTW_CMWCR_CLOCK_PCLK32             (0x0001U) /* PCLK/32 */
#define _0002_CMTW_CMWCR_CLOCK_PCLK128            (0x0002U) /* PCLK/128 */
#define _0003_CMTW_CMWCR_CLOCK_PCLK512            (0x0003U) /* PCLK/512 */

/* Compare Match Interrupt Enable (CMWIE) */

/* Disable Compare Match Interrupt */

#define _0000_CMTW_CMWCR_CMWIE_DISABLE            (0x0000U)

/* Enable Compare Match Interrupt */

#define _0008_CMTW_CMWCR_CMWIE_ENABLE             (0x0008U)

/* Input Capture 0 Interrupt Enable (IC0IE) */

/* Disable Input Capture 0 Interrupt */

#define _0000_CMTW_CMWCR_IC0IE_DISABLE            (0x0000U)

/* Enable Input Capture 0 Interrupt */

#define _0010_CMTW_CMWCR_IC0IE_ENABLE             (0x0010U)

/* Input Capture 1 Interrupt Enable (IC1IE) */

/* Disable Input Capture 1 Interrupt */

#define _0000_CMTW_CMWCR_IC1IE_DISABLE            (0x0000U)

/* Enable Input Capture 1 Interrupt */

#define _0020_CMTW_CMWCR_IC1IE_ENABLE             (0x0020U)

/* Output Compare 0 Interrupt Enable (OC0IE) */

/* Disable Output Compare 0 Interrupt */

#define _0000_CMTW_CMWCR_OC0IE_DISABLE            (0x0000U)

/* Enable Output Compare 0 Interrupt */

#define _0040_CMTW_CMWCR_OC0IE_ENABLE             (0x0040U)

/* Output Compare 1 Interrupt Enable (OC1IE) */

/* Disable Output Compare 1 Interrupt */

#define _0000_CMTW_CMWCR_OC1IE_DISABLE            (0x0000U)

/* Enable Output Compare 1 Interrupt */

#define _0080_CMTW_CMWCR_OC1IE_ENABLE             (0x0080U)

/* Timer Counter Size (CMS) */

#define _0000_CMTW_CMWCR_COUNTER_SIZE_32          (0x0000U) /* 32 bits */
#define _0200_CMTW_CMWCR_COUNTER_SIZE_16          (0x0200U) /* 16 bits */

/* Counter Clear (CCLR[2:0]) */

/* CMWCNT counter cleared by CMWCOR */

#define _0000_CMTW_CMWCR_CCLR_ENABLE_CMWCOR       (0x6000U)

/* Clearing of CMWCNT counter disabled */

#define _2000_CMTW_CMWCR_CCLR_DISABLE             (0x2000U)

/* Clearing of CMWCNT counter disabled */

#define _4000_CMTW_CMWCR_CCLR_DISABLE             (0x4000U)

/* Clearing of CMWCNT counter disabled */

#define _6000_CMTW_CMWCR_CCLR_DISABLE             (0x6000U)

/* CMWCNT counter cleared by CMWICR0 */

#define _8000_CMTW_CMWCR_CCLR_CMWICR0_ENABLE      (0x8000U)

/* CMWCNT counter cleared by CMWICR1 */

#define _A000_CMTW_CMWCR_CCLR_CMWICR1_ENABLE      (0xa000U)

/* CMWCNT counter cleared by CMWOCR0 */

#define _C000_CMTW_CMWCR_CCLR_CMWOCR0_ENABLE      (0xc000U)

/* CMWCNT counter cleared by CMWOCR1 */

#define _E000_CMTW_CMWCR_CCLR_CMWOCR1_ENABLE      (0xe000U)

/* Timer I/O Control Register (CMWIOR) */

/* Input Compare Control 0 (IC0[1:0]) */

#define _0000_CMTW_CMWIOR_IC0_RISE   (0x0000U) /* Rising edge */
#define _0001_CMTW_CMWIOR_IC0_FALL   (0x0001U) /* Falling edge */
#define _0002_CMTW_CMWIOR_IC0_BOTH   (0x0002U) /* Both edges */

/* Input Capture Control 1 (IC1[1:0]) */

#define _0000_CMTW_CMWIOR_IC1_RISE   (0x0000U) /* Rising edge */
#define _0004_CMTW_CMWIOR_IC1_FALL   (0x0004U) /* Falling edge */
#define _0008_CMTW_CMWIOR_IC1_BOTH   (0x0008U) /* Both edges */

/* Input Capture Enable 0 (IC0E) */

/* Disable input capture of CMWICR0 */

#define _0000_CMTW_CMWIOR_IC0E_DISABLE   (0x0000U)

/* Enable input capture of CMWICR0 */

#define _0010_CMTW_CMWIOR_IC0E_ENABLE    (0x0010U)

/* Input Capture Enable 1 (IC1E) */

/* Disable input capture of CMWICR1 */

#define _0000_CMTW_CMWIOR_IC1E_DISABLE            (0x0000U)

/* Enable input capture of CMWICR1 */

#define _0020_CMTW_CMWIOR_IC1E_ENABLE             (0x0020U)

/* Output Compare Control 0 (OC0[1:0]) */

/* Retains the output value */

#define _0000_CMTW_CMWIOR_OC0_RETAIN              (0x0000U)

/* Initially outputs 0 */

#define _0100_CMTW_CMWIOR_OC0_OUTPUT0             (0x0100U)

/* Initially outputs 1 */

#define _0200_CMTW_CMWIOR_OC0_OUTPUT1             (0x0200U)

/* Output Compare Control 1 (OC1[1:0]) */

/* Retains the output value */

#define _0000_CMTW_CMWIOR_OC1_RETAIN              (0x0000U)

/* Initially outputs 0 */

#define _0400_CMTW_CMWIOR_OC1_OUTPUT0             (0x0400U)

/* Initially outputs 1 */

#define _0800_CMTW_CMWIOR_OC1_OUTPUT1             (0x0800U)

/* Compare Match Enable 0 (OC0E) */

/* Disable compare match using CMWOCR0 */

#define _0000_CMTW_CMWIOR_OC0E_DISABLE            (0x0000U)

/* Enable compare match using CMWOCR0 */

#define _1000_CMTW_CMWIOR_OC0E_ENABLE             (0x1000U)

/* Compare Match Enable 1 (OC1E) */

/* Disable compare match using CMWOCR1 */

#define _0000_CMTW_CMWIOR_OC1E_DISABLE            (0x0000U)

/* Enable compare match using CMWOCR1 */

#define _2000_CMTW_CMWIOR_OC1E_ENABLE             (0x2000U)

/* Compare Match Enable (CMWE) */

/* Disable compare match using CMWCOR */

#define _0000_CMTW_CMWIOR_CMWE_DISABLE            (0x0000U)

/* Enable compare match using CMWCOR */

#define _8000_CMTW_CMWIOR_CMWE_ENABLE             (0x8000U)

/* Interrupt Source Priority Register n (IPRn) */

/* Interrupt Priority Level Select (IPR[3:0]) */

#define _00_CMTW_PRIORITY_LEVEL0   (0x00U) /* Level 0 (interrupt disabled) */
#define _01_CMTW_PRIORITY_LEVEL1   (0x01U) /* Level 1 */
#define _02_CMTW_PRIORITY_LEVEL2   (0x02U) /* Level 2 */
#define _03_CMTW_PRIORITY_LEVEL3   (0x03U) /* Level 3 */
#define _04_CMTW_PRIORITY_LEVEL4   (0x04U) /* Level 4 */
#define _05_CMTW_PRIORITY_LEVEL5   (0x05U) /* Level 5 */
#define _06_CMTW_PRIORITY_LEVEL6   (0x06U) /* Level 6 */
#define _07_CMTW_PRIORITY_LEVEL7   (0x07U) /* Level 7 */
#define _08_CMTW_PRIORITY_LEVEL8   (0x08U) /* Level 8 */
#define _09_CMTW_PRIORITY_LEVEL9   (0x09U) /* Level 9 */
#define _0A_CMTW_PRIORITY_LEVEL10  (0x0aU) /* Level 10 */
#define _0B_CMTW_PRIORITY_LEVEL11  (0x0bU) /* Level 11 */
#define _0C_CMTW_PRIORITY_LEVEL12  (0x0cU) /* Level 12 */
#define _0D_CMTW_PRIORITY_LEVEL13  (0x0dU) /* Level 13 */
#define _0E_CMTW_PRIORITY_LEVEL14  (0x0eU) /* Level 14 */
#define _0F_CMTW_PRIORITY_LEVEL15  (0x0fU) /* Level 15 (highest) */

#endif /* __ARCH_RENESAS_SRC_RX65N_CMTW_H */
