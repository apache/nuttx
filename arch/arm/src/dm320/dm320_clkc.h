/****************************************************************************
 * arch/arm/src/dm320/dm320_clkc.h
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

#ifndef __ARCH_ARM_SRC_DM320_DM320_CLKC_H
#define __ARCH_ARM_SRC_DM320_DM320_CLKC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock Controller Register Map (CLKC) *************************************/

#define DM320_CLKC_PLLA      (DM320_CLKC_REGISTER_BASE+0x0000) /* PLLA Configuration */
#define DM320_CLKC_PLLB      (DM320_CLKC_REGISTER_BASE+0x0002) /* PLLB Configuration */
#define DM320_CLKC_SEL0      (DM320_CLKC_REGISTER_BASE+0x0004) /* Input Clock Source Selection #0 */
#define DM320_CLKC_SEL1      (DM320_CLKC_REGISTER_BASE+0x0006) /* Input Slock Source Selection #1 */
#define DM320_CLKC_SEL2      (DM320_CLKC_REGISTER_BASE+0x0008) /* Input Clock Source Selection #2 */
#define DM320_CLKC_DIV0      (DM320_CLKC_REGISTER_BASE+0x000a) /* Clock Divisor Settings #0 */
#define DM320_CLKC_DIV1      (DM320_CLKC_REGISTER_BASE+0x000c) /* Clock Divisor Settings #1 */
#define DM320_CLKC_DIV2      (DM320_CLKC_REGISTER_BASE+0x000e) /* Clock Divisor Settings #2 */
#define DM320_CLKC_DIV3      (DM320_CLKC_REGISTER_BASE+0x0010) /* Clock Divisor Settings #3 */
#define DM320_CLKC_DIV4      (DM320_CLKC_REGISTER_BASE+0x0012) /* Clock Divisor Settings #4 */
#define DM320_CLKC_BYP       (DM320_CLKC_REGISTER_BASE+0x0014) /* Bypass Control */
#define DM320_CLKC_INV       (DM320_CLKC_REGISTER_BASE+0x0016) /* Inverse Control */
#define DM320_CLKC_MOD0      (DM320_CLKC_REGISTER_BASE+0x0018) /* Module Clock Enables #0 */
#define DM320_CLKC_MOD1      (DM320_CLKC_REGISTER_BASE+0x001a) /* Module ClockEnables #1 */
#define DM320_CLKC_MOD2      (DM320_CLKC_REGISTER_BASE+0x001c) /* Module ClockEnables #1 */
#define DM320_CLKC_LPCTL0    (DM320_CLKC_REGISTER_BASE+0x001e) /* Low Power Control #0 */
#define DM320_CLKC_LPCTL1    (DM320_CLKC_REGISTER_BASE+0x0020) /* Low Power Control #1 */
#define DM320_CLKC_OSEL      (DM320_CLKC_REGISTER_BASE+0x0022) /* Output Clock Selector */
#define DM320_CLKC_O0DIV     (DM320_CLKC_REGISTER_BASE+0x0024) /* Output Clock #0 Divider */
#define DM320_CLKC_O1DIV     (DM320_CLKC_REGISTER_BASE+0x0026) /* Output Clock #1 Divider */
#define DM320_CLKC_O2DIV     (DM320_CLKC_REGISTER_BASE+0x0028) /* Output Clock #2 Divider */
#define DM320_CLKC_PWM0C     (DM320_CLKC_REGISTER_BASE+0x002a) /* PWM #0 Cycle Count */
#define DM320_CLKC_PWM0H     (DM320_CLKC_REGISTER_BASE+0x002c) /* PWM #0 High Period */
#define DM320_CLKC_PWM1C     (DM320_CLKC_REGISTER_BASE+0x002e) /* PWM #1 Cycle Count */
#define DM320_CLKC_PWM1H     (DM320_CLKC_REGISTER_BASE+0x0030) /* PWM #1 Cycle Period */
#define DM320_CLKC_TEST0     (DM320_CLKC_REGISTER_BASE+0x08FE) /* Test #0 */
#define DM320_CLKC_TEST1     (DM320_CLKC_REGISTER_BASE+0x08FE) /* Test #1 */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_DM320_DM320_CLKC_H */
