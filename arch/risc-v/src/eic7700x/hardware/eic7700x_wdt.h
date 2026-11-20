/****************************************************************************
 * arch/risc-v/src/eic7700x/hardware/eic7700x_wdt.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_WDT_H
#define __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "eic7700x_clk.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Four Synopsys DesignWare watchdogs (TRM section 3.11), and their timeout
 * really does reset the chip: the reset chapter lists the watchdog beside
 * power-on and the reset key as a source, the pad RST_OUT_N follows it,
 * and the cause register remembers it as bit 3.  Verified on silicon:
 * component type 0x44570120, version 1.12.
 *
 * Two facts shape the driver.  The enable bit is write-once: nothing
 * but a reset clears it, so stopping the dog means asserting the
 * block's own reset line in the CRG, which is exactly the state the boot
 * firmware leaves all four in.  And the counter runs at the 200 MHz
 * peripheral clock with only sixteen power-of-two periods to choose
 * from, so timeouts run from a third of a millisecond to eleven seconds
 * and nothing in between comes exactly.
 */

#define EIC7700X_WDT_BASE(n)     (0x50800000ul + (n) * 0x4000)
#define EIC7700X_WDT_COUNT       4
#define EIC7700X_IRQ_WDT(n)      (RISCV_IRQ_EXT + 87 + (n))

/* Register offsets *********************************************************/

#define EIC7700X_WDT_CR          0x00  /* Control                          */
#define EIC7700X_WDT_TORR        0x04  /* Timeout range                    */
#define EIC7700X_WDT_CCVR        0x08  /* Current counter value            */
#define EIC7700X_WDT_CRR         0x0c  /* Counter restart                  */
#define EIC7700X_WDT_STAT        0x10  /* Interrupt status                 */
#define EIC7700X_WDT_EOI         0x14  /* Interrupt clear, on read         */
#define EIC7700X_WDT_PROT_LEVEL  0x1c  /* Resets 2: DO NOT TOUCH, see .c   */
#define EIC7700X_WDT_COMP_TYPE   0xfc  /* Reads 0x44570120                 */

#define WDT_COMP_TYPE_VALUE      (0x44570120)

/* Control ******************************************************************/

#define WDT_CR_EN                (1 << 0)  /* Write once until reset       */
#define WDT_CR_RMOD              (1 << 1)  /* 1: interrupt then reset      */

/* Timeout range ************************************************************/

/* The period is 2^(16 + TOP) pclk cycles, TOP 0 to 15.  The upper nibble
 * of TORR is a second range for a feature this instance was built
 * without, so only the low one matters; both are written the same for
 * tidiness, as the manual's own example does.
 */

#define WDT_TORR_TOP(n)          ((((n) & 0xf) << 4) | ((n) & 0xf))
#define WDT_TORR_TOP_MAX         15
#define WDT_TORR_BASE_SHIFT      16

/* Counter restart **********************************************************/

#define WDT_CRR_RESTART          0x76  /* The magic feed value             */

/* The blocks' reset lines in the CRG, touched directly rather than
 * through the reset framework because stopping a watchdog must work
 * from a dying kernel: the panic notifier stops the dogs to preserve
 * the crash scene, and a mutex or an allocation is exactly what a
 * panicking system no longer has.  Bit n set releases instance n.
 */

#define EIC7700X_WDT_RST_CTRL    (EIC7700X_CLK_BASE + 0x0444)
#define WDT_RST_RELEASE(n)       (1u << (n))

#endif /* __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_WDT_H */
