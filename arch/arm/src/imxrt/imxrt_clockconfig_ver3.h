/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig_ver3.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_VER3_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_VER3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/rt118x/imxrt118x_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPCG helpers so the shared code can enable/disable peripheral clock gates
 * with the same names as on other i.MX RT families.
 */

#define imxrt_clockall_lpuart1()    imxrt_ccm_gate_on(CCM_LPCG_LPUART1, true)
#define imxrt_clockall_lpuart2()    imxrt_ccm_gate_on(CCM_LPCG_LPUART2, true)
#define imxrt_clockall_lpuart3()    imxrt_ccm_gate_on(CCM_LPCG_LPUART3, true)
#define imxrt_clockall_lpuart4()    imxrt_ccm_gate_on(CCM_LPCG_LPUART4, true)
#define imxrt_clockall_lpuart5()    imxrt_ccm_gate_on(CCM_LPCG_LPUART5, true)
#define imxrt_clockall_lpuart6()    imxrt_ccm_gate_on(CCM_LPCG_LPUART6, true)
#define imxrt_clockall_lpuart7()    imxrt_ccm_gate_on(CCM_LPCG_LPUART7, true)
#define imxrt_clockall_lpuart8()    imxrt_ccm_gate_on(CCM_LPCG_LPUART8, true)
#define imxrt_clockall_lpuart9()    imxrt_ccm_gate_on(CCM_LPCG_LPUART9, true)
#define imxrt_clockall_lpuart10()   imxrt_ccm_gate_on(CCM_LPCG_LPUART10, true)
#define imxrt_clockall_lpuart11()   imxrt_ccm_gate_on(CCM_LPCG_LPUART11, true)
#define imxrt_clockall_lpuart12()   imxrt_ccm_gate_on(CCM_LPCG_LPUART12, true)

#define imxrt_clockall_xbar1()      imxrt_ccm_gate_on(CCM_LPCG_XBAR1, true)
#define imxrt_clockall_xbar2()      imxrt_ccm_gate_on(CCM_LPCG_XBAR2, true)
#define imxrt_clockall_xbar3()      imxrt_ccm_gate_on(CCM_LPCG_XBAR3, true)

#define imxrt_clockall_gpio1()      imxrt_ccm_gate_on(CCM_LPCG_GPIO1, true)
#define imxrt_clockall_gpio2()      imxrt_ccm_gate_on(CCM_LPCG_GPIO2, true)
#define imxrt_clockall_gpio3()      imxrt_ccm_gate_on(CCM_LPCG_GPIO3, true)
#define imxrt_clockall_gpio4()      imxrt_ccm_gate_on(CCM_LPCG_GPIO4, true)
#define imxrt_clockall_gpio5()      imxrt_ccm_gate_on(CCM_LPCG_GPIO5, true)
#define imxrt_clockall_gpio6()      imxrt_ccm_gate_on(CCM_LPCG_GPIO6, true)

#define imxrt_clockall_ocotp_ctrl() imxrt_ccm_gate_on(CCM_LPCG_OCOTP, true)

#define imxrt_clockall_usboh3()     imxrt_ccm_gate_on(CCM_LPCG_USB, true)
#define imxrt_clockoff_usboh3()     imxrt_ccm_gate_on(CCM_LPCG_USB, false)
#define imxrt_clockrun_usboh3()     imxrt_ccm_gate_on(CCM_LPCG_USB, true)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialise the RT1180 clock tree. */

void imxrt_clockconfig(void);

/* Program a clock root's MUX + DIV using a clock source name from
 * ccm_clock_name_e.  Returns OK, or -EINVAL if the (root, src) pair is
 * not valid.
 */

int imxrt_ccm_configure_root_clock(int root, int src, uint32_t div);

/* Enable or disable a low-power clock gate (LPCG). */

int imxrt_ccm_gate_on(int gate, bool enabled);

/* Compatibility shim for existing callers that use CCM_LPCG_DIR_ON /
 * CCM_LPCG_DIR_OFF as the value argument.
 */

void imxrt_periphclk_configure(unsigned int index, unsigned int value);

/* Return the frequency in Hz of the named clock source (OSC / PLL / PFD)
 * or zero if unknown.
 */

int imxrt_get_clock(int clkname, uint32_t *frequency);

/* Return the frequency in Hz of the selected clock root (after MUX and
 * DIV are applied).  Signature matches RT117x's imxrt_get_rootclock() so
 * that shared drivers do not need to distinguish between families.
 */

int imxrt_get_rootclock(uint32_t clkroot, uint32_t *frequency);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_VER3_H */
