/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_grtc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GRTC_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GRTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_GRTC_TASKS_CAPTURE_OFFSET(n)     (0x0000 + 4 * (n))  /* Capture counter */
#define NRF54L_GRTC_TASKS_START_OFFSET          0x0060              /* Start counter */
#define NRF54L_GRTC_TASKS_STOP_OFFSET           0x0064              /* Stop counter */
#define NRF54L_GRTC_TASKS_CLEAR_OFFSET          0x0068              /* Clear counter */
#define NRF54L_GRTC_TASKS_PWMSTART_OFFSET       0x006c              /* Start PWM */
#define NRF54L_GRTC_TASKS_PWMSTOP_OFFSET        0x0070              /* Stop PWM */
#define NRF54L_GRTC_SUBSCRIBE_CAPTURE_OFFSET(n) (0x0080 + 4 * (n))  /* Capture subscription */
#define NRF54L_GRTC_EVENTS_COMPARE_OFFSET(n)    (0x0100 + 4 * (n))  /* Compare match */
#define NRF54L_GRTC_EVENTS_RTCOMPARESYNC_OFFSET 0x0164              /* LF timer synchronized */
#define NRF54L_GRTC_EVENTS_PWMPERIODEND_OFFSET  0x016c              /* PWM period ended */
#define NRF54L_GRTC_EVENTS_PWMREADY_OFFSET      0x0174              /* PWM ready */
#define NRF54L_GRTC_EVENTS_CLKOUTREADY_OFFSET   0x0178              /* Clock output ready */
#define NRF54L_GRTC_PUBLISH_COMPARE_OFFSET(n)   (0x0180 + 4 * (n))  /* Compare publication */
#define NRF54L_GRTC_PUBLISH_PWMREADY_OFFSET     0x01f4              /* PWM ready publication */
#define NRF54L_GRTC_PUBLISH_CLKOUTREADY_OFFSET  0x01f8              /* Clock ready publication */
#define NRF54L_GRTC_SHORTS_OFFSET               0x0200              /* Local shortcuts */
#define NRF54L_GRTC_INTEN_OFFSET(n)             (0x0300 + 16 * (n)) /* Interrupt enable */
#define NRF54L_GRTC_INTENSET_OFFSET(n)          (0x0304 + 16 * (n)) /* Enable interrupt */
#define NRF54L_GRTC_INTENCLR_OFFSET(n)          (0x0308 + 16 * (n)) /* Disable interrupt */
#define NRF54L_GRTC_INTPEND_OFFSET(n)           (0x030c + 16 * (n)) /* Pending interrupts */
#define NRF54L_GRTC_EVTEN_OFFSET                0x0400              /* Event routing */
#define NRF54L_GRTC_EVTENSET_OFFSET             0x0404              /* Enable routing */
#define NRF54L_GRTC_EVTENCLR_OFFSET             0x0408              /* Disable routing */
#define NRF54L_GRTC_MODE_OFFSET                 0x0510              /* Counter mode */
#define NRF54L_GRTC_CCL_OFFSET(n)               (0x0520 + 16 * (n)) /* Compare low word */
#define NRF54L_GRTC_CCH_OFFSET(n)               (0x0524 + 16 * (n)) /* Compare high word */
#define NRF54L_GRTC_CCADD_OFFSET(n)             (0x0528 + 16 * (n)) /* Add to compare */
#define NRF54L_GRTC_CCEN_OFFSET(n)              (0x052c + 16 * (n)) /* Compare enable */
#define NRF54L_GRTC_TIMEOUT_OFFSET              0x06a4              /* SYSCOUNTER sleep timeout */
#define NRF54L_GRTC_INTERVAL_OFFSET             0x06a8              /* CC0 increment */
#define NRF54L_GRTC_WAKETIME_OFFSET             0x06ac              /* Wakeup time */
#define NRF54L_GRTC_STATUS_LFTIMER_OFFSET       0x06b0              /* LF timer status */
#define NRF54L_GRTC_STATUS_PWM_OFFSET           0x06b4              /* PWM status */
#define NRF54L_GRTC_STATUS_CLKOUT_OFFSET        0x06b8              /* Clock output status */
#define NRF54L_GRTC_PWMCONFIG_OFFSET            0x0710              /* PWM configuration */
#define NRF54L_GRTC_CLKOUT_OFFSET               0x0714              /* Clock output configuration */
#define NRF54L_GRTC_CLKCFG_OFFSET               0x0718              /* Clock configuration */
#define NRF54L_GRTC_SYSCOUNTERL_OFFSET(n)       (0x0720 + 16 * (n)) /* Counter low word */
#define NRF54L_GRTC_SYSCOUNTERH_OFFSET(n)       (0x0724 + 16 * (n)) /* Counter high word */
#define NRF54L_GRTC_SYSCOUNTER_ACTIVE_OFFSET(n) (0x0728 + 16 * (n)) /* Keep counter active */

/* Register bit definitions *************************************************/

/* Tasks and events */

#define GRTC_TASKS_CAPTURE                 (1 << 0)
#define GRTC_TASKS_START                   (1 << 0)
#define GRTC_TASKS_STOP                    (1 << 0)
#define GRTC_TASKS_CLEAR                   (1 << 0)
#define GRTC_TASKS_PWMSTART                (1 << 0)
#define GRTC_TASKS_PWMSTOP                 (1 << 0)
#define GRTC_EVENTS_COMPARE                (1 << 0)
#define GRTC_EVENTS_RTCOMPARESYNC          (1 << 0)
#define GRTC_EVENTS_PWMPERIODEND           (1 << 0)
#define GRTC_EVENTS_PWMREADY               (1 << 0)
#define GRTC_EVENTS_CLKOUTREADY            (1 << 0)

/* SUBSCRIBE and PUBLISH registers */

#define GRTC_SUBSCRIBE_CHIDX_SHIFT         (0)
#define GRTC_SUBSCRIBE_CHIDX_MASK          (0xff << GRTC_SUBSCRIBE_CHIDX_SHIFT)
#define GRTC_SUBSCRIBE_EN                  (1 << 31)
#define GRTC_PUBLISH_CHIDX_SHIFT           (0)
#define GRTC_PUBLISH_CHIDX_MASK            (0xff << GRTC_PUBLISH_CHIDX_SHIFT)
#define GRTC_PUBLISH_EN                    (1 << 31)

/* SHORTS register */

#define GRTC_SHORTS_RTCOMPARE_CLEAR        (1 << 0)

/* INTEN, INTENSET, INTENCLR and INTPEND registers */

#define GRTC_INT_COMPARE(n)                (1 << (n))
#define GRTC_INT_RTCOMPARESYNC             (1 << 25)
#define GRTC_INT_PWMPERIODEND              (1 << 27)
#define GRTC_INT_PWMREADY                  (1 << 29)
#define GRTC_INT_CLKOUTREADY               (1 << 30)

/* EVTEN, EVTENSET and EVTENCLR registers */

#define GRTC_EVTEN_PWMPERIODEND            (1 << 27)

/* MODE register */

#define GRTC_MODE_AUTOEN                  (1 << 0)
#define GRTC_MODE_SYSCOUNTEREN            (1 << 1)

/* CC registers */

#define GRTC_CCL_MASK                     (0xffffffff)
#define GRTC_CCH_MASK                     (0x000fffff)
#define GRTC_CCADD_VALUE_MASK             (0x7fffffff)
#define GRTC_CCADD_REFERENCE_CC           (1 << 31)
#define GRTC_CCEN_ACTIVE                  (1 << 0)
#if defined(CONFIG_ARCH_CHIP_NRF54LM20A) || defined(CONFIG_ARCH_CHIP_NRF54LM20B)
#  define GRTC_CCEN_PASTCC                (1 << 1)
#endif

/* TIMEOUT, INTERVAL and WAKETIME registers */

#define GRTC_TIMEOUT_MASK                 (0xffff)
#define GRTC_INTERVAL_MASK                (0xffff)
#define GRTC_WAKETIME_MASK                (0xff)

/* STATUS registers */

#define GRTC_STATUS_LFTIMER_READY         (1 << 0)
#define GRTC_STATUS_PWM_READY             (1 << 0)
#define GRTC_STATUS_CLKOUT_READY          (1 << 0)

/* PWMCONFIG register */

#define GRTC_PWMCONFIG_PERIOD_MASK        (0xff)

/* CLKOUT register */

#define GRTC_CLKOUT_CLK32K                (1 << 0)
#define GRTC_CLKOUT_CLKFAST               (1 << 1)

/* CLKCFG register */

#define GRTC_CLKCFG_CLKFASTDIV_SHIFT       (0)
#define GRTC_CLKCFG_CLKFASTDIV_MASK        (0xff << GRTC_CLKCFG_CLKFASTDIV_SHIFT)
#define GRTC_CLKCFG_CLKFASTDIV(n)          ((n) << GRTC_CLKCFG_CLKFASTDIV_SHIFT)
#define GRTC_CLKCFG_CLKSEL_SHIFT           (16)
#define GRTC_CLKCFG_CLKSEL_MASK            (3 << GRTC_CLKCFG_CLKSEL_SHIFT)
#define GRTC_CLKCFG_CLKSEL_LFXO            (0 << GRTC_CLKCFG_CLKSEL_SHIFT)
#define GRTC_CLKCFG_CLKSEL_LFCLK           (1 << GRTC_CLKCFG_CLKSEL_SHIFT)
#define GRTC_CLKCFG_CLKSEL_LFLPRC          (2 << GRTC_CLKCFG_CLKSEL_SHIFT)

/* SYSCOUNTER registers */

#define GRTC_SYSCOUNTERL_MASK             (0xffffffff)
#define GRTC_SYSCOUNTERH_MASK             (0x000fffff)
#if defined(CONFIG_ARCH_CHIP_NRF54LM20A) || defined(CONFIG_ARCH_CHIP_NRF54LM20B)
#  define GRTC_SYSCOUNTERH_LOADED         (1 << 29)
#endif
#define GRTC_SYSCOUNTERH_BUSY             (1 << 30)
#define GRTC_SYSCOUNTERH_OVERFLOW         (1 << 31)
#define GRTC_SYSCOUNTER_ACTIVE            (1 << 0)
#define GRTC_COUNTER_MAX                  (0x000fffffffffffffull)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GRTC_H */
