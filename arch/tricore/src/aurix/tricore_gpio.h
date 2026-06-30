/****************************************************************************
 * arch/tricore/src/aurix/tricore_gpio.h
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

/* Common GPIO pin encoding and API for Infineon AURIX TriCore.
 *
 * gpio_pinset_t bit layout:
 *
 *   31  30  29:27   26:24   23:16    15:12   11:10  9:8  7:4    3:0
 *   OD INIT PD[2:0] PL[2:0] PORT     ALTPULL MODE  ---  (rsvd) PIN
 *
 *   OD       - Open drain (1=open-drain, 0=push-pull)
 *   INIT     - Initial output value (1=high, 0=low)
 *   PD[2:0]  - Pad driver mode / strength (TC4x: 3 bits, TC3x: lower 2 bits)
 *   PL[2:0]  - Pad level selection / voltage (TC4x: 3, TC3x: lower 2 bits)
 *   PORT     - Port number (0 .. 40+)
 *   ALT/PULL - For output/periph: alternate function 0-15
 *              For input: 0=tristate, 1=pull-down, 2=pull-up
 *   MODE     - 0=input, 1=output, 2=peripheral
 *   PIN      - Pin within port (0-15)
 */

#ifndef __ARCH_TRICORE_SRC_AURIX_TRICORE_GPIO_H
#define __ARCH_TRICORE_SRC_AURIX_TRICORE_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/bits.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_PIN_SHIFT          0
#define GPIO_PIN_MASK           GENMASK(3, 0)
#define GPIO_PIN0               0
#define GPIO_PIN1               1
#define GPIO_PIN2               2
#define GPIO_PIN3               3
#define GPIO_PIN4               4
#define GPIO_PIN5               5
#define GPIO_PIN6               6
#define GPIO_PIN7               7
#define GPIO_PIN8               8
#define GPIO_PIN9               9
#define GPIO_PIN10              10
#define GPIO_PIN11              11
#define GPIO_PIN12              12
#define GPIO_PIN13              13
#define GPIO_PIN14              14
#define GPIO_PIN15              15

/* Mode: bits 11:10 */
#define GPIO_MODE_SHIFT         10
#define GPIO_MODE_MASK          GENMASK(11, 10)
#define GPIO_INPUT              (0u << GPIO_MODE_SHIFT)
#define GPIO_OUTPUT             (1u << GPIO_MODE_SHIFT)
#define GPIO_PERIPH             (2u << GPIO_MODE_SHIFT)

#define GPIO_FUNCALT_SHIFT      12
#define GPIO_FUNCALT_MASK       GENMASK(15, 12)

/* Output alternate function encodings */
#define GPIO_ALT0               (0u  << GPIO_FUNCALT_SHIFT)  /* GPIO */
#define GPIO_ALT1               (1u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT2               (2u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT3               (3u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT4               (4u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT5               (5u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT6               (6u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT7               (7u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT8               (8u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT9               (9u  << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT10              (10u << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT11              (11u << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT12              (12u << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT13              (13u << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT14              (14u << GPIO_FUNCALT_SHIFT)
#define GPIO_ALT15              (15u << GPIO_FUNCALT_SHIFT)

/* Input pull resistor encodings */
#define GPIO_PULL_NONE          (0u << GPIO_FUNCALT_SHIFT)
#define GPIO_PULL_DOWN          (1u << GPIO_FUNCALT_SHIFT)
#define GPIO_PULL_UP            (2u << GPIO_FUNCALT_SHIFT)

/* Port number: bits 23:16 */
#define GPIO_PORT_SHIFT         16
#define GPIO_PORT_MASK          GENMASK(22, 16)
#define GPIO_PORT(n)            ((uint32_t)(n) << GPIO_PORT_SHIFT)
#define GPIO_PORT0              0
#define GPIO_PORT1              1
#define GPIO_PORT2              2
#define GPIO_PORT3              3
#define GPIO_PORT4              4
#define GPIO_PORT5              5
#define GPIO_PORT6              6
#define GPIO_PORT7              7
#define GPIO_PORT8              8
#define GPIO_PORT9              9
#define GPIO_PORT10             10
#define GPIO_PORT11             11
#define GPIO_PORT12             12
#define GPIO_PORT13             13
#define GPIO_PORT14             14
#define GPIO_PORT15             15
#define GPIO_PORT16             16
#define GPIO_PORT17             17
#define GPIO_PORT18             18
#define GPIO_PORT19             19
#define GPIO_PORT20             20
#define GPIO_PORT21             21
#define GPIO_PORT22             22
#define GPIO_PORT23             23
#define GPIO_PORT24             24
#define GPIO_PORT25             25
#define GPIO_PORT26             26
#define GPIO_PORT27             27
#define GPIO_PORT28             28
#define GPIO_PORT29             29
#define GPIO_PORT30             30
#define GPIO_PORT31             31
#define GPIO_PORT32             32
#define GPIO_PORT33             33
#define GPIO_PORT34             34
#define GPIO_PORT35             35
#define GPIO_PORT36             36
#define GPIO_PORT37             37
#define GPIO_PORT38             38
#define GPIO_PORT39             39

#define GPIO_PERIPH_OWN_PAD     BIT(23)

#define GPIO_PADLEVEL_SHIFT     24
#define GPIO_PADLEVEL_MASK      GENMASK(26, 24)

#define GPIO_PL(n)              ((uint32_t)(n) << GPIO_PADLEVEL_SHIFT)

#define GPIO_PADDRV_SHIFT       27
#define GPIO_PADDRV_MASK        GENMASK(29, 27)

#define GPIO_PD(n)              ((uint32_t)(n) << GPIO_PADDRV_SHIFT)

#define GPIO_OUTPUT_HIGH        BIT(30)
#define GPIO_OUTPUT_LOW         0

#define GPIO_OPEN_DRAIN         BIT(31)

#define GPIO_GET_PIN(ps)        (((ps) & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT)
#define GPIO_GET_PORT(ps)       (((ps) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)
#define GPIO_GET_MODE(ps)       ((ps) & GPIO_MODE_MASK)
#define GPIO_GET_FUNCALT(ps)    (((ps) & GPIO_FUNCALT_MASK) >> GPIO_FUNCALT_SHIFT)
#define GPIO_GET_PADLEVEL(ps)   (((ps) & GPIO_PADLEVEL_MASK) >> GPIO_PADLEVEL_SHIFT)
#define GPIO_GET_PADDRV(ps)     (((ps) & GPIO_PADDRV_MASK) >> GPIO_PADDRV_SHIFT)
#define GPIO_IS_OUTPUT(ps)      (GPIO_GET_MODE(ps) != GPIO_INPUT)
#define GPIO_IS_OPENDRAIN(ps)   (!!((ps) & GPIO_OPEN_DRAIN))
#define GPIO_IS_INIT_HIGH(ps)   (!!((ps) & GPIO_OUTPUT_HIGH))
#define GPIO_IS_PERIPH_OWN_PAD(ps)  (!!((ps) & GPIO_PERIPH_OWN_PAD))

#define AURIX_GPIO(port_n, pin_n, mode, funcalt) \
  (GPIO_PORT(port_n) | (mode) | (funcalt) | ((pin_n) & GPIO_PIN_MASK))

#define AURIX_GPIO_FULL(port_n, pin_n, mode, funcalt, od, init, pd, pl, ph) \
  (GPIO_PORT(port_n) | (mode) | (funcalt) | ((pin_n) & GPIO_PIN_MASK) | \
   (od) | (init) | GPIO_PD(pd) | GPIO_PL(pl) | (ph))

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t gpio_pinset_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description.
 *
 * Input Parameters:
 *   pinset - Encoded pin configuration
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int aurix_config_gpio(gpio_pinset_t pinset);

#endif /* __ARCH_TRICORE_SRC_AURIX_TRICORE_GPIO_H */
