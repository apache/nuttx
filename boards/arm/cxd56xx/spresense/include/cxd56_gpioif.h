/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_gpioif.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_GPIOIF_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_GPIOIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* gpioif GPIO Interface driver */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin Pull Setting
 * Pin floating, pull up, pull down definitions
 */

#define PIN_FLOAT               (0) /**< Floating */
#define PIN_PULLUP              (1) /**< Internal Weak Pull Up */
#define PIN_PULLDOWN            (2) /**< Internal Weak Pull Down */
#define PIN_BUSKEEPER           (3) /**< Internal Bus-Keeper */

/* GPIO Interrupt Setting
 * GPIO interrupt level and edge trigger types
 */

#define INT_HIGH_LEVEL          (2) /**< High Level */
#define INT_LOW_LEVEL           (3) /**< Low Level */
#define INT_RISING_EDGE         (4) /**< Rising Edge */
#define INT_FALLING_EDGE        (5) /**< Falling Edge */
#define INT_BOTH_EDGE           (7) /**< Both Edge */

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* GPIO Configure
 *
 * [in] pin: Pin number
 * [in] mode: Function mode 0=GPIO
 * [in] input: Input Enable true=enable, false=disable
 * [in] drive: Drive Strength true=High Drive(4mA), false=Normal(2mA)
 * [in] pull: 0=float, 1=pullup, 2=pulldown, 3=buskeeper
 *          - PIN_FLOAT
 *          - PIN_PULLUP
 *          - PIN_PULLDOWN
 *          - PIN_BUSKEEPER
 *
 *  return: OK(0) is success. negative value is failure.
 */

int board_gpio_config(uint32_t pin, int mode, bool input, bool drive,
                      int pull);

/* GPIO Status
 *
 * [in] pin: Pin number
 * [out] input: Input Enable true=enable, false=disable
 * [out] output: Output Enable true=enable, false=disable
 * [out] drive: Drive Strength true=HighDrive(4mA), false=Normal(2mA)
 * [out] pull: 0=float, 1=pullup, 2=pulldown, 3=buskeeper
 *          - PIN_FLOAT
 *          - PIN_PULLUP
 *          - PIN_PULLDOWN
 *          - PIN_BUSKEEPER
 *
 * return: Pin function mode. negative value is failure.
 */

int board_gpio_status(uint32_t pin, bool *input, bool *output, bool *drive,
                      int *pull);

/* GPIO Write
 *
 * [in] pin: Pin number
 * [in] value: Write Value 0<high, 0=low, 0>hiz
 */

void board_gpio_write(uint32_t pin, int value);

/* GPIO Read
 *
 * param: [in] pin: Pin number
 *
 * return:  read value 1=high, 0=low
 */

int board_gpio_read(uint32_t pin);

/* GPIO Interrupt Configure
 *
 * [in] pin: Pin number
 * [in] mode: Interrupt polarity
 *          - #INT_HIGH_LEVEL
 *          - #INT_LOW_LEVEL
 *          - #INT_RISING_EDGE
 *          - #INT_FALLING_EDGE
 *          - #INT_BOTH_EDGE
 * [in] filter: Noise Filter true=enable, false=disable
 * [in] isr: Interrupt Service Routine
 *
 *  IRQ number. negative value is failure.
 */

int board_gpio_intconfig(uint32_t pin, int mode, bool filter, xcpt_t isr);

/* GPIO Interrupt Configure
 *
 * [in] pin: Pin number
 * [out] mode: Interrupt polarity
 *          - #INT_HIGH_LEVEL
 *          - #INT_LOW_LEVEL
 *          - #INT_RISING_EDGE
 *          - #INT_FALLING_EDGE
 *          - #INT_BOTH_EDGE
 * [out] filter: Noise Filter true=enable, false=disable
 * [out] enabled: Interrupt true=enable, false=disable
 *
 *  IRQ number. negative value is failure.
 */

int board_gpio_intstatus(uint32_t pin, int *mode,
                         bool *filter, bool *enabled);

/* GPIO Interrupt Enable/Disable
 *
 * [in] pin: Pin number
 * [in] enable: Interrupt true=enable, false=disable
 *
 *  IRQ number. negative value is failure.
 */

int board_gpio_int(uint32_t pin, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_GPIOIF_H */
