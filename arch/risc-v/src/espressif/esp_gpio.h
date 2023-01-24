/****************************************************************************
 * arch/risc-v/src/espressif/esp_gpio.h
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

#ifndef __ARCH_RISCV_SRC_ESPRESSIF_ESP_GPIO_H
#define __ARCH_RISCV_SRC_ESPRESSIF_ESP_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to esp_configgpio() ************************************/

/* Encoded pin attributes used with esp_configgpio()
 *
 * 11 10 9  8  7  6  5  4  3  2  1  0
 * -- -- -- -- -- -- -- -- -- -- -- --
 * DR DR DR FN FN FN OD PD PU F  O  I
 */

#define MODE_SHIFT          0
#define MODE_MASK           (7 << MODE_SHIFT)
#  define INPUT             (1 << 0)
#  define OUTPUT            (1 << 1)
#  define FUNCTION          (1 << 2)

#define PULL_SHIFT          3
#define PULL_MASK           (7 << PULL_SHIFT)
#  define PULLUP            (1 << 3)
#  define PULLDOWN          (1 << 4)
#  define OPEN_DRAIN        (1 << 5)

#define FUNCTION_SHIFT      6
#define FUNCTION_MASK       (7 << FUNCTION_SHIFT)
#  define FUNCTION_1        (1 << FUNCTION_SHIFT)
#  define FUNCTION_2        (2 << FUNCTION_SHIFT)
#  define FUNCTION_3        (3 << FUNCTION_SHIFT)
#  define FUNCTION_4        (4 << FUNCTION_SHIFT)
#  define FUNCTION_5        (5 << FUNCTION_SHIFT)
#  define FUNCTION_6        (6 << FUNCTION_SHIFT)

#define DRIVE_SHIFT         9
#define DRIVE_MASK          (7 << DRIVE_SHIFT)
#  define DRIVE_0           (1 << DRIVE_SHIFT)
#  define DRIVE_1           (2 << DRIVE_SHIFT)
#  define DRIVE_2           (3 << DRIVE_SHIFT)
#  define DRIVE_3           (4 << DRIVE_SHIFT)

#define INPUT_PULLUP        (INPUT | PULLUP)
#define INPUT_PULLDOWN      (INPUT | PULLDOWN)
#define OUTPUT_OPEN_DRAIN   (OUTPUT | OPEN_DRAIN)
#define INPUT_FUNCTION      (INPUT | FUNCTION)
#  define INPUT_FUNCTION_1  (INPUT_FUNCTION | FUNCTION_1)
#  define INPUT_FUNCTION_2  (INPUT_FUNCTION | FUNCTION_2)
#  define INPUT_FUNCTION_3  (INPUT_FUNCTION | FUNCTION_3)
#  define INPUT_FUNCTION_4  (INPUT_FUNCTION | FUNCTION_4)
#  define INPUT_FUNCTION_5  (INPUT_FUNCTION | FUNCTION_5)
#  define INPUT_FUNCTION_6  (INPUT_FUNCTION | FUNCTION_6)
#define OUTPUT_FUNCTION     (OUTPUT | FUNCTION)
#  define OUTPUT_FUNCTION_1 (OUTPUT_FUNCTION | FUNCTION_1)
#  define OUTPUT_FUNCTION_2 (OUTPUT_FUNCTION | FUNCTION_2)
#  define OUTPUT_FUNCTION_3 (OUTPUT_FUNCTION | FUNCTION_3)
#  define OUTPUT_FUNCTION_4 (OUTPUT_FUNCTION | FUNCTION_4)
#  define OUTPUT_FUNCTION_5 (OUTPUT_FUNCTION | FUNCTION_5)
#  define OUTPUT_FUNCTION_6 (OUTPUT_FUNCTION | FUNCTION_6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Must be big enough to hold the above encodings */

typedef uint16_t gpio_pinattr_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *   attr          - Attributes to be configured for the selected GPIO pin.
 *                   The following attributes are accepted:
 *                   - Direction (OUTPUT or INPUT)
 *                   - Pull (PULLUP, PULLDOWN or OPENDRAIN)
 *                   - Function (if not provided, assume function GPIO by
 *                     default)
 *                   - Drive strength (if not provided, assume DRIVE_2 by
 *                     default)
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp_configgpio(int pin, gpio_pinattr_t attr);

/****************************************************************************
 * Name: esp_gpio_matrix_in
 *
 * Description:
 *   Set GPIO input to a signal.
 *   NOTE: one GPIO can receive inputs from several signals.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *                   - If pin == 0x3c, cancel input to the signal, input 0
 *                     to signal.
 *                   - If pin == 0x3a, input nothing to signal.
 *                   - If pin == 0x38, cancel input to the signal, input 1
 *                     to signal.
 *   signal_idx    - Signal index.
 *   inv           - Flag indicating whether the signal is inverted.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpio_matrix_in(uint32_t pin, uint32_t signal_idx, bool inv);

/****************************************************************************
 * Name: esp_gpio_matrix_out
 *
 * Description:
 *   Set signal output to GPIO.
 *   NOTE: one signal can output to several GPIOs.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *   signal_idx    - Signal index.
 *                   - If signal_idx == 0x100, cancel output to the GPIO.
 *   out_inv       - Flag indicating whether the signal output is inverted.
 *   oen_inv       - Flag indicating whether the signal output enable is
 *                   inverted.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpio_matrix_out(uint32_t pin, uint32_t signal_idx, bool out_inv,
                         bool oen_inv);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESPRESSIF_ESP_GPIO_H */
