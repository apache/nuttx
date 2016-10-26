/****************************************************************************
 * arch/xtensa/src/esp32/esp32_gpio.h
 *
 * Developed for NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derivies in part from sample code provided by Expressif Systems:
 *
 *   Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_GPIO_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to esp32_configgpio() **********************************/

/* Encoded pin attributes used with esp32_configgpio() */

#define INPUT             0x01
#define OUTPUT            0x02
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define SPECIAL           0xf0
#define FUNCTION_0        0x00
#define FUNCTION_1        0x20
#define FUNCTION_2        0x40
#define FUNCTION_3        0x70
#define FUNCTION_4        0x80

/* Interrupt type used with esp32_gpioirqenable() */

#define DISABLED          0x00
#define RISING            0x01
#define FALLING           0x02
#define CHANGE            0x03
#define ONLOW             0x04
#define ONHIGH            0x05
#define ONLOW_WE          0x0c
#define ONHIGH_WE         0x0d

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Must be big enough to hold the the above encodings */

typedef uint8_t gpio_pinattr_t;
typedef uint8_t gpio_intrtype_t;

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqinitialize(void);
#else
#  define esp32_gpioirqinitialize()
#endif

/****************************************************************************
 * Name: esp32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int esp32_configgpio(int pin, gpio_pinattr_t attr);

/****************************************************************************
 * Name: esp32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void esp32_gpiowrite(int pin, bool value);

/****************************************************************************
 * Name: esp32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool esp32_gpioread(int pin);

/****************************************************************************
 * Name: esp32_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqenable(int irq, gpio_intrtype_t intrtype);
#else
#  define esp32_gpioirqenable(irq,intrtype)
#endif

/****************************************************************************
 * Name: esp32_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_GPIO_IRQ
void esp32_gpioirqdisable(int irq);
#else
#  define esp32_gpioirqdisable(irq)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

int digitalRead(uint8_t pin);

void attachInterrupt(uint8_t pin, void (*)(void), int mode);
void detachInterrupt(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_GPIO_H */
