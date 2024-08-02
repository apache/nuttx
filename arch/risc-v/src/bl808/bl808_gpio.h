/****************************************************************************
 * arch/risc-v/src/bl808/bl808_gpio.h
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

#ifndef __ARCH_RISCV_SRC_BL808_BL808_GPIO_H
#define __ARCH_RISCV_SRC_BL808_BL808_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Encoded GPIO Attributes
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .MUU DDSF FFFF
 */

/* Mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .M.. .... ....
 */

#define GPIO_MODE_SHIFT  (10)                     /* Bit 10: Port Mode */
#define GPIO_MODE_MASK   (1 << GPIO_MODE_SHIFT)
#define GPIO_INPUT     (1 << GPIO_MODE_SHIFT)  /* Input Enable */
#define GPIO_OUTPUT    (0 << GPIO_MODE_SHIFT)  /* Output Enable */

/* Input/Output pull-ups/downs:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... ..UU .... ....
 */

#define GPIO_PUPD_SHIFT (8) /* Bits 8-9: Pull-up/down */
#define GPIO_PUPD_MASK  (3 << GPIO_PUPD_SHIFT)
#define GPIO_FLOAT      (0 << GPIO_PUPD_SHIFT) /* No pull-up, pull-down */
#define GPIO_PULLUP     (1 << GPIO_PUPD_SHIFT) /* Pull-up */
#define GPIO_PULLDOWN   (2 << GPIO_PUPD_SHIFT) /* Pull-down */

/* Drive:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... DD.. ....
 */

#define GPIO_DRV_SHIFT (6) /* Bits 6-7: Drive */
#define GPIO_DRV_MASK  (3 << GPIO_DRV_SHIFT)
#define GPIO_DRV_0     (0 << GPIO_DRV_SHIFT)
#define GPIO_DRV_1     (1 << GPIO_DRV_SHIFT)
#define GPIO_DRV_2     (2 << GPIO_DRV_SHIFT)
#define GPIO_DRV_3     (3 << GPIO_DRV_SHIFT)

/* Input Schmitt trigger:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ..S. ....
 */

#define GPIO_SMT_SHIFT (5) /* Bit 5: SMT Enable */
#define GPIO_SMT_MASK  (3 << GPIO_SMT_SHIFT)
#define GPIO_SMT_DIS   (0 << GPIO_SMT_SHIFT)
#define GPIO_SMT_EN    (1 << GPIO_SMT_SHIFT)

/* GPIO type selection:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...F FFFF
 */

#define GPIO_FUNC_SHIFT  (0) /* Bits 0-4: GPIO Type */
#define GPIO_FUNC_MASK   (0x1f << GPIO_FUNC_SHIFT)
#define GPIO_FUNC_SDH    (0  << GPIO_FUNC_SHIFT)  /* SDH */
#define GPIO_FUNC_SPI0   (1  << GPIO_FUNC_SHIFT)  /* SPI0 */
#define GPIO_FUNC_FLASH  (2  << GPIO_FUNC_SHIFT)  /* Flash */
#define GPIO_FUNC_I2C0   (5  << GPIO_FUNC_SHIFT)  /* I2C0 */
#define GPIO_FUNC_I2C1   (6  << GPIO_FUNC_SHIFT)  /* I2C1 */
#define GPIO_FUNC_UART   (7  << GPIO_FUNC_SHIFT)  /* UART */
#define GPIO_FUNC_CAM    (9  << GPIO_FUNC_SHIFT)  /* CSI */
#define GPIO_FUNC_ANA    (10 << GPIO_FUNC_SHIFT)  /* Analog */
#define GPIO_FUNC_SWGPIO (11 << GPIO_FUNC_SHIFT)  /* Software GPIO */
#define GPIO_FUNC_PWM0   (16 << GPIO_FUNC_SHIFT)  /* PWM0 */
#define GPIO_FUNC_SPI1   (18 << GPIO_FUNC_SHIFT)  /* SPI1 */
#define GPIO_FUNC_I2C2   (19  << GPIO_FUNC_SHIFT) /* I2C2 */
#define GPIO_FUNC_I2C3   (20  << GPIO_FUNC_SHIFT) /* I2C3 */
#define GPIO_FUNC_JTAG_D0 (27 << GPIO_FUNC_SHIFT) /* JTAG */

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
 * Name: bl808_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int bl808_configgpio(int pin, gpio_pinattr_t attr);

/****************************************************************************
 * Name: bl808_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void bl808_gpiowrite(int pin, bool value);

/****************************************************************************
 * Name: bl808_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool bl808_gpioread(int pin);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL808_BL808_GPIO_H */
