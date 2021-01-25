/****************************************************************************
 * arch/risc-v/src/bl602/bl602_gpio.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_GPIO_H
#define __ARCH_RISCV_SRC_BL602_BL602_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"
#include "hardware/bl602_glb.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Bit-encoded input to bl602_configgpio() */

/* Each port bit of the general-purpose I/O (GPIO) ports can be individually
 * configured by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output floating
 *  - Output with pull-up
 *  - Output with pull-down
 *  - Alternate function (Digital and Analog)
 *
 * 16-bit Encoding:       1111 1100 0000 0000
 *                        5432 1098 7654 3210
 *                        ---- ---- ---- ----
 *                        .MUU DDSF FFFP PPPP
 */

/* Mode:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .M.. .... .... ....
 */

#define GPIO_MODE_SHIFT  (14)                    /* Bits 14: Port Mode */
#define GPIO_MODE_MASK   (1 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT     (1 << GPIO_MODE_SHIFT)  /* Input Enable */
#  define GPIO_OUTPUT    (0 << GPIO_MODE_SHIFT)  /* Output Enable */

/* Input/output pull-ups/downs:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * ..UU .... .... ....
 */

#define GPIO_PUPD_SHIFT (12) /* Bits 16-17: Pull-up/down */
#define GPIO_PUPD_MASK  (3 << GPIO_PUPD_SHIFT)
#define GPIO_FLOAT      (0 << GPIO_PUPD_SHIFT) /* No pull-up, pull-down */
#define GPIO_PULLUP     (1 << GPIO_PUPD_SHIFT) /* Pull-up */
#define GPIO_PULLDOWN   (2 << GPIO_PUPD_SHIFT) /* Pull-down */

/* Drive:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... DD.. .... ....
 */

#define GPIO_DRV_SHIFT (10) /* Bits 10-11: Drive */
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
 * .... ..S. .... ....
 */

#define GPIO_SMT_SHIFT (9) /* Bits 9: SMT Enable */
#define GPIO_SMT_MASK  (3 << GPIO_SMT_SHIFT)
#define GPIO_SMT_DIS   (0 << GPIO_SMT_SHIFT)
#define GPIO_SMT_EN    (1 << GPIO_SMT_SHIFT)

/* GPIO type selection:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... ...F FFF. ....
 */

#define GPIO_FUNC_SHIFT  (5) /* Bits 5-8: GPIO Type */
#define GPIO_FUNC_MASK   (15 << GPIO_FUNC_SHIFT)
#define GPIO_FUNC_SDIO   (1 << GPIO_FUNC_SHIFT)  /* SDIO */
#define GPIO_FUNC_FLASH  (2 << GPIO_FUNC_SHIFT)  /* Flash */
#define GPIO_FUNC_SPI    (4 << GPIO_FUNC_SHIFT)  /* SPI */
#define GPIO_FUNC_I2C    (6 << GPIO_FUNC_SHIFT)  /* I2C */
#define GPIO_FUNC_UART   (7 << GPIO_FUNC_SHIFT)  /* UART */
#define GPIO_FUNC_PWM    (8 << GPIO_FUNC_SHIFT)  /* PWM */
#define GPIO_FUNC_EXT_PA (9 << GPIO_FUNC_SHIFT)  /* Analog */
#define GPIO_FUNC_ANA    (10 << GPIO_FUNC_SHIFT) /* Analog */
#define GPIO_FUNC_SWGPIO (11 << GPIO_FUNC_SHIFT) /* Software GPIO */
#define GPIO_FUNC_JTAG   (14 << GPIO_FUNC_SHIFT) /* JTAG */

/* This identifies the bit in the port:
 *
 * 1111 1100 0000 0000
 * 5432 1098 7654 3210
 * ---- ---- ---- ----
 * .... .... ...P PPPP
 */

#define GPIO_PIN_SHIFT (0) /* Bits 0-4: GPIO number: 0-28 */
#define GPIO_PIN_MASK  (0x1f << GPIO_PIN_SHIFT)
#define GPIO_PIN0      (0 << GPIO_PIN_SHIFT)
#define GPIO_PIN1      (1 << GPIO_PIN_SHIFT)
#define GPIO_PIN2      (2 << GPIO_PIN_SHIFT)
#define GPIO_PIN3      (3 << GPIO_PIN_SHIFT)
#define GPIO_PIN4      (4 << GPIO_PIN_SHIFT)
#define GPIO_PIN5      (5 << GPIO_PIN_SHIFT)
#define GPIO_PIN6      (6 << GPIO_PIN_SHIFT)
#define GPIO_PIN7      (7 << GPIO_PIN_SHIFT)
#define GPIO_PIN8      (8 << GPIO_PIN_SHIFT)
#define GPIO_PIN9      (9 << GPIO_PIN_SHIFT)
#define GPIO_PIN10     (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11     (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12     (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13     (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14     (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15     (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16     (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17     (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18     (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19     (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20     (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21     (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22     (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23     (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24     (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25     (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26     (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27     (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28     (28 << GPIO_PIN_SHIFT)

/* GLB GPIO interrupt trigger mode type definition */

#define GLB_GPIO_INT_TRIG_NEG_PULSE \
  0 /* GPIO negedge pulse trigger interrupt */

#define GLB_GPIO_INT_TRIG_POS_PULSE \
  1 /* GPIO posedge pulse trigger interrupt */

#define GLB_GPIO_INT_TRIG_NEG_LEVEL \
  2 /* GPIO negedge level trigger interrupt (32k 3T) */

#define GLB_GPIO_INT_TRIG_POS_LEVEL \
  3 /* GPIO posedge level trigger interrupt (32k 3T) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint16_t gpio_pinset_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: bl602_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int bl602_configgpio(gpio_pinset_t cfgset);

/****************************************************************************
 * Name: bl602_gpio_deinit
 *
 * Description:
 *   Deinit a GPIO (Set GPIO to floating input state)
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int bl602_gpio_deinit(uint8_t pin);

/****************************************************************************
 * Name: bl602_config_uart_sel
 *
 * Description:
 *   Configure the GPIO UART pin selection mux based on bit-encoded
 *   description of the pin and the selection signal
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int bl602_config_uart_sel(gpio_pinset_t cfgset, uint8_t sig_sel);

/****************************************************************************
 * Name: bl602_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void bl602_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: bl602_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool bl602_gpioread(gpio_pinset_t pinset);

/****************************************************************************
 * Name: bl602_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int bl602_gpiosetevent(gpio_pinset_t pinset,
                       bool          risingedge,
                       bool          fallingedge,
                       bool          event,
                       xcpt_t        func,
                       void *        arg);

/****************************************************************************
 * Name: bl602_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int bl602_gpio_initialize(void);

/****************************************************************************
 * Function:  bl602_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int bl602_dumpgpio(gpio_pinset_t pinset, const char *msg);
#else
#define bl602_dumpgpio(p, m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_GPIO_H */
