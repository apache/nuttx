/****************************************************************************
 * arch/xtensa/src/esp32/rom/esp32_gpio.h
 *
 * Developed for NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derivies from sample code provided by Expressif Systems:
 *
 * Copyright 2010-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __XTENSA_SRC_ESP32_ROM_ESP32_GPIO_H
#define __XTENSA_SRC_ESP32_ROM_ESP32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip/esp32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MATRIX_DETACH_OUT_SIG     0x100  /* Detach an OUTPUT signal */
#define MATRIX_DETACH_IN_LOW_PIN  0x30   /* Detach non-inverted INPUT signal */
#define MATRIX_DETACH_IN_LOW_HIGH 0x38   /* Detach inverted INPUT signal */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum gpio_inttype_e
{
  GPIO_PIN_INTR_DISABLE = 0,
  GPIO_PIN_INTR_POSEDGE = 1,
  GPIO_PIN_INTR_NEGEDGE = 2,
  GPIO_PIN_INTR_ANYEGDE = 3,
  GPIO_PIN_INTR_LOLEVEL = 4,
  GPIO_PIN_INTR_HILEVEL = 5
};

typedef enum gpio_inttype_e GPIO_INT_TYPE;

/* GPIO interrupt handler, registered through gpio_intr_handler_register */

typedef void (*gpio_intr_handler_fn_t)(uint32_t intr_mask, bool high, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: gpio_init
 *
 * Description:
 *   Initialize GPIO. This includes reading the GPIO Configuration DataSet
 *   to initialize "output enables" and pin configurations for each gpio pin.
 *   Please do not call this function in SDK.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_init(void);

/****************************************************************************
 * Name: gpio_output_set
 *
 * Description:
 *   Change GPIO(0-31) pin output by setting, clearing, or disabling pins,
 *   GPIO0<->BIT(0).  There is no particular ordering guaranteed; so if the
 *   order of writes is significant, calling code should divide a single
 *   call into multiple calls.
 *
 * Input Parameters:
 *   set_mask     - the gpios that need high level.
 *   clear_mask   - the gpios that need low level.
 *   enable_mask  - the gpios that need be changed.
 *   disable_mask - the gpios that need diable output.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_output_set(uint32_t set_mask, uint32_t clear_mask,
                     uint32_t enable_mask, uint32_t disable_mask);

/****************************************************************************
 * Name: gpio_output_set_high
 *
 * Description:
 *   Change GPIO(32-39) pin output by setting, clearing, or disabling pins,
 *   GPIO32<->BIT(0).  There is no particular ordering guaranteed; so if the
 *   order of writes is significant, calling code should divide a single call
 *   into multiple calls.
 *
 * Input Parameters:
 *   set_mask     - the gpios that need high level.
 *   clear_mask   - the gpios that need low level.
 *   enable_mask  - the gpios that need be changed.
 *   disable_mask - the gpios that need diable output.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_output_set_high(uint32_t set_mask, uint32_t clear_mask,
                          uint32_t enable_mask, uint32_t disable_mask);

/****************************************************************************
 * Name: gpio_input_get
 *
 * Description:
 *   Sample the value of GPIO input pins(0-31) and returns a bitmask.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Bitmask for GPIO input pins, BIT(0) for GPIO0.
 *
 ****************************************************************************/

uint32_t gpio_input_get(void);

/****************************************************************************
 * Name: gpio_input_get_high
 *
 * Description:
 *   Sample the value of GPIO input pins(32-39) and returns a bitmask.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Bitmask for GPIO input pins, BIT(0) for GPIO32.
 *
 ****************************************************************************/

uint32_t gpio_input_get_high(void);

/****************************************************************************
 * Name: gpio_intr_handler_register
 *
 * Description:
 *   Register an application-specific interrupt handler for GPIO pin
 *   interrupts.  Once the interrupt handler is called, it will not be
 *   called again until after a call to gpio_intr_ack.
 *
 * Input Parameters:
 *   fn  - gpio application-specific interrupt handler
 *   arg - gpio application-specific interrupt handler argument.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_intr_handler_register(gpio_intr_handler_fn_t fn, void *arg);

/****************************************************************************
 * Name: gpio_intr_pending
 *
 * Description:
 *   Get gpio interrupts which happens but not processed.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Bitmask for GPIO pending interrupts, BIT(0) for GPIO0.
 *
 ****************************************************************************/

uint32_t gpio_intr_pending(void);

/****************************************************************************
 * Name: gpio_intr_pending_high
 *
 * Description:
 *   Get gpio interrupts which happens but not processed.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Bitmask for GPIO pending interrupts, BIT(0) for GPIO32.
 *
 ****************************************************************************/

uint32_t gpio_intr_pending_high(void);

/****************************************************************************
 * Name: gpio_intr_ack
 *
 * Description:
 *   Ack gpio interrupts to process pending interrupts.
 *
 * Input Parameters:
 *   ack_mask: bitmask for GPIO ack interrupts, BIT(0) for GPIO0.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_intr_ack(uint32_t ack_mask);

/****************************************************************************
 * Name: gpio_intr_ack_high
 *
 * Description:
 *   Ack gpio interrupts to process pending interrupts.
 *
 * Input Parameters:
 *   ack_mask: bitmask for GPIO ack interrupts, BIT(0) for GPIO32.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_intr_ack_high(uint32_t ack_mask);

/****************************************************************************
 * Name: gpio_pin_wakeup_enable
 *
 * Description:
 *   Set GPIO to wakeup the ESP32.
 *
 * Input Parameters:
 *   i          - gpio number.
 *   intr_state - only GPIO_PIN_INTR_LOLEVEL\GPIO_PIN_INTR_HILEVEL can be used
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pin_wakeup_enable(uint32_t i, GPIO_INT_TYPE intr_state);

/****************************************************************************
 * Name: gpio_pin_wakeup_disable
 *
 * Description:
 *   disable GPIOs to wakeup the ESP32.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pin_wakeup_disable(void);

/****************************************************************************
 * Name: gpio_matrix_in
 *
 * Description:
 *   Set gpio input to a signal, one gpio can input to several signals.
 *
 * Input Parameters:
 *   gpio - gpio number, 0~0x27
 *                        gpio == 0x30, input 0 to signal
 *                        gpio == 0x34, ???
 *                        gpio == 0x38, input 1 to signal
 *
 *   signal_idx - signal index.
 *   inv - the signal is inv or not
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv);

/****************************************************************************
 * Name: gpio_matrix_out
 *
 * Description:
 *   Set signal output to gpio, one signal can output to several gpios.
 *
 * Input Parameters:
 *   gpio - gpio number, 0~0x27
 *   signal_idx - signal index.
 *                signal_idx == 0x100, cancel output put to the gpio
 *   out_inv - the signal output is inv or not
 *   oen_inv - the signal output enable is inv or not
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_matrix_out(uint32_t gpio, uint32_t signal_idx, bool out_inv,
                     bool oen_inv);

/****************************************************************************
 * Name: gpio_pad_select_gpio
 *
 * Description:
 *   Select pad as a gpio function from IOMUX.
 *
 * Input Parameters:
 *   gpio_num - gpio number, 0~0x27
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pad_select_gpio(uint8_t gpio_num);

/****************************************************************************
 * Name: gpio_pad_set_drv
 *
 * Description:
 *   Set pad driver capability.
 *
 * Input Parameters:
 *   gpio_num - gpio number, 0~0x27
 *   drv      - 0-3
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pad_set_drv(uint8_t gpio_num, uint8_t drv);

/****************************************************************************
 * Name: gpio_pad_pullup
 *
 * Description:
 *   Pull up the pad from gpio number.
 *
 * Input Parameters:
 *   gpio_num - gpio number, 0~0x27
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pad_pullup(uint8_t gpio_num);

/****************************************************************************
 * Name: gpio_pad_pulldown
 *
 * Description:
 *   Pull down the pad from gpio number.
 *
 * Input Parameters:
 *   gpio_num - gpio number, 0~0x27
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pad_pulldown(uint8_t gpio_num);

/****************************************************************************
 * Name: gpio_pad_unhold
 *
 * Description:
 *   Unhold the pad from gpio number.
 *
 * Input Parameters:
 *   gpio_num - gpio number, 0~0x27
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pad_unhold(uint8_t gpio_num);

/****************************************************************************
 * Name: gpio_pad_hold
 *
 * Description:
 *   Hold the pad from gpio number.
 *
 * Input Parameters:
 *   gpio_num - gpio number, 0~0x27
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void gpio_pad_hold(uint8_t gpio_num);

#ifdef __cplusplus
}
#endif

#endif /* __XTENSA_SRC_ESP32_ROM_ESP32_GPIO_H */
