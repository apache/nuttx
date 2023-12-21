/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXIO_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "imxrt_config.h"
#include "hardware/imxrt_flexio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Calculate FlexIO timer trigger. */

#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x)   ((uint32_t)(x) << 1u)
#define FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(x) (((uint32_t)(x) << 2u) | 0x1u)
#define FLEXIO_TIMER_TRIGGER_SEL_TIMn(x)       (((uint32_t)(x) << 2u) | 0x3u)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Define time of timer trigger polarity. */

enum flexio_timer_trigger_polarity_e
{
  FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_HIGH = 0x0u, /* Active high. */
  FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW  = 0x1u, /* Active low. */
};

/* Define type of timer trigger source. */

enum flexio_timer_trigger_source_e
{
  FLEXIO_TIMER_TRIGGER_SOURCE_EXTERNAL = 0x0u, /* External trigger selected. */
  FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL = 0x1u, /* Internal trigger selected. */
};

/* Define type of timer/shifter pin configuration. */

enum flexio_pin_config_e
{
  FLEXIO_PIN_CONFIG_OUTPUT_DISABLED           = 0x0u, /* Pin output disabled. */
  FLEXIO_PIN_CONFIG_OPEN_DRAIN_OR_BIDIRECTION = 0x1u, /* Pin open drain or bidirectional output enable. */
  FLEXIO_PIN_CONFIG_BIDIRECTION_OUTPUT_DATA   = 0x2u, /* Pin bidirectional output data. */
  FLEXIO_PIN_CONFIG_OUTPUT                    = 0x3u, /* Pin output. */
};

/* Definition of pin polarity. */

enum flexio_pin_polarity_e
{
  FLEXIO_PIN_ACTIVE_HIGH = 0x0u, /* Active high. */
  FLEXIO_PIN_ACTIVE_LOW  = 0x1u, /* Active low. */
};

/* Define type of timer work mode. */

enum flexio_timer_mode_e
{
  FLEXIO_TIMER_MODE_DISABLED            = 0x0u, /* Timer Disabled. */
  FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT  = 0x1u, /* Dual 8-bit counters baud/bit mode. */
  FLEXIO_TIMER_MODE_DUAL8_BIT_PWM       = 0x2u, /* Dual 8-bit counters PWM mode. */
  FLEXIO_TIMER_MODE_SINGLE16_BIT        = 0x3u, /* Single 16-bit counter mode. */
};

/* Define type of timer initial output or timer reset condition. */

enum flexio_timer_output_e
{
  FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET   = 0x0u, /* Logic one when enabled and is not affected by timer reset. */
  FLEXIO_TIMER_OUTPUT_ZERO_NOT_AFFECTED_BY_RESET  = 0x1u, /* Logic zero when enabled and is not affected by timer reset. */
  FLEXIO_TIMER_OUTPUT_ONE_AFFECTED_BY_RESET       = 0x2u, /* Logic one when enabled and on timer reset. */
  FLEXIO_TIMER_OUTPUT_ZERO_AFFECTED_BY_RESET      = 0x3u, /* Logic zero when enabled and on timer reset. */
};

/* Define type of timer decrement. */

enum flexio_timer_decrement_source_e
{
  FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT = 0x0u, /* Decrement counter on FlexIO clock, Shift clock equals Timer output. */
  FLEXIO_TIMER_DEC_SRC_ON_TRIGGER_INPUT_SHIFT_TIMER_OUTPUT,        /* Decrement counter on Trigger input (both edges), shift clock equals Timer output. */
  FLEXIO_TIMER_DEC_SRC_ON_PIN_INPUT_SHIFT_PIN_INPUT,               /* Decrement counter on Pin input (both edges), Shift clock equals Pin input. */
  FLEXIO_TIMER_DEC_SRC_ON_TRIGGER_INPUT_SHIFT_TRIGGER_INPUT        /* Decrement counter on Trigger input (both edges), Shift clock equals Trigger input. */
};

/* Define type of timer reset condition. */

enum flexio_timer_reset_condition_e
{
  FLEXIO_TIMER_RESET_NEVER                                  = 0x0u, /* Timer never reset. */
  FLEXIO_TIMER_RESET_ON_TIMER_PIN_EQUAL_TO_TIMER_OUTPUT     = 0x2u, /* Timer reset on Timer Pin equal to Timer Output. */
  FLEXIO_TIMER_RESET_ON_TIMER_TRIGGER_EQUAL_TO_TIMER_OUTPUT = 0x3u, /* Timer reset on Timer Trigger equal to Timer Output. */
  FLEXIO_TIMER_RESET_ON_TIMER_PIN_RISING_EDGE               = 0x4u, /* Timer reset on Timer Pin rising edge. */
  FLEXIO_TIMER_RESET_ON_TIMER_TRIGGER_RISING_EDGE           = 0x6u, /* Timer reset on Trigger rising edge. */
  FLEXIO_TIMER_RESET_ON_TIMER_TRIGGER_BOTH_EDGE             = 0x7u, /* Timer reset on Trigger rising or falling edge. */
};

/* Define type of timer disable condition. */

enum flexio_timer_disable_condition_e
{
  FLEXIO_TIMER_DISABLE_NEVER                          = 0x0u, /* Timer never disabled. */
  FLEXIO_TIMER_DISABLE_ON_PRE_TIMER_DISABLE           = 0x1u, /* Timer disabled on Timer N-1 disable. */
  FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE               = 0x2u, /* Timer disabled on Timer compare. */
  FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE_TRIGGER_LOW   = 0x3u, /* Timer disabled on Timer compare and Trigger Low. */
  FLEXIO_TIMER_DISABLE_ON_PIN_BOTH_EDGE               = 0x4u, /* Timer disabled on Pin rising or falling edge. */
  FLEXIO_TIMER_DISABLE_ON_PIN_BOTH_EDGE_TRIGGER_HIGH  = 0x5u, /* Timer disabled on Pin rising or falling edge provided Trigger is high. */
  FLEXIO_TIMER_DISABLE_ON_TRIGGER_FALLING_EDGE        = 0x6u, /* Timer disabled on Trigger falling edge. */
};

/* Define type of timer enable condition. */

enum flexio_timer_enable_condition_e
{
  FLEXIO_TIMER_ENABLED_ALWAYS                         = 0x0u, /* Timer always enabled. */
  FLEXIO_TIMER_ENABLE_ON_PREV_TIMER_ENABLE            = 0x1u, /* Timer enabled on Timer N-1 enable. */
  FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH                 = 0x2u, /* Timer enabled on Trigger high. */
  FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH_PIN_HIGH        = 0x3u, /* Timer enabled on Trigger high and Pin high. */
  FLEXIO_TIMER_ENABLE_ON_PIN_RISING_EDGE              = 0x4u, /* Timer enabled on Pin rising edge. */
  FLEXIO_TIMER_ENABLE_ON_PIN_RISING_EDGE_TRIGGER_HIGH = 0x5u, /* Timer enabled on Pin rising edge and Trigger high. */
  FLEXIO_TIMER_ENABLE_ON_TRIGGER_RISING_EDGE          = 0x6u, /* Timer enabled on Trigger rising edge. */
  FLEXIO_TIMER_ENABLE_ON_TRIGGER_BOTH_EDGE            = 0x7u, /* Timer enabled on Trigger rising or falling edge. */
};

/* Define type of timer stop bit generate condition. */

enum flexio_timer_stop_bit_condition_e
{
  FLEXIO_TIMER_STOP_BIT_DISABLED                        = 0x0u, /* Stop bit disabled. */
  FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_COMPARE         = 0x1u, /* Stop bit is enabled on timer compare. */
  FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE         = 0x2u, /* Stop bit is enabled on timer disable. */
  FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_COMPARE_DISABLE = 0x3u, /* Stop bit is enabled on timer compare and timer disable. */
};

/* Define type of timer start bit generate condition. */

enum flexio_timer_start_bit_condition_e
{
  FLEXIO_TIMER_START_BIT_DISABLED = 0x0u, /* Start bit disabled. */
  FLEXIO_TIMER_START_BIT_ENABLED  = 0x1u, /* Start bit enabled. */
};

/* FlexIO as PWM channel output state */

enum flexio_timer_output_state_e
{
  FLEXIO_PWM_LOW = 0u, /* The output state of PWM channel is low */
  FLEXIO_PWM_HIGH,     /* The output state of PWM channel is high */
};

/* Define type of timer polarity for shifter control. */

enum flexio_shifter_timer_polarity_e
{
  FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE = 0x0u, /* Shift on positive edge of shift clock. */
  FLEXIO_SHIFTER_TIMER_POLARITY_ON_NEGATIVE = 0x1u, /* Shift on negative edge of shift clock. */
};

/* Define type of shifter working mode. */

enum flexio_shifter_mode_e
{
  FLEXIO_SHIFTER_DISABLED               = 0x0u, /* Shifter is disabled. */
  FLEXIO_SHIFTER_MODE_RECEIVE           = 0x1u, /* Receive mode. */
  FLEXIO_SHIFTER_MODE_TRANSMIT          = 0x2u, /* Transmit mode. */
  FLEXIO_SHIFTER_MODE_MATCH_STORE       = 0x4u, /* Match store mode. */
  FLEXIO_SHIFTER_MODE_MATCH_CONTINUOUS  = 0x5u, /* Match continuous mode. */
  FLEXIO_SHIFTER_MODE_STATE             = 0x6u, /* SHIFTBUF contents are used for storing programmable state attributes. */
  FLEXIO_SHIFTER_MODE_LOGIC = 0x7u,             /* SHIFTBUF contents are used for implementing programmable logic look up table. */
};

/* Define type of shifter input source. */

enum flexio_shifter_input_source_e
{
  FLEXIO_SHIFTER_INPUT_FROM_PIN                 = 0x0u, /* Shifter input from pin. */
  FLEXIO_SHIFTER_INPUT_FROM_NEXT_SHIFTER_OUTPUT = 0x1u, /* Shifter input from Shifter N+1. */
};

/* Define of STOP bit configuration. */

enum flexio_shifter_stop_bit_e
{
  FLEXIO_SHIFTER_STOP_BIT_DISABLE = 0x0u, /* Disable shifter stop bit. */
  FLEXIO_SHIFTER_STOP_BIT_LOW     = 0x2u, /* Set shifter stop bit to logic low level. */
  FLEXIO_SHIFTER_STOP_BIT_HIGH    = 0x3u, /* Set shifter stop bit to logic high level. */
};

/* Define type of START bit configuration. */

enum flexio_shifter_start_bit_e
{
  FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_ENABLE = 0x0u, /* Disable shifter start bit, transmitter loads data on enable. */
  FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_SHIFT  = 0x1u, /* Disable shifter start bit, transmitter loads data on first shift. */
  FLEXIO_SHIFTER_START_BIT_LOW                          = 0x2u, /* Set shifter start bit to logic low level. */
  FLEXIO_SHIFTER_START_BIT_HIGH                         = 0x3u, /* Set shifter start bit to logic high level. */
};

/* Define FlexIO shifter buffer type */

enum flexio_shifter_buffer_type_e
{
  FLEXIO_SHIFTER_BUFFER                     = 0x0u, /* Shifter Buffer N Register. */
  FLEXIO_SHIFTER_BUFFER_BIT_SWAPPED         = 0x1u, /* Shifter Buffer N Bit Byte Swapped Register. */
  FLEXIO_SHIFTER_BUFFER_BYTE_SWAPPED        = 0x2u, /* Shifter Buffer N Byte Swapped Register. */
  FLEXIO_SHIFTER_BUFFER_BIT_BYTE_SWAPPED    = 0x3u, /* Shifter Buffer N Bit Swapped Register. */
  FLEXIO_SHIFTER_BUFFER_NIBBLE_BYTE_SWAPPED = 0x4u, /* Shifter Buffer N Nibble Byte Swapped Register. */
  FLEXIO_SHIFTER_BUFFER_HALF_WORD_SWAPPED   = 0x5u, /* Shifter Buffer N Half Word Swapped Register. */
  FLEXIO_SHIFTER_BUFFER_NIBBLE_SWAPPED      = 0x6u, /* Shifter Buffer N Nibble Swapped Register. */
};

#ifdef CONFIG_IMXRT_FLEXIO

struct flexio_timer_config_s
{
  uint32_t trigger_select;
  enum flexio_timer_trigger_polarity_e    trigger_polarity;
  enum flexio_timer_trigger_source_e      trigger_source;
  enum flexio_pin_config_e                pin_config;
  uint32_t                                pin_select;
  enum flexio_pin_polarity_e              pin_polarity;
  enum flexio_timer_mode_e                timer_mode;
  enum flexio_timer_output_e              timer_output;
  enum flexio_timer_decrement_source_e    timer_decrement;
  enum flexio_timer_reset_condition_e     timer_reset;
  enum flexio_timer_disable_condition_e   timer_disable;
  enum flexio_timer_enable_condition_e    timer_enable;
  enum flexio_timer_stop_bit_condition_e  timer_stop;
  enum flexio_timer_start_bit_condition_e timer_start;
  uint32_t timer_compare;
};

struct flexio_shifter_config_s
{
  uint32_t timer_select;
  enum flexio_shifter_timer_polarity_e    timer_polarity;
  enum flexio_pin_config_e                pin_config;
  uint32_t                                pin_select;
  enum flexio_pin_polarity_e              pin_polarity;
  enum flexio_shifter_mode_e              shifter_mode;
  uint32_t                                parallel_width;
  enum flexio_shifter_input_source_e      input_source;
  enum flexio_shifter_stop_bit_e          shifter_stop;
  enum flexio_shifter_start_bit_e         shifter_start;
};

/* The FlexIO vtable */

struct flexio_dev_s;

struct flexio_ops_s
{
  void (*reset)(struct flexio_dev_s *dev);
  void (*enable)(struct flexio_dev_s *dev, bool enable);
  uint32_t (*read_pin_input)(struct flexio_dev_s *dev);
  uint8_t (*get_shifter_state)(struct flexio_dev_s *dev);
  void (*set_shifter_config)(struct flexio_dev_s *dev, uint8_t index,
          const struct flexio_shifter_config_s *shifter_config);
  void (*set_timer_config)(struct flexio_dev_s *dev, uint8_t index,
          const struct flexio_timer_config_s *timer_config);
  void (*set_clock_mode)(struct flexio_dev_s *dev, uint8_t index,
          enum flexio_timer_decrement_source_e clocksource);
  void (*enable_shifter_status_interrupts)(struct flexio_dev_s *dev,
          uint32_t mask);
  void (*disable_shifter_status_interrupts)(struct flexio_dev_s *dev,
          uint32_t mask);
  void (*enable_shifter_error_interrupts)(struct flexio_dev_s *dev,
          uint32_t mask);
  void (*disable_shifter_error_interrupts)(struct flexio_dev_s *dev,
          uint32_t mask);
  void (*enable_timer_status_interrupts)(struct flexio_dev_s *dev,
          uint32_t mask);
  void (*disable_timer_status_interrupts)(struct flexio_dev_s *dev,
          uint32_t mask);
  uint32_t (*get_shifter_status_flags)(struct flexio_dev_s *dev);
  void (*clear_shifter_status_flags)(struct flexio_dev_s *dev,
          uint32_t mask);
  uint32_t (*get_shifter_error_flags)(struct flexio_dev_s *dev);
  void (*clear_shifter_error_flags)(struct flexio_dev_s *dev,
          uint32_t mask);
  uint32_t (*get_timer_status_flags)(struct flexio_dev_s *dev);
  void (*clear_timer_status_flags)(struct flexio_dev_s *dev,
          uint32_t mask);
  void (*enable_shifter_status_dma)(struct flexio_dev_s *dev,
          uint32_t mask, bool enable);
  uint32_t (*get_shifter_buffer_address)(struct flexio_dev_s *dev,
              enum flexio_shifter_buffer_type_e type, uint8_t index);
};

/* FlexIO private data. This structure only defines the initial fields of
 * the structure visible to the FlexIO client. The specific implementation
 * may add additional, device specific fields
 */

struct flexio_dev_s
{
  const struct flexio_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
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
 * Name: imxrt_flexio_initialize
 *
 * Description:
 *   Initialize the selected FlexIO port
 *
 * Input Parameters:
 *   intf - Interface number
 *
 * Returned Value:
 *   Valid FlexIO device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct flexio_dev_s *imxrt_flexio_initialize(int intf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMXRT_FLEXIO */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXIO_H */
