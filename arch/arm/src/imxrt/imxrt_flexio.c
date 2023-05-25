/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexio.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "barriers.h"

#include "imxrt_gpio.h"
#include "imxrt_periphclks.h"
#include "imxrt_flexio.h"
#include "hardware/imxrt_flexio.h"

#ifdef CONFIG_IMXRT_FLEXIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the FlexIO controller. */

struct imxrt_flexiodev_s
{
  struct flexio_dev_s     flexio;       /* Externally visible part of the FlexIO interface */
  uint32_t                base;         /* FlexIO controller register base address */
  bool                    initialized;  /* TRUE: Controller has been initialized */
};

struct flexio_config_s
{
  bool enable_flexio;                   /* Enable/disable FlexIO module */
  bool enable_indoze;                   /* Enable/disable FlexIO operation in doze mode */
  bool enable_indebug;                  /* Enable/disable FlexIO operation in debug mode */
  bool enable_fast_access;              /* Enable/disable fast access to FlexIO registers, fast access requires
                                         * the FlexIO clock to be at least twice the frequency of the bus clock. */
};

static void imxrt_flexio_reset(struct flexio_dev_s *dev);
static void imxrt_flexio_enable(struct flexio_dev_s *dev, bool enable);
static uint32_t imxrt_flexio_read_pin_input(struct flexio_dev_s *dev);
static uint8_t imxrt_flexio_get_shifter_state(struct flexio_dev_s *dev);
static void imxrt_flexio_set_shifter_config(struct flexio_dev_s *dev,
              uint8_t index,
              const struct flexio_shifter_config_s *shifter_config);
static void imxrt_flexio_set_timer_config(struct flexio_dev_s *dev,
              uint8_t index,
              const struct flexio_timer_config_s *timer_config);
static void imxrt_flexio_set_clock_mode(struct flexio_dev_s *dev,
              uint8_t index,
              enum flexio_timer_decrement_source_e clocksource);
static void imxrt_flexio_enable_shifter_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask);
static void imxrt_flexio_disable_shifter_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask);
static void imxrt_flexio_enable_shifter_error_interrupts(
              struct flexio_dev_s *dev, uint32_t mask);
static void imxrt_flexio_disable_shifter_error_interrupts(
              struct flexio_dev_s *dev, uint32_t mask);
static void imxrt_flexio_enable_timer_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask);
static void imxrt_flexio_disable_timer_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask);
static uint32_t imxrt_flexio_get_shifter_status_flags(
              struct flexio_dev_s *dev);
static void imxrt_flexio_clear_shifter_status_flags(
              struct flexio_dev_s *dev, uint32_t mask);
static uint32_t imxrt_flexio_get_shifter_error_flags(
                  struct flexio_dev_s *dev);
static void imxrt_flexio_clear_shifter_error_flags(
              struct flexio_dev_s *dev, uint32_t mask);
static uint32_t imxrt_flexio_get_timer_status_flags(
                  struct flexio_dev_s *dev);
static void imxrt_flexio_clear_timer_status_flags(
              struct flexio_dev_s *dev, uint32_t mask);
static void imxrt_flexio_enable_shifter_status_dma(
              struct flexio_dev_s *dev, uint32_t mask, bool enable);
static uint32_t imxrt_flexio_get_shifter_buffer_address(
                  struct flexio_dev_s *dev,
                  enum flexio_shifter_buffer_type_e type, uint8_t index);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct flexio_ops_s g_flexio_ops =
{
  .reset = imxrt_flexio_reset,
  .enable = imxrt_flexio_enable,
  .read_pin_input = imxrt_flexio_read_pin_input,
  .get_shifter_state = imxrt_flexio_get_shifter_state,
  .set_clock_mode = imxrt_flexio_set_clock_mode,
  .enable_shifter_status_interrupts =
  imxrt_flexio_enable_shifter_status_interrupts,
  .disable_shifter_status_interrupts =
  imxrt_flexio_disable_shifter_status_interrupts,
  .enable_shifter_error_interrupts =
  imxrt_flexio_enable_shifter_error_interrupts,
  .disable_shifter_error_interrupts =
  imxrt_flexio_disable_shifter_error_interrupts,
  .enable_timer_status_interrupts =
  imxrt_flexio_enable_timer_status_interrupts,
  .disable_timer_status_interrupts =
  imxrt_flexio_disable_timer_status_interrupts,
  .get_shifter_status_flags = imxrt_flexio_get_shifter_status_flags,
  .clear_shifter_status_flags = imxrt_flexio_clear_shifter_status_flags,
  .get_shifter_error_flags = imxrt_flexio_get_shifter_error_flags,
  .clear_shifter_error_flags = imxrt_flexio_clear_shifter_error_flags,
  .get_timer_status_flags = imxrt_flexio_get_timer_status_flags,
  .clear_timer_status_flags = imxrt_flexio_clear_timer_status_flags,
  .enable_shifter_status_dma = imxrt_flexio_enable_shifter_status_dma,
  .get_shifter_buffer_address = imxrt_flexio_get_shifter_buffer_address,
  .set_shifter_config = imxrt_flexio_set_shifter_config,
  .set_timer_config = imxrt_flexio_set_timer_config,
};

#ifdef CONFIG_IMXRT_FLEXIO1
static struct imxrt_flexiodev_s g_flexio1_dev =
{
  .flexio =
  {
    .ops = &g_flexio_ops,
  },
  .base = IMXRT_FLEXIO1_BASE,
};
#endif

#ifdef CONFIG_IMXRT_FLEXIO2
static struct imxrt_flexiodev_s g_flexio2_dev =
{
  .flexio =
  {
    .ops = &g_flexio_ops,
  },
  .base = IMXRT_FLEXIO2_BASE,
};
#endif

#ifdef CONFIG_IMXRT_FLEXIO3
static struct imxrt_flexiodev_s g_flexio3_dev =
{
  .flexio =
  {
    .ops = &g_flexio_ops,
  },
  .base = IMXRT_FLEXIO3_BASE,
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static inline uint32_t imxrt_flexio_getreg32(struct flexio_dev_s *dev,
                                             uint32_t offset);
static inline void imxrt_flexio_putreg32(struct flexio_dev_s *dev,
                                         uint32_t value, uint32_t offset);

static inline void imxrt_flexio_modifyreg32(struct flexio_dev_s *dev,
                                            unsigned int offset,
                                            uint32_t clearbits,
                                            uint32_t setbits);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_flexio_getreg32
 *
 * Description:
 *   Get the contents of the ENET register at offset
 *
 * Input Parameters:
 *   dev    - private FlexIO device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t imxrt_flexio_getreg32(struct flexio_dev_s *dev,
                                             uint32_t offset)
{
  return getreg32(((struct imxrt_flexiodev_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: imxrt_flexio_putreg32
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 * Input Parameters:
 *   dev       - private FlexIO device structure
 *   offset    - offset to the register of interest
 *   clearbits - the 32-bit value to be written as 0s
 *   setbits   - the 32-bit value to be written as 1s
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imxrt_flexio_modifyreg32(struct flexio_dev_s *dev,
                                            unsigned int offset,
                                            uint32_t clearbits,
                                            uint32_t setbits)
{
  modifyreg32(((struct imxrt_flexiodev_s *)dev)->base + offset, clearbits,
              setbits);
}

/****************************************************************************
 * Name: imxrt_flexio_putreg32
 *
 * Description:
 *   Write a value to the FlexIO register at offset
 *
 * Input Parameters:
 *   dev    - private FlexIO device structure
 *   value  - the 32-bit value to be written
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void imxrt_flexio_putreg32(struct flexio_dev_s *dev,
                                         uint32_t value, uint32_t offset)
{
  putreg32(value, ((struct imxrt_flexiodev_s *)dev)->base + offset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Resets the FlexIO module.
 *
 * param dev FlexIO device
 */

static void imxrt_flexio_reset(struct flexio_dev_s *dev)
{
  /* do software reset, software reset operation affect all other FLEXIO
   * registers except CTRL
   */

  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_CTRL_OFFSET, 0,
                           FLEXIO_CTRL_SWRST_MASK);
  imxrt_flexio_putreg32(dev, 0, IMXRT_FLEXIO_CTRL_OFFSET);
}

/* Enables the FlexIO module operation.
 *
 * param dev FlexIO device
 * param enable true to enable, false to disable.
 */

static void imxrt_flexio_enable(struct flexio_dev_s *dev, bool enable)
{
  if (enable)
    {
      imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_CTRL_OFFSET, 0,
                               FLEXIO_CTRL_FLEXEN_MASK);
    }
  else
    {
      imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_CTRL_OFFSET,
                               FLEXIO_CTRL_FLEXEN_MASK, 0);
    }
}

/* Reads the input data on each of the FlexIO pins.
 *
 * param dev FlexIO device
 * return FlexIO pin input data
 */

static uint32_t imxrt_flexio_read_pin_input(struct flexio_dev_s *dev)
{
  uint32_t regval;

  regval = imxrt_flexio_getreg32(dev, IMXRT_FLEXIO_PIN_OFFSET);

  return regval;
}

/* Gets the current state pointer for state mode use.
 *
 * param dev FlexIO device
 * return current State pointer
 */

static uint8_t imxrt_flexio_get_shifter_state(struct flexio_dev_s *dev)
{
  uint32_t regval;

  regval = imxrt_flexio_getreg32(dev, IMXRT_FLEXIO_SHIFTSTATE_OFFSET);

  return ((uint8_t)regval & FLEXIO_SHIFTSTATE_STATE_MASK);
}

/* Configures the shifter with the shifter configuration. The configuration
 * structure covers both the SHIFTCTL and SHIFTCFG registers. To configure
 * the shifter to the proper mode, select which timer controls the shifter
 * to shift, whether to generate start bit/stop bit, and the polarity of
 * start bit and stop bit.
 *
 * Example
 * code
 * struct flexio_shifter_config_s config = {
 * .timer_select = 0,
 * .timer_polarity = FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE,
 * .pin_config = FLEXIO_PIN_CONFIG_OPEN_DRAIN_OR_BIDIRECTION,
 * .pin_polarity = FLEXIO_PIN_ACTIVE_LOW,
 * .shifter_mode = FLEXIO_SHIFTER_MODE_TRANSMIT,
 * .input_source = FLEXIO_SHIFTER_INPUT_FROM_PIN,
 * .shifter_stop = FLEXIO_SHIFTER_STOP_BIT_HIGH,
 * .shifter_start = FLEXIO_SHIFTER_START_BIT_LOW
 * };
 * imxrt_flexio_set_shifter_config(dev, &config);
 * endcode
 *
 * param dev FlexIO device address
 * param index Shifter index
 * param shifter_config Pointer to flexio_shifter_config_s structure
 */

static void imxrt_flexio_set_shifter_config(struct flexio_dev_s *dev,
                                            uint8_t index,
              const struct flexio_shifter_config_s *shifter_config)
{
  uint32_t regval;

  regval =
    FLEXIO_SHIFTCFG_INSRC(shifter_config->input_source) |
    FLEXIO_SHIFTCFG_PWIDTH(shifter_config->parallel_width) |
    FLEXIO_SHIFTCFG_SSTOP(shifter_config->shifter_stop) |
    FLEXIO_SHIFTCFG_SSTART(shifter_config->shifter_start);

  imxrt_flexio_putreg32(dev, regval,
                        IMXRT_FLEXIO_SHIFTCFG0_OFFSET + index * 0x4);

  regval =
    FLEXIO_SHIFTCTL_TIMSEL(shifter_config->timer_select) |
    FLEXIO_SHIFTCTL_TIMPOL(shifter_config->timer_polarity) |
    FLEXIO_SHIFTCTL_PINCFG(shifter_config->pin_config) |
    FLEXIO_SHIFTCTL_PINSEL(shifter_config->pin_select) |
    FLEXIO_SHIFTCTL_PINPOL(shifter_config->pin_polarity) |
    FLEXIO_SHIFTCTL_SMOD(shifter_config->shifter_mode);

  imxrt_flexio_putreg32(dev, regval,
                        IMXRT_FLEXIO_SHIFTCTL0_OFFSET + index * 0x4);
}

/* Configures the timer with the timer configuration. The configuration
 * structure covers both the TIMCTL and TIMCFG registers. To configure the
 * timer to the proper mode, select trigger source for timer and the timer
 * pin output and the timing for timer.
 *
 * Example
 * code
 * struct flexio_timer_config_s config = {
 * .trigger_select = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0),
 * .trigger_polarity = FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW,
 * .trigger_source = FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL,
 * .pin_config = FLEXIO_PIN_CONFIG_OPEN_DRAIN_OR_BIDIRECTION,
 * .pin_select = 0,
 * .pin_polarity = FLEXIO_PIN_ACTIVE_HIGH,
 * .timer_mode = FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT,
 * .timer_output = FLEXIO_TIMER_OUTPUT_ZERO_NOT_AFFECTED_BY_RESET,
 * .timer_decrement =
 *    FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT,
 * .timer_reset = FLEXIO_TIMER_RESET_ON_TIMER_PIN_EQUAL_TO_TIMER_OUTPUT,
 * .timer_disable = FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE,
 * .timer_enable = FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH,
 * .timer_stop = FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE,
 * .timer_start = FLEXIO_TIMER_START_BIT_ENABLED
 * };
 * imxrt_flexio_set_timer_config(dev, &config);
 * endcode
 *
 * param dev FlexIO peripheral dev address
 * param index Timer index
 * param timer_config Pointer to the flexio_timer_config_s structure
 */

static void imxrt_flexio_set_timer_config(struct flexio_dev_s *dev,
                                          uint8_t index,
              const struct flexio_timer_config_s *timer_config)
{
  uint32_t regval;

  regval =
    FLEXIO_TIMCFG_TIMOUT(timer_config->timer_output) |
    FLEXIO_TIMCFG_TIMDEC(timer_config->timer_decrement) |
    FLEXIO_TIMCFG_TIMRST(timer_config->timer_reset) |
    FLEXIO_TIMCFG_TIMDIS(timer_config->timer_disable) |
    FLEXIO_TIMCFG_TIMENA(timer_config->timer_enable) |
    FLEXIO_TIMCFG_TSTOP(timer_config->timer_stop) |
    FLEXIO_TIMCFG_TSTART(timer_config->timer_start);

  imxrt_flexio_putreg32(dev, regval,
                        IMXRT_FLEXIO_TIMCFG0_OFFSET + index * 0x4);

  regval = FLEXIO_TIMCMP_CMP(timer_config->timer_compare);

  imxrt_flexio_putreg32(dev, regval,
                        IMXRT_FLEXIO_TIMCMP0_OFFSET + index * 0x4);

  regval =
    FLEXIO_TIMCTL_TRGSEL(timer_config->trigger_select) |
    FLEXIO_TIMCTL_TRGPOL(timer_config->trigger_polarity) |
    FLEXIO_TIMCTL_TRGSRC(timer_config->trigger_source) |
    FLEXIO_TIMCTL_PINCFG(timer_config->pin_config) |
    FLEXIO_TIMCTL_PINSEL(timer_config->pin_select) |
    FLEXIO_TIMCTL_PINPOL(timer_config->pin_polarity) |
    FLEXIO_TIMCTL_TIMOD(timer_config->timer_mode);

  imxrt_flexio_putreg32(dev, regval,
                        IMXRT_FLEXIO_TIMCTL0_OFFSET + index * 0x4);
}

/* This function set the value of the prescaler on flexio channels
 *
 * param base Pointer to the FlexIO simulated peripheral type.
 * param clocksource Set clock value
 */

static void imxrt_flexio_set_clock_mode(struct flexio_dev_s *dev,
                                        uint8_t index,
              enum flexio_timer_decrement_source_e clocksource)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_TIMCFG0_OFFSET + index * 0x4,
                           FLEXIO_TIMCFG_TIMDEC_MASK,
                           FLEXIO_TIMCFG_TIMDEC(clocksource));
}

/* Enables the shifter status interrupt. The interrupt generates when the
 * corresponding SSF is set.
 *
 * param dev FlexIO device
 * param mask The shifter status mask which can be calculated by
 * (1 << shifter index)
 * note For multiple shifter status interrupt enable, for example, two
 * shifter status enable, can calculate the mask by using
 * ((1 << shifter index0) | (1 << shifter index1))
 */

static void imxrt_flexio_enable_shifter_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_SHIFTSIEN_OFFSET, 0, mask);
}

/* Disables the shifter status interrupt. The interrupt won't generate when
 * the corresponding SSF is set.
 *
 * param dev FlexIO device
 * param mask The shifter status mask which can be calculated by
 *            (1 << shifter index)
 * note For multiple shifter status interrupt enable, for example, two
 *      shifter status enable, can calculate the mask by using
 *      ((1 << shifter index0) | (1 << shifter index1))
 */

static void imxrt_flexio_disable_shifter_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_SHIFTSIEN_OFFSET, mask, 0);
}

/* Enables the shifter error interrupt. The interrupt generates when the
 * corresponding SEF is set.
 *
 * param dev FlexIO device
 * param mask The shifter error mask which can be calculated by
 *            (1 << shifter index)
 * note For multiple shifter error interrupt enable, for example, two shifter
 *      error enable, can calculate the mask by using ((1 << shifter index0)
 *      | (1 << shifter index1))
 */

static void imxrt_flexio_enable_shifter_error_interrupts(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_SHIFTEIEN_OFFSET, 0, mask);
}

/* Disables the shifter error interrupt. The interrupt won't generate when
 * the corresponding SEF is set.
 *
 * param dev FlexIO device
 * param mask The shifter error mask which can be calculated by
 *            (1 << shifter index)
 * note For multiple shifter error interrupt enable, for example, two shifter
 *      error enable, can calculate the mask by using ((1 << shifter index0)
 *      | (1 << shifter index1))
 */

static void imxrt_flexio_disable_shifter_error_interrupts(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_SHIFTEIEN_OFFSET, mask, 0);
}

/* Enables the timer status interrupt. The interrupt generates when the
 * corresponding SSF is set.
 *
 * param dev FlexIO device
 * param mask The timer status mask which can be calculated by
 *            (1 << timer index)
 * note For multiple timer status interrupt enable, for example, two timer
 *      status enable, can calculate the mask by using ((1 << timer index0) |
 *      (1 << timer index1))
 */

static void imxrt_flexio_enable_timer_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_TIMIEN_OFFSET, 0, mask);
}

/* Disables the timer status interrupt. The interrupt won't generate when the
 * corresponding SSF is set.
 *
 * param dev FlexIO device
 * param mask The timer status mask which can be calculated by
 *            (1 << timer index)
 * note For multiple timer status interrupt enable, for example, two timer
 *      status enable, can calculate the mask by using ((1 << timer index0) |
 *      (1 << timer index1))
 */

static void imxrt_flexio_disable_timer_status_interrupts(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_TIMIEN_OFFSET, mask, 0);
}

/* Gets the shifter status flags.
 *
 * param dev FlexIO device
 * return Shifter status flags
 */

static uint32_t imxrt_flexio_get_shifter_status_flags(
                  struct flexio_dev_s *dev)
{
  uint32_t regval;

  regval = imxrt_flexio_getreg32(dev, IMXRT_FLEXIO_SHIFTSTAT_OFFSET);

  return (regval & FLEXIO_SHIFTSTAT_SSF_MASK);
}

/* Clears the shifter status flags.
 *
 * param dev FlexIO device
 * param mask The shifter status mask which can be calculated by
 *            (1 << shifter index)
 * note For clearing multiple shifter status flags, for example, two shifter
 *      status flags, can calculate the mask by using ((1 << shifter index0)
 *      | (1 << shifter index1))
 */

static void imxrt_flexio_clear_shifter_status_flags(
              struct flexio_dev_s *dev, uint32_t mask)
{
  imxrt_flexio_putreg32(dev, mask, IMXRT_FLEXIO_SHIFTSTAT_OFFSET);
}

/* Gets the shifter error flags.
 *
 * param dev FlexIO device
 * return Shifter error flags
 */

static uint32_t imxrt_flexio_get_shifter_error_flags(
                  struct flexio_dev_s *dev)
{
  uint32_t regval;

  regval = imxrt_flexio_getreg32(dev, IMXRT_FLEXIO_SHIFTERR_OFFSET);

  return (regval & FLEXIO_SHIFTERR_SEF_MASK);
}

/* Clears the shifter error flags.
 *
 * param dev FlexIO device
 * param mask The shifter error mask which can be calculated by
 *            (1 << shifter index)
 *
 * note For clearing multiple shifter error flags, for example, two shifter
 * error flags, can calculate the mask by using ((1 << shifter index0) |
 * (1 << shifter index1))
 */

static void imxrt_flexio_clear_shifter_error_flags(struct flexio_dev_s *dev,
                                                   uint32_t mask)
{
  imxrt_flexio_putreg32(dev, mask, IMXRT_FLEXIO_SHIFTERR_OFFSET);
}

/* Gets the timer status flags.
 *
 * param dev FlexIO device
 * return Timer status flags
 */

static uint32_t imxrt_flexio_get_timer_status_flags(struct flexio_dev_s *dev)
{
  uint32_t regval;

  regval = imxrt_flexio_getreg32(dev, IMXRT_FLEXIO_TIMSTAT_OFFSET);

  return (regval & FLEXIO_TIMSTAT_TSF_MASK);
}

/* Clears the timer status flags.
 *
 * param dev FlexIO device
 * param mask The timer status mask which can be calculated by
 *            (1 << timer index)
 *
 * note For clearing multiple timer status flags, for example, two timer
 * status flags, can calculate the mask by using ((1 << timer index0) |
 * (1 << timer index1))
 */

static void imxrt_flexio_clear_timer_status_flags(struct flexio_dev_s *dev,
                                                  uint32_t mask)
{
  imxrt_flexio_putreg32(dev, mask, IMXRT_FLEXIO_TIMSTAT_OFFSET);
}

/* Enables/disables the shifter status DMA. The DMA request generates when
 * the corresponding SSF is set.
 *
 * For multiple shifter status DMA enables, for example, calculate
 * the mask by using ((1 << shifter index0) | (1 << shifter index1))
 *
 * param dev FlexIO device
 * param mask The shifter status mask which can be calculated by
 *            (1 << shifter index)
 * param enable True to enable, false to disable.
 */

static void imxrt_flexio_enable_shifter_status_dma(struct flexio_dev_s *dev,
                                                  uint32_t mask, bool enable)
{
  if (enable)
    {
      imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_SHIFTSDEN_OFFSET, 0, mask);
    }
  else
    {
      imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_SHIFTSDEN_OFFSET, mask, 0);
    }
}

/* Configures the FlexIO with a FlexIO configuration. The configuration
 * structure can be filled by the user or be set with default values by
 * imxrt_flexio_get_default_config().
 *
 * Example
 * code
 * struct flexio_config_s config = {
 * .enable_flexio = true,
 * .enable_indoze = false,
 * .enable_indebug = true,
 * .enable_fast_access = false
 * };
 * imxrt_flexio_init(base, &config);
 * endcode
 *
 * param dev FlexIO device
 * param user_config pointer to flexio_config_s structure
 */

static void imxrt_flexio_init(struct flexio_dev_s *dev,
                              const struct flexio_config_s *user_config)
{
  imxrt_flexio_reset(dev);

  imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_CTRL_OFFSET,
                      (FLEXIO_CTRL_DOZEN_MASK |
                       FLEXIO_CTRL_DBGE_MASK |
                       FLEXIO_CTRL_FASTACC_MASK |
                       FLEXIO_CTRL_FLEXEN_MASK),
                      (FLEXIO_CTRL_DBGE(user_config->enable_indebug) |
                       FLEXIO_CTRL_FASTACC(user_config->enable_fast_access) |
                       FLEXIO_CTRL_FLEXEN(user_config->enable_flexio)));

  if (!user_config->enable_indoze)
    {
      imxrt_flexio_modifyreg32(dev, IMXRT_FLEXIO_CTRL_OFFSET, 0,
                               FLEXIO_CTRL_DOZEN_MASK);
    }
}

/* Gets the default configuration to configure the FlexIO module.
 *
 * Example:
 * code
 * struct flexio_config_s config;
 * imxrt_flexio_get_default_config(&config);
 * endcode
 *
 * param user_config pointer to flexio_config_s structure
 */

static void imxrt_flexio_get_default_config(
              struct flexio_config_s *user_config)
{
  DEBUGASSERT(user_config != NULL);

  memset(user_config, 0, sizeof(*user_config));

  user_config->enable_flexio      = true;
  user_config->enable_indoze      = false;
  user_config->enable_indebug     = true;
  user_config->enable_fast_access = false;
}

/* Gets the shifter buffer address for the DMA transfer usage.
 *
 * param dev FlexIO peripheral dev address
 * param type Shifter type of enum flexio_shifter_buffer_type_e
 * param index Shifter index
 * return Corresponding shifter buffer address
 */

static uint32_t imxrt_flexio_get_shifter_buffer_address(
                  struct flexio_dev_s *dev,
                  enum flexio_shifter_buffer_type_e type, uint8_t index)
{
  uint32_t address = 0;

  UNUSED(dev);

  DEBUGASSERT(index < FLEXIO_SHIFTBUF_COUNT);

  switch (type)
    {
    case FLEXIO_SHIFTER_BUFFER:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUF0_OFFSET + index * 0x4);
      break;

    case FLEXIO_SHIFTER_BUFFER_BIT_SWAPPED:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUFBIS0_OFFSET + index * 0x4);
      break;

    case FLEXIO_SHIFTER_BUFFER_BYTE_SWAPPED:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUFBYS0_OFFSET + index * 0x4);
      break;

    case FLEXIO_SHIFTER_BUFFER_BIT_BYTE_SWAPPED:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUFBBS0_OFFSET + index * 0x4);
      break;

    case FLEXIO_SHIFTER_BUFFER_NIBBLE_BYTE_SWAPPED:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUFNBS0_OFFSET + index * 0x4);
      break;

    case FLEXIO_SHIFTER_BUFFER_HALF_WORD_SWAPPED:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUFHWS0_OFFSET + index * 0x4);
      break;

    case FLEXIO_SHIFTER_BUFFER_NIBBLE_SWAPPED:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUFNIS0_OFFSET + index * 0x4);
      break;

    default:
      address = (uint32_t)(((struct imxrt_flexiodev_s *)dev)->base +
                           IMXRT_FLEXIO_SHIFTBUF0_OFFSET + index * 0x4);
      break;
    }

  return address;
}

/****************************************************************************
 * Name: imxrt_flexio_initialize
 *
 * Description:
 *   Initialize the selected FlexIO port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid FlexIO device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct flexio_dev_s *imxrt_flexio_initialize(int intf)
{
  struct imxrt_flexiodev_s *priv;
  struct flexio_config_s flexio_config;

  switch (intf)
    {
#ifdef CONFIG_IMXRT_FLEXIO1
    case 1:
      priv = &g_flexio1_dev;
      imxrt_clockall_flexio1();
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXIO2
    case 2:
      priv = &g_flexio2_dev;
      imxrt_clockall_flexio2();
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXIO3
    case 3:
      priv = &g_flexio3_dev;
      imxrt_clockall_flexio2();
      break;
#endif
    default:
      return NULL;
    }

  /* Has the FlexSPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* Now perform one time initialization */

      /* Perform hardware initialization. Puts the FlexIO into an active
       * state.
       */

      imxrt_flexio_get_default_config(&flexio_config);
      imxrt_flexio_init((struct flexio_dev_s *)priv, &flexio_config);

      /* Enable interrupts at the NVIC */

      priv->initialized = true;
    }

  return &priv->flexio;
}

#endif /* CONFIG_IMXRT_FLEXIO */
