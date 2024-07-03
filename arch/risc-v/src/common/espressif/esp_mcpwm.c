/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_mcpwm.c
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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <inttypes.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/capture.h>

#include "esp_gpio.h"
#include "esp_irq.h"

#include "esp_attr.h"
#include "hal/mcpwm_hal.h"
#include "hal/mcpwm_ll.h"
#include "soc/mcpwm_periph.h"
#include "periph_ctrl.h"
#include "hal/clk_tree_hal.h"
#include "esp_clk_tree.h"

#ifdef CONFIG_ESP_MCPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_ESP32H2
#  define MCPWM_DEV_CLK_SOURCE MCPWM_TIMER_CLK_SRC_PLL96M
#else
#  define MCPWM_DEV_CLK_SOURCE SOC_MOD_CLK_PLL_F160M
#endif

#define MCPWM_DEV_CLK_PRESCALE      4
#define MCPWM_CAPTURE_DEFAULT_GROUP 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcpwm_dev_common_s
{
  mcpwm_hal_init_config_t group;
  mcpwm_hal_context_t hal;
  spinlock_t mcpwm_spinlock;
  bool initialized;          /* MCPWM peripheral and HAL has been initialized */
  bool capture_initialized;  /* Capture submodule has been initialized */
  int group_prescale;
};

#ifdef CONFIG_ESP_MCPWM_CAPTURE
/* Capture event data. The 'last_' value is used to calculate frequency */

struct mcpwm_capture_event_data_s
{
  uint32_t pos_edge_count;
  uint32_t neg_edge_count;
  uint32_t last_pos_edge_count;
};

enum mcpwm_capture_channel_e
{
  MCPWM_CAP_CHANNEL_0,  /* MCPWM capture channel number 0 */
  MCPWM_CAP_CHANNEL_1,  /* MCPWM capture channel number 1 */
  MCPWM_CAP_CHANNEL_2,  /* MCPWM capture channel number 2 */
  MCPWM_CAP_CHANNEL_MAX /* Number of MCPWM capture channels */
};

/* Lowe-half data structure for a capture channel */

struct mcpwm_cap_channel_lowerhalf_s
{
  /* The following block is part of the upper-half device struct */

  const struct cap_ops_s *ops;

  /* The following is private to the ESP MCPWM driver */

  struct mcpwm_dev_common_s *common;
  struct mcpwm_capture_event_data_s *data;
  int channel_id;
  int gpio_pin;
  uint32_t clock;
  uint32_t freq;
  uint8_t duty;
  uint8_t isr_count;
  bool ready;
  bool enabled;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_mcpwm_group_start(void);

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_mcpwm_capture_set_gpio(
  struct mcpwm_cap_channel_lowerhalf_s *lower);
static int esp_mcpwm_capture_isr_register(int (*fn)(int, void *, void *),
                                          void *arg);
static int IRAM_ATTR mcpwm_capture_driver_isr_default(int irq, void *context,
                                                      void *arg);

/* Lower half methods required by capture driver */

static int esp_capture_start(struct cap_lowerhalf_s *lower);
static int esp_capture_stop(struct cap_lowerhalf_s *lower);
static int esp_capture_getduty(struct cap_lowerhalf_s *lower,
                               uint8_t *duty);
static int esp_capture_getfreq(struct cap_lowerhalf_s *lower,
                               uint32_t *freq);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mcpwm_dev_common_s mcpwm_common =
{
  .group.group_id      = MCPWM_CAPTURE_DEFAULT_GROUP,
  .initialized         = false,
  .capture_initialized = false,
  .group_prescale      = MCPWM_DEV_CLK_PRESCALE,
};

#ifdef CONFIG_ESP_MCPWM_CAPTURE
/* Lower half methods required by the capture driver */

static const struct cap_ops_s mcpwm_cap_ops =
{
  .start   = esp_capture_start,
  .stop    = esp_capture_stop,
  .getduty = esp_capture_getduty,
  .getfreq = esp_capture_getfreq,
};

/* Data structures for the available capture channels */

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0
static struct mcpwm_capture_event_data_s event_data_ch0;
static struct mcpwm_cap_channel_lowerhalf_s mcpwm_cap_ch0_lowerhalf =
{
  .ops        = &mcpwm_cap_ops,
  .common     = &mcpwm_common,
  .data       = &event_data_ch0,
  .channel_id = MCPWM_CAP_CHANNEL_0,
  .ready      = false,
};
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH1
static struct mcpwm_capture_event_data_s event_data_ch1;
static struct mcpwm_cap_channel_lowerhalf_s mcpwm_cap_ch1_lowerhalf =
{
  .ops        = &mcpwm_cap_ops,
  .common     = &mcpwm_common,
  .data       = &event_data_ch1,
  .channel_id = MCPWM_CAP_CHANNEL_1,
  .ready      = false,
};
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH2
static struct mcpwm_capture_event_data_s event_data_ch2;
static struct mcpwm_cap_channel_lowerhalf_s mcpwm_cap_ch2_lowerhalf =
{
  .ops        = &mcpwm_cap_ops,
  .common     = &mcpwm_common,
  .data       = &event_data_ch2,
  .channel_id = MCPWM_CAP_CHANNEL_2,
  .ready      = false,
};
#endif
#endif  /* CONFIG_ESP_MCPWM_CAPTURE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_capture_start
 *
 * Description:
 *   This function is a requirement of the upper-half driver. When called,
 *   enables the capture channel, interruption routine, sets the positive
 *   edge to trigger this interrupt and resets the frequency and duty
 *   values. The positive edge is always the first expected.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_capture_start(struct cap_lowerhalf_s *lower)
{
  struct mcpwm_cap_channel_lowerhalf_s *priv = (
    struct mcpwm_cap_channel_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  mcpwm_hal_context_t *hal = &priv->common->hal;

  /* Enable channel and interruption for rising edge */

  mcpwm_ll_capture_enable_channel(hal->dev, priv->channel_id, true);
  mcpwm_ll_intr_enable(hal->dev,
                       MCPWM_LL_EVENT_CAPTURE(priv->channel_id),
                       true);
  mcpwm_ll_intr_clear_status(hal->dev,
                             MCPWM_LL_EVENT_CAPTURE(priv->channel_id));
  mcpwm_ll_capture_enable_posedge(hal->dev, priv->channel_id, true);
  mcpwm_ll_capture_enable_negedge(hal->dev, priv->channel_id, false);

  /* Reset values of interest */

  priv->freq = 0;
  priv->duty = 0;
  priv->isr_count = 0;
  priv->enabled = true;
  priv->ready = false;

  cpinfo("Channel enabled: %d\n", priv->channel_id);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_capture_stop
 *
 * Description:
 *   This function is a requirement of the upper-half driver. When called,
 *   disables the capture channel and the interrupt routine associated.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_capture_stop(struct cap_lowerhalf_s *lower)
{
  struct mcpwm_cap_channel_lowerhalf_s *priv = (
    struct mcpwm_cap_channel_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  mcpwm_hal_context_t *hal = &priv->common->hal;

  /* Disable channel and interrupts */

  mcpwm_ll_capture_enable_channel(hal->dev, priv->channel_id, false);
  mcpwm_ll_intr_enable(hal->dev,
                       MCPWM_LL_EVENT_CAPTURE(priv->channel_id),
                       false);
  priv->enabled = false;

  cpinfo("Channel disabled: %d\n", priv->channel_id);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_capture_getduty
 *
 * Description:
 *   This function is a requirement of the upper-half driver. Returns
 *   the last calculated duty cycle value.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   duty  - uint8_t pointer where the duty cycle value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_capture_getduty(struct cap_lowerhalf_s *lower,
                               uint8_t *duty)
{
  struct mcpwm_cap_channel_lowerhalf_s *priv = (
    struct mcpwm_cap_channel_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  *duty = priv->duty;
  cpinfo("Get duty called from channel %d\n", priv->channel_id);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_capture_getfreq
 *
 * Description:
 *   This function is a requirement of the upper-half driver. Returns
 *   the last calculated frequency value.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   duty  - uint8_t pointer where the frequency value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_capture_getfreq(struct cap_lowerhalf_s *lower,
                               uint32_t *freq)
{
  struct mcpwm_cap_channel_lowerhalf_s *priv = (
    struct mcpwm_cap_channel_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  *freq = priv->freq;
  cpinfo("Get freq called from channel %d\n", priv->channel_id);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_mcpwm_group_start
 *
 * Description:
 *   Initializes the MCPWM0 module, MCPWM HAL.
 *   Should be called only once.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_mcpwm_group_start(void)
{
  mcpwm_hal_context_t *hal = &mcpwm_common.hal;

  /* HAL and MCPWM Initialization */

  periph_module_enable(PERIPH_MCPWM0_MODULE);
  mcpwm_hal_init(hal, &mcpwm_common.group);
  mcpwm_hal_timer_reset(hal, 0);
  mcpwm_ll_group_set_clock_source(hal->dev, MCPWM_DEV_CLK_SOURCE);
  mcpwm_ll_group_set_clock_prescale(hal->dev, 4);
  mcpwm_ll_group_enable_clock(hal->dev, true);

  mcpwm_common.initialized = true;
}

/****************************************************************************
 * Name: esp_mcpwm_capture_set_gpio
 *
 * Description:
 *   Configures the lower-half GPIO pin to be used as a capture input.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_mcpwm_capture_set_gpio(
  struct mcpwm_cap_channel_lowerhalf_s *lower)
{
  mcpwm_hal_context_t *hal = &lower->common->hal;
  int ret;

  ret = esp_configgpio(lower->gpio_pin, INPUT | PULLUP);
  if (ret < 0)
    {
      cperr("Failed configuring GPIO pin\n");
      return ret;
    }

  esp_gpio_matrix_in(lower->gpio_pin,
    mcpwm_periph_signals.groups[0].captures[lower->channel_id].cap_sig,
    false);

  cpinfo("GPIO: %d configured for channel %d\n", lower->gpio_pin,
         lower->channel_id);
  return ret;
}
#endif

/****************************************************************************
 * Name: esp_mcpwm_capture_isr_register
 *
 * Description:
 *   Registers a callback function for a channel interrupt request.
 *
 * Input Parameters:
 *   fn  - Pointer to ISR function that is to be called.
 *   arg - Pointer to arguments passed to said function.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_mcpwm_capture_isr_register(int (*fn)(int, void *, void *),
                                          void *arg)
{
  int cpuint;
  int ret;

  cpuint = esp_setup_irq(mcpwm_periph_signals.groups[0].irq_id,
                         ESP_IRQ_PRIORITY_DEFAULT,
                         ESP_IRQ_TRIGGER_LEVEL);
  if (cpuint < 0)
    {
      cperr("Failed to allocate a CPU interrupt.\n");
      return -ENOMEM;
    }

  ret = irq_attach(ESP_IRQ_MCPWM0,
                   mcpwm_capture_driver_isr_default,
                   &mcpwm_common);
  if (ret < 0)
    {
      cperr("Couldn't attach IRQ to handler.\n");
      esp_teardown_irq(mcpwm_periph_signals.groups[0].irq_id, cpuint);
      return ret;
    }

  up_enable_irq(ESP_IRQ_MCPWM0);

  return ret;
}
#endif

/****************************************************************************
 * Name: mcpwm_capture_driver_isr_default
 *
 * Description:
 *   Default function called when a capture interrupt occurs.
 *   It reads the capture timer value and the interrupt edge. When positive
 *   edge triggered the interrupt, the current capture value is stored and
 *   the interrupt edge is modified to falling edge.
 *   When the negative edge triggers the interrupt, the timer count
 *   difference is calculated and the high time period is obtained.
 *
 *   Two pulses are required to properly calculate the frequency.
 *
 * Input Parameters:
 *   irq     - The interrupt request number.
 *   context - Pointer to the interrupt context.
 *   arg     - Pointer to the argument to be passed to the ISR.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int IRAM_ATTR mcpwm_capture_driver_isr_default(int irq, void *context,
                                                      void *arg)
{
  struct mcpwm_dev_common_s *common = (struct mcpwm_dev_common_s *)arg;
  struct mcpwm_cap_channel_lowerhalf_s *lower = NULL;
  struct mcpwm_capture_event_data_s *data = NULL;
  irqstate_t flags;
  uint32_t cap_value;
  uint32_t status;
  mcpwm_capture_edge_t cap_edge;

  flags = spin_lock_irqsave(common->mcpwm_spinlock);
  status = mcpwm_ll_intr_get_status(common->hal.dev);

  if (status & MCPWM_LL_EVENT_CAPTURE(MCPWM_CAP_CHANNEL_0))
    {
#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0
      lower = &mcpwm_cap_ch0_lowerhalf;
#endif
    }
#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH1
  else if (status & MCPWM_LL_EVENT_CAPTURE(MCPWM_CAP_CHANNEL_1))
    {
      lower = &mcpwm_cap_ch1_lowerhalf;
    }
#endif
#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH2
  else if (status & MCPWM_LL_EVENT_CAPTURE(MCPWM_CAP_CHANNEL_2))
    {
      lower = &mcpwm_cap_ch2_lowerhalf;
    }
#endif
  else
    {
      return -ERANGE;
    }

  mcpwm_ll_intr_clear_status(common->hal.dev,
                             status &
                             MCPWM_LL_EVENT_CAPTURE(lower->channel_id));

  /* At least 4 interrupts are required on a channel before a
   * frequency measurement can be executed, otherwise you can
   * obtain negative values.
   */

  if (lower->isr_count < 4)
    {
      lower->isr_count++;
    }

  if (!lower->ready && (lower->isr_count >= 4))
    {
      cpinfo("Channel %d ready\n", lower->channel_id);
      lower->ready = true;
    }

  data = lower->data;
  cap_value = mcpwm_ll_capture_get_value(common->hal.dev,
                                         lower->channel_id);
  cap_edge = mcpwm_ll_capture_get_edge(common->hal.dev,
                                       lower->channel_id);

  if (cap_edge == MCPWM_CAP_EDGE_POS)
    {
      data->last_pos_edge_count = data->pos_edge_count;
      data->pos_edge_count = cap_value;
      data->neg_edge_count = data->pos_edge_count;
      mcpwm_ll_capture_enable_negedge(common->hal.dev,
                                      lower->channel_id,
                                      true);
      mcpwm_ll_capture_enable_posedge(common->hal.dev,
                                      lower->channel_id,
                                      false);
    }
  else
    {
      data->neg_edge_count = cap_value;

      if (lower->ready)
        {
          uint32_t high_time = data->neg_edge_count - data->pos_edge_count;
          uint32_t period = data->pos_edge_count - data->last_pos_edge_count;

          if (period != 0)
            {
              lower->freq = lower->clock / period;
              lower->duty = 100 * high_time / period;
            }
        }

      mcpwm_ll_capture_enable_negedge(common->hal.dev,
                                      lower->channel_id,
                                      false);
      mcpwm_ll_capture_enable_posedge(common->hal.dev,
                                      lower->channel_id,
                                      true);
    }

  spin_unlock_irqrestore(common->mcpwm_spinlock, flags);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mcpwm_capture_initialize
 *
 * Description:
 *   This function initializes the specified MCPWM peripheral and the capture
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   channel - Channel to be initialized [0-3].
 *   pin     - GPIO pin assigned to this channel.
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the Capture device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
struct cap_lowerhalf_s *esp_mcpwm_capture_initialize(int channel, int pin)
{
  struct mcpwm_cap_channel_lowerhalf_s *lower = NULL;
  uint32_t group_clock;

  /* Single time initialization for the entire MCPWM Peripheral
   * and MCPWM Capture group.
   */

  if (!mcpwm_common.initialized)
    {
      esp_mcpwm_group_start();
    }

  if (!mcpwm_common.capture_initialized)
    {
      mcpwm_ll_capture_enable_timer(mcpwm_common.hal.dev, true);
      esp_mcpwm_capture_isr_register(mcpwm_capture_driver_isr_default,
                                     &mcpwm_common);
      mcpwm_common.capture_initialized = true;
    }

  switch (channel)
    {
#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0
      case MCPWM_CAP_CHANNEL_0:
        lower = &mcpwm_cap_ch0_lowerhalf;
        break;
#endif
#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH1
      case MCPWM_CAP_CHANNEL_1:
        lower = &mcpwm_cap_ch1_lowerhalf;
        break;
#endif
#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH2
      case MCPWM_CAP_CHANNEL_2:
        lower = &mcpwm_cap_ch2_lowerhalf;
        break;
#endif
      default:
        cperr("Invalid channel selection: %d\n", channel);
        return NULL;
    }

  esp_clk_tree_src_get_freq_hz(MCPWM_DEV_CLK_SOURCE,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &group_clock);

  /* Set the clock to be used when calculating frequency */

  lower->gpio_pin = pin;
  lower->clock = group_clock / MCPWM_DEV_CLK_PRESCALE;

  /* Configure GPIO pin */

  esp_mcpwm_capture_set_gpio(lower);

  return (struct cap_lowerhalf_s *)lower;
}
#endif

#endif /* CONFIG_ESP_MCPWM */
