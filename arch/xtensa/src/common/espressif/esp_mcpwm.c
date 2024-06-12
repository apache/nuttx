/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_mcpwm.c
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
#include <errno.h>
#include <debug.h>
#include <inttypes.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/capture.h>

#include "xtensa.h"
#ifdef CONFIG_ARCH_CHIP_ESP32
#include "hardware/esp32_soc.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"
#elif CONFIG_ARCH_CHIP_ESP32S3
#include "hardware/esp32s3_soc.h"
#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#endif

#include "hal/mcpwm_hal.h"
#include "hal/mcpwm_ll.h"
#include "soc/mcpwm_periph.h"
#include "periph_ctrl.h"
#include "hal/clk_tree_hal.h"

#ifdef CONFIG_ESP_MCPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCPWM_CAPTURE_DEFAULT_GROUP 0
#ifdef CONFIG_ARCH_CHIP_ESP32
#  define esp_configgpio      esp32_configgpio
#  define esp_gpio_matrix_in  esp32_gpio_matrix_in
#  define esp_setup_irq       esp32_setup_irq
#  define esp_teardown_irq    esp32_teardown_irq
#  define ESP_CPUINT_LEVEL    ESP32_CPUINT_LEVEL
#elif CONFIG_ARCH_CHIP_ESP32S3
#  define esp_configgpio      esp32s3_configgpio
#  define esp_gpio_matrix_in  esp32s3_gpio_matrix_in
#  define esp_setup_irq       esp32s3_setup_irq
#  define esp_teardown_irq    esp32s3_teardown_irq
#  define ESP_CPUINT_LEVEL    ESP32S3_CPUINT_LEVEL
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Capture event data. The 'last_' value is used to calculate frequency */

struct mcpwm_dev_common_s
{
  mcpwm_hal_init_config_t group;
  mcpwm_hal_context_t hal;
  spinlock_t mcpwm_spinlock;
  bool initialized;          /* MCPWM periph. and HAL has been initialized */
#ifdef CONFIG_ESP_MCPWM_CAPTURE
  bool capture_initialized;  /* Capture submodule has been initialized */
#endif
};

#ifdef CONFIG_ESP_MCPWM_CAPTURE
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

struct mcpwm_cap_channel_lowerhalf_s
{
  /* The following block is part of the upper-half device struct */

  FAR const struct cap_ops_s *ops;

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
static int esp_mcpwm_capture_isr_register(int (*fn)(int, void *, void *),
                                          void *arg);
static int IRAM_ATTR mcpwm_capture_driver_isr_default(int irq, void *context,
                                                      void *arg);
static int esp_mcpwm_capture_set_gpio(
  struct mcpwm_cap_channel_lowerhalf_s *lower);
#endif

/* Lower half methods required by capture driver */

static int esp_capture_start(struct cap_lowerhalf_s *lower);
static int esp_capture_stop(struct cap_lowerhalf_s *lower);
static int esp_capture_getduty(struct cap_lowerhalf_s *lower,
                               uint8_t *duty);
static int esp_capture_getfreq(struct cap_lowerhalf_s *lower,
                               uint32_t *freq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mcpwm_dev_common_s g_mcpwm_common =
{
  .group.group_id = 0,
  .initialized = false,
  .capture_initialized = false,
};

#ifdef CONFIG_ESP_MCPWM_CAPTURE
/* Lower half methods required by capture driver */

static const struct cap_ops_s mcpwm_cap_ops =
{
  .start       = esp_capture_start,
  .stop        = esp_capture_stop,
  .getduty     = esp_capture_getduty,
  .getfreq     = esp_capture_getfreq,
};

/* Data structures for the available capture channels */

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0
static struct mcpwm_capture_event_data_s event_data_ch0;
static struct mcpwm_cap_channel_lowerhalf_s mcpwm_cap_ch0_lowerhalf =
{
  .ops = &mcpwm_cap_ops,
  .common = &g_mcpwm_common,
  .data = &event_data_ch0,
  .channel_id = MCPWM_CAP_CHANNEL_0,
  .ready = false,
};
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH1
static struct mcpwm_capture_event_data_s event_data_ch1;
static struct mcpwm_cap_channel_lowerhalf_s mcpwm_cap_ch1_lowerhalf =
{
  .ops = &mcpwm_cap_ops,
  .common = &g_mcpwm_common,
  .data = &event_data_ch1,
  .channel_id = MCPWM_CAP_CHANNEL_1,
  .ready = false,
};
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE_CH2
static struct mcpwm_capture_event_data_s event_data_ch2;
static struct mcpwm_cap_channel_lowerhalf_s mcpwm_cap_ch2_lowerhalf =
{
  .ops = &mcpwm_cap_ops,
  .common = &g_mcpwm_common,
  .data = &event_data_ch2,
  .channel_id = MCPWM_CAP_CHANNEL_2,
  .ready = false,
};
#endif
#endif  /* CONFIG_ESP_MCPWM_CAPTURE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mcpwm_group_start
 *
 * Description:
 *   Initializes the PWM0 module and the MCPWM HAL.
 *   Should be called only once for the MCPWM peripheral.
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
  periph_module_enable(PERIPH_PWM0_MODULE);
  mcpwm_hal_init(&g_mcpwm_common.hal, &g_mcpwm_common.group);
  mcpwm_hal_timer_reset(&g_mcpwm_common.hal, 0);

  g_mcpwm_common.initialized = true;
}

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
  mcpwm_hal_context_t *hal = &priv->common->hal;
  irqstate_t flags;
  flags = spin_lock_irqsave(priv->common->mcpwm_spinlock);

  DEBUGASSERT(priv->common->initialized);

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
  spin_unlock_irqrestore(priv->common->mcpwm_spinlock, flags);

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
  mcpwm_hal_context_t *hal = &priv->common->hal;
  irqstate_t flags;
  flags = spin_lock_irqsave(priv->common->mcpwm_spinlock);

  /* Disable channel and interrupts */

  mcpwm_ll_capture_enable_channel(hal->dev, priv->channel_id, false);
  mcpwm_ll_intr_enable(hal->dev,
                       MCPWM_LL_EVENT_CAPTURE(priv->channel_id),
                       false);
  priv->enabled = false;

  cpinfo("Channel disabled: %d\n", priv->channel_id);
  spin_unlock_irqrestore(priv->common->mcpwm_spinlock, flags);

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
  *freq = priv->freq;
  cpinfo("Get freq called from channel %d\n", priv->channel_id);
  return OK;
}
#endif

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

  ret = esp_configgpio(lower->gpio_pin, INPUT_FUNCTION | INPUT_PULLUP);
  if (ret < 0)
    {
      cperr("Failed configuring GPIO pin\n");
      return ret;
    }

  esp_gpio_matrix_in(
    lower->gpio_pin,
    mcpwm_periph_signals.groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
      captures[lower->channel_id].cap_sig,
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
  int cpu = up_cpu_index();

  DEBUGASSERT(fn);

  cpuint = esp_setup_irq(cpu, mcpwm_periph_signals.groups[0].irq_id,
                         1, ESP_CPUINT_LEVEL);
  if (cpuint < 0)
    {
      cperr("Failed to allocate a CPU interrupt.\n");
      return -ENOMEM;
    }

  ret = irq_attach(mcpwm_periph_signals.groups[0].irq_id +
                   XTENSA_IRQ_FIRSTPERIPH, fn, arg);
  if (ret < 0)
    {
      cperr("Couldn't attach IRQ to handler.\n");
      esp_teardown_irq(cpu, mcpwm_periph_signals.groups[0].irq_id, cpuint);
      return ret;
    }

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
  uint32_t status;
  uint32_t cap_value;
  mcpwm_capture_edge_t cap_edge;
  irqstate_t flags;

  flags = spin_lock_irqsave(common->mcpwm_spinlock);
  status = mcpwm_ll_intr_get_status(common->hal.dev);

  /* Evaluate which channel triggered the capture interrupt */

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
  cap_value = mcpwm_ll_capture_get_value(common->hal.dev, lower->channel_id);
  cap_edge = mcpwm_ll_capture_get_edge(common->hal.dev, lower->channel_id);

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

  if (!g_mcpwm_common.initialized)
    {
      esp_mcpwm_group_start();
    }

  if (!g_mcpwm_common.capture_initialized)
    {
      esp_mcpwm_capture_isr_register(mcpwm_capture_driver_isr_default,
                                     &g_mcpwm_common);
      mcpwm_ll_capture_enable_timer(g_mcpwm_common.hal.dev, true);
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

  lower->clock = clk_hal_apb_get_freq_hz();
  lower->gpio_pin = pin;
  esp_mcpwm_capture_set_gpio(lower);

  return (struct cap_lowerhalf_s *) lower;
}
#endif

#endif /* CONFIG_ESP_MCPWM */
