/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_mcpwm.c
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
#ifdef CONFIG_ESP_MCPWM_CAPTURE
#include <nuttx/timers/capture.h>
#endif
#ifdef CONFIG_ESP_MCPWM_MOTOR
#include <nuttx/motor/motor.h>
#endif

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
#include "hal/mcpwm_types.h"
#include "soc/mcpwm_periph.h"
#include "periph_ctrl.h"
#include "esp_clk_tree.h"
#include "hal/clk_tree_hal.h"

#ifdef CONFIG_ESP_MCPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCPWM_CAPTURE_DEFAULT_GROUP 0
#ifdef CONFIG_ARCH_CHIP_ESP32
#  define esp_configgpio      esp32_configgpio
#  define esp_gpio_matrix_in  esp32_gpio_matrix_in
#  define esp_gpio_matrix_out esp32_gpio_matrix_out
#  define esp_setup_irq       esp32_setup_irq
#  define esp_teardown_irq    esp32_teardown_irq
#  define ESP_CPUINT_LEVEL    ESP32_CPUINT_LEVEL
#elif CONFIG_ARCH_CHIP_ESP32S3
#  define esp_configgpio      esp32s3_configgpio
#  define esp_gpio_matrix_in  esp32s3_gpio_matrix_in
#  define esp_gpio_matrix_out esp32s3_gpio_matrix_out
#  define esp_setup_irq       esp32s3_setup_irq
#  define esp_teardown_irq    esp32s3_teardown_irq
#  define ESP_CPUINT_LEVEL    ESP32S3_CPUINT_LEVEL
#endif
#ifdef CONFIG_ESP_MCPWM_MOTOR_BDC
/* Peak counter at 13330 in up-down mode allows frequencies at a prescale
 * of: 2 kHz @ 2; 1.5 kHz @ 3; 1.2 kHz @ 4; 1 kHz @ 5.
 */
#  define PEAK_COUNTER 13330
#  define MCPWM_MAX_PWM_OUT_FREQ 3000
#  define MCPWM_MIN_PWM_OUT_FREQ 25
#endif
#ifdef CONFIG_ESP_MCPMW_MOTOR_CH0_FAULT
#  define ESP_MCPMW_MOTOR_FAULT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  MCPWM_GENERATOR_0,
  MCPWM_GENERATOR_1,
  MCPWM_GENERATOR_MAX
} mcpwm_generator_e;

typedef enum
{
  MCPWM_OPERATOR_0,
  MCPWM_OPERATOR_1,
  MCPWM_OPERATOR_2,
  MCPWM_OPERATOR_MAX
} mcpwm_operator_e;

typedef enum
{
  MCPWM_TIMER_0,
  MCPWM_TIMER_1,
  MCPWM_TIMER_2,
  MCPWM_TIMER_MAX
} mcpwm_timer_e;

typedef enum
{
  MCPWM_FAULT_0,
  MCPWM_FAULT_1,
  MCPWM_FAULT_2,
  MCPWM_FAULT_MAX
} mcpwm_fault_e;

typedef enum
{
  MCPWM_MOTOR_CHANNEL_0,
  MCPWM_MOTOR_CHANNEL_1,
  MCPWM_MOTOR_CHANNEL_2,
  MCPWM_MOTOR_CHANNEL_MAX
} mcpwm_motor_channel_e;

enum mcpwm_capture_channel_e
{
  MCPWM_CAP_CHANNEL_0,
  MCPWM_CAP_CHANNEL_1,
  MCPWM_CAP_CHANNEL_2,
  MCPWM_CAP_CHANNEL_MAX
};

struct mcpwm_dev_common_s
{
  mcpwm_hal_init_config_t group;
  mcpwm_hal_context_t hal;
  spinlock_t mcpwm_spinlock;
  bool initialized;          /* MCPWM periph. and HAL has been initialized */
  bool isr_initialized;      /* Shared ISR has been initialized */
};

#ifdef CONFIG_ESP_MCPWM_MOTOR

struct mcpwm_motor_lowerhalf_s
{
  /* The following block is part of the upper-half device struct */

  const struct motor_ops_s *ops;    /* Arch-specific operations */
  uint8_t                  opmode;  /* Motor operation mode */
  uint8_t                  opflags; /* Motor operation flags */
  struct motor_limits_s    limits;  /* Motor absolute limits */
  struct motor_params_s    param;   /* Motor settings */
  struct motor_state_s     state;   /* Motor state */
  void                     *priv;   /* Private data */

  /* The following is private to the ESP MCPWM driver */

  struct mcpwm_dev_common_s *common;
  mcpwm_timer_e             timer_id;
  mcpwm_motor_channel_e     channel_id;
  mcpwm_operator_e          operator_id;
  uint32_t                  pwm_frequency;
  uint16_t                  counter_peak;
  uint8_t                   timer_prescale;
  int                       fault_pin;  /* GPIO Pin for fault detection */
  int                       generator_pins[MCPWM_GENERATOR_MAX];
#ifdef ESP_MCPMW_MOTOR_FAULT
  mcpwm_fault_e             fault_id;
#endif
};
#endif /* CONFIG_ESP_MCPWM_MOTOR */

#ifdef CONFIG_ESP_MCPWM_CAPTURE
/* Capture event data. The 'last_' value is used to calculate frequency */

struct mcpwm_capture_event_data_s
{
  uint32_t pos_edge_count;
  uint32_t neg_edge_count;
  uint32_t last_pos_edge_count;
};

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

/* General use for MCPWM peripheral */

static void esp_mcpwm_group_start(void);

/* MCPWM Capture specific */

#ifdef CONFIG_ESP_MCPWM_CAPTURE
static int esp_mcpwm_capture_set_gpio(
  struct mcpwm_cap_channel_lowerhalf_s *lower);

/* Upper-half functions required by capture driver */

static int esp_capture_start(struct cap_lowerhalf_s *lower);
static int esp_capture_stop(struct cap_lowerhalf_s *lower);
static int esp_capture_getduty(struct cap_lowerhalf_s *lower,
                               uint8_t *duty);
static int esp_capture_getfreq(struct cap_lowerhalf_s *lower,
                               uint32_t *freq);
#endif

/* MCPWM Motor Control */

#ifdef CONFIG_ESP_MCPWM_MOTOR

/* Upper-half functions required by motor driver */

static int esp_motor_setup(struct motor_lowerhalf_s *dev);
static int esp_motor_shutdown(struct motor_lowerhalf_s *dev);
static int esp_motor_stop(struct motor_lowerhalf_s *dev);
static int esp_motor_start(struct motor_lowerhalf_s *dev);
static int esp_motor_mode_set(struct motor_lowerhalf_s *dev, uint8_t mode);
static int esp_motor_fault_set(struct motor_lowerhalf_s *dev, uint8_t fault);
static int esp_motor_fault_get(struct motor_lowerhalf_s *dev,
                               uint8_t *fault);
static int esp_motor_params_set(struct motor_lowerhalf_s *dev,
                                struct motor_params_s *param);
static int esp_motor_state_get(struct motor_lowerhalf_s *dev,
                               struct motor_state_s *state);
static int esp_motor_limits_set(struct motor_lowerhalf_s *dev,
                                struct motor_limits_s *limits);
static int esp_motor_fault_clear(struct motor_lowerhalf_s *dev,
                                 uint8_t fault);
static int esp_motor_ioctl(struct motor_lowerhalf_s *dev, int cmd,
                           unsigned long arg);

/* Lower-half motor functions */

static int esp_mcpwm_motor_set_gpio(
  struct mcpwm_motor_lowerhalf_s *lower, bool enable);
static int esp_motor_set_duty_cycle(
  struct mcpwm_motor_lowerhalf_s *lower, float duty);
static int esp_motor_pwm_config(struct mcpwm_motor_lowerhalf_s *lower);
static int esp_motor_bdc_set_direction(
  struct mcpwm_motor_lowerhalf_s *lower);
#endif

/* MCPWM Fault Control */

#ifdef ESP_MCPMW_MOTOR_FAULT
static int esp_mcpwm_fault_gpio_config(
  struct mcpwm_motor_lowerhalf_s *lower, bool enable);
static int esp_motor_fault_configure(
  struct mcpwm_motor_lowerhalf_s *lower, bool enable);
#endif

/* MCPWM Interrupt */

#if defined(CONFIG_ESP_MCPWM_CAPTURE) || defined(ESP_MCPMW_MOTOR_FAULT)
static int esp_mcpwm_isr_register(int (*fn)(int, void *, void *), void *arg);
static int IRAM_ATTR mcpwm_driver_isr_default(int irq, void *context,
                                              void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common MCPWM data structure */

static struct mcpwm_dev_common_s g_mcpwm_common =
{
  .group.group_id      = 0,
  .initialized         = false,
  .isr_initialized     = false,
};

/* Motor specific data structures */

#ifdef CONFIG_ESP_MCPWM_MOTOR
static const struct motor_ops_s mcpwm_motor_ops =
{
  .setup       = esp_motor_setup,
  .shutdown    = esp_motor_shutdown,
  .stop        = esp_motor_stop,
  .start       = esp_motor_start,
  .params_set  = esp_motor_params_set,
  .mode_set    = esp_motor_mode_set,
  .limits_set  = esp_motor_limits_set,
  .fault_set   = esp_motor_fault_set,
  .state_get   = esp_motor_state_get,
  .fault_get   = esp_motor_fault_get,
  .fault_clear = esp_motor_fault_clear,
  .ioctl       = esp_motor_ioctl,
};

#if defined(CONFIG_ESP_MCPWM_MOTOR_CH0) &&\
    defined(CONFIG_ESP_MCPWM_MOTOR_BDC)
static struct mcpwm_motor_lowerhalf_s mcpwm_bdc_ch0_lowerhalf =
{
  .ops          = &mcpwm_motor_ops,
  .common       = &g_mcpwm_common,
  .channel_id   = MCPWM_MOTOR_CHANNEL_0,
  .timer_id     = MCPWM_TIMER_0,
  .operator_id  = MCPWM_OPERATOR_0,
  .counter_peak = PEAK_COUNTER,
#ifdef ESP_MCPMW_MOTOR_FAULT
  .fault_id     = MCPWM_FAULT_0,
#endif
};
#endif /* CONFIG_ESP_MCPWM_MOTOR_BDC_CH0 && CONFIG_ESP_MCPWM_MOTOR_BDC */
#endif /* CONFIG_ESP_MCPWM_MOTOR */

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
  .ops        = &mcpwm_cap_ops,
  .common     = &g_mcpwm_common,
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
  .common     = &g_mcpwm_common,
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
  .common     = &g_mcpwm_common,
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
  g_mcpwm_common.initialized = true;
}

/****************************************************************************
 * Name: esp_motor_setup
 *
 * Description:
 *   Configures the MCPWM operator and generator, setting the PWM clock and
 *   output pins.
 *   If required, also configures fault detection. When done, sets the the
 *   motor state to IDLE.
 *
 * Input Parameters:
 *   dev - Pointer to the motor channel lower-half data structure.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_setup(struct motor_lowerhalf_s *dev)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *)dev;
  mcpwm_hal_context_t *hal = &priv->common->hal;
  uint32_t base_clock;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&g_mcpwm_common.mcpwm_spinlock);
  if ((priv->state.state == MOTOR_STATE_FAULT) ||
      (priv->state.state == MOTOR_STATE_CRITICAL))
    {
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      mtrerr("Motor is in fault state. Clear faults first\n");
      return ERROR;
    }

  mtrinfo("State: %d\n", priv->state.state);

  esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_PLL_F160M,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &base_clock);

  mcpwm_hal_timer_reset(hal, priv->timer_id);
  mcpwm_hal_operator_reset(hal, priv->operator_id);

  /* Setup control in UP-DOWN mode for duty cycle control
   * Peak value modifies the maximum and minimum possible frequencies.
   * Important: the HAL subtracts 1 from the target value.
   */

  priv->timer_prescale = \
    base_clock / ((2 * priv->counter_peak + 1) * priv->pwm_frequency);

  mcpwm_ll_timer_set_peak(hal->dev, priv->timer_id,
                          priv->counter_peak, true);
  mcpwm_ll_timer_set_clock_prescale(hal->dev, priv->timer_id,
                                    priv->timer_prescale);
  mcpwm_ll_timer_update_period_at_once(hal->dev, priv->timer_id);
  mcpwm_ll_timer_enable_update_period_on_tez(hal->dev, priv->timer_id, true);
  mcpwm_ll_timer_set_count_mode(hal->dev, priv->timer_id,
                                MCPWM_TIMER_COUNT_MODE_UP_DOWN);

  mcpwm_ll_operator_flush_shadow(hal->dev, priv->operator_id);
  mcpwm_ll_operator_connect_timer(hal->dev, priv->operator_id,
                                  priv->timer_id);
  mcpwm_ll_operator_enable_update_compare_on_tez(hal->dev, priv->operator_id,
                                                 MCPWM_GENERATOR_0, true);

#ifdef ESP_MCPMW_MOTOR_FAULT
  esp_motor_fault_configure(priv, true);
#endif
  priv->state.state = MOTOR_STATE_IDLE;

  spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
  mtrinfo("Channel %d starts: prescale %d | freq: %"PRIu32"\n",
          priv->channel_id, priv->timer_prescale - 1, priv->pwm_frequency);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_shutdown
 *
 * Description:
 *   Stop the PWM timer and disable output on GPIO matrix.
 *
 * Input Parameters:
 *   dev - Pointer to the motor channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_shutdown(struct motor_lowerhalf_s *dev)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *)dev;
  mcpwm_hal_context_t *hal = &priv->common->hal;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_mcpwm_common.mcpwm_spinlock);

  /* Stop the motor */

  esp_motor_stop(dev);

  /* Stop the PWM timer */

  mcpwm_ll_timer_set_count_mode(hal->dev, priv->timer_id,
                                MCPWM_TIMER_COUNT_MODE_PAUSE);
  mcpwm_ll_timer_set_start_stop_command(hal->dev, priv->timer_id,
                                        MCPWM_TIMER_STOP_EMPTY);

  /* Disable fault detection */

#ifdef ESP_MCPMW_MOTOR_FAULT
  esp_motor_fault_configure(priv, false);
#endif

  spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_stop
 *
 * Description:
 *   Holds the motor at a stand-still. PWM_A and PWM_B are kept low.
 *
 * Input Parameters:
 *   dev - Pointer to the motor channel lower-half data structure.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_stop(struct motor_lowerhalf_s *dev)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *)dev;
  mcpwm_hal_context_t *hal = &priv->common->hal;
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&g_mcpwm_common.mcpwm_spinlock);
  if (priv->state.state == MOTOR_STATE_IDLE)
    {
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      mtrerr("Motor already stopped\n");
      return -EPERM;
    }

  mcpwm_ll_timer_set_start_stop_command(hal->dev, priv->timer_id,
                                        MCPWM_TIMER_STOP_EMPTY);
  mcpwm_ll_gen_set_continue_force_level(hal->dev,
                                        priv->operator_id,
                                        MCPWM_GENERATOR_0, 0);
  mcpwm_ll_gen_set_continue_force_level(hal->dev,
                                        priv->operator_id,
                                        MCPWM_GENERATOR_1, 0);

  ret = esp_motor_set_duty_cycle(priv, 0.0);
  if (ret < 0)
    {
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      mtrerr("Failed setting duty cycle to 0 on stop: %d\n", ret);
      return ret;
    }

  if ((priv->state.state == MOTOR_STATE_FAULT) ||
      (priv->state.state == MOTOR_STATE_CRITICAL))
    {
      mtrinfo("Channel %d stopped in fault state\n", priv->channel_id);
    }
  else
    {
      priv->state.state = MOTOR_STATE_IDLE;
    }

  priv->param.lock = false;
  spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
  mtrinfo("Channel %d stopped\n", priv->channel_id);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_start
 *
 * Description:
 *   Start the motor by disabling forced actions and setting the duty cycle.
 *   The motor parameters must have been set before calling this function.
 *
 * Input Parameters:
 *   dev - Pointer to the motor channel lower-half data structure.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_start(struct motor_lowerhalf_s *dev)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *)dev;

  mcpwm_hal_context_t *hal = &priv->common->hal;
  irqstate_t flags;
  int ret;
  float duty;

  flags = spin_lock_irqsave(&g_mcpwm_common.mcpwm_spinlock);
  if (priv->state.state == MOTOR_STATE_RUN)
    {
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      mtrerr("Motor already running\n");
      return -EINVAL;
    }

  if ((priv->state.state == MOTOR_STATE_CRITICAL) ||
       (priv->state.state == MOTOR_STATE_FAULT))
    {
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      mtrerr("Motor is in fault state\n");
      return -EINVAL;
    }

  ret = esp_motor_pwm_config(priv);
  if (ret < 0)
    {
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      mtrerr("Failed setting PWM configuration\n");
      return ret;
    }

  /* Set motor direction */

  esp_motor_bdc_set_direction(priv);
  mcpwm_ll_timer_set_start_stop_command(hal->dev, priv->timer_id,
                                        MCPWM_TIMER_START_NO_STOP);

  /* Set duty cycle based on motor parameter and limits */

#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
  if (priv->opmode == MOTOR_OPMODE_SPEED)
    {
      duty = priv->param.speed / priv->limits.speed;
      ret = esp_motor_set_duty_cycle(priv, duty);
      if (ret < 0)
        {
          spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
          mtrerr("Failed starting motor\n");
          return ret;
        }
    }
#endif

  priv->state.state = MOTOR_STATE_RUN;
  spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
  mtrinfo("Motor start\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_params_set
 *
 * Description:
 *   Set parameters to run the motor.
 *
 * Input Parameters:
 *   dev   - Pointer to the motor channel lower-half data structure.
 *   param - Pointer to the motor parameter structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_params_set(struct motor_lowerhalf_s *dev,
  struct motor_params_s *param)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(param != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
  priv->param.position = param->position;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
  priv->param.speed = param->speed;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
  priv->param.torque = param->torque;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
  priv->param.force = param->force;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_ACCELERATION
  priv->param.acceleration = param->acceleration;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_DECELERATION
  priv->param.deceleration = param->deceleration;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_DIRECTION
  priv->param.direction = param->direction;
#endif
  priv->param.lock = false;

  /* Refresh duty and direction on the go */

  if (priv->state.state == MOTOR_STATE_RUN)
    {
      float duty = priv->param.speed / priv->limits.speed;
      esp_motor_set_duty_cycle(priv, duty);
      esp_motor_bdc_set_direction(priv);
    }

  mtrinfo("Motor parameters set\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_mode_set
 *
 * Description:
 *   Sets the motor operating mode.
 *
 * Input Parameters:
 *   dev  - Pointer to the motor channel lower-half data structure.
 *   mode - Must be one of the motor_opmode_e enum.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_mode_set(struct motor_lowerhalf_s *dev, uint8_t mode)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

  if (mode > MOTOR_OPMODE_PATTERN)
    {
      mtrerr("Invalid operation mode: %u\n", mode);
      return -EINVAL;
    }

  priv->opmode = mode;
  mtrinfo("Mode set: %u\n", priv->opmode);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_limits_set
 *
 * Description:
 *   Set motor limits. Must be called before start the motor.
 *
 * Input Parameters:
 *   dev    - Pointer to the motor channel lower-half data structure.
 *   limits - Pointer to the motor limits data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_limits_set(struct motor_lowerhalf_s *dev,
  struct motor_limits_s *limits)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(limits != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

#ifdef CONFIG_MOTOR_UPPER_HAVE_POSITION
  priv->limits.position = limits->position;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_SPEED
  priv->limits.speed = limits->speed;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_TORQUE
  priv->limits.torque = limits->torque;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_FORCE
  priv->limits.force = limits->force;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_ACCELERATION
  priv->limits.acceleration = limits->acceleration;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_DECELERATION
  priv->limits.deceleration = limits->deceleration;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_VOLTAGE
  priv->limits.v_in = limits->v_in;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_CURRENT
  priv->limits.i_in = limits->i_in;
#endif
#ifdef CONFIG_MOTOR_UPPER_HAVE_INPUT_POWER
  priv->limits.p_in = limits->p_in;
#endif
  priv->limits.lock = true;
  mtrinfo("limits set and locked %d\n", priv->limits.lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_fault_set
 *
 * Description:
 *   Sets the fault state for the motor.
 *
 * Input Parameters:
 *   lower - Pointer to the motor channel lower-half data structure.
 *   fault - Fault value. Must be one of motor_fault_e enum.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_fault_set(struct motor_lowerhalf_s *dev, uint8_t fault)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

  if (fault > MOTOR_FAULT_OTHER)
    {
      mtrerr("Invalid fault value: %u\n", fault);
      return -EINVAL;
    }

  priv->state.state = MOTOR_STATE_FAULT;
  priv->state.fault |= fault;
  mtrinfo("%u\n", priv->state.fault);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_state_get
 *
 * Description:
 *   Get the current motor state.
 *
 * Input Parameters:
 *   dev   - Pointer to the motor channel lower-half data structure.
 *   state - Pointer to the motor state data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_state_get(struct motor_lowerhalf_s *dev,
  struct motor_state_s *state)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(state != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

  state->state = priv->state.state;
  state->fault = priv->state.fault;
  memcpy(&state->state, &priv->state, sizeof(struct motor_feedback_s));
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_fault_get
 *
 * Description:
 *   Get current motor fault state.
 *
 * Input Parameters:
 *   dev   - Pointer to the motor channel lower-half data structure.
 *   fault - Fault state value. Must be one of motor_fault_e enum.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_fault_get(struct motor_lowerhalf_s *dev, uint8_t *fault)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(fault != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

  *fault = priv->state.fault;
  mtrinfo("%u\n", priv->state.fault);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_fault_clear
 *
 * Description:
 *   Clears a motor fault.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   fault - Fault state to clear (one of motor_fault_e enum).
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_fault_clear(struct motor_lowerhalf_s *dev,
  uint8_t fault)
{
  DEBUGASSERT(dev != NULL);

  struct mcpwm_motor_lowerhalf_s *priv = (
    struct mcpwm_motor_lowerhalf_s *) dev;

  if (fault > MOTOR_FAULT_OTHER)
    {
      mtrerr("Invalid fault value: %u\n", fault);
      return -EINVAL;
    }

  priv->state.fault &= ~fault;
  if (priv->state.fault == 0)
    {
      priv->state.state = MOTOR_STATE_IDLE;
      mtrinfo("All faults clear\n");
      return OK;
    }

  mtrinfo("Fault clear: %u\n", fault);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_ioctl
 *
 * Description:
 *  Unused but required for upper-half motor driver.
 *
 * Input Parameters:
 *   dev - Pointer to the motor channel lower-half data structure.
 *   cmd - Custom IOCTL.
 *   arg - Argument to be passed for custom IOCTL.
 *
 * Returned Value:
 *   Returns 1.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_ioctl(struct motor_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  return 1;
}
#endif

/****************************************************************************
 * Name: esp_motor_fault_configure
 *
 * Description:
 *   Configure fault detection for motor channel. Enables the interrupt
 *   handling to set and clear fault state and sets brake action on fault
 *   (holds PWM at low state) in one-shot mode.
 *
 * Input Parameters:
 *   dev    - Pointer to the motor channel lower-half data structure.
 *   enable - True to setup motor fault. False to disable fault detection.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef ESP_MCPMW_MOTOR_FAULT
static int esp_motor_fault_configure(struct mcpwm_motor_lowerhalf_s *lower,
  bool enable)
{
  DEBUGASSERT(lower != NULL);

  mcpwm_hal_context_t *hal = &lower->common->hal;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_mcpwm_common.mcpwm_spinlock);
  if (!enable)
    {
      mcpwm_ll_fault_enable_detection(hal->dev, lower->fault_id, false);
      mcpwm_ll_intr_enable(hal->dev,
                           MCPWM_LL_EVENT_FAULT_ENTER(lower->fault_id),
                           false);
      mcpwm_ll_intr_enable(hal->dev,
                           MCPWM_LL_EVENT_FAULT_EXIT(lower->fault_id),
                           false);
      spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
      return OK;
    }

  /* Detect fault when signal is high and also enable software trigger */

  mcpwm_ll_fault_set_active_level(hal->dev, lower->fault_id, true);
  mcpwm_ll_fault_enable_detection(hal->dev, lower->fault_id, true);
  mcpwm_ll_brake_enable_oneshot_mode(hal->dev, lower->operator_id,
                                     lower->fault_id, true);
  mcpwm_ll_brake_enable_soft_ost(hal->dev, lower->operator_id, true);

  /* Make sure the brake event can be triggered when the timer is
   * counting up AND down because it is running in up-down mode.
   */

  mcpwm_ll_generator_set_action_on_brake_event(hal->dev,
                                               lower->operator_id,
                                               MCPWM_GENERATOR_0,
                                               MCPWM_TIMER_DIRECTION_UP,
                                               MCPWM_OPER_BRAKE_MODE_OST,
                                               1);
  mcpwm_ll_generator_set_action_on_brake_event(hal->dev,
                                               lower->operator_id,
                                               MCPWM_GENERATOR_0,
                                               MCPWM_TIMER_DIRECTION_DOWN,
                                               MCPWM_OPER_BRAKE_MODE_OST,
                                               1);
  mcpwm_ll_generator_set_action_on_brake_event(hal->dev,
                                               lower->operator_id,
                                               MCPWM_GENERATOR_1,
                                               MCPWM_TIMER_DIRECTION_UP,
                                               MCPWM_OPER_BRAKE_MODE_OST,
                                               1);
  mcpwm_ll_generator_set_action_on_brake_event(hal->dev,
                                               lower->operator_id,
                                               MCPWM_GENERATOR_1,
                                               MCPWM_TIMER_DIRECTION_DOWN,
                                               MCPWM_OPER_BRAKE_MODE_OST,
                                               1);

  /* Enable interrupt requests for motor fault */

  mcpwm_ll_brake_clear_ost(hal->dev, lower->operator_id);
  mcpwm_ll_intr_enable(hal->dev,
                       MCPWM_LL_EVENT_FAULT_ENTER(lower->fault_id),
                       true);
  mcpwm_ll_intr_enable(hal->dev,
                       MCPWM_LL_EVENT_FAULT_EXIT(lower->fault_id),
                       true);
  mcpwm_ll_intr_clear_status(hal->dev,
                             MCPWM_LL_EVENT_FAULT_ENTER(lower->fault_id));
  mcpwm_ll_intr_clear_status(hal->dev,
                             MCPWM_LL_EVENT_FAULT_EXIT(lower->fault_id));

  spin_unlock_irqrestore(&g_mcpwm_common.mcpwm_spinlock, flags);
  mtrinfo("Brake configured for motor channel %d on fault id %d",
          lower->channel_id, lower->fault_id);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_set_duty_cycle
 *
 * Description:
 *   Sets the duty cycle on output PWM_A and PWM_B.
 *
 * Input Parameters:
 *   lower - Pointer to the motor channel lower-half data structure.
 *   duty  - Duty-cycle value from 0.0 to 1.0.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_set_duty_cycle(struct mcpwm_motor_lowerhalf_s *lower,
  float duty)
{
  DEBUGASSERT(lower != NULL);

  mcpwm_hal_context_t *hal = &lower->common->hal;

  if (duty < 0.0 || duty > 1.0)
    {
      mtrerr("Invalid duty cycle value: %f\n", duty);
      return -EINVAL;
    }

  uint32_t pwm_count = -1 * lower->counter_peak * (duty - 1.0);
  mcpwm_ll_operator_set_compare_value(hal->dev, lower->operator_id,
                                      MCPWM_GENERATOR_0, pwm_count);
  mtrinfo("Duty %f compare value set: %u\n", duty, pwm_count);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_bdc_set_direction
 *
 * Description:
 *   Sets direction of motor spin by disabling PWM_A or PWM_B. If
 *   CONFIG_MOTOR_UPPER_HAVE_DIRECTION is not defined, defaults to CW.
 *
 * Input Parameters:
 *   lower - Pointer to the motor channel lower-half data structure.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_bdc_set_direction(struct mcpwm_motor_lowerhalf_s *lower)
{
  DEBUGASSERT(lower != NULL);

  mcpwm_hal_context_t *hal = &lower->common->hal;
  int8_t direction;

  if (lower->opmode == MOTOR_OPMODE_SPEED)
    {
#ifdef CONFIG_MOTOR_UPPER_HAVE_DIRECTION
      if (lower->param.direction == MOTOR_DIR_CW)
        {
          mcpwm_ll_gen_disable_continue_force_action(hal->dev,
                                                     lower->operator_id,
                                                     MCPWM_GENERATOR_0);
          mcpwm_ll_gen_set_continue_force_level(hal->dev,
                                                lower->operator_id,
                                                MCPWM_GENERATOR_1, 0);
        }
      else
        {
          mcpwm_ll_gen_disable_continue_force_action(hal->dev,
                                                     lower->operator_id,
                                                     MCPWM_GENERATOR_1);
          mcpwm_ll_gen_set_continue_force_level(hal->dev,
                                                lower->operator_id,
                                                MCPWM_GENERATOR_0, 0);
          direction = lower->param.direction;
        }
#else
      mcpwm_ll_gen_disable_continue_force_action(hal->dev,
                                                  lower->operator_id,
                                                  MCPWM_GENERATOR_0);
      mcpwm_ll_gen_set_continue_force_level(hal->dev,
                                            lower->operator_id,
                                            MCPWM_GENERATOR_1, 0);
      direction = MOTOR_DIR_CW;
#endif
    }

  mtrinfo("Motor direction set: %d\n", direction);
  return OK;
}
#endif

/****************************************************************************
 * Name: esp_motor_pwm_config
 *
 * Description:
 *  Configures the control waveform by setting some configurations for
 *  timer events. Included configurations are: speed control.
 *
 * Input Parameters:
 *   lower - Pointer to the motor channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_motor_pwm_config(struct mcpwm_motor_lowerhalf_s *lower)
{
  mcpwm_hal_context_t *hal = &lower->common->hal;

  /* Configure PWM_A and PWM_B as complementary */

  if (lower->opmode == MOTOR_OPMODE_SPEED)
    {
      /* PWM_A Output */

      mcpwm_ll_generator_set_action_on_compare_event(hal->dev,
        lower->operator_id,
        MCPWM_GENERATOR_0,
        MCPWM_TIMER_DIRECTION_UP,
        MCPWM_GENERATOR_0,
        MCPWM_GEN_ACTION_HIGH);
      mcpwm_ll_generator_set_action_on_compare_event(
        hal->dev,
        lower->operator_id,
        MCPWM_GENERATOR_0,
        MCPWM_TIMER_DIRECTION_DOWN,
        MCPWM_GENERATOR_0,
        MCPWM_GEN_ACTION_LOW);

      /* PWM_B Output */

      mcpwm_ll_generator_set_action_on_compare_event(hal->dev,
        lower->operator_id,
        MCPWM_GENERATOR_1,
        MCPWM_TIMER_DIRECTION_UP,
        MCPWM_GENERATOR_0,
        MCPWM_GEN_ACTION_HIGH);
      mcpwm_ll_generator_set_action_on_compare_event(hal->dev,
        lower->operator_id,
        MCPWM_GENERATOR_1,
        MCPWM_TIMER_DIRECTION_DOWN,
        MCPWM_GENERATOR_0,
        MCPWM_GEN_ACTION_LOW);
    }
  else
    {
      mtrerr("Invalid operation mode\n");
      return -EPERM;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_mcpwm_fault_configure
 *
 * Description:
 *   Configures the fault GPIO.
 *
 * Input Parameters:
 *   lower  - Pointer to the motor channel lower-half data structure.
 *   enable - True to configure motor fault. False to disable.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef ESP_MCPMW_MOTOR_FAULT
static int esp_mcpwm_fault_gpio_config(struct mcpwm_motor_lowerhalf_s *lower,
  bool enable)
{
  int ret;

  if (!enable)
    {
      esp_gpio_matrix_in(0x3a,
        mcpwm_periph_signals.groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
        gpio_faults[lower->fault_id].fault_sig,
        false);
      return OK;
    }

  ret = esp_configgpio(lower->fault_pin, INPUT_PULLDOWN);
  if (ret < 0)
    {
      mtrerr("Failed configuring fault GPIO\n");
      return ret;
    }

  esp_gpio_matrix_in(
    lower->fault_pin,
    mcpwm_periph_signals.groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
    gpio_faults[lower->fault_id].fault_sig,
    false);

  mtrinfo("Fault signal configured for GPIO %d in channel %d\n",
          lower->fault_pin, lower->channel_id);

  return ret;
}
#endif

/****************************************************************************
 * Name: esp_mcpwm_motor_set_gpio
 *
 * Description:
 *   Configures the GPIO pins to be used as motor PWM output and the fault
 *   GPIO (if enabled).
 *
 * Input Parameters:
 *   lower  - Pointer to the motor channel lower-half data structure.
 *   enable - True to configure GPIO for motor PWM. False to disable.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR
static int esp_mcpwm_motor_set_gpio(struct mcpwm_motor_lowerhalf_s *lower,
  bool enable)
{
  int ret;

  if (!enable)
    {
      esp_gpio_matrix_out(lower->generator_pins[MCPWM_GENERATOR_0], 0x100,
                          false, false);

      esp_gpio_matrix_out(lower->generator_pins[MCPWM_GENERATOR_1], 0x100,
                          false, false);
      return OK;
    }

  ret = esp_configgpio(lower->generator_pins[MCPWM_GENERATOR_0],
                       OUTPUT_FUNCTION);
  if (ret < 0)
    {
      mtrerr("Failed configuring PWM_A GPIO\n");
      return ret;
    }

  ret = esp_configgpio(lower->generator_pins[MCPWM_GENERATOR_1],
                       OUTPUT_FUNCTION);
  if (ret < 0)
    {
      mtrerr("Failed configuring PWM_B GPIO\n");
      return ret;
    }

  esp_gpio_matrix_out(
    lower->generator_pins[MCPWM_GENERATOR_0],
    mcpwm_periph_signals.groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
    operators[lower->channel_id].generators[MCPWM_GENERATOR_0].pwm_sig,
    false, false);

  esp_gpio_matrix_out(
    lower->generator_pins[MCPWM_GENERATOR_1],
    mcpwm_periph_signals.groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
    operators[lower->channel_id].generators[MCPWM_GENERATOR_1].pwm_sig,
    false, false);

  /* Connects the PWM output to the Capture input */

#ifdef CONFIG_ESP_MCPWM_TEST_LOOPBACK
  esp_gpio_matrix_out(CONFIG_ESP_MCPWM_CAPTURE_CH0_GPIO,
                      mcpwm_periph_signals.\
                      groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
                      operators[lower->channel_id].\
                      generators[MCPWM_GENERATOR_0].pwm_sig,
                      0, 0);
  esp_gpio_matrix_out(CONFIG_ESP_MCPWM_CAPTURE_CH1_GPIO,
                      mcpwm_periph_signals.\
                      groups[MCPWM_CAPTURE_DEFAULT_GROUP].\
                      operators[lower->channel_id].\
                      generators[MCPWM_GENERATOR_1].pwm_sig,
                      0, 0);
  mtrinfo("Loopback for capture device is enabled\n");
#endif

  mtrinfo("GPIO: %d PWM_A configured for channel %d\n",
         lower->generator_pins[MCPWM_GENERATOR_0],
         lower->channel_id);
  mtrinfo("GPIO: %d PWM_B configured for channel %d\n",
         lower->generator_pins[MCPWM_GENERATOR_1],
         lower->channel_id);

  return ret;
}
#endif

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
  flags = spin_lock_irqsave(&priv->common->mcpwm_spinlock);

  DEBUGASSERT(priv->common->initialized);

  /* Enable channel and interruption for rising edge */

  mcpwm_ll_capture_enable_timer(hal->dev, true);
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
  spin_unlock_irqrestore(&priv->common->mcpwm_spinlock, flags);

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
  flags = spin_lock_irqsave(&priv->common->mcpwm_spinlock);

  /* Disable channel and interrupts */

  mcpwm_ll_capture_enable_timer(hal->dev, false);
  mcpwm_ll_capture_enable_channel(hal->dev, priv->channel_id, false);
  mcpwm_ll_intr_enable(hal->dev,
                       MCPWM_LL_EVENT_CAPTURE(priv->channel_id),
                       false);
  priv->enabled = false;

  cpinfo("Channel disabled: %d\n", priv->channel_id);
  spin_unlock_irqrestore(&priv->common->mcpwm_spinlock, flags);

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

#if defined(CONFIG_ESP_MCPWM_CAPTURE) || defined(ESP_MCPMW_MOTOR_FAULT)
static int esp_mcpwm_isr_register(int (*fn)(int, void *, void *), void *arg)
{
  int cpuint;
  int ret;
  int cpu = this_cpu();

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
 * Name: mcpwm_driver_isr_default
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

#if defined(CONFIG_ESP_MCPWM_CAPTURE) || defined(ESP_MCPMW_MOTOR_FAULT)
static int IRAM_ATTR mcpwm_driver_isr_default(int irq, void *context,
                                              void *arg)
{
  struct mcpwm_dev_common_s *common = (struct mcpwm_dev_common_s *)arg;
  uint32_t status;
  irqstate_t flags;
#ifdef CONFIG_ESP_MCPWM_CAPTURE
  struct mcpwm_cap_channel_lowerhalf_s *lower = NULL;
  struct mcpwm_capture_event_data_s *data = NULL;
  uint32_t cap_value;
  mcpwm_capture_edge_t cap_edge;
#endif
#ifdef CONFIG_ESP_MCPWM_MOTOR
  struct mcpwm_motor_lowerhalf_s *priv = NULL;
#endif

  flags = spin_lock_irqsave(&common->mcpwm_spinlock);
  status = mcpwm_ll_intr_get_status(common->hal.dev);

  /* Evaluate capture interrupt for all 3 cap channels */

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

  /* Evaluate fault interrupt which can be of the ENTER or EXIT type */

#ifdef CONFIG_ESP_MCPMW_MOTOR_CH0_FAULT
  if (status & MCPWM_LL_EVENT_FAULT_ENTER(MCPWM_FAULT_0))
    {
      priv = &mcpwm_bdc_ch0_lowerhalf;
      mcpwm_ll_intr_clear_status(common->hal.dev,
                                 status &
                                 MCPWM_LL_EVENT_FAULT_ENTER(MCPWM_FAULT_0));
      esp_motor_fault_set((struct motor_lowerhalf_s *)priv,
                          MOTOR_FAULT_OTHER);
      mcpwm_ll_brake_trigger_soft_ost(common->hal.dev, priv->operator_id);
    }

  if (status & MCPWM_LL_EVENT_FAULT_EXIT(MCPWM_FAULT_0))
    {
      priv = &mcpwm_bdc_ch0_lowerhalf;
      mcpwm_ll_intr_clear_status(common->hal.dev,
                                 status &
                                 MCPWM_LL_EVENT_FAULT_EXIT(MCPWM_FAULT_0));
      esp_motor_fault_clear((struct motor_lowerhalf_s *)priv,
                            MOTOR_FAULT_OTHER);
      mcpwm_ll_brake_clear_ost(common->hal.dev, MCPWM_FAULT_0);
    }
#endif

  /* If capture is disabled or the interrupt was not related to it,
   * simply return. Otherwise, continue executing capture math
   */

#ifdef CONFIG_ESP_MCPWM_CAPTURE
  if (lower == NULL)
    {
      spin_unlock_irqrestore(&common->mcpwm_spinlock, flags);
      return OK;
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
#endif

  spin_unlock_irqrestore(&common->mcpwm_spinlock, flags);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_motor_bdc_initialize
 *
 * Description:
 *   This function initializes the MCPWM peripheral and configures the
 *   motor control driver.
 *
 * Input Parameters:
 *   channel    - Channel to be initialized [0-3].
 *   frequency  - PWM output frequency in Hz.
 *   pwm_a_pin  - GPIO pin for PWM_A output.
 *   pwm_b_pin  - GPIO pin for PWM_B output (complements PWM_A).
 *   fault_pin  - Indicates input pin to detect fault (to be implemented).
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the Capture device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR_BDC
struct motor_lowerhalf_s *esp_motor_bdc_initialize(int channel,
  uint32_t frequency, int pwm_a_pin, int pwm_b_pin, int fault_pin)
{
  struct mcpwm_motor_lowerhalf_s *lower = NULL;
  int ret;
  uint32_t ref_clock;

  if ((frequency > MCPWM_MAX_PWM_OUT_FREQ) ||
      (frequency < MCPWM_MIN_PWM_OUT_FREQ))
    {
      mtrerr("Invalid frequency set. Must be between %d and %d Hz\n",
             MCPWM_MIN_PWM_OUT_FREQ, MCPWM_MAX_PWM_OUT_FREQ);
      return NULL;
    }

  if (!g_mcpwm_common.initialized)
    {
      esp_mcpwm_group_start();
    }

#ifdef ESP_MCPMW_MOTOR_FAULT
  if (!g_mcpwm_common.isr_initialized)
    {
      esp_mcpwm_isr_register(mcpwm_driver_isr_default, &g_mcpwm_common);
      g_mcpwm_common.isr_initialized = true;
    }
#endif

  switch (channel)
    {
      case 0:
        lower = &mcpwm_bdc_ch0_lowerhalf;
        lower->pwm_frequency = frequency;
        break;
      default:
        mtrerr("Invalid channel selection: %d\n", channel);
        return NULL;
    }

  lower->generator_pins[MCPWM_GENERATOR_0] = pwm_a_pin;
  lower->generator_pins[MCPWM_GENERATOR_1] = pwm_b_pin;
  lower->fault_pin = fault_pin;

  /* Configure GPIOs before starting */

#ifdef ESP_MCPMW_MOTOR_FAULT
  ret = esp_mcpwm_fault_gpio_config(lower, true);
  if (ret < 0)
    {
      mtrerr("Failed configuring motor fault GPIOs\n");
      return NULL;
    }
#endif

  ret = esp_mcpwm_motor_set_gpio(lower, true);
  if (ret < 0)
    {
      mtrerr("Failed configuring motor PWM GPIOs\n");
      return NULL;
    }

  mtrinfo("Channel %d initialized. GPIO: PWM_A: %d | PWM_B: %d | Freq: %d\n",
          lower->channel_id, lower->generator_pins[MCPWM_GENERATOR_0],
          lower->generator_pins[MCPWM_GENERATOR_1], lower->pwm_frequency);
  return (struct motor_lowerhalf_s *) lower;
}
#endif

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

  if (!g_mcpwm_common.isr_initialized)
    {
      esp_mcpwm_isr_register(mcpwm_driver_isr_default,
                                     &g_mcpwm_common);
      g_mcpwm_common.isr_initialized = true;
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
