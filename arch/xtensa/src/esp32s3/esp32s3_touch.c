/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_touch.c
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

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <arch/irq.h>

#include "xtensa.h"

#include "esp32s3_gpio.h"
#include "esp32s3_irq.h"
#include "esp32s3_touch.h"
#include "esp32s3_touch_lowerhalf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOUCH_GET_IO_NUM(channel) (touch_channel_to_rtcio[channel])

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct touch_config_volt_s
{
  enum touch_high_volt_e refh;
  enum touch_low_volt_e refl;
  enum touch_volt_atten_e atten;
};

struct touch_config_meas_mode_s
{
  enum touch_cnt_slope_e slope;
  enum touch_tie_opt_e tie_opt;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
static int touch_interrupt(int irq, void *context, void *arg);
static void touch_restore_irq(void *arg);
#endif
static void touch_config(enum touch_pad_e tp);
static void touch_init(struct touch_config_s *config);
static void touch_set_meas_mode(enum touch_pad_e tp,
                                struct touch_config_meas_mode_s *meas);
static void touch_set_voltage(struct touch_config_volt_s *volt);
static void touch_io_init(enum touch_pad_e tp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t *touch_mux = NULL;
static spinlock_t lock;
static uint32_t touch_pad_logic_threshold[TOUCH_SENSOR_PINS];

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
static uint16_t touch_pad_isr_enabled = 0x0000;
static enum touch_intr_mask_e touch_pad_isr_types = 0x0000;
static int touch_last_irq = -1;
static int (*touch_release_cb)(int, void *, void *) = NULL;
static struct rt_timer_args_s irq_timer_args;
static struct rt_timer_s *irq_timer_handler = NULL;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: touch_interrupt
 *
 * Description:
 *   Touch pads interrupt handler.
 *
 * Input Parameters:
 *   irq - Interrupt request number;
 *   context - Context data from the ISR;
 *   arg - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.

 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
static int touch_interrupt(int irq, void *context, void *arg)
{
  uint32_t intr_mask = *((uint32_t *) context);
  enum touch_pad_e pad_num = touch_lh_get_current_meas_channel();
  touch_last_irq = ESP32S3_TOUCHPAD2IRQ(pad_num);

  touch_lh_intr_disable(touch_pad_isr_types);

  rt_timer_start(irq_timer_handler,
                 CONFIG_ESP32S3_TOUCH_IRQ_INTERVAL_MS * USEC_PER_MSEC,
                 false);

  /* Read and clear the touch interrupt status */

  if (intr_mask & TOUCH_INTR_MASK_TIMEOUT)
    {
      touch_lh_timer_force_done();
    }

  if (intr_mask & (TOUCH_INTR_MASK_ACTIVE |
                   TOUCH_INTR_MASK_INACTIVE))
    {
      if ((touch_pad_isr_enabled >> pad_num) & 0x1)
        {
          irq_dispatch(touch_last_irq, context);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: touch_restore_irq
 *
 * Description:
 *   IRQ timer callback.
 *   Re-enables touch IRQ after a certain time to avoid spam.
 *
 * Input Parameters:
 *   arg - Pointer to a memory location containing the function arguments.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
static void touch_restore_irq(void *arg)
{
  if (touch_last_irq > 0 && touch_release_cb != NULL)
    {
      /* Call the button interrup handler again so we can detect touch pad
       * releases
       */

      touch_release_cb(touch_last_irq, NULL, NULL);
    }

  touch_lh_intr_enable(touch_pad_isr_types);
}
#endif

/****************************************************************************
 * Name: touch_init
 *
 * Description:
 *   Initialize the touch pad driver.
 *
 * Input Parameters:
 *   config - The touch configuration structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_init(struct touch_config_s *config)
{
  if (touch_mux != NULL)
    {
      iinfo("Touch pad driver already initialized.\n");
      return;
    }

  touch_mux = (mutex_t *) kmm_zalloc(sizeof(mutex_t));

  if (touch_mux == NULL)
    {
      ierr("Failed to initialize touch pad driver.\n");
      PANIC();
    }

  nxmutex_init(touch_mux);

  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_stop_fsm();
  touch_lh_intr_disable(TOUCH_INTR_MASK_ALL);
  touch_lh_intr_clear(TOUCH_INTR_MASK_ALL);
  touch_lh_clear_channel_mask(TOUCH_BIT_MASK_ALL);
  touch_lh_clear_trigger_status_mask();
  touch_lh_set_meas_time(TOUCH_MEASURE_CYCLE_DEFAULT);
  touch_lh_set_sleep_time(TOUCH_SLEEP_CYCLE_DEFAULT);

  /* Configure the touch-sensor power domain into self-bias since
   * bandgap-bias level is different under sleep-mode compared to
   * running-mode. Self-bias is always on after chip startup.
   */

  touch_lh_sleep_low_power(true);
  touch_lh_set_voltage_high(TOUCH_HIGH_VOLTAGE_THRESHOLD);
  touch_lh_set_voltage_low(TOUCH_LOW_VOLTAGE_THRESHOLD);
  touch_lh_set_voltage_attenuation(TOUCH_ATTEN_VOLTAGE_THRESHOLD);
  touch_lh_set_idle_channel_connect(TOUCH_IDLE_CH_CONNECT_DEFAULT);

  /* Clear touch channels to initialize the channel value (benchmark,
   * raw_data).
   * Note: Should call it after enable clock gate.
   */

  touch_lh_clkgate(true);
  touch_lh_reset_benchmark(TOUCH_PAD_ALL);
  touch_lh_sleep_reset_benchmark();

#ifdef CONFIG_ESP32S3_TOUCH_FILTER
  touch_lh_filter_set_filter_mode(config->filter_mode);
  touch_lh_filter_set_debounce(config->filter_debounce_cnt);
  touch_lh_filter_set_noise_thres(config->filter_noise_thr);
  touch_lh_filter_set_jitter_step(config->filter_jitter_step);
  touch_lh_filter_set_smooth_mode(config->filter_smh_lvl);
  touch_lh_filter_enable();
#endif

#ifdef CONFIG_ESP32S3_TOUCH_DENOISE
  touch_lh_set_slope(TOUCH_DENOISE_CHANNEL, TOUCH_SLOPE_DEFAULT);
  touch_lh_set_tie_option(TOUCH_DENOISE_CHANNEL, TOUCH_TIE_OPT_DEFAULT);
  touch_lh_denoise_set_cap_level(config->denoise_cap_level);
  touch_lh_denoise_set_grade(config->denoise_grade);
  touch_lh_denoise_enable();
#endif

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
  irq_timer_args.arg = NULL;
  irq_timer_args.callback = touch_restore_irq;
  rt_timer_create(&(irq_timer_args), &(irq_timer_handler));

  touch_pad_isr_types = TOUCH_INTR_MASK_ACTIVE |
                        TOUCH_INTR_MASK_INACTIVE |
                        TOUCH_INTR_MASK_TIMEOUT;

  int ret = 0;

  ret |= irq_attach(ESP32S3_IRQ_RTC_TOUCH_DONE,
                    touch_interrupt,
                    NULL);

  ret |= irq_attach(ESP32S3_IRQ_RTC_TOUCH_ACTIVE,
                    touch_interrupt,
                    NULL);

  ret |= irq_attach(ESP32S3_IRQ_RTC_TOUCH_INACTIVE,
                    touch_interrupt,
                    NULL);

  ret |= irq_attach(ESP32S3_IRQ_RTC_TOUCH_SCAN_DONE,
                    touch_interrupt,
                    NULL);

  ret |= irq_attach(ESP32S3_IRQ_RTC_TOUCH_TIMEOUT,
                    touch_interrupt,
                    NULL);

  ret |= irq_attach(ESP32S3_IRQ_RTC_TOUCH_APPROACH_LOOP_DONE,
                    touch_interrupt,
                    NULL);

  if (ret < 0)
    {
      ierr("ERROR: irq_attach() failed.\n");
    }
#endif

  spin_unlock_irqrestore(&lock, flags);
}

/****************************************************************************
 * Name: touch_config
 *
 * Description:
 *   Configure a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_config(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  if (touch_mux == NULL)
    {
      ierr("ERROR: Touch pads not initialized.\n");
      return;
    }

  touch_io_init(tp);

  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_set_slope(tp, TOUCH_SLOPE_DEFAULT);
  touch_lh_set_tie_option(tp, TOUCH_TIE_OPT_DEFAULT);
  touch_lh_set_channel_mask(BIT(tp));

  spin_unlock_irqrestore(&lock, flags);
}

/****************************************************************************
 * Name: touch_set_voltage
 *
 * Description:
 *   Set the touch pads voltage configuration.
 *
 * Input Parameters:
 *   volt - The new configuration struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_set_voltage(struct touch_config_volt_s *volt)
{
  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_set_voltage_high(volt->refh);
  touch_lh_set_voltage_low(volt->refl);
  touch_lh_set_voltage_attenuation(volt->atten);

  spin_unlock_irqrestore(&lock, flags);
}

/****************************************************************************
 * Name: touch_set_meas_mode
 *
 * Description:
 *   Set the measurement mode for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   meas - The new configuration struct.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_set_meas_mode(enum touch_pad_e tp,
                                struct touch_config_meas_mode_s *meas)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_set_slope(tp, meas->slope);
  touch_lh_set_tie_option(tp, meas->tie_opt);

  spin_unlock_irqrestore(&lock, flags);
}

/****************************************************************************
 * Name: touch_io_init
 *
 * Description:
 *   Initialize GPIOs for a given touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_io_init(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  esp32s3_configrtcio(tp, RTC_FUNCTION_RTCIO);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_configtouch
 *
 * Description:
 *   Configure a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   config - The touch pad configuration structure.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

int esp32s3_configtouch(enum touch_pad_e tp, struct touch_config_s config)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  struct touch_config_volt_s volt_config =
    {
      .refh = config.refh,
      .refl = config.refl,
      .atten = config.atten
    };

  struct touch_config_meas_mode_s meas_config =
    {
      .slope = config.slope,
      .tie_opt = config.tie_opt
    };

  touch_init(&config);

  touch_set_meas_mode(tp, &meas_config);
  touch_lh_set_fsm_mode(config.fsm_mode);
  touch_set_voltage(&volt_config);
  touch_config(tp);
  touch_lh_start_fsm();

  return OK;
}

/****************************************************************************
 * Name: esp32s3_touchread
 *
 * Description:
 *   Read a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   0 if touch pad pressed, 1 if released.
 *
 ****************************************************************************/

bool esp32s3_touchread(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  uint32_t value = esp32s3_touchreadraw(tp);

  return (value > touch_pad_logic_threshold[tp]);
}

/****************************************************************************
 * Name: esp32s3_touchreadraw
 *
 * Description:
 *   Read the analog value of a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The number of charge cycles in the last measurement.
 *
 ****************************************************************************/

uint32_t esp32s3_touchreadraw(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  irqstate_t flags = spin_lock_irqsave(&lock);

#ifdef CONFIG_ESP32S3_TOUCH_FILTER
  uint32_t value = touch_lh_filter_read_smooth(tp);
#else
  uint32_t value = touch_lh_read_raw_data(tp);
#endif

  spin_unlock_irqrestore(&lock, flags);

  iinfo("Touch pad %d value: %u\n", tp, value);

  return value;
}

/****************************************************************************
 * Name: esp32s3_touchbenchmark
 *
 * Description:
 *   Read the touch pad channel benchmark.
 *   After initialization, the benchmark value is the maximum during the
 *   first measurement period.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The benchmark value.
 *
 ****************************************************************************/

uint32_t esp32s3_touchbenchmark(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  irqstate_t flags = spin_lock_irqsave(&lock);

  uint32_t value = touch_lh_read_benchmark(tp);

  spin_unlock_irqrestore(&lock, flags);

  iinfo("Touch pad %d benchmark value: %u\n", tp, value);

  return value;
}

/****************************************************************************
 * Name: esp32s3_touchsetthreshold
 *
 * Description:
 *   Set the touch pad channel threshold.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   threshold - The threshold value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_touchsetthreshold(enum touch_pad_e tp, uint32_t threshold)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_set_threshold(tp, threshold);
  touch_pad_logic_threshold[tp] = threshold;

  spin_unlock_irqrestore(&lock, flags);

  iinfo("Touch pad %d threshold set to: %u\n", tp, threshold);
}

/****************************************************************************
 * Name: esp32s3_touchirqenable
 *
 * Description:
 *   Enable the interrupt for the specified touch pad.
 *
 * Input Parameters:
 *   irq - The touch pad irq number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
void esp32s3_touchirqenable(int irq)
{
  DEBUGASSERT(irq >= ESP32S3_FIRST_RTCIOIRQ_TOUCHPAD &&
              irq <= ESP32S3_LAST_RTCIOIRQ_TOUCHPAD);

  int bit = ESP32S3_IRQ2TOUCHPAD(irq);

  touch_lh_intr_disable(touch_pad_isr_types);

  touch_pad_isr_enabled |= (UINT32_C(1) << bit);

  touch_lh_intr_enable(touch_pad_isr_types);
}
#endif

/****************************************************************************
 * Name: esp32s3_touchirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified touch pad.
 *
 * Input Parameters:
 *   irq - The touch pad irq number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_TOUCH_IRQ
void esp32s3_touchirqdisable(int irq)
{
  DEBUGASSERT(irq >= ESP32S3_FIRST_RTCIOIRQ_TOUCHPAD &&
              irq <= ESP32S3_LAST_RTCIOIRQ_TOUCHPAD);

  int bit = ESP32S3_IRQ2TOUCHPAD(irq);

  touch_lh_intr_disable(touch_pad_isr_types);

  touch_pad_isr_enabled &= (~(UINT32_C(1) << bit));

  touch_lh_intr_enable(touch_pad_isr_types);
}
#endif

/****************************************************************************
 * Name: esp32s3_touchregisterreleasecb
 *
 * Description:
 *   Register the release callback.
 *
 * Input Parameters:
 *   func - The handler function to be used.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_touchregisterreleasecb(int (*func)(int, void *, void *))
{
  DEBUGASSERT(func != NULL);

  touch_release_cb = func;
}
