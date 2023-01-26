/****************************************************************************
 * arch/xtensa/src/esp32/esp32_touch.c
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

#include "esp32_gpio.h"
#include "esp32_irq.h"
#include "esp32_rt_timer.h"
#include "esp32_rtc.h"
#include "esp32_touch.h"
#include "esp32_touch_lowerhalf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOUCH_PAD_FILTER_FACTOR_DEFAULT (4) /* IIR filter coefficient */
#define TOUCH_PAD_SHIFT_DEFAULT         (4) /* Increase computing accuracy */
#define TOUCH_PAD_SHIFT_ROUND_DEFAULT   (8) /* ROUND = 2^(n-1) */
#define TOUCH_THRESHOLD_NO_USE          (0)
#define TOUCH_GET_RTCIO_NUM(channel)    (touch_channel_to_rtcio[channel])

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

#ifdef CONFIG_ESP32_TOUCH_FILTER
struct touch_filter_s
{
  struct rt_timer_args_s filter_timer_args;
  struct rt_timer_s *filter_timer_handler;
  uint16_t filtered_val[TOUCH_SENSOR_PINS];
  uint16_t raw_val[TOUCH_SENSOR_PINS];
  uint32_t period_ms;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32_TOUCH_IRQ
static int touch_interrupt(int irq, void *context, void *arg);
static void touch_restore_irq(void *arg);
#endif
#ifdef CONFIG_ESP32_TOUCH_FILTER
static uint32_t touch_filter_iir(uint32_t in_now,
                                 uint32_t out_last,
                                 uint32_t k);
static void touch_filter_cb(void *arg);
static void touch_filter_start(uint32_t filter_period_ms);
#endif
static uint16_t touch_read(enum touch_pad_e tp);
static void touch_clear_group_mask(uint16_t set1_mask,
                                   uint16_t set2_mask,
                                   uint16_t en_mask);
static void touch_config(enum touch_pad_e tp);
static void touch_init(void);
static void touch_set_group_mask(uint16_t set1_mask,
                                 uint16_t set2_mask,
                                 uint16_t en_mask);
static void touch_set_meas_mode(enum touch_pad_e tp,
                                struct touch_config_meas_mode_s *meas);
static void touch_set_voltage(struct touch_config_volt_s *volt);
static void touch_io_init(enum touch_pad_e tp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_TOUCH_FILTER
static struct touch_filter_s *touch_pad_filter = NULL;
#endif
#ifdef CONFIG_ESP32_TOUCH_IRQ
static uint16_t touch_pad_isr_enabled = 0x0000;
static int touch_last_irq = -1;
static int (*touch_release_cb)(int, void *, void *) = NULL;
static struct rt_timer_args_s irq_timer_args;
static struct rt_timer_s *irq_timer_handler = NULL;
#endif
static mutex_t *touch_mux = NULL;
static uint16_t touch_pad_init_bit = 0x0000;
static uint16_t touch_pad_logic_threshold[TOUCH_SENSOR_PINS];
static spinlock_t lock;

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

#ifdef CONFIG_ESP32_TOUCH_IRQ
static int touch_interrupt(int irq, void *context, void *arg)
{
  uint32_t status;
  int i;

  touch_lh_intr_disable();
  status = touch_lh_read_trigger_status_mask();
  touch_lh_clear_trigger_status_mask();

  rt_timer_start(irq_timer_handler,
                 CONFIG_ESP32_TOUCH_IRQ_INTERVAL_MS * USEC_PER_MSEC,
                 false);

  /* Read and clear the touch interrupt status */

  for (i = 0; i < ESP32_NIRQ_RTCIO_TOUCHPAD; i++)
    {
      if ((touch_pad_init_bit >> i) &
          (touch_pad_isr_enabled >> i) &
          (status >> i) & 0x1)
        {
          touch_last_irq = ESP32_FIRST_RTCIOIRQ_TOUCHPAD + i;
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

#ifdef CONFIG_ESP32_TOUCH_IRQ
static void touch_restore_irq(void *arg)
{
  if (touch_last_irq > 0 && touch_release_cb != NULL)
    {
      /* Call the button interrup handler again so we can detect touch pad
       * releases
       */

      touch_release_cb(touch_last_irq, NULL, NULL);
    }

  touch_lh_intr_enable();
}
#endif

/****************************************************************************
 * Name: touch_init
 *
 * Description:
 *   Initialize the touch pad driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_init(void)
{
  if (touch_mux == NULL)
    {
      touch_mux = (mutex_t *) kmm_zalloc(sizeof(mutex_t));

      if (touch_mux == NULL)
        {
          ierr("Failed to initialize touch pad driver.\n");
          PANIC();
        }

      nxmutex_init(touch_mux);

      irqstate_t flags = spin_lock_irqsave(&lock);

      touch_lh_stop_fsm();
      touch_lh_intr_disable();
      touch_lh_intr_clear();
      touch_lh_clear_channel_mask(TOUCH_BIT_MASK_ALL);
      touch_lh_clear_group_mask(TOUCH_BIT_MASK_ALL, TOUCH_BIT_MASK_ALL);
      touch_lh_set_trigger_mode(TOUCH_TRIGGER_MODE_DEFAULT);
      touch_lh_set_trigger_source(TOUCH_TRIGGER_SOURCE_DEFAULT);
      touch_lh_clear_trigger_status_mask();
      touch_lh_set_meas_time(TOUCH_MEASURE_CYCLE_DEFAULT);
      touch_lh_set_sleep_time(TOUCH_SLEEP_CYCLE_DEFAULT);
      touch_lh_set_fsm_mode(TOUCH_FSM_MODE_DEFAULT);
      touch_lh_start_fsm();

#ifdef CONFIG_ESP32_TOUCH_IRQ
      irq_timer_args.arg = NULL;
      irq_timer_args.callback = touch_restore_irq;
      rt_timer_create(&(irq_timer_args), &(irq_timer_handler));

      int ret = irq_attach(ESP32_IRQ_RTC_TOUCH, touch_interrupt, NULL);
      if (ret < 0)
        {
          ierr("ERROR: irq_attach() failed: %d\n", ret);
        }
#endif

      spin_unlock_irqrestore(&lock, flags);
    }
}

/****************************************************************************
 * Name: touch_filter_iir
 *
 * Description:
 *   Infinite Impulse Response filter function.
 *
 * Input Parameters:
 *   in_now - Raw value to be filtered;
 *   out_last - Last value outputed;
 *   k - The filter coefficient.
 *
 * Returned Value:
 *   Filtered value.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_TOUCH_FILTER
static uint32_t touch_filter_iir(uint32_t in_now,
                                 uint32_t out_last,
                                 uint32_t k)
{
  if (k == 0)
    {
      return in_now;
    }
  else
    {
      uint32_t out_now = (in_now + (k - 1) * out_last) / k;
      return out_now;
    }
}

/****************************************************************************
 * Name: touch_filter_cb
 *
 * Description:
 *   Filter timer callback.
 *   Measures channels, applies filter and restart timers.
 *
 * Input Parameters:
 *   arg - Pointer to a memory location containing the function arguments.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_filter_cb(void *arg)
{
  if (touch_pad_filter == NULL || touch_mux == NULL)
    {
      return;
    }

  uint16_t val = 0;
  static uint32_t filtered_temp[TOUCH_SENSOR_PINS];

  nxmutex_lock(touch_mux);

  for (int i = 0; i < TOUCH_SENSOR_PINS; i++)
    {
      if ((touch_pad_init_bit >> i) & 0x1)
        {
          val = touch_read(i);
          touch_pad_filter->raw_val[i] = val;
          filtered_temp[i] = filtered_temp[i] == 0 ? ((uint32_t)val <<
            TOUCH_PAD_SHIFT_DEFAULT) : filtered_temp[i];
          filtered_temp[i] =
            touch_filter_iir((val << TOUCH_PAD_SHIFT_DEFAULT),
                             filtered_temp[i],
                             TOUCH_PAD_FILTER_FACTOR_DEFAULT);
          touch_pad_filter->filtered_val[i] = (filtered_temp[i] +
            TOUCH_PAD_SHIFT_ROUND_DEFAULT) >> TOUCH_PAD_SHIFT_DEFAULT;
        }
    }

  rt_timer_start(touch_pad_filter->filter_timer_handler,
                 touch_pad_filter->period_ms * USEC_PER_MSEC,
                 false);
  nxmutex_unlock(touch_mux);
}

/****************************************************************************
 * Name: touch_filter_start
 *
 * Description:
 *   Start the touch pad filter.
 *
 * Input Parameters:
 *   filter_period_ms - The filter measurement interval in milliseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_filter_start(uint32_t filter_period_ms)
{
  if (touch_mux == NULL)
    {
      ierr("ERROR: Touch pads not initialized.\n");
      return;
    }

  nxmutex_lock(touch_mux);

  if (touch_pad_filter == NULL)
    {
      touch_pad_filter = (struct touch_filter_s *)
        kmm_zalloc(sizeof(struct touch_filter_s));

      if (touch_pad_filter == NULL)
        {
          ierr("ERROR: Failed to initialize touch filter.\n");
          nxmutex_unlock(touch_mux);
          return;
        }

      touch_pad_filter->filter_timer_args.arg = NULL;
      touch_pad_filter->filter_timer_args.callback = touch_filter_cb;
      rt_timer_create(&(touch_pad_filter->filter_timer_args),
                      &(touch_pad_filter->filter_timer_handler));

      touch_pad_filter->period_ms = filter_period_ms;
    }

  nxmutex_unlock(touch_mux);
  touch_filter_cb(NULL);
}
#endif

/****************************************************************************
 * Name: touch_set_group_mask
 *
 * Description:
 *   Activate channels in touch pad groups.
 *
 * Input Parameters:
 *   set1_mask - The SET1 group bitmask;
 *   set2_mask - The SET2 group bitmask;
 *   en_mask - The working group bitmask.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_set_group_mask(uint16_t set1_mask,
                                 uint16_t set2_mask,
                                 uint16_t en_mask)
{
  DEBUGASSERT(set1_mask <= TOUCH_BIT_MASK_ALL &&
              set2_mask <= TOUCH_BIT_MASK_ALL &&
              en_mask <= TOUCH_BIT_MASK_ALL);

  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_set_group_mask(set1_mask, set2_mask);
  touch_lh_set_channel_mask(en_mask);

  spin_unlock_irqrestore(&lock, flags);
}

/****************************************************************************
 * Name: touch_clear_group_mask
 *
 * Description:
 *   Deactivate channels in touch pad groups.
 *
 * Input Parameters:
 *   set1_mask - The SET1 group bitmask;
 *   set2_mask - The SET2 group bitmask;
 *   en_mask - The working group bitmask.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void touch_clear_group_mask(uint16_t set1_mask,
                                   uint16_t set2_mask,
                                   uint16_t en_mask)
{
  DEBUGASSERT(set1_mask <= TOUCH_BIT_MASK_ALL &&
              set2_mask <= TOUCH_BIT_MASK_ALL &&
              en_mask <= TOUCH_BIT_MASK_ALL);

  irqstate_t flags = spin_lock_irqsave(&lock);

  touch_lh_clear_channel_mask(en_mask);
  touch_lh_clear_group_mask(set1_mask, set2_mask);

  spin_unlock_irqrestore(&lock, flags);
}

/****************************************************************************
 * Name: touch_config
 *
 * Description:
 *   Configure a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   threshold - The interrupt threshold value.
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
  touch_lh_set_threshold(tp, TOUCH_THRESHOLD_NO_USE);

  spin_unlock_irqrestore(&lock, flags);

  enum touch_fsm_mode_e mode = touch_lh_get_fsm_mode();

  if (mode == TOUCH_FSM_MODE_SW)
    {
      touch_clear_group_mask((1 << tp), (1 << tp), (1 << tp));
      touch_pad_init_bit |= (1 << tp);
    }
  else
    {
      uint32_t wait_time_ms = 0;
      uint16_t sleep_time = touch_lh_get_sleep_time();
      uint16_t meas_cycle = touch_lh_get_meas_time();
      uint32_t rtc_slow_clk_freq = esp32_rtc_clk_slow_freq_get_hz();
      uint32_t rtc_fast_clk_freq = esp32_rtc_clk_fast_freq_get_hz();
      touch_set_group_mask((1 << tp), (1 << tp), (1 << tp));

      /* If the FSM mode is 'TOUCH_FSM_MODE_TIMER', The data will be ready
       * after one measurement cycle, after this function is executed.
       * Otherwise, the "touch_value" by "touch_read" is 0.
       */

      wait_time_ms = sleep_time / (rtc_slow_clk_freq / 1000) + meas_cycle /
        (rtc_fast_clk_freq / 1000);
      up_udelay(wait_time_ms ? wait_time_ms * USEC_PER_MSEC : 1);
      touch_pad_init_bit |= (1 << tp);
    }
}

/****************************************************************************
 * Name: touch_read
 *
 * Description:
 *   Read a touch pad.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The read value.
 *
 ****************************************************************************/

static uint16_t touch_read(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  enum touch_fsm_mode_e mode = touch_lh_get_fsm_mode();
  uint16_t val = 0xffff;

  if (mode == TOUCH_FSM_MODE_SW)
    {
      touch_set_group_mask((1 << tp), (1 << tp), (1 << tp));

      irqstate_t flags = spin_lock_irqsave(&lock);
      touch_lh_start_sw_meas();
      spin_unlock_irqrestore(&lock, flags);

      while (!touch_lh_meas_is_done());
      val = touch_lh_read_raw_data(tp);
      touch_clear_group_mask((1 << tp), (1 << tp), (1 << tp));
    }
  else
    {
      while (!touch_lh_meas_is_done());
      val = touch_lh_read_raw_data(tp);
    }

  return val;
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

  uint8_t rtcio_num = TOUCH_GET_RTCIO_NUM(tp);
  esp32_configrtcio(rtcio_num, RTC_FUNCTION_RTCIO);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_configtouch
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

int esp32_configtouch(enum touch_pad_e tp, struct touch_config_s config)
{
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

  touch_init();

  /* Make sure to set the mode before enabling the touch pad as it influences
   * the configuration
   */

  touch_set_meas_mode(tp, &meas_config);
  touch_lh_set_fsm_mode(config.fsm_mode);
  touch_set_voltage(&volt_config);
  touch_config(tp);

#ifdef CONFIG_ESP32_TOUCH_FILTER
  touch_filter_start(config.filter_period);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32_touchread
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

bool esp32_touchread(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  uint16_t value = esp32_touchreadraw(tp);

  return (value > touch_pad_logic_threshold[tp]);
}

/****************************************************************************
 * Name: esp32_touchreadraw
 *
 * Description:
 *   Read the analog value of a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel.
 *
 * Returned Value:
 *   The number of charge and discharge cycles done in the last measurement.
 *
 ****************************************************************************/

uint16_t esp32_touchreadraw(enum touch_pad_e tp)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

#ifdef CONFIG_ESP32_TOUCH_FILTER
  uint16_t value = touch_pad_filter->filtered_val[tp];
#else
  uint16_t value = touch_read(tp);
#endif

  iinfo("Touch pad %d value: %u\n", tp, value);

  return value;
}

/****************************************************************************
 * Name: esp32_touchsetthreshold
 *
 * Description:
 *   Configure the threshold for a touch pad channel.
 *
 * Input Parameters:
 *   tp - The touch pad channel;
 *   threshold - The threshold value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32_touchsetthreshold(enum touch_pad_e tp, uint16_t threshold)
{
  DEBUGASSERT(tp < TOUCH_SENSOR_PINS);

  iinfo("Setting touch pad %d threshold to %u.\n", tp, threshold);

  touch_lh_set_threshold(tp, threshold);
  touch_pad_logic_threshold[tp] = threshold;
}

/****************************************************************************
 * Name: esp32_touchirqenable
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

#ifdef CONFIG_ESP32_TOUCH_IRQ
void esp32_touchirqenable(int irq)
{
  DEBUGASSERT(irq >= ESP32_FIRST_RTCIOIRQ_TOUCHPAD &&
              irq <= ESP32_LAST_RTCIOIRQ_TOUCHPAD);

  int bit = ESP32_IRQ2TOUCHPAD(irq);

  touch_lh_intr_disable();

  touch_pad_isr_enabled |= (UINT32_C(1) << bit);

  touch_lh_intr_enable();
}
#endif

/****************************************************************************
 * Name: esp32_touchirqdisable
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

#ifdef CONFIG_ESP32_TOUCH_IRQ
void esp32_touchirqdisable(int irq)
{
  DEBUGASSERT(irq >= ESP32_FIRST_RTCIOIRQ_TOUCHPAD &&
              irq <= ESP32_LAST_RTCIOIRQ_TOUCHPAD);

  int bit = ESP32_IRQ2TOUCHPAD(irq);

  touch_lh_intr_disable();

  touch_pad_isr_enabled &= (~(UINT32_C(1) << bit));

  touch_lh_intr_enable();
}
#endif

/****************************************************************************
 * Name: esp32_touchregisterreleasecb
 *
 * Description:
 *   Register the release callback.
 *
 ****************************************************************************/

void esp32_touchregisterreleasecb(int (*func)(int, void *, void *))
{
  DEBUGASSERT(func != NULL);

  touch_release_cb = func;
}
