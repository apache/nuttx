/****************************************************************************
 * arch/risc-v/src/espressif/esp_hr_timer.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>

#include "esp_irq.h"
#include "esp_hr_timer.h"

#include "esp_attr.h"
#include "hal/systimer_hal.h"
#include "hal/systimer_ll.h"
#include "periph_ctrl.h"
#include "systimer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if SOC_SYSTIMER_INT_LEVEL
#  define SYSTIMER_TRIGGER_TYPE ESP_IRQ_TRIGGER_LEVEL
#else
#  define SYSTIMER_TRIGGER_TYPE ESP_IRQ_TRIGGER_EDGE
#endif /* SOC_SYSTIMER_INT_LEVEL */

#ifdef CONFIG_SCHED_HPWORK
static_assert(CONFIG_ESPRESSIF_HR_TIMER_TASK_PRIORITY <
                CONFIG_SCHED_HPWORKPRIORITY,
              "High Resolution Timer task priority should be smaller than"
                " High priority worker thread");
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_hr_timer_context_s
{
  pid_t pid;
  sem_t toutsem;
  struct list_node runlist;
  struct list_node toutlist;
  systimer_hal_context_t hal;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_hr_timer_thread(int argc, char *argv[]);
static int esp_hr_timer_isr(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_hr_timer_context_s g_hr_timer_context =
{
  .pid = INVALID_PROCESS_ID,
  .toutsem = SEM_INITIALIZER(0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_hr_timer_thread
 *
 * Description:
 *   HR Timer working thread: Waits for a timeout semaphore, scans
 *   the timeout list and processes all the timers in the list.
 *
 * Input Parameters:
 *   argc          - Not used.
 *   argv          - Not used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int esp_hr_timer_thread(int argc, char *argv[])
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;

  while (true)
    {
      /* Waiting for all timers to time out */

      VERIFY(nxsem_wait_uninterruptible(&priv->toutsem));

      irqstate_t flags = enter_critical_section();

      /* Process all the timers in list */

      while (!list_is_empty(&priv->toutlist))
        {
          struct esp_hr_timer_s *timer;
          enum esp_hr_timer_state_e raw_state;

          /* Get the first timer in the list */

          timer = container_of(priv->toutlist.next,
                               struct esp_hr_timer_s, list);

          /* Cache the raw state to decide how to deal with this timer */

          raw_state = timer->state;

          /* Delete the timer from the list */

          list_delete(&timer->list);

          /* Set timer's state to idle so it can be restarted by the user. */

          timer->state = HR_TIMER_IDLE;

          leave_critical_section(flags);

          if (raw_state == HR_TIMER_TIMEOUT)
            {
              timer->callback(timer->arg);
            }
          else if (raw_state == HR_TIMER_DELETE)
            {
              kmm_free(timer);
            }

          /* Enter critical section for next scanning list */

          flags = enter_critical_section();

          if (raw_state == HR_TIMER_TIMEOUT)
            {
              /* Check if the timer is in "repeat" mode */

              if (timer->flags & HR_TIMER_REPEAT)
                {
                  esp_hr_timer_start(timer, timer->timeout, true);
                }
            }
        }

      leave_critical_section(flags);
    }

  return 0;
}

/****************************************************************************
 * Name: esp_hr_timer_isr
 *
 * Description:
 *   Handler to be executed by the Systimer ISR.
 *
 * Input Parameters:
 *   irq           - IRQ associated to that interrupt.
 *   context       - Interrupt register state save info.
 *   arg           - A pointer to the argument provided when the interrupt
 *                   was registered.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int IRAM_ATTR esp_hr_timer_isr(int irq, void *context, void *arg)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;
  bool wake = false;
  irqstate_t flags;

  /* Clear interrupt register status */

  systimer_ll_clear_alarm_int(priv->hal.dev, SYSTIMER_ALARM_ESPTIMER);

  flags = enter_critical_section();

  /* Check if there is a timer running */

  if (!list_is_empty(&priv->runlist))
    {
      struct esp_hr_timer_s *timer;
      uint64_t counter;

      /* When stop/delete timer, in the same time the hardware timer
       * interrupt triggers, function "stop/delete" remove the timer
       * from running list, so the 1st timer is not which triggers.
       */

      timer = container_of(priv->runlist.next, struct esp_hr_timer_s, list);
      counter = systimer_hal_get_time(&priv->hal, SYSTIMER_COUNTER_ESPTIMER);
      if (timer->alarm <= counter)
        {
          /* Remove the first timer from the running list and add it to
           * the timeout list.
           *
           * Set the timer's state to be HR_TIMER_TIMEOUT to avoid
           * other operations.
           */

          list_delete(&timer->list);
          timer->state = HR_TIMER_TIMEOUT;
          list_add_after(&priv->toutlist, &timer->list);
          wake = true;

          /* Check if there is a timer running */

          if (!list_is_empty(&priv->runlist))
            {
              /* Reset hardware timer alarm with next timer's alarm value */

              timer = container_of(priv->runlist.next,
                                   struct esp_hr_timer_s, list);

              systimer_counter_value_t systimer_alarm =
                {
                  .val = priv->hal.us_to_ticks(timer->alarm),
                };

              systimer_ll_enable_alarm(priv->hal.dev,
                                       SYSTIMER_ALARM_ESPTIMER, false);
              systimer_ll_set_alarm_target(priv->hal.dev,
                                           SYSTIMER_ALARM_ESPTIMER,
                                           systimer_alarm.val);
              systimer_ll_apply_alarm_value(priv->hal.dev,
                                            SYSTIMER_ALARM_ESPTIMER);
            }
        }

      /* If there is a timer in the list, the alarm should be enabled */

      systimer_ll_enable_alarm(priv->hal.dev, SYSTIMER_ALARM_ESPTIMER, true);
    }

  if (wake)
    {
      /* Wake up the thread to process timed-out timers */

      int ret = nxsem_post(&priv->toutsem);
      if (ret < 0)
        {
          tmrerr("Failed to post sem ret=%d\n", ret);
        }
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_hr_timer_create
 *
 * Description:
 *   Create a High Resolution Timer from the provided arguments.
 *
 * Input Parameters:
 *   args          - HR Timer creation arguments.
 *
 * Output Parameters:
 *   timer_handle  - HR Timer handle pointer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_hr_timer_create(const struct esp_hr_timer_args_s *args,
                        struct esp_hr_timer_s **timer_handle)
{
  struct esp_hr_timer_s *timer;

  timer = (struct esp_hr_timer_s *)kmm_malloc(sizeof(*timer));
  if (timer == NULL)
    {
      tmrerr("Failed to allocate %d bytes\n", sizeof(*timer));

      return -ENOMEM;
    }

  timer->callback = args->callback;
  timer->arg      = args->arg;
  timer->flags    = HR_TIMER_NOFLAGS;
  timer->state    = HR_TIMER_IDLE;
  list_initialize(&timer->list);

  *timer_handle = timer;

  return OK;
}

/****************************************************************************
 * Name: esp_hr_timer_start
 *
 * Description:
 *   Start the High Resolution Timer.
 *
 * Input Parameters:
 *   timer         - HR Timer pointer.
 *   timeout       - Timeout value.
 *   repeat        - Repeat mode (true: enabled, false: disabled).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_start(struct esp_hr_timer_s *timer,
                        uint64_t timeout,
                        bool repeat)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;
  bool inserted = false;
  uint64_t counter;
  struct esp_hr_timer_s *p;
  irqstate_t flags = enter_critical_section();

  /* Only idle timer can be started */

  if (timer->state != HR_TIMER_IDLE)
    {
      esp_hr_timer_stop(timer);
    }

  /* Calculate the timer's alarm value */

  counter = systimer_hal_get_time(&priv->hal, SYSTIMER_COUNTER_ESPTIMER);
  timer->timeout = timeout;
  timer->alarm = timer->timeout + counter;

  if (repeat)
    {
      timer->flags |= HR_TIMER_REPEAT;
    }
  else
    {
      timer->flags &= ~HR_TIMER_REPEAT;
    }

  /* Scan the timer list and insert the new timer into previous
   * node of timer whose alarm value is larger than new one
   */

  list_for_every_entry(&priv->runlist, p, struct esp_hr_timer_s, list)
    {
      if (p->alarm > timer->alarm)
        {
          list_add_before(&p->list, &timer->list);
          inserted = true;
          break;
        }
    }

  /* If we didn't find a larger one, insert the new timer at the tail
   * of the list.
   */

  if (!inserted)
    {
      list_add_tail(&priv->runlist, &timer->list);
    }

  timer->state = HR_TIMER_READY;

  /* If this timer is at the head of the list */

  if (timer == container_of(priv->runlist.next,
                            struct esp_hr_timer_s, list))
    {
      /* Reset the hardware timer alarm */

      systimer_hal_set_alarm_target(&priv->hal, SYSTIMER_ALARM_ESPTIMER,
                                    timer->alarm);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_hr_timer_stop
 *
 * Description:
 *   Stop the High Resolution Timer.
 *
 * Input Parameters:
 *   timer         - HR Timer pointer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_stop(struct esp_hr_timer_s *timer)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;

  irqstate_t flags = enter_critical_section();

  /* "start" function can set the timer's repeat flag, and "stop" function
   * should remove this flag.
   */

  timer->flags &= ~HR_TIMER_REPEAT;

  /* Only timers in "ready" state can be stopped */

  if (timer->state == HR_TIMER_READY)
    {
      bool ishead;

      /* Check if the timer is at the head of the list */

      if (timer == container_of(priv->runlist.next,
                                struct esp_hr_timer_s, list))
        {
          ishead = true;
        }
      else
        {
          ishead = false;
        }

      list_delete(&timer->list);
      timer->state = HR_TIMER_IDLE;

      /* If the timer is at the head of the list */

      if (ishead)
        {
          if (!list_is_empty(&priv->runlist))
            {
              struct esp_hr_timer_s *next_timer;

              /* Set the value from the next timer as the new hardware timer
               * alarm value.
               */

              next_timer = container_of(priv->runlist.next,
                                        struct esp_hr_timer_s,
                                        list);

              systimer_hal_set_alarm_target(&priv->hal,
                                            SYSTIMER_ALARM_ESPTIMER,
                                            next_timer->alarm);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_hr_timer_delete
 *
 * Description:
 *   Stop and delete the High Resolution Timer.
 *
 * Input Parameters:
 *   timer         - HR Timer pointer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_delete(struct esp_hr_timer_s *timer)
{
  int ret;
  irqstate_t flags;

  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;

  flags = enter_critical_section();

  if (timer->state == HR_TIMER_READY)
    {
      esp_hr_timer_stop(timer);
    }
  else if (timer->state == HR_TIMER_TIMEOUT)
    {
      list_delete(&timer->list);
    }
  else if (timer->state == HR_TIMER_DELETE)
    {
      goto exit;
    }

  list_add_after(&priv->toutlist, &timer->list);
  timer->state = HR_TIMER_DELETE;

  /* Wake up the thread to process deleted timers */

  ret = nxsem_post(&priv->toutsem);
  if (ret < 0)
    {
      tmrerr("Failed to post sem ret=%d\n", ret);
    }

exit:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_hr_timer_time_us
 *
 * Description:
 *   Get time of the High Resolution Timer in microseconds.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Time of the HR Timer in microseconds.
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp_hr_timer_time_us(void)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;

  return systimer_hal_get_time(&priv->hal, SYSTIMER_COUNTER_ESPTIMER);
}

/****************************************************************************
 * Name: esp_hr_timer_get_alarm
 *
 * Description:
 *   Get the timestamp when the next timeout is expected to occur.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Timestamp of the nearest timer event in microseconds.
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp_hr_timer_get_alarm(void)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;
  uint64_t counter;
  uint64_t alarm_value;

  irqstate_t flags = enter_critical_section();

  counter = systimer_hal_get_time(&priv->hal, SYSTIMER_COUNTER_ESPTIMER);
  alarm_value = systimer_hal_get_alarm_value(&priv->hal,
                                             SYSTIMER_ALARM_ESPTIMER);
  alarm_value = priv->hal.ticks_to_us(alarm_value);

  if (alarm_value <= counter)
    {
      alarm_value = 0;
    }
  else
    {
      alarm_value -= counter;
    }

  leave_critical_section(flags);

  return alarm_value;
}

/****************************************************************************
 * Name: esp_hr_timer_calibration
 *
 * Description:
 *   Adjust current High Resolution Timer by a certain value.
 *
 * Input Parameters:
 *   time_us       - Adjustment to apply to the HR Timer in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp_hr_timer_calibration(uint64_t time_us)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;

  irqstate_t flags = enter_critical_section();
  systimer_hal_counter_value_advance(&priv->hal, SYSTIMER_COUNTER_ESPTIMER,
                                     time_us);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_hr_timer_init
 *
 * Description:
 *   Initialize High Resolution Timer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_hr_timer_init(void)
{
  struct esp_hr_timer_context_s *priv = &g_hr_timer_context;

  int pid = kthread_create(CONFIG_ESPRESSIF_HR_TIMER_TASK_NAME,
                           CONFIG_ESPRESSIF_HR_TIMER_TASK_PRIORITY,
                           CONFIG_ESPRESSIF_HR_TIMER_TASK_STACK_SIZE,
                           esp_hr_timer_thread,
                           NULL);
  if (pid < 0)
    {
      tmrerr("Failed to create HR Timer task=%d\n", pid);

      return pid;
    }

  list_initialize(&priv->runlist);
  list_initialize(&priv->toutlist);

  priv->pid = (pid_t)pid;

    {
      irqstate_t flags = enter_critical_section();

      periph_module_enable(PERIPH_SYSTIMER_MODULE);
      systimer_hal_init(&priv->hal);
      systimer_hal_tick_rate_ops_t ops =
        {
          .ticks_to_us = systimer_ticks_to_us,
          .us_to_ticks = systimer_us_to_ticks,
        };

      systimer_hal_set_tick_rate_ops(&priv->hal, &ops);
      systimer_ll_set_counter_value(priv->hal.dev,
                                    SYSTIMER_COUNTER_ESPTIMER,
                                    0);
      systimer_ll_apply_counter_value(priv->hal.dev,
                                      SYSTIMER_COUNTER_ESPTIMER);

#if !SOC_SYSTIMER_FIXED_DIVIDER
      ASSERT(esp_clk_xtal_freq() == (40 * 1000000));

      systimer_hal_set_steps_per_tick(&priv->hal, 0, 2); /* For XTAL */
      systimer_hal_set_steps_per_tick(&priv->hal, 1, 1); /* For PLL */
#endif

      systimer_hal_enable_alarm_int(&priv->hal, SYSTIMER_ALARM_ESPTIMER);
      systimer_hal_enable_counter(&priv->hal, SYSTIMER_COUNTER_ESPTIMER);
      systimer_hal_select_alarm_mode(&priv->hal, SYSTIMER_ALARM_ESPTIMER,
                                     SYSTIMER_ALARM_MODE_ONESHOT);
      systimer_hal_connect_alarm_counter(&priv->hal, SYSTIMER_ALARM_ESPTIMER,
                                         SYSTIMER_COUNTER_ESPTIMER);

      leave_critical_section(flags);
    }

  esp_setup_irq(SYSTIMER_TARGET2_EDGE_INTR_SOURCE,
                ESP_IRQ_PRIORITY_DEFAULT,
                SYSTIMER_TRIGGER_TYPE);

  /* Attach the systimer interrupt */

  irq_attach(ESP_IRQ_SYSTIMER_TARGET2_EDGE, (xcpt_t)esp_hr_timer_isr, NULL);

  /* Enable the allocated CPU interrupt */

  up_enable_irq(ESP_IRQ_SYSTIMER_TARGET2_EDGE);

  return 0;
}
