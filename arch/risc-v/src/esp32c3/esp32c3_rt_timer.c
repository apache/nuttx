/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rt_timer.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include "hardware/esp32c3_soc.h"
#include "esp32c3_tim.h"
#include "esp32c3_rt_timer.h"
#include "esp32c3_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORKPRIORITY
#  if CONFIG_ESP32C3_RT_TIMER_TASK_PRIORITY >= CONFIG_SCHED_HPWORKPRIORITY
#    error "RT timer priority should be smaller that high-prio workqueue"
#  endif
#endif

#define RT_TIMER_TASK_NAME        CONFIG_ESP32C3_RT_TIMER_TASK_NAME
#define RT_TIMER_TASK_PRIORITY    CONFIG_ESP32C3_RT_TIMER_TASK_PRIORITY
#define RT_TIMER_TASK_STACK_SIZE  CONFIG_ESP32C3_RT_TIMER_TASK_STACK_SIZE

#define CYCLES_PER_USEC           16 /* Timer running at 16 MHz*/
#define USEC_TO_CYCLES(u)         ((u) * CYCLES_PER_USEC)
#define CYCLES_TO_USEC(c)         ((c) / CYCLES_PER_USEC)
#define ESP32C3_RT_TIMER          ESP32C3_SYSTIM /* Systimer 1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3_rt_priv_s
{
  int pid;
  sem_t toutsem;
  struct list_node runlist;
  struct list_node toutlist;
  struct esp32c3_tim_dev_s *timer;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32c3_rt_priv_s g_rt_priv =
{
  .pid = -EINVAL,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: start_rt_timer
 *
 * Description:
 *   Start the timer by inserting it into the running list and reset the
 *   hardware timer alarm value if this timer is at the head of the list.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *   timeout - Timeout value
 *   repeat  - repeat mode (true: enabled, false: disabled)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void start_rt_timer(struct rt_timer_s *timer,
                           uint64_t timeout,
                           bool repeat)
{
  irqstate_t flags;
  struct rt_timer_s *p;
  bool inserted = false;
  uint64_t counter;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  flags = enter_critical_section();

  /* Only idle timer can be started */

  if (timer->state == RT_TIMER_IDLE)
    {
      /* Calculate the timer's alarm value */

      ESP32C3_TIM_GETCTR(priv->timer, &counter);
      counter = CYCLES_TO_USEC(counter);
      timer->timeout = timeout;
      timer->alarm = timer->timeout + counter;

      if (repeat)
        {
          timer->flags |= RT_TIMER_REPEAT;
        }
      else
        {
          timer->flags &= ~RT_TIMER_REPEAT;
        }

      /* Scan the timer list and insert the new timer into previous
       * node of timer whose alarm value is larger than new one
       */

      list_for_every_entry(&priv->runlist, p, struct rt_timer_s, list)
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

      timer->state = RT_TIMER_READY;

      /* If this timer is at the head of the list */

      if (timer == container_of(priv->runlist.next,
                                struct rt_timer_s, list))
        {
          /* Reset the hardware timer alarm */

          ESP32C3_TIM_SETALRM(priv->timer, false);
          ESP32C3_TIM_SETALRVL(priv->timer, USEC_TO_CYCLES(timer->alarm));
          ESP32C3_TIM_SETALRM(priv->timer, true);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stop_rt_timer
 *
 * Description:
 *   Stop the timer by removing it from the running list and reset the
 *   hardware timer alarm value if this timer is at the head of list.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void stop_rt_timer(struct rt_timer_s *timer)
{
  irqstate_t flags;
  bool ishead;
  struct rt_timer_s *next_timer;
  uint64_t alarm;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  flags = enter_critical_section();

  /* "start" function can set the timer's repeat flag, and "stop" function
   * should remove this flag.
   */

  timer->flags &= ~RT_TIMER_REPEAT;

  /* Only timers in "ready" state can be stopped */

  if (timer->state == RT_TIMER_READY)
    {
      /* Check if the timer is at the head of the list */

      if (timer == container_of(priv->runlist.next,
                                struct rt_timer_s, list))
        {
          ishead = true;
        }
      else
        {
          ishead = false;
        }

      list_delete(&timer->list);
      timer->state = RT_TIMER_IDLE;

      /* If the timer is at the head of the list */

      if (ishead)
        {
          if (!list_is_empty(&priv->runlist))
            {
              /* Set the value from the next timer as the new hardware timer
               * alarm value.
               */

              next_timer = container_of(priv->runlist.next,
                                        struct rt_timer_s,
                                        list);
              alarm = next_timer->alarm;

              ESP32C3_TIM_SETALRM(priv->timer, false);
              ESP32C3_TIM_SETALRVL(priv->timer, USEC_TO_CYCLES(alarm));
              ESP32C3_TIM_SETALRM(priv->timer, true);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: delete_rt_timer
 *
 * Description:
 *   Delete the timer by removing it from the list, then set the timer's
 *   state to "RT_TIMER_DELETE" and finally insert it into the work list
 *   to let the rt-timer's thread to delete it and free the resources.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void delete_rt_timer(struct rt_timer_s *timer)
{
  int ret;
  irqstate_t flags;

  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  flags = enter_critical_section();

  if (timer->state == RT_TIMER_READY)
    {
      stop_rt_timer(timer);
    }
  else if (timer->state == RT_TIMER_TIMEOUT)
    {
      list_delete(&timer->list);
    }
  else if (timer->state == RT_TIMER_DELETE)
    {
      goto exit;
    }

  list_add_after(&priv->toutlist, &timer->list);
  timer->state = RT_TIMER_DELETE;

  /* Wake up the thread to process deleted timers */

  ret = nxsem_post(&priv->toutsem);
  if (ret < 0)
    {
      tmrerr("ERROR: Failed to post sem ret=%d\n", ret);
    }

exit:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rt_timer_thread
 *
 * Description:
 *   RT timer working thread: Waits for a timeout semaphore, scans
 *   the timeout list and processes all the timers in the list.
 *
 * Input Parameters:
 *   argc - Not used
 *   argv - Not used
 *
 * Returned Value:
 *   0.
 *
 ****************************************************************************/

static int rt_timer_thread(int argc, char *argv[])
{
  int ret;
  irqstate_t flags;
  struct rt_timer_s *timer;
  enum rt_timer_state_e raw_state;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  while (1)
    {
      /* Waiting for all timers to time out */

      ret = nxsem_wait(&priv->toutsem);
      if (ret)
        {
          tmrerr("ERROR: Wait toutsem error=%d\n", ret);
          assert(0);
        }

      flags = enter_critical_section();

      /* Process all the timers in list */

      while (!list_is_empty(&priv->toutlist))
        {
          /* Get the first timer in the list */

          timer = container_of(priv->toutlist.next,
                               struct rt_timer_s, list);

          /* Cache the raw state to decide how to deal with this timer */

          raw_state = timer->state;

          /* Delete the timer from the list */

          list_delete(&timer->list);

          /* Set timer's state to idle so it can be restarted by the user. */

          timer->state = RT_TIMER_IDLE;

          leave_critical_section(flags);

          if (raw_state == RT_TIMER_TIMEOUT)
            {
              timer->callback(timer->arg);
            }
          else if (raw_state == RT_TIMER_DELETE)
            {
              kmm_free(timer);
            }

          /* Enter critical section for next scanning list */

          flags = enter_critical_section();

          if (raw_state == RT_TIMER_TIMEOUT)
            {
              /* Check if the timer is in "repeat" mode */

              if (timer->flags & RT_TIMER_REPEAT)
                {
                  start_rt_timer(timer, timer->timeout, true);
                }
            }
        }

      leave_critical_section(flags);
    }

  return 0;
}

/****************************************************************************
 * Name: rt_timer_isr
 *
 * Description:
 *   Hardware timer interrupt service routine.
 *
 * Input Parameters:
 *   irq     - Not used
 *   context - Not used
 *   arg     - Not used
 *
 * Returned Value:
 *   0.
 *
 ****************************************************************************/

static int rt_timer_isr(int irq, void *context, void *arg)
{
  int ret;
  irqstate_t flags;
  struct rt_timer_s *timer;
  uint64_t alarm;
  uint64_t counter;
  bool wake = false;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  /* Clear interrupt register status */

  ESP32C3_TIM_ACKINT(priv->timer);

  flags = enter_critical_section();

  /* Check if there is a timer running */

  if (!list_is_empty(&priv->runlist))
    {
      /**
       * When stop/delete timer, in the same time the hardware timer
       * interrupt triggers, function "stop/delete" remove the timer
       * from running list, so the 1st timer is not which triggers.
       */

      timer = container_of(priv->runlist.next, struct rt_timer_s, list);
      ESP32C3_TIM_GETCTR(priv->timer, &counter);
      counter = CYCLES_TO_USEC(counter);
      if (timer->alarm <= counter)
        {
          /* Remove the first timer from the running list and add it to
           * the timeout list.
           *
           * Set the timer's state to be RT_TIMER_TIMEOUT to avoid
           * other operations.
           */

          list_delete(&timer->list);
          timer->state = RT_TIMER_TIMEOUT;
          list_add_after(&priv->toutlist, &timer->list);
          wake = true;

          /* Check if there is a timer running */

          if (!list_is_empty(&priv->runlist))
            {
              /* Reset hardware timer alarm with next timer's alarm value */

              timer = container_of(priv->runlist.next,
                                   struct rt_timer_s, list);
              alarm = timer->alarm;

              ESP32C3_TIM_SETALRM(priv->timer, false);
              ESP32C3_TIM_SETALRVL(priv->timer, USEC_TO_CYCLES(alarm));
            }
        }

      /* If there is a timer in the list, the alarm should be enabled */

      ESP32C3_TIM_SETALRM(priv->timer, true);
    }

  if (wake)
    {
      /* Wake up the thread to process timed-out timers */

      ret = nxsem_post(&priv->toutsem);
      if (ret < 0)
        {
          tmrerr("ERROR: Failed to post sem ret=%d\n", ret);
        }
    }

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rt_timer_create
 *
 * Description:
 *   Create a RT timer from the provided arguments.
 *
 * Input Parameters:
 *   args         - Input RT timer creation arguments
 *   timer_handle - Output RT timer handle pointer
 *
 * Returned Value:
 *   0 is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int rt_timer_create(const struct rt_timer_args_s *args,
                    struct rt_timer_s **timer_handle)
{
  struct rt_timer_s *timer;

  timer = (struct rt_timer_s *)kmm_malloc(sizeof(*timer));
  if (!timer)
    {
      tmrerr("ERROR: Failed to allocate %d bytes\n", sizeof(*timer));
      return -ENOMEM;
    }

  timer->callback = args->callback;
  timer->arg      = args->arg;
  timer->flags    = RT_TIMER_NOFLAGS;
  timer->state    = RT_TIMER_IDLE;
  list_initialize(&timer->list);

  *timer_handle = timer;

  return 0;
}

/****************************************************************************
 * Name: rt_timer_start
 *
 * Description:
 *   Start the RT timer.
 *
 * Input Parameters:
 *   timer   - RT timer pointer
 *   timeout - Timeout value
 *   repeat  - repeat mode (true: enabled, false: disabled)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rt_timer_start(struct rt_timer_s *timer,
                    uint64_t timeout,
                    bool repeat)
{
  stop_rt_timer(timer);

  start_rt_timer(timer, timeout, repeat);
}

/****************************************************************************
 * Name: rt_timer_stop
 *
 * Description:
 *   Stop the RT timer.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rt_timer_stop(struct rt_timer_s *timer)
{
  stop_rt_timer(timer);
}

/****************************************************************************
 * Name: rt_timer_delete
 *
 * Description:
 *   Stop and delete the RT timer.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rt_timer_delete(struct rt_timer_s *timer)
{
  delete_rt_timer(timer);
}

/****************************************************************************
 * Name: rt_timer_time_us
 *
 * Description:
 *   Get current counter value of the RT timer in microseconds.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Time of the RT timer in microseconds.
 *
 ****************************************************************************/

uint64_t IRAM_ATTR rt_timer_time_us(void)
{
  uint64_t counter;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  ESP32C3_TIM_GETCTR(priv->timer, &counter);
  counter = CYCLES_TO_USEC(counter);

  return counter;
}

/****************************************************************************
 * Name: rt_timer_get_alarm
 *
 * Description:
 *   Get the remaining time to the next timeout.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Timestamp of the nearest timer event in microseconds.
 *
 ****************************************************************************/

uint64_t IRAM_ATTR rt_timer_get_alarm(void)
{
  irqstate_t flags;
  uint64_t counter;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;
  uint64_t alarm_value = 0;

  flags = enter_critical_section();

  ESP32C3_TIM_GETCTR(priv->timer, &counter);
  counter = CYCLES_TO_USEC(counter);
  ESP32C3_TIM_GETALRVL(priv->timer, &alarm_value);
  alarm_value = CYCLES_TO_USEC(alarm_value);

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
 * Name: rt_timer_calibration
 *
 * Description:
 *   Adjust current RT timer by a certain value.
 *
 * Input Parameters:
 *   time_us - adjustment to apply to the RT timer in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR rt_timer_calibration(uint64_t time_us)
{
  uint64_t counter;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;
  irqstate_t flags;

  flags = enter_critical_section();
  ESP32C3_TIM_GETCTR(priv->timer, &counter);
  counter = CYCLES_TO_USEC(counter);
  counter += time_us;
  ESP32C3_TIM_SETCTR(priv->timer, USEC_TO_CYCLES(counter));
  ESP32C3_TIM_RLD_NOW(priv->timer);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_rt_timer_init
 *
 * Description:
 *   Initialize ESP32-C3 RT timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_rt_timer_init(void)
{
  int pid;
  irqstate_t flags;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  priv->timer = esp32c3_tim_init(ESP32C3_RT_TIMER);
  if (priv->timer == NULL)
    {
      tmrerr("ERROR: Failed to initialize ESP32-C3 timer0\n");
      return -EINVAL;
    }

  nxsem_init(&priv->toutsem, 0, 0);

  pid = kthread_create(RT_TIMER_TASK_NAME,
                       RT_TIMER_TASK_PRIORITY,
                       RT_TIMER_TASK_STACK_SIZE,
                       rt_timer_thread,
                       NULL);
  if (pid < 0)
    {
      tmrerr("ERROR: Failed to create RT timer task error=%d\n", pid);
      esp32c3_tim_deinit(priv->timer);
      return pid;
    }

  list_initialize(&priv->runlist);
  list_initialize(&priv->toutlist);

  priv->pid = pid;

  flags = enter_critical_section();

  /* ESP32-C3 hardware timer configuration:
   * 1 count = 1/16 us
   * Clear the counter.
   * Set the ISR.
   * Enable timeout interrupt.
   * Start the counter.
   * NOTE: No interrupt will be triggered
   * until ESP32C3_TIM_SETALRM is set.
   */

  ESP32C3_TIM_CLEAR(priv->timer);
  ESP32C3_TIM_SETISR(priv->timer, rt_timer_isr, NULL);
  ESP32C3_TIM_ENABLEINT(priv->timer);
  ESP32C3_TIM_START(priv->timer);

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: esp32c3_rt_timer_deinit
 *
 * Description:
 *   Deinitialize ESP32-C3 RT timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_rt_timer_deinit(void)
{
  irqstate_t flags;
  struct esp32c3_rt_priv_s *priv = &g_rt_priv;

  flags = enter_critical_section();

  ESP32C3_TIM_STOP(priv->timer);
  ESP32C3_TIM_DISABLEINT(priv->timer);
  ESP32C3_TIM_SETISR(priv->timer, NULL, NULL);
  esp32c3_tim_deinit(priv->timer);
  priv->timer = NULL;

  leave_critical_section(flags);

  if (priv->pid != -EINVAL)
    {
      kthread_delete(priv->pid);
      priv->pid = -EINVAL;
    }

  nxsem_destroy(&priv->toutsem);
}
