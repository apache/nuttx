/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_rt_timer.c
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
#include <nuttx/spinlock.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32s3_irq.h"
#include "esp32s3_rt_timer.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_systimer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RT_TIMER_TASK_NAME        CONFIG_ESP32S3_RT_TIMER_TASK_NAME
#define RT_TIMER_TASK_PRIORITY    CONFIG_ESP32S3_RT_TIMER_TASK_PRIORITY
#define RT_TIMER_TASK_STACK_SIZE  CONFIG_ESP32S3_RT_TIMER_TASK_STACK_SIZE

#ifdef CONFIG_SCHED_HPWORKPRIORITY
static_assert(RT_TIMER_TASK_PRIORITY < CONFIG_SCHED_HPWORKPRIORITY,
  "RT Timer priority should be smaller than high-prio workqueue");
#endif

/* Timer running at 16 MHz */

#define CYCLES_PER_USEC           16
#define USEC_TO_CYCLES(u)         ((u) * CYCLES_PER_USEC)
#define CYCLES_TO_USEC(c)         ((c) / CYCLES_PER_USEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_rt_priv_s
{
  pid_t pid;                  /* PID of RT Timer kernel thread */
  int cpuint;                 /* CPU interrupt assigned to this timer */
  int core;                   /* Core that is taking care of the timer
                               * interrupts
                               */
  sem_t toutsem;              /* Semaphore for synchronizing access to list
                               * of timed-out timers
                               */
  struct list_node runlist;   /* List of timers in the running state */
  struct list_node toutlist;  /* List of timed-out timers */
  spinlock_t lock;            /* Device-specific lock */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32s3_rt_priv_s g_rt_priv =
{
  .pid     = INVALID_PROCESS_ID,
  .cpuint  = -ENOMEM,
  .core    = -ENODEV,
  .toutsem = SEM_INITIALIZER(0),
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rt_timer_getcounter
 *
 * Description:
 *   Get the current counter value.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Current counter value.
 *
 ****************************************************************************/

static inline uint64_t rt_timer_getcounter(void)
{
  uint32_t lo;
  uint32_t lo_start;
  uint32_t hi;
  uint64_t counter;

  /* Trigger an update event */

  modifyreg32(SYSTIMER_UNIT1_OP_REG, 0, SYSTIMER_TIMER_UNIT1_UPDATE);

  /* Wait until the value is valid */

  while ((getreg32(SYSTIMER_UNIT1_OP_REG) &
          SYSTIMER_TIMER_UNIT1_VALUE_VALID) !=
         SYSTIMER_TIMER_UNIT1_VALUE_VALID);

  /* Read LO, HI, then LO again, check that LO returns the same value.
   * This accounts for the case when an interrupt may happen between reading
   * HI and LO values, and this function may get called from the ISR.
   * In this case, the repeated read will return consistent values.
   */

  lo_start = getreg32(SYSTIMER_UNIT1_VALUE_LO_REG);
  do
    {
      lo = lo_start;
      hi = getreg32(SYSTIMER_UNIT1_VALUE_HI_REG);
      lo_start = getreg32(SYSTIMER_UNIT1_VALUE_LO_REG);
    }
  while (lo_start != lo);

  counter = ((uint64_t) hi << 32) | lo;

  return counter;
}

/****************************************************************************
 * Name: rt_timer_setcounter
 *
 * Description:
 *   Set the counter value.
 *
 * Input Parameters:
 *   value         - The value to be loaded to the counter.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void rt_timer_setcounter(uint64_t value)
{
  /* Set counter value */

  putreg32(value & SYSTIMER_TIMER_UNIT1_VALUE_LO_V,
           SYSTIMER_UNIT1_VALUE_LO_REG);
  putreg32((value >> 32) & SYSTIMER_TIMER_UNIT1_VALUE_HI_V,
           SYSTIMER_UNIT1_VALUE_HI_REG);

  /* Apply counter value */

  putreg32(SYSTIMER_TIMER_UNIT1_LOAD, SYSTIMER_UNIT1_LOAD_REG);
}

/****************************************************************************
 * Name: rt_timer_getalarmvalue
 *
 * Description:
 *   Get the alarm value.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Remaining ticks for expiration.
 *
 ****************************************************************************/

static inline uint64_t rt_timer_getalarmvalue(void)
{
  uint32_t hi = getreg32(SYSTIMER_TARGET2_HI_REG);
  uint32_t lo = getreg32(SYSTIMER_TARGET2_LO_REG);
  uint64_t ticks = ((uint64_t) hi << 32) | lo;

  return ticks;
}

/****************************************************************************
 * Name: rt_timer_setalarmvalue
 *
 * Description:
 *   Set the value that will trigger an alarm when the counter value matches
 *   this value.
 *
 * Input Parameters:
 *   value         - The alarm value.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void rt_timer_setalarmvalue(uint64_t value)
{
  /* Set alarm value */

  putreg32(value & 0xffffffff, SYSTIMER_TARGET2_LO_REG);
  putreg32((value >> 32) & 0xfffff, SYSTIMER_TARGET2_HI_REG);

  /* Apply alarm value */

  putreg32(SYSTIMER_TIMER_COMP2_LOAD, SYSTIMER_COMP2_LOAD_REG);
}

/****************************************************************************
 * Name: rt_timer_setalarm
 *
 * Description:
 *   Enable/Disable the alarm.
 *
 * Input Parameters:
 *   enable        - A variable to indicate the action. If true, enable
 *                   the alarm, otherwise disable it.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void rt_timer_setalarm(bool enable)
{
  if (enable)
    {
      modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TARGET2_WORK_EN);
    }
  else
    {
      modifyreg32(SYSTIMER_CONF_REG, SYSTIMER_TARGET2_WORK_EN, 0);
    }
}

/****************************************************************************
 * Name: rt_timer_setisr
 *
 * Description:
 *   Allocate a CPU Interrupt, connect the peripheral source to this
 *   Interrupt, register the callback and enable the CPU Interrupt.
 *   In case a NULL handler is provided, deallocate the interrupt and
 *   unregister the previously provided handler.
 *
 * Input Parameters:
 *   handler       - Callback to be invoked on timer interrupt.
 *   arg           - Argument to be passed to the handler callback.
 *
 * Returned Values:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

static int rt_timer_setisr(xcpt_t handler, void *arg)
{
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;
  int ret = OK;

  /* Disable interrupt when callback is removed. */

  if (handler == NULL)
    {
      /* If a CPU Interrupt was previously allocated, then deallocate it */

      if (priv->cpuint != -ENOMEM)
        {
          /* Disable CPU Interrupt, free a previously allocated
           * CPU Interrupt
           */

          up_disable_irq(ESP32S3_IRQ_SYSTIMER_TARGET2);
          esp32s3_teardown_irq(priv->core, ESP32S3_PERIPH_SYSTIMER_TARGET2,
                               priv->cpuint);
          irq_detach(ESP32S3_IRQ_SYSTIMER_TARGET2);

          priv->cpuint = -ENOMEM;
          priv->core   = -ENODEV;
        }
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      if (priv->cpuint != -ENOMEM)
        {
          /* Disable the previous IRQ */

          up_disable_irq(ESP32S3_IRQ_SYSTIMER_TARGET2);
        }

      /* Set up to receive peripheral interrupts on the current CPU */

      priv->core = up_cpu_index();
      priv->cpuint = esp32s3_setup_irq(priv->core,
                                       ESP32S3_PERIPH_SYSTIMER_TARGET2,
                                       1, ESP32S3_CPUINT_LEVEL);
      if (priv->cpuint < 0)
        {
          tmrerr("ERROR: No CPU Interrupt available");
          ret = priv->cpuint;
          goto errout;
        }

      /* Associate an IRQ Number (from the timer) to an ISR */

      ret = irq_attach(ESP32S3_IRQ_SYSTIMER_TARGET2, handler, arg);
      if (ret != OK)
        {
          esp32s3_teardown_irq(priv->core, ESP32S3_PERIPH_SYSTIMER_TARGET2,
                               priv->cpuint);
          tmrerr("ERROR: Failed to associate an IRQ Number");
          goto errout;
        }

      /* Enable the CPU Interrupt that is linked to the timer */

      up_enable_irq(ESP32S3_IRQ_SYSTIMER_TARGET2);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: start_rt_timer
 *
 * Description:
 *   Start the timer by inserting it into the running list and reset the
 *   hardware timer alarm value if this timer is at the head of the list.
 *   Larger timeouts go to the end of the list (tail).
 *
 * Input Parameters:
 *   timer         - Pointer to the RT Timer state structure.
 *   timeout       - Timeout value.
 *   repeat        - Repeat mode (true: enabled, false: disabled).
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
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  flags = spin_lock_irqsave(&priv->lock);

  /* Only idle timer can be started */

  if (timer->state == RT_TIMER_IDLE)
    {
      struct rt_timer_s *temp_p;
      bool inserted = false;

      /* Calculate the timer's alarm value */

      uint64_t counter = rt_timer_getcounter();
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

      /* Scan the timer list and insert the new timer into previous node of
       * timer whose alarm value is larger than new one.
       */

      list_for_every_entry(&priv->runlist, temp_p, struct rt_timer_s, list)
        {
          if (temp_p->alarm > timer->alarm)
            {
              list_add_before(&temp_p->list, &timer->list);
              inserted = true;
              break;
            }
        }

      /* If we didn't find a larger one, insert the new timer at the tail of
       * the list.
       */

      if (!inserted)
        {
          list_add_tail(&priv->runlist, &timer->list);
        }

      timer->state = RT_TIMER_READY;

      /* Check if this timer is at the head of the list */

      if (timer == container_of(priv->runlist.next, struct rt_timer_s, list))
        {
          /* Reset the hardware timer alarm */

          rt_timer_setalarm(false);
          rt_timer_setalarmvalue(USEC_TO_CYCLES(timer->alarm));
          rt_timer_setalarm(true);
        }
    }
  else
    {
      tmrwarn("Timer not in idle mode. Only idle timer can be started!\n");
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: stop_rt_timer
 *
 * Description:
 *   Stop the timer by removing it from the running list and reset the
 *   hardware timer alarm value if this timer is at the head of list.
 *
 * Input Parameters:
 *   timer         - Pointer to the RT Timer state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void stop_rt_timer(struct rt_timer_s *timer)
{
  irqstate_t flags;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  flags = spin_lock_irqsave(&priv->lock);

  /* "start" function can set the timer's repeat flag, and "stop" function
   * should remove this flag.
   */

  timer->flags &= ~RT_TIMER_REPEAT;

  /* Only timers in "ready" state can be stopped */

  if (timer->state == RT_TIMER_READY)
    {
      bool ishead;

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

      if (ishead)
        {
          if (!list_is_empty(&priv->runlist))
            {
              /* Set the value from the next timer as the new hardware timer
               * alarm value.
               */

              struct rt_timer_s *next_timer =
                container_of(priv->runlist.next, struct rt_timer_s, list);

              rt_timer_setalarm(false);
              rt_timer_setalarmvalue(USEC_TO_CYCLES(next_timer->alarm));
              rt_timer_setalarm(true);
            }
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: rt_timer_thread
 *
 * Description:
 *   RT Timer working thread: Waits for a timeout semaphore, scans the
 *   timeout list and processes all the timers in the list.
 *
 * Input Parameters:
 *   argc          - Not used.
 *   argv          - Not used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

static int rt_timer_thread(int argc, char *argv[])
{
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  while (1)
    {
      /* Waiting for all timers to time out */

      DEBUGVERIFY(nxsem_wait_uninterruptible(&priv->toutsem));

      irqstate_t flags = spin_lock_irqsave(&priv->lock);

      /* Process all the timers in list */

      while (!list_is_empty(&priv->toutlist))
        {
          /* Get the first timer in the list */

          struct rt_timer_s *timer = container_of(priv->toutlist.next,
                                                  struct rt_timer_s, list);

          /* Cache the raw state to decide how to deal with this timer */

          enum rt_timer_state_e raw_state = timer->state;

          /* Delete the timer from the list */

          list_delete(&timer->list);

          /* Set timer's state to idle so it can be restarted by the user. */

          timer->state = RT_TIMER_IDLE;

          spin_unlock_irqrestore(&priv->lock, flags);

          if (raw_state == RT_TIMER_TIMEOUT)
            {
              timer->callback(timer->arg);
            }
          else if (raw_state == RT_TIMER_DELETE)
            {
              kmm_free(timer);
            }

          /* Enter critical section for next scanning list */

          flags = spin_lock_irqsave(&priv->lock);

          if (raw_state == RT_TIMER_TIMEOUT)
            {
              /* Check if the timer is in "repeat" mode */

              if ((timer->flags & RT_TIMER_REPEAT) != 0)
                {
                  start_rt_timer(timer, timer->timeout, true);
                }
            }
        }

      spin_unlock_irqrestore(&priv->lock, flags);
    }

  return OK;
}

/****************************************************************************
 * Name: rt_timer_isr
 *
 * Description:
 *   Hardware timer interrupt service routine.
 *
 * Input Parameters:
 *   dev           - Pointer to the driver state structure.
 *   handler       - Callback to be invoked on timer interrupt.
 *   arg           - Argument to be passed to the handler callback.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

static int rt_timer_isr(int irq, void *context, void *arg)
{
  irqstate_t flags;
  bool wake = false;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  /* Clear interrupt register status */

  modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_TARGET2_INT_CLR);

  flags = spin_lock_irqsave(&priv->lock);

  /* Check if there is a timer running */

  if (!list_is_empty(&priv->runlist))
    {
      struct rt_timer_s *timer;

      /* When stop/delete timer, at the same time the hardware timer
       * interrupt triggers, function "stop/delete" removes the timer
       * from running list, so the timer at head is not is not the one being
       * triggered.
       */

      uint64_t counter = rt_timer_getcounter();
      counter = CYCLES_TO_USEC(counter);
      timer = container_of(priv->runlist.next, struct rt_timer_s, list);
      if (timer->alarm <= counter)
        {
          /* Remove the first timer from the running list and add it to
           * the timeout list.
           *
           * Set the timer's state to RT_TIMER_TIMEOUT to avoid any other
           * operations.
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

              rt_timer_setalarm(false);
              rt_timer_setalarmvalue(USEC_TO_CYCLES(timer->alarm));
            }
        }

      /* If there is a timer in the list, the alarm should be enabled */

      rt_timer_setalarm(true);
    }

  if (wake)
    {
      /* Wake up the thread to process timed-out timers */

      int ret = nxsem_post(&priv->toutsem);
      if (ret < 0)
        {
          tmrerr("ERROR: Failed to post sem ret=%d\n", ret);
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_rt_timer_create
 *
 * Description:
 *   Create an RT Timer from the provided arguments.
 *
 * Input Parameters:
 *   args         - RT Timer creation arguments.
 *
 * Output Parameters:
 *   timer_handle - Pointer to RT Timer handle.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int esp32s3_rt_timer_create(const struct rt_timer_args_s *args,
                            struct rt_timer_s **timer_handle)
{
  struct rt_timer_s *timer;

  DEBUGASSERT(args != NULL);
  DEBUGASSERT(args->callback != NULL);

  timer = (struct rt_timer_s *)kmm_malloc(sizeof(*timer));
  if (timer == NULL)
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

  return OK;
}

/****************************************************************************
 * Name: esp32s3_rt_timer_start
 *
 * Description:
 *   Start the RT Timer.
 *
 * Input Parameters:
 *   timer         - Pointer to the RT Timer state structure.
 *   timeout       - Timeout value.
 *   repeat        - Repeat mode (true: enabled, false: disabled).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_rt_timer_start(struct rt_timer_s *timer,
                            uint64_t timeout,
                            bool repeat)
{
  stop_rt_timer(timer);

  start_rt_timer(timer, timeout, repeat);
}

/****************************************************************************
 * Name: esp32s3_rt_timer_stop
 *
 * Description:
 *   Stop the RT Timer.
 *
 * Input Parameters:
 *   timer         - Pointer to the RT Timer state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_rt_timer_stop(struct rt_timer_s *timer)
{
  stop_rt_timer(timer);
}

/****************************************************************************
 * Name: esp32s3_rt_timer_delete
 *
 * Description:
 *   Stop and delete the RT Timer.
 *
 *   Delete the timer by removing it from the list, then set the timer's
 *   state to "RT_TIMER_DELETE" and finally insert it into the work list
 *   to let the RT Timer's thread to delete it and free the resources.
 *
 * Input Parameters:
 *   timer         - Pointer to the RT Timer state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_rt_timer_delete(struct rt_timer_s *timer)
{
  int ret;
  irqstate_t flags;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  flags = spin_lock_irqsave(&priv->lock);

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
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32s3_rt_timer_time_us
 *
 * Description:
 *   Get current counter value of the RT Timer in microseconds.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Time of the RT Timer in microseconds.
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp32s3_rt_timer_time_us(void)
{
  uint64_t counter = rt_timer_getcounter();
  counter = CYCLES_TO_USEC(counter);

  return counter;
}

/****************************************************************************
 * Name: esp32s3_rt_timer_get_alarm
 *
 * Description:
 *   Get the remaining time to the next timeout.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Timestamp of the nearest timer event in microseconds.
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp32s3_rt_timer_get_alarm(void)
{
  irqstate_t flags;
  uint64_t counter;
  uint64_t alarm_value = 0;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  flags = spin_lock_irqsave(&priv->lock);

  counter = rt_timer_getcounter();
  counter = CYCLES_TO_USEC(counter);

  alarm_value = rt_timer_getalarmvalue();
  alarm_value = CYCLES_TO_USEC(alarm_value);

  if (alarm_value <= counter)
    {
      alarm_value = 0;
    }
  else
    {
      alarm_value -= counter;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return alarm_value;
}

/****************************************************************************
 * Name: esp32s3_rt_timer_calibration
 *
 * Description:
 *   Adjust current RT Timer by a certain value.
 *
 * Input Parameters:
 *   time_us       - Adjustment to apply to the RT Timer in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_rt_timer_calibration(uint64_t time_us)
{
  irqstate_t flags;
  uint64_t counter;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  flags = spin_lock_irqsave(&priv->lock);
  counter  = rt_timer_getcounter();
  counter  = CYCLES_TO_USEC(counter);
  counter += time_us;
  rt_timer_setcounter(USEC_TO_CYCLES(counter));
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: esp32s3_rt_timer_init
 *
 * Description:
 *   Initialize ESP32-S3 RT Timer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int esp32s3_rt_timer_init(void)
{
  int pid;
  irqstate_t flags;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  pid = kthread_create(RT_TIMER_TASK_NAME,
                       RT_TIMER_TASK_PRIORITY,
                       RT_TIMER_TASK_STACK_SIZE,
                       rt_timer_thread,
                       NULL);
  if (pid < 0)
    {
      tmrerr("ERROR: Failed to create RT Timer task error=%d\n", pid);
      return pid;
    }

  list_initialize(&priv->runlist);
  list_initialize(&priv->toutlist);

  priv->pid = (pid_t)pid;

  flags = spin_lock_irqsave(&priv->lock);

  /* ESP32-S3 hardware timer configuration:
   * 1 count = 1/16 us
   * 1) Set Alarm mode (non-periodic).
   * 2) Clear the counter.
   * 3) Set the ISR.
   * 4) Enable timeout interrupt.
   * 5) Start the counter.
   * NOTE: No interrupt will be triggered until rt_timer_setalarm is set.
   */

  /* Clock and reset of Systimer peripheral is already performed in
   * up_timer_initialize(), either in esp32s3_timerisr.c or in
   * esp32s3_tickless.c.
   * Set comparator 2 to use counter 1 and set the mode to oneshot mode,
   * i.e., disable periodic mode.
   */

  modifyreg32(SYSTIMER_TARGET2_CONF_REG, SYSTIMER_TARGET2_PERIOD_MODE,
              SYSTIMER_TARGET2_TIMER_UNIT_SEL);

  rt_timer_setcounter(0);
  rt_timer_setisr(rt_timer_isr, NULL);

  /* Ensure Systimer 1 keeps running even when the CPUs are temporarily
   * stalled.
   * This is required for the correct operation of the Wi-Fi driver.
   */

  modifyreg32(SYSTIMER_CONF_REG, SYSTIMER_TIMER_UNIT1_CORE0_STALL_EN, 0);
  modifyreg32(SYSTIMER_CONF_REG, SYSTIMER_TIMER_UNIT1_CORE1_STALL_EN, 0);

  /* Enable interrupts */

  modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_TARGET2_INT_CLR);
  modifyreg32(SYSTIMER_INT_ENA_REG, 0, SYSTIMER_TARGET2_INT_ENA);

  /* Start counter 1 */

  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TIMER_UNIT1_WORK_EN);

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_rt_timer_deinit
 *
 * Description:
 *   Deinitialize ESP32-S3 RT Timer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_rt_timer_deinit(void)
{
  irqstate_t flags;
  struct esp32s3_rt_priv_s *priv = &g_rt_priv;

  flags = spin_lock_irqsave(&priv->lock);

  /* Stop counter 1 */

  modifyreg32(SYSTIMER_CONF_REG, SYSTIMER_TIMER_UNIT1_WORK_EN, 0);

  /* Disable interrupts */

  modifyreg32(SYSTIMER_INT_ENA_REG, SYSTIMER_TARGET2_INT_ENA, 0);
  modifyreg32(SYSTIMER_INT_CLR_REG, SYSTIMER_TARGET2_INT_CLR, 0);

  rt_timer_setisr(NULL, NULL);

  spin_unlock_irqrestore(&priv->lock, flags);

  if (priv->pid != INVALID_PROCESS_ID)
    {
      kthread_delete(priv->pid);
      priv->pid = INVALID_PROCESS_ID;
    }
}
