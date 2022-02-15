/****************************************************************************
 * arch/risc-v/src/bl602/bl602_os_hal.c
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

#include <bl602_os_hal.h>

#ifdef CONFIG_BL602_WIRELESS

#include <bl_os_adapter/bl_os_adapter.h>
#include <bl_os_adapter/bl_os_log.h>

#endif

#include <debug.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <sys/time.h>
#include <timer/timer.h>
#include <clock/clock.h>
#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/mqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>

#include <bl602_netdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OS_HPWORK HPWORK
#define OS_LPWORK LPWORK

#ifdef CONFIG_BL602_WIRELESS

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mq_adpt
{
  struct file mq;           /* Message queue handle */
  uint32_t    msgsize;      /* Message size */
  char        name[16];     /* Message queue name */
};

struct irq_adpt
{
  void (*func)(void *arg);  /* Interrupt callback function */
  void *arg;                /* Interrupt private data */
};

enum bl_os_timer_mode
{
  BL_OS_TIEMR_ONCE = 0,
  BL_OS_TIEMR_CYCLE
};

typedef enum bl_os_timer_mode bl_os_timer_mode_t;

struct timer_adpt
{
  struct wdog_s wdog;
  int32_t delay;
  bl_os_timer_mode_t mode;
  void (*func)(void *arg);
  void *arg;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern void *__attribute__((weak)) _wifi_log_flag;

bl_ops_funcs_t g_bl_ops_funcs =
{
  ._version = BL_OS_ADAPTER_VERSION,
  ._printf = bl_os_printf,
  ._assert = bl_os_assert_func,
  ._init = bl_os_api_init,
  ._enter_critical = bl_os_enter_critical,
  ._exit_critical = bl_os_exit_critical,
  ._msleep = bl_os_msleep,
  ._sleep = bl_os_sleep,
  ._event_group_create = bl_os_event_create,
  ._event_group_delete = bl_os_event_delete,
  ._event_group_send = bl_os_event_send,
  ._event_group_wait = bl_os_event_wait,
  ._event_register = bl_os_event_register,
  ._event_notify = bl_os_event_notify,
  ._task_create = bl_os_task_create,
  ._task_delete = bl_os_task_delete,
  ._task_get_current_task = bl_os_task_get_current_task,
  ._task_notify_create = bl_os_task_notify_create,
  ._task_notify = bl_os_task_notify,
  ._task_wait = bl_os_task_wait,
  ._lock_gaint = bl_os_lock_gaint,
  ._unlock_gaint = bl_os_unlock_gaint,
  ._irq_attach = bl_os_irq_attach,
  ._irq_enable = bl_os_irq_enable,
  ._irq_disable = bl_os_irq_disable,
  ._workqueue_create = bl_os_workqueue_create,
  ._workqueue_submit_hp = bl_os_workqueue_submit_hpwork,
  ._workqueue_submit_lp = bl_os_workqueue_submit_lpwork,
  ._timer_create = bl_os_timer_create,
  ._timer_delete = bl_os_timer_delete,
  ._timer_start_once = bl_os_timer_start_once,
  ._timer_start_periodic = bl_os_timer_start_periodic,
  ._sem_create = bl_os_sem_create,
  ._sem_delete = bl_os_sem_delete,
  ._sem_take = bl_os_sem_take,
  ._sem_give = bl_os_sem_give,
  ._mutex_create = bl_os_mutex_create,
  ._mutex_delete = bl_os_mutex_delete,
  ._mutex_lock = bl_os_mutex_lock,
  ._mutex_unlock = bl_os_mutex_unlock,
  ._queue_create = bl_os_mq_creat,
  ._queue_delete = bl_os_mq_delete,
  ._queue_send_wait = bl_os_mq_send_wait,
  ._queue_send = bl_os_mq_send,
  ._queue_recv = bl_os_mq_recv,
  ._malloc = bl_os_malloc,
  ._free = bl_os_free,
  ._zalloc = bl_os_zalloc,
  ._get_time_ms = bl_os_clock_gettime_ms,
  ._get_tick = bl_os_get_tick,
  ._log_write = bl_os_log_write
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl_os_assert_func
 *
 * Description:
 *   Delete timer and free resource
 *
 * Input Parameters:
 *   file  - assert file
 *   line  - assert line
 *   func  - assert function
 *   expr  - assert condition
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl_os_assert_func(const char *file, int line,
                       const char *func, const char *expr)
{
  printf("Assert failed in %s, %s:%d (%s)",
         func, file, line, expr);
  abort();
}

/****************************************************************************
 * Name: bl_os_event_create
 *
 * Description:
 *   Create event group
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Event group data pointer
 *
 ****************************************************************************/

void *bl_os_event_create(void)
{
  return (void *)0;
}

/****************************************************************************
 * Name: bl_os_event_delete
 *
 * Description:
 *   Delete event and free resource
 *
 * Input Parameters:
 *   event  - event data point
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl_os_event_delete(void *event)
{
}

/****************************************************************************
 * Name: bl_os_event_send
 *
 * Description:
 *   Set event bits
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Event value after setting
 *
 ****************************************************************************/

uint32_t bl_os_event_send(void *event, uint32_t bits)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_event_wait
 *
 * Description:
 *   Delete timer and free resource
 *
 * Input Parameters:
 *   event
 *   bits_to_wait_for
 *   clear_on_exit
 *   wait_for_all_bits
 *   block_time_tick
 *
 * Returned Value:
 *   Current event value
 *
 ****************************************************************************/

uint32_t bl_os_event_wait(void *event,
                          uint32_t bits_to_wait_for,
                          int clear_on_exit,
                          int wait_for_all_bits,
                          uint32_t block_time_tick)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_event_register
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_event_register(int type, void *cb, void *arg)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_event_notify
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_event_notify(int evt, int val)
{
  bl602_net_event(evt, val);
  return 0;
}

/****************************************************************************
 * Name: bl_os_task_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_task_create(const char *name,
                      void *entry,
                      uint32_t stack_depth,
                      void *param,
                      uint32_t prio,
                      void *task_handle)
{
  return nxtask_create(name, prio, stack_depth, entry, (char **)&param);
}

/****************************************************************************
 * Name: bl_os_task_delete
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_task_delete(void *task_handle)
{
  pid_t task = (int)task_handle;

  task_delete((pid_t)task);
}

/****************************************************************************
 * Name: bl_os_task_get_current_task
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bl_os_task_get_current_task(void)
{
  return (void *)0;
}

/****************************************************************************
 * Name: bl_os_task_notify_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bl_os_task_notify_create(void)
{
  return bl_os_sem_create(0);
}

/****************************************************************************
 * Name: bl_os_task_get_current_task
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_task_notify(void *task_handle)
{
  bl_os_sem_give(task_handle);
}

/****************************************************************************
 * Name: bl_os_task_get_current_task
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_task_wait(void *task_handle, uint32_t tick)
{
  bl_os_sem_take(task_handle, tick);
}

/****************************************************************************
 * Name: bl_os_api_init
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_api_init(void)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_lock_gaint
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_lock_gaint(void)
{
}

/****************************************************************************
 * Name: bl_os_unlock_gaint
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_unlock_gaint(void)
{
}

/****************************************************************************
 * Name: bl_os_enter_critical
 *
 * Description:
 *   Enter critical state
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU PS value
 *
 ****************************************************************************/

uint32_t bl_os_enter_critical(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  return flags;
}

/****************************************************************************
 * Name: bl_os_exit_critical
 *
 * Description:
 *   Exit from critical state
 *
 * Input Parameters:
 *   level - CPU PS value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl_os_exit_critical(uint32_t level)
{
  leave_critical_section(level);
}

/****************************************************************************
 * Name: bl_os_msleep
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_msleep(long msec)
{
  useconds_t usec = msec * 1000;

  return nxsig_usleep(usec);
}

/****************************************************************************
 * Name: bl_os_sleep
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_sleep(unsigned int seconds)
{
  return nxsig_sleep(seconds);
}

/****************************************************************************
 * Name: bl_os_printf
 *
 * Description:
 *   Output format string and its arguments
 *
 * Input Parameters:
 *   format - format string
 *
 * Returned Value:
 *   0
 *
 ****************************************************************************/

void bl_os_printf(const char *__fmt, ...)
{
  if (&_wifi_log_flag)
    {
      va_list arg;

      va_start(arg, __fmt);
      vsyslog(LOG_INFO, __fmt, arg);
      va_end(arg);
    }
}

/****************************************************************************
 * Name: bl_os_malloc
 *
 * Description:
 *   Allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

void *bl_os_malloc(unsigned int size)
{
  return kmm_malloc(size);
}

/****************************************************************************
 * Name: bl_os_free
 *
 * Description:
 *   Free a block of memory
 *
 * Input Parameters:
 *   ptr - memory block
 *
 * Returned Value:
 *   No
 *
 ****************************************************************************/

void bl_os_free(void *ptr)
{
  kmm_free(ptr);
}

/****************************************************************************
 * Name: bl_os_zalloc
 *
 * Description:
 *   Allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

void *bl_os_zalloc(unsigned int size)
{
  return kmm_zalloc(size);
}

/****************************************************************************
 * Name: bl_os_update_time
 *
 * Description:
 *   Transform ticks to time and add this time to timespec value
 *
 * Input Parameters:
 *   timespec - Input timespec data pointer
 *   ticks    - System ticks
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl_os_update_time(struct timespec *timespec, uint32_t ticks)
{
  uint32_t tmp;

  tmp = TICK2SEC(ticks);
  timespec->tv_sec += tmp;

  ticks -= SEC2TICK(tmp);
  tmp = TICK2NSEC(ticks);

  timespec->tv_nsec += tmp;
}

/****************************************************************************
 * Name: bl_os_errno_trans
 *
 * Description:
 *   Transform from nuttx Os error code to Wi-Fi adapter error code
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   Wi-Fi adapter error code
 *
 ****************************************************************************/

static inline int32_t bl_os_errno_trans(int ret)
{
  if (!ret)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: bl_os_mq_creat
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bl_os_mq_creat(uint32_t queue_len, uint32_t item_size)
{
  struct mq_attr attr;
  struct mq_adpt *mq_adpt;
  int ret;

  mq_adpt = (struct mq_adpt *)kmm_malloc(sizeof(struct mq_adpt));

  if (!mq_adpt)
    {
      wlerr("ERROR: Failed to kmm_malloc\n");
      return NULL;
    }

  snprintf(mq_adpt->name, sizeof(mq_adpt->name),
           "/tmp/%p", mq_adpt);

  attr.mq_maxmsg  = queue_len;
  attr.mq_msgsize = item_size;
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&mq_adpt->mq, mq_adpt->name,
                     O_RDWR | O_CREAT, 0644, &attr);

  if (ret < 0)
    {
      wlerr("ERROR: Failed to create mqueue\n");
      kmm_free(mq_adpt);
      return NULL;
    }

  mq_adpt->msgsize = item_size;

  return (void *)mq_adpt;
}

/****************************************************************************
 * Name: bl_os_mq_delete
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_mq_delete(void *mq)
{
  struct mq_adpt *mq_adpt = (struct mq_adpt *)mq;

  file_mq_close(&mq_adpt->mq);
  file_mq_unlink(mq_adpt->name);
  kmm_free(mq_adpt);
}

/****************************************************************************
 * Name: bl_os_mq_send_generic
 *
 * Description:
 *   Generic send message to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *   prio  - Message priority
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int bl_os_mq_send_wait(void *queue, void *item, uint32_t len,
                       uint32_t ticks, int prio)
{
  int ret;
  struct timespec timeout;
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  if (ticks == BL_OS_WAITING_FOREVER || ticks == 0)
    {
      /* Wi-Fi interrupt function will call this adapter function to send
       * message to message queue, so here we should call kernel API
       * instead of application API
       */

      ret = file_mq_send(&mq_adpt->mq, (const char *)item,
                         len, prio);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to send message to mqueue error=%d\n",
                ret);
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to get time\n");
          return false;
        }

      if (ticks)
        {
          bl_os_update_time(&timeout, ticks);
        }

      ret = file_mq_timedsend(&mq_adpt->mq, (const char *)item,
                              len, prio, &timeout);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to timedsend message to mqueue error=%d\n",
                ret);
        }
    }

  return bl_os_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_mq_send
 *
 * Description:
 *   Send message of low priority to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int bl_os_mq_send(void *queue, void *item, uint32_t len)
{
  return bl_os_mq_send_wait(queue, item, len, BL_OS_WAITING_FOREVER, 0);
}

/****************************************************************************
 * Name: bl_os_mq_recv
 *
 * Description:
 *   Receive message from queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int bl_os_mq_recv(void *queue, void *item, uint32_t len, uint32_t tick)
{
  ssize_t ret;
  struct timespec timeout;
  unsigned int prio;
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  if (tick == BL_OS_WAITING_FOREVER)
    {
      ret = file_mq_receive(&mq_adpt->mq,
                            (char *)item,
                            mq_adpt->msgsize,
                            &prio);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to receive from mqueue error=%d\n", ret);
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);

      if (ret < 0)
        {
          wlerr("ERROR: Failed to get time\n");
          return false;
        }

      if (tick)
        {
          bl_os_update_time(&timeout, tick);
        }

      ret = file_mq_timedreceive(&mq_adpt->mq,
                                 (char *)item,
                                 mq_adpt->msgsize,
                                 &prio,
                                 &timeout);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to timedreceive from mqueue error=%d\n", ret);
        }
    }

  return ret < 0 ? false : true;
}

/****************************************************************************
 * Name: bl_os_timer_callback
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void bl_os_timer_callback(wdparm_t arg)
{
  struct timer_adpt *timer;
  bl_os_timer_mode_t mode;

  timer = (struct timer_adpt *)arg;
  mode = timer->mode;

  if (timer->func)
    {
      timer->func(timer->arg);
    }

  if (mode == BL_OS_TIEMR_CYCLE)
    {
      wd_start(&timer->wdog, timer->delay, bl_os_timer_callback, arg);
    }
}

/****************************************************************************
 * Name: bl_os_timer_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bl_os_timer_create(void *func, void *argv)
{
  struct timer_adpt *timer;

  timer = (struct timer_adpt *)kmm_malloc(sizeof(struct timer_adpt));

  if (!timer)
    {
      assert(0);
    }

  memset((void *)timer, 0, sizeof(struct timer_adpt));

  timer->func = func;
  timer->arg = argv;

  return timer;
}

/****************************************************************************
 * Name: bl_os_timer_delete
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_timer_delete(void *timerid, uint32_t tick)
{
  struct timer_adpt *timer;
  int ret;

  timer = (struct timer_adpt *)timerid;

  if (!timer)
    {
      return -EFAULT;
    }

  ret = wd_cancel(&timer->wdog);

  kmm_free(timerid);

  return ret;
}

/****************************************************************************
 * Name: os_timer_start_once
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_timer_start_once(void *timerid, long t_sec, long t_nsec)
{
  struct timer_adpt *timer;
  struct timespec reltime;
  int32_t tick;

  timer = (struct timer_adpt *)timerid;

  if (!timer)
    {
      return -EFAULT;
    }

  reltime.tv_nsec = t_nsec;
  reltime.tv_sec = t_sec;

  clock_time2ticks(&reltime, &tick);

  timer->mode = BL_OS_TIEMR_ONCE;
  timer->delay = tick;

  return wd_start(&timer->wdog,
                  timer->delay,
                  bl_os_timer_callback,
                  (wdparm_t)timer);
}

/****************************************************************************
 * Name: os_timer_start_periodic
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_timer_start_periodic(void *timerid, long t_sec, long t_nsec)
{
  struct timer_adpt *timer;
  struct timespec reltime;
  int32_t tick;

  timer = (struct timer_adpt *)timerid;

  if (!timer)
    {
      return -EFAULT;
    }

  reltime.tv_nsec = t_nsec;
  reltime.tv_sec = t_sec;

  clock_time2ticks(&reltime, &tick);

  timer->mode = BL_OS_TIEMR_CYCLE;
  timer->delay = tick;

  return wd_start(&timer->wdog,
                  timer->delay,
                  bl_os_timer_callback,
                  (wdparm_t)timer);
}

/****************************************************************************
 * Name: bl_os_workqueue_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bl_os_workqueue_create(void)
{
  struct work_s *work = NULL;
  work = (struct work_s *)kmm_calloc(1, sizeof(struct work_s));

  if (!work)
    {
      assert(0);
    }

  return (void *)work;
}

/****************************************************************************
 * Name: bl_os_workqueue_submit_hpwork
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_workqueue_submit_hpwork(void *work,
                                  void *worker,
                                  void *argv,
                                  long tick)
{
  if (!work)
    {
      return -EINVAL;
    }

  if (!work_available((struct work_s *)work))
    {
      return -EINVAL;
    }

  return work_queue(OS_HPWORK, work, (worker_t)worker, argv, tick);
}

/****************************************************************************
 * Name: bl_os_workqueue_submit_lpwork
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bl_os_workqueue_submit_lpwork(void *work,
                                  void *worker,
                                  void *argv,
                                  long tick)
{
  if (!work)
    {
      return -EINVAL;
    }

  if (!work_available((struct work_s *)work))
    {
      return -EINVAL;
    }

  return work_queue(OS_LPWORK, work, (worker_t)worker, argv, tick);
}

/****************************************************************************
 * Name: bl_os_clock_gettime_ms
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

uint64_t bl_os_clock_gettime_ms(void)
{
  struct timespec ts;
  clock_systime_timespec(&ts);
  return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/****************************************************************************
 * Name: bl_os_get_tick
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

uint32_t bl_os_get_tick()
{
  uint32_t tick;

  tick = bl_os_clock_gettime_ms();

  return tick;
}

/****************************************************************************
 * Name: bl_os_isr_adpt_cb
 *
 * Description:
 *   Wi-Fi interrupt adapter callback function
 *
 * Input Parameters:
 *   arg - interrupt adapter private data
 *
 * Returned Value:
 *   0 on success
 *
 ****************************************************************************/

static int bl_os_isr_adpt_cb(int irq, void *context, void *arg)
{
  struct irq_adpt *adapter = (struct irq_adpt *)arg;

  adapter->func(adapter->arg);

  return 0;
}

/****************************************************************************
 * Name: bl_os_irq_attach
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_irq_attach(int32_t n, void *f, void *arg)
{
  int ret;
  struct irq_adpt *adapter;

  wlinfo("INFO: n=%ld f=%p arg=%p\n", n, f, arg);

  adapter = (struct irq_adpt *)kmm_malloc(sizeof(struct irq_adpt));

  if (!adapter)
    {
      DEBUGASSERT(0);
    }

  adapter->func = f;
  adapter->arg  = arg;

  ret = irq_attach(n, bl_os_isr_adpt_cb, (void *)adapter);

  if (ret != OK)
    {
      DEBUGASSERT(0);
    }
}

/****************************************************************************
 * Name: bl_os_irq_enable
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_irq_enable(int32_t n)
{
  up_enable_irq(n);
}

/****************************************************************************
 * Name: bl_os_irq_disable
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bl_os_irq_disable(int32_t n)
{
  up_disable_irq(n);
}

/****************************************************************************
 * Name: bl_os_mutex_create
 *
 * Description:
 *   Create mutex
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Mutex data pointer
 *
 ****************************************************************************/

void *bl_os_mutex_create(void)
{
  int ret;
  sem_t *sem;
  int tmp;

  tmp = sizeof(sem_t);
  sem = (sem_t *)kmm_malloc(tmp);
  if (!sem)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = nxsem_init(sem, 0, 1);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize sem error=%d\n", ret);
      kmm_free(sem);
      return NULL;
    }

  return sem;
}

/****************************************************************************
 * Name: bl_os_mutex_delete
 *
 * Description:
 *   Delete mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl_os_mutex_delete(void *mutex_data)
{
  sem_t *sem = (sem_t *)mutex_data;

  nxsem_destroy(sem);
  kmm_free(sem);
}

/****************************************************************************
 * Name: bl_os_mutex_lock
 *
 * Description:
 *   Lock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bl_os_mutex_lock(void *mutex_data)
{
  int ret;
  sem_t *sem = (sem_t *)mutex_data;

  ret = nxsem_wait(sem);
  if (ret)
    {
      wlerr("ERROR: Failed to wait sem\n");
    }

  return bl_os_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_mutex_unlock
 *
 * Description:
 *   Lock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bl_os_mutex_unlock(void *mutex_data)
{
  int ret;
  sem_t *sem = (sem_t *)mutex_data;

  ret = nxsem_post(sem);
  if (ret)
    {
      wlerr("ERROR: Failed to post sem error=%d\n", ret);
    }

  return bl_os_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_sem_create
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - No mean
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/

void *bl_os_sem_create(uint32_t init)
{
  int ret;
  sem_t *sem;
  int tmp;

  tmp = sizeof(sem_t);
  sem = (sem_t *)kmm_malloc(tmp);
  if (!sem)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = nxsem_init(sem, 0, init);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize sem error=%d\n", ret);
      kmm_free(sem);
      return NULL;
    }

  return sem;
}

/****************************************************************************
 * Name: bl_os_sem_delete
 *
 * Description:
 *   Delete semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl_os_sem_delete(void *semphr)
{
  sem_t *sem = (sem_t *)semphr;

  nxsem_destroy(sem);
  kmm_free(sem);
}

/****************************************************************************
 * Name: bl_os_sem_take
 *
 * Description:
 *   Wait semaphore within a certain period of time
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *   ticks  - Wait system ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bl_os_sem_take(void *semphr, uint32_t ticks)
{
  int ret;
  struct timespec timeout;
  sem_t *sem = (sem_t *)semphr;

  if (ticks == BL_OS_WAITING_FOREVER)
    {
      ret = nxsem_wait(sem);
      if (ret)
        {
          wlerr("ERROR: Failed to wait sem\n");
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to get time\n");
          return false;
        }

      if (ticks)
        {
          bl_os_update_time(&timeout, ticks);
        }

      ret = nxsem_timedwait(sem, &timeout);
      if (ret)
        {
          wlerr("ERROR: Failed to wait sem in %lu ticks\n", ticks);
        }
    }

  return bl_os_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_sem_give
 *
 * Description:
 *   Post semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bl_os_sem_give(void *semphr)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  ret = nxsem_post(sem);
  if (ret)
    {
      wlerr("ERROR: Failed to post sem error=%d\n", ret);
    }

  return bl_os_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_log_writev
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   tag    - log TAG, no mean here
 *   file   - file name
 *   line   - assert line
 *   format - format string
 *   args   - arguments list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bl_os_log_writev(uint32_t level,
                             const char *tag,
                             const char *file,
                             int line,
                             const char *format,
                             va_list args)
{
  switch (level)
    {
    case LOG_LEVEL_ERROR:
      {
        vsyslog(LOG_ERR, format, args);
        break;
      }

    case LOG_LEVEL_WARN:
      {
        vsyslog(LOG_WARNING, format, args);
        break;
      }

    case LOG_LEVEL_INFO:
      {
        vsyslog(LOG_INFO, format, args);
        break;
      }
    }
}

/****************************************************************************
 * Name: bl_os_log_write
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   file   - file name
 *   line   - assert line
 *   tag    - log TAG, no mean here
 *   format - format string
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl_os_log_write(uint32_t level,
                     const char *tag,
                     const char *file,
                     int line,
                     const char *format,
                     ...)
{
  if (&_wifi_log_flag)
    {
      va_list list;
      va_start(list, format);
      bl_os_log_writev(level, tag, NULL, 0, format, list);
      va_end(list);
    }
}
#endif
