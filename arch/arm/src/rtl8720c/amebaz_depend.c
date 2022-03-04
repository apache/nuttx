/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_depend.c
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

#include "amebaz_depend.h"
#include <nuttx/mqueue.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* stdio.h Wrapper Start */

int __wrap_printf(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vsyslog(LOG_INFO, fmt, &ap);
  va_end(ap);
  return 0;
}

/* stdio.h Wrapper End */

static int uxcriticalnesting = 0;

/* Critical Operation Start */

void save_and_cli(void)
{
  enter_critical_section();
  uxcriticalnesting++;
}

void restore_flags(void)
{
  ASSERT(uxcriticalnesting);
  uxcriticalnesting--;
  if (uxcriticalnesting == 0)
    {
      leave_critical_section(0);
    }
}

void rtw_enter_critical(void **plock, unsigned long *pirql)
{
  save_and_cli();
}

void rtw_exit_critical(void **plock, unsigned long *pirql)
{
  restore_flags();
}

void rtw_enter_critical_from_isr(void **plock, unsigned long *pirql)
{
  save_and_cli();
}

void rtw_exit_critical_from_isr(void **plock, unsigned long *pirql)
{
  restore_flags();
}

/* Critical Operation End */

/* arpa/inet.h Wrapper Start */

uint16_t _htons(uint16_t n)
{
  return htons(n);
}

uint16_t _ntohs(uint16_t n)
{
  return _htons(n);
}

/* arpa/inet.h Wrapper End */

/* stdlib.h Wrapper Start */

uint8_t *rtw_vmalloc(uint32_t n)
{
  return malloc(n);
}

uint8_t *rtw_zvmalloc(uint32_t n)
{
  return calloc(1, n);
}

void rtw_vmfree(uint8_t *pbuf, uint32_t n)
{
  return free(pbuf);
}

uint8_t *rtw_malloc(uint32_t n)
{
  return rtw_vmalloc(n);
}

uint8_t *rtw_zmalloc(uint32_t n)
{
  return rtw_zvmalloc(n);
}

void rtw_mfree(uint8_t *pbuf, uint32_t n)
{
  return rtw_vmfree(pbuf, n);
}

/* stdlib.h Wrapper End */

/* string.h Wrapper Start */

void rtw_memcpy(void *dst, void *src, uint32_t n)
{
  memcpy(dst, src, n);
}

int rtw_memcmp(void *dst, void *src, uint32_t n)
{
  return memcmp(dst, src, n) ? 0 : 1;
}

void rtw_memset(void *pbuf, int c, uint32_t n)
{
  memset(pbuf, c, n);
}

/* string.h Wrapper End */

/* Semaphore Start */

void rtw_init_sema(void **sema, int init_val)
{
  sem_t *_sema;
  _sema = calloc(1, sizeof(sem_t));
  if (!_sema)
    {
      return;
    }

  if (sem_init(_sema, 0, init_val))
    {
      free(_sema);
      return;
    }

  *sema = _sema;
}

void rtw_free_sema(void **sema)
{
  sem_destroy(*sema);
  free(*sema);
  *sema = NULL;
}

void rtw_up_sema(void **sema)
{
  sem_post(*sema);
}

void rtw_up_sema_from_isr(void **sema)
{
  rtw_up_sema(sema);
}

uint32_t rtw_down_timeout_sema(void **sema, uint32_t timeout)
{
  struct timespec abstime;
  int ret;
  if (timeout == 0xffffffff)
    {
      ret = sem_wait(*sema);
    }

  else
    {
      clock_gettime(CLOCK_REALTIME, &abstime);
      abstime.tv_sec += timeout / 1000;
      abstime.tv_nsec += (timeout % 1000) * 1000 * 1000;
      if (abstime.tv_nsec >= (1000 * 1000000))
        {
          abstime.tv_sec += 1;
          abstime.tv_nsec -= (1000 * 1000000);
        }

      ret = sem_timedwait(*sema, &abstime);
    }

  return !ret;
}

uint32_t rtw_down_sema(void **sema)
{
  return rtw_down_timeout_sema(sema, 0xffffffff);
}

/* Semaphore End */

/* MUTual EXclusion Start */

void rtw_mutex_init(void **pmutex)
{
  rtw_init_sema(pmutex, 1);
}

void rtw_mutex_free(void **pmutex)
{
  rtw_free_sema(pmutex);
}

void rtw_mutex_put(void **pmutex)
{
  rtw_up_sema(pmutex);
}

void rtw_mutex_get(void **pmutex)
{
  rtw_down_sema(pmutex);
}

int rtw_mutex_get_timeout(void **pmutex, uint32_t ms)
{
  return rtw_down_timeout_sema(pmutex, ms);
}

int rtw_enter_critical_mutex(void **pmutex, unsigned long *pirql)
{
  return rtw_down_sema(pmutex);
}

void rtw_exit_critical_mutex(void **pmutex, unsigned long *pirql)
{
  rtw_up_sema(pmutex);
}

/* MUTual EXclusion End */

/* Spinlocks Start */

void rtw_spinlock_init(void **plock)
{
  rtw_init_sema(plock, 1);
}

void rtw_spinlock_free(void **plock)
{
  rtw_free_sema(plock);
}

void rtw_spin_lock(void **plock)
{
  rtw_down_sema(plock);
}

void rtw_spin_unlock(void **plock)
{
  rtw_up_sema(plock);
}

void rtw_spinlock_irqsave(void **plock, unsigned long *pirql)
{
  rtw_spin_lock(plock);
}

void rtw_spinunlock_irqsave(void **plock, unsigned long *pirql)
{
  rtw_spin_unlock(plock);
}

void rtw_enter_critical_bh(void **plock, unsigned long *pirql)
{
  rtw_spin_lock(plock);
}

void rtw_exit_critical_bh(void **plock, unsigned long *pirql)
{
  rtw_spin_unlock(plock);
}

/* Spinlocks End */

/* mqueue.h Wrapper Start */

int rtw_init_xqueue(void **queue,
                    const char *name, uint32_t size, uint32_t len)
{
  struct mq_attr attr;
  struct file *mq;
  int ret;
  mq = malloc(sizeof(struct file));
  if (!mq)
    {
      return -ENOMEM;
    }

  attr.mq_maxmsg = len;
  attr.mq_msgsize = size;
  attr.mq_curmsgs = 0;
  attr.mq_flags = 0;
  ret = file_mq_open(mq, name, O_RDWR | O_CREAT, 0644, &attr);
  if (ret < 0)
    {
      free(mq);
      return -ENOMEM;
    }

  *queue = mq;
  return 0;
}

int rtw_push_to_xqueue(void **queue, void *message, uint32_t timeout_ms)
{
  struct file *mq = *queue;
  struct mq_attr attr;
  file_mq_getattr(mq, &attr);
  return file_mq_send(mq, message, attr.mq_msgsize, 1);
}

int rtw_pop_from_xqueue(void **queue, void *message, uint32_t timeout_ms)
{
  struct file *mq = *queue;
  struct mq_attr attr;
  unsigned int prio;
  file_mq_getattr(mq, &attr);
  return !file_mq_receive(mq, message, attr.mq_msgsize, &prio);
}

int rtw_deinit_xqueue(void **queue)
{
  struct file *mq = *queue;
  int ret;
  ret = file_mq_close(mq);
  if (ret >= 0)
    {
      free(mq);
    }

  return ret;
}

/* mqueue.h Wrapper End */

/* time.h Wrapper Start */

uint32_t rtw_get_current_time(void)
{
  return clock();
}

uint32_t rtw_systime_to_ms(uint32_t systime)
{
  return TICK2MSEC(systime);
}

uint32_t rtw_systime_to_sec(uint32_t systime)
{
  return TICK2SEC(systime);
}

uint32_t rtw_ms_to_systime(uint32_t ms)
{
  return MSEC2TICK(ms);
}

uint32_t rtw_sec_to_systime(uint32_t sec)
{
  return SEC2TICK(sec);
}

void rtw_yield_os(void)
{
  sched_yield();
}

void rtw_usleep_os(int us)
{
  usleep(us);
}

void rtw_msleep_os(int ms)
{
  rtw_usleep_os(ms * 1000);
}

void rtw_mdelay_os(int ms)
{
  rtw_msleep_os(ms);
}

void rtw_udelay_os(int us)
{
  rtw_usleep_os(us);
}

int32_t rtw_get_passing_time_ms(uint32_t start)
{
  return rtw_systime_to_ms(rtw_get_current_time() - start);
}

int32_t rtw_get_time_interval_ms(uint32_t start, uint32_t end)
{
  return rtw_systime_to_ms(end - start);
}

/* time.h Wrapper End */

/* Atomic Operation Start */

void ATOMIC_SET(atomic_t *v, int i)
{
  v->counter = i;
}

int ATOMIC_READ(atomic_t *v)
{
  return v->counter;
}

void ATOMIC_ADD(atomic_t *v, int i)
{
  save_and_cli();
  v->counter += i;
  restore_flags();
}

void ATOMIC_SUB(atomic_t *v, int i)
{
  save_and_cli();
  v->counter -= i;
  restore_flags();
}

void ATOMIC_INC(atomic_t *v)
{
  ATOMIC_ADD(v, 1);
}

void ATOMIC_DEC(atomic_t *v)
{
  ATOMIC_SUB(v, 1);
}

int ATOMIC_ADD_RETURN(atomic_t *v, int i)
{
  int temp;
  save_and_cli();
  temp = v->counter;
  temp += i;
  v->counter = temp;
  restore_flags();
  return temp;
}

int ATOMIC_SUB_RETURN(atomic_t *v, int i)
{
  int temp;
  save_and_cli();
  temp = v->counter;
  temp -= i;
  v->counter = temp;
  restore_flags();
  return temp;
}

int ATOMIC_INC_RETURN(atomic_t *v)
{
  return ATOMIC_ADD_RETURN(v, 1);
}

int ATOMIC_DEC_RETURN(atomic_t *v)
{
  return ATOMIC_SUB_RETURN(v, 1);
}

int ATOMIC_DEC_AND_TEST(atomic_t *v)
{
  return ATOMIC_DEC_RETURN(v) == 0;
}

/* Atomic Operation End */

/* stdlib.h Wrapper Start */

static unsigned int __div64_32(uint64_t *n, unsigned int base)
{
  uint64_t rem = *n;
  uint64_t b = base;
  uint64_t res;
  uint64_t d = 1;
  unsigned int high = rem >> 32;
  res = 0;
  if (high >= base)
    {
      high /= base;
      res = (uint64_t) high << 32;
      rem -= (uint64_t)(high * base) << 32;
    }

  while ((uint64_t)b > 0 && b < rem)
    {
      b = b + b;
      d = d + d;
    }

  do
    {
      if (rem >= b)
        {
          rem -= b;
          res += d;
        }

      b >>= 1;
      d >>= 1;
    }
  while (d);
  *n = res;
  return rem;
}

uint64_t rtw_modular64(uint64_t x, uint64_t y)
{
  unsigned int __base = (y);
  unsigned int __rem;
  if (((x) >> 32) == 0)
    {
      __rem = (unsigned int)(x) % __base;
      (x) = (unsigned int)(x) / __base;
    }

  else
    {
      __rem = __div64_32(&(x), __base);
    }

  return __rem;
}

static int arc4random(void)
{
  uint32_t res = rtw_get_current_time();
  static unsigned long rtw_seed = 0xdeadb00b;
  rtw_seed = ((rtw_seed & 0x007f00ff) << 7) ^
             ((rtw_seed & 0x0f80ff00) >> 8) ^
             (res << 13) ^ (res >> 9);
  return (int)rtw_seed;
}

int rtw_get_random_bytes(void *dst, uint32_t size)
{
  unsigned int ranbuf;
  unsigned int *lp;
  int i;
  int count;
  count = size / sizeof(unsigned int);
  lp = (unsigned int *)dst;
  for (i = 0; i < count; i++)
    {
      lp[i] = arc4random();
      size -= sizeof(unsigned int);
    }

  if (size > 0)
    {
      ranbuf = arc4random();
      memcpy(&lp[i], &ranbuf, size);
    }

  return 0;
}

/* stdlib.h Wrapper End */

/* Thread Wrapper Start */

static int nuttx_task_hook(int argc, FAR char *argv[])
{
  struct task_struct *task;
  struct nthread_wrapper *wrap;
  task = (FAR struct task_struct *)
         ((uintptr_t)strtoul(argv[1], NULL, 0));
  if (!task || !task->priv)
    {
      return 0;
    }

  wrap = task->priv;
  if (wrap->func)
    {
      wrap->func(wrap->thctx);
    }

  return 0;
}

int rtw_create_task(struct task_struct *task, const char *name,
                    uint32_t stack_size, uint32_t priority,
                    thread_func_t func, void *thctx)
{
  struct nthread_wrapper *wrap;
  char *argv[2];
  char arg1[16];
  int pid;
  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)task);
  argv[0] = arg1;
  argv[1] = NULL;
  wrap = malloc(sizeof(*wrap));
  if (!wrap)
    {
      return -ENOMEM;
    }

  wrap->func = func;
  wrap->thctx = thctx;
  task->name = name;
  task->priv = wrap;
  if (!strcmp(name, "rtw_recv_tasklet"))
    {
      stack_size =
        CONFIG_IEEE80211_REALTEK_AMEBAZ_RECV_STACKSIZE / sizeof(int);
    }

  pid = kthread_create(name,
                       SCHED_PRIORITY_DEFAULT + priority,
                       stack_size * sizeof(int),
                       nuttx_task_hook, argv);
  if (pid < 0)
    {
      free(wrap);
      return pid;
    }

  wrap->pid = pid;
  return 1;
}

void rtw_delete_task(struct task_struct *task)
{
  struct nthread_wrapper *wrap = task->priv;
  if (kill(wrap->pid, SIGKILL))
    {
      return;
    }

  free(wrap);
  task->priv = NULL;
}

void rtw_set_priority_task(struct task_struct *task,
                           unsigned int newpriority)
{
}

int rtw_get_priority_task(struct task_struct *task)
{
}

void rtw_suspend_task(struct task_struct *task)
{
}

void rtw_resume_task(struct task_struct *task)
{
}

/* Thread Wrapper End */

/* Timer Wrapper Start */

void *rtw_timer_create(const signed char *pctimername,
                      unsigned long xtimerperiodinticks,
                      uint32_t uxautoreload,
                      void *pvtimerid,
                      thread_func_t pxcallbackfunction)
{
  struct ntimer_wrapper *wrap;
  wrap = calloc(1, sizeof(*wrap));
  if (!wrap)
    {
      return NULL;
    }

  wrap->callback = pxcallbackfunction;
  return wrap;
}

uint32_t rtw_timer_stop(void *xtimer,
                       unsigned long xblocktime)
{
  struct ntimer_wrapper *wrap = xtimer;
  if (!work_available(&wrap->work))
    {
      work_cancel(LPWORK, &wrap->work);
    }

  return 1;
}

uint32_t rtw_timer_delete(void *xtimer,
                         unsigned long xblocktime)
{
  struct ntimer_wrapper *wrap = xtimer;
  rtw_timer_stop(xtimer, xblocktime);
  free(wrap);
  return 1;
}

uint32_t rtw_timer_is_timer_active(void *xtimer)
{
  struct ntimer_wrapper *wrap = xtimer;
  return !work_available(&wrap->work);
}

uint32_t rtw_timer_change_period(void *xtimer,
                               unsigned long xnewperiod,
                               unsigned long xblocktime)
{
  struct ntimer_wrapper *wrap = xtimer;
  if (work_available(&wrap->work))
    {
      work_queue(LPWORK, &wrap->work, wrap->callback, wrap, xnewperiod);
    }

  return 1;
}

void *rtw_timer_get_id(void *xtimer)
{
  return xtimer;
}

uint32_t rtw_timer_start(void *xtimer, unsigned long xblocktime)
{
  return rtw_timer_change_period(xtimer, 0, xblocktime);
}

uint32_t rtw_timer_start_from_isr(void *xtimer,
                                         long *pxhigherprioritytaskwoken)
{
  return rtw_timer_start(xtimer, 0);
}

uint32_t rtw_timer_stop_from_isr(void *xtimer,
                                        long *pxhigherprioritytaskwoken)
{
  return rtw_timer_stop(xtimer, 0);
}

uint32_t rtw_timer_reset_from_isr(void *xtimer,
                                         long *pxhigherprioritytaskwoken)
{
  return rtw_timer_start(xtimer, 0);
}

uint32_t rtw_timer_change_period_from_isr(void *xtimer,
                                           unsigned long xnewperiod,
                                           long *pxhigherprioritytaskwoken)
{
  return rtw_timer_change_period(xtimer, xnewperiod, 0);
}

uint32_t rtw_timer_reset(void *xtimer, unsigned long xblocktime)
{
  return rtw_timer_start(xtimer, 0);
}

/* Timer Wrapper End */

/* List Wrapper Start */

static void _list_add(struct list_head *newitem,
                      struct list_head *prev,
                      struct list_head *next)
{
  next->prev = newitem;
  newitem->next = next;
  newitem->prev = prev;
  prev->next = newitem;
}

static void list_add(struct list_head *newitem, struct list_head *head)
{
  _list_add(newitem, head, head->next);
}

static void list_add_tail(struct list_head *newitem, struct list_head *head)
{
  _list_add(newitem, head->prev, head);
}

static void list_del(struct list_head *prev, struct list_head *next)
{
  next->prev = prev;
  prev->next = next;
}

void rtw_list_insert_head(struct list_head *plist, struct list_head *phead)
{
  list_add(plist, phead);
}

void rtw_list_insert_tail(struct list_head *plist, struct list_head *phead)
{
  list_add_tail(plist, phead);
}

void rtw_list_delete(struct list_head *plist)
{
  list_del(plist->prev, plist->next);
  plist->next = plist->prev = plist;
}

void rtw_init_listhead(struct list_head *list)
{
  list->next = list->prev = list;
}

uint32_t rtw_is_list_empty(struct list_head *phead)
{
  return phead->next == phead;
}

/* List Wrapper End */

/* Queue Wrapper Start */

void rtw_init_queue(_queue *pqueue)
{
  rtw_init_listhead(&(pqueue->queue));
  rtw_spinlock_init(&(pqueue->lock));
}

uint32_t rtw_queue_empty(_queue *pqueue)
{
  return (rtw_is_list_empty(&(pqueue->queue)));
}

uint32_t rtw_end_of_queue_search(struct list_head *head,
                                 struct list_head *plist)
{
  return (head == plist);
}

/* Queue Wrapper End */

/* Device lock Wrapper Start */

static uint32_t mutex_init;
static void *device_mutex[5];
static void device_mutex_init(uint32_t device)
{
  irqstate_t status;
  if (!(mutex_init & (1 << device)))
    {
      status = enter_critical_section();
      if (!(mutex_init & (1 << device)))
        {
          rtw_mutex_init(&device_mutex[device]);
          mutex_init |= (1 << device);
        }

      leave_critical_section(status);
    }
}

void device_mutex_lock(uint32_t device)
{
  device_mutex_init(device);
  rtw_mutex_get(&device_mutex[device]);
}

void device_mutex_unlock(uint32_t device)
{
  device_mutex_init(device);
  rtw_mutex_put(&device_mutex[device]);
}

/* Device lock Wrapper End */

/* malloc.h Wrapper Start */

uint32_t rtw_get_free_heap_size(void)
{
  struct mallinfo mem;
  mem = mallinfo();
  return mem.arena;
}

/* malloc.h Wrapper End */

/* Unnecessary Start */

void      init_mem_monitor(struct list_head *pmem_table,
                           int *used_num)
{
}

void      deinit_mem_monitor(struct list_head *pmem_table,
                             int *used_num)
{
}

int       rtw_netif_queue_stopped(void *pnetdev)
{
  return 0;
}

void      rtw_netif_wake_queue(void *pnetdev)
{
}

void      rtw_netif_start_queue(void *pnetdev)
{
}

void      rtw_netif_stop_queue(void *pnetdev)
{
}

void      flush_signals_thread(void)
{
}

void      rtw_wakeup_task(struct task_struct *task)
{
}

void      rtw_thread_enter(char *name)
{
}

void      rtw_thread_exit(void)
{
}

uint8_t   rtw_get_scheduler_state(void)
{
  return 1;  /* OS_SCHEDULER_RUNNING */
}

long      xtask_get_scheduler_state(void)
{
  return 2;  /* taskSCHEDULER_RUNNING */
}

void      rtw_cpu_lock(void)
{
}

void      rtw_cpu_unlock(void)
{
}

void      rtw_create_secure_context(uint32_t n)
{
}

void      rtw_acquire_wakelock(void)
{
}

void      rtw_release_wakelock(void)
{
}

void      rtw_wakelock_timeout(uint32_t ms)
{
}

void      cli(void)
{
}

uint32_t  xtask_get_tick_count(void)
{
  return rtw_get_current_time();
}

char      *pctask_get_name(void *xtasktoquery)
{
  return NULL;
}

/* Unnecessary End */

/* Legacy Start */

const struct osdep_service_ops osdep_service =
{
  .rtw_vmalloc                  = rtw_vmalloc,
  .rtw_zvmalloc                 = rtw_zvmalloc,
  .rtw_vmfree                   = rtw_vmfree,
  .rtw_malloc                   = rtw_malloc,
  .rtw_zmalloc                  = rtw_zmalloc,
  .rtw_mfree                    = rtw_mfree,
  .rtw_memcpy                   = rtw_memcpy,
  .rtw_memcmp                   = rtw_memcmp,
  .rtw_memset                   = rtw_memset,
  .rtw_init_sema                = rtw_init_sema,
  .rtw_free_sema                = rtw_free_sema,
  .rtw_up_sema                  = rtw_up_sema,
  .rtw_up_sema_from_isr         = rtw_up_sema_from_isr,
  .rtw_down_timeout_sema        = rtw_down_timeout_sema,
  .rtw_mutex_init               = rtw_mutex_init,
  .rtw_mutex_free               = rtw_mutex_free,
  .rtw_mutex_get                = rtw_mutex_get,
  .rtw_mutex_get_timeout        = rtw_mutex_get_timeout,
  .rtw_mutex_put                = rtw_mutex_put,
  .rtw_enter_critical           = rtw_enter_critical,
  .rtw_exit_critical            = rtw_exit_critical,
  .rtw_enter_critical_from_isr  = rtw_enter_critical,
  .rtw_exit_critical_from_isr   = rtw_exit_critical,
  .rtw_enter_critical_bh        = NULL,
  .rtw_exit_critical_bh         = NULL,
  .rtw_enter_critical_mutex     = rtw_enter_critical_mutex,
  .rtw_exit_critical_mutex      = rtw_exit_critical_mutex,
  .rtw_cpu_lock                 = rtw_cpu_lock,
  .rtw_cpu_unlock               = rtw_cpu_unlock,
  .rtw_spinlock_init            = rtw_spinlock_init,
  .rtw_spinlock_free            = rtw_spinlock_free,
  .rtw_spin_lock                = rtw_spin_lock,
  .rtw_spin_unlock              = rtw_spin_unlock,
  .rtw_spinlock_irqsave         = rtw_spinlock_irqsave,
  .rtw_spinunlock_irqsave       = rtw_spinunlock_irqsave,
  .rtw_init_xqueue              = rtw_init_xqueue,
  .rtw_push_to_xqueue           = rtw_push_to_xqueue,
  .rtw_pop_from_xqueue          = rtw_pop_from_xqueue,
  .rtw_deinit_xqueue            = rtw_deinit_xqueue,
  .rtw_get_current_time         = rtw_get_current_time,
  .rtw_systime_to_ms            = rtw_systime_to_ms,
  .rtw_systime_to_sec           = rtw_systime_to_sec,
  .rtw_ms_to_systime            = rtw_ms_to_systime,
  .rtw_sec_to_systime           = rtw_sec_to_systime,
  .rtw_msleep_os                = rtw_msleep_os,
  .rtw_usleep_os                = rtw_usleep_os,
  .rtw_mdelay_os                = rtw_msleep_os,
  .rtw_udelay_os                = rtw_usleep_os,
  .rtw_yield_os                 = rtw_yield_os,
  .ATOMIC_SET                   = ATOMIC_SET,
  .ATOMIC_READ                  = ATOMIC_READ,
  .ATOMIC_ADD                   = ATOMIC_ADD,
  .ATOMIC_SUB                   = ATOMIC_SUB,
  .ATOMIC_INC                   = ATOMIC_INC,
  .ATOMIC_DEC                   = ATOMIC_DEC,
  .ATOMIC_ADD_RETURN            = ATOMIC_ADD_RETURN,
  .ATOMIC_SUB_RETURN            = ATOMIC_SUB_RETURN,
  .ATOMIC_INC_RETURN            = ATOMIC_INC_RETURN,
  .ATOMIC_DEC_RETURN            = ATOMIC_DEC_RETURN,
  .rtw_modular64                = rtw_modular64,
  .rtw_get_random_bytes         = rtw_get_random_bytes,
  .rtw_get_free_heap_size          = rtw_get_free_heap_size,
  .rtw_create_task              = rtw_create_task,
  .rtw_delete_task              = rtw_delete_task,
  .rtw_wakeup_task              = NULL,
  .rtw_thread_enter             = rtw_thread_enter,
  .rtw_thread_exit              = rtw_thread_exit,
  .rtw_timer_create              = rtw_timer_create,
  .rtw_timer_delete              = rtw_timer_delete,
  .rtw_timer_is_timer_active       = rtw_timer_is_timer_active,
  .rtw_timer_stop                = rtw_timer_stop,
  .rtw_timer_change_period        = rtw_timer_change_period,
  .rtw_timer_get_id               = rtw_timer_get_id,
  .rtw_timer_start               = rtw_timer_start,
  .rtw_timer_start_from_isr        = rtw_timer_start_from_isr,
  .rtw_timer_stop_from_isr         = rtw_timer_stop_from_isr,
  .rtw_timer_reset_from_isr        = rtw_timer_reset_from_isr,
  .rtw_timer_change_period_from_isr = rtw_timer_change_period_from_isr,
  .rtw_timer_reset               = rtw_timer_reset,
  .rtw_acquire_wakelock         = rtw_acquire_wakelock,
  .rtw_release_wakelock         = rtw_release_wakelock,
  .rtw_wakelock_timeout         = rtw_wakelock_timeout,
  .rtw_get_scheduler_state      = rtw_get_scheduler_state,
  .rtw_create_secure_context    = rtw_create_secure_context,
};

/* Legacy End */

