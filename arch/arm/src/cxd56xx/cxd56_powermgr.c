/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_powermgr.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#include <debug.h>
#include <errno.h>
#include <sched.h>
#include <mqueue.h>
#include <fcntl.h>
#include <queue.h>

#include <arch/chip/pm.h>

#include "arm_internal.h"
#include "arm_arch.h"
#include "cxd56_powermgr.h"
#include "cxd56_icc.h"
#include "cxd56_pmic.h"
#include "chip.h"
#include "hardware/cxd5602_backupmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INTC_REG_INV(n) (CXD56_INTC_BASE + 0x20 + ((n) << 2))
#define INTC_REG_EN(n)  (CXD56_INTC_BASE + 0x10 + ((n) << 2))

/* bootmask control */

#define NON_MASKABLE_BOOTMASK   (PM_BOOT_POR_NORMAL | \
                                 PM_BOOT_POR_DEADBATT | \
                                 PM_BOOT_WDT_REBOOT | \
                                 PM_BOOT_WDT_RESET | \
                                 PM_BOOT_DEEP_USB_ATTACH | \
                                 PM_BOOT_DEEP_OTHERS)
#define DEEP_PROHIBIT_BOOTMASK  (PM_BOOT_DEEP_WKUPS | \
                                 PM_BOOT_DEEP_RTC)
#define CONFIG_INT_WKUP_REG     (0x38)
#define WKUPL_ENABLE            (0x2)
#define WKUPL_DISABLE           (0x0)

#define MSGID_BOOT          0
#define MSGID_FREQLOCK      1
#define MSGID_CLK_CHG_START 2
#define MSGID_CLK_CHG_END   3
#define MSGID_GET_CLK       4
#define MSGID_HOT_SLEEP     5
#define MSGID_RESUME        6

#define MQMSG_CLK_CHG_START 0
#define MQMSG_CLK_CHG_END   1
#define MQMSG_HOT_SLEEP     2

#define CXD56_PM_SYS_CPU  (0)
#define CXD56_PM_SYS_APP  (2)

#define PM_CPUFREQLOCK_FLAG_INITIALIZED (0x8000)

#ifdef CONFIG_CXD56_PM_DEBUG
#  define pmerr(format, ...)  _err(format, ##__VA_ARGS__)
#  define pmwarn(format, ...) _warn(format, ##__VA_ARGS__)
#  define pminfo(format, ...) _info(format, ##__VA_ARGS__)
#else
#  define pmerr(x...)
#  define pmwarn(x...)
#  define pminfo(x...)
#endif

void up_cpuctxload(void);
int cxd56_cpu_context_sleep(void);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_pm_target_id_s
{
  uint32_t sysiop;
  uint32_t app;
  uint32_t pmu;
  uint32_t hostif;
  uint32_t scu;
  uint32_t gps;
};

struct cxd56_pm_hotsleep_info_s
{
  int cpu_id;
  uint32_t entry;
  uint64_t requested_sleeptime;
  uint64_t sleep_starttime;
  uint64_t wakeuptime;
};

struct cxd56_pm_message_s
{
  uint8_t  mid;
  uint32_t data;
};

struct pm_cbentry_s
{
  struct dq_entry_s dq_entry;
  uint32_t          target;
  cxd56_pm_callback callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  cxd56_pm_do_callback(uint8_t id,
                                 FAR struct cxd56_pm_target_id_s *table);
static int  cxd56_pm_needcallback(uint32_t target,
                                  FAR struct cxd56_pm_target_id_s *table);
static void cxd56_pm_clkchange(struct cxd56_pm_message_s *message);
static int  cxd56_pm_semtake(FAR sem_t *id);
static void cxd56_pm_checkfreqlock(void);
static int  cxd56_pm_maintask(int argc, FAR char *argv[]);
#if defined(CONFIG_CXD56_HOT_SLEEP)
static void cxd56_pm_do_hotsleep(uint32_t idletime);
static void cxd56_pm_intc_suspend(void);
static void cxd56_pm_intc_resume(void);
#endif
static int cxd56_pmmsghandler(int cpuid, int protoid, uint32_t pdata,
                              uint32_t data, FAR void *userdata);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_pm_target_id_s g_target_id_table;
static mqd_t      g_queuedesc;
static sem_t      g_bootsync;
static sem_t      g_regcblock;
static sem_t      g_freqlock;
static sem_t      g_freqlockwait;
static dq_queue_t g_cbqueue;
static sq_queue_t g_freqlockqueue;
static sq_queue_t g_wakelockqueue;
static uint32_t   g_clockcange_start;
static int        g_freqlock_flag;

static struct pm_cpu_wakelock_s g_wlock =
  PM_CPUWAKELOCK_INIT(PM_CPUWAKELOCK_TAG('P', 'M', 0));

static int cxd56_pm_semtake(FAR sem_t *id)
{
  return nxsem_wait_uninterruptible(id);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int cxd56_pm_needcallback(uint32_t target,
                                 FAR struct cxd56_pm_target_id_s *table)
{
  uint32_t mask;

  switch (target & PM_DOMAIN_MASK)
    {
    case PM_DOMAIN_SYSIOP:
      mask = table->sysiop;
      break;
    case PM_DOMAIN_HOSTIF:
      mask = table->hostif;
      break;
    case PM_DOMAIN_PMU:
      mask = table->pmu;
      break;
    case PM_DOMAIN_SCU:
      mask = table->scu;
      break;
    case PM_DOMAIN_APP:
      mask = table->app;
      break;
    case PM_DOMAIN_GPS:
      mask = table->gps;
      break;
    default:
      return 0;
    }

  return mask & target;
}

static int cxd56_pmsendmsg(int mid, uint32_t data)
{
  iccmsg_t msg;

  msg.cpuid     = 0;
  msg.msgid     = 0; /* Power manager message does not used this field. */
  msg.protodata = mid;
  msg.data      = data;
  return cxd56_iccsend(CXD56_PROTO_PM, &msg, 0);
}

static int cxd56_pm_do_callback(uint8_t id,
                                struct cxd56_pm_target_id_s *targets)
{
  FAR struct pm_cbentry_s *entry;
  FAR dq_entry_t          *cur;
  FAR dq_entry_t          *last;
  int ret = 0;

  for (cur = dq_peek(&g_cbqueue); cur; cur = dq_next(cur))
    {
      entry = (FAR struct pm_cbentry_s *)cur;
      last = cur;
      if (cxd56_pm_needcallback(entry->target, targets))
        {
          ret = entry->callback(id);
          if (ret != 0)
            {
              break;
            }
        }
    }

  /* If one of the callbacks has been failed, then recovery call to
   * previously called entries.
   */

  if (ret != 0)
    {
      /* Replace callback ID to recovery */

      if (id == CXD56_PM_CALLBACK_ID_CLK_CHG_START)
        {
          id = CXD56_PM_CALLBACK_ID_CLK_CHG_END;
        }

      if (id == CXD56_PM_CALLBACK_ID_HOT_SLEEP)
        {
          id = CXD56_PM_CALLBACK_ID_HOT_BOOT;
        }

      for (cur = dq_peek(&g_cbqueue); cur != last; cur = dq_next(cur))
        {
          entry = (FAR struct pm_cbentry_s *)cur;
          if (cxd56_pm_needcallback(entry->target, targets))
            {
              entry->callback(id);
            }
        }
    }

  return ret;
}

static void cxd56_pm_clkchange(struct cxd56_pm_message_s *message)
{
  uint8_t id;
  int mid;
  int ret;

  switch (message->mid)
    {
    case MQMSG_CLK_CHG_START:
      id = CXD56_PM_CALLBACK_ID_CLK_CHG_START;
      mid = MSGID_CLK_CHG_START;
      g_clockcange_start = 1;
      break;
    case MQMSG_CLK_CHG_END:
      if (g_clockcange_start == 0)
        {
          return;
        }

      id = CXD56_PM_CALLBACK_ID_CLK_CHG_END;
      mid = MSGID_CLK_CHG_END;
      g_clockcange_start = 0;
      break;
    default:
      return;
    }

  cxd56_pm_semtake(&g_regcblock);

  ret = cxd56_pm_do_callback(id, &g_target_id_table);

  cxd56_pmsendmsg(mid, ret);

  nxsem_post(&g_regcblock);
}

static void cxd56_pm_checkfreqlock(void)
{
  FAR sq_entry_t *entry;
  FAR struct pm_cpu_freqlock_s *lock;
  int flag = PM_CPUFREQLOCK_FLAG_INITIALIZED;

  for (entry = sq_peek(&g_freqlockqueue); entry; entry = sq_next(entry))
    {
      lock = (FAR struct pm_cpu_freqlock_s *)entry;
      flag |= lock->flag & PM_CPUFREQLOCK_FLAG_LV;
      flag |= lock->flag & PM_CPUFREQLOCK_FLAG_HV;
    }

  if (g_freqlock_flag != flag)
    {
      g_freqlock_flag = flag;
      cxd56_pmsendmsg(MSGID_FREQLOCK, flag);
      cxd56_pm_semtake(&g_freqlockwait);
    }
}

#if defined(CONFIG_CXD56_HOT_SLEEP)
static void cxd56_pm_intc_suspend(void)
{
  int i;

  for (i = 0; i < 4; i++)
    {
      BKUP->irq_inv_map[i] = getreg32(INTC_REG_INV(i));
      BKUP->irq_wake_map[i] = getreg32(INTC_REG_EN(i));
      putreg32(0, INTC_REG_INV(i));
      putreg32(0, INTC_REG_EN(i));
    }
}

static void cxd56_pm_intc_resume(void)
{
  int i;

  for (i = 0; i < 4; i++)
    {
      putreg32(BKUP->irq_inv_map[i], INTC_REG_INV(i));
      putreg32(BKUP->irq_wake_map[i], INTC_REG_EN(i));
    }
}

static void cxd56_pm_do_hotsleep(uint32_t idletime)
{
  irqstate_t flags;
  uint64_t time;
  uint32_t tick;
  struct cxd56_pm_hotsleep_info_s info;
  struct cxd56_pm_target_id_s table;
  iccmsg_t msg;
  int ret;

  if (up_pm_count_acquire_wakelock() != 0)
    {
      return;
    }

  table.sysiop = PM_DOMAIN_SYSIOP | ~PM_DOMAIN_MASK;
  table.app = PM_DOMAIN_APP | ~PM_DOMAIN_MASK;
  table.pmu = PM_DOMAIN_PMU | ~PM_DOMAIN_MASK;
  table.hostif = PM_DOMAIN_HOSTIF | ~PM_DOMAIN_MASK;
  table.scu = PM_DOMAIN_SCU | ~PM_DOMAIN_MASK;
  table.gps = PM_DOMAIN_GPS | ~PM_DOMAIN_MASK;

  ret = cxd56_pm_do_callback(CXD56_PM_CALLBACK_ID_HOT_SLEEP, &table);
  if (ret != 0)
    {
      return;
    }

  flags = enter_critical_section();

  info.cpu_id = CXD56_PM_SYS_APP;
  info.entry = (uint32_t)up_cpuctxload | 0x1;
  info.requested_sleeptime = idletime;
  info.sleep_starttime = 0;
  info.wakeuptime = 0;

  cxd56_pmsendmsg(MSGID_HOT_SLEEP, (uint32_t)(uintptr_t)&info);

  cxd56_pm_intc_suspend();
  cxd56_cpu_context_sleep();

  leave_critical_section(flags);

  if (info.sleep_starttime != 0)
    {
      time = info.wakeuptime - info.sleep_starttime;
      tick = (time * USEC_PER_MSEC) / CONFIG_USEC_PER_TICK;
      sched_process_timer_skip(tick);
    }

  cxd56_pm_do_callback(CXD56_PM_CALLBACK_ID_HOT_BOOT, &table);

  cxd56_pm_intc_resume();

  cxd56_pmsendmsg(MSGID_RESUME, 0);
}
#endif

static int cxd56_pm_maintask(int argc, FAR char *argv[])
{
  int size;
  struct cxd56_pm_message_s message;
  struct mq_attr attr;

  attr.mq_maxmsg  = 8;
  attr.mq_msgsize = sizeof(struct cxd56_pm_message_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  g_queuedesc = mq_open("cxd56_pm_message", O_RDWR | O_CREAT, 0666, &attr);
  DEBUGASSERT((int)g_queuedesc != ERROR);
  if (g_queuedesc < 0)
    {
      pmerr("Failed to create message queue\n");
    }

  /* Register power manager messaging protocol handler. */

  cxd56_iccinit(CXD56_PROTO_PM);

  cxd56_iccregisterhandler(CXD56_PROTO_PM, cxd56_pmmsghandler, NULL);

  /* Notify that cxd56_pm_maintask is ready */

  nxsem_post(&g_bootsync);

  while (1)
    {
      size = mq_receive(g_queuedesc, (FAR char *)&message, sizeof(message),
                        NULL);
      if (size == sizeof(message))
        {
          switch (message.mid)
            {
            case MQMSG_CLK_CHG_START:
            case MQMSG_CLK_CHG_END:
              cxd56_pm_clkchange(&message);
              break;
            case MQMSG_HOT_SLEEP:
#if defined(CONFIG_CXD56_HOT_SLEEP)
              cxd56_pm_do_hotsleep(message.data);
#endif
              break;
            default:
              break;
            }
        }
    }

  return EXIT_SUCCESS;
}

FAR void *cxd56_pm_register_callback(uint32_t target,
                                     cxd56_pm_callback callback)
{
  struct pm_cbentry_s *entry = NULL;

  cxd56_pm_semtake(&g_regcblock);

  entry = (struct pm_cbentry_s *)kmm_malloc(sizeof(struct pm_cbentry_s));
  if (entry == NULL)
    {
      nxsem_post(&g_regcblock);
      return NULL;
    }

  entry->target = target;
  entry->callback = callback;

  dq_addlast((FAR dq_entry_t *)entry, &g_cbqueue);
  nxsem_post(&g_regcblock);

  return (void *)entry;
}

void cxd56_pm_unregister_callback(FAR void *handle)
{
  cxd56_pm_semtake(&g_regcblock);

  dq_rem((FAR dq_entry_t *)handle, &g_cbqueue);
  kmm_free(handle);

  nxsem_post(&g_regcblock);
}

static int cxd56_pmmsghandler(int cpuid, int protoid, uint32_t pdata,
                              uint32_t data, FAR void *userdata)
{
  uint32_t msgid;
  struct cxd56_pm_message_s message;
  int ret;

  msgid = pdata;

  if (msgid == MSGID_CLK_CHG_START)
    {
      message.mid = MQMSG_CLK_CHG_START;
      ret = mq_send(g_queuedesc, (FAR const char *)&message, sizeof(message),
                    CXD56_PM_MESSAGE_PRIO);
      if (ret < 0)
        {
          pmerr("ERR:mq_send(CLK_CHG_START)\n");
        }
    }
  else if (msgid == MSGID_CLK_CHG_END)
    {
      message.mid = MQMSG_CLK_CHG_END;
      ret = mq_send(g_queuedesc, (FAR const char *)&message, sizeof(message),
                    CXD56_PM_MESSAGE_PRIO);
      if (ret < 0)
        {
          pmerr("ERR:mq_send(CLK_CHG_END)\n");
        }
    }
  else if (msgid == MSGID_FREQLOCK)
    {
      nxsem_post(&g_freqlockwait);
    }
  else
    {
      pmerr("Unknown message %d\n", msgid);
    }

  return 0;
}

int cxd56_pm_bootup(void)
{
  /* BOOT indicate to M0P */

  cxd56_pmsendmsg(MSGID_BOOT, (uint32_t)(uintptr_t)&g_target_id_table);
  return OK;
}

/****************************************************************************
 * Name: up_pm_acquire_freqlock
 *
 * Description:
 *   Acquire the specified freqlock. If the higher freqlock is acquired, the
 *   system can clockup until it is released.
 *
 * Parameter:
 *   lock - the pointer of a wakelock variable
 *
 ****************************************************************************/

void up_pm_acquire_freqlock(struct pm_cpu_freqlock_s *lock)
{
  FAR sq_entry_t *entry;

  DEBUGASSERT(lock);

  up_pm_acquire_wakelock(&g_wlock);

  cxd56_pm_semtake(&g_freqlock);

  for (entry = sq_peek(&g_freqlockqueue); entry; entry = sq_next(entry))
    {
      if (entry == (struct sq_entry_s *)lock)
        {
          break;
        }
    }

  if (!entry)
    {
      sq_addlast((FAR sq_entry_t *)lock, &g_freqlockqueue);
      cxd56_pm_checkfreqlock();
    }

  lock->count++;

  nxsem_post(&g_freqlock);

  up_pm_release_wakelock(&g_wlock);
}

/****************************************************************************
 * Name: up_pm_release_freqlock
 *
 * Description:
 *   Release the specified freqlock. If the freqlock are released, the system
 *   can drop to the lower clock mode for power saving.
 *
 * Parameter:
 *   lock - the pointer of a freqlock variable
 *
 ****************************************************************************/

void up_pm_release_freqlock(struct pm_cpu_freqlock_s *lock)
{
  FAR sq_entry_t *entry;

  DEBUGASSERT(lock);

  up_pm_acquire_wakelock(&g_wlock);

  cxd56_pm_semtake(&g_freqlock);

  for (entry = sq_peek(&g_freqlockqueue); entry; entry = sq_next(entry))
    {
      if (entry == (struct sq_entry_s *)lock)
        {
          lock->count--;
          if (lock->count <= 0)
            {
              sq_rem(entry, &g_freqlockqueue);
              cxd56_pm_checkfreqlock();
            }
          break;
        }
    }

  nxsem_post(&g_freqlock);

  up_pm_release_wakelock(&g_wlock);
}

/****************************************************************************
 * Name: up_pm_get_freqlock_count
 *
 * Description:
 *   Get the locked count of the specified freqlock
 *
 * Parameter:
 *   lock - the pointer of a freqlock variable
 *
 * Return:
 *   the locked count of the specified freqlock
 *
 ****************************************************************************/

int up_pm_get_freqlock_count(struct pm_cpu_freqlock_s *lock)
{
  FAR sq_entry_t *entry;
  int count = 0;

  DEBUGASSERT(lock);

  cxd56_pm_semtake(&g_freqlock);

  for (entry = sq_peek(&g_freqlockqueue); entry; entry = sq_next(entry))
    {
      if (entry == (struct sq_entry_s *)lock)
        {
          count = lock->count;
          break;
        }
    }

  nxsem_post(&g_freqlock);
  return count;
}

/****************************************************************************
 * Name: up_pm_acquire_wakelock
 *
 * Description:
 *   Acquire the specified wakelock. If any wakelock is acquired, CPU can't
 *   enter to the hot sleep state.
 *
 * Parameter:
 *   lock - the pointer of a wakelock variable
 *
 ****************************************************************************/

void up_pm_acquire_wakelock(struct pm_cpu_wakelock_s *lock)
{
  irqstate_t flags;
  FAR sq_entry_t *entry;

  DEBUGASSERT(lock);

  flags = enter_critical_section();

  for (entry = sq_peek(&g_wakelockqueue); entry; entry = sq_next(entry))
    {
      if (entry == (struct sq_entry_s *)lock)
        {
          break;
        }
    }

  if (!entry)
    {
      sq_addlast((FAR sq_entry_t *)lock, &g_wakelockqueue);
    }

  lock->count++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_pm_release_wakelock
 *
 * Description:
 *   Release the specified wakelock. If all of the wakelock are released,
 *   CPU can enter to the hot sleep state.
 *
 * Parameter:
 *   lock - the pointer of a wakelock variable
 *
 ****************************************************************************/

void up_pm_release_wakelock(struct pm_cpu_wakelock_s *lock)
{
  irqstate_t flags;
  FAR sq_entry_t *entry;

  DEBUGASSERT(lock);

  flags = enter_critical_section();

  for (entry = sq_peek(&g_wakelockqueue); entry; entry = sq_next(entry))
    {
      if (entry == (struct sq_entry_s *)lock)
        {
          lock->count--;
          if (lock->count <= 0)
            {
              sq_rem(entry, &g_wakelockqueue);
            }
          break;
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_pm_count_acquire_wakelock
 *
 * Description:
 *   Count the total number of wakelock
 *
 * Return:
 *   the total number of wakelock
 *
 ****************************************************************************/

int up_pm_count_acquire_wakelock(void)
{
  irqstate_t flags;
  int num;

  flags = enter_critical_section();

  num = sq_count(&g_wakelockqueue);

  leave_critical_section(flags);

  return num;
}

int cxd56_pm_hotsleep(int idletime)
{
  struct cxd56_pm_message_s message;
  int ret;

  if (up_pm_count_acquire_wakelock() != 0)
    {
      return -1;
    }

  message.mid = MQMSG_HOT_SLEEP;
  message.data = (uint32_t)idletime;
  ret = mq_send(g_queuedesc, (FAR const char *)&message, sizeof(message),
                CXD56_PM_MESSAGE_PRIO);
  if (ret < 0)
    {
      pmerr("ERR:mq_send(HOT_SLEEP)\n");
      return -1;
    }

  return 0;
}

int cxd56_pm_initialize(void)
{
  int taskid;
  int ret;

  dq_init(&g_cbqueue);
  sq_init(&g_freqlockqueue);
  sq_init(&g_wakelockqueue);

  ret = nxsem_init(&g_regcblock, 0, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxsem_init(&g_freqlock, 0, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxsem_init(&g_freqlockwait, 0, 0);
  nxsem_set_protocol(&g_freqlockwait, SEM_PRIO_NONE);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxsem_init(&g_bootsync, 0, 0);
  nxsem_set_protocol(&g_bootsync, SEM_PRIO_NONE);
  if (ret < 0)
    {
      return ret;
    }

  taskid = task_create("cxd56_pm_task", CXD56_PM_TASK_PRIO,
                       CXD56_PM_TASK_STACKSIZE, cxd56_pm_maintask,
                       (FAR char * const *)NULL);
  if (taskid < 0)
    {
      return -EPERM;
    }

  /* wait until cxd56_pm_maintask thread is ready */

  cxd56_pm_semtake(&g_bootsync);

  return OK;
}

/****************************************************************************
 * Name: up_pm_get_bootcause
 *
 * Description:
 *   Get the system boot cause. This boot cause indicates the cause why the
 *   system is launched from the state of power-off,
 *   deep sleep or cold sleep.
 *   Each boot cause is defined as PM_BOOT_XXX.
 *
 * Return:
 *   Boot cause
 *
 ****************************************************************************/

uint32_t up_pm_get_bootcause(void)
{
  return BKUP->bootcause;
}

/****************************************************************************
 * Name: up_pm_get_bootmask
 *
 * Description:
 *   Get the system boot mask. This boot mask indicates whether the specified
 *   bit is enabled or not as the boot cause. If a bit of boot mask is set,
 *   the boot cause is enabled. Each boot mask is defined as PM_BOOT_XXX.
 *
 * Return:
 *   Boot mask
 *
 ****************************************************************************/

uint32_t up_pm_get_bootmask(void)
{
  return BKUP->bootmask;
}

/****************************************************************************
 * Name: up_pm_set_bootmask
 *
 * Description:
 *   Enable the boot cause of the specified bit.
 *
 * Parameter:
 *   mask - OR of Boot mask defined as PM_BOOT_XXX
 *
 * Return:
 *   Updated boot mask
 *
 ****************************************************************************/

uint32_t up_pm_set_bootmask(uint32_t mask)
{
  irqstate_t flags;

  /* Enable emergency recovery by WKUPL
   * only when bootmask is changed from disable to enable
   */

#ifdef CONFIG_CXD56_PMIC
  if (!(BKUP->bootmask & PM_BOOT_DEEP_WKUPL) && (mask & PM_BOOT_DEEP_WKUPL))
    {
      uint8_t value = WKUPL_ENABLE;
      cxd56_pmic_write(CONFIG_INT_WKUP_REG, &value, sizeof(value));
    }
#endif

  flags = enter_critical_section();

  BKUP->bootmask |= mask;

  leave_critical_section(flags);

  return BKUP->bootmask;
}

/****************************************************************************
 * Name: up_pm_clr_bootmask
 *
 * Description:
 *   Disable the boot cause of the specified bit.
 *
 * Parameter:
 *   mask - OR of Boot mask defined as PM_BOOT_XXX
 *
 * Return:
 *   Updated boot mask
 *
 ****************************************************************************/

uint32_t up_pm_clr_bootmask(uint32_t mask)
{
  irqstate_t flags;

  /* Disable emergency recovery by WKUPL,
   * only when bootmask is changed from enable to disable
   */

#ifdef CONFIG_CXD56_PMIC
  if ((BKUP->bootmask & PM_BOOT_DEEP_WKUPL) && (mask & PM_BOOT_DEEP_WKUPL))
    {
      uint8_t value = WKUPL_DISABLE;
      cxd56_pmic_write(CONFIG_INT_WKUP_REG, &value, sizeof(value));
    }
#endif

  flags = enter_critical_section();

  /* Check if non-maskable bit */

  if (mask & NON_MASKABLE_BOOTMASK)
    {
      pmwarn("Can't be disabled 0x%08x\n", (mask & NON_MASKABLE_BOOTMASK));
      mask &= ~NON_MASKABLE_BOOTMASK;
    }

  /* Make it impossible to be disable both WKUPS and RTC in DEEP */

  if (((BKUP->bootmask & ~mask) & DEEP_PROHIBIT_BOOTMASK) == 0)
    {
      pmwarn("Can't be disabled both 0x%08x and 0x%08x\n",
            PM_BOOT_DEEP_WKUPS, PM_BOOT_DEEP_RTC);
      mask &= ~DEEP_PROHIBIT_BOOTMASK;
    }

  BKUP->bootmask &= ~mask;

  leave_critical_section(flags);

  return BKUP->bootmask;
}

/****************************************************************************
 * Name: up_pm_sleep
 *
 * Description:
 *   Enter sleep mode. This function never returns.
 *
 * Parameter:
 *   mode - PM_SLEEP_DEEP or PM_SLEEP_COLD
 *
 ****************************************************************************/

int up_pm_sleep(enum pm_sleepmode_e mode)
{
  int fw_pm_deepsleep(void *);
  int fw_pm_coldsleep(void *);

  switch (mode)
    {
    case PM_SLEEP_DEEP:
      fw_pm_deepsleep(NULL);
      break;
    case PM_SLEEP_COLD:
      fw_pm_coldsleep(NULL);
      break;
    }

  __asm volatile ("dsb");
  for (; ; );
}

/****************************************************************************
 * Name: up_pm_reboot
 *
 * Description:
 *   System reboot. This function never returns.
 *
 ****************************************************************************/

int up_pm_reboot(void)
{
  void fw_pm_reboot(void);
  fw_pm_reboot();
  __asm volatile ("dsb");
  for (; ; );
}
