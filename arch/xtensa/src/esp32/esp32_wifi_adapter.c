/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wifi_adapter.c
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

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <mqueue.h>
#include <fcntl.h>
#include <unistd.h>
#include <clock/clock.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "nuttx/kmalloc.h"
#include "nuttx/spinlock.h"
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/kthread.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_emac.h"
#include "esp32_cpuint.h"
#include "esp32_wifi_adapter.h"

#include "espidf_wifi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PHY_RF_MASK   ((1 << PHY_BT_MODULE) | (1 << PHY_WIFI_MODULE))

#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
#  define NVS_FS_PREFIX CONFIG_ESP32_WIFI_FS_MOUNTPT
#  define NVS_DIR_BASE  NVS_FS_PREFIX"/wifi."
#  define NVS_FILE_MODE 0777
#endif

#define WIFI_CONNECT_TIMEOUT  CONFIG_ESP32_WIFI_CONNECT_TIMEOUT

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* WiFi interrupt adapter private data */

struct irq_adpt
{
  void (*func)(void *arg);  /* Interrupt callback function */
  void *arg;                /* Interrupt private data */
};

/* WiFi message queue private data */

struct mq_adpt
{
  mqd_t    mq;              /* Message queue handle */
  uint32_t msgsize;         /* Message size */
  char     name[16];        /* Message queue name */
};

/* WiFi time private data */

struct time_adpt
{
  time_t      sec;          /* Second value */
  suseconds_t usec;         /* Micro second value */
};

/* WiFi timer private data */

struct timer_adpt
{
  struct wdog_s wdog;       /* Timer handle */
  struct work_s work;       /* Work private data */
  bool          repeat;     /* Flags indicate if it is cycle */
  uint32_t      delay;      /* Timeout ticks */

  /* Timer callback function */

  void          (*func)(void *priv);
  void          *priv;      /* Timer private data */
};

/* WiFi event private data */

struct evt_adpt
{
  sq_entry_t entry;         /* Sequence entry */
  int32_t id;               /* Event ID */
  uint8_t buf[0];           /* Event private data */
};

/* WiFi event notification private data */

struct wifi_notify
{
  bool assigned;            /* Flag indicate if it is used */
  pid_t pid;                /* Signal's target thread PID */
  struct sigevent event;    /* Signal event private data */
  struct sigwork_s work;    /* Signal work private data */
};

/* WiFi NVS private data */

struct nvs_adpt
{
  char *index_name;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_set_isr(int32_t n, void *f, void *arg);
static void esp32_ints_on(uint32_t mask);
static void esp32_ints_off(uint32_t mask);
static void *esp_spin_lock_create(void);
static void esp_spin_lock_delete(void *lock);
static uint32_t esp_wifi_int_disable(void *wifi_int_mux);
static void esp_wifi_int_restore(void *wifi_int_mux, uint32_t tmp);
static void IRAM_ATTR esp_task_yield_from_isr(void);
static void *esp_semphr_create(uint32_t max, uint32_t init);
static void esp_semphr_delete(void *semphr);
static int32_t esp_semphr_take(void *semphr, uint32_t block_time_tick);
static int32_t esp_semphr_give(void *semphr);
static void *esp_thread_semphr_get(void);
static void *esp_mutex_create(void);
static void *esp_recursive_mutex_create(void);
static void esp_mutex_delete(void *mutex_data);
static int32_t esp_mutex_lock(void *mutex_data);
static int32_t esp_mutex_unlock(void *mutex_data);
static void *esp_queue_create(uint32_t queue_len, uint32_t item_size);
static void esp_queue_delete(void *queue);
static int32_t esp_queue_send(void *queue, void *item,
                              uint32_t block_time_tick);
static int32_t esp_queue_send_from_isr(void *queue, void *item, void *hptw);
static int32_t esp_queue_send_to_back(void *queue, void *item,
                                      uint32_t block_time_tick);
static int32_t esp_queue_send_to_front(void *queue, void *item,
                                       uint32_t block_time_tick);
static int32_t esp_queue_recv(void *queue, void *item,
                              uint32_t block_time_tick);
static uint32_t esp_queue_msg_waiting(void *queue);
static void *esp_event_group_create(void);
static void esp_event_group_delete(void *event);
static uint32_t esp_event_group_set_bits(void *event, uint32_t bits);
static uint32_t esp_event_group_clear_bits(void *event, uint32_t bits);
static uint32_t esp_event_group_wait_bits(void *event,
                                          uint32_t bits_to_wait_for,
                                          int32_t clear_on_exit,
                                          int32_t wait_for_all_bits,
                                          uint32_t block_time_tick);
static int32_t esp_task_create_pinned_to_core(void *task_func,
                                              const char *name,
                                              uint32_t stack_depth,
                                              void *param,
                                              uint32_t prio,
                                              void *task_handle,
                                              uint32_t core_id);
static int32_t esp_task_create(void *task_func, const char *name,
                               uint32_t stack_depth, void *param,
                               uint32_t prio, void *task_handle);
static void esp_task_delete(void *task_handle);
static void esp_task_delay(uint32_t tick);
static int32_t esp_task_ms_to_tick(uint32_t ms);
static void *esp_task_get_current_task(void);
static int32_t esp_task_get_max_priority(void);
static void *esp_malloc(uint32_t size);
static uint32_t esp_rand(void);
static void esp_dport_access_stall_other_cpu_start(void);
static void esp_dport_access_stall_other_cpu_end(void);
static int32_t esp_phy_deinit_rf(uint32_t module);
static void esp_phy_init(uint32_t module);
static void esp_phy_enable_clock(void);
static void esp_phy_disable_clock(void);
static int32_t esp_wifi_read_mac(uint8_t *mac, uint32_t type);
static void esp_timer_arm(void *timer, uint32_t tmout, bool repeat);
static void esp_timer_disarm(void *timer);
static void esp32_timer_done(void *timer);
static void esp_timer_setfn(void *timer, void *pfunction, void *parg);
static void esp_timer_cb(wdparm_t parm);
static void esp_timer_arm_us(void *timer, uint32_t us, bool repeat);
static void esp_periph_module_enable(uint32_t periph);
static void esp_periph_module_disable(uint32_t periph);
static int32_t esp_nvs_set_i8(uint32_t handle, const char *key,
                              int8_t value);
static int32_t esp_nvs_get_i8(uint32_t handle, const char *key,
                              int8_t *out_value);
static int32_t esp_nvs_set_u8(uint32_t handle, const char *key,
                              uint8_t value);
static int32_t esp_nvs_get_u8(uint32_t handle, const char *key,
                              uint8_t *out_value);
static int32_t esp_nvs_set_u16(uint32_t handle, const char *key,
                               uint16_t value);
static int32_t esp_nvs_get_u16(uint32_t handle, const char *key,
                               uint16_t *out_value);
static int32_t esp_nvs_open(const char *name, uint32_t open_mode,
                            uint32_t *out_handle);
static void esp_nvs_close(uint32_t handle);
static int32_t esp_nvs_commit(uint32_t handle);
static int32_t esp_nvs_set_blob(uint32_t handle, const char *key,
                                const void *value, size_t length);
static int32_t esp_nvs_get_blob(uint32_t handle, const char *key,
                                void *out_value, size_t *length);
static int32_t esp_nvs_erase_key(uint32_t handle, const char *key);
static int32_t esp_get_random(uint8_t *buf, size_t len);
static int32_t esp_get_time(void *t);
static void esp_log_writev(uint32_t level, const char *tag,
                           const char *format, va_list args);
static void *esp_malloc_internal(size_t size);
static void *esp_realloc_internal(void *ptr, size_t size);
static void *esp_calloc_internal(size_t n, size_t size);
static void *esp_zalloc_internal(size_t size);
static void *esp_wifi_malloc(size_t size);
static void *esp_wifi_realloc(void *ptr, size_t size);
static void *esp_wifi_calloc(size_t n, size_t size);
static void *esp_wifi_zalloc(size_t size);
static int32_t esp_modem_enter_sleep(uint32_t module);
static int32_t esp_modem_exit_sleep(uint32_t module);
static int32_t esp_modem_register_sleep(uint32_t module);
static int32_t esp_modem_deregister_sleep(uint32_t module);
static void *esp_wifi_create_queue(int32_t queue_len, int32_t item_size);
static void esp_wifi_delete_queue(void *queue);
static uint32_t esp_coex_status_get(void);
static void esp_coex_condition_set(uint32_t type, bool dissatisfy);
static int32_t esp_coex_wifi_request(uint32_t event, uint32_t latency,
                                     uint32_t duration);
static int32_t esp_coex_wifi_release(uint32_t event);

/****************************************************************************
 * Public Functions declaration
 ****************************************************************************/

int64_t esp_timer_get_time(void);
void esp_fill_random(void *buf, size_t len);
void esp_log_write(uint32_t level, const char *tag, const char *format, ...);
uint32_t esp_log_timestamp(void);
uint8_t esp_crc8(const uint8_t *p, uint32_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* WiFi interrupt private data */

static int s_wifi_irq;

/* WiFi thread private data */

static pthread_key_t s_wifi_thread_key;
static bool s_wifi_tkey_init;

/* WiFi sleep private data */

static uint32_t s_esp32_module_mask;
static uint32_t s_esp32_module_sleep;
static bool s_esp32_sleep;
static uint32_t s_phy_clk_en_cnt = 0;
static bool s_esp23_phy_en;
static uint32_t s_esp32_phy_init_mask;
static int64_t s_esp32_phy_rf_stop_tm;

/* WiFi event private data */

static struct work_s s_wifi_evt_work;
static sq_queue_t s_wifi_evt_queue;
static struct wifi_notify s_wifi_notify[WIFI_ADPT_EVT_MAX];
static sem_t s_connect_sem;
static bool s_connected;

static uint8_t s_ssid[32];
static uint8_t s_password[64];
static uint8_t s_ssid_len;
static uint8_t s_password_len;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* WiFi OS adapter data */

wifi_osi_funcs_t g_wifi_osi_funcs =
{
  ._version = ESP_WIFI_OS_ADAPTER_VERSION,
  ._set_isr = esp_set_isr,
  ._ints_on = esp32_ints_on,
  ._ints_off = esp32_ints_off,
  ._spin_lock_create = esp_spin_lock_create,
  ._spin_lock_delete = esp_spin_lock_delete,
  ._wifi_int_disable = esp_wifi_int_disable,
  ._wifi_int_restore = esp_wifi_int_restore,
  ._task_yield_from_isr = esp_task_yield_from_isr,
  ._semphr_create = esp_semphr_create,
  ._semphr_delete = esp_semphr_delete,
  ._semphr_take = esp_semphr_take,
  ._semphr_give = esp_semphr_give,
  ._wifi_thread_semphr_get = esp_thread_semphr_get,
  ._mutex_create = esp_mutex_create,
  ._recursive_mutex_create = esp_recursive_mutex_create,
  ._mutex_delete = esp_mutex_delete,
  ._mutex_lock = esp_mutex_lock,
  ._mutex_unlock = esp_mutex_unlock,
  ._queue_create = esp_queue_create,
  ._queue_delete = esp_queue_delete,
  ._queue_send = esp_queue_send,
  ._queue_send_from_isr = esp_queue_send_from_isr,
  ._queue_send_to_back = esp_queue_send_to_back,
  ._queue_send_to_front = esp_queue_send_to_front,
  ._queue_recv = esp_queue_recv,
  ._queue_msg_waiting = esp_queue_msg_waiting,
  ._event_group_create = esp_event_group_create,
  ._event_group_delete = esp_event_group_delete,
  ._event_group_set_bits = esp_event_group_set_bits,
  ._event_group_clear_bits = esp_event_group_clear_bits,
  ._event_group_wait_bits = esp_event_group_wait_bits,
  ._task_create_pinned_to_core = esp_task_create_pinned_to_core,
  ._task_create = esp_task_create,
  ._task_delete = esp_task_delete,
  ._task_delay = esp_task_delay,
  ._task_ms_to_tick = esp_task_ms_to_tick,
  ._task_get_current_task = esp_task_get_current_task,
  ._task_get_max_priority = esp_task_get_max_priority,
  ._malloc = esp_malloc,
  ._free = free,
  ._event_post = esp_event_post,
  ._get_free_heap_size = esp_get_free_heap_size,
  ._rand = esp_rand,
  ._dport_access_stall_other_cpu_start_wrap =
      esp_dport_access_stall_other_cpu_start,
  ._dport_access_stall_other_cpu_end_wrap =
      esp_dport_access_stall_other_cpu_end,
  ._phy_rf_deinit = esp_phy_deinit_rf,
  ._phy_load_cal_and_init = esp_phy_init,
  ._phy_common_clock_enable = esp_phy_enable_clock,
  ._phy_common_clock_disable = esp_phy_disable_clock,
  ._read_mac = esp_wifi_read_mac,
  ._timer_arm = esp_timer_arm,
  ._timer_disarm = esp_timer_disarm,
  ._timer_done = esp32_timer_done,
  ._timer_setfn = esp_timer_setfn,
  ._timer_arm_us = esp_timer_arm_us,
  ._periph_module_enable = esp_periph_module_enable,
  ._periph_module_disable = esp_periph_module_disable,
  ._esp_timer_get_time = esp_timer_get_time,
  ._nvs_set_i8 = esp_nvs_set_i8,
  ._nvs_get_i8 = esp_nvs_get_i8,
  ._nvs_set_u8 = esp_nvs_set_u8,
  ._nvs_get_u8 = esp_nvs_get_u8,
  ._nvs_set_u16 = esp_nvs_set_u16,
  ._nvs_get_u16 = esp_nvs_get_u16,
  ._nvs_open = esp_nvs_open,
  ._nvs_close = esp_nvs_close,
  ._nvs_commit = esp_nvs_commit,
  ._nvs_set_blob = esp_nvs_set_blob,
  ._nvs_get_blob = esp_nvs_get_blob,
  ._nvs_erase_key = esp_nvs_erase_key,
  ._get_random = esp_get_random,
  ._get_time = esp_get_time,
  ._random = esp_random,
  ._log_write = esp_log_write,
  ._log_writev = esp_log_writev,
  ._log_timestamp = esp_log_timestamp,
  ._malloc_internal =  esp_malloc_internal,
  ._realloc_internal = esp_realloc_internal,
  ._calloc_internal = esp_calloc_internal,
  ._zalloc_internal = esp_zalloc_internal,
  ._wifi_malloc = esp_wifi_malloc,
  ._wifi_realloc = esp_wifi_realloc,
  ._wifi_calloc = esp_wifi_calloc,
  ._wifi_zalloc = esp_wifi_zalloc,
  ._wifi_create_queue = esp_wifi_create_queue,
  ._wifi_delete_queue = esp_wifi_delete_queue,
  ._modem_sleep_enter = esp_modem_enter_sleep,
  ._modem_sleep_exit = esp_modem_exit_sleep,
  ._modem_sleep_register = esp_modem_register_sleep,
  ._modem_sleep_deregister = esp_modem_deregister_sleep,
  ._coex_status_get = esp_coex_status_get,
  ._coex_condition_set = esp_coex_condition_set,
  ._coex_wifi_request = esp_coex_wifi_request,
  ._coex_wifi_release = esp_coex_wifi_release,
  ._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};

/* WiFi feature capacity data */

uint64_t g_wifi_feature_caps;

/* WiFi TAG string data */

ESP_EVENT_DEFINE_BASE(WIFI_EVENT);

/****************************************************************************
 * Private Functions and Public Functions only used by libraries
 ****************************************************************************/

/****************************************************************************
 * Name: esp_errno_trans
 *
 * Description:
 *   Transform from nuttx error code to WiFi adapter error code
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   WiFi adapter error code
 *
 ****************************************************************************/

static inline int32_t esp_errno_trans(int ret)
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
 * Name: esp_int_adpt_cb
 *
 * Description:
 *   WiFi interrupt adapter callback function
 *
 * Input Parameters:
 *   arg - interrupt adapter private data
 *
 * Returned Value:
 *   0 on success
 *
 ****************************************************************************/

static int esp_int_adpt_cb(int irq, void *context, FAR void *arg)
{
  struct irq_adpt *adapter = (struct irq_adpt *)arg;

  adapter->func(adapter->arg);

  return 0;
}

/****************************************************************************
 * Name: esp_thread_semphr_free
 *
 * Description:
 *   Delete thread self's semaphore
 *
 * Input Parameters:
 *   semphr - Semphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_thread_semphr_free(void *semphr)
{
  if (semphr)
    {
      esp_semphr_delete(semphr);
    }
}

/****************************************************************************
 * Name: esp_update_time
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

static void esp_update_time(struct timespec *timespec, uint32_t ticks)
{
  uint32_t tmp;

  tmp = TICK2SEC(ticks);
  timespec->tv_sec += tmp;

  ticks -= SEC2TICK(tmp);
  tmp = TICK2NSEC(ticks);

  timespec->tv_nsec += tmp;
}

/****************************************************************************
 * Name: esp_set_isr
 *
 * Description:
 *   Register interrupt function
 *
 * Input Parameters:
 *   n   - Interrupt ID
 *   f   - Interrupt function
 *   arg - Function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_set_isr(int32_t n, void *f, void *arg)
{
  int ret;
  struct irq_adpt *adapter;
  int irq;
  int cpu = 0;
  int tmp;

  irq = esp32_alloc_levelint(1);
  if (irq < 0)
    {
      wlerr("ERROR: Failed to alloc interrupt\n");
      assert(0);
      return ;
    }

  up_disable_irq(irq);

  tmp = sizeof(struct irq_adpt);
  adapter = kmm_malloc(tmp);
  if (!adapter)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      assert(0);
      return ;
    }

  adapter->func = f;
  adapter->arg = arg;

  tmp = n + XTENSA_IRQ_FIRSTPERIPH;
  ret = irq_attach(tmp, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("ERROR: Failed to attach IRQ %d\n", tmp);
      assert(0);
      return ;
    }

  esp32_attach_peripheral(cpu, n, irq);

  s_wifi_irq = irq;
}

/****************************************************************************
 * Name: esp32_ints_on
 *
 * Description:
 *   Enable WiFi interrupt
 *
 * Input Parameters:
 *   mask - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_ints_on(uint32_t mask)
{
  up_enable_irq(s_wifi_irq);
}

/****************************************************************************
 * Name: esp32_ints_on
 *
 * Description:
 *   Disable WiFi interrupt
 *
 * Input Parameters:
 *   mask - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_ints_off(uint32_t mask)
{
  up_disable_irq(s_wifi_irq);
}

/****************************************************************************
 * Name: esp_spin_lock_create
 *
 * Description:
 *   Create spin lock in SMP mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Spin lock data pointer
 *
 ****************************************************************************/

static void *esp_spin_lock_create(void)
{
#ifdef CONFIG_SMP
  spinlock_t *lock;
  int tmp;

  tmp = sizeof(struct spinlock_t);
  lock = kmm_malloc(tmp);
  if (!lock)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      DEBUGASSERT(0);
    }

  spin_initialize(lock, SP_UNLOCKED);

  return lock;
#else
  /* If return NULL, code may check fail  */

  return (void *)1;
#endif
}

/****************************************************************************
 * Name: esp_spin_lock_delete
 *
 * Description:
 *   Delete spin lock
 *
 * Input Parameters:
 *   lock - Spin lock data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_spin_lock_delete(void *lock)
{
#ifdef CONFIG_SMP
  kmm_free(lock);
#else
  DEBUGASSERT((int)lock == 1);
#endif
}

/****************************************************************************
 * Name: esp_wifi_int_disable
 *
 * Description:
 *   Enter critical by disable interrup, and take spin lock if
 *   in SMP mode
 *
 * Input Parameters:
 *   wifi_int_mux - Spin lock data pointer
 *
 * Returned Value:
 *   CPU PS value.
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp_wifi_int_disable(void *wifi_int_mux)
{
  irqstate_t flags;

  flags = enter_critical_section();

#ifdef CONFIG_SMP
  spin_lock((volatile spinlock_t *)wifi_int_mux);
#endif

  return (uint32_t)flags;
}

/****************************************************************************
 * Name: esp_wifi_int_restore
 *
 * Description:
 *   Exit from critical by enable interrup, and release spin
 *   lock if in SMP mode
 *
 * Input Parameters:
 *   wifi_int_mux - Spin lock data pointer
 *   tmp          - CPU PS value.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp_wifi_int_restore(void *wifi_int_mux, uint32_t tmp)
{
  irqstate_t flags = (irqstate_t)tmp;

#ifdef CONFIG_SMP
  spin_unlock((volatile spinlock_t *)wifi_int_mux);
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_task_yield_from_isr
 *
 * Description:
 *   Do nothing in NuttX
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp_task_yield_from_isr(void)
{
  /* Do nothing */
}

/****************************************************************************
 * Name: esp_semphr_create
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - No mean
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semphore data pointer
 *
 ****************************************************************************/

static void *esp_semphr_create(uint32_t max, uint32_t init)
{
  int ret;
  sem_t *sem;
  int tmp;

  tmp = sizeof(sem_t);
  sem = kmm_malloc(tmp);
  if (!sem)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = sem_init(sem, 0, init);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize sem error=%d\n", ret);
      kmm_free(sem);
      return NULL;
    }

  return sem;
}

/****************************************************************************
 * Name: esp_semphr_delete
 *
 * Description:
 *   Delete semaphore
 *
 * Input Parameters:
 *   semphr - Semphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_semphr_delete(void *semphr)
{
  sem_t *sem = (sem_t *)semphr;

  sem_destroy(sem);
  kmm_free(sem);
}

/****************************************************************************
 * Name: esp_semphr_take
 *
 * Description:
 *   Wait semaphore within a certain period of time
 *
 * Input Parameters:
 *   semphr - Semphore data pointer
 *   ticks  - Wait system ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_semphr_take(void *semphr, uint32_t ticks)
{
  int ret;
  struct timespec timeout;
  sem_t *sem = (sem_t *)semphr;

  if (ticks == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = sem_wait(sem);
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
          esp_update_time(&timeout, ticks);
        }

      ret = sem_timedwait(sem, &timeout);
      if (ret)
        {
          wlerr("ERROR: Failed to wait sem in %d ticks\n", ticks);
        }
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_semphr_give
 *
 * Description:
 *   Post semaphore
 *
 * Input Parameters:
 *   semphr - Semphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_semphr_give(void *semphr)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  ret = sem_post(sem);
  if (ret)
    {
      wlerr("ERROR: Failed to post sem error=%d\n", ret);
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_thread_semphr_get
 *
 * Description:
 *   Get thread self's semaphore
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Semphore data pointer
 *
 ****************************************************************************/

static void *esp_thread_semphr_get(void)
{
  int ret;
  void *sem;

  if (s_wifi_tkey_init)
  {
    ret = pthread_key_create(&s_wifi_thread_key, esp_thread_semphr_free);
    if (ret)
      {
        wlerr("ERROR: Failed to create pthread key\n");
        return NULL;
      }

    s_wifi_tkey_init = true;
  }

  sem = pthread_getspecific(s_wifi_thread_key);
  if (!sem)
    {
      sem = esp_semphr_create(1, 0);
      if (!sem)
        {
          wlerr("ERROR: Failed to create semaphore\n");
          return NULL;
        }

      ret = pthread_setspecific(s_wifi_thread_key, sem);
      if (ret)
        {
          wlerr("ERROR: Failed to set specific\n");
          esp_semphr_delete(sem);
          return NULL;
        }
    }

  return sem;
}

/****************************************************************************
 * Name: esp_mutex_create
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

static void *esp_mutex_create(void)
{
  int ret;
  pthread_mutex_t *mutex;
  int tmp;

  tmp = sizeof(pthread_mutex_t);
  mutex = kmm_malloc(tmp);
  if (!mutex)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = pthread_mutex_init(mutex, NULL);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize mutex error=%d\n", ret);
      kmm_free(mutex);
      return NULL;
    }

  return mutex;
}

/****************************************************************************
 * Name: esp_recursive_mutex_create
 *
 * Description:
 *   Create recursive mutex
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Recursive mutex data pointer
 *
 ****************************************************************************/

static void *esp_recursive_mutex_create(void)
{
  int ret;
  pthread_mutex_t *mutex;
  pthread_mutexattr_t attr;
  int tmp;

  ret = pthread_mutexattr_init(&attr);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize attr error=%d\n", ret);
      return NULL;
    }

  ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  if (ret)
    {
      wlerr("ERROR: Failed to set attr type error=%d\n", ret);
      return NULL;
    }

  tmp = sizeof(pthread_mutex_t);
  mutex = kmm_malloc(tmp);
  if (!mutex)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = pthread_mutex_init(mutex, &attr);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize mutex error=%d\n", ret);
      kmm_free(mutex);
      return NULL;
    }

  return mutex;
}

/****************************************************************************
 * Name: esp_mutex_delete
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

static void esp_mutex_delete(void *mutex_data)
{
  pthread_mutex_t *mutex = (pthread_mutex_t *)mutex_data;

  pthread_mutex_destroy(mutex);
  kmm_free(mutex);
}

/****************************************************************************
 * Name: esp_mutex_lock
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

static int32_t esp_mutex_lock(void *mutex_data)
{
  int ret;
  pthread_mutex_t *mutex = (pthread_mutex_t *)mutex_data;

  ret = pthread_mutex_lock(mutex);
  if (ret)
    {
      wlerr("ERROR: Failed to lock mutex error=%d\n", ret);
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_mutex_unlock
 *
 * Description:
 *   Unlock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_mutex_unlock(void *mutex_data)
{
  int ret;
  pthread_mutex_t *mutex = (pthread_mutex_t *)mutex_data;

  ret = pthread_mutex_unlock(mutex);
  if (ret)
    {
      wlerr("ERROR: Failed to unlock mutex error=%d\n", ret);
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_queue_create
 *
 * Description:
 *   Create message queue
 *
 * Input Parameters:
 *   queue_len - queue message number
 *   item_size - message size
 *
 * Returned Value:
 *   Message queue data pointer
 *
 ****************************************************************************/

static void *esp_queue_create(uint32_t queue_len, uint32_t item_size)
{
  struct mq_attr attr;
  mqd_t mq;
  struct mq_adpt *mq_adpt;

  mq_adpt = kmm_malloc(sizeof(struct mq_adpt));
  if (!mq_adpt)
    {
      wlerr("ERROR: Failed to malloc\n");
      return NULL;
    }

  snprintf(mq_adpt->name, sizeof(mq_adpt->name),
           "/tmp/%X", mq_adpt);

  attr.mq_maxmsg  = queue_len;
  attr.mq_msgsize = item_size;
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  mq = mq_open(mq_adpt->name, O_RDWR | O_CREAT, 0644, &attr);
  if (!mq)
    {
      wlerr("ERROR: Failed to create mqueue\n");
      kmm_free(mq_adpt);
      return NULL;
    }

  mq_adpt->mq = mq;
  mq_adpt->msgsize = item_size;

  return (void *)mq_adpt;
}

/****************************************************************************
 * Name: esp_queue_delete
 *
 * Description:
 *   Delete message queue
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_queue_delete(void *queue)
{
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  mq_close(mq_adpt->mq);
  mq_unlink(mq_adpt->name);
  kmm_free(mq_adpt);
}

/****************************************************************************
 * Name: esp_queue_send_generic
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

static int32_t esp_queue_send_generic(void *queue, void *item,
                                      uint32_t ticks, int prio)
{
  int ret;
  struct timespec timeout;
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  if (ticks == OSI_FUNCS_TIME_BLOCKING || ticks == 0)
    {
      /**
       * WiFi interrupt function will call this adapter function to send
       * message to message queue, so here we should call kernel API
       * instead of application API
       */

      ret = nxmq_send(mq_adpt->mq, (const char *)item,
                    mq_adpt->msgsize, prio);
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
          esp_update_time(&timeout, ticks);
        }

      ret = mq_timedsend(mq_adpt->mq, (const char *)item,
                         mq_adpt->msgsize, prio, &timeout);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to timedsend message to mqueue error=%d\n",
               ret);
        }
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_queue_send
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

static int32_t esp_queue_send(void *queue, void *item, uint32_t ticks)
{
  return esp_queue_send_generic(queue, item, ticks, 0);
}

/****************************************************************************
 * Name: esp_queue_send_from_isr
 *
 * Description:
 *   Send message of low priority to queue in ISR within
 *   a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   hptw  - No mean
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_queue_send_from_isr(void *queue, void *item, void *hptw)
{
  /* Force to set the value to be false */

  *((int *)hptw) = false;

  return esp_queue_send_generic(queue, item, 0, 0);
}

/****************************************************************************
 * Name: esp_queue_send_to_back
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

static int32_t esp_queue_send_to_back(void *queue, void *item,
                                      uint32_t ticks)
{
  return esp_queue_send_generic(queue, item, ticks, 0);
}

/****************************************************************************
 * Name: esp_queue_send_from_isr
 *
 * Description:
 *   Send message of high priority to queue within a certain period of time
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

static int32_t esp_queue_send_to_front(void *queue, void *item,
                                       uint32_t ticks)
{
  return esp_queue_send_generic(queue, item, ticks, 1);
}

/****************************************************************************
 * Name: esp_queue_recv
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

static int32_t esp_queue_recv(void *queue, void *item, uint32_t ticks)
{
  ssize_t ret;
  struct timespec timeout;
  unsigned int prio;
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  if (ticks == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = mq_receive(mq_adpt->mq, (char *)item,
                       mq_adpt->msgsize, &prio);
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

      if (ticks)
        {
          esp_update_time(&timeout, ticks);
        }

      ret = mq_timedreceive(mq_adpt->mq, (char *)item,
                            mq_adpt->msgsize, &prio, &timeout);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to timedreceive from mqueue error=%d\n",
               ret);
        }
    }

  return ret > 0 ? true : false;
}

/****************************************************************************
 * Name: esp_queue_msg_waiting
 *
 * Description:
 *   Get message number in the message queue
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *
 * Returned Value:
 *   Message number
 *
 ****************************************************************************/

static uint32_t esp_queue_msg_waiting(void *queue)
{
  int ret;
  struct mq_attr attr;
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  ret = mq_getattr(mq_adpt->mq, &attr);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to get attr from mqueue error=%d\n", ret);
      return 0;
    }

  return attr.mq_curmsgs;
}

/****************************************************************************
 * Name: esp_event_group_create
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void *esp_event_group_create(void)
{
  DEBUGASSERT(0);

  return NULL;
}

/****************************************************************************
 * Name: esp_event_group_delete
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void esp_event_group_delete(void *event)
{
  DEBUGASSERT(0);
}

/****************************************************************************
 * Name: esp_event_group_set_bits
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t esp_event_group_set_bits(void *event, uint32_t bits)
{
  DEBUGASSERT(0);

  return false;
}

/****************************************************************************
 * Name: esp_event_group_clear_bits
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t esp_event_group_clear_bits(void *event, uint32_t bits)
{
  DEBUGASSERT(0);

  return false;
}

/****************************************************************************
 * Name: esp_event_group_wait_bits
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t esp_event_group_wait_bits(void *event,
                                          uint32_t bits_to_wait_for,
                                          int32_t clear_on_exit,
                                          int32_t wait_for_all_bits,
                                          uint32_t block_time_tick)
{
  DEBUGASSERT(0);

  return false;
}

/****************************************************************************
 * Name: esp_task_create_pinned_to_core
 *
 * Description:
 *   Create task and bind it to target CPU, the task will run when it
 *   is created
 *
 * Input Parameters:
 *   entry       - Task entry
 *   name        - Task name
 *   stack_depth - Task stack size
 *   param       - Task private data
 *   prio        - Task priority
 *   task_handle - Task handle pointer which is used to pause, resume
 *                 and delete the task
 *   core_id     - CPU which the task runs in
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_task_create_pinned_to_core(void *entry,
                                              const char *name,
                                              uint32_t stack_depth,
                                              void *param,
                                              uint32_t prio,
                                              void *task_handle,
                                              uint32_t core_id)
{
  int pid;
#ifdef CONFIG_SMP
  int ret;
  cpu_set_t cpuset;
#endif

  pid = kthread_create(name, prio, stack_depth, entry,
                      (char * const *)param);
  if (pid > 0)
    {
      *((int *)task_handle) = pid;

#ifdef CONFIG_SMP
      if (core_id < CONFIG_SMP_NCPUS)
        {
          CPU_ZERO(&cpuset);
          CPU_SET(core_id, &cpuset);
          ret = nxsched_set_affinity(pid, sizeof(cpuset), &cpuset);
          if (ret)
            {
              wlerr("ERROR: Failed to set affinity error=%d\n", ret);
              return false;
            }
        }
#endif
    }
  else
    {
      wlerr("ERROR: Failed to create task\n");
    }

  return pid > 0 ? true : false;
}

/****************************************************************************
 * Name: esp_task_create
 *
 * Description:
 *   Create task and the task will run when it is created
 *
 * Input Parameters:
 *   entry       - Task entry
 *   name        - Task name
 *   stack_depth - Task stack size
 *   param       - Task private data
 *   prio        - Task priority
 *   task_handle - Task handle pointer which is used to pause, resume
 *                 and delete the task
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_task_create(void *entry, const char *name,
                               uint32_t stack_depth, void *param,
                               uint32_t prio, void *task_handle)
{
  return esp_task_create_pinned_to_core(entry, name, stack_depth, param,
                                        prio, task_handle, UINT32_MAX);
}

/****************************************************************************
 * Name: esp_task_delete
 *
 * Description:
 *   Delete the target task
 *
 * Input Parameters:
 *   task_handle - Task handle pointer which is used to pause, resume
 *                 and delete the task
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_task_delete(void *task_handle)
{
  pid_t pid = (pid_t)((uintptr_t)task_handle);

  kthread_delete(pid);
}

/****************************************************************************
 * Name: esp_task_delay
 *
 * Description:
 *   Current task wait for some ticks
 *
 * Input Parameters:
 *   tick - Waiting ticks
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_task_delay(uint32_t tick)
{
  useconds_t us = TICK2USEC(tick);

  usleep(us);
}

/****************************************************************************
 * Name: esp_task_ms_to_tick
 *
 * Description:
 *   Transform from millim seconds to system ticks
 *
 * Input Parameters:
 *   ms - Millim seconds
 *
 * Returned Value:
 *   System ticks
 *
 ****************************************************************************/

static int32_t esp_task_ms_to_tick(uint32_t ms)
{
  return MSEC2TICK(ms);
}

/****************************************************************************
 * Name: esp_task_get_current_task
 *
 * Description:
 *   Transform from millim seconds to system ticks
 *
 * Input Parameters:
 *   ms - Millim seconds
 *
 * Returned Value:
 *   System ticks
 *
 ****************************************************************************/

static void *esp_task_get_current_task(void)
{
  pid_t pid = getpid();

  return (void *)((uintptr_t)pid);
}

/****************************************************************************
 * Name: esp_task_get_current_task
 *
 * Description:
 *   Get OS task maxium priority
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Task maxium priority
 *
 ****************************************************************************/

static int32_t esp_task_get_max_priority(void)
{
  return SCHED_PRIORITY_MAX;
}

/****************************************************************************
 * Name: esp_malloc_internal
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

static void *esp_malloc(uint32_t size)
{
  return malloc(size);
}

/****************************************************************************
 * Name: esp_event_id_map
 *
 * Description:
 *   Transform from esp-idf event ID to WiFi adapter event ID
 *
 * Input Parameters:
 *   event_id - esp-idf event ID
 *
 * Returned Value:
 *   WiFi adapter event ID
 *
 ****************************************************************************/

static int esp_event_id_map(int event_id)
{
  int id;

  switch (event_id)
    {
      case WIFI_EVENT_STA_START:
        id = WIFI_ADPT_EVT_STA_START;
        break;
      case WIFI_EVENT_STA_CONNECTED:
        id = WIFI_ADPT_EVT_STA_CONNECT;
        break;
      case WIFI_EVENT_STA_DISCONNECTED:
        id = WIFI_ADPT_EVT_STA_DISCONNECT;
        break;
      case WIFI_EVENT_STA_AUTHMODE_CHANGE:
        id = WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE;
        break;
      case WIFI_EVENT_STA_STOP:
        id = WIFI_ADPT_EVT_STA_STOP;
        break;
      default:
        return -1;
    }

  return id;
}

/****************************************************************************
 * Name: esp_evt_work_cb
 *
 * Description:
 *   Process the cached event
 *
 * Input Parameters:
 *   arg - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_evt_work_cb(FAR void *arg)
{
  int ret;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;
  struct wifi_notify *notify;

  while (1)
    {
      flags = enter_critical_section();
      evt_adpt = (struct evt_adpt *)sq_remfirst(&s_wifi_evt_queue);
      leave_critical_section(flags);
      if (!evt_adpt)
        {
          break;
        }

      switch (evt_adpt->id)
        {
          case WIFI_ADPT_EVT_STA_START:
            ret = esp_wifi_connect();
            if (ret)
              {
                wlerr("ERROR: Failed to connect\n");
              }
            break;
          case WIFI_ADPT_EVT_STA_CONNECT:
            s_connected = true;
            ret = sem_post(&s_connect_sem);
            if (ret)
              {
                wlerr("ERROR: Failed to post sem error=%d\n", errno);
              }
            break;
          case WIFI_ADPT_EVT_STA_DISCONNECT:
            s_connected = false;
            ret = esp_wifi_connect();
            if (ret)
              {
                wlerr("ERROR: Failed to connect\n");
              }
            break;
          default:
            break;
        }

      notify = &s_wifi_notify[evt_adpt->id];
      if (notify->assigned)
        {
          notify->event.sigev_value.sival_ptr = evt_adpt->buf;

          ret = nxsig_notification(notify->pid, &notify->event,
                                   SI_QUEUE, &notify->work);
          if (ret < 0)
            {
              wlwarn("ERROR: nxsig_notification event ID=%d failed: %d\n",
                     evt_adpt->id, ret);
            }
        }

      free(evt_adpt);
    }
}

/****************************************************************************
 * Name: esp_event_post
 *
 * Description:
 *   Active work queue and let the work to process the cached event
 *
 * Input Parameters:
 *   event_base      - Event set name
 *   event_id        - Event ID
 *   event_data      - Event private data
 *   event_data_size - Event data size
 *   ticks           - Waiting system ticks
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_event_post(esp_event_base_t event_base,
                       int32_t event_id,
                       void *event_data,
                       size_t event_data_size,
                       uint32_t ticks)
{
  size_t size;
  int32_t id;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;

  wlinfo("Event: base=%s id=%d data=%p data_size=%d ticks=%u\n", event_base,
         event_id, event_data, event_data_size, ticks);

  id = esp_event_id_map(event_id);
  if (id < 0)
    {
      wlerr("ERROR: No process event %d\n", event_id);
      return -1;
    }

  size = event_data_size + sizeof(struct evt_adpt);
  evt_adpt = malloc(size);
  if (!evt_adpt)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", size);
      return -1;
    }

  evt_adpt->id = id;
  memcpy(evt_adpt->buf, event_data, event_data_size);

  flags = enter_critical_section();
  sq_addlast(&evt_adpt->entry, &s_wifi_evt_queue);
  leave_critical_section(flags);

  work_queue(LPWORK, &s_wifi_evt_work, esp_evt_work_cb, NULL, 0);

  return 0;
}

/****************************************************************************
 * Name: esp_task_get_current_task
 *
 * Description:
 *   Get free heap size by byte
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Free heap size
 *
 ****************************************************************************/

uint32_t esp_get_free_heap_size(void)
{
  int ret;
  struct mallinfo info;

  ret = mm_mallinfo(&g_mmheap, &info);
  if (ret)
    {
      wlerr("ERROR: Failed to create task\n");
      return 0;
    }

  return info.fordblks;
}

/****************************************************************************
 * Name: esp_dport_access_stall_other_cpu_start
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void esp_dport_access_stall_other_cpu_start(void)
{
#ifdef CONFIG_SMP
  DEBUGASSERT(0);
#endif
}

/****************************************************************************
 * Name: esp_dport_access_stall_other_cpu_end
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void esp_dport_access_stall_other_cpu_end(void)
{
#ifdef CONFIG_SMP
  DEBUGASSERT(0);
#endif
}

/****************************************************************************
 * Name: esp_phy_rf_init
 *
 * Description:
 *   Initialize PHY hardware with given parameters
 *
 * Input Parameters:
 *   init_data        - PHY hardware initialization parameters
 *   mode             - PHY RF calculation mode
 *   calibration_data - PHY RF calculation parameters
 *   module           - PHY mode which is to be initialized
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_phy_rf_init(const esp_phy_init_data_t *init_data,
                        esp_phy_calibration_mode_t mode,
                        esp_phy_calibration_data_t *calibration_data,
                        phy_rf_module_t module)
{
  irqstate_t flags;
  int64_t time;
  bool enable = false;

  if (module >= PHY_MODULE_COUNT)
    {
      return -1;
    }

  flags = enter_critical_section();

  s_esp32_phy_init_mask |= 1 << module;

  if (s_esp23_phy_en)
    {
      leave_critical_section(flags);
      return 0;
    }

  if (module == PHY_MODEM_MODULE)
    {
      if (s_esp32_phy_init_mask & PHY_RF_MASK)
        {
          enable = true;
        }
    }
  else if (module == PHY_WIFI_MODULE || module == PHY_BT_MODULE)
    {
      enable = true;
    }

  if (enable)
    {
      if (s_esp32_phy_rf_stop_tm)
        {
          time = esp_timer_get_time() - s_esp32_phy_rf_stop_tm;
          esp_wifi_internal_update_mac_time((uint32_t)time);
          s_esp32_phy_rf_stop_tm = 0;
        }

      esp_phy_enable_clock();

      phy_set_wifi_mode_only(0);

      register_chipv7_phy(init_data, calibration_data, mode);

      s_esp23_phy_en = true;
    }

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: esp_phy_deinit_rf
 *
 * Description:
 *   Deinitialize PHY hardware
 *
 * Input Parameters:
 *   module - PHY mode which is to be deinitialized
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_phy_deinit_rf(uint32_t module)
{
  irqstate_t flags;
  bool disable = false;

  if (module >= PHY_MODULE_COUNT)
    {
      return -1;
    }

  flags = enter_critical_section();

  s_esp32_phy_init_mask |= ~(1 << module);

  if (!s_esp23_phy_en)
    {
      leave_critical_section(flags);
      return 0;
    }

  if (module == PHY_MODEM_MODULE)
    {
      disable = true;
    }
  else if (module == PHY_WIFI_MODULE || module == PHY_BT_MODULE)
    {
      if (!(s_esp32_phy_init_mask & PHY_RF_MASK))
        {
          disable = true;
        }
    }

  if (disable)
    {
      phy_close_rf();

      s_esp32_phy_rf_stop_tm = esp_timer_get_time();

      esp_phy_disable_clock();

      s_esp23_phy_en = false;
    }

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: esp_phy_init
 *
 * Description:
 *   Initialize PHY hardware
 *
 * Input Parameters:
 *   module - PHY mode which is to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_phy_init(uint32_t module)
{
  int ret;
  esp_phy_calibration_data_t *cal_data;

  cal_data = kmm_zalloc(sizeof(esp_phy_calibration_data_t));
  if (!cal_data)
    {
      wlerr("ERROR: Failed to malloc");
      DEBUGASSERT(0);
    }

  ret = esp_phy_rf_init(&phy_init_data, PHY_RF_CAL_FULL, cal_data, module);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize RF");
      DEBUGASSERT(0);
    }

  kmm_free(cal_data);
}

/****************************************************************************
 * Name: esp_phy_enable_clock
 *
 * Description:
 *   Enable PHY hardware clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_phy_enable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (s_phy_clk_en_cnt == 0)
    {
      modifyreg32(DPORT_WIFI_CLK_EN_REG, 0,
                  DPORT_WIFI_CLK_WIFI_BT_COMMON_M);
    }

  s_phy_clk_en_cnt++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_phy_disable_clock
 *
 * Description:
 *   Disable PHY hardware clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_phy_disable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (s_phy_clk_en_cnt)
    {
      s_phy_clk_en_cnt--;
      if (!s_phy_clk_en_cnt)
        {
          modifyreg32(DPORT_WIFI_CLK_EN_REG,
                      DPORT_WIFI_CLK_WIFI_BT_COMMON_M,
                      0);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_read_mac
 *
 * Description:
 *   Read MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *   type - MAC address type
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_read_mac(uint8_t *mac, esp_mac_type_t type)
{
  uint32_t regval[2];
  uint8_t tmp;
  uint8_t *data = (uint8_t *)regval;
  uint8_t crc;
  int i;

  if (type > ESP_MAC_WIFI_SOFTAP)
    {
      wlerr("ERROR: Input type is error=%d\n", type);
      return -1;
    }

  regval[0] = getreg32(MAC_ADDR0_REG);
  regval[1] = getreg32(MAC_ADDR1_REG);

  crc = data[6];
  for (i = 0; i < 6; i++)
    {
      mac[i] = data[5 - i];
    }

  if (crc != esp_crc8(mac, 6))
    {
      wlerr("ERROR: Failed to check MAC address CRC\n");
      return -1;
    }

  if (type == ESP_MAC_WIFI_SOFTAP)
    {
      tmp = mac[0];
      for (i = 0; i < 64; i++)
        {
          mac[0] = tmp | 0x02;
          mac[0] ^= i << 2;

          if (mac[0] != tmp)
            {
              break;
            }
        }

      if (i >= 64)
        {
          wlerr("ERROR: Failed to generate softAP MAC\n");
          return -1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_read_mac
 *
 * Description:
 *   Read MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *   type - MAC address type
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_wifi_read_mac(uint8_t *mac, uint32_t type)
{
  return esp_read_mac(mac, type);
}

/****************************************************************************
 * Name: esp_timer_arm
 *
 * Description:
 *   Set timer timeout period and repeat flag
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *   ms     - millim seconds
 *   repeat - true: run cycle, false: run once
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_timer_arm(void *ptimer, uint32_t ms, bool repeat)
{
  esp_timer_arm_us(ptimer, ms * 1000, repeat);
}

/****************************************************************************
 * Name: esp_timer_disarm
 *
 * Description:
 *   Disable timer
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_timer_disarm(void *ptimer)
{
  struct timer_adpt *timer_adpt;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;

  if (ets_timer->priv)
    {
      timer_adpt = (struct timer_adpt *)ets_timer->priv;

      wd_cancel(&timer_adpt->wdog);
      work_cancel(LPWORK, &timer_adpt->work);
    }
}

/****************************************************************************
 * Name: esp32_timer_done
 *
 * Description:
 *   Disable and free timer
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_timer_done(void *ptimer)
{
  struct timer_adpt *timer_adpt;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;

  if (ets_timer->priv)
    {
      timer_adpt = (struct timer_adpt *)ets_timer->priv;

      wd_cancel(&timer_adpt->wdog);
      work_cancel(LPWORK, &timer_adpt->work);

      kmm_free(timer_adpt);

      ets_timer->priv = NULL;
    }
}

/****************************************************************************
 * Name: esp_timer_setfn
 *
 * Description:
 *   Set timer callback function and private data
 *
 * Input Parameters:
 *   ptimer    - Timer data pointer
 *   pfunction - Callback function
 *   parg      - Callback function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_timer_setfn(void *ptimer, void *pfunction, void *parg)
{
  struct timer_adpt *timer_adpt;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;

  if (ets_timer->priv)
    {
      return ;
    }

  timer_adpt = kmm_zalloc(sizeof(struct timer_adpt));
  if (!timer_adpt)
    {
      wlerr("ERROR: Failed to malloc\n");
      return ;
    }

  timer_adpt->func = pfunction;
  timer_adpt->priv = parg;

  ets_timer->priv = timer_adpt;
}

/****************************************************************************
 * Name: esp_timer_work_cb
 *
 * Description:
 *   Process timer callback function in workqueue and active
 *   it if it has repeat flag
 *
 * Input Parameters:
 *   arg - Timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_timer_work_cb(FAR void *arg)
{
  struct ets_timer *ets_timer = (struct ets_timer *)arg;
  struct timer_adpt *timer_adpt =
            (struct timer_adpt *)ets_timer->priv;

  timer_adpt->func(timer_adpt->priv);
  if (timer_adpt->repeat)
    {
      wd_start(&timer_adpt->wdog, timer_adpt->delay,
               esp_timer_cb, (wdparm_t)ets_timer);
    }
}

/****************************************************************************
 * Name: esp_timer_cb
 *
 * Description:
 *   Post event to work queue and let work queue to process the timer's
 *   real callback function
 *
 * Input Parameters:
 *   parm - Timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_timer_cb(wdparm_t parm)
{
  struct ets_timer *ets_timer = (struct ets_timer *)parm;
  struct timer_adpt *timer_adpt =
            (struct timer_adpt *)ets_timer->priv;

  work_queue(LPWORK, &timer_adpt->work,
             esp_timer_work_cb, ets_timer, 0);
}

/****************************************************************************
 * Name: esp_timer_arm_us
 *
 * Description:
 *   Set timer timeout period and repeat flag
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *   us     - micro seconds
 *   repeat - true: run cycle, false: run once
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_timer_arm_us(void *ptimer, uint32_t us, bool repeat)
{
  uint32_t delay;
  struct timer_adpt *timer_adpt;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;

  if (ets_timer->priv)
    {
      timer_adpt = (struct timer_adpt *)ets_timer->priv;

      wd_cancel(&timer_adpt->wdog);
      work_cancel(LPWORK, &timer_adpt->work);

      delay = USEC2TICK(us);
      timer_adpt->repeat = (uint32_t)repeat;
      timer_adpt->delay = delay;

      wd_start(&timer_adpt->wdog, delay,
               esp_timer_cb, (wdparm_t)ets_timer);
    }
}

/****************************************************************************
 * Name: esp_periph_module_enable
 *
 * Description:
 *   Enable WiFi module clock
 *
 * Input Parameters:
 *   periph - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_periph_module_enable(uint32_t periph)
{
  modifyreg32(DPORT_WIFI_CLK_EN_REG, 0, DPORT_WIFI_CLK_WIFI_EN_M);
}

/****************************************************************************
 * Name: esp_periph_module_enable
 *
 * Description:
 *   Disable WiFi module clock
 *
 * Input Parameters:
 *   periph - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_periph_module_disable(uint32_t periph)
{
  modifyreg32(DPORT_WIFI_CLK_EN_REG, DPORT_WIFI_CLK_WIFI_EN_M, 0);
}

/****************************************************************************
 * Name: esp_timer_get_time
 *
 * Description:
 *   Get system time of type int64_t
 *
 * Input Parameters:
 *   periph - No mean
 *
 * Returned Value:
 *   System time
 *
 ****************************************************************************/

int64_t esp_timer_get_time(void)
{
  int64_t us;
  struct timeval tv;
  int ret;

  ret = gettimeofday(&tv, NULL);
  if (!ret)
    {
      us = tv.tv_sec * (1000 * 1000) + tv.tv_usec;
    }
  else
    {
      us = 0;
      wlerr("ERROR: Failed to get time of day\n");
    }

  return us;
}

/****************************************************************************
 * Name: esp_nvs_set_i8
 *
 * Description:
 *   Save data of type int8_t into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored data
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_set_i8(uint32_t handle,
                              const char *key,
                              int8_t value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  return esp_nvs_set_blob(handle, key, &value, sizeof(int8_t));
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_get_i8
 *
 * Description:
 *   Read data of type int8_t from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_get_i8(uint32_t handle,
                              const char *key,
                              int8_t *out_value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  size_t len = sizeof(int8_t);

  return esp_nvs_get_blob(handle, key, out_value, &len);
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_set_u8
 *
 * Description:
 *   Save data of type uint8_t into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored data
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_set_u8(uint32_t handle,
                              const char *key,
                              uint8_t value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  return esp_nvs_set_blob(handle, key, &value, sizeof(uint8_t));
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_get_u8
 *
 * Description:
 *   Read data of type uint8_t from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_get_u8(uint32_t handle,
                              const char *key,
                              uint8_t *out_value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  size_t len = sizeof(uint8_t);

  return esp_nvs_get_blob(handle, key, out_value, &len);
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_set_u16
 *
 * Description:
 *   Save data of type uint16_t into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored data
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_set_u16(uint32_t handle,
                               const char *key,
                               uint16_t value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  return esp_nvs_set_blob(handle, key, &value, sizeof(uint16_t));
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_get_u16
 *
 * Description:
 *   Read data of type uint16_t from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_get_u16(uint32_t handle,
                               const char *key,
                               uint16_t *out_value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  size_t len = sizeof(uint16_t);

  return esp_nvs_get_blob(handle, key, out_value, &len);
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_open
 *
 * Description:
 *   Create a file system storage data object
 *
 * Input Parameters:
 *   name       - Storage index
 *   open_mode  - Storage mode
 *   out_handle - Storage handle
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_open(const char *name,
                            uint32_t open_mode,
                            uint32_t *out_handle)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  int ret;
  struct nvs_adpt *nvs_adpt;
  int tmp;
  char *index_name;

  tmp = sizeof(struct nvs_adpt);
  nvs_adpt = kmm_malloc(tmp);
  if (!nvs_adpt)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return -1;
    }

  ret = asprintf(&index_name, "%s", name);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to create NVS index_name string\n");
      kmm_free(nvs_adpt);
      return -1;
    }

  nvs_adpt->index_name = index_name;
  *out_handle = (uint32_t)nvs_adpt;

  return 0;
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_close
 *
 * Description:
 *   Close storage data object and free resource
 *
 * Input Parameters:
 *   handle - NVS handle
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static void esp_nvs_close(uint32_t handle)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  struct nvs_adpt *nvs_adpt = (struct nvs_adpt *)handle;

  kmm_free(nvs_adpt->index_name);
  kmm_free(nvs_adpt);
#else
  DEBUGASSERT(0);
#endif
}

/****************************************************************************
 * Name: esp_nvs_commit
 *
 * Description:
 *   This function has no practical effect
 *
 ****************************************************************************/

static int32_t esp_nvs_commit(uint32_t handle)
{
  return 0;
}

/****************************************************************************
 * Name: esp_nvs_set_blob
 *
 * Description:
 *   Save a block of data into file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
 *   value  - Stored buffer pointer
 *   length - Buffer length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_set_blob(uint32_t handle,
                                const char *key,
                                const void *value,
                                size_t length)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  int fd;
  int ret;
  char *dir;
  struct nvs_adpt *nvs_adpt = (struct nvs_adpt *)handle;
  char *index_name = nvs_adpt->index_name;

  ret = asprintf(&dir, NVS_DIR_BASE"%s.%s", index_name, key);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to create NVS dir string\n");
      return -1;
    }

  ret = unlink(dir);
  if (ret)
    {
      if (errno != ENOENT)
        {
          wlerr("ERROR: Failed to unlink %s error=%d\n", dir, errno);
          free(dir);
          return -1;
        }
    }

  fd = open(dir, O_WRONLY | O_CREAT, NVS_FILE_MODE);
  if (fd < 0)
    {
      wlerr("ERROR: Failed to set open %s\n", dir);
      free(dir);
      return -1;
    }

  ret = write(fd, value, length);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to write to %s\n", dir);
      free(dir);
      close(fd);
      return -1;
    }

  free(dir);
  close(fd);

  return 0;
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_get_blob
 *
 * Description:
 *   Read a block of data from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *   out_value - Read buffer pointer
 *   length    - Buffer length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_get_blob(uint32_t handle,
                                const char *key,
                                void *out_value,
                                size_t *length)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  int fd;
  int ret;
  char *dir;
  struct nvs_adpt *nvs_adpt = (struct nvs_adpt *)handle;
  char *index_name = nvs_adpt->index_name;

  ret = asprintf(&dir, NVS_DIR_BASE"%s.%s", index_name, key);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to create NVS dir string\n");
      return -1;
    }

  fd = open(dir, O_RDONLY, NVS_FILE_MODE);
  if (fd < 0)
    {
      free(dir);
      if (errno == ENOENT)
        {
          wlinfo("INFO: No file %s\n", dir);
          return ESP_ERR_NVS_NOT_FOUND;
        }
      wlerr("ERROR: Failed to get open %s\n", dir);
      return -1;
    }

  ret = read(fd, out_value, *length);
  if (ret <= 0)
    {
      wlerr("ERROR: Failed to write to %s\n", dir);
      free(dir);
      close(fd);
      return -1;
    }
  else
    {
      *length = ret;
    }

  free(dir);
  close(fd);

  return 0;
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_nvs_erase_key
 *
 * Description:
 *   Read a block of data from file system
 *
 * Input Parameters:
 *   handle    - NVS handle
 *   key       - Data index
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_nvs_erase_key(uint32_t handle, const char *key)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  int ret;
  char *dir;
  struct nvs_adpt *nvs_adpt = (struct nvs_adpt *)handle;
  char *index_name = nvs_adpt->index_name;

  ret = asprintf(&dir, NVS_DIR_BASE"%s.%s", index_name, key);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to create NVS dir string\n");
      return -1;
    }

  ret = unlink(dir);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to delete NVS file %s\n", dir);
      free(dir);
      return -1;
    }

  free(dir);

  return 0;
#else
  DEBUGASSERT(0);

  return -1;
#endif
}

/****************************************************************************
 * Name: esp_fill_random
 *
 * Description:
 *   Fill random data int given buffer of given length
 *
 * Input Parameters:
 *   buf - buffer pointer
 *   len - buffer length
 *
 * Returned Value:
 *
 ****************************************************************************/

void esp_fill_random(void *buf, size_t len)
{
  uint8_t *p = (uint8_t *)buf;
  uint32_t tmp;
  uint32_t n;

  while (len > 0)
    {
      tmp = random();
      n = len < 4 ? len : 4;

      memcpy(p, &tmp, n);

      p += n;
      len -= n;
    }
}

/****************************************************************************
 * Name: esp_get_random
 *
 * Description:
 *   Fill random data int given buffer of given length
 *
 * Input Parameters:
 *   buf - buffer pointer
 *   len - buffer length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_get_random(uint8_t *buf, size_t len)
{
  esp_fill_random(buf, len);

  return 0;
}

/****************************************************************************
 * Name: esp_get_time
 *
 * Description:
 *   Get std C time
 *
 * Input Parameters:
 *   t - buffer to store time of type timeval
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_get_time(void *t)
{
  int ret;
  struct timeval tv;
  struct time_adpt *time_adpt = (struct time_adpt *)t;

  ret = gettimeofday(&tv, NULL);
  if (!ret)
    {
      time_adpt->sec  = (time_t)tv.tv_sec;
      time_adpt->usec = (suseconds_t)tv.tv_usec;
    }
  else
    {
      wlerr("ERROR: Failed to get time of day\n");
    }

  return ret;
}

/****************************************************************************
 * Name: esp_rand
 *
 * Description:
 *   Get random data of type uint32_t
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Random data
 *
 ****************************************************************************/

static uint32_t esp_rand(void)
{
  return esp_random();
}

/****************************************************************************
 * Name: esp_log_writev
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   tag    - log TAG, no mean here
 *   format - format string
 *   args   - arguments list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_log_writev(uint32_t level, const char *tag,
                           const char *format, va_list args)
{
  int pri;

  switch (level)
    {
      case ESP_LOG_ERROR:
        pri = LOG_ERR;
        break;
      case ESP_LOG_WARN:
        pri = LOG_WARNING;
        break;
      case ESP_LOG_INFO:
        pri = LOG_INFO;
        break;
      default:
        pri = LOG_DEBUG;
        break;
    }

  vsyslog(pri, format, args);
}

/****************************************************************************
 * Name: esp_log_write
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   tag    - log TAG, no mean here
 *   format - format string
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_log_write(uint32_t level,
                   const char *tag,
                   const char *format, ...)
{
    va_list list;
    va_start(list, format);
    esp_log_writev(level, tag, format, list);
    va_end(list);
}

/****************************************************************************
 * Name: esp_log_timestamp
 *
 * Description:
 *   Get system time by millim second
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   System time
 *
 ****************************************************************************/

uint32_t esp_log_timestamp(void)
{
  return (uint32_t)(esp_timer_get_time() / 1000);
}

/****************************************************************************
 * Name: esp_malloc_internal
 *
 * Description:
 *   Drivers allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

static void *esp_malloc_internal(size_t size)
{
  return kmm_malloc(size);
}

/****************************************************************************
 * Name: esp_realloc_internal
 *
 * Description:
 *   Drivers allocate a block of memory by old memory block
 *
 * Input Parameters:
 *   ptr  - old memory pointer
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/

static void *esp_realloc_internal(void *ptr, size_t size)
{
  return kmm_realloc(ptr, size);
}

/****************************************************************************
 * Name: esp_calloc_internal
 *
 * Description:
 *   Drivers allocate some continuous blocks of memory
 *
 * Input Parameters:
 *   n    - memory block number
 *   size - memory block size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/

static void *esp_calloc_internal(size_t n, size_t size)
{
  return kmm_calloc(n, size);
}

/****************************************************************************
 * Name: esp_zalloc_internal
 *
 * Description:
 *   Drivers allocate a block of memory and clear it with 0
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/

static void *esp_zalloc_internal(size_t size)
{
  return kmm_zalloc(size);
}

/****************************************************************************
 * Name: esp_malloc_internal
 *
 * Description:
 *   Applications allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

static void *esp_wifi_malloc(size_t size)
{
  return malloc(size);
}

/****************************************************************************
 * Name: esp_realloc_internal
 *
 * Description:
 *   Applications allocate a block of memory by old memory block
 *
 * Input Parameters:
 *   ptr  - old memory pointer
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/

static void *esp_wifi_realloc(void *ptr, size_t size)
{
  return realloc(ptr, size);
}

/****************************************************************************
 * Name: esp_calloc_internal
 *
 * Description:
 *   Applications allocate some continuous blocks of memory
 *
 * Input Parameters:
 *   n    - memory block number
 *   size - memory block size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/

static void *esp_wifi_calloc(size_t n, size_t size)
{
  return calloc(n, size);
}

/****************************************************************************
 * Name: esp_zalloc_internal
 *
 * Description:
 *   Applications allocate a block of memory and clear it with 0
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   New memory pointer
 *
 ****************************************************************************/

static void *esp_wifi_zalloc(size_t size)
{
  return zalloc(size);
}

/****************************************************************************
 * Name: esp_wifi_create_queue
 *
 * Description:
 *   Create WiFi static message queue
 *
 * Input Parameters:
 *   queue_len - queue message number
 *   item_size - message size
 *
 * Returned Value:
 *   WiFi static message queue data pointer
 *
 ****************************************************************************/

static void *esp_wifi_create_queue(int32_t queue_len, int32_t item_size)
{
  wifi_static_queue_t *wifi_queue;

  wifi_queue = kmm_malloc(sizeof(wifi_static_queue_t));
  if (!wifi_queue)
    {
      wlerr("ERROR: Failed to malloc\n");
      return NULL;
    }

  wifi_queue->handle = esp_queue_create(queue_len, item_size);
  if (!wifi_queue->handle)
    {
      wlerr("ERROR: Failed to create queue\n");
      kmm_free(wifi_queue);
      return NULL;
    }

  return wifi_queue;
}

/****************************************************************************
 * Name: esp_wifi_delete_queue
 *
 * Description:
 *   Delete WiFi static message queue
 *
 * Input Parameters:
 *   queue - WiFi static message queue data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_wifi_delete_queue(void *queue)
{
  wifi_static_queue_t *wifi_queue = (wifi_static_queue_t *)queue;

  esp_queue_delete(wifi_queue->handle);
  kmm_free(wifi_queue);
}

/****************************************************************************
 * Name: esp_modem_enter_sleep
 *
 * Description:
 *   Let given module to enter sleep mode
 *
 * Input Parameters:
 *   module - hardware module ID
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_modem_enter_sleep(uint32_t module)
{
  int ret = 0;
  irqstate_t flags;
  uint32_t bit;

  if (module >= (uint32_t)MODEM_MODULE_COUNT)
    {
      return -1;
    }

  bit = 1 << module;

  if (!(s_esp32_module_mask & bit))
    {
      return -1;
    }

  flags = enter_critical_section();

  s_esp32_module_sleep |= bit;
  if (!s_esp32_sleep && (s_esp32_module_sleep == s_esp32_module_mask))
    {
      ret = esp_phy_deinit_rf(PHY_MODEM_MODULE);
      if (ret)
        {
          wlerr("ERROR: Failed to close RF\n");
        }
      else
        {
          s_esp32_sleep = true;
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp_modem_enter_sleep
 *
 * Description:
 *   Let given module to exit from sleep mode
 *
 * Input Parameters:
 *   module - hardware module ID
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_modem_exit_sleep(uint32_t module)
{
  int ret = 0;
  irqstate_t flags;
  uint32_t bit;

  if (module >= (uint32_t)MODEM_MODULE_COUNT)
    {
      return -1;
    }

  bit = 1 << module;

  if (!(s_esp32_module_mask & bit))
    {
      return -1;
    }

  flags = enter_critical_section();

  s_esp32_module_sleep &= ~bit;
  if (s_esp32_sleep)
    {
      ret = esp_phy_rf_init(NULL, PHY_RF_CAL_NONE,
                            NULL, PHY_MODEM_MODULE);
      if (ret)
        {
          wlerr("ERROR: Failed to open RF\n");
        }
      else
        {
          s_esp32_sleep = false;
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp_modem_register_sleep
 *
 * Description:
 *   Regitser given module so that it can enter sleep mode
 *
 * Input Parameters:
 *   module - hardware module ID
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_modem_register_sleep(uint32_t module)
{
  irqstate_t flags;
  uint32_t bit;

  if (module >= (uint32_t)MODEM_MODULE_COUNT)
    {
      return -1;
    }

  bit = 1 << module;

  flags = enter_critical_section();

  if (s_esp32_module_mask & bit)
    {
      /* Has registered and return success */

      return 0;
    }

  s_esp32_module_mask |= bit;
  s_esp32_module_sleep |= bit;

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: esp_modem_deregister_sleep
 *
 * Description:
 *   Deregitser given module so that it can't enter sleep mode
 *
 * Input Parameters:
 *   module - hardware module ID
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

static int32_t esp_modem_deregister_sleep(uint32_t module)
{
  int ret;
  irqstate_t flags;
  uint32_t bit;

  if (module >= (uint32_t)MODEM_MODULE_COUNT)
    {
      return -1;
    }

  bit = 1 << module;

  flags = enter_critical_section();

  if (!(s_esp32_module_mask & bit))
    {
      /* Has deregistered and return success */

      return 0;
    }

  s_esp32_module_mask &= ~bit;
  s_esp32_module_sleep &= ~bit;
  if (!s_esp32_module_mask)
    {
      s_esp32_module_mask = 0;
      if (s_esp32_sleep)
        {
          s_esp32_sleep = false;
          ret = esp_phy_rf_init(NULL, PHY_RF_CAL_NONE,
                                NULL, PHY_MODEM_MODULE);
          if (ret)
            {
              wlerr("ERROR: Failed to open RF\n");
            }
        }
    }

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: esp_coex_status_get
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t esp_coex_status_get(void)
{
  return 0;
}

/****************************************************************************
 * Name: esp_coex_condition_set
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void esp_coex_condition_set(uint32_t type, bool dissatisfy)
{
}

/****************************************************************************
 * Name: esp_coex_wifi_request
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int32_t esp_coex_wifi_request(uint32_t event, uint32_t latency,
                                     uint32_t duration)
{
  return 0;
}

/****************************************************************************
 * Name: esp_coex_wifi_release
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int32_t esp_coex_wifi_release(uint32_t event)
{
  return 0;
}

/****************************************************************************
 * Functions needed by libphy.a
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dport_access_reg_read
 *
 * Description:
 *   Read regitser value safely in SMP
 *
 * Input Parameters:
 *   reg - Regitser address
 *
 * Returned Value:
 *   Regitser value
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp_dport_access_reg_read(uint32_t reg)
{
#ifdef CONFIG_SMP
  DEBUGASSERT(0);
#else
  return getreg32(reg);
#endif
}

/****************************************************************************
 * Name: phy_enter_critical
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

uint32_t IRAM_ATTR phy_enter_critical(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  return flags;
}

/****************************************************************************
 * Name: phy_exit_critical
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

void IRAM_ATTR phy_exit_critical(uint32_t level)
{
  leave_critical_section(level);
}

/****************************************************************************
 * Name: phy_printf
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

int phy_printf(const char *format, ...)
{
  va_list arg;

  va_start(arg, format);
  wlinfo(format, arg);
  va_end(arg);

  return 0;
}

/****************************************************************************
 * Functions needed by libnet80211.a
 ****************************************************************************/

/****************************************************************************
 * Name: net80211_printf
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

int net80211_printf(const char *format, ...)
{
  va_list arg;

  va_start(arg, format);
  vsyslog(LOG_INFO, format, arg);
  va_end(arg);

  return 0;
}

/****************************************************************************
 * Functions needed by libwpa_supplicant.a
 ****************************************************************************/

/****************************************************************************
 * Name: esp_timer_create
 *
 * Description:
 *   Create timer with given arguments
 *
 * Input Parameters:
 *   create_args - Timer arguments data pointer
 *   out_handle  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_create(const esp_timer_create_args_t *create_args,
                         esp_timer_handle_t *out_handle)
{
  struct ets_timer *ets_timer;

  ets_timer = kmm_zalloc(sizeof(struct ets_timer));
  if (!ets_timer)
    {
      wlerr("ERROR: Failed to malloc\n");
      return -1;
    }

  esp_timer_setfn(ets_timer, create_args->callback, create_args->arg);
  if (!ets_timer->priv)
    {
      wlerr("ERROR: Failed to set timer func\n");
      return -1;
    }

  *out_handle = (esp_timer_handle_t)ets_timer;

  return 0;
}

/****************************************************************************
 * Name: esp_timer_start_once
 *
 * Description:
 *   Start timer with one shot mode
 *
 * Input Parameters:
 *   timer      - Timer handle pointer
 *   timeout_us - Timeout value by micro second
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
  struct ets_timer *ets_timer = (struct ets_timer *)timer;

  esp_timer_arm_us(ets_timer, timeout_us, false);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_start_periodic
 *
 * Description:
 *   Start timer with periodic mode
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *   period - Timeout value by micro second
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period)
{
  struct ets_timer *ets_timer = (struct ets_timer *)timer;

  esp_timer_arm_us(ets_timer, period, true);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_stop
 *
 * Description:
 *   Stop timer
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_stop(esp_timer_handle_t timer)
{
  struct ets_timer *ets_timer = (struct ets_timer *)timer;

  esp_timer_disarm(ets_timer);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_delete
 *
 * Description:
 *   Delete timer and free recource
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_delete(esp_timer_handle_t timer)
{
  struct ets_timer *ets_timer = (struct ets_timer *)timer;

  esp32_timer_done(ets_timer);
  kmm_free(ets_timer);

  return 0;
}

/****************************************************************************
 * Name: __assert_func
 *
 * Description:
 *   Delete timer and free recource
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

void __assert_func(const char *file, int line,
                   const char *func, const char *expr)
{
    wlerr("ERROR: Assert failed in %s, %s:%d (%s)",
          func, file, line, expr);
    abort();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_event_send_internal
 *
 * Description:
 *   Post event message to queue
 *
 * Input Parameters:
 *   event_base      - Event set name
 *   event_id        - Event ID
 *   event_data      - Event private data
 *   event_data_size - Event data size
 *   ticks_to_wait   - Waiting system ticks
 *
 * Returned Value:
 *   Task maxium priority
 *
 ****************************************************************************/

int32_t esp_event_send_internal(esp_event_base_t event_base,
                                int32_t event_id,
                                void *event_data,
                                size_t event_data_size,
                                uint32_t ticks_to_wait)
{
  int32_t ret;

  ret = esp_event_post(event_base, event_id, event_data,
                       event_data_size, ticks_to_wait);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_init
 *
 * Description:
 *   Initialize WiFi
 *
 * Input Parameters:
 *   config - Initialization config parameters
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int32_t esp_wifi_init(const wifi_init_config_t *config)
{
  int32_t ret;

  ret = esp_wifi_init_internal(config);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize WiFi error=%d\n", ret);
      return -1;
    }

  ret = esp_supplicant_init();
  if (ret)
    {
      wlerr("ERROR: Failed to initialize WPA supplicant error=%d\n", ret);
      esp_wifi_deinit_internal();
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_deinit
 *
 * Description:
 *   Deinitialize WiFi and free resource
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int32_t esp_wifi_deinit(void)
{
  int ret;

  ret = esp_supplicant_deinit();
  if (ret)
    {
      wlerr("ERROR: Failed to deinitialize supplicant\n");
      return ret;
    }

  ret = esp_wifi_deinit_internal();
  if (ret != 0)
    {
      wlerr("ERROR: Failed to deinitialize WiFi\n");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_send_data
 *
 * Description:
 *   Use WiFi station interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int esp_wifi_sta_send_data(void *pbuf, uint32_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_STA, pbuf, len);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_register_recv_cb
 *
 * Description:
 *   Regitser WiFi receive packet callback function
 *
 * Input Parameters:
 *   input_cb - Receive callback function
 *
 * Returned Value:
 *   0 if success or others if fail
 *
 ****************************************************************************/

int esp_wifi_sta_register_recv_cb(int (*recv_cb)(void *buffer,
                                                 uint16_t len,
                                                 void *eb))
{
  int ret;

  ret = esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t)recv_cb);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_read_mac
 *
 * Description:
 *   Read station interface MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_sta_read_mac(uint8_t *mac)
{
  return esp_read_mac(mac, ESP_MAC_WIFI_STA);
}

/****************************************************************************
 * Name: esp_wifi_free_eb
 *
 * Description:
 *   Free WiFi receive callback input eb pointer
 *
 * Input Parameters:
 *   eb - WiFi receive callback input eb pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_free_eb(void *eb)
{
  esp_wifi_internal_free_rx_buffer(eb);
}

/****************************************************************************
 * Name: esp_wifi_notify_subscribe
 *
 * Description:
 *   Enable event notification
 *
 * Input Parameters:
 *   pid   - Task PID
 *   event - Signal event data pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_notify_subscribe(pid_t pid, FAR struct sigevent *event)
{
  int id;
  struct wifi_notify *notify;
  int ret = -1;

  wlinfo("PID=%d event=%p\n", pid, event);

  if (event->sigev_notify != SIGEV_SIGNAL)
    {
      wlerr("ERROR: sigev_notify %d is invalid\n", event->sigev_signo);
      return -1;
    }

  id = esp_event_id_map(event->sigev_signo);
  if (id < 0)
    {
      wlerr("ERROR: No process event %d\n", event->sigev_signo);
    }
  else
    {
      notify = &s_wifi_notify[id];

      if (notify->assigned)
        {
          wlerr("ERROR: sigev_signo %d has subscribed\n",
                event->sigev_signo);
        }
      else
        {
          if (pid == 0)
            {
              pid = getpid();
              wlinfo("Actual PID=%d\n", pid);
            }

          notify->pid = pid;
          notify->event = *event;
          notify->assigned = true;

          ret = 0;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_adapter_init
 *
 * Description:
 *   Initialize ESP32 WiFi adapter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_adapter_init(void)
{
  int ret;
  wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();

  ret = sem_init(&s_connect_sem, 0, 0);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize sem error=%d\n", errno);
      return -1;
    }

#ifndef CONFIG_ESP32_WIFI_SAVE_PARAM
  init_cfg.nvs_enable = 0;
#endif

  ret = esp_wifi_init(&init_cfg);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize WiFi error=%d\n", ret);
      sem_destroy(&s_connect_sem);
      return -1;
    }

  ret = esp_wifi_set_ps(WIFI_PS_NONE);
  if (ret)
    {
      wlerr("ERROR: Failed to close power save, error=%d\n", ret);
      esp_wifi_deinit();
      sem_destroy(&s_connect_sem);
      return -1;
    }

  sq_init(&s_wifi_evt_queue);

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_set_password
 *
 * Description:
 *   Set WiFi password
 *
 * Input Parameters:
 *   pdata - Password buffer pointer
 *   len   - Password length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_set_password(const uint8_t *pdata, uint8_t len)
{
  memcpy(s_password, pdata, len);
  s_password_len = len;

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_set_ssid
 *
 * Description:
 *   Set WiFi SSID
 *
 * Input Parameters:
 *   pdata - SSID buffer pointer
 *   len   - SSID length
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_set_ssid(const uint8_t *pdata, uint8_t len)
{
  memcpy(s_ssid, pdata, len);
  s_ssid_len = len;

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_connect_internal
 *
 * Description:
 *   Trigger WiFi connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_connect_internal(void)
{
  int ret;
  wifi_config_t wifi_cfg;
  struct timespec timeout;

  if (s_connected)
    {
      wlinfo("INFO: WiFi has connected AP\n");
      return 0;
    }

  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  if (ret)
    {
      wlerr("ERROR: Failed to set station mode error=%d\n", ret);
      esp_wifi_deinit();
      return -1;
    }

  memset(&wifi_cfg, 0, sizeof(wifi_config_t));
  memcpy((char *)wifi_cfg.sta.ssid, s_ssid, s_ssid_len);
  memcpy((char *)wifi_cfg.sta.password, s_password, s_password_len);

  ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg);
  if (ret)
    {
      wlerr("ERROR: Failed to set WiFi config error=%d\n", ret);
      return -1;
    }

  ret = esp_wifi_start();
  if (ret)
    {
      wlerr("ERROR: Failed to set start config error=%d\n", ret);
      return -1;
    }

  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec += WIFI_CONNECT_TIMEOUT;

  ret = sem_timedwait(&s_connect_sem, &timeout);
  if (ret)
    {
      wlerr("ERROR: Failed to wait sem error=%d\n", errno);
      esp_wifi_stop();
      return -1;
    }

  if (!s_connected)
    {
      wlerr("ERROR: Process connection error\n");
      esp_wifi_stop();
      return -1;
    }

  return 0;
}
