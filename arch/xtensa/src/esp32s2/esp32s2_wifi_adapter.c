/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_wifi_adapter.c
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

#include <inttypes.h>
#include <assert.h>
#include <debug.h>
#include <pthread.h>
#include <math.h>
#include <clock/clock.h>
#include <sys/time.h>
#include <irq/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/kthread.h>
#include <nuttx/wqueue.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include "xtensa.h"
#include "esp_attr.h"
#include "hardware/esp32s2_system.h"
#include "hardware/esp32s2_rtccntl.h"

#include "espressif/esp_wireless.h"
#include "espressif/esp_wifi_utils.h"

#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_INITIALIZED_VAL (0x5aa5a55a)
#define RTC_CLK_CAL_FRACT    19     /* Number of fractional bits in values returned by rtc_clk_cal */
#define ets_timer            _ETSTIMER_
#define ESP_MAX_PRIORITIES   25

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Wi-Fi interrupt adapter private data */

struct irq_adpt
{
  void (*func)(void *arg);  /* Interrupt callback function */
  void *arg;                /* Interrupt private data */
};

/* Wi-Fi message queue private data */

struct mq_adpt
{
  struct file mq;           /* Message queue handle */
  uint32_t    msgsize;      /* Message size */
  char        name[16];     /* Message queue name */
};

/* Wi-Fi time private data */

struct time_adpt
{
  time_t      sec;          /* Second value */
  suseconds_t usec;         /* Micro second value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool wifi_env_is_chip(void);
static void wifi_set_intr(int32_t cpu_no, uint32_t intr_source,
                          uint32_t intr_num, int32_t intr_prio);
static void wifi_clear_intr(uint32_t intr_source, uint32_t intr_num);
static void esp_set_isr(int32_t n, void *f, void *arg);
static void esp32s2_ints_on(uint32_t mask);
static void esp32s2_ints_off(uint32_t mask);
static bool wifi_is_from_isr(void);
static void *esp_spin_lock_create(void);
static void esp_spin_lock_delete(void *lock);
static uint32_t esp_wifi_int_disable(void *wifi_int_mux);
static void esp_wifi_int_restore(void *wifi_int_mux, uint32_t tmp);
static void esp_task_yield_from_isr(void);
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
                                          int clear_on_exit,
                                          int wait_for_all_bits,
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
static void *esp_malloc(size_t size);
static void esp_free(void *ptr);
static uint32_t esp_get_free_heap_size(void);
static uint32_t esp_rand(void);
static void wifi_apb80m_request(void);
static void wifi_apb80m_release(void);
static void IRAM_ATTR esp_empty_wrapper(void);
static void esp_phy_enable_wrapper(void);
static void esp_phy_disable_wrapper(void);
static int esp_wifi_read_mac(uint8_t *mac, unsigned int type);
static void esp_timer_arm(void *timer, uint32_t tmout, bool repeat);
static void esp_timer_disarm(void *timer);
static void esp32s2_timer_done(void *timer);
static void esp_timer_setfn(void *timer, void *pfunction, void *parg);
static void esp_timer_arm_us(void *timer, uint32_t us, bool repeat);
static void wifi_reset_mac(void);
static void wifi_clock_enable(void);
static void wifi_clock_disable(void);
static int64_t esp32s2_timer_get_time(void);
static int esp_nvs_set_i8(uint32_t handle, const char *key,
                          int8_t value);
static int esp_nvs_get_i8(uint32_t handle, const char *key,
                          int8_t *out_value);
static int esp_nvs_set_u8(uint32_t handle, const char *key,
                          uint8_t value);
static int esp_nvs_get_u8(uint32_t handle, const char *key,
                          uint8_t *out_value);
static int esp_nvs_set_u16(uint32_t handle, const char *key,
                           uint16_t value);
static int esp_nvs_get_u16(uint32_t handle, const char *key,
                           uint16_t *out_value);
static int esp_nvs_open(const char *name, unsigned int open_mode,
                        uint32_t *out_handle);
static void esp_nvs_close(uint32_t handle);
static int esp_nvs_commit(uint32_t handle);
static int esp_nvs_set_blob(uint32_t handle, const char *key,
                            const void *value, size_t length);
static int esp_nvs_get_blob(uint32_t handle, const char *key,
                            void *out_value, size_t *length);
static int esp_nvs_erase_key(uint32_t handle, const char *key);
static int esp_get_random(uint8_t *buf, size_t len);
static int esp_get_time(void *t);
static uint32_t esp_clk_slowclk_cal_get_wrapper(void);
static void esp_log_writev_wrapper(unsigned int level, const char *tag,
                                   const char *format, va_list args);
static void esp_log_write_wrapper(unsigned int level,
                                  const char *tag,
                                  const char *format, ...);
static int32_t esp_event_post_wrapper(const char *event_base,
                                      int32_t event_id,
                                      void *event_data,
                                      size_t event_data_size,
                                      uint32_t ticks_to_wait);
static void *esp_malloc_internal(size_t size);
static void *esp_realloc_internal(void *ptr, size_t size);
static void *esp_calloc_internal(size_t n, size_t size);
static void *esp_zalloc_internal(size_t size);
static void *esp_wifi_malloc(size_t size);
static void *esp_wifi_realloc(void *ptr, size_t size);
static void *esp_wifi_calloc(size_t n, size_t size);
static void *esp_wifi_zalloc(size_t size);
static void *esp_wifi_create_queue(int queue_len, int item_size);
static void esp_wifi_delete_queue(void *queue);
static int coex_init_wrapper(void);
static void coex_deinit_wrapper(void);
static int coex_enable_wrapper(void);
static void coex_disable_wrapper(void);
static uint32_t coex_status_get_wrapper(void);
static int coex_wifi_request_wrapper(uint32_t event, uint32_t latency,
                                     uint32_t duration);
static int coex_wifi_release_wrapper(uint32_t event);
static unsigned long esp_random_ulong(void);
static int coex_wifi_channel_set_wrapper(uint8_t primary, uint8_t secondary);
static int coex_event_duration_get_wrapper(uint32_t event,
                                        uint32_t *duration);
static int coex_pti_get_wrapper(uint32_t event, uint8_t *pti);
static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                            uint32_t status);
static void coex_schm_status_bit_set_wrapper(uint32_t type,
                                          uint32_t status);
static int coex_schm_interval_set_wrapper(uint32_t interval);
static uint32_t coex_schm_interval_get_wrapper(void);
static uint8_t coex_schm_curr_period_get_wrapper(void);
static void *coex_schm_curr_phase_get_wrapper(void);
static int coex_register_start_cb_wrapper(int (* cb)(void));
static int coex_schm_process_restart_wrapper(void);
static int coex_schm_register_cb_wrapper(int type, int(*cb)(int));
static int coex_schm_flexible_period_set_wrapper(uint8_t period);
static uint8_t coex_schm_flexible_period_get_wrapper(void);
static void * coex_schm_get_phase_by_idx_wrapper(int phase_idx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Wi-Fi OS adapter data */

wifi_osi_funcs_t g_wifi_osi_funcs =
{
  ._version = ESP_WIFI_OS_ADAPTER_VERSION,
  ._env_is_chip = wifi_env_is_chip,
  ._set_intr = wifi_set_intr,
  ._clear_intr = wifi_clear_intr,
  ._set_isr = esp_set_isr,
  ._ints_on = esp32s2_ints_on,
  ._ints_off = esp32s2_ints_off,
  ._is_from_isr = wifi_is_from_isr,
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
  ._free = esp_free,
  ._event_post = esp_event_post_wrapper,
  ._get_free_heap_size = esp_get_free_heap_size,
  ._rand = esp_rand,
  ._dport_access_stall_other_cpu_start_wrap = esp_empty_wrapper,
  ._dport_access_stall_other_cpu_end_wrap = esp_empty_wrapper,
  ._wifi_apb80m_request = wifi_apb80m_request,
  ._wifi_apb80m_release = wifi_apb80m_release,
  ._phy_disable = esp_phy_disable_wrapper,
  ._phy_enable = esp_phy_enable_wrapper,
  ._phy_common_clock_enable = esp_phy_common_clock_enable,
  ._phy_common_clock_disable = esp_phy_common_clock_disable,
  ._phy_update_country_info = esp_phy_update_country_info,
  ._read_mac = esp_wifi_read_mac,
  ._timer_arm = esp_timer_arm,
  ._timer_disarm = esp_timer_disarm,
  ._timer_done = esp32s2_timer_done,
  ._timer_setfn = esp_timer_setfn,
  ._timer_arm_us = esp_timer_arm_us,
  ._wifi_reset_mac = wifi_reset_mac,
  ._wifi_clock_enable = wifi_clock_enable,
  ._wifi_clock_disable = wifi_clock_disable,
  ._wifi_rtc_enable_iso = esp_empty_wrapper,
  ._wifi_rtc_disable_iso = esp_empty_wrapper,
  ._esp_timer_get_time = esp32s2_timer_get_time,
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
  ._random = esp_random_ulong,
  ._slowclk_cal_get = esp_clk_slowclk_cal_get_wrapper,
  ._log_write = esp_log_write_wrapper,
  ._log_writev = esp_log_writev_wrapper,
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
  ._coex_init = coex_init_wrapper,
  ._coex_deinit = coex_deinit_wrapper,
  ._coex_enable = coex_enable_wrapper,
  ._coex_disable = coex_disable_wrapper,
  ._coex_status_get = coex_status_get_wrapper,
  ._coex_wifi_request = coex_wifi_request_wrapper,
  ._coex_wifi_release = coex_wifi_release_wrapper,
  ._coex_wifi_channel_set = coex_wifi_channel_set_wrapper,
  ._coex_event_duration_get = coex_event_duration_get_wrapper,
  ._coex_pti_get = coex_pti_get_wrapper,
  ._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
  ._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
  ._coex_schm_interval_set = coex_schm_interval_set_wrapper,
  ._coex_schm_interval_get = coex_schm_interval_get_wrapper,
  ._coex_schm_curr_period_get = coex_schm_curr_period_get_wrapper,
  ._coex_schm_curr_phase_get = coex_schm_curr_phase_get_wrapper,
  ._coex_register_start_cb = coex_register_start_cb_wrapper,
  ._coex_schm_process_restart = coex_schm_process_restart_wrapper,
  ._coex_schm_register_cb = coex_schm_register_cb_wrapper,
  ._coex_schm_flexible_period_set = coex_schm_flexible_period_set_wrapper,
  ._coex_schm_flexible_period_get = coex_schm_flexible_period_get_wrapper,
  ._coex_schm_get_phase_by_idx = coex_schm_get_phase_by_idx_wrapper,
  ._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_int_adpt_cb
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

static int esp_int_adpt_cb(int irq, void *context, void *arg)
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
 *   semphr - Semaphore data pointer
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
  struct timespec ts;

  clock_ticks2time(&ts, ticks);
  clock_timespec_add(timespec, &ts, timespec);
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
  uint32_t tmp;
  struct irq_adpt *adapter;
  int irq = n + XTENSA_IRQ_FIRSTPERIPH;

  wlinfo("n=%ld f=%p arg=%p", n, f, arg);

  if (g_irqvector[irq].handler &&
      g_irqvector[irq].handler != irq_unexpected_isr)
    {
      wlinfo("irq=%d has been set handler=%p\n", irq,
             g_irqvector[irq].handler);
      return;
    }

  tmp = sizeof(struct irq_adpt);
  adapter = kmm_malloc(tmp);
  if (!adapter)
    {
      wlerr("Failed to alloc %" PRIu32 " memory\n", tmp);
      PANIC();
      return;
    }

  adapter->func = f;
  adapter->arg = arg;

  ret = irq_attach(ESP32S2_IRQ_MAC, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("Failed to attach IRQ %d\n", irq);
      PANIC();
      return;
    }

  ret = irq_attach(ESP32S2_IRQ_PWR, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("Failed to attach IRQ %d\n", irq);
      PANIC();
      return;
    }
}

/****************************************************************************
 * Name: esp32s2_ints_on
 *
 * Description:
 *   Enable Wi-Fi interrupt
 *
 * Input Parameters:
 *   mask - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s2_ints_on(uint32_t mask)
{
  int irq = __builtin_ffs(mask) - 1;

  wlinfo("INFO mask=%08lx irq=%d\n", mask, irq);

  up_enable_irq(ESP32S2_IRQ_MAC);
  up_enable_irq(ESP32S2_IRQ_PWR);
}

/****************************************************************************
 * Name: esp32s2_ints_off
 *
 * Description:
 *   Disable Wi-Fi interrupt
 *
 * Input Parameters:
 *   mask - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s2_ints_off(uint32_t mask)
{
  uint32_t irq = __builtin_ffs(mask) - 1;

  wlinfo("INFO mask=%08" PRIu32 " irq=%" PRIu32 "\n", mask, irq);

  up_disable_irq(ESP32S2_IRQ_MAC);
  up_disable_irq(ESP32S2_IRQ_PWR);
}

/****************************************************************************
 * Name: wifi_is_from_isr
 *
 * Description:
 *   Check current is in interrupt
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if in interrupt or false if not
 *
 ****************************************************************************/

static bool IRAM_ATTR wifi_is_from_isr(void)
{
  return up_interrupt_context();
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
  spinlock_t *lock;
  int tmp;

  tmp = sizeof(*lock);
  lock = kmm_malloc(tmp);
  if (!lock)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      DEBUGPANIC();
    }

  spin_lock_init(lock);

  return lock;
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
  kmm_free(lock);
}

/****************************************************************************
 * Name: esp_wifi_int_disable
 *
 * Description:
 *   Enter critical section by disabling interrupts and taking the spin lock
 *   if in SMP mode.
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

  flags = spin_lock_irqsave((spinlock_t *)wifi_int_mux);

  return (uint32_t)flags;
}

/****************************************************************************
 * Name: esp_wifi_int_restore
 *
 * Description:
 *   Exit from critical section by enabling interrupts and releasing the spin
 *   lock if in SMP mode.
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

  spin_unlock_irqrestore((spinlock_t *)wifi_int_mux, flags);
}

/****************************************************************************
 * Name: esp_task_yield_from_isr
 *
 * Description:
 *   Perform a solicited context switch on FreeRTOS. Do nothing in NuttX.
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
 *   Semaphore data pointer
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
      wlerr("Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = nxsem_init(sem, 0, init);
  if (ret)
    {
      wlerr("Failed to initialize sem error=%d\n", ret);
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
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_semphr_delete(void *semphr)
{
  sem_t *sem = (sem_t *)semphr;

  nxsem_destroy(sem);
  kmm_free(sem);
}

/****************************************************************************
 * Name: esp_semphr_take
 *
 * Description:
 *   Wait semaphore within a certain period of time
 *
 * Input Parameters:
 *   semphr          - Semaphore data pointer
 *   block_time_tick - Wait system ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_semphr_take(void *semphr, uint32_t block_time_tick)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  if (block_time_tick == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = nxsem_wait(sem);
    }
  else
    {
      if (block_time_tick > 0)
        {
          ret = nxsem_tickwait(sem, block_time_tick);
        }
      else
        {
          ret = nxsem_trywait(sem);
        }
    }

  if (ret)
    {
      wlerr("ERROR: Failed to wait sem in %" PRIu32 " ticks. Error=%d\n",
            block_time_tick, ret);
    }

  return nuttx_err_to_common_err(ret);
}

/****************************************************************************
 * Name: esp_semphr_give
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

static int32_t esp_semphr_give(void *semphr)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  ret = nxsem_post(sem);
  if (ret)
    {
      wlerr("Failed to post sem error=%d\n", ret);
    }

  return nuttx_err_to_common_err(ret);
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
 *   Semaphore data pointer
 *
 ****************************************************************************/

static void *esp_thread_semphr_get(void)
{
  static int wifi_task_key = -1;
  int ret;
  void *sem;

  if (wifi_task_key < 0)
    {
      ret = task_tls_alloc(esp_thread_semphr_free);
      if (ret < 0)
        {
          wlerr("Failed to create task local key\n");
          return NULL;
        }

      wifi_task_key = ret;
    }

  sem = (void *)task_tls_get_value(wifi_task_key);
  if (sem == NULL)
    {
      sem = esp_semphr_create(1, 0);
      if (!sem)
        {
          wlerr("Failed to create semaphore\n");
          return NULL;
        }

      ret = task_tls_set_value(wifi_task_key, (uintptr_t)sem);
      if (ret != OK)
        {
          wlerr("Failed to save semaphore on task local storage: %d\n", ret);
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
      wlerr("Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = pthread_mutex_init(mutex, NULL);
  if (ret)
    {
      wlerr("Failed to initialize mutex error=%d\n", ret);
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
      wlerr("Failed to initialize attr error=%d\n", ret);
      return NULL;
    }

  ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  if (ret)
    {
      wlerr("Failed to set attr type error=%d\n", ret);
      return NULL;
    }

  tmp = sizeof(pthread_mutex_t);
  mutex = kmm_malloc(tmp);
  if (!mutex)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = pthread_mutex_init(mutex, &attr);
  if (ret)
    {
      wlerr("Failed to initialize mutex error=%d\n", ret);
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
      wlerr("Failed to lock mutex error=%d\n", ret);
    }

  return nuttx_err_to_common_err(ret);
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
      wlerr("Failed to unlock mutex error=%d\n", ret);
    }

  return nuttx_err_to_common_err(ret);
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
  struct mq_adpt *mq_adpt;
  int ret;

  mq_adpt = kmm_malloc(sizeof(struct mq_adpt));
  if (!mq_adpt)
    {
      wlerr("Failed to kmm_malloc\n");
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
      wlerr("Failed to create mqueue\n");
      kmm_free(mq_adpt);
      return NULL;
    }

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

  file_mq_close(&mq_adpt->mq);
  file_mq_unlink(mq_adpt->name);
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
      /* Wi-Fi interrupt function will call this adapter function to send
       * message to message queue, so here we should call kernel API
       * instead of application API
       */

      ret = file_mq_send(&mq_adpt->mq, (const char *)item,
                         mq_adpt->msgsize, prio);
      if (ret < 0)
        {
          wlerr("Failed to send message to mqueue error=%d\n",
               ret);
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to get time\n");
          return false;
        }

      if (ticks)
        {
          esp_update_time(&timeout, ticks);
        }

      ret = file_mq_timedsend(&mq_adpt->mq, (const char *)item,
                              mq_adpt->msgsize, prio, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to timedsend message to mqueue error=%d\n",
               ret);
        }
    }

  return nuttx_err_to_common_err(ret);
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
 *   hptw  - Unused.
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_queue_send_from_isr(void *queue, void *item, void *hptw)
{
  *(int *)hptw = 0;

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
 * Name: esp_queue_send_from_to_front
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
      ret = file_mq_receive(&mq_adpt->mq, (char *)item,
                            mq_adpt->msgsize, &prio);
      if (ret < 0)
        {
          wlerr("Failed to receive from mqueue error=%d\n", ret);
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to get time\n");
          return false;
        }

      if (ticks)
        {
          esp_update_time(&timeout, ticks);
        }

      ret = file_mq_timedreceive(&mq_adpt->mq, (char *)item,
                                 mq_adpt->msgsize, &prio, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to timedreceive from mqueue error=%d\n",
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

  ret = file_mq_getattr(&mq_adpt->mq, &attr);
  if (ret < 0)
    {
      wlerr("Failed to get attr from mqueue error=%d\n", ret);
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
  DEBUGPANIC();

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
  DEBUGPANIC();
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
  DEBUGPANIC();

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
  DEBUGPANIC();

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
                                          int clear_on_exit,
                                          int wait_for_all_bits,
                                          uint32_t block_time_tick)
{
  DEBUGPANIC();

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
  uint32_t target_prio = prio;

  if (target_prio < ESP_MAX_PRIORITIES)
    {
      target_prio += esp_task_get_max_priority() - ESP_MAX_PRIORITIES;
    }

  pid = kthread_create(name, target_prio, stack_depth, entry,
                       (char * const *)param);
  if (pid > 0)
    {
      if (task_handle != NULL)
        {
          *((int *)task_handle) = pid;
        }
    }
  else
    {
      wlerr("Failed to create task\n");
    }

  return pid > 0;
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

  nxsched_usleep(us);
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
  pid_t pid = nxsched_gettid();

  return (void *)((uintptr_t)pid);
}

/****************************************************************************
 * Name: esp_task_get_max_priority
 *
 * Description:
 *   Get OS task maximum priority
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Task maximum priority
 *
 ****************************************************************************/

static int32_t esp_task_get_max_priority(void)
{
  return SCHED_PRIORITY_MAX;
}

/****************************************************************************
 * Name: esp_malloc
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

static void *esp_malloc(size_t size)
{
  return kmm_malloc(size);
}

/****************************************************************************
 * Name: esp_free
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

static void esp_free(void *ptr)
{
#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  if (xtensa_imm_heapmember(ptr))
    {
      xtensa_imm_free(ptr);
    }
  else
#endif
#ifdef CONFIG_MM_KERNEL_HEAP
  if (kmm_heapmember(ptr))
#endif
    {
      kmm_free(ptr);
    }
#ifdef CONFIG_MM_KERNEL_HEAP
  else
    {
      free(ptr);
    }
#endif
}

/****************************************************************************
 * Name: wifi_env_is_chip
 *
 * Description:
 *   Config chip environment
 *
 * Returned Value:
 *   True if on chip or false if on FPGA.
 *
 ****************************************************************************/

static bool wifi_env_is_chip(void)
{
  return true;
}

/****************************************************************************
 * Name: wifi_set_intr
 *
 * Description:
 *   Do nothing
 *
 * Input Parameters:
 *     cpu_no      - The CPU which the interrupt number belongs.
 *     intr_source - The interrupt hardware source number.
 *     intr_num    - The interrupt number CPU.
 *     intr_prio   - The interrupt priority.
 *
 * Returned Value:
 *     None
 *
 ****************************************************************************/

static void wifi_set_intr(int32_t cpu_no, uint32_t intr_source,
                          uint32_t intr_num, int32_t intr_prio)
{
  wlinfo("cpu_no=%" PRId32 ", intr_source=%" PRIu32
         ", intr_num=%" PRIu32 ", intr_prio=%" PRId32 "\n",
         cpu_no, intr_source, intr_num, intr_prio);
}

/****************************************************************************
 * Name: wifi_clear_intr
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void IRAM_ATTR wifi_clear_intr(uint32_t intr_source,
                                      uint32_t intr_num)
{
}

/****************************************************************************
 * Name: esp_event_post_wrapper
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

static int32_t esp_event_post_wrapper(const char *event_base,
                                      int32_t event_id,
                                      void *event_data,
                                      size_t event_data_size,
                                      uint32_t ticks_to_wait)
{
  return (int32_t)esp_event_post(event_base,
                                 event_id,
                                 event_data,
                                 event_data_size,
                                 ticks_to_wait);
}

/****************************************************************************
 * Name: esp_get_free_heap_size
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

static uint32_t esp_get_free_heap_size(void)
{
  struct mallinfo info;

  info = kmm_mallinfo();
  return info.fordblks;
}

/****************************************************************************
 * Name: wifi_apb80m_request
 *
 * Description:
 *   Take Wi-Fi lock in auto-sleep
 *
 ****************************************************************************/

static void wifi_apb80m_request(void)
{
#ifdef CONFIG_ESP32S2_AUTO_SLEEP
  esp32s2_pm_lockacquire();
#endif
}

/****************************************************************************
 * Name: wifi_apb80m_release
 *
 * Description:
 *   Release Wi-Fi lock in auto-sleep
 *
 ****************************************************************************/

static void wifi_apb80m_release(void)
{
#ifdef CONFIG_ESP32S2_AUTO_SLEEP
  esp32s2_pm_lockrelease();
#endif
}

/****************************************************************************
 * Name: esp_empty_wrapper
 *
 * Description:
 *   This is a wrapper for unused functions in ESP WiFi compatibility.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp_empty_wrapper(void)
{
}

/****************************************************************************
 * Name: esp_phy_enable_wrapper
 *
 * Description:
 *   This is a wrapper for enabling the ESP PHY. It calls the esp_phy_enable
 *   function with PHY_MODEM_WIFI as the argument.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_phy_enable_wrapper(void)
{
    esp_phy_enable(PHY_MODEM_WIFI);
}

/****************************************************************************
 * Name: esp_phy_disable_wrapper
 *
 * Description:
 *   This is a wrapper for disabling the ESP PHY. It first calls the
 *   phy_wifi_enable_set function with 0 as the argument.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_phy_disable_wrapper(void)
{
    esp_phy_disable(PHY_MODEM_WIFI);
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

static int esp_wifi_read_mac(uint8_t *mac, unsigned int type)
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
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;
  esp_timer_handle_t esp_timer = (esp_timer_handle_t)ets_timer->timer_arg;

  if (ets_timer->timer_expire == TIMER_INITIALIZED_VAL)
    {
      esp_timer_stop(esp_timer);
    }
}

/****************************************************************************
 * Name: esp32s2_timer_done
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

static void esp32s2_timer_done(void *ptimer)
{
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;
  esp_timer_handle_t esp_timer = (esp_timer_handle_t)ets_timer->timer_arg;

  if (ets_timer->timer_expire == TIMER_INITIALIZED_VAL)
    {
      ets_timer->timer_expire = 0;
      esp_timer_delete(esp_timer);
      ets_timer->timer_arg = NULL;
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
  int ret;
  esp_timer_handle_t esp_timer;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;

  if (ets_timer->timer_expire != TIMER_INITIALIZED_VAL)
    {
      ets_timer->timer_arg = NULL;
    }

  if (ets_timer->timer_arg == NULL)
    {
      const esp_timer_create_args_t create_args =
        {
          .callback = pfunction,
          .arg = parg,
          .name = "ETSTimer",
          .dispatch_method = ESP_TIMER_TASK
        };

      ret = esp_timer_create(&create_args, &esp_timer);
      if (ret)
        {
          wlerr("Failed to create ets_timer error=%d\n", ret);
        }
      else
        {
          ets_timer->timer_arg = esp_timer;
          ets_timer->timer_expire = TIMER_INITIALIZED_VAL;
        }
    }
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
  int ret;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;
  esp_timer_handle_t esp_timer = (esp_timer_handle_t)ets_timer->timer_arg;

  if (ets_timer->timer_expire == TIMER_INITIALIZED_VAL)
    {
      esp_timer_stop(esp_timer);
      if (!repeat)
        {
          ret = esp_timer_start_once(esp_timer, us);
        }
      else
        {
          ret = esp_timer_start_periodic(esp_timer, us);
        }

      if (ret)
        {
          wlerr("Fail to start %s timer error%d\n",
                repeat ? "periodic" : "once",
                ret);
        }
    }
}

/****************************************************************************
 * Name: wifi_reset_mac
 *
 * Description:
 *   Reset Wi-Fi hardware MAC
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wifi_reset_mac(void)
{
  periph_module_reset(PERIPH_WIFI_MODULE);
}

/****************************************************************************
 * Name: wifi_clock_enable
 *
 * Description:
 *   Enable Wi-Fi clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wifi_clock_enable(void)
{
  wifi_module_enable();
}

/****************************************************************************
 * Name: wifi_clock_disable
 *
 * Description:
 *   Disable Wi-Fi clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wifi_clock_disable(void)
{
  wifi_module_disable();
}

/****************************************************************************
 * Name: esp32s2_timer_get_time
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

int64_t esp32s2_timer_get_time(void)
{
  return (int64_t)rt_timer_time_us();
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

static int esp_nvs_set_i8(uint32_t handle,
                          const char *key,
                          int8_t value)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_get_i8(uint32_t handle,
                          const char *key,
                          int8_t *out_value)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_set_u8(uint32_t handle,
                          const char *key,
                          uint8_t value)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_get_u8(uint32_t handle,
                          const char *key,
                          uint8_t *out_value)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_set_u16(uint32_t handle,
                           const char *key,
                           uint16_t value)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_get_u16(uint32_t handle,
                          const char *key,
                          uint16_t *out_value)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_open(const char *name,
                        unsigned int open_mode,
                        uint32_t *out_handle)
{
  DEBUGPANIC();

  return -1;
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
  DEBUGPANIC();
}

/****************************************************************************
 * Name: esp_nvs_commit
 *
 * Description:
 *   This function has no practical effect
 *
 ****************************************************************************/

static int esp_nvs_commit(uint32_t handle)
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

static int esp_nvs_set_blob(uint32_t handle,
                            const char *key,
                            const void *value,
                            size_t length)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_get_blob(uint32_t handle,
                            const char *key,
                            void *out_value,
                            size_t *length)
{
  DEBUGPANIC();

  return -1;
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

static int esp_nvs_erase_key(uint32_t handle, const char *key)
{
  DEBUGPANIC();

  return -1;
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

static int esp_get_random(uint8_t *buf, size_t len)
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

static int esp_get_time(void *t)
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
      wlerr("Failed to get time of day\n");
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
 * Name: esp_clk_slowclk_cal_get_wrapper
 *
 * Description:
 *   Get the calibration value of RTC slow clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The calibration value obtained using rtc_clk_cal
 *
 ****************************************************************************/

static uint32_t esp_clk_slowclk_cal_get_wrapper(void)
{
  /* The bit width of WiFi light sleep clock calibration is 12 while the one
   * of the system is 19. It should shift 19 - 12 = 7.
   */

  if (REG_GET_FIELD(SYSTEM_BT_LPCK_DIV_FRAC_REG, SYSTEM_LPCLK_SEL_XTAL))
    {
      uint64_t time_per_us = 1000000ULL;
      return (((time_per_us << RTC_CLK_CAL_FRACT) / (MHZ)) >>
              (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
    }
    else
    {
      return (getreg32(RTC_SLOW_CLK_CAL_REG) >>
              (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
    }
}

/****************************************************************************
 * Name: esp_log_writev_wrapper
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

static void esp_log_writev_wrapper(unsigned int level, const char *tag,
                                   const char *format, va_list args)
{
  esp_log_level_t max_level;

#if defined (CONFIG_DEBUG_WIRELESS_INFO)
  max_level = ESP_LOG_VERBOSE;
#elif defined (CONFIG_DEBUG_WIRELESS_WARN)
  max_level = ESP_LOG_WARN;
#elif defined (CONFIG_DEBUG_WIRELESS_ERROR)
  max_level = ESP_LOG_ERROR;
#else
  max_level = ESP_LOG_NONE;
#endif

  if (level <= max_level)
    {
      esp_log_writev(level, tag, format, args);
    }
}

/****************************************************************************
 * Name: esp_log_write_wrapper
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

static void esp_log_write_wrapper(unsigned int level,
                                  const char *tag,
                                  const char *format, ...)
{
  esp_log_level_t max_level;

#if defined (CONFIG_DEBUG_WIRELESS_INFO)
  max_level = ESP_LOG_VERBOSE;
#elif defined (CONFIG_DEBUG_WIRELESS_WARN)
  max_level = ESP_LOG_WARN;
#elif defined (CONFIG_DEBUG_WIRELESS_ERROR)
  max_level = ESP_LOG_ERROR;
#else
  max_level = ESP_LOG_NONE;
#endif

  if (level <= max_level)
    {
      va_list list;
      va_start(list, format);
      esp_log_writev(level, tag, format, list);
      va_end(list);
    }
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
#ifdef CONFIG_MM_KERNEL_HEAP
  return kmm_malloc(size);
#elif defined(CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
  return xtensa_imm_malloc(size);
#else
  void *ptr = kmm_malloc(size);

  if (ptr != NULL)
    {
      if (esp32s2_ptr_extram(ptr))
        {
          kmm_free(ptr);
          return NULL;
        }
    }

  return ptr;
#endif
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
#ifdef CONFIG_MM_KERNEL_HEAP
  return kmm_realloc(ptr, size);
#elif defined(CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
  return xtensa_imm_realloc(ptr, size);
#else
  void *old_ptr = ptr;
  void *new_ptr = NULL;
  size_t old_size = 0;
  if (size == 0)
    {
      kmm_free(ptr);
      return NULL;
    }

  new_ptr = kmm_malloc(size);
  if (new_ptr != NULL)
    {
      if (esp32s2_ptr_extram(new_ptr))
        {
          kmm_free(new_ptr);
          return NULL;
        }

      old_size = kmm_malloc_size(old_ptr);
      DEBUGASSERT(old_size > 0);
      memcpy(new_ptr, old_ptr, MIN(old_size, size));
      kmm_free(old_ptr);
      return new_ptr;
    }

  return NULL;
#endif
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
#ifdef CONFIG_MM_KERNEL_HEAP
  return kmm_calloc(n, size);
#elif defined(CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
  return xtensa_imm_calloc(n, size);
#else
  void *ptr = kmm_calloc(n, size);
  if (ptr != NULL)
    {
      if (esp32s2_ptr_extram(ptr))
        {
          kmm_free(ptr);
          return NULL;
        }
    }

  return ptr;
#endif
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
#ifdef CONFIG_MM_KERNEL_HEAP
  return kmm_zalloc(size);
#elif defined(CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
  return xtensa_imm_zalloc(size);
#else
  void *ptr = kmm_zalloc(size);
  if (ptr != NULL)
    {
      if (esp32s2_ptr_extram(ptr))
        {
          kmm_free(ptr);
          return NULL;
        }
    }

  return ptr;
#endif
}

/****************************************************************************
 * Name: esp_wifi_malloc
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
  return kmm_malloc(size);
}

/****************************************************************************
 * Name: esp_wifi_realloc
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
  return kmm_realloc(ptr, size);
}

/****************************************************************************
 * Name: esp_wifi_calloc
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
  return kmm_calloc(n, size);
}

/****************************************************************************
 * Name: esp_wifi_zalloc
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
  return kmm_zalloc(size);
}

/****************************************************************************
 * Name: esp_wifi_create_queue
 *
 * Description:
 *   Create Wi-Fi static message queue
 *
 * Input Parameters:
 *   queue_len - queue message number
 *   item_size - message size
 *
 * Returned Value:
 *   Wi-Fi static message queue data pointer
 *
 ****************************************************************************/

static void *esp_wifi_create_queue(int queue_len, int item_size)
{
  wifi_static_queue_t *wifi_queue;

  wifi_queue = kmm_malloc(sizeof(wifi_static_queue_t));
  if (!wifi_queue)
    {
      wlerr("Failed to kmm_malloc\n");
      return NULL;
    }

  wifi_queue->handle = esp_queue_create(queue_len, item_size);
  if (!wifi_queue->handle)
    {
      wlerr("Failed to create queue\n");
      kmm_free(wifi_queue);
      return NULL;
    }

  return wifi_queue;
}

/****************************************************************************
 * Name: esp_wifi_delete_queue
 *
 * Description:
 *   Delete Wi-Fi static message queue
 *
 * Input Parameters:
 *   queue - Wi-Fi static message queue data pointer
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
 * Name: coex_init_wrapper
 *
 * Description:
 *   Init software coexist
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_init_wrapper(void)
{
  return 0;
}

/****************************************************************************
 * Name: coex_deinit_wrapper
 *
 * Description:
 *   De-init software coexist
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void coex_deinit_wrapper(void)
{
}

/****************************************************************************
 * Name: coex_enable_wrapper
 *
 * Description:
 *   Enable software coexist
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_enable_wrapper(void)
{
  return 0;
}

/****************************************************************************
 * Name: coex_disable_wrapper
 *
 * Description:
 *   Disable software coexist
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void coex_disable_wrapper(void)
{
}

/****************************************************************************
 * Name: coex_status_get_wrapper
 *
 * Description:
 *   Get software coexist status.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static IRAM_ATTR uint32_t coex_status_get_wrapper(void)
{
  return 0;
}

/****************************************************************************
 * Name: coex_wifi_request_wrapper
 *
 * Description:
 *   Request Wi-Fi coexistence.
 *
 * Input Parameters:
 *   event    - WiFi event
 *   latency  - WiFi will request coexistence after latency
 *   duration - duration for WiFi to request coexistence
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_wifi_request_wrapper(uint32_t event, uint32_t latency,
                                     uint32_t duration)
{
  return 0;
}

/****************************************************************************
 * Name: coex_wifi_release_wrapper
 *
 * Description:
 *   Release Wi-Fi coexistence.
 *
 * Input Parameters:
 *   event    - WiFi event
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static IRAM_ATTR int coex_wifi_release_wrapper(uint32_t event)
{
  return 0;
}

/****************************************************************************
 * Name: coex_wifi_channel_set_wrapper
 *
 * Description:
 *   Set Wi-Fi channel to coexistence module.
 *
 * Input Parameters:
 *   primary   - WiFi primary channel
 *   secondary - WiFi secondary channel
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_wifi_channel_set_wrapper(uint8_t primary, uint8_t secondary)
{
  return 0;
}

/****************************************************************************
 * Name: coex_event_duration_get_wrapper
 *
 * Description:
 *   Get coexistence event duration.
 *
 * Input Parameters:
 *   event    - Coexistence event
 *   duration - Coexistence event duration
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_event_duration_get_wrapper(uint32_t event,
                                           uint32_t *duration)
{
  return 0;
}

/****************************************************************************
 * Name: coex_pti_get_wrapper
 *
 * Description:
 *   Get coexistence event priority.
 *
 * Input Parameters:
 *   event - Coexistence event
 *   pti   - Coexistence event priority
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_pti_get_wrapper(uint32_t event, uint8_t *pti)
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_status_bit_clear_wrapper
 *
 * Description:
 *   Clear coexistence status.
 *
 * Input Parameters:
 *   type   - Coexistence status type
 *   status - Coexistence status
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                               uint32_t status)
{
}

/****************************************************************************
 * Name: coex_schm_status_bit_set_wrapper
 *
 * Description:
 *   Set coexistence status.
 *
 * Input Parameters:
 *   type   - Coexistence status type
 *   status - Coexistence status
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
}

/****************************************************************************
 * Name: coex_schm_interval_set_wrapper
 *
 * Description:
 *   Set coexistence scheme interval.
 *
 * Input Parameters:
 *   interval - Coexistence scheme interval
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static IRAM_ATTR int coex_schm_interval_set_wrapper(uint32_t interval)
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_interval_get_wrapper
 *
 * Description:
 *   Get coexistence scheme interval.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Coexistence scheme interval
 *
 ****************************************************************************/

static uint32_t coex_schm_interval_get_wrapper(void)
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_curr_period_get_wrapper
 *
 * Description:
 *   Get current coexistence scheme period.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Coexistence scheme period
 *
 ****************************************************************************/

static uint8_t coex_schm_curr_period_get_wrapper(void)
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_curr_phase_get_wrapper
 *
 * Description:
 *   Get current coexistence scheme phase.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Coexistence scheme phase
 *
 ****************************************************************************/

static void *coex_schm_curr_phase_get_wrapper(void)
{
  return NULL;
}

/****************************************************************************
 * Name: coex_register_start_cb_wrapper
 *
 * Description:
 *   Register Wi-Fi callback for coexistence starts.
 *
 * Input Parameters:
 *   cb - WiFi callback
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_register_start_cb_wrapper(int (* cb)(void))
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_process_restart_wrapper
 *
 * Description:
 *   Restart current coexistence scheme.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_schm_process_restart_wrapper(void)
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_register_cb_wrapper
 *
 * Description:
 *   Register callback for coexistence scheme.
 *
 * Input Parameters:
 *   type     - callback type
 *   callback - callback
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_schm_register_cb_wrapper(int type, int(*cb)(int))
{
  return 0;
}

/****************************************************************************
 * Name: coex_schm_flexible_period_set_wrapper
 *
 * Description:
 *   This is a wrapper for coex_schm_flexible_period_set. It sets the
 *   flexible period for the coexistence mechanism. If power management
 *   feature is enabled (CONFIG_ESP_COEX_POWER_MANAGEMENT), it calls the
 *   function with the given period. If the feature is not enabled, it
 *   returns 0.
 *
 * Input Parameters:
 *   period - The period to set for the coexistence mechanism.
 *
 * Returned Value:
 *   If power management is enabled, it returns the result of the
 *   coex_schm_flexible_period_set function. Otherwise, it returns 0.
 *
 ****************************************************************************/

static int coex_schm_flexible_period_set_wrapper(uint8_t period)
{
#if CONFIG_ESP_COEX_POWER_MANAGEMENT
  return coex_schm_flexible_period_set(period);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_flexible_period_get_wrapper
 *
 * Description:
 *   This is a wrapper for coex_schm_flexible_period_get. If power management
 *   feature is enabled (CONFIG_ESP_COEX_POWER_MANAGEMENT), it calls the
 *   function and returns its result. If the feature is not enabled, it
 *   returns 1.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   If power management is enabled, it returns the result of the
 *   coex_schm_flexible_period_get function. Otherwise, it returns 1.
 *
 ****************************************************************************/

static uint8_t coex_schm_flexible_period_get_wrapper(void)
{
#if CONFIG_ESP_COEX_POWER_MANAGEMENT
  return coex_schm_flexible_period_get();
#else
  return 1;
#endif
}

/****************************************************************************
 * Name: coex_schm_get_phase_by_idx_wrapper
 *
 * Description:
 *   This is a wrapper for coex_schm_get_phase_by_idx. If software
 *   coexistence is enabled (CONFIG_SW_COEXIST_ENABLE), it calls the function
 *   to get the coexistence phase by index. If software coexistence is not
 *   enabled, it returns NULL.
 *
 * Input Parameters:
 *   phase_idx - Index of the coexistence phase to retrieve
 *
 * Returned Value:
 *   If software coexistence is enabled, returns pointer to the coexistence
 *   phase. Otherwise returns NULL.
 *
 ****************************************************************************/

static void * coex_schm_get_phase_by_idx_wrapper(int phase_idx)
{
#if CONFIG_SW_COEXIST_ENABLE
  return coex_schm_get_phase_by_idx(phase_idx);
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: esp_random_ulong
 *
 * Description:
 *   Get random value and convert to unsigned long.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Random value
 *
 ****************************************************************************/

static unsigned long esp_random_ulong(void)
{
  return esp_random();
}
