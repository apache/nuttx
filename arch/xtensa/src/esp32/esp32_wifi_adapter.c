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

#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <clock/clock.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <irq/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/kthread.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/arch.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/tls.h>

#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_emac.h"
#include "hardware/esp32_soc.h"
#include "esp32_irq.h"
#include "esp32_wireless.h"
#include "esp32_wifi_adapter.h"
#include "esp32_rt_timer.h"
#include "esp32_wifi_utils.h"

#ifdef CONFIG_PM
#  include "esp32_pm.h"
#endif

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
#  include "esp_coexist_internal.h"
#endif

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

#define TIMER_INITIALIZED_VAL (0x5aa5a55a)

#define ESP_WIFI_11B_MAX_BITRATE       11
#define ESP_WIFI_11G_MAX_BITRATE       54
#define ESP_WIFI_11N_MCS7_HT20_BITRATE 72
#define ESP_WIFI_11N_MCS7_HT40_BITRATE 150

#define SSID_MAX_LEN                   (32)
#define PWD_MAX_LEN                    (64)

#ifndef CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL
#define CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL 3
#endif

#define DEFAULT_LISTEN_INTERVAL CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL

/* CONFIG_POWER_SAVE_MODEM */

#if defined(CONFIG_EXAMPLE_POWER_SAVE_MIN_MODEM)
#  define DEFAULT_PS_MODE WIFI_PS_MIN_MODEM
#elif defined(CONFIG_EXAMPLE_POWER_SAVE_MAX_MODEM)
#  define DEFAULT_PS_MODE WIFI_PS_MAX_MODEM
#elif defined(CONFIG_EXAMPLE_POWER_SAVE_NONE)
#  define DEFAULT_PS_MODE WIFI_PS_NONE
#else
#  define DEFAULT_PS_MODE WIFI_PS_NONE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Wi-Fi Station state */

enum wifi_sta_state
{
  WIFI_STA_STATE_NULL,
  WIFI_STA_STATE_START,
  WIFI_STA_STATE_CONNECT,
  WIFI_STA_STATE_DISCONNECT,
  WIFI_STA_STATE_STOP
};

/* Wi-Fi SoftAP state */

enum wifi_softap_state
{
  WIFI_SOFTAP_STATE_NULL,
  WIFI_SOFTAP_STATE_START,
  WIFI_SOFTAP_STATE_STOP
};

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

/* Wi-Fi timer private data */

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

/* Wi-Fi event private data */

struct evt_adpt
{
  sq_entry_t entry;         /* Sequence entry */
  int32_t id;               /* Event ID */
  uint8_t buf[0];           /* Event private data */
};

/* Wi-Fi event notification private data */

struct wifi_notify
{
  bool assigned;            /* Flag indicate if it is used */
  pid_t pid;                /* Signal's target thread PID */
  struct sigevent event;    /* Signal event private data */
  struct sigwork_s work;    /* Signal work private data */
};

/* Wi-Fi NVS private data */

struct nvs_adpt
{
  char *index_name;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool wifi_env_is_chip(void);
static void wifi_set_intr(int32_t cpu_no, uint32_t intr_source,
                          uint32_t intr_num, int32_t intr_prio);
static void wifi_clear_intr(uint32_t intr_source, uint32_t intr_num);
static void esp_set_isr(int32_t n, void *f, void *arg);
static void esp32_ints_on(uint32_t mask);
static void esp32_ints_off(uint32_t mask);
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

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
static int32_t esp_semphr_take_from_isr(void *semphr, void *hptw);
static int32_t esp_semphr_give_from_isr(void *semphr, void *hptw);
static int wifi_is_in_isr(void);
#endif

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
static void esp_free(void *ptr);
static uint32_t esp_rand(void);
static void esp_dport_access_stall_other_cpu_start(void);
static void esp_dport_access_stall_other_cpu_end(void);
static void wifi_apb80m_request(void);
static void wifi_apb80m_release(void);
static int32_t wifi_phy_update_country_info(const char *country);
static int32_t esp_wifi_read_mac(uint8_t *mac, uint32_t type);
static void esp_timer_arm(void *timer, uint32_t tmout, bool repeat);
static void esp_timer_disarm(void *timer);
static void esp32_timer_done(void *timer);
static void esp_timer_setfn(void *timer, void *pfunction, void *parg);
static void esp_timer_arm_us(void *timer, uint32_t us, bool repeat);
static void wifi_reset_mac(void);
static void wifi_clock_enable(void);
static void wifi_clock_disable(void);
static void wifi_rtc_enable_iso(void);
static void wifi_rtc_disable_iso(void);
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
                           const char *format, va_list args)
            printf_like(3, 0);
static void *esp_malloc_internal(size_t size);
static void *esp_realloc_internal(void *ptr, size_t size);
static void *esp_calloc_internal(size_t n, size_t size);
static void *esp_zalloc_internal(size_t size);
static void *esp_wifi_malloc(size_t size);
static void *esp_wifi_realloc(void *ptr, size_t size);
static void *esp_wifi_calloc(size_t n, size_t size);
static void *esp_wifi_zalloc(size_t size);
static void *esp_wifi_create_queue(int32_t queue_len, int32_t item_size);
static void esp_wifi_delete_queue(void *queue);
static int wifi_coex_init(void);
static void wifi_coex_deinit(void);
static int wifi_coex_enable(void);
static void wifi_coex_disable(void);
static uint32_t esp_coex_status_get(void);
static void esp_coex_condition_set(uint32_t type, bool dissatisfy);
static int32_t esp_coex_wifi_request(uint32_t event, uint32_t latency,
                                     uint32_t duration);
static int32_t esp_coex_wifi_release(uint32_t event);
static unsigned long esp_random_ulong(void);
static int wifi_coex_wifi_set_channel(uint8_t primary, uint8_t secondary);
static int wifi_coex_get_event_duration(uint32_t event,
                                        uint32_t *duration);
static int wifi_coex_get_pti(uint32_t event, uint8_t *pti);
static void wifi_coex_clear_schm_status_bit(uint32_t type,
                                            uint32_t status);
static void wifi_coex_set_schm_status_bit(uint32_t type,
                                          uint32_t status);
static int wifi_coex_set_schm_interval(uint32_t interval);
static uint32_t wifi_coex_get_schm_interval(void);
static uint8_t wifi_coex_get_schm_curr_period(void);
static void *wifi_coex_get_schm_curr_phase(void);
static int wifi_coex_set_schm_curr_phase_idx(int idx);
static int wifi_coex_get_schm_curr_phase_idx(void);

extern void coex_bt_high_prio(void);

/****************************************************************************
 * Public Functions declaration
 ****************************************************************************/

int64_t esp_timer_get_time(void);
void esp_fill_random(void *buf, size_t len);
void esp_log_write(uint32_t level, const char *tag, const char *format, ...)
     printf_like(3, 4);
uint32_t esp_log_timestamp(void);
uint8_t esp_crc8(const uint8_t *p, uint32_t len);
void intr_matrix_set(int cpu_no, uint32_t model_num, uint32_t intr_num);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Wi-Fi event private data */

static struct work_s g_wifi_evt_work;
static sq_queue_t g_wifi_evt_queue;
static struct wifi_notify g_wifi_notify[WIFI_ADPT_EVT_MAX];
static mutex_t g_wifiexcl_lock = NXMUTEX_INITIALIZER;

/* Callback function to update Wi-Fi MAC time */

wifi_mac_time_update_cb_t g_wifi_mac_time_update_cb;

/* Wi-Fi adapter reference */

static int g_wifi_ref;

#ifdef ESP32_WLAN_HAS_STA

/* If reconnect automatically */

static bool g_sta_reconnect;

/* If Wi-Fi sta starts */

static bool g_sta_started;

/* If Wi-Fi sta connected */

static bool g_sta_connected;

/* Wi-Fi station TX done callback function */

static wifi_txdone_cb_t g_sta_txdone_cb;
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP

/* If Wi-Fi SoftAP starts */

static bool g_softap_started;

/* Wi-Fi SoftAP TX done callback function */

static wifi_txdone_cb_t g_softap_txdone_cb;
#endif

/* Device specific lock */

static spinlock_t g_lock;

/* Wi-Fi and BT coexistence OS adapter data */

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
coex_adapter_funcs_t g_coex_adapter_funcs =
{
  ._version = COEX_ADAPTER_VERSION,
  ._spin_lock_create = esp_spin_lock_create,
  ._spin_lock_delete = esp_spin_lock_delete,
  ._int_enable = esp_wifi_int_restore,
  ._int_disable = esp_wifi_int_disable,
  ._task_yield_from_isr = esp_task_yield_from_isr,
  ._semphr_create = esp_semphr_create,
  ._semphr_delete = esp_semphr_delete,
  ._semphr_take_from_isr = esp_semphr_take_from_isr,
  ._semphr_give_from_isr = esp_semphr_give_from_isr,
  ._semphr_take = esp_semphr_take,
  ._semphr_give = esp_semphr_give,
  ._is_in_isr = wifi_is_in_isr,
  ._malloc_internal =  esp_malloc_internal,
  ._free = esp_free,
  ._timer_disarm = esp_timer_disarm,
  ._timer_done = esp32_timer_done,
  ._timer_setfn = esp_timer_setfn,
  ._timer_arm_us = esp_timer_arm_us,
  ._esp_timer_get_time = esp_timer_get_time,
  ._magic = COEX_ADAPTER_MAGIC,
};
#endif

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
  ._ints_on = esp32_ints_on,
  ._ints_off = esp32_ints_off,
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
  ._event_post = esp_event_post,
  ._get_free_heap_size = esp_get_free_heap_size,
  ._rand = esp_rand,
  ._dport_access_stall_other_cpu_start_wrap =
      esp_dport_access_stall_other_cpu_start,
  ._dport_access_stall_other_cpu_end_wrap =
      esp_dport_access_stall_other_cpu_end,
  ._wifi_apb80m_request = wifi_apb80m_request,
  ._wifi_apb80m_release = wifi_apb80m_release,
  ._phy_disable = esp32_phy_disable,
  ._phy_enable = esp32_phy_enable,
  ._phy_common_clock_enable = esp32_phy_enable_clock,
  ._phy_common_clock_disable = esp32_phy_disable_clock,
  ._phy_update_country_info = wifi_phy_update_country_info,
  ._read_mac = esp_wifi_read_mac,
  ._timer_arm = esp_timer_arm,
  ._timer_disarm = esp_timer_disarm,
  ._timer_done = esp32_timer_done,
  ._timer_setfn = esp_timer_setfn,
  ._timer_arm_us = esp_timer_arm_us,
  ._wifi_reset_mac = wifi_reset_mac,
  ._wifi_clock_enable = wifi_clock_enable,
  ._wifi_clock_disable = wifi_clock_disable,
  ._wifi_rtc_enable_iso = wifi_rtc_enable_iso,
  ._wifi_rtc_disable_iso = wifi_rtc_disable_iso,
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
  ._random = esp_random_ulong,
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
  ._coex_init = wifi_coex_init,
  ._coex_deinit = wifi_coex_deinit,
  ._coex_enable = wifi_coex_enable,
  ._coex_disable = wifi_coex_disable,
  ._coex_status_get = esp_coex_status_get,
  ._coex_condition_set = esp_coex_condition_set,
  ._coex_wifi_request = esp_coex_wifi_request,
  ._coex_wifi_release = esp_coex_wifi_release,
  ._coex_wifi_channel_set = wifi_coex_wifi_set_channel,
  ._coex_event_duration_get = wifi_coex_get_event_duration,
  ._coex_pti_get = wifi_coex_get_pti,
  ._coex_schm_status_bit_clear = wifi_coex_clear_schm_status_bit,
  ._coex_schm_status_bit_set = wifi_coex_set_schm_status_bit,
  ._coex_schm_interval_set = wifi_coex_set_schm_interval,
  ._coex_schm_interval_get = wifi_coex_get_schm_interval,
  ._coex_schm_curr_period_get = wifi_coex_get_schm_curr_period,
  ._coex_schm_curr_phase_get = wifi_coex_get_schm_curr_phase,
  ._coex_schm_curr_phase_idx_set = wifi_coex_set_schm_curr_phase_idx,
  ._coex_schm_curr_phase_idx_get = wifi_coex_get_schm_curr_phase_idx,
  ._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};

/* Wi-Fi feature capacity data */

uint64_t g_wifi_feature_caps = CONFIG_FEATURE_WPA3_SAE_BIT;

/* Wi-Fi TAG string data */

ESP_EVENT_DEFINE_BASE(WIFI_EVENT);

/****************************************************************************
 * Private Functions and Public Functions only used by libraries
 ****************************************************************************/

/****************************************************************************
 * Name: osi_errno_trans
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

static inline int32_t osi_errno_trans(int ret)
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
 * Name: osi_errno_trans
 *
 * Description:
 *   Transform from ESP Wi-Fi error code to NuttX error code
 *
 * Input Parameters:
 *   ret - ESP Wi-Fi error code
 *
 * Returned Value:
 *   NuttX error code
 *
 ****************************************************************************/

static int32_t wifi_errno_trans(int ret)
{
  int wifierr;

  /* Unmask component error bits */

  wifierr = ret & 0xfff;

  if (wifierr == ESP_OK)
    {
      return OK;
    }
  else if (wifierr == ESP_ERR_NO_MEM)
    {
      return -ENOMEM;
    }
  else if (wifierr == ESP_ERR_INVALID_ARG)
    {
      return -EINVAL;
    }
  else if (wifierr == ESP_ERR_INVALID_STATE)
    {
      return -EIO;
    }
  else if (wifierr == ESP_ERR_INVALID_SIZE)
    {
      return -EINVAL;
    }
  else if (wifierr == ESP_ERR_NOT_FOUND)
    {
      return -ENOSYS;
    }
  else if (wifierr == ESP_ERR_NOT_SUPPORTED)
    {
      return -ENOSYS;
    }
  else if (wifierr == ESP_ERR_TIMEOUT)
    {
      return -ETIMEDOUT;
    }
  else if (wifierr == ESP_ERR_INVALID_MAC)
    {
      return -EINVAL;
    }
  else
    {
      return ERROR;
    }
}

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
  uint32_t tmp;

  tmp = TICK2SEC(ticks);
  timespec->tv_sec += tmp;

  ticks -= SEC2TICK(tmp);
  tmp = TICK2NSEC(ticks);

  timespec->tv_nsec += tmp;
}

/****************************************************************************
 * Name: esp_wifi_lock
 *
 * Description:
 *   Lock or unlock the event process
 *
 * Input Parameters:
 *   lock - true: Lock event process, false: unlock event process
 *
 * Returned Value:
 *   The result of lock or unlock the event process
 *
 ****************************************************************************/

static int esp_wifi_lock(bool lock)
{
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&g_wifiexcl_lock);
      if (ret < 0)
        {
          wlinfo("Failed to lock Wi-Fi ret=%d\n", ret);
        }
    }
  else
    {
      ret = nxmutex_unlock(&g_wifiexcl_lock);
      if (ret < 0)
        {
          wlinfo("Failed to unlock Wi-Fi ret=%d\n", ret);
        }
    }

  return ret;
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

  wlinfo("n=%d f=%p arg=%p irq=%d\n", n, f, arg, irq);

  if (g_irqvector[irq].handler &&
      g_irqvector[irq].handler != irq_unexpected_isr)
    {
      wlinfo("irq=%d has been set handler=%p\n", irq,
             g_irqvector[irq].handler);
      return ;
    }

  tmp = sizeof(struct irq_adpt);
  adapter = kmm_malloc(tmp);
  if (!adapter)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      assert(0);
      return ;
    }

  adapter->func = f;
  adapter->arg = arg;

  ret = irq_attach(irq, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("Failed to attach IRQ %d\n", irq);
      assert(0);
      return ;
    }
}

/****************************************************************************
 * Name: esp32_ints_on
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

static void esp32_ints_on(uint32_t mask)
{
  int irq = __builtin_ffs(mask) - 1;

  wlinfo("INFO mask=%08x irq=%d\n", mask, irq);

  up_enable_irq(ESP32_IRQ_MAC);
}

/****************************************************************************
 * Name: esp32_ints_off
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

static void esp32_ints_off(uint32_t mask)
{
  uint32_t irq = __builtin_ffs(mask) - 1;

  wlinfo("INFO mask=%08x irq=%d\n", mask, irq);

  up_disable_irq(ESP32_IRQ_MAC);
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
#ifdef CONFIG_SMP
  spinlock_t *lock;
  int tmp;

  tmp = sizeof(*lock);
  lock = kmm_malloc(tmp);
  if (!lock)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      DEBUGPANIC();
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
 *   semphr - Semaphore data pointer
 *   ticks  - Wait system ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_semphr_take(void *semphr, uint32_t ticks)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  if (ticks == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = nxsem_wait(sem);
      if (ret)
        {
          wlerr("Failed to wait sem\n");
        }
    }
  else
    {
      ret = nxsem_tickwait(sem, ticks);
      if (ret)
        {
          wlerr("Failed to wait sem in %d ticks\n", ticks);
        }
    }

  return osi_errno_trans(ret);
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

  return osi_errno_trans(ret);
}
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST

/****************************************************************************
 * Name: esp_semphr_take_from_isr
 *
 * Description:
 *   Try to take semaphore from within an interrupt service routine.
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_semphr_take_from_isr(void *semphr, void *hptw)
{
  *(int *)hptw = 0;

  return esp_semphr_take(semphr, 0);
}

/****************************************************************************
 * Name: esp_semphr_give_from_isr
 *
 * Description:
 *   Post semaphore from within an interrupt service routine.
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t esp_semphr_give_from_isr(void *semphr, void *hptw)
{
  *(int *)hptw = 0;

  return esp_semphr_give(semphr);
}

/****************************************************************************
 * Name: wifi_is_in_isr
 *
 * Description:
 *   Check whether current execution context is of an interrupt service
 *   routine.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if in interrupt or false if not
 *
 ****************************************************************************/

static int IRAM_ATTR wifi_is_in_isr(void)
{
  return up_interrupt_context();
}
#endif

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

  return osi_errno_trans(ret);
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

  return osi_errno_trans(ret);
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

  return osi_errno_trans(ret);
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
                                          int32_t clear_on_exit,
                                          int32_t wait_for_all_bits,
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
#ifdef CONFIG_SMP
  int ret;
  cpu_set_t cpuset;
#endif

  pid = kthread_create(name, prio, stack_depth, entry,
                      (char * const *)param);
  if (pid > 0)
    {
      if (task_handle != NULL)
        {
          *((int *)task_handle) = pid;
        }
#ifdef CONFIG_SMP
      if (core_id < CONFIG_SMP_NCPUS)
        {
          CPU_ZERO(&cpuset);
          CPU_SET(core_id, &cpuset);
          ret = nxsched_set_affinity(pid, sizeof(cpuset), &cpuset);
          if (ret)
            {
              wlerr("Failed to set affinity error=%d\n", ret);
              return false;
            }
        }
#endif
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

  nxsig_usleep(us);
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
  pid_t pid = nxsched_getpid();

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

static void *esp_malloc(uint32_t size)
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
 * Name: esp_event_id_map
 *
 * Description:
 *   Transform from esp-idf event ID to Wi-Fi adapter event ID
 *
 * Input Parameters:
 *   event_id - esp-idf event ID
 *
 * Returned Value:
 *   Wi-Fi adapter event ID
 *
 ****************************************************************************/

static int esp_event_id_map(int event_id)
{
  int id;

  switch (event_id)
    {
      case WIFI_EVENT_SCAN_DONE:
        id = WIFI_ADPT_EVT_SCAN_DONE;
        break;

#ifdef ESP32_WLAN_HAS_STA
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
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP
      case WIFI_EVENT_AP_START:
        id = WIFI_ADPT_EVT_AP_START;
        break;

      case WIFI_EVENT_AP_STOP:
        id = WIFI_ADPT_EVT_AP_STOP;
        break;

      case WIFI_EVENT_AP_STACONNECTED:
        id = WIFI_ADPT_EVT_AP_STACONNECTED;
        break;

      case WIFI_EVENT_AP_STADISCONNECTED:
        id = WIFI_ADPT_EVT_AP_STADISCONNECTED;
        break;
#endif
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

static void esp_evt_work_cb(void *arg)
{
  int ret;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;
  struct wifi_notify *notify;

  while (1)
    {
      flags = spin_lock_irqsave(&g_lock);
      evt_adpt = (struct evt_adpt *)sq_remfirst(&g_wifi_evt_queue);
      spin_unlock_irqrestore(&g_lock, flags);
      if (!evt_adpt)
        {
          break;
        }

      esp_wifi_lock(true);

      switch (evt_adpt->id)
        {
          case WIFI_ADPT_EVT_SCAN_DONE:
            esp_wifi_scan_event_parse();
            break;

#ifdef ESP32_WLAN_HAS_STA
          case WIFI_ADPT_EVT_STA_START:
            wlinfo("Wi-Fi sta start\n");
            g_sta_connected = false;
            ret = esp_wifi_set_ps(DEFAULT_PS_MODE);
            if (ret)
              {
                wlerr("Failed to close PS\n");
              }
            break;

          case WIFI_ADPT_EVT_STA_CONNECT:
            wlinfo("Wi-Fi sta connect\n");
            g_sta_connected = true;
            break;

          case WIFI_ADPT_EVT_STA_DISCONNECT:
            wlinfo("Wi-Fi sta disconnect\n");
            g_sta_connected = false;
            if (g_sta_reconnect)
              {
                ret = esp_wifi_connect();
                if (ret)
                  {
                    wlerr("Failed to connect AP error=%d\n", ret);
                  }
              }
            break;

          case WIFI_ADPT_EVT_STA_STOP:
            wlinfo("Wi-Fi sta stop\n");
            g_sta_connected = false;
            break;
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP
          case WIFI_ADPT_EVT_AP_START:
            wlinfo("INFO: Wi-Fi softap start\n");
            break;

          case WIFI_ADPT_EVT_AP_STOP:
            wlinfo("INFO: Wi-Fi softap stop\n");
            break;

          case WIFI_ADPT_EVT_AP_STACONNECTED:
            wlinfo("INFO: Wi-Fi station join\n");
            break;

          case WIFI_ADPT_EVT_AP_STADISCONNECTED:
            wlinfo("INFO: Wi-Fi station leave\n");
            break;
#endif
          default:
            break;
        }

      notify = &g_wifi_notify[evt_adpt->id];
      if (notify->assigned)
        {
          notify->event.sigev_value.sival_ptr = evt_adpt->buf;

          ret = nxsig_notification(notify->pid, &notify->event,
                                   SI_QUEUE, &notify->work);
          if (ret < 0)
            {
              wlwarn("nxsig_notification event ID=%d failed: %d\n",
                     evt_adpt->id, ret);
            }
        }

      esp_wifi_lock(false);

      kmm_free(evt_adpt);
    }
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

  /* Force to bind Wi-Fi interrupt to CPU0 */

  intr_matrix_set(0, intr_source, intr_num);
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
      wlinfo("No process event %d\n", event_id);
      return -1;
    }

  size = event_data_size + sizeof(struct evt_adpt);
  evt_adpt = kmm_malloc(size);
  if (!evt_adpt)
    {
      wlerr("Failed to alloc %d memory\n", size);
      return -1;
    }

  evt_adpt->id = id;
  memcpy(evt_adpt->buf, event_data, event_data_size);

  flags = spin_lock_irqsave(&g_lock);
  sq_addlast(&evt_adpt->entry, &g_wifi_evt_queue);
  spin_unlock_irqrestore(&g_lock, flags);

  work_queue(LPWORK, &g_wifi_evt_work, esp_evt_work_cb, NULL, 0);

  return 0;
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

uint32_t esp_get_free_heap_size(void)
{
  struct mallinfo info;

  info = kmm_mallinfo();
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
  DEBUGPANIC();
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
  DEBUGPANIC();
#endif
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
#ifdef CONFIG_ESP32_AUTO_SLEEP
  esp32_pm_lockacquire();
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
#ifdef CONFIG_ESP32_AUTO_SLEEP
  esp32_pm_lockrelease();
#endif
}

/****************************************************************************
 * Name: wifi_phy_update_country_info
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int32_t wifi_phy_update_country_info(const char *country)
{
  return -1;
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
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;
  esp_timer_handle_t esp_timer = (esp_timer_handle_t)ets_timer->priv;

  if (ets_timer->expire == TIMER_INITIALIZED_VAL)
    {
      esp_timer_stop(esp_timer);
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
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;
  esp_timer_handle_t esp_timer = (esp_timer_handle_t)ets_timer->priv;

  if (ets_timer->expire == TIMER_INITIALIZED_VAL)
    {
      ets_timer->expire = 0;
      esp_timer_delete(esp_timer);
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
  int ret;
  esp_timer_handle_t esp_timer;
  struct ets_timer *ets_timer = (struct ets_timer *)ptimer;

  if (ets_timer->expire != TIMER_INITIALIZED_VAL)
    {
      ets_timer->priv = NULL;
    }

  if (ets_timer->priv == NULL)
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
          ets_timer->priv = esp_timer;
          ets_timer->expire = TIMER_INITIALIZED_VAL;
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
  esp_timer_handle_t esp_timer = (esp_timer_handle_t)ets_timer->priv;

  if (ets_timer->expire == TIMER_INITIALIZED_VAL)
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
  modifyreg32(DPORT_WIFI_RST_EN_REG, 0, DPORT_MAC_RST_EN);
  modifyreg32(DPORT_WIFI_RST_EN_REG, DPORT_MAC_RST_EN, 0);
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
  modifyreg32(DPORT_WIFI_CLK_EN_REG, 0, DPORT_WIFI_CLK_WIFI_EN_M);
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
  modifyreg32(DPORT_WIFI_CLK_EN_REG, DPORT_WIFI_CLK_WIFI_EN_M, 0);
}

/****************************************************************************
 * Name: wifi_rtc_enable_iso
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void wifi_rtc_enable_iso(void)
{
}

/****************************************************************************
 * Name: wifi_rtc_disable_iso
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void wifi_rtc_disable_iso(void)
{
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

static int32_t esp_nvs_set_i8(uint32_t handle,
                              const char *key,
                              int8_t value)
{
#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  return esp_nvs_set_blob(handle, key, &value, sizeof(int8_t));
#else
  DEBUGPANIC();

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
  DEBUGPANIC();

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
  DEBUGPANIC();

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
  DEBUGPANIC();

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
  DEBUGPANIC();

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
  DEBUGPANIC();

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
      wlerr("Failed to alloc %d memory\n", tmp);
      return -1;
    }

  ret = asprintf(&index_name, "%s", name);
  if (ret < 0)
    {
      wlerr("Failed to create NVS index_name string\n");
      kmm_free(nvs_adpt);
      return -1;
    }

  nvs_adpt->index_name = index_name;
  *out_handle = (uint32_t)nvs_adpt;

  return 0;
#else
  DEBUGPANIC();

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
  DEBUGPANIC();
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
  struct file file;
  int ret;
  char *dir;
  struct nvs_adpt *nvs_adpt = (struct nvs_adpt *)handle;
  char *index_name = nvs_adpt->index_name;

  ret = asprintf(&dir, NVS_DIR_BASE"%s.%s", index_name, key);
  if (ret < 0)
    {
      wlerr("Failed to create NVS dir string\n");
      return -1;
    }

  ret = nx_unlink(dir);
  if (ret)
    {
      if (ret != -ENOENT)
        {
          wlerr("Failed to unlink %s error=%d\n", dir, ret);
          kmm_free(dir);
          return -1;
        }
    }

  ret = file_open(&file, dir, O_WRONLY | O_CREAT, NVS_FILE_MODE);
  if (ret < 0)
    {
      wlerr("Failed to set open %s\n", dir);
      kmm_free(dir);
      return -1;
    }

  ret = file_write(&file, value, length);
  if (ret < 0)
    {
      wlerr("Failed to write to %s\n", dir);
      kmm_free(dir);
      file_close(&file);
      return -1;
    }

  kmm_free(dir);
  file_close(&file);

  return 0;
#else
  DEBUGPANIC();

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
  struct file file;
  int ret;
  char *dir;
  struct nvs_adpt *nvs_adpt = (struct nvs_adpt *)handle;
  char *index_name = nvs_adpt->index_name;

  ret = asprintf(&dir, NVS_DIR_BASE"%s.%s", index_name, key);
  if (ret < 0)
    {
      wlerr("Failed to create NVS dir string\n");
      return -1;
    }

  ret = file_open(&file, dir, O_RDONLY);
  if (ret < 0)
    {
      if (ret == -ENOENT)
        {
          wlinfo("No file %s\n", dir);
          kmm_free(dir);
          return ESP_ERR_NVS_NOT_FOUND;
        }
      wlerr("Failed to get open %s\n", dir);
      kmm_free(dir);
      return -1;
    }

  ret = file_read(&file, out_value, *length);
  if (ret <= 0)
    {
      wlerr("Failed to write to %s\n", dir);
      kmm_free(dir);
      file_close(&file);
      return -1;
    }
  else
    {
      *length = ret;
    }

  kmm_free(dir);
  file_close(&file);

  return 0;
#else
  DEBUGPANIC();

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
      wlerr("Failed to create NVS dir string\n");
      return -1;
    }

  ret = nx_unlink(dir);
  if (ret < 0)
    {
      wlerr("Failed to delete NVS file %s\n", dir);
      kmm_free(dir);
      return -1;
    }

  kmm_free(dir);

  return 0;
#else
  DEBUGPANIC();

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
      tmp = esp_random();
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
  switch (level)
    {
#ifdef CONFIG_DEBUG_WIRELESS_ERROR
      case ESP_LOG_ERROR:
        vsyslog(LOG_ERR, format, args);
        break;
#endif
#ifdef CONFIG_DEBUG_WIRELESS_WARN
      case ESP_LOG_WARN:
        vsyslog(LOG_WARNING, format, args);
        break;
#endif
#ifdef CONFIG_DEBUG_WIRELESS_INFO
      case ESP_LOG_INFO:
        vsyslog(LOG_INFO, format, args);
        break;
      default:
        vsyslog(LOG_DEBUG, format, args);
        break;
#endif
    }
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
#ifdef CONFIG_MM_KERNEL_HEAP
  return kmm_malloc(size);
#elif defined(CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
  return xtensa_imm_malloc(size);
#else
  void *ptr = kmm_malloc(size);
  if (esp32_ptr_extram(ptr))
    {
      kmm_free(ptr);
      return NULL;
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
      if (esp32_ptr_extram(new_ptr))
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
  if (esp32_ptr_extram(ptr))
    {
      kmm_free(ptr);
      return NULL;
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
  if (esp32_ptr_extram(ptr))
    {
      kmm_free(ptr);
      return NULL;
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

static void *esp_wifi_create_queue(int32_t queue_len, int32_t item_size)
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
 * Name: wifi_coex_init
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_init(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_init();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_deinit
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void wifi_coex_deinit(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  coex_deinit();
#endif
}

/****************************************************************************
 * Name: wifi_coex_enable
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_enable(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_enable();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_disable
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void wifi_coex_disable(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  coex_disable();
#endif
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
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_status_get();
#else
  return 0;
#endif
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
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  coex_condition_set(type, dissatisfy);
#endif
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
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_wifi_request(event, latency, duration);
#else
  return 0;
#endif
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
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
    return coex_wifi_release(event);
#else
    return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_wifi_set_channel
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_wifi_set_channel(uint8_t primary, uint8_t secondary)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_wifi_channel_set(primary, secondary);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_get_event_duration
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_get_event_duration(uint32_t event, uint32_t *duration)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_event_duration_get(event, duration);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_get_pti
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_get_pti(uint32_t event, uint8_t *pti)
{
  return 0;
}

/****************************************************************************
 * Name: wifi_coex_clear_schm_status_bit
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void wifi_coex_clear_schm_status_bit(uint32_t type, uint32_t status)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  coex_schm_status_bit_clear(type, status);
#endif
}

/****************************************************************************
 * Name: wifi_coex_set_schm_status_bit
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void wifi_coex_set_schm_status_bit(uint32_t type, uint32_t status)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  coex_schm_status_bit_set(type, status);
#endif
}

/****************************************************************************
 * Name: wifi_coex_set_schm_interval
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_set_schm_interval(uint32_t interval)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_schm_interval_set(interval);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_get_schm_interval
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t wifi_coex_get_schm_interval(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_schm_interval_get();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_get_schm_curr_period
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint8_t wifi_coex_get_schm_curr_period(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_schm_curr_period_get();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_get_schm_curr_phase
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void *wifi_coex_get_schm_curr_phase(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_schm_curr_phase_get();
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: wifi_coex_set_schm_curr_phase_idx
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_set_schm_curr_phase_idx(int idx)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_schm_curr_phase_idx_set(idx);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: wifi_coex_get_schm_curr_phase_idx
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static int wifi_coex_get_schm_curr_phase_idx(void)
{
#if defined(CONFIG_ESP32_WIFI_BT_COEXIST)
  return coex_schm_curr_phase_idx_get();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: esp_random_ulong
 *
 * Description:
 *   A simpler wrapper of esp_random.
 *   Just convert the return value from uint32_t to unsigned long.
 *
 ****************************************************************************/

static unsigned long esp_random_ulong(void)
{
  return esp_random();
}

/****************************************************************************
 * Name: esp_wifi_tx_done_cb
 *
 * Description:
 *   Wi-Fi TX done callback function.
 *
 ****************************************************************************/

static IRAM_ATTR void esp_wifi_tx_done_cb(uint8_t ifidx, uint8_t *data,
                                          uint16_t *len, bool txstatus)
{
#ifdef ESP32_WLAN_HAS_STA
  if (ifidx == ESP_IF_WIFI_STA)
    {
      if (g_sta_txdone_cb)
        {
          g_sta_txdone_cb(data, len, txstatus);
        }
    }
  else
#endif
#ifdef ESP32_WLAN_HAS_SOFTAP
  if (ifidx == ESP_IF_WIFI_AP)
    {
      if (g_softap_txdone_cb)
        {
          g_softap_txdone_cb(data, len, txstatus);
        }
    }
  else
#endif
    {
      wlerr("ifidx=%d is error\n", ifidx);
    }
}

/****************************************************************************
 * Name: esp_wifi_set_auth_param
 *
 * Description:
 *   Converts a ESP32 authenticate mode values to WEXT authenticate mode.
 *
 * Input Parameters:
 *   wifi_auth - ESP32 authenticate mode
 *
 * Returned Value:
 *     authenticate mode
 *
 ****************************************************************************/

static int esp_wifi_auth_trans(uint32_t wifi_auth)
{
  int auth_mode = IW_AUTH_WPA_VERSION_DISABLED;

  switch (wifi_auth)
    {
      case WIFI_AUTH_OPEN:
        auth_mode = IW_AUTH_WPA_VERSION_DISABLED;
        break;

      case WIFI_AUTH_WPA_PSK:
        auth_mode = IW_AUTH_WPA_VERSION_WPA;
        break;

      case WIFI_AUTH_WPA2_PSK:
      case WIFI_AUTH_WPA_WPA2_PSK:
        auth_mode = IW_AUTH_WPA_VERSION_WPA2;
        break;

      default:
        wlerr("Failed to transfer wireless authmode: %d", wifi_auth);
        break;
    }

  return auth_mode;
}

/****************************************************************************
 * Name: esp_wifi_set_auth_param
 *
 * Description:
 *   Converts a ESP32 cipher type values to WEXT cipher type values.
 *
 * Input Parameters:
 *   wifi_cipher - ESP32 cipher type
 *
 * Returned Value:
 *     cipher type
 *
 ****************************************************************************/

static int esp_wifi_cipher_trans(uint32_t wifi_cipher)
{
  int cipher_mode = IW_AUTH_CIPHER_NONE;

  switch (wifi_cipher)
    {
      case WIFI_CIPHER_TYPE_NONE:
        cipher_mode = IW_AUTH_CIPHER_NONE;
        break;

      case WIFI_CIPHER_TYPE_WEP40:
        cipher_mode = IW_AUTH_CIPHER_WEP40;
        break;

      case WIFI_CIPHER_TYPE_WEP104:
        cipher_mode = IW_AUTH_CIPHER_WEP104;
        break;

      case WIFI_CIPHER_TYPE_TKIP:
        cipher_mode = IW_AUTH_CIPHER_TKIP;
        break;

      case WIFI_CIPHER_TYPE_CCMP:
      case WIFI_CIPHER_TYPE_TKIP_CCMP:
        cipher_mode = IW_AUTH_CIPHER_CCMP;
        break;

      case WIFI_CIPHER_TYPE_AES_CMAC128:
        cipher_mode = IW_AUTH_CIPHER_AES_CMAC;
        break;

      default:
        wlerr("Failed to transfer wireless authmode: %d",
               wifi_cipher);
        break;
    }

  return cipher_mode;
}

/****************************************************************************
 * Name: esp_freq_to_channel
 *
 * Description:
 *   Converts Wi-Fi frequency to channel.
 *
 * Input Parameters:
 *   freq - Wi-Fi frequency
 *
 * Returned Value:
 *   Wi-Fi channel
 *
 ****************************************************************************/

static int esp_freq_to_channel(uint16_t freq)
{
  int channel = 0;
  if (freq >= 2412 && freq <= 2484)
    {
      if (freq == 2484)
        {
          channel = 14;
        }
      else
        {
          channel = freq - 2407;
          if (channel % 5)
            {
              return 0;
            }

          channel /= 5;
        }

      return channel;
    }

  if (freq >= 5005 && freq < 5900)
    {
      if (freq % 5)
        {
          return 0;
        }

      channel = (freq - 5000) / 5;
      return channel;
    }

  if (freq >= 4905 && freq < 5000)
    {
      if (freq % 5)
        {
          return 0;
        }

      channel = (freq - 4000) / 5;
      return channel;
    }

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
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  va_list arg;

  va_start(arg, format);
  vsyslog(LOG_INFO, format, arg);
  va_end(arg);
#endif

  return 0;
}

/****************************************************************************
 * Functions needed by libmesh.a
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mesh_send_event_internal
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

int esp_mesh_send_event_internal(int32_t event_id,
                                 void *event_data,
                                 size_t event_data_size)
{
  return -1;
}

/****************************************************************************
 * Name: esp_mesh_get_topology
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

void *esp_mesh_get_topology(void)
{
  return NULL;
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
  int ret;
  struct rt_timer_args_s rt_timer_args;
  struct rt_timer_s *rt_timer;

  rt_timer_args.arg = create_args->arg;
  rt_timer_args.callback = create_args->callback;

  ret = rt_timer_create(&rt_timer_args, &rt_timer);
  if (ret)
    {
      wlerr("Failed to create rt_timer error=%d\n", ret);
      return ret;
    }

  *out_handle = (esp_timer_handle_t)rt_timer;

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
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  rt_timer_start(rt_timer, timeout_us, false);

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
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  rt_timer_start(rt_timer, period, true);

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
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  rt_timer_stop(rt_timer);

  return 0;
}

/****************************************************************************
 * Name: esp_timer_delete
 *
 * Description:
 *   Delete timer and free resource
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
  struct rt_timer_s *rt_timer = (struct rt_timer_s *)timer;

  rt_timer_delete(rt_timer);

  return 0;
}

/****************************************************************************
 * Name: __assert_func
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

void __assert_func(const char *file, int line,
                   const char *func, const char *expr)
{
  wlerr("Assert failed in %s, %s:%d (%s)",
        func, file, line, expr);
  abort();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wifi_bt_coexist_init
 *
 * Description:
 *   Initialize ESP32 Wi-Fi and BT coexistence module.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
int esp32_wifi_bt_coexist_init(void)
{
  coex_dbg_set_log_level(COEX_LOG_INFO);
  esp_coex_adapter_register(&g_coex_adapter_funcs);
  coex_pre_init();

  return 0;
}
#endif

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
 *   Task maximum priority
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
 *   Initialize Wi-Fi
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
      wlerr("Failed to initialize Wi-Fi error=%d\n", ret);
      return ret;
    }

  ret = esp_supplicant_init();
  if (ret)
    {
      wlerr("Failed to initialize WPA supplicant error=%d\n", ret);
      esp_wifi_deinit_internal();
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: esp_wifi_deinit
 *
 * Description:
 *   Deinitialize Wi-Fi and free resource
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
      wlerr("Failed to deinitialize supplicant\n");
      return ret;
    }

  ret = esp_wifi_deinit_internal();
  if (ret != 0)
    {
      wlerr("Failed to deinitialize Wi-Fi\n");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_free_eb
 *
 * Description:
 *   Free Wi-Fi receive callback input eb pointer
 *
 * Input Parameters:
 *   eb - Wi-Fi receive callback input eb pointer
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

int esp_wifi_notify_subscribe(pid_t pid, struct sigevent *event)
{
  int id;
  struct wifi_notify *notify;
  int ret = -1;

  wlinfo("PID=%d event=%p\n", pid, event);

  esp_wifi_lock(true);

  if (event->sigev_notify == SIGEV_SIGNAL)
    {
      id = esp_event_id_map(event->sigev_signo);
      if (id < 0)
        {
          wlerr("No process event %d\n", event->sigev_signo);
        }
      else
        {
          notify = &g_wifi_notify[id];

          if (notify->assigned)
            {
              wlerr("sigev_signo %d has subscribed\n",
                    event->sigev_signo);
            }
          else
            {
              if (pid == 0)
                {
                  pid = nxsched_getpid();
                  wlinfo("Actual PID=%d\n", pid);
                }

              notify->pid = pid;
              notify->event = *event;
              notify->assigned = true;

              ret = 0;
            }
        }
    }
  else if (event->sigev_notify == SIGEV_NONE)
    {
      id = esp_event_id_map(event->sigev_signo);
      if (id < 0)
        {
          wlerr("No process event %d\n", event->sigev_signo);
        }
      else
        {
          notify = &g_wifi_notify[id];

          if (!notify->assigned)
            {
              wlerr("sigev_signo %d has not subscribed\n",
                    event->sigev_signo);
            }
          else
            {
              notify->assigned = false;

              ret = 0;
            }
        }
    }
  else
    {
      wlerr("sigev_notify %d is invalid\n", event->sigev_signo);
    }

  esp_wifi_lock(false);

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_adapter_init
 *
 * Description:
 *   Initialize ESP32 Wi-Fi adapter
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
  wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();

  esp_wifi_lock(true);

  if (g_wifi_ref)
    {
      wlinfo("Wi-Fi adapter is already initialized\n");
      g_wifi_ref++;
      esp_wifi_lock(false);
      return OK;
    }

  sq_init(&g_wifi_evt_queue);

#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  wifi_cfg.nvs_enable = 1;
#else
  wifi_cfg.nvs_enable = 0;
#endif

#ifdef CONFIG_ESP32_WIFI_TX_AMPDU
  wifi_cfg.ampdu_tx_enable = 1;
#else
  wifi_cfg.ampdu_tx_enable = 0;
#endif

#ifdef CONFIG_ESP32_WIFI_RX_AMPDU
  wifi_cfg.ampdu_rx_enable = 1;
#else
  wifi_cfg.ampdu_rx_enable = 0;
#endif

#ifdef CONFIG_ESP32_WIFI_STA_DISCONNECT_PM
  wifi_cfg.sta_disconnected_pm = true;
#else
  wifi_cfg.sta_disconnected_pm = false;
#endif

  wifi_cfg.rx_ba_win          = CONFIG_ESP32_WIFI_RXBA_AMPDU_WZ;
  wifi_cfg.static_rx_buf_num  = CONFIG_ESP32_WIFI_STATIC_RXBUF_NUM;
  wifi_cfg.dynamic_rx_buf_num = CONFIG_ESP32_WIFI_DYNAMIC_RXBUF_NUM;
  wifi_cfg.dynamic_tx_buf_num = CONFIG_ESP32_WIFI_DYNAMIC_TXBUF_NUM;

  ret = esp_wifi_init(&wifi_cfg);
  if (ret)
    {
      wlerr("Failed to initialize Wi-Fi error=%d\n", ret);
      ret = wifi_errno_trans(ret);
      goto errout_init_wifi;
    }

  ret = esp_wifi_set_tx_done_cb(esp_wifi_tx_done_cb);
  if (ret)
    {
      wlerr("Failed to register TX done callback ret=%d\n", ret);
      ret = wifi_errno_trans(ret);
      goto errout_init_txdone;
    }

  g_wifi_ref++;

  wlinfo("OK to initialize Wi-Fi adapter\n");

  esp_wifi_lock(false);

  return OK;

errout_init_txdone:
  esp_wifi_deinit();
errout_init_wifi:
  esp_wifi_lock(false);

  return ret;
}

/****************************************************************************
 * Station functions
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_wifi_sta_start
 *
 * Description:
 *   Start Wi-Fi station.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_start(void)
{
  int ret;
  wifi_mode_t mode;

  esp_wifi_lock(true);

  ret = esp_wifi_stop();
  if (ret)
    {
      wlinfo("Failed to stop Wi-Fi ret=%d\n", ret);
    }

#ifdef ESP32_WLAN_HAS_SOFTAP
  if (g_softap_started)
    {
      mode = WIFI_MODE_APSTA;
    }
#else
  mode = WIFI_MODE_STA;
#endif

  ret = esp_wifi_set_mode(mode);
  if (ret)
    {
      wlerr("Failed to set Wi-Fi mode=%d ret=%d\n", mode, ret);
      ret = wifi_errno_trans(ret);
      goto errout_set_mode;
    }

  ret = esp_wifi_start();
  if (ret)
    {
      wlerr("Failed to start Wi-Fi with mode=%d ret=%d\n", mode, ret);
      ret = wifi_errno_trans(ret);
      goto errout_set_mode;
    }

  g_sta_started = true;

  wlinfo("OK to start Wi-Fi station\n");

  esp_wifi_lock(false);
  return OK;

errout_set_mode:
  esp_wifi_lock(false);
  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_stop
 *
 * Description:
 *   Stop Wi-Fi station.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_stop(void)
{
  int ret;

  esp_wifi_lock(true);

  ret = esp_wifi_stop();
  if (ret)
    {
      wlinfo("Failed to stop Wi-Fi ret=%d\n", ret);
    }

  g_sta_started = false;

#ifdef ESP32_WLAN_HAS_SOFTAP
  if (g_softap_started)
    {
      ret = esp_wifi_set_mode(WIFI_MODE_AP);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi AP mode ret=%d\n", ret);
          ret = wifi_errno_trans(ret);
          goto errout_set_mode;
        }

      ret = esp_wifi_start();
      if (ret)
        {
          wlerr("Failed to start Wi-Fi AP ret=%d\n", ret);
          ret = wifi_errno_trans(ret);
          goto errout_set_mode;
        }
    }
#endif

  wlinfo("OK to stop Wi-Fi station\n");

  esp_wifi_lock(false);
  return OK;

#ifdef ESP32_WLAN_HAS_SOFTAP
errout_set_mode:
  esp_wifi_lock(true);
  return ret;
#endif
}

/****************************************************************************
 * Name: esp_wifi_sta_send_data
 *
 * Description:
 *   Use Wi-Fi station interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_send_data(void *pbuf, uint32_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_STA, pbuf, len);

  return wifi_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_wifi_sta_register_recv_cb
 *
 * Description:
 *   Register Wi-Fi station receive packet callback function
 *
 * Input Parameters:
 *   recv_cb - Receive callback function
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_register_recv_cb(int (*recv_cb)(void *buffer,
                                                 uint16_t len,
                                                 void *eb))
{
  int ret;

  ret = esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t)recv_cb);

  return wifi_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_wifi_sta_register_txdone_cb
 *
 * Description:
 *   Register the station TX done callback function.
 *
 * Input Parameters:
 *   cb - The callback function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_sta_register_txdone_cb(wifi_txdone_cb_t cb)
{
  g_sta_txdone_cb = cb;
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
 * Name: esp_wifi_set_password
 *
 * Description:
 *   Set/Get Wi-Fi station password
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_password(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_encode_ext *ext = iwr->u.encoding.pointer;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[PWD_MAX_LEN + 1];
#endif

  DEBUGASSERT(ext != NULL);

  pdata = ext->key;
  len   = ext->key_len;

  if (set && len > PWD_MAX_LEN)
    {
      return -EINVAL;
    }

  ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
  if (ret)
    {
      wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
      return wifi_errno_trans(ret);
    }

  if (set)
    {
      memset(wifi_cfg.sta.password, 0x0, PWD_MAX_LEN);
      memcpy(wifi_cfg.sta.password, pdata, len);

      wifi_cfg.sta.pmf_cfg.capable = true;
      wifi_cfg.sta.listen_interval = DEFAULT_LISTEN_INTERVAL;

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.sta.password, PWD_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.sta.password, len);
        }

      if (g_sta_connected)
        {
          wifi_ap_record_t ap_info;

          ret = esp_wifi_sta_get_ap_info(&ap_info);
          if (ret)
            {
              wlerr("Failed to get AP record ret=%d", ret);
              return wifi_errno_trans(ret);
            }

          switch (ap_info.pairwise_cipher)
            {
              case WIFI_CIPHER_TYPE_NONE:
                ext->alg = IW_ENCODE_ALG_NONE;
                break;

              case WIFI_CIPHER_TYPE_WEP40:
              case WIFI_CIPHER_TYPE_WEP104:
                ext->alg = IW_ENCODE_ALG_WEP;
                break;

              case WIFI_CIPHER_TYPE_TKIP:
                ext->alg = IW_ENCODE_ALG_TKIP;
                break;

              case WIFI_CIPHER_TYPE_CCMP:
              case WIFI_CIPHER_TYPE_TKIP_CCMP:
                ext->alg = IW_ENCODE_ALG_CCMP;
                break;

              case WIFI_CIPHER_TYPE_AES_CMAC128:
                ext->alg = IW_ENCODE_ALG_AES_CMAC;
                break;

              default:
                wlerr("Failed to transfer wireless authmode: %d",
                      ap_info.pairwise_cipher);
                return -EIO;
            }
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi station password=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_essid
 *
 * Description:
 *   Set/Get Wi-Fi station ESSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_essid(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_point *essid = &iwr->u.essid;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[SSID_MAX_LEN + 1];
#endif

  DEBUGASSERT(essid != NULL);

  pdata = essid->pointer;
  len   = essid->length;

  if (set && len > SSID_MAX_LEN)
    {
      return -EINVAL;
    }

  ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
  if (ret)
    {
      wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
      return wifi_errno_trans(ret);
    }

  if (set)
    {
      memset(wifi_cfg.sta.ssid, 0x0, SSID_MAX_LEN);
      memcpy(wifi_cfg.sta.ssid, pdata, len);

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.sta.ssid, SSID_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.sta.ssid, len);
        }

      if (g_sta_connected)
        {
          essid->flags = IW_ESSID_ON;
        }
      else
        {
          essid->flags = IW_ESSID_OFF;
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi station ssid=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_bssid
 *
 * Description:
 *   Set/Get Wi-Fi station BSSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_bssid(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_config_t wifi_cfg;
  struct sockaddr *sockaddr;
  char *pdata;

  ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
  if (ret)
    {
      wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
      return wifi_errno_trans(ret);
    }

  sockaddr = &iwr->u.ap_addr;
  pdata    = sockaddr->sa_data;

  if (set)
    {
      wifi_cfg.sta.bssid_set = true;
      memcpy(wifi_cfg.sta.bssid, pdata, MAC_LEN);

      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      memcpy(pdata, wifi_cfg.sta.bssid, MAC_LEN);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_connect
 *
 * Description:
 *   Trigger Wi-Fi station connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_connect(void)
{
  int ret;
  uint32_t ticks;

  esp_wifi_lock(true);

  if (g_sta_connected)
    {
      wlinfo("Wi-Fi has connected AP\n");
      esp_wifi_lock(false);
      return OK;
    }

  g_sta_reconnect = true;

  ret = esp_wifi_connect();
  if (ret)
    {
      wlerr("Failed to connect ret=%d\n", ret);
      ret = wifi_errno_trans(ret);
      goto errout_wifi_connect;
    }

  esp_wifi_lock(false);

  ticks = SEC2TICK(WIFI_CONNECT_TIMEOUT);
  do
    {
      if (g_sta_connected)
        {
          break;
        }

      esp_task_delay(1);
    }
  while (ticks--);

  if (!g_sta_connected)
    {
      g_sta_reconnect = false;
      wlinfo("Failed to connect to AP\n");
      return -1;
    }

  return OK;

errout_wifi_connect:
  g_sta_reconnect = false;
  esp_wifi_lock(false);
  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_disconnect
 *
 * Description:
 *   Trigger Wi-Fi station disconnection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_disconnect(void)
{
  int ret;

  esp_wifi_lock(true);

  g_sta_reconnect = false;

  ret = esp_wifi_disconnect();
  if (ret)
    {
      wlerr("Failed to disconnect ret=%d\n", ret);
      ret = wifi_errno_trans(ret);
    }
  else
    {
      wlinfo("OK to disconnect Wi-Fi station\n");
    }

  esp_wifi_lock(false);
  return ret;
}

/****************************************************************************
 * Name: esp_wifi_sta_mode
 *
 * Description:
 *   Set/Get Wi-Fi Station mode code.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_mode(struct iwreq *iwr, bool set)
{
  if (set == false)
    {
      iwr->u.mode = IW_MODE_INFRA;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_auth
 *
 * Description:
 *   Set/Get station authentication mode params.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_auth(struct iwreq *iwr, bool set)
{
  int ret;
  int cmd;
  wifi_ap_record_t ap_info;

  if (set)
    {
      return OK;
    }
  else
    {
      if (g_sta_connected == false)
        {
          return -ENOTCONN;
        }

      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret)
        {
          wlerr("Failed to get AP record ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      cmd = iwr->u.param.flags & IW_AUTH_INDEX;
      switch (cmd)
        {
          case IW_AUTH_WPA_VERSION:
            iwr->u.param.value = esp_wifi_auth_trans(ap_info.authmode);
            break;

          case IW_AUTH_CIPHER_PAIRWISE:
            iwr->u.param.value =
                esp_wifi_cipher_trans(ap_info.pairwise_cipher);
            break;

          case IW_AUTH_CIPHER_GROUP:
            iwr->u.param.value = esp_wifi_cipher_trans(ap_info.group_cipher);
            break;

          case IW_AUTH_KEY_MGMT:
          case IW_AUTH_TKIP_COUNTERMEASURES:
          case IW_AUTH_DROP_UNENCRYPTED:
          case IW_AUTH_80211_AUTH_ALG:
          case IW_AUTH_WPA_ENABLED:
          case IW_AUTH_RX_UNENCRYPTED_EAPOL:
          case IW_AUTH_ROAMING_CONTROL:
          case IW_AUTH_PRIVACY_INVOKED:
          default:
            wlerr("Unknown cmd %d\n", cmd);
            return -ENOSYS;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_freq
 *
 * Description:
 *   Set/Get station frequency.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_freq(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_config_t wifi_cfg;

  if (set && (iwr->u.freq.flags == IW_FREQ_FIXED))
    {
      ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      wifi_cfg.sta.channel = esp_freq_to_channel(iwr->u.freq.m);
      ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      if (g_sta_connected)
        {
          wifi_ap_record_t ap_info;

          ret = esp_wifi_sta_get_ap_info(&ap_info);
          if (ret)
            {
              wlerr("Failed to get AP record ret=%d\n", ret);
              return wifi_errno_trans(ret);
            }

          iwr->u.freq.flags = IW_FREQ_FIXED;
          iwr->u.freq.e     = 0;
          iwr->u.freq.m     = 2407 + 5 * ap_info.primary;
        }
      else
        {
          iwr->u.freq.flags = IW_FREQ_AUTO;
          iwr->u.freq.e     = 0;
          iwr->u.freq.m     = 2412;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_bitrate
 *
 * Description:
 *   Get station default bit rate (Mbps).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_bitrate(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_ap_record_t ap_info;

  if (set)
    {
      return -ENOSYS;
    }
  else
    {
      if (g_sta_connected == false)
        {
          iwr->u.bitrate.fixed = IW_FREQ_AUTO;
          return OK;
        }

      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret)
        {
          wlerr("Failed to get AP record ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      iwr->u.bitrate.fixed = IW_FREQ_FIXED;
      if (ap_info.phy_11n)
        {
          if (ap_info.second)
            {
              iwr->u.bitrate.value = ESP_WIFI_11N_MCS7_HT40_BITRATE;
            }
          else
            {
              iwr->u.bitrate.value = ESP_WIFI_11N_MCS7_HT20_BITRATE;
            }
        }
      else if (ap_info.phy_11g)
        {
          iwr->u.bitrate.value = ESP_WIFI_11G_MAX_BITRATE;
        }
      else if (ap_info.phy_11b)
        {
          iwr->u.bitrate.value = ESP_WIFI_11B_MAX_BITRATE;
        }
      else
        {
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_get_txpower
 *
 * Description:
 *   Get station transmit power (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_txpower(struct iwreq *iwr, bool set)
{
  int ret;
  int8_t power;
  double power_dbm;

  if (set)
    {
      if (iwr->u.txpower.flags == IW_TXPOW_RELATIVE)
        {
          power = (int8_t)iwr->u.txpower.value;
        }
      else
        {
          if (iwr->u.txpower.flags == IW_TXPOW_MWATT)
            {
              power_dbm = ceil(10 * log10(iwr->u.txpower.value));
            }
          else
            {
              power_dbm = iwr->u.txpower.value;
            }

          power = (int8_t)(power_dbm * 4);
        }

      /* The value set by this API will be mapped to the max_tx_power
       * of the structure wifi_country_t variable. Param power unit is
       * 0.25dBm, range is [8, 84] corresponding to 2dBm - 20dBm.
       * Relationship between set value and actual value.
       * As follows: {set value range, actual value} =
       * {{[8,  19],8}, {[20, 27],20}, {[28, 33],28},
       * {[34, 43],34}, {[44, 51],44}, {[52, 55],52},
       * {[56, 59],56}, {[60, 65],60}, {[66, 71],66},
       * {[72, 79],72}, {[80, 84],80}}.
       */

      if (power < 8 || power > 84)
        {
          wlerr("Failed to set transmit power =%d\n", power);
          return -ENOSYS;
        }

      esp_wifi_set_max_tx_power(power);
      return OK;
    }
  else
    {
      ret = esp_wifi_get_max_tx_power(&power);
      if (ret)
        {
          wlerr("Failed to get transmit power ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      iwr->u.txpower.disabled = 0;
      iwr->u.txpower.flags    = IW_TXPOW_DBM;
      iwr->u.txpower.value    = power / 4;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_get_channel_range
 *
 * Description:
 *   Get station range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_channel(struct iwreq *iwr, bool set)
{
  int ret;
  int k;
  wifi_country_t country;
  struct iw_range *range;

  if (set)
    {
      return -ENOSYS;
    }
  else
    {
      ret = esp_wifi_get_country(&country);
      if (ret)
        {
          wlerr("Failed to get country info ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      range = (struct iw_range *)iwr->u.data.pointer;
      range->num_frequency = country.nchan;
      for (k = 1; k <= range->num_frequency; k++)
        {
          range->freq[k - 1].i = k;
          range->freq[k - 1].e = 0;
          range->freq[k - 1].m = 2407 + 5 * k;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_country(struct iwreq *iwr, bool set)
{
  int ret;
  char *country_code;
  wifi_country_t country;

  if (set)
    {
      memset(&country, 0x00, sizeof(wifi_country_t));
      country.schan  = 1;
      country.policy = 0;

      country_code = (char *)iwr->u.data.pointer;
      if (strlen(country_code) != 2)
        {
          wlerr("Invalid input arguments\n");
          return -EINVAL;
        }

      if (strncmp(country_code, "US", 3) == 0 ||
          strncmp(country_code, "CA", 3) == 0)
        {
          country.nchan  = 11;
        }
      else if(strncmp(country_code, "JP", 3) == 0)
        {
          country.nchan  = 14;
        }
      else
        {
          country.nchan  = 13;
        }

      memcpy(country.cc, country_code, 2);
      ret = esp_wifi_set_country(&country);
      if (ret)
        {
          wlerr("Failed to  Configure country ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_rssi
 *
 * Description:
 *   Get Wi-Fi sensitivity (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_sta_rssi(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_ap_record_t ap_info;

  if (set)
    {
      return -ENOSYS;
    }
  else
    {
      if (g_sta_connected == false)
        {
          iwr->u.sens.value = 128;
          return OK;
        }

      ret = esp_wifi_sta_get_ap_info(&ap_info);
      if (ret)
        {
          wlerr("Failed to get AP record ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      iwr->u.sens.value = -(ap_info.rssi);
    }

  return OK;
}
#endif

/****************************************************************************
 * SoftAP functions
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_SOFTAP

/****************************************************************************
 * Name: esp_wifi_softap_start
 *
 * Description:
 *   Start Wi-Fi SoftAP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_start(void)
{
  int ret;
  wifi_mode_t mode;

  esp_wifi_lock(true);

  ret = esp_wifi_stop();
  if (ret)
    {
      wlinfo("Failed to stop Wi-Fi ret=%d\n", ret);
    }

#ifdef ESP32_WLAN_HAS_STA
  if (g_sta_started)
    {
      mode = WIFI_MODE_APSTA;
    }
#else
  mode = WIFI_MODE_AP;
#endif

  ret = esp_wifi_set_mode(mode);
  if (ret)
    {
      wlerr("Failed to set Wi-Fi mode=%d ret=%d\n", mode, ret);
      ret = wifi_errno_trans(ret);
      goto errout_set_mode;
    }

  ret = esp_wifi_start();
  if (ret)
    {
      wlerr("Failed to start Wi-Fi with mode=%d ret=%d\n", mode, ret);
      ret = wifi_errno_trans(ret);
      goto errout_set_mode;
    }

  g_softap_started = true;

  wlinfo("OK to start Wi-Fi SoftAP\n");

  esp_wifi_lock(false);
  return OK;

errout_set_mode:
  esp_wifi_lock(false);
  return ret;
}

/****************************************************************************
 * Name: esp_wifi_softap_stop
 *
 * Description:
 *   Stop Wi-Fi SoftAP.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_stop(void)
{
  int ret;

  esp_wifi_lock(true);

  ret = esp_wifi_stop();
  if (ret)
    {
      wlinfo("Failed to stop Wi-Fi ret=%d\n", ret);
    }

  g_softap_started = false;

#ifdef ESP32_WLAN_HAS_STA
  if (g_sta_started)
    {
      ret = esp_wifi_set_mode(WIFI_MODE_STA);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi AP mode ret=%d\n", ret);
          ret = wifi_errno_trans(ret);
          goto errout_set_mode;
        }

      ret = esp_wifi_start();
      if (ret)
        {
          wlerr("Failed to start Wi-Fi AP ret=%d\n", ret);
          ret = wifi_errno_trans(ret);
          goto errout_set_mode;
        }
    }
#endif

  wlinfo("OK to stop Wi-Fi SoftAP\n");

  esp_wifi_lock(false);
  return OK;

#ifdef ESP32_WLAN_HAS_STA
errout_set_mode:
  esp_wifi_lock(true);
  return ret;
#endif
}

/****************************************************************************
 * Name: esp_wifi_softap_send_data
 *
 * Description:
 *   Use Wi-Fi SoftAP interface to send 802.3 frame
 *
 * Input Parameters:
 *   pbuf - Packet buffer pointer
 *   len  - Packet length
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_send_data(void *pbuf, uint32_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_AP, pbuf, len);

  return wifi_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_wifi_softap_register_recv_cb
 *
 * Description:
 *   Register Wi-Fi SoftAP receive packet callback function
 *
 * Input Parameters:
 *   recv_cb - Receive callback function
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_register_recv_cb(int (*recv_cb)(void *buffer,
                                                    uint16_t len,
                                                    void *eb))
{
  int ret;

  ret = esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t)recv_cb);

  return wifi_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_wifi_softap_register_txdone_cb
 *
 * Description:
 *   Register the SoftAP TX done callback function.
 *
 * Input Parameters:
 *   cb - The callback function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_softap_register_txdone_cb(wifi_txdone_cb_t cb)
{
  g_softap_txdone_cb = cb;
}

/****************************************************************************
 * Name: esp_wifi_softap_read_mac
 *
 * Description:
 *   Read SoftAP interface MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_softap_read_mac(uint8_t *mac)
{
  return esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
}

/****************************************************************************
 * Name: esp_wifi_softap_password
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP password
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_password(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_encode_ext *ext = iwr->u.encoding.pointer;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[PWD_MAX_LEN + 1];
#endif

  DEBUGASSERT(ext != NULL);

  pdata = ext->key;
  len   = ext->key_len;

  if (set && len > PWD_MAX_LEN)
    {
      return -EINVAL;
    }

  ret = esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);
  if (ret)
    {
      wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
      return wifi_errno_trans(ret);
    }

  ext   = iwr->u.encoding.pointer;
  pdata = ext->key;
  len   = ext->key_len;

  if (set)
    {
      /* Clear the password field and copy the user password to it */

      memset(wifi_cfg.ap.password, 0x0, PWD_MAX_LEN);
      memcpy(wifi_cfg.ap.password, pdata, len);

      /* Enable the WPA2 password by default */

      wifi_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

      /* Setup the config to the SoftAP */

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.ap.password, PWD_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.ap.password, len);
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi SoftAP password=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_essid
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP ESSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_essid(struct iwreq *iwr, bool set)
{
  int ret;
  int size;
  wifi_config_t wifi_cfg;
  struct iw_point *essid = &iwr->u.essid;
  uint8_t *pdata;
  uint8_t len;
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  char buf[SSID_MAX_LEN + 1];
#endif

  DEBUGASSERT(essid != NULL);

  pdata = essid->pointer;
  len   = essid->length;

  if (set && len > SSID_MAX_LEN)
    {
      return -EINVAL;
    }

  ret = esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);
  if (ret)
    {
      wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
      return wifi_errno_trans(ret);
    }

  if (set)
    {
      memset(wifi_cfg.ap.ssid, 0x0, SSID_MAX_LEN);
      memcpy(wifi_cfg.ap.ssid, pdata, len);

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      size = strnlen((char *)wifi_cfg.ap.ssid, SSID_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          len = size;
          memcpy(pdata, wifi_cfg.ap.ssid, len);
        }
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  memcpy(buf, pdata, len);
  buf[len] = 0;
  wlinfo("Wi-Fi SoftAP ssid=%s len=%d\n", buf, len);
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_bssid
 *
 * Description:
 *   Set/Get Wi-Fi softAP BSSID
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set   - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_bssid(struct iwreq *iwr, bool set)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wifi_softap_connect
 *
 * Description:
 *   Trigger Wi-Fi SoftAP accept connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_connect(void)
{
  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_disconnect
 *
 * Description:
 *   Trigger Wi-Fi SoftAP drop connection action
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_disconnect(void)
{
  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_mode
 *
 * Description:
 *   Set/Get Wi-Fi SoftAP mode code.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_mode(struct iwreq *iwr, bool set)
{
  if (set == false)
    {
      iwr->u.mode = IW_MODE_MASTER;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_auth
 *
 * Description:
 *   Set/get authentication mode params.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_auth(struct iwreq *iwr, bool set)
{
  int ret;
  int cmd;
  wifi_config_t wifi_cfg;

  if (set)
    {
      ret = esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }

      cmd = iwr->u.param.flags & IW_AUTH_INDEX;
      switch (cmd)
        {
          case IW_AUTH_WPA_VERSION:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_WPA_VERSION_DISABLED:
                    wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
                    break;

                  case IW_AUTH_WPA_VERSION_WPA:
                    wifi_cfg.ap.authmode = WIFI_AUTH_WPA_PSK;
                    break;

                  case IW_AUTH_WPA_VERSION_WPA2:
                    wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
                    break;

                  default:
                    wlerr("Invalid wpa version %" PRId32 "\n",
                          iwr->u.param.value);
                    return -EINVAL;
                }
            }

            break;
          case IW_AUTH_CIPHER_PAIRWISE:
          case IW_AUTH_CIPHER_GROUP:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_CIPHER_NONE:
                    wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
                    break;

                  case IW_AUTH_CIPHER_WEP40:
                  case IW_AUTH_CIPHER_WEP104:
                    wifi_cfg.ap.authmode = WIFI_AUTH_WEP;
                    break;

                  case IW_AUTH_CIPHER_TKIP:
                    wifi_cfg.ap.authmode = WIFI_AUTH_WPA_PSK;
                    break;

                  case IW_AUTH_CIPHER_CCMP:
                  case IW_AUTH_CIPHER_AES_CMAC:
                    wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
                    break;

                  default:
                    wlerr("Invalid cipher mode %" PRId32 "\n",
                          iwr->u.param.value);
                    return -EINVAL;
                }
            }

            break;
          case IW_AUTH_KEY_MGMT:
          case IW_AUTH_TKIP_COUNTERMEASURES:
          case IW_AUTH_DROP_UNENCRYPTED:
          case IW_AUTH_80211_AUTH_ALG:
          case IW_AUTH_WPA_ENABLED:
          case IW_AUTH_RX_UNENCRYPTED_EAPOL:
          case IW_AUTH_ROAMING_CONTROL:
          case IW_AUTH_PRIVACY_INVOKED:
          default:
            wlerr("Unknown cmd %d\n", cmd);
            return -EINVAL;
        }

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      return -ENOSYS;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_freq
 *
 * Description:
 *   Set/Get SoftAP frequency.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_freq(struct iwreq *iwr, bool set)
{
  int ret;
  wifi_config_t wifi_cfg;

  ret = esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);
  if (ret)
    {
      wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
      return wifi_errno_trans(ret);
    }

  if (set)
    {
      int channel = esp_freq_to_channel(iwr->u.freq.m);

      wifi_cfg.ap.channel = channel;

      ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
          return wifi_errno_trans(ret);
        }
    }
  else
    {
      iwr->u.freq.flags = IW_FREQ_FIXED;
      iwr->u.freq.e     = 0;
      iwr->u.freq.m     = 2407 + 5 * wifi_cfg.ap.channel;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_softap_get_bitrate
 *
 * Description:
 *   Get SoftAP default bit rate (Mbps).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_bitrate(struct iwreq *iwr, bool set)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wifi_softap_txpower
 *
 * Description:
 *   Get SoftAP transmit power (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_txpower(struct iwreq *iwr, bool set)
{
  return esp_wifi_sta_txpower(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_softap_channel
 *
 * Description:
 *   Get SoftAP range of channel parameters.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_channel(struct iwreq *iwr, bool set)
{
  return esp_wifi_sta_channel(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_softap_country
 *
 * Description:
 *   Configure country info.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_country(struct iwreq *iwr, bool set)
{
  return esp_wifi_sta_country(iwr, set);
}

/****************************************************************************
 * Name: esp_wifi_softap_rssi
 *
 * Description:
 *   Get Wi-Fi sensitivity (dBm).
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *   set - true: set data; false: get data
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_softap_rssi(struct iwreq *iwr, bool set)
{
  return -ENOSYS;
}

#endif

/****************************************************************************
 * Name: esp_wifi_stop_callback
 *
 * Description:
 *   Callback to stop Wi-Fi
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_wifi_stop_callback(void)
{
  wlinfo("INFO: Try to stop Wi-Fi\n");

  int ret = esp_wifi_stop();
  if (ret)
    {
      wlerr("ERROR: Failed to stop Wi-Fi ret=%d\n", ret);
    }
  else
    {
      nxsig_sleep(1);
    }
}
