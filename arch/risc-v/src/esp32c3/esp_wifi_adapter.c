/****************************************************************************
 * arch/risc-v/src/esp32c3/esp_wifi_adapter.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "esp_irq.h"
#include "esp_hr_timer.h"

#include "esp_types.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_private/wifi_os_adapter.h"
#include "esp_private/wifi.h"
#include "esp_phy_init.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/syscon_reg.h"
#include "soc/system_reg.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"
#include "os.h"
#include "esp_smartconfig.h"
#include "private/esp_coexist_internal.h"
#include "rom/ets_sys.h"
#include "private/esp_modem_wrapper.h"

#include "esp_wlan.h"
#include "esp_wifi_adapter.h"
#include "esp_wifi_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MHZ (1000000)

#define PHY_RF_MASK   ((1 << PHY_BT_MODULE) | (1 << PHY_WIFI_MODULE))

#define WIFI_CONNECT_TIMEOUT  CONFIG_ESPRESSIF_WIFI_CONNECT_TIMEOUT

#define ESP_WIFI_11B_MAX_BITRATE       11
#define ESP_WIFI_11G_MAX_BITRATE       54
#define ESP_WIFI_11N_MCS7_HT20_BITRATE 72
#define ESP_WIFI_11N_MCS7_HT40_BITRATE 150

#ifndef CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL
#define CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL 3
#endif

#define DEFAULT_LISTEN_INTERVAL CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL

#define RTC_CLK_CAL_FRACT  19  //!< Number of fractional bits in values returned by rtc_clk_cal

#define ets_timer       _ETSTIMER_

/* CONFIG_POWER_SAVE_MODEM */

#if defined(CONFIG_ESP_POWER_SAVE_MIN_MODEM)
#  define DEFAULT_PS_MODE WIFI_PS_MIN_MODEM
#elif defined(CONFIG_ESP_POWER_SAVE_MAX_MODEM)
#  define DEFAULT_PS_MODE WIFI_PS_MAX_MODEM
#elif defined(CONFIG_ESP_POWER_SAVE_NONE)
#  define DEFAULT_PS_MODE WIFI_PS_NONE
#else
#  define DEFAULT_PS_MODE WIFI_PS_NONE
#endif

#define ESP_MAX_PRIORITIES (25)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Wi-Fi event ID */

enum wifi_adpt_evt_e
{
  WIFI_ADPT_EVT_SCAN_DONE = 0,
  WIFI_ADPT_EVT_STA_START,
  WIFI_ADPT_EVT_STA_CONNECT,
  WIFI_ADPT_EVT_STA_DISCONNECT,
  WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE,
  WIFI_ADPT_EVT_STA_STOP,
  WIFI_ADPT_EVT_AP_START,
  WIFI_ADPT_EVT_AP_STOP,
  WIFI_ADPT_EVT_AP_STACONNECTED,
  WIFI_ADPT_EVT_AP_STADISCONNECTED,
  WIFI_ADPT_EVT_MAX,
};

/* Wi-Fi Station state */

enum wifi_sta_state
{
  WIFI_STA_STATE_NULL,
  WIFI_STA_STATE_START,
  WIFI_STA_STATE_CONNECT,
  WIFI_STA_STATE_DISCONNECT,
  WIFI_STA_STATE_STOP
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

/* Wi-Fi event callback function */

typedef void (*wifi_evt_cb_t)(void *p);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Private functions order is defined as:
 *  - A first block containing the functions in the same order as of the
 *    ESP-IDF' corresponding `esp_adapter.c` to ease comparison;
 *  - A second block of auxiliary functions block ordered by ascending;
 */

/* First block of functions */

static void *wifi_zalloc_wrapper(size_t size);
static void *wifi_create_queue(int queue_len, int item_size);
static void wifi_delete_queue(wifi_static_queue_t *queue);
static void *wifi_create_queue_wrapper(int queue_len, int item_size);
static void wifi_delete_queue_wrapper(void *queue);
static void set_intr_wrapper(int32_t cpu_no, uint32_t intr_source,
                             uint32_t intr_num, int32_t intr_prio);
static void clear_intr_wrapper(uint32_t intr_source, uint32_t intr_num);
static void set_isr_wrapper(int32_t n, void *f, void *arg);
static void enable_intr_wrapper(uint32_t intr_mask);
static void disable_intr_wrapper(uint32_t intr_mask);
static bool is_from_isr_wrapper(void);
static void wifi_thread_semphr_free(void *data);
static void *wifi_thread_semphr_get_wrapper(void);
static void *recursive_mutex_create_wrapper(void);
static void *mutex_create_wrapper(void);
static void mutex_delete_wrapper(void *mutex);
static int32_t mutex_lock_wrapper(void *mutex);
static int32_t mutex_unlock_wrapper(void *mutex);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static int32_t queue_send_wrapper(void *queue,
                                  void *item,
                                  uint32_t block_time_tick);
static int32_t queue_send_from_isr_wrapper(void *queue,
                                           void *item,
                                           void *hptw);
static int32_t queue_send_to_back_wrapper(void *queue,
                                          void *item,
                                          uint32_t block_time_tick);
static int32_t queue_send_to_front_wrapper(void *queue,
                                           void *item,
                                           uint32_t block_time_tick);
static int32_t queue_recv_wrapper(void *queue,
                                  void *item,
                                  uint32_t block_time_tick);
static uint32_t event_group_wait_bits_wrapper(void *event,
                                              uint32_t bits_to_wait_for,
                                              int clear_on_exit,
                                              int wait_for_all_bits,
                                              uint32_t block_time_tick);
static int32_t task_create_pinned_to_core_wrapper(void *task_func,
                                                  const char *name,
                                                  uint32_t stack_depth,
                                                  void *param,
                                                  uint32_t prio,
                                                  void *task_handle,
                                                  uint32_t core_id);
static int32_t task_create_wrapper(void *task_func,
                                   const char *name,
                                   uint32_t stack_depth,
                                   void *param,
                                   uint32_t prio,
                                   void *task_handle);
static int32_t task_ms_to_tick_wrapper(uint32_t ms);
static int32_t task_get_max_priority_wrapper(void);
int32_t esp_event_post_wrapper(const char *event_base,
                               int32_t event_id,
                               void *event_data,
                               size_t event_data_size,
                               uint32_t ticks);
static void wifi_apb80m_request_wrapper(void);
static void wifi_apb80m_release_wrapper(void);
static void esp_phy_enable_wrapper(void);
static void esp_phy_disable_wrapper(void);
static void timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat);
static void wifi_reset_mac_wrapper(void);
static void wifi_rtc_enable_iso_wrapper(void);
static void IRAM_ATTR wifi_rtc_disable_iso_wrapper(void);
static void wifi_clock_enable_wrapper(void);
static void wifi_clock_disable_wrapper(void);
static int get_time_wrapper(void *t);
static void *realloc_internal_wrapper(void *ptr, size_t size);
static void *calloc_internal_wrapper(size_t n, size_t size);
static void *zalloc_internal_wrapper(size_t size);
static int nvs_open_wrapper(const char *name, unsigned int open_mode,
                            uint32_t *out_handle);
static void esp_log_writev_wrapper(unsigned int level,
                                   const char *tag,
                                   const char *format,
                                   va_list args);
static void esp_log_write_wrapper(unsigned int level,
                                  const char *tag,
                                  const char *format, ...);
static int esp_read_mac_wrapper(uint8_t *mac, unsigned int type);
static int coex_init_wrapper(void);
static void coex_deinit_wrapper(void);
static int coex_enable_wrapper(void);
static void coex_disable_wrapper(void);
static uint32_t coex_status_get_wrapper(void);
static int coex_wifi_request_wrapper(uint32_t event,
                                     uint32_t latency,
                                     uint32_t duration);
static int coex_wifi_release_wrapper(uint32_t event);
static int coex_wifi_channel_set_wrapper(uint8_t primary,
                                         uint8_t secondary);
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
static void esp_empty_wrapper(void);

/* Second block of functions
 * These functions are auxiliary functions that are used by the first block
 * of functions or software adapters for the Wi-Fi driver
 */

static int esp_event_id_map(int event_id);
static void esp_evt_work_cb(void *arg);
static int esp_freq_to_channel(uint16_t freq);
static uint32_t esp_get_free_heap_size(void);
static void *event_group_create_wrapper(void);
static void event_group_delete_wrapper(void *event);
static uint32_t event_group_set_bits_wrapper(void *event, uint32_t bits);
static uint32_t event_group_clear_bits_wrapper(void *event, uint32_t bits);
static int esp_int_adpt_cb(int irq, void *context, void *arg);
static int esp_nvs_commit(uint32_t handle);
static int esp_nvs_erase_key(uint32_t handle, const char *key);
static int esp_nvs_get_blob(uint32_t handle,
                            const char *key,
                            void *out_value,
                            size_t *length);
static int esp_nvs_set_blob(uint32_t handle,
                            const char *key,
                            const void *value,
                            size_t length);
static int esp_nvs_get_i8(uint32_t handle,
                          const char *key,
                          int8_t *out_value);
static int esp_nvs_set_i8(uint32_t handle, const char *key, int8_t value);
static int esp_nvs_get_u8(uint32_t handle,
                          const char *key,
                          uint8_t *out_value);
static int esp_nvs_set_u8(uint32_t handle, const char *key, uint8_t value);
static int esp_nvs_get_u16(uint32_t handle,
                           const char *key,
                           uint16_t *out_value);
static int esp_nvs_set_u16(uint32_t handle, const char *key, uint16_t value);
static void esp_nvs_close(uint32_t handle);
static void esp_update_time(struct timespec *timespec, uint32_t ticks);
static int esp_wifi_auth_trans(uint32_t wifi_auth);
static int esp_wifi_cipher_trans(uint32_t wifi_cipher);
static int esp_wifi_lock(bool lock);
static uint32_t queue_msg_waiting_wrapper(void *queue);
static void task_delay_wrapper(uint32_t tick);
static void task_delete_wrapper(void *task_handle);
static void *task_get_current_task_wrapper(void);
static void vqueue_delete_adapter(void *queue);
static void vsemaphore_delete_adapter(void *semphr);
static void *xqueue_create_adapter(uint32_t queue_len, uint32_t item_size);
static int32_t xqueue_send_adapter(void *queue,
                                   void *item,
                                   uint32_t ticks,
                                   int prio);
void *xsemaphore_create_counting_adapter(uint32_t max, uint32_t init);

#ifdef CONFIG_PM
extern void wifi_apb80m_request(void);
extern void wifi_apb80m_release(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Wi-Fi event private data */

static struct work_s g_wifi_evt_work;
static sq_queue_t g_wifi_evt_queue;
static struct wifi_notify g_wifi_notify[WIFI_ADPT_EVT_MAX];
static mutex_t g_wifiexcl_lock = NXMUTEX_INITIALIZER;

/* Wi-Fi adapter reference */

static int g_wifi_ref;

#ifdef ESP_WLAN_HAS_STA

/* If reconnect automatically */

static bool g_sta_reconnect;

/* If Wi-Fi sta starts */

static bool g_sta_started;

/* If Wi-Fi sta connected */

static bool g_sta_connected;

/* Wi-Fi interface configuration */

static wifi_config_t g_sta_wifi_cfg;

#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP

/* If Wi-Fi SoftAP starts */

static bool g_softap_started;

/* Wi-Fi interface configuration */

static wifi_config_t g_softap_wifi_cfg;

#endif /* ESP_WLAN_HAS_SOFTAP */

/* Device specific lock */

static spinlock_t g_lock;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Wi-Fi OS adapter data */

wifi_osi_funcs_t g_wifi_osi_funcs =
{
  ._version = ESP_WIFI_OS_ADAPTER_VERSION,
  ._env_is_chip = esp_coex_common_env_is_chip_wrapper,
  ._set_intr = set_intr_wrapper,
  ._clear_intr = clear_intr_wrapper,
  ._set_isr = set_isr_wrapper,
  ._ints_on = enable_intr_wrapper,
  ._ints_off = disable_intr_wrapper,
  ._is_from_isr = is_from_isr_wrapper,
  ._spin_lock_create = esp_coex_common_spin_lock_create_wrapper,
  ._spin_lock_delete = free,
  ._wifi_int_disable = esp_coex_common_int_disable_wrapper,
  ._wifi_int_restore = esp_coex_common_int_restore_wrapper,
  ._task_yield_from_isr = esp_coex_common_task_yield_from_isr_wrapper,
  ._semphr_create = esp_coex_common_semphr_create_wrapper,
  ._semphr_delete = esp_coex_common_semphr_delete_wrapper,
  ._semphr_take = esp_coex_common_semphr_take_wrapper,
  ._semphr_give = esp_coex_common_semphr_give_wrapper,
  ._wifi_thread_semphr_get = wifi_thread_semphr_get_wrapper,
  ._mutex_create = mutex_create_wrapper,
  ._recursive_mutex_create = recursive_mutex_create_wrapper,
  ._mutex_delete = mutex_delete_wrapper,
  ._mutex_lock = mutex_lock_wrapper,
  ._mutex_unlock = mutex_unlock_wrapper,
  ._queue_create = queue_create_wrapper,
  ._queue_delete = vqueue_delete_adapter,
  ._queue_send = queue_send_wrapper,
  ._queue_send_from_isr = queue_send_from_isr_wrapper,
  ._queue_send_to_back = queue_send_to_back_wrapper,
  ._queue_send_to_front = queue_send_to_front_wrapper,
  ._queue_recv = queue_recv_wrapper,
  ._queue_msg_waiting = queue_msg_waiting_wrapper,
  ._event_group_create = event_group_create_wrapper,
  ._event_group_delete = event_group_delete_wrapper,
  ._event_group_set_bits = event_group_set_bits_wrapper,
  ._event_group_clear_bits = event_group_clear_bits_wrapper,
  ._event_group_wait_bits = event_group_wait_bits_wrapper,
  ._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
  ._task_create = task_create_wrapper,
  ._task_delete = task_delete_wrapper,
  ._task_delay = task_delay_wrapper,
  ._task_ms_to_tick = task_ms_to_tick_wrapper,
  ._task_get_current_task = task_get_current_task_wrapper,
  ._task_get_max_priority = task_get_max_priority_wrapper,
  ._malloc = malloc,
  ._free = free,
  ._event_post = esp_event_post_wrapper,
  ._get_free_heap_size = esp_get_free_heap_size,
  ._rand = esp_random,
  ._dport_access_stall_other_cpu_start_wrap =
      esp_empty_wrapper,
  ._dport_access_stall_other_cpu_end_wrap =
      esp_empty_wrapper,
  ._wifi_apb80m_request = wifi_apb80m_request_wrapper,
  ._wifi_apb80m_release = wifi_apb80m_release_wrapper,
  ._phy_disable = esp_phy_disable_wrapper,
  ._phy_enable = esp_phy_enable_wrapper,
  ._phy_update_country_info = esp_phy_update_country_info,
  ._read_mac = esp_read_mac_wrapper,
  ._timer_arm = timer_arm_wrapper,
  ._timer_disarm = esp_coex_common_timer_disarm_wrapper,
  ._timer_done = esp_coex_common_timer_done_wrapper,
  ._timer_setfn = esp_coex_common_timer_setfn_wrapper,
  ._timer_arm_us = esp_coex_common_timer_arm_us_wrapper,
  ._wifi_reset_mac = wifi_reset_mac_wrapper,
  ._wifi_clock_enable = wifi_clock_enable_wrapper,
  ._wifi_clock_disable = wifi_clock_disable_wrapper,
  ._wifi_rtc_enable_iso = wifi_rtc_enable_iso_wrapper,
  ._wifi_rtc_disable_iso = wifi_rtc_disable_iso_wrapper,
  ._esp_timer_get_time = (int64_t(*)(void))esp_hr_timer_time_us,
  ._nvs_set_i8 = esp_nvs_set_i8,
  ._nvs_get_i8 = esp_nvs_get_i8,
  ._nvs_set_u8 = esp_nvs_set_u8,
  ._nvs_get_u8 = esp_nvs_get_u8,
  ._nvs_set_u16 = esp_nvs_set_u16,
  ._nvs_get_u16 = esp_nvs_get_u16,
  ._nvs_open = nvs_open_wrapper,
  ._nvs_close = esp_nvs_close,
  ._nvs_commit = esp_nvs_commit,
  ._nvs_set_blob = esp_nvs_set_blob,
  ._nvs_get_blob = esp_nvs_get_blob,
  ._nvs_erase_key = esp_nvs_erase_key,
  ._get_random = os_get_random,
  ._get_time = get_time_wrapper,
  ._random = os_random,
  ._slowclk_cal_get = esp_coex_common_clk_slowclk_cal_get_wrapper,
  ._log_write = esp_log_write_wrapper,
  ._log_writev = esp_log_writev_wrapper,
  ._log_timestamp = esp_log_timestamp,
  ._malloc_internal =  esp_coex_common_malloc_internal_wrapper,
  ._realloc_internal = realloc_internal_wrapper,
  ._calloc_internal = calloc_internal_wrapper,
  ._zalloc_internal = zalloc_internal_wrapper,
  ._wifi_malloc = wifi_malloc,
  ._wifi_realloc = wifi_realloc,
  ._wifi_calloc = wifi_calloc,
  ._wifi_zalloc = wifi_zalloc_wrapper,
  ._wifi_create_queue = wifi_create_queue_wrapper,
  ._wifi_delete_queue = wifi_delete_queue_wrapper,
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
  ._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Private functions order is defined as:
 *  - A first block containing the functions in the same order as of the
 *    ESP-IDF' corresponding `esp_adapter.c` to ease comparison;
 *  - A second block of auxiliary functions block ordered by ascending;
 */

/* First block of functions */

/****************************************************************************
 * Name: wifi_zalloc_wrapper
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

static IRAM_ATTR void *wifi_zalloc_wrapper(size_t size)
{
  return zalloc(size);
}

/****************************************************************************
 * Name: wifi_create_queue
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

static void *wifi_create_queue(int queue_len, int item_size)
{
  wifi_static_queue_t *wifi_queue;

  wifi_queue = kmm_malloc(sizeof(wifi_static_queue_t));
  if (!wifi_queue)
    {
      wlerr("Failed to kmm_malloc\n");
      return NULL;
    }

  wifi_queue->handle = xqueue_create_adapter(queue_len, item_size);
  if (!wifi_queue->handle)
    {
      wlerr("Failed to create queue\n");
      kmm_free(wifi_queue);
      return NULL;
    }

  return wifi_queue;
}

/****************************************************************************
 * Name: wifi_delete_queue
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

static void wifi_delete_queue(wifi_static_queue_t *queue)
{
  if (queue)
    {
      vqueue_delete_adapter(queue->handle);
      kmm_free(queue);
    }
}

/****************************************************************************
 * Name: wifi_create_queue_wrapper
 *
 * Description:
 *   This function creates a new queue for Wi-Fi operations. It is a wrapper
 *   around the wifi_create_queue function, providing a consistent interface
 *   for the Wi-Fi module.
 *
 * Input Parameters:
 *   queue_len - The maximum number of items that the queue can hold.
 *   item_size - The size of each item in the queue.
 *
 * Returned Value:
 *   A pointer to the newly created queue, or NULL if the operation failed.
 *
 ****************************************************************************/

static void *wifi_create_queue_wrapper(int queue_len, int item_size)
{
  return wifi_create_queue(queue_len, item_size);
}

/****************************************************************************
 * Name: wifi_delete_queue_wrapper
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

static void wifi_delete_queue_wrapper(void *queue)
{
  wifi_delete_queue(queue);
}

/****************************************************************************
 * Name: set_intr_wrapper
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

static void set_intr_wrapper(int32_t cpu_no, uint32_t intr_source,
                             uint32_t intr_num, int32_t intr_prio)
{
  wlinfo("cpu_no=%" PRId32 ", intr_source=%" PRIu32
         ", intr_num=%" PRIu32 ", intr_prio=%" PRId32 "\n",
         cpu_no, intr_source, intr_num, intr_prio);

  esp_route_intr(intr_source, intr_num, intr_prio, ESP_IRQ_TRIGGER_LEVEL);
  esp_set_irq(ESP_SOURCE2IRQ(intr_source), intr_num);
}

/****************************************************************************
 * Name: clear_intr_wrapper
 *
 * Description:
 *   This function is intended to clear a specific interrupt. However, this
 *   functionality is not supported in the current implementation.
 *
 * Input Parameters:
 *   intr_source - The source of the interrupt.
 *   intr_num - The number of the interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR clear_intr_wrapper(uint32_t intr_source,
                                         uint32_t intr_num)
{
}

/****************************************************************************
 * Name: set_isr_wrapper
 *
 * Description:
 *   Register interrupt function
 *
 * Input Parameters:
 *   n   - CPU interrupt number
 *   f   - Interrupt function
 *   arg - Function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void set_isr_wrapper(int32_t n, void *f, void *arg)
{
  int ret;
  uint32_t tmp;
  struct irq_adpt *adapter;
  int irq = esp_get_irq(n);

  wlinfo("n=%ld f=%p arg=%p irq=%d\n", n, f, arg, irq);

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
      wlerr("Failed to alloc %ld memory\n", tmp);
      PANIC();
      return;
    }

  adapter->func = f;
  adapter->arg = arg;

  ret = irq_attach(irq, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("Failed to attach IRQ %d\n", irq);
      PANIC();
      return;
    }
}

/****************************************************************************
 * Name: enable_intr_wrapper
 *
 * Description:
 *   Enable a specific Wi-Fi interrupt.
 *
 * Input Parameters:
 *   intr_mask - A mask where the bit corresponding to the interrupt to be
 *               enabled is set.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void enable_intr_wrapper(uint32_t intr_mask)
{
  int cpuint = __builtin_ffs(intr_mask) - 1;
  int irq = esp_get_irq(cpuint);

  wlinfo("intr_mask=%08lx cpuint=%d irq=%d\n", intr_mask, cpuint, irq);

  up_enable_irq(irq);
}

/****************************************************************************
 * Name: disable_intr_wrapper
 *
 * Description:
 *   Disable a specific Wi-Fi interrupt.
 *
 * Input Parameters:
 *   intr_mask - A mask where the bit corresponding to the interrupt to be
 *               disabled is set.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void disable_intr_wrapper(uint32_t intr_mask)
{
  int cpuint = __builtin_ffs(intr_mask) - 1;
  int irq = esp_get_irq(cpuint);

  wlinfo("intr_mask=%08lx cpuint=%d irq=%d\n", intr_mask, cpuint, irq);

  up_disable_irq(irq);
}

/****************************************************************************
 * Name: is_from_isr_wrapper
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

static bool IRAM_ATTR is_from_isr_wrapper(void)
{
  return up_interrupt_context();
}

/****************************************************************************
 * Name: wifi_thread_semphr_free
 *
 * Description:
 *   Delete thread self's semaphore
 *
 * Input Parameters:
 *   data - Semaphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wifi_thread_semphr_free(void *data)
{
  void *sem = (void *)data;

  if (sem)
    {
      vsemaphore_delete_adapter(sem);
    }
}

/****************************************************************************
 * Name: wifi_thread_semphr_get_wrapper
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

static void *wifi_thread_semphr_get_wrapper(void)
{
  static int wifi_task_key = -1;
  int ret;
  void *sem;

  if (wifi_task_key < 0)
    {
      ret = task_tls_alloc(wifi_thread_semphr_free);
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
      sem = xsemaphore_create_counting_adapter(1, 0);
      if (!sem)
        {
          wlerr("Failed to create semaphore\n");
          return NULL;
        }

      ret = task_tls_set_value(wifi_task_key, (uintptr_t)sem);
      if (ret != OK)
        {
          wlerr("Failed to save semaphore on task local storage: %d\n", ret);
          vsemaphore_delete_adapter(sem);
          return NULL;
        }
    }

  return sem;
}

/****************************************************************************
 * Name: recursive_mutex_create_wrapper
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

static void *recursive_mutex_create_wrapper(void)
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
 * Name: mutex_create_wrapper
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

static void *mutex_create_wrapper(void)
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
 * Name: mutex_delete_wrapper
 *
 * Description:
 *   Delete mutex
 *
 * Input Parameters:
 *   mutex - mutex data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mutex_delete_wrapper(void *mutex)
{
  pthread_mutex_t *mutex_data = (pthread_mutex_t *)mutex;

  pthread_mutex_destroy(mutex_data);
  kmm_free(mutex_data);
}

/****************************************************************************
 * Name: mutex_lock_wrapper
 *
 * Description:
 *   Lock mutex
 *
 * Input Parameters:
 *   mutex - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t IRAM_ATTR mutex_lock_wrapper(void *mutex)
{
  int ret;
  pthread_mutex_t *mutex_data = (pthread_mutex_t *)mutex;

  ret = pthread_mutex_lock(mutex_data);
  if (ret)
    {
      wlerr("Failed to lock mutex error=%d\n", ret);
    }

  return nuttx_err_to_freertos(ret);
}

/****************************************************************************
 * Name: mutex_unlock_wrapper
 *
 * Description:
 *   Unlock mutex
 *
 * Input Parameters:
 *   mutex - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t IRAM_ATTR mutex_unlock_wrapper(void *mutex)
{
  int ret;
  pthread_mutex_t *mutex_data = (pthread_mutex_t *)mutex;

  ret = pthread_mutex_unlock(mutex_data);
  if (ret)
    {
      wlerr("Failed to unlock mutex error=%d\n", ret);
    }

  return nuttx_err_to_freertos(ret);
}

/****************************************************************************
 * Name: queue_create_wrapper
 *
 * Description:
 *   This function creates a new queue adapter with the specified length and
 *   item size. It is a wrapper around the xqueue_create_adapter function.
 *
 * Input Parameters:
 *   queue_len - The maximum number of items that the queue can hold.
 *   item_size - The size of each item in the queue.
 *
 * Returned Value:
 *   A pointer to the newly created queue adapter, or NULL if the operation
 *   failed.
 *
 ****************************************************************************/

static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
  return xqueue_create_adapter(queue_len, item_size);
}

/****************************************************************************
 * Name: queue_send_wrapper
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

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t ticks)
{
  return xqueue_send_adapter(queue, item, ticks, 0);
}

/****************************************************************************
 * Name: queue_send_from_isr_wrapper
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

static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue,
                                                     void *item,
                                                     void *hptw)
{
  *(int *)hptw = 0;

  return xqueue_send_adapter(queue, item, 0, 0);
}

/****************************************************************************
 * Name: queue_send_to_back_wrapper
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

static int32_t queue_send_to_back_wrapper(void *queue,
                                          void *item,
                                          uint32_t ticks)
{
  return xqueue_send_adapter(queue, item, ticks, 0);
}

/****************************************************************************
 * Name: queue_send_to_front_wrapper
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

static int32_t queue_send_to_front_wrapper(void *queue,
                                           void *item,
                                           uint32_t ticks)
{
  return xqueue_send_adapter(queue, item, ticks, 1);
}

/****************************************************************************
 * Name: queue_recv_wrapper
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

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t ticks)
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
 * Name: event_group_wait_bits_wrapper
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t event_group_wait_bits_wrapper(void *event,
                                              uint32_t bits_to_wait_for,
                                              int clear_on_exit,
                                              int wait_for_all_bits,
                                              uint32_t block_time_tick)
{
  DEBUGPANIC();

  return false;
}

/****************************************************************************
 * Name: task_create_pinned_to_core_wrapper
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

static int32_t task_create_pinned_to_core_wrapper(void *entry,
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
  uint32_t target_prio = prio;

  if (target_prio < ESP_MAX_PRIORITIES)
    {
      target_prio += task_get_max_priority_wrapper() - ESP_MAX_PRIORITIES;
    }

  pid = kthread_create(name, target_prio, stack_depth, entry,
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
 * Name: task_create_wrapper
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

static int32_t task_create_wrapper(void *entry,
                                  const char *name,
                                  uint32_t stack_depth,
                                  void *param,
                                  uint32_t prio,
                                  void *task_handle)
{
  return task_create_pinned_to_core_wrapper(entry,
                                            name,
                                            stack_depth,
                                            param,
                                            prio,
                                            task_handle,
                                            UINT32_MAX);
}

/****************************************************************************
 * Name: task_ms_to_tick_wrapper
 *
 * Description:
 *   This function converts a duration from milliseconds to system ticks.
 *   It is a wrapper around the NuttX MSEC2TICK macro.
 *
 * Input Parameters:
 *   ms - The duration in milliseconds.
 *
 * Returned Value:
 *   The duration in system ticks.
 *
 ****************************************************************************/

static int32_t task_ms_to_tick_wrapper(uint32_t ms)
{
  return MSEC2TICK(ms);
}

/****************************************************************************
 * Name: task_get_max_priority_wrapper
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

static int32_t task_get_max_priority_wrapper(void)
{
  return SCHED_PRIORITY_MAX;
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

int32_t esp_event_post_wrapper(const char *event_base,
                               int32_t event_id,
                               void *event_data,
                               size_t event_data_size,
                               uint32_t ticks)
{
  size_t size;
  int32_t id;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;

  wlinfo("Event: base=%s id=%ld data=%p data_size=%d ticks=%lu\n",
         event_base, event_id, event_data, event_data_size, ticks);

  id = esp_event_id_map(event_id);
  if (id < 0)
    {
      wlinfo("No process event %ld\n", event_id);
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
 * Name: wifi_apb80m_request_wrapper
 *
 * Description:
 *   This function acquires the Wi-Fi lock in auto-sleep mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR wifi_apb80m_request_wrapper(void)
{
#ifdef CONFIG_PM
  wifi_apb80m_request();
#endif
}

/****************************************************************************
 * Name: wifi_apb80m_release_wrapper
 *
 * Description:
 *   This function releases the Wi-Fi lock in auto-sleep mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR wifi_apb80m_release_wrapper(void)
{
#ifdef CONFIG_PM
  wifi_apb80m_release();
#endif
}

/****************************************************************************
 * Name: esp_phy_enable_wrapper
 *
 * Description:
 *   This function enables the WiFi PHY. It first enables the PHY for the
 *   WiFi modem, then sets the WiFi PHY enable flag to 1.
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
  phy_wifi_enable_set(1);
}

/****************************************************************************
 * Name: esp_phy_disable_wrapper
 *
 * Description:
 *   This function disables the WiFi PHY. It first sets the WiFi PHY enable
 *   flag to 0, then disables the PHY for the WiFi modem.
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
  phy_wifi_enable_set(0);
  esp_phy_disable(PHY_MODEM_WIFI);
}

/****************************************************************************
 * Name: timer_arm_wrapper
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

static void IRAM_ATTR timer_arm_wrapper(void *ptimer,
                                        uint32_t tmout,
                                        bool repeat)
{
  ets_timer_arm(ptimer, tmout, repeat);
}

/****************************************************************************
 * Name: wifi_reset_mac_wrapper
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

static void wifi_reset_mac_wrapper(void)
{
  periph_module_reset(PERIPH_WIFI_MODULE);
}

/****************************************************************************
 * Name: wifi_rtc_enable_iso_wrapper
 *
 * Description:
 *   This function is a wrapper for enabling the isolation of the Wi-Fi
 *   Real-Time Clock (RTC). If the MAC/BB power down configuration option
 *   is set, it powers down the MAC/BB.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR wifi_rtc_enable_iso_wrapper(void)
{
#if CONFIG_MAC_BB_PD
  esp_mac_bb_power_down();
#endif
}

/****************************************************************************
 * Name: wifi_rtc_disable_iso_wrapper
 *
 * Description:
 *   This function is a wrapper for disabling the isolation of the Wi-Fi
 *   Real-Time Clock (RTC). If the MAC/BB power down configuration option
 *   is set, it powers up the MAC/BB.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR wifi_rtc_disable_iso_wrapper(void)
{
#if CONFIG_MAC_BB_PD
  esp_mac_bb_power_up();
#endif
}

/****************************************************************************
 * Name: wifi_clock_enable_wrapper
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

static void wifi_clock_enable_wrapper(void)
{
  wifi_module_enable();
}

/****************************************************************************
 * Name: wifi_clock_disable_wrapper
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

static void wifi_clock_disable_wrapper(void)
{
  wifi_module_disable();
}

/****************************************************************************
 * Name: get_time_wrapper
 *
 * Description:
 *   Get std C time
 *
 * Input Parameters:
 *   t - buffer to store time of type timeval
 *
 * Returned Value:
 *   Zero (OK) on success;  -1 is returned on failure with the errno variable
 *   set appropriately.
 *
 ****************************************************************************/

static int get_time_wrapper(void *t)
{
  return os_get_time(t);
}

/****************************************************************************
 * Name: realloc_internal_wrapper
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

static IRAM_ATTR void *realloc_internal_wrapper(void *ptr, size_t size)
{
  return kmm_realloc(ptr, size);
}

/****************************************************************************
 * Name: calloc_internal_wrapper
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

static IRAM_ATTR void *calloc_internal_wrapper(size_t n, size_t size)
{
  return kmm_calloc(n, size);
}

/****************************************************************************
 * Name: zalloc_internal_wrapper
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

static IRAM_ATTR void *zalloc_internal_wrapper(size_t size)
{
  return kmm_zalloc(size);
}

/****************************************************************************
 * Name: nvs_open_wrapper
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

static int nvs_open_wrapper(const char *name,
                            unsigned int open_mode,
                            uint32_t *out_handle)
{
  DEBUGPANIC();

  return -1;
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

static void esp_log_writev_wrapper(unsigned int level,
                                   const char *tag,
                                   const char *format,
                                   va_list args)
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
 * Name: esp_read_mac_wrapper
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

static int esp_read_mac_wrapper(uint8_t *mac, unsigned int type)
{
  return esp_read_mac(mac, type);
}

/****************************************************************************
 * Name: coex_init_wrapper
 *
 * Description:
 *   Init software coexist
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_init_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_init();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_deinit_wrapper
 *
 * Description:
 *   De-init software coexist
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_deinit_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  coex_deinit();
#endif
}

/****************************************************************************
 * Name: coex_enable_wrapper
 *
 * Description:
 *   Enable software coexist
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_enable_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_enable();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_disable_wrapper
 *
 * Description:
 *   Disable software coexist
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_disable_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  coex_disable();
#endif
}

/****************************************************************************
 * Name: coex_status_get_wrapper
 *
 * Description:
 *   Get software coexist status.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static IRAM_ATTR uint32_t coex_status_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_status_get();
#else
  return 0;
#endif
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

static int coex_wifi_request_wrapper(uint32_t event,
                                     uint32_t latency,
                                     uint32_t duration)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_wifi_request(event, latency, duration);
#else
  return 0;
#endif
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
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_wifi_release(event);
#else
  return 0;
#endif
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
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_wifi_channel_set(primary, secondary);
#else
  return 0;
#endif
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

static IRAM_ATTR int coex_event_duration_get_wrapper(uint32_t event,
                                                     uint32_t *duration)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_event_duration_get(event, duration);
#else
  return 0;
#endif
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
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_pti_get(event, pti);
#else
  return 0;
#endif
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
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                               uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  coex_schm_status_bit_clear(type, status);
#endif
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
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  coex_schm_status_bit_set(type, status);
#endif
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
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_schm_interval_set(interval);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_interval_get_wrapper
 *
 * Description:
 *   Get coexistence scheme interval.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Coexistence scheme interval
 *
 ****************************************************************************/

static uint32_t coex_schm_interval_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_schm_interval_get();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_curr_period_get_wrapper
 *
 * Description:
 *   Get current coexistence scheme period.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Coexistence scheme period
 *
 ****************************************************************************/

static uint8_t coex_schm_curr_period_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_schm_curr_period_get();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_curr_phase_get_wrapper
 *
 * Description:
 *   Get current coexistence scheme phase.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Coexistence scheme phase
 *
 ****************************************************************************/

static void *coex_schm_curr_phase_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
  return coex_schm_curr_phase_get();
#else
  return NULL;
#endif
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
#if CONFIG_SW_COEXIST_ENABLE
  return coex_register_start_cb(cb);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_process_restart_wrapper
 *
 * Description:
 *   Restart current coexistence scheme.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_schm_process_restart_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE
  return coex_schm_process_restart();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_register_cb_wrapper
 *
 * Description:
 *   Register callback for coexistence scheme.
 *
 * Input Parameters:
 *   type - callback type
 *   cb   - callback
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int coex_schm_register_cb_wrapper(int type, int(*cb)(int))
{
#if CONFIG_SW_COEXIST_ENABLE
  return coex_schm_register_callback(type, cb);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_schm_flexible_period_set_wrapper
 *
 * Description:
 *   This function sets the coexistence scheme flexible period. If the
 *   coexistence power management feature is enabled
 *   (CONFIG_ESP_COEX_POWER_MANAGEMENT), it calls the function
 *   coex_schm_flexible_period_set with the given period and returns its
 *   result. If the feature is not enabled, it returns 0.
 *
 * Input Parameters:
 *   period - The flexible period to set.
 *
 * Returned Value:
 *   ESP_OK on success, or the result of coex_schm_flexible_period_set.
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
 *   This function gets the coexistence scheme flexible period. If the
 *   coexistence power management feature is enabled
 *   (CONFIG_ESP_COEX_POWER_MANAGEMENT), it calls the function
 *   coex_schm_flexible_period_get and returns its result. If the feature is
 *   not enabled, it returns 1.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The coexistence scheme flexible period.
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
 * Name: esp_empty_wrapper
 *
 * Description:
 *   This function is an empty wrapper, designed to be used where a function
 *   pointer is required but no operation is needed.
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

/* Second block of functions
 * These functions are auxiliary functions that are used by the first block
 * of functions or software adapters for the Wi-Fi driver
 */

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

#ifdef ESP_WLAN_HAS_STA
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
#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP
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
#endif /* ESP_WLAN_HAS_SOFTAP */

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
  wifi_ps_type_t ps_type = DEFAULT_PS_MODE;

  while (1)
    {
      flags = spin_lock_irqsave(&g_lock);
      evt_adpt = (struct evt_adpt *)sq_remfirst(&g_wifi_evt_queue);
      spin_unlock_irqrestore(&g_lock, flags);
      if (!evt_adpt)
        {
          break;
        }

      /* Some of the following logic (eg. esp_wlan_sta_set_linkstatus)
       * can take net_lock(). To maintain the consistent locking order,
       * we take net_lock() here before taking esp_wifi_lock. Note that
       * net_lock() is a recursive lock.
       */

      net_lock();
      esp_wifi_lock(true);

      switch (evt_adpt->id)
        {
          case WIFI_ADPT_EVT_SCAN_DONE:
            esp_wifi_scan_event_parse();
            break;

#ifdef ESP_WLAN_HAS_STA
          case WIFI_ADPT_EVT_STA_START:
            wlinfo("Wi-Fi sta start\n");

            g_sta_connected = false;

            ret = esp_wifi_set_ps(ps_type);
            if (ret)
              {
                wlerr("Failed to set power save type\n");
                break;
              }
            else
              {
                wlinfo("INFO: Set ps type=%d\n", ps_type);
              }

            ret = esp_wifi_get_config(WIFI_IF_STA, &g_sta_wifi_cfg);
            if (ret)
              {
                wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
              }
            break;

          case WIFI_ADPT_EVT_STA_CONNECT:
            wlinfo("Wi-Fi sta connect\n");
            g_sta_connected = true;
            ret = esp_wlan_sta_set_linkstatus(true);
            if (ret < 0)
              {
                wlerr("ERROR: Failed to set Wi-Fi station link status\n");
              }

            break;

          case WIFI_ADPT_EVT_STA_DISCONNECT:
            wlinfo("Wi-Fi sta disconnect\n");
            g_sta_connected = false;
            ret = esp_wlan_sta_set_linkstatus(false);
            if (ret < 0)
              {
                wlerr("ERROR: Failed to set Wi-Fi station link status\n");
              }

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
#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP
          case WIFI_ADPT_EVT_AP_START:
            wlinfo("INFO: Wi-Fi softap start\n");

            ret = esp_wifi_set_ps(ps_type);
            if (ret)
              {
                wlerr("Failed to set power save type\n");
                break;
              }
            else
              {
                wlinfo("INFO: Set ps type=%d\n", ps_type);
              }

            ret = esp_wifi_get_config(WIFI_IF_AP, &g_softap_wifi_cfg);
            if (ret)
              {
                wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
              }
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
#endif /* ESP_WLAN_HAS_SOFTAP */
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
              wlwarn("nxsig_notification event ID=%ld failed: %d\n",
                     evt_adpt->id, ret);
            }
        }

      esp_wifi_lock(false);
      net_unlock();

      kmm_free(evt_adpt);
    }
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
 * Name: event_group_create_wrapper
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void *event_group_create_wrapper(void)
{
  DEBUGPANIC();

  return NULL;
}

/****************************************************************************
 * Name: event_group_delete_wrapper
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static void event_group_delete_wrapper(void *event)
{
  DEBUGPANIC();
}

/****************************************************************************
 * Name: event_group_set_bits_wrapper
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t event_group_set_bits_wrapper(void *event, uint32_t bits)
{
  DEBUGPANIC();

  return false;
}

/****************************************************************************
 * Name: event_group_clear_bits_wrapper
 *
 * Description:
 *   Don't support
 *
 ****************************************************************************/

static uint32_t event_group_clear_bits_wrapper(void *event, uint32_t bits)
{
  DEBUGPANIC();

  return false;
}

/****************************************************************************
 * Name: esp_int_adpt_cb
 *
 * Description:
 *   This is the callback function for the Wi-Fi interrupt adapter. It
 *   retrieves the adapter from the argument, then calls the function
 *   stored in the adapter with its argument.
 *
 * Input Parameters:
 *   irq     - The IRQ number that caused this interrupt.
 *   context - The register context at the time of the interrupt.
 *   arg     - A pointer to the interrupt adapter's private data.
 *
 * Returned Value:
 *   Always returns 0.
 *
 ****************************************************************************/

static int esp_int_adpt_cb(int irq, void *context, void *arg)
{
  struct irq_adpt *adapter = (struct irq_adpt *)arg;

  adapter->func(adapter->arg);

  return 0;
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
 * Name: esp_nvs_erase_key
 *
 * Description:
 *   Read a block of data from file system
 *
 * Input Parameters:
 *   handle - NVS handle
 *   key    - Data index
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

#ifdef ESP_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_wifi_auth_trans
 *
 * Description:
 *   Converts an ESP32-C3 authenticate mode values to WEXT authenticate mode.
 *
 * Input Parameters:
 *   wifi_auth - ESP32-C3 authenticate mode
 *
 * Returned Value:
 *   authenticate mode
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

      case WIFI_AUTH_WPA3_PSK:
      case WIFI_AUTH_WPA2_WPA3_PSK:
        auth_mode = IW_AUTH_WPA_VERSION_WPA3;
        break;

      default:
        wlerr("Failed to transfer wireless authmode: %ld", wifi_auth);
        break;
    }

  return auth_mode;
}

/****************************************************************************
 * Name: esp_wifi_cipher_trans
 *
 * Description:
 *   Converts a ESP32-C3 cipher type values to WEXT cipher type values.
 *
 * Input Parameters:
 *   wifi_cipher - ESP32-C3 cipher type
 *
 * Returned Value:
 *   cipher type
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
        wlerr("Failed to transfer wireless authmode: %ld",
               wifi_cipher);
        break;
    }

  return cipher_mode;
}

#endif /* ESP_WLAN_HAS_STA */

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
 * Name: queue_msg_waiting_wrapper
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

static uint32_t queue_msg_waiting_wrapper(void *queue)
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
 * Name: task_delay_wrapper
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

static void task_delay_wrapper(uint32_t tick)
{
  useconds_t us = TICK2USEC(tick);

  nxsig_usleep(us);
}

/****************************************************************************
 * Name: task_delete_wrapper
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

static void task_delete_wrapper(void *task_handle)
{
  pid_t pid = (pid_t)((uintptr_t)task_handle);

  kthread_delete(pid);
}

/****************************************************************************
 * Name: task_get_current_task_wrapper
 *
 * Description:
 *   This function gets the current task's PID and returns it as a void
 *   pointer. This is a wrapper around the NuttX function nxsched_gettid.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current task's PID as a void pointer.
 *
 ****************************************************************************/

static void *task_get_current_task_wrapper(void)
{
  pid_t pid = nxsched_gettid();

  return (void *)((uintptr_t)pid);
}

/****************************************************************************
 * Name: vqueue_delete_adapter
 *
 * Description:
 *   This function deletes a queue adapter. It closes the message queue,
 *   unlinks it, and then frees the memory allocated for the queue adapter.
 *
 * Input Parameters:
 *   queue - A pointer to the queue adapter to be deleted.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void vqueue_delete_adapter(void *queue)
{
  struct mq_adpt *mq_adpt = (struct mq_adpt *)queue;

  file_mq_close(&mq_adpt->mq);
  file_mq_unlink(mq_adpt->name);
  kmm_free(mq_adpt);
}

/****************************************************************************
 * Name: vsemaphore_delete_adapter
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

static void vsemaphore_delete_adapter(void *semphr)
{
  sem_t *sem = (sem_t *)semphr;

  nxsem_destroy(sem);
  kmm_free(sem);
}

/****************************************************************************
 * Name: xqueue_create_adapter
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

static void *xqueue_create_adapter(uint32_t queue_len, uint32_t item_size)
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
 * Name: xqueue_send_adapter
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

static int32_t xqueue_send_adapter(void *queue,
                                   void *item,
                                   uint32_t ticks,
                                   int prio)
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

  return nuttx_err_to_freertos(ret);
}

/****************************************************************************
 * Name: xsemaphore_create_counting_adapter
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - No meanining for NuttX
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/

void *xsemaphore_create_counting_adapter(uint32_t max, uint32_t init)
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wifi_malloc
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

IRAM_ATTR void *wifi_malloc(size_t size)
{
  return malloc(size);
}

/****************************************************************************
 * Name: wifi_realloc
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

IRAM_ATTR void *wifi_realloc(void *ptr, size_t size)
{
  return realloc(ptr, size);
}

/****************************************************************************
 * Name: wifi_calloc
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

IRAM_ATTR void *wifi_calloc(size_t n, size_t size)
{
  return calloc(n, size);
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
                  pid = nxsched_gettid();
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
 *   Initialize ESP32-C3 Wi-Fi adapter
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

  wifi_cfg.nvs_enable = 0;

#ifdef CONFIG_ESPRESSIF_WIFI_AMPDU_TX_ENABLED
  wifi_cfg.ampdu_tx_enable = 1;
#else
  wifi_cfg.ampdu_tx_enable = 0;
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_AMPDU_RX_ENABLED
  wifi_cfg.ampdu_rx_enable = 1;
#else
  wifi_cfg.ampdu_rx_enable = 0;
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_STA_DISCONNECT_PM
  wifi_cfg.sta_disconnected_pm = true;
#else
  wifi_cfg.sta_disconnected_pm = false;
#endif

  wifi_cfg.rx_ba_win          = CONFIG_ESPRESSIF_WIFI_TX_BA_WIN;
  wifi_cfg.static_rx_buf_num  = CONFIG_ESPRESSIF_WIFI_STATIC_RX_BUFFER_NUM;
  wifi_cfg.dynamic_rx_buf_num = CONFIG_ESPRESSIF_WIFI_DYNAMIC_RX_BUFFER_NUM;
  wifi_cfg.dynamic_tx_buf_num = CONFIG_ESPRESSIF_WIFI_DYNAMIC_TX_BUFFER_NUM;

  ret = esp_wifi_init(&wifi_cfg);
  if (ret)
    {
      wlerr("Failed to initialize Wi-Fi error=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      goto errout_init_wifi;
    }

  ret = esp_wifi_set_tx_done_cb(esp_wifi_tx_done_cb);
  if (ret)
    {
      wlerr("Failed to register TX done callback ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
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

#ifdef ESP_WLAN_HAS_STA

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

#ifdef ESP_WLAN_HAS_SOFTAP
  if (g_softap_started)
    {
      mode = WIFI_MODE_APSTA;
    }
  else
#endif /* ESP_WLAN_HAS_SOFTAP */
    {
      mode = WIFI_MODE_STA;
    }

  ret = esp_wifi_set_mode(mode);
  if (ret)
    {
      wlerr("Failed to set Wi-Fi mode=%d ret=%d\n", mode, ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  ret = esp_wifi_start();
  if (ret)
    {
      wlerr("Failed to start Wi-Fi with mode=%d ret=%d\n", mode, ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  g_sta_started = true;

  wlinfo("OK to start Wi-Fi station\n");

errout:
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

#ifdef ESP_WLAN_HAS_SOFTAP
  if (g_softap_started)
    {
      ret = esp_wifi_set_mode(WIFI_MODE_AP);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi AP mode ret=%d\n", ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }

      ret = esp_wifi_start();
      if (ret)
        {
          wlerr("Failed to start Wi-Fi AP ret=%d\n", ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }
    }
#endif /* ESP_WLAN_HAS_SOFTAP */

  wlinfo("OK to stop Wi-Fi station\n");

#ifdef ESP_WLAN_HAS_SOFTAP
errout:
#endif /* ESP_WLAN_HAS_SOFTAP */

  esp_wifi_lock(false);
  return ret;
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

int esp_wifi_sta_send_data(void *pbuf, size_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_STA, pbuf, len);

  return esp_wifi_to_errno(ret);
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

  wifi_cfg = g_sta_wifi_cfg;

  if (set)
    {
      len = ext->key_len;
      if (len > PWD_MAX_LEN)
        {
          return -EINVAL;
        }

      memset(wifi_cfg.sta.password, 0x0, PWD_MAX_LEN);

      if (ext->alg != IW_ENCODE_ALG_NONE)
        {
          memcpy(wifi_cfg.sta.password, pdata, len);
        }

      wifi_cfg.sta.pmf_cfg.capable = true;

      if (g_sta_connected)
        {
          ret = esp_wifi_sta_disconnect();
          if (ret)
            {
              wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
              return ret;
            }

          ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }

          ret = esp_wifi_sta_connect();
          if (ret)
            {
              wlerr("Failed to connect to Wi-Fi AP ret=%d\n", ret);
              return ret;
            }
        }

      g_sta_wifi_cfg = wifi_cfg;
    }
  else
    {
      len = iwr->u.encoding.length - sizeof(*ext);
      size = strnlen((char *)wifi_cfg.sta.password, PWD_MAX_LEN);
      if (len < size)
        {
          return -EINVAL;
        }
      else
        {
          ext->key_len = size;
          memcpy(pdata, wifi_cfg.sta.password, ext->key_len);
        }

      if (g_sta_connected)
        {
          wifi_ap_record_t ap_info;

          ret = esp_wifi_sta_get_ap_info(&ap_info);
          if (ret)
            {
              wlerr("Failed to get AP record ret=%d", ret);
              return esp_wifi_to_errno(ret);
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

  wifi_cfg = g_sta_wifi_cfg;

  if (set)
    {
      memset(wifi_cfg.sta.ssid, 0x0, SSID_MAX_LEN);
      memcpy(wifi_cfg.sta.ssid, pdata, len);
      memset(wifi_cfg.sta.sae_h2e_identifier, 0x0, SAE_H2E_IDENTIFIER_LEN);
      wifi_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

      if (g_sta_connected)
        {
          ret = esp_wifi_sta_disconnect();
          if (ret)
            {
              wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
              return ret;
            }

          ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }

          ret = esp_wifi_sta_connect();
          if (ret)
            {
              wlerr("Failed to connect to Wi-Fi AP ret=%d\n", ret);
              return ret;
            }
        }

      g_sta_wifi_cfg = wifi_cfg;
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

  sockaddr = &iwr->u.ap_addr;
  pdata    = sockaddr->sa_data;

  wifi_cfg = g_sta_wifi_cfg;

  if (set)
    {
      wifi_cfg.sta.bssid_set = true;
      memcpy(wifi_cfg.sta.bssid, pdata, MAC_LEN);

      if (g_sta_connected)
        {
          ret = esp_wifi_sta_disconnect();
          if (ret)
            {
              wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
              return ret;
            }

          ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }

          ret = esp_wifi_sta_connect();
          if (ret)
            {
              wlerr("Failed to connect to Wi-Fi AP ret=%d\n", ret);
              return ret;
            }
        }

      g_sta_wifi_cfg = wifi_cfg;
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

  ret = esp_wifi_set_config(WIFI_IF_STA, &g_sta_wifi_cfg);
  if (ret)
    {
      wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
      return esp_wifi_to_errno(ret);
    }

  ret = esp_wifi_connect();
  if (ret)
    {
      wlerr("Failed to connect ret=%d\n", ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  esp_wifi_lock(false);

  ticks = SEC2TICK(WIFI_CONNECT_TIMEOUT);
  do
    {
      if (g_sta_connected)
        {
          break;
        }

      task_delay_wrapper(1);
    }
  while (ticks--);

  if (!g_sta_connected)
    {
      g_sta_reconnect = false;
      wlinfo("Failed to connect to AP\n");
      return -1;
    }

  return OK;

errout:
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
      ret = esp_wifi_to_errno(ret);
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
  wifi_config_t wifi_cfg;
  wifi_ap_record_t ap_info;

  wifi_cfg = g_sta_wifi_cfg;

  if (set)
    {
      cmd = iwr->u.param.flags & IW_AUTH_INDEX;
      switch (cmd)
        {
          case IW_AUTH_WPA_VERSION:
            {
              switch (iwr->u.param.value)
                {
                  case IW_AUTH_WPA_VERSION_DISABLED:
                    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
                    break;

                  case IW_AUTH_WPA_VERSION_WPA:
                    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
                    break;

                  case IW_AUTH_WPA_VERSION_WPA2:
                    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                    break;

                  case IW_AUTH_WPA_VERSION_WPA3:
                    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA3_PSK;
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
                    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
                    break;

                  case IW_AUTH_CIPHER_WEP40:
                  case IW_AUTH_CIPHER_WEP104:
                    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WEP;
                    break;

                  case IW_AUTH_CIPHER_TKIP:
                  case IW_AUTH_CIPHER_CCMP:
                  case IW_AUTH_CIPHER_AES_CMAC:
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

      size_t password_len = strlen((const char *)wifi_cfg.sta.password);
      wifi_auth_mode_t authmode = wifi_cfg.sta.threshold.authmode;

      if (g_sta_connected &&
          ((password_len > 0 && authmode != WIFI_AUTH_OPEN) ||
           (password_len == 0 && authmode == WIFI_AUTH_OPEN)))
        {
          ret = esp_wifi_sta_disconnect();
          if (ret)
            {
              wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
              return ret;
            }

          ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }

          ret = esp_wifi_sta_connect();
          if (ret)
            {
              wlerr("Failed to connect to Wi-Fi AP ret=%d\n", ret);
              return ret;
            }
        }

      g_sta_wifi_cfg = wifi_cfg;
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
          return esp_wifi_to_errno(ret);
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

  if (set && (iwr->u.freq.flags == IW_FREQ_FIXED))
    {
      wifi_config_t wifi_cfg = g_sta_wifi_cfg;

      wifi_cfg.sta.channel = esp_freq_to_channel(iwr->u.freq.m);

      if (g_sta_connected)
        {
          ret = esp_wifi_sta_disconnect();
          if (ret)
            {
              wlerr("Failed to disconnect from Wi-Fi AP ret=%d\n", ret);
              return ret;
            }

          ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }

          ret = esp_wifi_sta_connect();
          if (ret)
            {
              wlerr("Failed to connect to Wi-Fi AP ret=%d\n", ret);
              return ret;
            }
        }

      g_sta_wifi_cfg = wifi_cfg;
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
              return esp_wifi_to_errno(ret);
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
          return esp_wifi_to_errno(ret);
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

#endif /* ESP_WLAN_HAS_STA */

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
          return esp_wifi_to_errno(ret);
        }

      iwr->u.txpower.disabled = 0;
      iwr->u.txpower.flags    = IW_TXPOW_DBM;
      iwr->u.txpower.value    = power / 4;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wifi_sta_channel
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
          return esp_wifi_to_errno(ret);
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
          return esp_wifi_to_errno(ret);
        }
    }
  else
    {
      return -ENOSYS;
    }

  return OK;
}

#ifdef ESP_WLAN_HAS_STA

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
          return esp_wifi_to_errno(ret);
        }

      iwr->u.sens.value = -(ap_info.rssi);
    }

  return OK;
}
#endif /* ESP_WLAN_HAS_STA */

/****************************************************************************
 * SoftAP functions
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP

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

#ifdef ESP_WLAN_HAS_STA
  if (g_sta_started)
    {
      mode = WIFI_MODE_APSTA;
    }
  else
#endif /* ESP_WLAN_HAS_STA */
    {
      mode = WIFI_MODE_AP;
    }

  ret = esp_wifi_set_mode(mode);
  if (ret)
    {
      wlerr("Failed to set Wi-Fi mode=%d ret=%d\n", mode, ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  ret = esp_wifi_start();
  if (ret)
    {
      wlerr("Failed to start Wi-Fi with mode=%d ret=%d\n", mode, ret);
      ret = esp_wifi_to_errno(ret);
      goto errout;
    }

  g_softap_started = true;

  wlinfo("OK to start Wi-Fi SoftAP\n");

errout:
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

#ifdef ESP_WLAN_HAS_STA
  if (g_sta_started)
    {
      ret = esp_wifi_set_mode(WIFI_MODE_STA);
      if (ret)
        {
          wlerr("Failed to set Wi-Fi AP mode ret=%d\n", ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }

      ret = esp_wifi_start();
      if (ret)
        {
          wlerr("Failed to start Wi-Fi STA ret=%d\n", ret);
          ret = esp_wifi_to_errno(ret);
          goto errout;
        }
    }
#endif /* ESP_WLAN_HAS_STA */

  wlinfo("OK to stop Wi-Fi SoftAP\n");

#ifdef ESP_WLAN_HAS_STA
errout:
#endif /* ESP_WLAN_HAS_STA */

  esp_wifi_lock(false);
  return ret;
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

int esp_wifi_softap_send_data(void *pbuf, size_t len)
{
  int ret;

  ret = esp_wifi_internal_tx(WIFI_IF_AP, pbuf, len);

  return esp_wifi_to_errno(ret);
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
  len = ext->key_len;

  if (set && len > PWD_MAX_LEN)
    {
      return -EINVAL;
    }

  pdata = ext->key;
  len   = ext->key_len;

  wifi_cfg = g_softap_wifi_cfg;

  if (set)
    {
      /* Clear the password field and copy the user password to it */

      memset(wifi_cfg.ap.password, 0x0, PWD_MAX_LEN);

      if (ext->alg != IW_ENCODE_ALG_NONE)
        {
          memcpy(wifi_cfg.ap.password, pdata, len);
        }

      if (g_softap_started)
        {
          ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }
        }

      g_softap_wifi_cfg = wifi_cfg;
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

  wifi_cfg = g_softap_wifi_cfg;

  if (set)
    {
      memset(wifi_cfg.ap.ssid, 0x0, SSID_MAX_LEN);
      memcpy(wifi_cfg.ap.ssid, pdata, len);
      wifi_cfg.ap.ssid_len = len;
      if (g_softap_started)
        {
          ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }
        }

      g_softap_wifi_cfg = wifi_cfg;
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
 *   set - true: set data; false: get data
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
  int ret;

  ret = esp_wifi_set_config(WIFI_IF_AP, &g_softap_wifi_cfg);
  if (ret)
    {
      wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
      return esp_wifi_to_errno(ret);
    }

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

  wifi_cfg = g_softap_wifi_cfg;

  if (set)
    {
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

                  case IW_AUTH_WPA_VERSION_WPA3:
                    wifi_cfg.ap.pmf_cfg.required = true;
                    wifi_cfg.ap.pmf_cfg.capable = false;
                    wifi_cfg.ap.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
                    wifi_cfg.ap.authmode = WIFI_AUTH_WPA3_PSK;
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
                  case IW_AUTH_CIPHER_CCMP:
                  case IW_AUTH_CIPHER_AES_CMAC:
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

      size_t password_len = strlen((const char *)wifi_cfg.ap.password);

      if (g_softap_started &&
          ((password_len > 0 && wifi_cfg.ap.authmode != WIFI_AUTH_OPEN) ||
           (password_len == 0 && wifi_cfg.ap.authmode == WIFI_AUTH_OPEN)))
        {
          ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }
        }

      g_softap_wifi_cfg = wifi_cfg;
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

  wifi_cfg = g_softap_wifi_cfg;

  if (set)
    {
      int channel = esp_freq_to_channel(iwr->u.freq.m);

      wifi_cfg.ap.channel = channel;

      if (g_softap_started)
        {
          ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
          if (ret)
            {
              wlerr("Failed to set Wi-Fi config data ret=%d\n", ret);
              return esp_wifi_to_errno(ret);
            }
        }

      g_softap_wifi_cfg = wifi_cfg;
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

#endif /* ESP_WLAN_HAS_SOFTAP */
