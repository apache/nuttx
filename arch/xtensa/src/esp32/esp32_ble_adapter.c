/****************************************************************************
 * arch/xtensa/src/esp32/esp32_ble_adapter.c
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

#include <nuttx/config.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <clock/clock.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/kthread.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <irq/irq.h>

#include "hardware/esp32_dport.h"
#include "hardware/wdev_reg.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "utils/memory_reserve.h"
#include "esp32_rt_timer.h"
#include "esp32_wireless.h"
#include "esp32_irq.h"
#include "esp32_spicache.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_private/phy.h"
#include "esp_private/wifi.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "periph_ctrl.h"
#include "rom/ets_sys.h"
#include "soc/soc_caps.h"
#include "xtensa/core-macros.h"
#include "xtensa/xtensa_api.h"
#include "esp_coexist_internal.h"

#include "esp32_ble_adapter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bluetooth system and controller config */

#define BTDM_CFG_BT_DATA_RELEASE            (1<<0)
#define BTDM_CFG_HCI_UART                   (1<<1)
#define BTDM_CFG_CONTROLLER_RUN_APP_CPU     (1<<2)
#define BTDM_CFG_SCAN_DUPLICATE_OPTIONS     (1<<3)
#define BTDM_CFG_SEND_ADV_RESERVED_SIZE     (1<<4)
#define BTDM_CFG_BLE_FULL_SCAN_SUPPORTED    (1<<5)

/* Sleep mode */

#define BTDM_MODEM_SLEEP_MODE_NONE          (0)
#define BTDM_MODEM_SLEEP_MODE_ORIG          (1)
#define BTDM_MODEM_SLEEP_MODE_EVED          (2)  /* sleep mode for BLE controller, used only for internal test. */

/* Low Power Clock Selection */

#define BTDM_LPCLK_SEL_XTAL                 (0)
#define BTDM_LPCLK_SEL_XTAL32K              (1)
#define BTDM_LPCLK_SEL_RTC_SLOW             (2)
#define BTDM_LPCLK_SEL_8M                   (3)

/* Sleep and wakeup interval control */

#define BTDM_MIN_SLEEP_DURATION             (12) /* threshold of interval in slots to allow to fall into modem sleep */
#define BTDM_MODEM_WAKE_UP_DELAY            (4)  /* delay in slots of modem wake up procedure, including re-enable PHY/RF */

#define OSI_FUNCS_TIME_BLOCKING             0xffffffff
#define OSI_VERSION                         0x00010004
#define OSI_MAGIC_VALUE                     0xfadebead

#ifdef CONFIG_PM
#  define BTDM_MIN_TIMER_UNCERTAINTY_US     (500)
#endif

#define BTDM_ASYNC_WAKEUP_REQ_HCI           0
#define BTDM_ASYNC_WAKEUP_REQ_COEX          1
#define BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA     2
#define BTDM_ASYNC_WAKEUP_REQMAX            3

#define MSG_QUEUE_NAME_SIZE                 16

#ifdef CONFIG_ESP32_SPIFLASH
#  define BLE_TASK_EVENT_QUEUE_ITEM_SIZE    8
#  define BLE_TASK_EVENT_QUEUE_LEN          8
#endif

#ifdef CONFIG_ESP32_BLE_INTERRUPT_SAVE_STATUS
#  define NR_IRQSTATE_FLAGS   CONFIG_ESP32_BLE_INTERRUPT_SAVE_STATUS
#else
#  define NR_IRQSTATE_FLAGS   3
#endif

#define RTC_CLK_CAL_FRACT  19  /* Number of fractional bits in values returned by rtc_clk_cal */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* VHCI function interface */

typedef struct vhci_host_callback_s
{
  void (*notify_host_send_available)(void);               /* callback used to notify that the host can send packet to controller */
  int (*notify_host_recv)(uint8_t *data, uint16_t len);   /* callback used to notify that the controller has a packet to send to the host */
} vhci_host_callback_t;

/* Dram region */

typedef struct
{
  esp_bt_mode_t mode;
  intptr_t start;
  intptr_t end;
} btdm_dram_available_region_t;

struct osi_funcs_s
{
  uint32_t _version;
  xt_handler (*_set_isr)(int n, xt_handler f, void *arg);
  void (*_ints_on)(unsigned int mask);
  void (*_interrupt_disable)(void);
  void (*_interrupt_restore)(void);
  void (*_task_yield)(void);
  void (*_task_yield_from_isr)(void);
  void *(*_semphr_create)(uint32_t max, uint32_t init);
  void (*_semphr_delete)(void *semphr);
  int32_t (*_semphr_take_from_isr)(void *semphr, void *hptw);
  int32_t (*_semphr_give_from_isr)(void *semphr, void *hptw);
  int32_t (*_semphr_take)(void *semphr, uint32_t block_time_ms);
  int32_t (*_semphr_give)(void *semphr);
  void *(*_mutex_create)(void);
  void (*_mutex_delete)(void *mutex);
  int32_t (*_mutex_lock)(void *mutex);
  int32_t (*_mutex_unlock)(void *mutex);
  void *(* _queue_create)(uint32_t queue_len, uint32_t item_size);
  void (* _queue_delete)(void *queue);
  int32_t (* _queue_send)(void *queue, void *item, uint32_t block_time_ms);
  int32_t (* _queue_send_from_isr)(void *queue, void *item, void *hptw);
  int32_t (* _queue_recv)(void *queue, void *item, uint32_t block_time_ms);
  int32_t (* _queue_recv_from_isr)(void *queue, void *item, void *hptw);
  int32_t (* _task_create)(void *task_func,
                           const char *name,
                           uint32_t stack_depth,
                           void *param,
                           uint32_t prio,
                           void *task_handle,
                           uint32_t core_id);
  void (* _task_delete)(void *task_handle);
  bool (* _is_in_isr)(void);
  int (* _cause_sw_intr_to_core)(int core_id, int intr_no);
  void *(* _malloc)(size_t size);
  void *(* _malloc_internal)(size_t size);
  void (* _free)(void *p);
  int32_t (* _read_efuse_mac)(uint8_t mac[6]);
  void (* _srand)(unsigned int seed);
  int (* _rand)(void);
  uint32_t (* _btdm_lpcycles_2_us)(uint32_t cycles);
  uint32_t (* _btdm_us_2_lpcycles)(uint32_t us);
  bool (* _btdm_sleep_check_duration)(uint32_t *slot_cnt);
  void (* _btdm_sleep_enter_phase1)(uint32_t lpcycles);  /* called when interrupt is disabled */
  void (* _btdm_sleep_enter_phase2)(void);
  void (* _btdm_sleep_exit_phase1)(void);  /* called from ISR */
  void (* _btdm_sleep_exit_phase2)(void);  /* called from ISR */
  void (* _btdm_sleep_exit_phase3)(void);  /* called from task */
  bool (* _coex_bt_wakeup_request)(void);
  void (* _coex_bt_wakeup_request_end)(void);
  int (* _coex_bt_request)(uint32_t event,
                           uint32_t latency,
                           uint32_t duration);
  int (* _coex_bt_release)(uint32_t event);
  int (* _coex_register_bt_cb)(coex_func_cb_t cb);
  uint32_t (* _coex_bb_reset_lock)(void);
  void (* _coex_bb_reset_unlock)(uint32_t restore);
  int (* _coex_schm_register_btdm_callback)(void *callback);
  void (* _coex_schm_status_bit_clear)(uint32_t type, uint32_t status);
  void (* _coex_schm_status_bit_set)(uint32_t type, uint32_t status);
  uint32_t (* _coex_schm_interval_get)(void);
  uint8_t (* _coex_schm_curr_period_get)(void);
  void *(* _coex_schm_curr_phase_get)(void);
  int (* _coex_wifi_channel_get)(uint8_t *primary, uint8_t *secondary);
  int (* _coex_register_wifi_channel_change_callback)(void *cb);
  xt_handler (*_set_isr_l3)(int n, xt_handler f, void *arg);
  void (*_interrupt_l3_disable)(void);
  void (*_interrupt_l3_restore)(void);
  void *(* _customer_queue_create)(uint32_t queue_len, uint32_t item_size);
  int (* _coex_version_get)(unsigned int *major,
                            unsigned int *minor,
                            unsigned int *patch);
  uint32_t _magic;
};

typedef void (*workitem_handler_t)(void *arg);

/* BLE message queue private data */

struct mq_adpt_s
{
  struct file mq;                        /* Message queue handle */
  uint32_t    msgsize;                   /* Message size */
  char        name[MSG_QUEUE_NAME_SIZE]; /* Message queue name */
};

/* BLE interrupt adapter private data */

struct irq_adpt_s
{
  void (*func)(void *arg);  /* Interrupt callback function */
  void *arg;                /* Interrupt private data */
};

/* vendor dependent signals to be posted to controller task */

typedef enum
{
  BTDM_VND_OL_SIG_WAKEUP_TMR,
  BTDM_VND_OL_SIG_NUM,
} btdm_vnd_ol_sig_t;

#ifdef CONFIG_PM
/* wakeup request sources */

typedef enum
{
  BTDM_ASYNC_WAKEUP_SRC_VHCI,
  BTDM_ASYNC_WAKEUP_SRC_DISA,
  BTDM_ASYNC_WAKEUP_SRC_TMR,
  BTDM_ASYNC_WAKEUP_SRC_MAX,
} btdm_wakeup_src_t;
#endif

/* Superseded semaphore definition */

struct bt_sem_s
{
  sem_t sem;
#ifdef CONFIG_ESP32_SPIFLASH
  struct esp_semcache_s sc;
#endif
};

/* prototype of function to handle vendor dependent signals */

typedef void (*btdm_vnd_ol_task_func_t)(void *param);

typedef void (*osi_intr_handler)(void);

/* List of nested IRQ status flags */

struct irqstate_list_s
{
  struct irqstate_list_s *flink;
  irqstate_t flags;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Functions to be registered to struct osi_funcs_s
 ****************************************************************************/

/* Note: Functions name prefixed with `esp_` usually refers to the common
 * source code shared between different devices. Avoid creating functions
 * with the same prefix. Adding a different prefix, however, would differ
 * this source from other devices. So, it's recommended to not use any kind
 * of prefix to refer to the SoC.
 */

static xt_handler ble_set_isr(int n, xt_handler f, void *arg);
static void ints_on(uint32_t mask);
static void IRAM_ATTR interrupt_disable(void);
static void IRAM_ATTR interrupt_restore(void);
static void IRAM_ATTR task_yield_from_isr(void);
static void *semphr_create_wrapper(uint32_t max, uint32_t init);
static void semphr_delete_wrapper(void *semphr);
static int IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw);
static int IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw);
static int semphr_take_wrapper(void *semphr, uint32_t block_time_ms);
static int semphr_give_wrapper(void *semphr);
static void *mutex_create_wrapper(void);
static void mutex_delete_wrapper(void *mutex);
static int mutex_lock_wrapper(void *mutex);
static int mutex_unlock_wrapper(void *mutex);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static void queue_delete_wrapper(void *queue);
static int queue_send_wrapper(void *queue,
                              void *item,
                              uint32_t block_time_ms);
static int IRAM_ATTR queue_send_from_isr_wrapper(void *queue,
                                                 void *item,
                                                 void *hptw);
static int queue_recv_wrapper(void *queue,
                              void *item,
                              uint32_t block_time_ms);
static int IRAM_ATTR queue_recv_from_isr_wrapper(void *queue,
                                                 void *item,
                                                 void *hptw);
static int task_create_wrapper(void *task_func,
                               const char *name,
                               uint32_t stack_depth,
                               void *param,
                               uint32_t prio,
                               void *task_handle,
                               uint32_t core_id);
static void task_delete_wrapper(void *task_handle);
static bool IRAM_ATTR is_in_isr_wrapper(void);
static int IRAM_ATTR cause_sw_intr_to_core_wrapper(int core_id, int intr_no);
static void *malloc_wrapper(size_t size);
static void *malloc_internal_wrapper(size_t size);
static int IRAM_ATTR read_mac_wrapper(uint8_t mac[6]);
static void IRAM_ATTR srand_wrapper(unsigned int seed);
static int IRAM_ATTR rand_wrapper(void);
static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles);
static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us);
static bool IRAM_ATTR btdm_sleep_check_duration(uint32_t *slot_cnt);
static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles);
static void btdm_sleep_enter_phase2_wrapper(void);
static void btdm_sleep_exit_phase3_wrapper(void);
static bool coex_bt_wakeup_request(void);
static void coex_bt_wakeup_request_end(void);
static int coex_bt_request_wrapper(uint32_t event,
                                   uint32_t latency,
                                   uint32_t duration);
static int coex_bt_release_wrapper(uint32_t event);
static int adapter_coex_register_bt_cb_wrapper(coex_func_cb_t cb);
static uint32_t coex_bb_reset_lock_wrapper(void);
static void coex_bb_reset_unlock_wrapper(uint32_t restore);
static int coex_schm_register_btdm_callback_wrapper(void *callback);
static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                               uint32_t status);
static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status);
static uint32_t coex_schm_interval_get_wrapper(void);
static uint8_t coex_schm_curr_period_get_wrapper(void);
static void *coex_schm_curr_phase_get_wrapper(void);
static int coex_wifi_channel_get_wrapper(uint8_t *primary,
                                         uint8_t *secondary);
static int coex_register_wifi_channel_change_callback_wrapper(void *cb);
static int coex_version_get_wrapper(unsigned int *major,
                                    unsigned int *minor,
                                    unsigned int *patch);

/****************************************************************************
 * Other functions
 ****************************************************************************/

static int32_t esp_task_create_pinned_to_core(void *entry,
                                              const char *name,
                                              uint32_t stack_depth,
                                              void *param,
                                              uint32_t prio,
                                              void *task_handle,
                                              uint32_t core_id);
static IRAM_ATTR int32_t esp_queue_send_generic(void *queue,
                                                void *item,
                                                uint32_t ticks,
                                                int prio);
static void esp_update_time(struct timespec *timespec, uint32_t ticks);
static void IRAM_ATTR cause_sw_intr(void *arg);
#ifdef CONFIG_PM
static void btdm_slp_tmr_customer_callback(void * arg);
static void IRAM_ATTR btdm_slp_tmr_callback(void *arg);
#endif
static int IRAM_ATTR esp_int_adpt_cb(int irq, void *context, void *arg);
static void btdm_wakeup_request_callback(void * arg);
static void btdm_controller_mem_init(void);
static uint32_t btdm_config_mask_load(void);
static void bt_controller_deinit_internal(void);
static bool async_wakeup_request(int event);
static void async_wakeup_request_end(int event);

/****************************************************************************
 * Extern Functions declaration and value
 ****************************************************************************/

/* Not for user call, so don't put to include file */

/* OSI */

extern int btdm_osi_funcs_register(void *osi_funcs);

/* Initialise and De-initialise */

extern int btdm_controller_init(uint32_t config_mask,
                                esp_bt_controller_config_t *config_opts);
extern void btdm_controller_deinit(void);
extern int btdm_controller_enable(esp_bt_mode_t mode);
extern void btdm_controller_disable(void);
extern uint8_t btdm_controller_get_mode(void);
extern const char *btdm_controller_get_compile_version(void);
extern void btdm_rf_bb_init_phase2(void); /* shall be called after PHY/RF is enabled */
extern int btdm_dispatch_work_to_controller(workitem_handler_t callback,
                                            void *arg,
                                            bool blocking);

/* Sleep */

extern void btdm_controller_enable_sleep(bool enable);
extern void btdm_controller_set_sleep_mode(uint8_t mode);
extern uint8_t btdm_controller_get_sleep_mode(void);
extern bool btdm_power_state_active(void);
extern void btdm_wakeup_request(void);
extern void btdm_in_wakeup_requesting_set(bool in_wakeup_requesting);

/* Low Power Clock */

extern bool btdm_lpclk_select_src(uint32_t sel);
extern bool btdm_lpclk_set_div(uint32_t div);

/* VHCI */

extern bool api_vhci_host_check_send_available(void); /* Functions in bt lib */
extern void api_vhci_host_send_packet(uint8_t * data, uint16_t len);
extern int api_vhci_host_register_callback(const vhci_host_callback_t
                                           *callback);

/* TX power */

extern int ble_txpwr_set(int power_type, int power_level);
extern int ble_txpwr_get(int power_type);
extern int bredr_txpwr_set(int min_power_level, int max_power_level);
extern int bredr_txpwr_get(int *min_power_level, int *max_power_level);
extern void bredr_sco_datapath_set(uint8_t data_path);
extern void btdm_controller_scan_duplicate_list_clear(void);

/* Shutdown */

extern void esp_bt_controller_shutdown(void);

extern uint8_t _bss_start_btdm[];
extern uint8_t _bss_end_btdm[];
extern uint8_t _data_start_btdm[];
extern uint8_t _data_end_btdm[];
extern uint32_t _data_start_btdm_rom;
extern uint32_t _data_end_btdm_rom;

extern uint32_t _bt_bss_start;
extern uint32_t _bt_bss_end;
extern uint32_t _nimble_bss_start;
extern uint32_t _nimble_bss_end;
extern uint32_t _btdm_bss_start;
extern uint32_t _btdm_bss_end;
extern uint32_t _bt_data_start;
extern uint32_t _bt_data_end;
extern uint32_t _nimble_data_start;
extern uint32_t _nimble_data_end;
extern uint32_t _btdm_data_start;
extern uint32_t _btdm_data_end;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* OSI funcs */

static struct osi_funcs_s g_osi_funcs_ro =
{
  ._version = OSI_VERSION,
  ._set_isr = ble_set_isr,
  ._ints_on = ints_on,
  ._interrupt_disable = interrupt_disable,
  ._interrupt_restore = interrupt_restore,
  ._task_yield = task_yield_from_isr,
  ._task_yield_from_isr = task_yield_from_isr,
  ._semphr_create = semphr_create_wrapper,
  ._semphr_delete = semphr_delete_wrapper,
  ._semphr_take_from_isr = semphr_take_from_isr_wrapper,
  ._semphr_give_from_isr = semphr_give_from_isr_wrapper,
  ._semphr_take = semphr_take_wrapper,
  ._semphr_give = semphr_give_wrapper,
  ._mutex_create = mutex_create_wrapper,
  ._mutex_delete = mutex_delete_wrapper,
  ._mutex_lock = mutex_lock_wrapper,
  ._mutex_unlock = mutex_unlock_wrapper,
  ._queue_create = queue_create_wrapper,
  ._queue_delete = queue_delete_wrapper,
  ._queue_send = queue_send_wrapper,
  ._queue_send_from_isr = queue_send_from_isr_wrapper,
  ._queue_recv = queue_recv_wrapper,
  ._queue_recv_from_isr = queue_recv_from_isr_wrapper,
  ._task_create = task_create_wrapper,
  ._task_delete = task_delete_wrapper,
  ._is_in_isr = is_in_isr_wrapper,
  ._cause_sw_intr_to_core = cause_sw_intr_to_core_wrapper,
  ._malloc = malloc_wrapper,
  ._malloc_internal = malloc_internal_wrapper,
  ._free = free,
  ._read_efuse_mac = read_mac_wrapper,
  ._srand = srand_wrapper,
  ._rand = rand_wrapper,
  ._btdm_lpcycles_2_us = btdm_lpcycles_2_us,
  ._btdm_us_2_lpcycles = btdm_us_2_lpcycles,
  ._btdm_sleep_check_duration = btdm_sleep_check_duration,
  ._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
  ._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
  ._btdm_sleep_exit_phase1 = NULL,
  ._btdm_sleep_exit_phase2 = NULL,
  ._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
  ._coex_bt_wakeup_request = coex_bt_wakeup_request,
  ._coex_bt_wakeup_request_end = coex_bt_wakeup_request_end,
  ._coex_bt_request = coex_bt_request_wrapper,
  ._coex_bt_release = coex_bt_release_wrapper,
  ._coex_register_bt_cb = adapter_coex_register_bt_cb_wrapper,
  ._coex_bb_reset_lock = coex_bb_reset_lock_wrapper,
  ._coex_bb_reset_unlock = coex_bb_reset_unlock_wrapper,
  ._coex_schm_register_btdm_callback =
      coex_schm_register_btdm_callback_wrapper,
  ._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
  ._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
  ._coex_schm_interval_get = coex_schm_interval_get_wrapper,
  ._coex_schm_curr_period_get = coex_schm_curr_period_get_wrapper,
  ._coex_schm_curr_phase_get = coex_schm_curr_phase_get_wrapper,
  ._coex_wifi_channel_get = coex_wifi_channel_get_wrapper,
  ._coex_register_wifi_channel_change_callback =
                 coex_register_wifi_channel_change_callback_wrapper,
  ._set_isr_l3 = ble_set_isr,
  ._interrupt_l3_disable = interrupt_disable,
  ._interrupt_l3_restore = interrupt_restore,
  ._customer_queue_create = NULL,
  ._coex_version_get = coex_version_get_wrapper,
  ._magic = OSI_MAGIC_VALUE,
};

/* The mode column will be modified by release function to indicate
 * the available region.
 */

static btdm_dram_available_region_t g_btdm_dram_available_region[] =
{
  /* The following is .data */

  {
    ESP_BT_MODE_BTDM,
    SOC_MEM_BT_DATA_START,
    SOC_MEM_BT_DATA_END
  },

  /* The following is memory which HW will use */

  {
    ESP_BT_MODE_BTDM,
    SOC_MEM_BT_EM_BTDM0_START,
    SOC_MEM_BT_EM_BTDM0_END
  },
  {
    ESP_BT_MODE_BLE,
    SOC_MEM_BT_EM_BLE_START,
    SOC_MEM_BT_EM_BLE_END
  },
  {
    ESP_BT_MODE_BTDM,
    SOC_MEM_BT_EM_BTDM1_START,
    SOC_MEM_BT_EM_BTDM1_END
  },
  {
    ESP_BT_MODE_CLASSIC_BT,
    SOC_MEM_BT_EM_BREDR_START,
    SOC_MEM_BT_EM_BREDR_REAL_END
  },

  /* The following is .bss */

  {
    ESP_BT_MODE_BTDM,
    SOC_MEM_BT_BSS_START,
    SOC_MEM_BT_BSS_END
  },
  {
    ESP_BT_MODE_BTDM,
    SOC_MEM_BT_MISC_START,
    SOC_MEM_BT_MISC_END
  },
};

/* Reserve the full memory region used by Bluetooth Controller.
 * Some may be released later at runtime.
 */

SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_EM_START,
                          SOC_MEM_BT_EM_BREDR_REAL_END,
                          rom_bt_em);
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_BSS_START,
                          SOC_MEM_BT_BSS_END,
                          rom_bt_bss);
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_MISC_START,
                          SOC_MEM_BT_MISC_END,
                          rom_bt_misc);
SOC_RESERVE_MEMORY_REGION(SOC_MEM_BT_DATA_START,
                          SOC_MEM_BT_DATA_END,
                          rom_bt_data);

static DRAM_ATTR struct osi_funcs_s *g_osi_funcs_p;

/* timestamp when PHY/RF was switched on */

static DRAM_ATTR int64_t g_time_phy_rf_just_enabled = 0;

static DRAM_ATTR esp_bt_controller_status_t g_btdm_controller_status =
    ESP_BT_CONTROLLER_STATUS_IDLE;

/* measured average low power clock period in micro seconds */

static DRAM_ATTR uint32_t g_btdm_lpcycle_us = 0;

/* number of fractional bit for g_btdm_lpcycle_us */

static DRAM_ATTR uint8_t g_btdm_lpcycle_us_frac = 0;

#ifdef CONFIG_BTDM_CTRL_MODEM_SLEEP_MODE_ORIG

/* used low power clock */

static DRAM_ATTR uint8_t g_btdm_lpclk_sel;

#endif /* CONFIG_BTDM_CTRL_MODEM_SLEEP_MODE_ORIG */

/* semaphore used for blocking VHCI API to wait for controller to wake up */

static DRAM_ATTR struct bt_sem_s * g_wakeup_req_sem = NULL;

#ifdef CONFIG_PM

/* wakeup timer */

static DRAM_ATTR esp_timer_handle_t g_btdm_slp_tmr;

static bool g_pm_lock_acquired = true;

static DRAM_ATTR bool g_btdm_allow_light_sleep;

#endif

/* BT interrupt private data */

static sq_queue_t g_ble_int_flags_free;

static sq_queue_t g_ble_int_flags_used;

static struct irqstate_list_s g_ble_int_flags[NR_IRQSTATE_FLAGS];

/* Cached queue control variables */

#ifdef CONFIG_ESP32_SPIFLASH
static struct esp_queuecache_s g_esp_queuecache[BLE_TASK_EVENT_QUEUE_LEN];
static uint8_t g_esp_queuecache_buffer[BLE_TASK_EVENT_QUEUE_ITEM_SIZE];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_errno_trans
 *
 * Description:
 *   Transform NuttX error code to a boolean value. Returns true if the
 *   input error code is 0 (no error), and false otherwise.
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   Boolean value indicating the absence (true) or presence (false) of an
 *   error
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
 * Name: esp_bt_power_domain_on
 *
 * Description:
 *   Power up the Bluetooth module. This function is a wrapper for the
 *   esp_wifi_bt_power_domain_on function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp_bt_power_domain_on(void)
{
  esp_wifi_bt_power_domain_on();
}

/****************************************************************************
 * Name: esp_bt_power_domain_off
 *
 * Description:
 *   Power down the Bluetooth module. This function is a wrapper for the
 *   esp_wifi_bt_power_domain_off function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp_bt_power_domain_off(void)
{
  esp_wifi_bt_power_domain_off();
}

/****************************************************************************
 * Name: btdm_check_and_init_bb
 *
 * Description:
 *   Check and initialize the Bluetooth baseband (BB). If the PHY/RF has
 *   been switched off since the last Bluetooth baseband initialization, it
 *   re-initializes the baseband.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void btdm_check_and_init_bb(void)
{
  int64_t latest_ts = esp_phy_rf_get_on_ts();

  if (latest_ts != g_time_phy_rf_just_enabled ||
      g_time_phy_rf_just_enabled == 0)
    {
      btdm_rf_bb_init_phase2();
      g_time_phy_rf_just_enabled = latest_ts;
    }
}

/****************************************************************************
 * Functions to be registered to struct osi_funcs_s
 ****************************************************************************/

/****************************************************************************
 * Name: ble_set_isr
 *
 * Description:
 *   Register interrupt function
 *
 * Input Parameters:
 *   n   - CPU Interrupt ID
 *   f   - Interrupt function
 *   arg - Function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static xt_handler ble_set_isr(int n, xt_handler f, void *arg)
{
  int ret;
  uint32_t tmp;
  struct irq_adpt_s *adapter;
  int irq = esp32_getirq(0, n);

  wlinfo("n=%d f=%p arg=%p irq=%d\n", n, f, arg, irq);

  if (g_irqvector[irq].handler &&
      g_irqvector[irq].handler != irq_unexpected_isr)
    {
      wlinfo("irq=%d has been set handler=%p\n", irq,
             g_irqvector[irq].handler);
      return NULL;
    }

  tmp = sizeof(struct irq_adpt_s);
  adapter = kmm_malloc(tmp);
  if (!adapter)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      DEBUGPANIC();
      return NULL;
    }

  adapter->func = f;
  adapter->arg = arg;

  ret = irq_attach(irq, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("Failed to attach IRQ %d\n", irq);
      DEBUGPANIC();
      return NULL;
    }

  return NULL;
}

/****************************************************************************
 * Name: ints_on
 *
 * Description:
 *   Enable BLE interrupt
 *
 * Input Parameters:
 *   mask - Mask used to indicate the bits to enable interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ints_on(uint32_t mask)
{
  uint32_t bit;
  int irq;

  for (int i = 0; i < 32; i++)
    {
      bit = 1 << i;
      if (bit & mask)
        {
          irq = esp32_getirq(0, i);
          DEBUGVERIFY(esp32_irq_set_iram_isr(irq));
          up_enable_irq(irq);
          wlinfo("Enabled bit %d\n", irq);
        }
    }

  UNUSED(irq);
}

/****************************************************************************
 * Name: interrupt_disable
 *
 * Description:
 *   Enter critical section by disabling interrupts and taking the spin lock
 *   if in SMP mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR interrupt_disable(void)
{
  struct irqstate_list_s *irqstate;

  irqstate = (struct irqstate_list_s *)sq_remlast(&g_ble_int_flags_free);

  ASSERT(irqstate != NULL);

  irqstate->flags = enter_critical_section();

  sq_addlast((sq_entry_t *)irqstate, &g_ble_int_flags_used);
}

/****************************************************************************
 * Name: interrupt_restore
 *
 * Description:
 *   Exit from critical section by enabling interrupts and releasing the spin
 *   lock if in SMP mode.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR interrupt_restore(void)
{
  struct irqstate_list_s *irqstate;

  irqstate = (struct irqstate_list_s *)sq_remlast(&g_ble_int_flags_used);

  ASSERT(irqstate != NULL);

  leave_critical_section(irqstate->flags);

  sq_addlast((sq_entry_t *)irqstate, &g_ble_int_flags_free);
}

/****************************************************************************
 * Name: task_yield_from_isr
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

static void IRAM_ATTR task_yield_from_isr(void)
{
}

/****************************************************************************
 * Name: semphr_create_wrapper
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - Unused
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/

static void *semphr_create_wrapper(uint32_t max, uint32_t init)
{
  int ret;
  struct bt_sem_s *bt_sem;
  int tmp;

  tmp = sizeof(struct bt_sem_s);
  bt_sem = kmm_malloc(tmp);
  DEBUGASSERT(bt_sem);
  if (!bt_sem)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = nxsem_init(&bt_sem->sem, 0, init);
  DEBUGASSERT(ret == OK);
  if (ret)
    {
      wlerr("ERROR: Failed to initialize sem error=%d\n", ret);
      kmm_free(bt_sem);
      return NULL;
    }

#ifdef CONFIG_ESP32_SPIFLASH
  esp_init_semcache(&bt_sem->sc, &bt_sem->sem);
#endif

  return bt_sem;
}

/****************************************************************************
 * Name: semphr_delete_wrapper
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

static void semphr_delete_wrapper(void *semphr)
{
  struct bt_sem_s *bt_sem = (struct bt_sem_s *)semphr;
  sem_destroy(&bt_sem->sem);
  kmm_free(bt_sem);
}

/****************************************************************************
 * Name: semphr_take_from_isr_wrapper
 *
 * Description:
 *   Take a semaphore from an ISR
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer.
 *   hptw     - Unused.
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
  *(int *)hptw = 0;

  DEBUGPANIC();
  return 0;
}

/****************************************************************************
 * Name: semphr_give_from_isr_wrapper
 *
 * Description:
 *   Post semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *   hptw   - Unused.
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
  int ret;
  struct bt_sem_s *bt_sem = (struct bt_sem_s *)semphr;

#ifdef CONFIG_ESP32_SPIFLASH
  if (spi_flash_cache_enabled())
    {
      ret = semphr_give_wrapper(bt_sem);
    }
  else
    {
      esp_post_semcache(&bt_sem->sc);
      ret = 0;
    }
#else
  ret = semphr_give_wrapper(bt_sem);
#endif

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: semphr_take_wrapper
 *
 * Description:
 *   Wait semaphore within a certain period of time
 *
 * Input Parameters:
 *   semphr         - Semaphore data pointer
 *   block_time_ms  - Wait time
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int semphr_take_wrapper(void *semphr, uint32_t block_time_ms)
{
  int ret;
  struct bt_sem_s *bt_sem = (struct bt_sem_s *)semphr;

  if (block_time_ms == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = nxsem_wait(&bt_sem->sem);
    }
  else
    {
      if (block_time_ms > 0)
        {
          ret = nxsem_tickwait(&bt_sem->sem, MSEC2TICK(block_time_ms));
        }
      else
        {
          ret = nxsem_trywait(&bt_sem->sem);
        }
    }

  if (ret)
    {
      wlerr("ERROR: Failed to wait sem in %u ticks. Error=%d\n",
            MSEC2TICK(block_time_ms), ret);
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: semphr_give_wrapper
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

static int semphr_give_wrapper(void *semphr)
{
  int ret;
  struct bt_sem_s *bt_sem = (struct bt_sem_s *)semphr;

  ret = nxsem_post(&bt_sem->sem);
  if (ret)
    {
      wlerr("Failed to post sem error=%d\n", ret);
    }

  return esp_errno_trans(ret);
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
  DEBUGASSERT(mutex);

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
 *   None
 *
 * Returned Value:
 *   Mutex data pointer
 *
 ****************************************************************************/

static void mutex_delete_wrapper(void *mutex)
{
  pthread_mutex_destroy(mutex);
  kmm_free(mutex);
}

/****************************************************************************
 * Name: mutex_lock_wrapper
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

static int mutex_lock_wrapper(void *mutex)
{
  int ret;

  ret = pthread_mutex_lock(mutex);
  if (ret)
    {
      wlerr("Failed to lock mutex error=%d\n", ret);
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: mutex_unlock_wrapper
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

static int mutex_unlock_wrapper(void *mutex)
{
  int ret;

  ret = pthread_mutex_unlock(mutex);
  if (ret)
    {
      wlerr("Failed to unlock mutex error=%d\n", ret);
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: queue_create_wrapper
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

static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
  struct mq_attr attr;
  struct mq_adpt_s *mq_adpt;
  int ret;

  mq_adpt = kmm_malloc(sizeof(struct mq_adpt_s));
  DEBUGASSERT(mq_adpt);

  snprintf(mq_adpt->name, sizeof(mq_adpt->name), "/tmp/%p", mq_adpt);

  attr.mq_maxmsg  = queue_len;
  attr.mq_msgsize = item_size;
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&mq_adpt->mq, mq_adpt->name,
                     O_RDWR | O_CREAT, 0644, &attr);

  if (ret < 0)
    {
      wlerr("Failed to create mqueue %d\n", ret);
      kmm_free(mq_adpt);
      return NULL;
    }

  mq_adpt->msgsize = item_size;

#ifdef CONFIG_ESP32_SPIFLASH
  if (queue_len <= BLE_TASK_EVENT_QUEUE_LEN &&
      item_size == BLE_TASK_EVENT_QUEUE_ITEM_SIZE)
    {
      esp_init_queuecache(g_esp_queuecache,
                          &mq_adpt->mq,
                          g_esp_queuecache_buffer,
                          BLE_TASK_EVENT_QUEUE_LEN,
                          BLE_TASK_EVENT_QUEUE_ITEM_SIZE);
    }
  else
    {
      wlerr("Failed to create queue cache."
            " Please incresase BLE_TASK_EVENT_QUEUE_LEN to, at least, %d",
            queue_len);
      return NULL;
    }
#endif

  return (void *)mq_adpt;
}

/****************************************************************************
 * Name: queue_delete_wrapper
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

static void queue_delete_wrapper(void *queue)
{
  struct mq_adpt_s *mq_adpt = (struct mq_adpt_s *)queue;

  file_mq_close(&mq_adpt->mq);
  file_mq_unlink(mq_adpt->name);
  kmm_free(mq_adpt);
}

/****************************************************************************
 * Name: queue_send_wrapper
 *
 * Description:
 *   Generic send message to queue within a certain period of time
 *
 * Input Parameters:
 *   queue         - Message queue data pointer
 *   item          - Item to be sent.
 *   block_time_ms - Wait time
 *
 * Returned Value:uint32_t
 *   True if success or false if fail
 *
 ****************************************************************************/

static int queue_send_wrapper(void *queue, void *item,
                              uint32_t block_time_ms)
{
  return esp_queue_send_generic(queue, item, block_time_ms, 0);
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
 *   hptw  - Unused
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int IRAM_ATTR queue_send_from_isr_wrapper(void *queue,
                                                 void *item,
                                                 void *hptw)
{
  *((int *)hptw) = false;
  return esp_queue_send_generic(queue, item, 0, 0);
}

/****************************************************************************
 * Name: queue_recv_wrapper
 *
 * Description:
 *   Receive message from queue within a certain period of time
 *
 * Input Parameters:
 *   queue         - Message queue data pointer
 *   item          - Message data pointer
 *   block_time_ms - Wait time
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int queue_recv_wrapper(void *queue, void *item,
                              uint32_t block_time_ms)
{
  ssize_t ret;
  struct timespec timeout;
  unsigned int prio;
  struct mq_adpt_s *mq_adpt = (struct mq_adpt_s *)queue;

  if (block_time_ms == OSI_FUNCS_TIME_BLOCKING)
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
          wlerr("Failed to get time %d\n", ret);
          return false;
        }

      if (block_time_ms)
        {
          esp_update_time(&timeout, MSEC2TICK(block_time_ms));
        }

      ret = file_mq_timedreceive(&mq_adpt->mq, (char *)item,
                                 mq_adpt->msgsize, &prio, &timeout);

      if (ret < 0)
        {
          wlerr("Failed to timedreceive from mqueue error=%d\n", ret);
        }
    }

  return ret > 0 ? true : false;
}

/****************************************************************************
 * Name: queue_recv_from_isr_wrapper
 *
 * Description:
 *   Receive message from queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   hptw  - Unused
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int IRAM_ATTR queue_recv_from_isr_wrapper(void *queue,
                                                 void *item,
                                                 void *hptw)
{
  DEBUGPANIC();
  return 0;
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

static int task_create_wrapper(void *task_func, const char *name,
                               uint32_t stack_depth, void *param,
                               uint32_t prio, void *task_handle,
                               uint32_t core_id)
{
  return esp_task_create_pinned_to_core(task_func, name,
                                        stack_depth, param,
                                        prio, task_handle, core_id);
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
 * Name: is_in_isr_wrapper
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

static bool IRAM_ATTR is_in_isr_wrapper(void)
{
  return up_interrupt_context();
}

/****************************************************************************
 * Name: cause_sw_intr_to_core_wrapper
 *
 * Description:
 *   Just a wrapper to cause_sw_intr
 *
 * Input Parameters:
 *  core_id - ID of the CPU core, not used.
 *  intr_no - Number of the software interrupt
 *
 * Returned Value:
 *   Always return OK.
 *
 ****************************************************************************/

static int IRAM_ATTR cause_sw_intr_to_core_wrapper(int core_id, int intr_no)
{
  cause_sw_intr((void *)intr_no);
  return ESP_OK;
}

/****************************************************************************
 * Name: malloc_wrapper
 *
 * Description:
 *   Malloc buffer
 *
 * Input Parameters:
 *  size - buffer size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void *malloc_wrapper(size_t size)
{
  void * p = NULL;

  p = kmm_malloc(size);
  DEBUGASSERT(p);

  return p;
}

/****************************************************************************
 * Name: malloc_internal_wrapper
 *
 * Description:
 *   Malloc buffer in DRAM
 *
 * Input Parameters:
 *  szie - buffer size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void *malloc_internal_wrapper(size_t size)
{
  void * p = NULL;

  p = kmm_malloc(size);
  DEBUGASSERT(p);

  return p;
}

/****************************************************************************
 * Name: read_mac_wrapper
 *
 * Description:
 *   Get Mac Address
 *
 * Input Parameters:
 *   mac - mac address
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise, -1 (ERROR) is returned.
 *
 ****************************************************************************/

static int IRAM_ATTR read_mac_wrapper(uint8_t mac[6])
{
  return esp_read_mac(mac, ESP_MAC_BT);
}

/****************************************************************************
 * Name: srand_wrapper
 *
 * Description:
 *   Get random value with seed input. Not implemented.
 *
 * Input Parameters:
 *   seed - Value to be used as seed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR srand_wrapper(unsigned int seed)
{
  /* empty function */
}

/****************************************************************************
 * Name: rand_wrapper
 *
 * Description:
 *   Get random value.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Random value
 *
 ****************************************************************************/

static int IRAM_ATTR rand_wrapper(void)
{
  return esp_random();
}

/****************************************************************************
 * Name: btdm_lpcycles_2_us
 *
 * Description:
 *    Converts a number of low power clock cycles into a duration in us
 *
 * Input Parameters:
 *    cycles - number of CPU cycles
 *
 * Returned Value:
 *    us - value equivalent to the CPU cycles in us
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles)
{
  /* The number of lp cycles should not lead to overflow. Thrs: 100s
   * clock measurement is conducted
   */

  uint64_t us = (uint64_t)g_btdm_lpcycle_us * cycles;
  us = (us + (1 << (g_btdm_lpcycle_us_frac - 1))) >> g_btdm_lpcycle_us_frac;
  return (uint32_t)us;
}

/****************************************************************************
 * Name: btdm_us_2_lpcycles
 *
 * Description:
 *   Converts a duration in slots into a number of low power clock cycles.
 *
 * Input Parameters:
 *    us - duration in us
 *
 * Returned Value:
 *   cycles
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us)
{
  /* The number of sleep duration(us) should not lead to overflow. Thrs: 100s
   * Compute the sleep duration in us to low power clock cycles, with
   * calibration result applied clock measurement is conducted
   */

  uint64_t cycles;
  cycles = ((uint64_t)(us) << g_btdm_lpcycle_us_frac) / g_btdm_lpcycle_us;
  return (uint32_t)cycles;
}

/****************************************************************************
 * Name: btdm_sleep_check_duration
 *
 * Description:
 *   Wake up in advance considering the delay in enabling PHY/RF.
 *
 * Input Parameters:
 *   half_slot_cnt - half slots to allow to fall into modem sleep
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static bool IRAM_ATTR btdm_sleep_check_duration(uint32_t *slot_cnt)
{
  if (*slot_cnt < BTDM_MIN_SLEEP_DURATION)
    {
      return false;
    }

  *slot_cnt -= BTDM_MODEM_WAKE_UP_DELAY;
  return true;
}

/****************************************************************************
 * Name: btdm_sleep_enter_phase1_wrapper
 *
 * Description:
 *   ESP32 BLE lightsleep callback function.
 *
 * Input Parameters:
 *   lpcycles - light sleep cycles
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles)
{
#ifdef CONFIG_PM
  uint32_t us_to_sleep;
  uint32_t uncertainty;

  if (g_lp_cntl.wakeup_timer_required == false)
    {
      return;
    }

  /* start a timer to wake up and acquire the pm_lock before sleep awakes */

  us_to_sleep = btdm_lpcycles_2_us(lpcycles, NULL);

  DEBUGASSERT(us_to_sleep > BTDM_MIN_TIMER_UNCERTAINTY_US);
  uncertainty = (us_to_sleep >> 11);

  if (uncertainty < BTDM_MIN_TIMER_UNCERTAINTY_US)
    {
      uncertainty = BTDM_MIN_TIMER_UNCERTAINTY_US;
    }

  DEBUGASSERT(g_lp_stat.wakeup_timer_started == false);

  if (esp_timer_start_once(g_btdm_slp_tmr,
                           us_to_sleep - uncertainty) == ESP_OK)
    {
      g_lp_stat.wakeup_timer_started = true;
    }
  else
    {
      wlerr("timer start failed");
      DEBUGPANIC();
    }
#endif
}

/****************************************************************************
 * Name: btdm_sleep_enter_phase2_wrapper
 *
 * Description:
 *   ESP32 BLE lightsleep callback function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btdm_sleep_enter_phase2_wrapper(void)
{
  if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG)
    {
      esp_phy_disable();
#ifdef CONFIG_PM
      if (g_pm_lock_acquired)
        {
          esp32_pm_lockrelease();
          g_pm_lock_acquired = false;
        }
#endif
    }
  else if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_EVED)
    {
      esp_phy_disable();

      /* pause bluetooth baseband */

      periph_module_disable(PERIPH_BT_BASEBAND_MODULE);
    }
}

/****************************************************************************
 * Name: btdm_sleep_exit_phase3_wrapper
 *
 * Description:
 *   ESP32 BLE lightsleep callback function..
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void btdm_sleep_exit_phase3_wrapper(void)
{
#ifdef CONFIG_PM
  if (g_pm_lock_acquired == false)
    {
      g_pm_lock_acquired = true;
      esp32_pm_lockacquire();
    }
#endif

  if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG)
    {
      esp_phy_enable();
      btdm_check_and_init_bb();
#ifdef CONFIG_PM
      esp_timer_stop(g_btdm_slp_tmr);
#endif
    }
  else if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_EVED)
    {
      /* resume bluetooth baseband */

      periph_module_enable(PERIPH_BT_BASEBAND_MODULE);
      esp_phy_enable();
    }
}

/****************************************************************************
 * Name: coex_bt_wakeup_request
 *
 * Description:
 *   Request a Wi-Fi/BLE Coex wakeup request
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   true if request lock is needed, false otherwise
 *
 ****************************************************************************/

static bool coex_bt_wakeup_request(void)
{
  return async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_COEX);
}

/****************************************************************************
 * Name: coex_bt_wakeup_request_end
 *
 * Description:
 *   Finish Wi-Fi/BLE Coex wakeup request
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void coex_bt_wakeup_request_end(void)
{
  async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_COEX);
}

/****************************************************************************
 * Name: coex_bt_request_wrapper
 *
 * Description:
 *   Bluetooth requests coexistence.
 *
 * Input Parameters:
 *   event    - Bluetooth event
 *   latency  - Bluetooth will request coexistence after latency
 *   duration - duration for Bluetooth to request coexistence
 *
 * Returned Value:
 *   0 on success, other values indicate failure
 *
 ****************************************************************************/

static int IRAM_ATTR coex_bt_request_wrapper(uint32_t event,
                                             uint32_t latency,
                                             uint32_t duration)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_bt_request(event, latency, duration);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_bt_release_wrapper
 *
 * Description:
 *   Bluetooth releases coexistence.
 *
 * Input Parameters:
 *   event - Bluetooth event
 *
 * Returned Value:
 *   0 on success, other values indicate failure
 *
 ****************************************************************************/

static int IRAM_ATTR coex_bt_release_wrapper(uint32_t event)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_bt_release(event);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: adapter_coex_register_bt_cb_wrapper
 *
 * Description:
 *   Bluetooth registers callback function to coexistence module.
 *   This function is only used on ESP32.
 *
 * Input Parameters:
 *   cb - callback function registered to coexistence module
 *
 * Returned Value:
 *   0 on success, other values indicate failure
 *
 ****************************************************************************/

static int adapter_coex_register_bt_cb_wrapper(coex_func_cb_t cb)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_register_bt_cb(cb);
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_bb_reset_lock_wrapper
 *
 * Description:
 *   To acquire the spin-lock used in resetting Bluetooth baseband.
 *   This function is only used to workaround ESP32 hardware issue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Value of the spinlock to be restored
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR coex_bb_reset_lock_wrapper(void)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_bb_reset_lock();
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: coex_bb_reset_unlock_wrapper
 *
 * Description:
 *   To release the spin-lock used in resetting Bluetooth baseband.
 *   This function is only used to workaround ESP32 hardware issue.
 *
 * Input Parameters:
 *   restore - value of the spinlock returned from previous call of
 *             coex_bb_rest_lock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR coex_bb_reset_unlock_wrapper(uint32_t restore)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  coex_bb_reset_unlock(restore);
#endif
}

/****************************************************************************
 * Name: coex_schm_register_btdm_callback_wrapper
 *
 * Description:
 *   Register callback for coexistence scheme.
 *
 * Input Parameters:
 *   callback - callback function to be registered
 *
 * Returned Value:
 *   0 on success, other values indicate failure
 *
 ****************************************************************************/

static int coex_schm_register_btdm_callback_wrapper(void *callback)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_schm_register_callback(COEX_SCHM_CALLBACK_TYPE_BT, callback);
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
 *   type - Coexistence status type
 *   status - Coexistence status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                               uint32_t status)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
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
 *   type - Coexistence status type
 *   status - Coexistence status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  coex_schm_status_bit_set(type, status);
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
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
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
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_schm_interval_get();
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
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_schm_curr_phase_get();
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: coex_wifi_channel_get_wrapper
 *
 * Description:
 *   Get WiFi channel from coexistence module.
 *
 * Input Parameters:
 *   primary - pointer to value of WiFi primary channel
 *   secondary - pointer to value of WiFi secondary channel
 *
 * Returned Value:
 *   0 on success, other values indicate failure
 *
 ****************************************************************************/

static int coex_wifi_channel_get_wrapper(uint8_t *primary,
                                         uint8_t *secondary)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_wifi_channel_get(primary, secondary);
#else
  return -1;
#endif
}

/****************************************************************************
 * Name: coex_register_wifi_channel_change_callback_wrapper
 *
 * Description:
 *   Bluetooth registers callback function to receive notification when Wi-Fi
 *   channel changes.
 *
 * Input Parameters:
 *   cb - callback function registered to coexistence module
 *
 * Returned Value:
 *   0 on success, other values indicate failure
 *
 ****************************************************************************/

static int coex_register_wifi_channel_change_callback_wrapper(void *cb)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  return coex_register_wifi_channel_change_callback(cb);
#else
  return -1;
#endif
}

/****************************************************************************
 * Name: coex_version_get_wrapper
 *
 * Description:
 *   Get the version of the coexistence module.
 *
 * Input Parameters:
 *   major - pointer to store the major version number
 *   minor - pointer to store the minor version number
 *   patch - pointer to store the patch version number
 *
 * Returned Value:
 *   0 on success, -1 on failure
 *
 ****************************************************************************/

static int coex_version_get_wrapper(unsigned int *major,
                                    unsigned int *minor,
                                    unsigned int *patch)
{
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  const char *ver_str = coex_version_get();

  if (ver_str != NULL)
    {
      unsigned int _major = 0;
      unsigned int _minor = 0;
      unsigned int _patch = 0;

      if (sscanf(ver_str, "%u.%u.%u", &_major, &_minor, &_patch) != 3)
        {
          return -1;
        }

      if (major != NULL)
        {
          *major = _major;
        }

      if (minor != NULL)
        {
          *minor = _minor;
        }

      if (patch != NULL)
        {
          *patch = _patch;
        }

      return 0;
    }
#endif

  return -1;
}

/****************************************************************************
 * Other functions
 ****************************************************************************/

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

  DEBUGASSERT(task_handle != NULL);

#ifdef CONFIG_SMP
  ret = sched_lock();
  if (ret)
    {
      wlerr("Failed to lock scheduler before creating pinned thread\n");
      return false;
    }
#endif

  pid = kthread_create(name, prio, stack_depth, entry,
                       (char * const *)param);
  if (pid > 0)
    {
      if (task_handle)
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
      wlerr("Failed to create task, error %d\n", pid);
    }

#ifdef CONFIG_SMP
  ret = sched_unlock();
  if (ret)
    {
      wlerr("Failed to unlock scheduler after creating pinned thread\n");
      return false;
    }
#endif

  return pid > 0;
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

static IRAM_ATTR int32_t esp_queue_send_generic(void *queue, void *item,
                                                uint32_t ticks, int prio)
{
  int ret;
  struct timespec timeout;
  struct mq_adpt_s *mq_adpt = (struct mq_adpt_s *)queue;

#ifdef CONFIG_ESP32_SPIFLASH
  if (!spi_flash_cache_enabled())
    {
      esp_send_queuecache(queue, item, mq_adpt->msgsize);
      return esp_errno_trans(OK);
    }
#endif

  if (ticks == OSI_FUNCS_TIME_BLOCKING || ticks == 0)
    {
      /* BLE interrupt function will call this adapter function to send
       * message to message queue, so here we should call kernel API
       * instead of application API
       */

      ret = file_mq_send(&mq_adpt->mq, (const char *)item,
                         mq_adpt->msgsize, prio);
      if (ret < 0)
        {
          wlerr("Failed to send message to mqueue error=%d\n", ret);
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to get time %d\n", ret);
          return esp_errno_trans(ret);
        }

      if (ticks)
        {
          esp_update_time(&timeout, ticks);
        }

      ret = file_mq_timedsend(&mq_adpt->mq, (const char *)item,
                              mq_adpt->msgsize, prio, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to timedsend message to mqueue error=%d\n", ret);
        }
    }

  return esp_errno_trans(ret);
}

/****************************************************************************
 * Name: esp_update_time
 *
 * Description:
 *   Transform ticks to time and add this time to timespec value
 *
 * Input Parameters:
 *   ticks    - System ticks
 *
 * Output Parameters:
 *   timespec - Input timespec data pointer
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
 * Name: cause_sw_intr
 *
 * Description:
 *   Set a software interrupt bit
 *
 * Input Parameters:
 *  arg - number of the bit as void pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR cause_sw_intr(void *arg)
{
  /* just convert void * to int, because the width is the same */

  uint32_t intr_no = (uint32_t)arg;
  XTHAL_SET_INTSET((1 << intr_no));
}

#ifdef CONFIG_PM

/****************************************************************************
 * Name: btdm_slp_tmr_customer_callback
 *
 * Description:
 *   Callback function for ESP BLE sleep timer. This function is dispatched
 *   to the controller from `btdm_slp_tmr_callback`. If the power management
 *   lock has been released, it acquires the lock again.
 *
 * Input Parameters:
 *   arg - Unused
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btdm_slp_tmr_customer_callback(void * arg)
{
  (void)(arg);

  if (g_pm_lock_acquired == false)
    {
      g_pm_lock_acquired = true;
      esp32_pm_lockacquire();
    }
}

/****************************************************************************
 * Name: btdm_slp_tmr_callback
 *
 * Description:
 *   ESP BLE sleep callback function.
 *
 * Input Parameters:
 *   arg - Unused
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR btdm_slp_tmr_callback(void *arg)
{
  (void)(arg);
  btdm_dispatch_work_to_controller(btdm_slp_tmr_customer_callback,
                                   NULL,
                                   true);
}
#endif

/****************************************************************************
 * Name: esp_int_adpt_cb
 *
 * Description:
 *   BT interrupt adapter callback function
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (not used)
 *   arg     - Argument passed to the interrupt callback
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int IRAM_ATTR esp_int_adpt_cb(int irq, void *context, void *arg)
{
  struct irq_adpt_s *adapter = (struct irq_adpt_s *)arg;

  adapter->func(adapter->arg);

  return OK;
}

/****************************************************************************
 * Name: btdm_wakeup_request_callback
 *
 * Description:
 *   Callback function for ESP BLE wakeup request. This function is
 *   dispatched when a wakeup request is received. If the power management
 *   lock has not been acquired, it acquires the lock. It also stops the
 *   sleep timer and gives the wakeup request semaphore.
 *
 * Input Parameters:
 *   arg - Unused
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btdm_wakeup_request_callback(void * arg)
{
  (void)(arg);

#if CONFIG_PM
  if (g_pm_lock_acquired == false)
    {
      g_pm_lock_acquired = true;
      esp32_pm_lockacquire();
    }

  esp_timer_stop(g_btdm_slp_tmr);
#endif
  btdm_wakeup_request();

  semphr_give_wrapper(g_wakeup_req_sem);
}

/****************************************************************************
 * Name: btdm_controller_mem_init
 *
 * Description:
 *    Initialize BT controller to allocate task and other resource.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void btdm_controller_mem_init(void)
{
  int btdm_dram_regions;

  /* initialise .data section */

  memcpy(_data_start_btdm, (void *)_data_start_btdm_rom,
         _data_end_btdm - _data_start_btdm);

  wlinfo(".data initialise [0x%08x] <== [0x%08x]\n",
         (uint32_t)_data_start_btdm, _data_start_btdm_rom);

  /* initial em, .bss section */

  btdm_dram_regions = sizeof(g_btdm_dram_available_region)
                      / sizeof(btdm_dram_available_region_t);

  for (int i = 1; i < btdm_dram_regions; i++)
    {
      if (g_btdm_dram_available_region[i].mode != ESP_BT_MODE_IDLE)
        {
          memset((void *)g_btdm_dram_available_region[i].start, 0x0,
                 g_btdm_dram_available_region[i].end - \
                 g_btdm_dram_available_region[i].start);
          wlinfo(".bss initialise [0x%08x] - [0x%08x]\n",
                 g_btdm_dram_available_region[i].start,
                 g_btdm_dram_available_region[i].end);
        }
    }
}

/****************************************************************************
 * Name: btdm_config_mask_load
 *
 * Description:
 *   Create a mask with all ESP32 BLE supported features
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The uint32_t mask
 *
 ****************************************************************************/

static uint32_t btdm_config_mask_load(void)
{
  uint32_t mask = 0x0;

#ifdef CONFIG_UART_BTH4
  mask |= BTDM_CFG_HCI_UART;
#endif

#ifdef CONFIG_BTDM_CTRL_PINNED_TO_CORE_1
  mask |= BTDM_CFG_CONTROLLER_RUN_APP_CPU;
#endif

#ifdef CONFIG_BTDM_CTRL_FULL_SCAN_SUPPORTED
  mask |= BTDM_CFG_BLE_FULL_SCAN_SUPPORTED;
#endif

  mask |= BTDM_CFG_SCAN_DUPLICATE_OPTIONS;

  mask |= BTDM_CFG_SEND_ADV_RESERVED_SIZE;

  return mask;
}

/****************************************************************************
 * Name: bt_controller_deinit_internal
 *
 * Description:
 *   Deinitialize the internal structures of the Bluetooth controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bt_controller_deinit_internal(void)
{
  periph_module_disable(PERIPH_BT_MODULE);

#ifdef CONFIG_PM

  if (g_btdm_slp_tmr != NULL)
    {
      esp_timer_stop(g_btdm_slp_tmr);
      esp_timer_delete(g_btdm_slp_tmr);
      g_btdm_slp_tmr = NULL;
    }

  g_pm_lock_acquired = false;
#endif

  if (g_wakeup_req_sem)
    {
      semphr_delete_wrapper(g_wakeup_req_sem);
      g_wakeup_req_sem = NULL;
    }

  if (g_osi_funcs_p)
    {
      free(g_osi_funcs_p);
      g_osi_funcs_p = NULL;
    }

  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

  g_btdm_lpcycle_us = 0;
  btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE);

  esp_bt_power_domain_off();

  esp_phy_modem_deinit();
}

/****************************************************************************
 * Name: async_wakeup_request
 *
 * Description:
 *   Request the BLE Controller to wakeup
 *
 * Input Parameters:
 *   event - the event that triggered the wakeup
 *
 * Returned Value:
 *   true if request lock is needed, false otherwise
 *
 ****************************************************************************/

static bool async_wakeup_request(int event)
{
  bool request_lock = false;
  bool do_wakeup_request = false;

  switch (event)
    {
      case BTDM_ASYNC_WAKEUP_REQ_HCI:
        btdm_in_wakeup_requesting_set(true);

        /* No break */

      case BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA:
        if (!btdm_power_state_active())
          {
            do_wakeup_request = true;

            btdm_dispatch_work_to_controller(btdm_wakeup_request_callback,
                                             NULL,
                                             true);
            semphr_take_wrapper(g_wakeup_req_sem, OSI_FUNCS_TIME_BLOCKING);
          }
        break;
      case BTDM_ASYNC_WAKEUP_REQ_COEX:
        if (!btdm_power_state_active())
          {
            do_wakeup_request = true;
#if CONFIG_PM
            if (g_pm_lock_acquired == false)
              {
                g_pm_lock_acquired = true;
                esp32_pm_lockacquire();
              }

            esp_timer_stop(g_btdm_slp_tmr);
#endif
            btdm_wakeup_request();
          }
        break;
      default:
        return false;
    }

  return do_wakeup_request;
}

/****************************************************************************
 * Name: async_wakeup_request_end
 *
 * Description:
 *   Finish a wakeup request
 *
 * Input Parameters:
 *   event - the event that triggered the wakeup
 *
 * Returned Value:
 *   true if request lock is needed, false otherwise
 *
 ****************************************************************************/

static void async_wakeup_request_end(int event)
{
  bool request_lock = false;

  switch (event)
    {
      case BTDM_ASYNC_WAKEUP_REQ_HCI:
        request_lock = true;
        break;
      case BTDM_ASYNC_WAKEUP_REQ_COEX:
      case BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA:
        request_lock = false;
        break;
      default:
        return;
    }

  if (request_lock)
    {
      btdm_in_wakeup_requesting_set(false);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_bt_controller_init
 *
 * Description:
 *    Init  BT controller.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_bt_controller_init(void)
{
  esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  uint32_t btdm_cfg_mask = 0;
  int err;
  int i;
  bool select_src_ret;
  bool set_div_ret;
#ifdef CONFIG_PM
  esp_timer_create_args_t create_args =
    {
      .callback = btdm_slp_tmr_callback,
      .arg = NULL,
      .name = "btSlp"
    };
#endif

  /* If all the bt available memory was already released,
   * cannot initialize bluetooth controller
   */

  if (g_btdm_dram_available_region[0].mode == ESP_BT_MODE_IDLE)
    {
      wlerr("Error, bt available memory was released\n");
      return -EIO;
    }

  /* Initialize list of interrupt flags to enable chained critical sections
   * to return sucessfully.
   */

  sq_init(&g_ble_int_flags_free);
  sq_init(&g_ble_int_flags_used);

  for (i = 0; i < NR_IRQSTATE_FLAGS; i++)
    {
      sq_addlast((sq_entry_t *)&g_ble_int_flags[i], &g_ble_int_flags_free);
    }

#ifdef CONFIG_ESP32_SPIFLASH

  /* Initialize interfaces that enable BLE ISRs to run during a
   * SPI flash operation.
   */

  if (esp_wireless_init() != OK)
    {
      return -EIO;
    }
#endif

  g_osi_funcs_p =
      (struct osi_funcs_s *)kmm_malloc(sizeof(struct osi_funcs_s));

  if (g_osi_funcs_p == NULL)
    {
      return -ENOMEM;
    }

  memcpy(g_osi_funcs_p, &g_osi_funcs_ro, sizeof(struct osi_funcs_s));
  if (btdm_osi_funcs_register(g_osi_funcs_p) != 0)
    {
      wlerr("Invalid OSI Functions\n");
      return -EINVAL;
    }

  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE)
    {
      return ESP_ERR_INVALID_STATE;
    }

  /* overwrite some parameters */

  cfg.controller_task_stack_size = CONFIG_ESP32_BLE_TASK_STACK_SIZE;
  cfg.controller_task_prio       = CONFIG_ESP32_BLE_TASK_PRIORITY;
  cfg.bt_max_sync_conn           = CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF;
  cfg.magic                      = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL;

  if (((cfg.mode & ESP_BT_MODE_BLE) && (cfg.ble_max_conn <= 0 ||
        cfg.ble_max_conn > BTDM_CONTROLLER_BLE_MAX_CONN_LIMIT)) ||
      ((cfg.mode & ESP_BT_MODE_CLASSIC_BT) && (cfg.bt_max_acl_conn <= 0 ||
        cfg.bt_max_acl_conn > BTDM_CONTROLLER_BR_EDR_MAX_ACL_CONN_LIMIT)) ||
      ((cfg.mode & ESP_BT_MODE_CLASSIC_BT) &&
       (cfg.bt_max_sync_conn > BTDM_CONTROLLER_BR_EDR_MAX_SYNC_CONN_LIMIT)))
    {
      wlerr("%s  %d\n", __func__, __LINE__);
      return -EINVAL;
    }

  wlinfo("BT controller compile version [%s]",
          btdm_controller_get_compile_version());

  g_wakeup_req_sem = semphr_create_wrapper(1, 0);
  if (g_wakeup_req_sem == NULL)
    {
      err = -ENOMEM;
      goto error;
    }

  esp_phy_modem_init();

  esp_bt_power_domain_on();

  btdm_controller_mem_init();

  periph_module_enable(PERIPH_BT_MODULE);

#ifdef CONFIG_PM
  g_btdm_allow_light_sleep = false;
#endif

  /* set default sleep clock cycle and its fractional bits */

  g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
  g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

#ifdef CONFIG_BTDM_CTRL_MODEM_SLEEP_MODE_ORIG
  g_btdm_lpclk_sel = BTDM_LPCLK_SEL_XTAL; /* set default value */
#ifdef CONFIG_BTDM_CTRL_LPCLK_SEL_EXT_32K_XTAL

  /* check whether or not EXT_CRYS is working */

  if (rtc_clk_slow_src_get() == SOC_RTC_SLOW_CLK_SRC_XTAL32K)
    {
      g_btdm_lpclk_sel = BTDM_LPCLK_SEL_XTAL32K; /* External 32kHz XTAL */
#ifdef CONFIG_PM
      g_btdm_allow_light_sleep = true;
#endif
    }
  else
    {
      wlwarn("32.768kHz XTAL not detected, fall back to main XTAL as "
             "Bluetooth sleep clock\n"
             "light sleep mode will not be able to apply when bluetooth "
             "is enabled");
      g_btdm_lpclk_sel = BTDM_LPCLK_SEL_XTAL; /* set default value */
    }
#else
  g_btdm_lpclk_sel = BTDM_LPCLK_SEL_XTAL; /* set default value */
#endif

  if (g_btdm_lpclk_sel == BTDM_LPCLK_SEL_XTAL)
    {
      select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL);
      set_div_ret = btdm_lpclk_set_div(esp_clk_xtal_freq() * 2 / MHZ - 1);
      assert(select_src_ret && set_div_ret);
      g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
      g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);
    }
  else
    {
      /* g_btdm_lpclk_sel == BTDM_LPCLK_SEL_XTAL32K */

      select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL32K);
      set_div_ret = btdm_lpclk_set_div(0);
      assert(select_src_ret && set_div_ret);
      g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
      g_btdm_lpcycle_us = (RTC_CLK_CAL_FRACT > 15) ?
                          (1000000 << (RTC_CLK_CAL_FRACT - 15)) :
                          (1000000 >> (15 - RTC_CLK_CAL_FRACT));
      assert(g_btdm_lpcycle_us != 0);
    }

  btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_ORIG);

#elif CONFIG_BTDM_CTRL_MODEM_SLEEP_MODE_EVED
  btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_EVED);
  UNUSED(select_src_ret);
  UNUSED(set_div_ret);
#else
  btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE);
  UNUSED(select_src_ret);
  UNUSED(set_div_ret);
#endif

#ifdef CONFIG_PM
  if ((err = esp_timer_create(&create_args, &g_btdm_slp_tmr) != OK))
    {
      wlerr("Failed to create timer");
      goto error;
    }

  g_pm_lock_acquired = true;
#endif

#if CONFIG_ESP32_WIFI_BT_COEXIST
  coex_init();
#endif

  btdm_cfg_mask = btdm_config_mask_load();

  wlinfo("Going to call btdm_controller_init\n");

  if (btdm_controller_init(btdm_cfg_mask, &cfg) != 0)
    {
      wlerr("Failed to initialize the BLE Controller\n");
      err = -ENOMEM;
      goto error;
    }

  wlinfo("The btdm_controller_init was initialized\n");

  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

  return OK;

error:

  bt_controller_deinit_internal();

  return err;
}

/****************************************************************************
 * Name: esp32_bt_controller_deinit
 *
 * Description:
 *   Deinit BT controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise, -1 (ERROR) is returned.
 *
 ****************************************************************************/

int esp32_bt_controller_deinit(void)
{
  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED)
    {
      return ERROR;
    }

  btdm_controller_deinit();

  bt_controller_deinit_internal();

  return OK;
}

/****************************************************************************
 * Name: esp32_bt_controller_enable
 *
 * Description:
 *   Enable BT controller.
 *
 * Input Parameters:
 *   mode - the mode(BLE/BT/BTDM) to enable. For compatible of API, retain
 *   this argument. This mode must be equal as the mode in "cfg" of
 *   esp_bt_controller_init().
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_bt_controller_enable(esp_bt_mode_t mode)
{
  int ret = OK;

  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED)
    {
      return ERROR;
    }

  if (mode != btdm_controller_get_mode())
    {
      wlerr("invalid mode %d, controller support mode is %d",
            mode, btdm_controller_get_mode());
      return ERROR;
    }

#ifdef CONFIG_PM
  if (g_btdm_allow_light_sleep == false)
    {
      esp32_pm_lockacquire();
    }

  esp32_pm_lockacquire();
#endif

  esp_phy_enable();

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  coex_enable();
#endif

  if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG)
    {
      btdm_controller_enable_sleep(true);
    }

  /* inititalize bluetooth baseband */

  btdm_check_and_init_bb();

  ret = btdm_controller_enable(mode);
  if (ret != 0)
    {
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
      coex_disable();
#endif
      esp_phy_disable();
#ifdef CONFIG_PM
      if (g_btdm_allow_light_sleep == false)
        {
          esp32_pm_lockrelease();
        }

      esp32_pm_lockrelease();
#endif
      return ERROR;
    }

  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;

  return OK;
}

/****************************************************************************
 * Name: esp32_bt_controller_disable
 *
 * Description:
 *   Disable BT controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_bt_controller_disable(void)
{
  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return ERROR;
    }

  /* disable modem sleep and wake up from sleep mode */

  if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG)
    {
      btdm_controller_enable_sleep(false);
      async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA);
      while (btdm_power_state_active() == false)
        {
          nxsig_usleep(1000);
        }
    }

  btdm_controller_disable();

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  coex_disable();
#endif

  esp_phy_disable();
  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

#ifdef CONFIG_PM
  if (g_btdm_allow_light_sleep == false)
    {
      esp32_pm_lockrelease();
    }

  esp32_pm_lockrelease();
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32_bt_controller_get_status
 *
 * Description:
 *   Returns the status of the BT Controller
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current status (type esp_bt_controller_status_t)
 *
 ****************************************************************************/

esp_bt_controller_status_t esp32_bt_controller_get_status(void)
{
  return g_btdm_controller_status;
}

/****************************************************************************
 * Name: esp32_vhci_host_check_send_available
 *
 * Description:
 *   Check if the host can send packet to controller or not.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   bool - true or false
 *
 ****************************************************************************/

bool esp32_vhci_host_check_send_available(void)
{
  return api_vhci_host_check_send_available();
}

/****************************************************************************
 * Name: esp32_vhci_host_send_packet
 *
 * Description:
 *   Host send packet to controller.
 *
 * Input Parameters:
 *   data - the packet pointer
 *   len  - the packet length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
  async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_HCI);

  api_vhci_host_send_packet(data, len);

  async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_HCI);
}

/****************************************************************************
 * Name: esp32_vhci_register_callback
 *
 * Description:
 *    Register the vhci reference callback.
 *
 * Input Parameters:
 *  callback - struct defined by vhci_host_callback structure.
 *
 * Returned Value:
 *   status - success or fail
 *
 ****************************************************************************/

int esp32_vhci_register_callback(const esp_vhci_host_callback_t *callback)
{
  int ret = ERROR;

  ret = api_vhci_host_register_callback(
            (const vhci_host_callback_t *)callback) == 0 ? 0 : -1;
  return ret;
}
