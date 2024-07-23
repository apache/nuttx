/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_ble_adapter.c
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

#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_syscon.h"
#include "hardware/wdev_reg.h"
#include "rom/esp32s3_spiflash.h"
#include "xtensa.h"
#include "esp_attr.h"
#include "esp32s3_irq.h"
#include "esp32s3_rt_timer.h"
#include "esp32s3_rtc.h"
#include "esp32s3_spiflash.h"
#include "esp32s3_wireless.h"

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
#include "esp_coexist_internal.h"

#include "esp32s3_ble_adapter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BTDM_MIN_TIMER_UNCERTAINTY_US    (1800)

/* Sleep and wakeup interval control */

#define BTDM_MIN_SLEEP_DURATION          (24) /* Threshold of interval in half slots to allow to fall into sleep mode */
#define BTDM_MODEM_WAKE_UP_DELAY         (8)  /* delay in half slots of modem wake up procedure, including re-enable PHY/RF */

/* Low Power Clock Selection */

#define BTDM_LPCLK_SEL_XTAL              (0)
#define BTDM_LPCLK_SEL_XTAL32K           (1)
#define BTDM_LPCLK_SEL_RTC_SLOW          (2)
#define BTDM_LPCLK_SEL_8M                (3)

#define OSI_FUNCS_TIME_BLOCKING          0xffffffff
#define OSI_VERSION                      0x00010006
#define OSI_MAGIC_VALUE                  0xfadebead

#ifdef CONFIG_ESP32S3_SPIFLASH
#  define BLE_TASK_EVENT_QUEUE_ITEM_SIZE  8
#  define BLE_TASK_EVENT_QUEUE_LEN        8
#endif

#ifdef CONFIG_ESP32S3_BLE_INTERRUPT_SAVE_STATUS
#  define NR_IRQSTATE_FLAGS   CONFIG_ESP32S3_BLE_INTERRUPT_SAVE_STATUS
#else
#  define NR_IRQSTATE_FLAGS   3
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* VHCI function interface */

typedef struct vhci_host_callback_s
{
  void (*notify_host_send_available)(void);               /* callback used to notify that the host can send packet to controller */
  int (*notify_host_recv)(uint8_t *data, uint16_t len);   /* callback used to notify that the controller has a packet to send to the host */
} vhci_host_callback_t;

/* BLE OS function */

struct osi_funcs_s
{
  uint32_t _magic;
  uint32_t _version;
  void (*_interrupt_set)(int cpu_no, int intr_source,
                         int interrupt_no, int interrpt_prio);
  void (*_interrupt_clear)(int interrupt_source, int interrupt_no);
  void (*_interrupt_handler_set)(int interrupt_no, void * fn, void *arg);
  void (*_interrupt_disable)(void);
  void (*_interrupt_restore)(void);
  void (*_task_yield)(void);
  void (*_task_yield_from_isr)(void);
  void *(*_semphr_create)(uint32_t max, uint32_t init);
  void (*_semphr_delete)(void *semphr);
  int (*_semphr_take_from_isr)(void *semphr, void *hptw);
  int (*_semphr_give_from_isr)(void *semphr, void *hptw);
  int (*_semphr_take)(void *semphr, uint32_t block_time_ms);
  int (*_semphr_give)(void *semphr);
  void *(*_mutex_create)(void);
  void (*_mutex_delete)(void *mutex);
  int (*_mutex_lock)(void *mutex);
  int (*_mutex_unlock)(void *mutex);
  void *(* _queue_create)(uint32_t queue_len, uint32_t item_size);
  void (* _queue_delete)(void *queue);
  int (* _queue_send)(void *queue, void *item, uint32_t block_time_ms);
  int (* _queue_send_from_isr)(void *queue, void *item, void *hptw);
  int (* _queue_recv)(void *queue, void *item, uint32_t block_time_ms);
  int (* _queue_recv_from_isr)(void *queue, void *item, void *hptw);
  int (* _task_create)(void *task_func, const char *name,
                       uint32_t stack_depth, void *param, uint32_t prio,
                       void *task_handle, uint32_t core_id);
  void (* _task_delete)(void *task_handle);
  bool (* _is_in_isr)(void);
  int (* _cause_sw_intr_to_core)(int core_id, int intr_no);
  void *(* _malloc)(size_t size);
  void *(* _malloc_internal)(size_t size);
  void (* _free)(void *p);
  int (* _read_efuse_mac)(uint8_t mac[6]);
  void (* _srand)(unsigned int seed);
  int (* _rand)(void);
  uint32_t (* _btdm_lpcycles_2_hus)(uint32_t cycles, uint32_t *error_corr);
  uint32_t (* _btdm_hus_2_lpcycles)(uint32_t us);
  bool (* _btdm_sleep_check_duration)(int32_t *slot_cnt);
  void (* _btdm_sleep_enter_phase1)(uint32_t lpcycles);  /* called when interrupt is disabled */
  void (* _btdm_sleep_enter_phase2)(void);
  void (* _btdm_sleep_exit_phase1)(void);  /* called from ISR */
  void (* _btdm_sleep_exit_phase2)(void);  /* called from ISR */
  void (* _btdm_sleep_exit_phase3)(void);  /* called from task */
  void (* _coex_wifi_sleep_set)(bool sleep);
  int (* _coex_core_ble_conn_dyn_prio_get)(bool *low, bool *high);
  void (* _coex_schm_status_bit_set)(uint32_t type, uint32_t status);
  void (* _coex_schm_status_bit_clear)(uint32_t type, uint32_t status);
  void (* _interrupt_on)(int intr_num);
  void (* _interrupt_off)(int intr_num);
  void (* _esp_hw_power_down)(void);
  void (* _esp_hw_power_up)(void);
  void (* _ets_backup_dma_copy)(uint32_t reg,
                                uint32_t mem_addr, uint32_t num,
                                bool to_rem);
};

/* BLE message queue private data */

struct mq_adpt_s
{
  struct file mq;           /* Message queue handle */
  uint32_t    msgsize;      /* Message size */
  char        name[16];     /* Message queue name */
};

/* BLE interrupt adapter private data */

struct irq_adpt_s
{
  void (*func)(void *arg);  /* Interrupt callback function */
  void *arg;                /* Interrupt private data */
};

typedef enum btdm_vnd_ol_sig_e
{
  BTDM_VND_OL_SIG_WAKEUP_TMR,
  BTDM_VND_OL_SIG_NUM,
} btdm_vnd_ol_sig_t;

/* low power control struct */

typedef union
{
  struct
  {
    uint32_t enable                  :  1; /* whether low power mode is required */
    uint32_t lpclk_sel               :  2; /* low power clock source */
    uint32_t mac_bb_pd               :  1; /* whether hardware(MAC, BB) force-power-down is required during sleep */
    uint32_t wakeup_timer_required   :  1; /* whether system timer is needed */
    uint32_t no_light_sleep          :  1; /* do not allow system to enter light sleep after bluetooth is enabled */
    uint32_t main_xtal_pu            :  1; /* power up main XTAL */
    uint32_t reserved                : 25; /* reserved */
  };
  uint32_t val;
} btdm_lpcntl_t;

/* low power control status */

typedef union
{
  struct
  {
    uint32_t pm_lock_released        :  1; /* whether power management lock is released */
    uint32_t mac_bb_pd               :  1; /* whether hardware(MAC, BB) is powered down */
    uint32_t phy_enabled             :  1; /* whether phy is switched on */
    uint32_t wakeup_timer_started    :  1; /* whether wakeup timer is started */
    uint32_t reserved                : 28; /* reserved */
  };
  uint32_t val;
} btdm_lpstat_t;

/* wakeup request sources */

enum btdm_wakeup_src_e
{
  BTDM_ASYNC_WAKEUP_SRC_VHCI,
  BTDM_ASYNC_WAKEUP_SRC_DISA,
  BTDM_ASYNC_WAKEUP_SRC_TMR,
  BTDM_ASYNC_WAKEUP_SRC_MAX,
};

/* Superseded semaphore definition */

struct bt_sem_s
{
  sem_t sem;
#ifdef CONFIG_ESP32S3_SPIFLASH
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

static void interrupt_set_wrapper(int cpu_no, int intr_source,
                                  int intr_num, int intr_prio);
static void interrupt_clear_wrapper(int intr_source, int intr_num);
static void interrupt_handler_set_wrapper(int intr_num, void *fn, void *arg);
static void IRAM_ATTR interrupt_disable(void);
static void IRAM_ATTR interrupt_restore(void);
static void IRAM_ATTR task_yield_from_isr(void);
static void *semphr_create_wrapper(uint32_t max, uint32_t init);
static void semphr_delete_wrapper(void *semphr);
static int IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw);
static int IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw);
static int  semphr_take_wrapper(void *semphr, uint32_t block_time_ms);
static int  semphr_give_wrapper(void *semphr);
static void *mutex_create_wrapper(void);
static void mutex_delete_wrapper(void *mutex);
static int  mutex_lock_wrapper(void *mutex);
static int  mutex_unlock_wrapper(void *mutex);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static void queue_delete_wrapper(void *queue);
static int queue_send_wrapper(void *queue, void *item,
                              uint32_t block_time_ms);
static int IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item,
                                                 void *hptw);
static int queue_recv_wrapper(void *queue, void *item,
                              uint32_t block_time_ms);
static int IRAM_ATTR queue_recv_from_isr_wrapper(void *queue, void *item,
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
static void *malloc_wrapper(size_t size);
static void *malloc_internal_wrapper(size_t size);
static int IRAM_ATTR read_mac_wrapper(uint8_t mac[6]);
static void IRAM_ATTR srand_wrapper(unsigned int seed);
static int IRAM_ATTR rand_wrapper(void);
static uint32_t IRAM_ATTR btdm_lpcycles_2_hus(uint32_t cycles,
                                              uint32_t *error_corr);
static uint32_t IRAM_ATTR btdm_hus_2_lpcycles(uint32_t us);
static bool IRAM_ATTR btdm_sleep_check_duration(int32_t *half_slot_cnt);
static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles);
static void btdm_sleep_enter_phase2_wrapper(void);
static void btdm_sleep_exit_phase3_wrapper(void);
static void coex_wifi_sleep_set_hook(bool sleep);
static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status);
static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                               uint32_t status);

static void interrupt_on_wrapper(int intr_num);
static void interrupt_off_wrapper(int intr_num);
void IRAM_ATTR btdm_hw_mac_power_down_wrapper(void);
void IRAM_ATTR btdm_hw_mac_power_up_wrapper(void);
void IRAM_ATTR btdm_backup_dma_copy_wrapper(uint32_t reg, uint32_t mem_addr,
                                            uint32_t num,  bool to_mem);

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
static void IRAM_ATTR btdm_slp_tmr_callback(void *arg);
static int IRAM_ATTR esp_int_adpt_cb(int irq, void *context, void *arg);
static void IRAM_ATTR btdm_sleep_exit_phase0(void *param);
#if CONFIG_MAC_BB_PD
static void IRAM_ATTR btdm_mac_bb_power_down_cb(void);
static void IRAM_ATTR btdm_mac_bb_power_up_cb(void);
#endif
static void btdm_controller_mem_init(void);
static void bt_controller_deinit_internal(void);
static bool async_wakeup_request(int event);
static void async_wakeup_request_end(int event);

/****************************************************************************
 * Extern Functions declaration and value
 ****************************************************************************/

extern int btdm_osi_funcs_register(void *osi_funcs);

/* Initialise and De-initialise */

extern int btdm_controller_init(esp_bt_controller_config_t *config_opts);
extern void btdm_controller_deinit(void);
extern int btdm_controller_enable(esp_bt_mode_t mode);
extern void btdm_controller_disable(void);
extern uint8_t btdm_controller_get_mode(void);
extern const char *btdm_controller_get_compile_version(void);
extern void btdm_rf_bb_init_phase2(void); /* shall be called after PHY/RF is enabled */

/* Sleep */

extern void btdm_controller_enable_sleep(bool enable);
extern uint8_t btdm_controller_get_sleep_mode(void);
extern bool btdm_power_state_active(void);
extern void btdm_wakeup_request(void);
extern void btdm_in_wakeup_requesting_set(bool in_wakeup_requesting);

/* vendor dependent tasks to be posted and handled by controller task */

extern int btdm_vnd_offload_task_register(btdm_vnd_ol_sig_t sig,
                                          btdm_vnd_ol_task_func_t func);
extern int btdm_vnd_offload_task_deregister(btdm_vnd_ol_sig_t sig);
extern int r_btdm_vnd_offload_post_from_isr(btdm_vnd_ol_sig_t sig,
                                          void *param, bool need_yield);
extern int r_btdm_vnd_offload_post(btdm_vnd_ol_sig_t sig, void *param);

/* Low Power Clock */

extern bool btdm_lpclk_select_src(uint32_t sel);
extern bool btdm_lpclk_set_div(uint32_t div);
extern int btdm_hci_tl_io_event_post(int event);

/* VHCI */

extern bool api_vhci_host_check_send_available(void); /* Functions in bt lib */
extern void api_vhci_host_send_packet(uint8_t * data, uint16_t len);
extern int api_vhci_host_register_callback(const vhci_host_callback_t
                                           *callback);

/* TX power */

extern int ble_txpwr_set(int power_type, int power_level);
extern int ble_txpwr_get(int power_type);

extern uint16_t l2c_ble_link_get_tx_buf_num(void);
extern int coex_core_ble_conn_dyn_prio_get(bool *low, bool *high);
extern void coex_pti_v2(void);

extern bool btdm_deep_sleep_mem_init(void);
extern void btdm_deep_sleep_mem_deinit(void);
extern void btdm_ble_power_down_dma_copy(bool copy);
extern uint8_t btdm_sleep_clock_sync(void);

#if CONFIG_MAC_BB_PD
extern void esp_mac_bb_power_down(void);
extern void esp_mac_bb_power_up(void);
extern void ets_backup_dma_copy(uint32_t reg, uint32_t mem_addr,
                                uint32_t num, bool to_mem);
#endif

extern uint8_t _bt_bss_start[];
extern uint8_t _bt_bss_end[];
extern uint8_t _btdm_bss_start[];
extern uint8_t _btdm_bss_end[];
extern uint8_t _bt_data_start[];
extern uint8_t _bt_data_end[];
extern uint8_t _btdm_data_start[];
extern uint8_t _btdm_data_end[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* BLE OS adapter data */

static struct osi_funcs_s g_osi_funcs =
{
  ._magic = OSI_MAGIC_VALUE,
  ._version = OSI_VERSION,
  ._interrupt_set = interrupt_set_wrapper,
  ._interrupt_clear = interrupt_clear_wrapper,
  ._interrupt_handler_set = interrupt_handler_set_wrapper,
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
  ._cause_sw_intr_to_core = NULL,
  ._malloc = malloc_wrapper,
  ._malloc_internal = malloc_internal_wrapper,
  ._free = free,
  ._read_efuse_mac = read_mac_wrapper,
  ._srand = srand_wrapper,
  ._rand = rand_wrapper,
  ._btdm_lpcycles_2_hus = btdm_lpcycles_2_hus,
  ._btdm_hus_2_lpcycles = btdm_hus_2_lpcycles,
  ._btdm_sleep_check_duration = btdm_sleep_check_duration,
  ._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
  ._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
  ._btdm_sleep_exit_phase1 = NULL,
  ._btdm_sleep_exit_phase2 = NULL,
  ._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
  ._coex_wifi_sleep_set = coex_wifi_sleep_set_hook,
  ._coex_core_ble_conn_dyn_prio_get = NULL,
  ._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
  ._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
  ._interrupt_on = interrupt_on_wrapper,
  ._interrupt_off = interrupt_off_wrapper,
  ._esp_hw_power_down = btdm_hw_mac_power_down_wrapper,
  ._esp_hw_power_up = btdm_hw_mac_power_up_wrapper,
  ._ets_backup_dma_copy = btdm_backup_dma_copy_wrapper,
};

static DRAM_ATTR struct osi_funcs_s *osi_funcs_p;

/* Controller status */

static DRAM_ATTR esp_bt_controller_status_t g_btdm_controller_status =
    ESP_BT_CONTROLLER_STATUS_IDLE;

/* measured average low power clock period in micro seconds */

static DRAM_ATTR uint32_t g_btdm_lpcycle_us = 0;

/* number of fractional bit for g_btdm_lpcycle_us */

static DRAM_ATTR uint8_t g_btdm_lpcycle_us_frac = 0;

/* low power status struct */

static DRAM_ATTR btdm_lpstat_t g_lp_stat;

/* low power control struct */

static DRAM_ATTR btdm_lpcntl_t g_lp_cntl;

/* semaphore used for blocking VHCI API to wait for controller to wake up */

static DRAM_ATTR void * g_wakeup_req_sem = NULL;

/* wakeup timer */

static DRAM_ATTR esp_timer_handle_t g_btdm_slp_tmr;

#ifdef CONFIG_PM

/* pm_lock to prevent light sleep due to incompatibility currently */

static DRAM_ATTR void * g_light_sleep_pm_lock;
#endif

/* BT interrupt private data */

static sq_queue_t g_ble_int_flags_free;

static sq_queue_t g_ble_int_flags_used;

static struct irqstate_list_s g_ble_int_flags[NR_IRQSTATE_FLAGS];

/* Cached queue control variables */

#ifdef CONFIG_ESP32S3_SPIFLASH
static struct esp_queuecache_s g_esp_queuecache;
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
 *   Transform from nuttx error code to Wi-Fi adapter error code
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   Wi-Fi adapter error code
 *
 ****************************************************************************/

static inline int32_t esp_errno_trans(int ret)
{
  return ret == 0;
}

/****************************************************************************
 * Functions to be registered to struct osi_funcs_s
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bt_power_domain_on
 *
 * Description:
 *   Power up the BT domain
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static IRAM_ATTR void esp_bt_power_domain_on(void)
{
#if SOC_PM_SUPPORT_BT_PD
  modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_BT_FORCE_PD, 0);
  modifyreg32(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_BT_FORCE_ISO, 0);
#endif
  esp_wifi_bt_power_domain_on();
}

/****************************************************************************
 * Name: esp_bt_power_domain_off
 *
 * Description:
 *   Power down the BT domain
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static IRAM_ATTR void esp_bt_power_domain_off(void)
{
#if SOC_PM_SUPPORT_BT_PD
  modifyreg32(RTC_CNTL_DIG_ISO_REG, 0, RTC_CNTL_BT_FORCE_ISO);
  modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_BT_FORCE_PD);
#endif
  esp_wifi_bt_power_domain_off();
}

/****************************************************************************
 * Name: interrupt_set_wrapper
 *
 * Description:
 *   Bind IRQ and resource with given parameters.
 *
 * Input Parameters:
 *   cpu_no      - The CPU which the interrupt number belongs.
 *   intr_source - The interrupt peripheral number.
 *   intr_num    - The interrupt CPU number.
 *   intr_prio   - The interrupt priority.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void interrupt_set_wrapper(int cpu_no,
                                  int intr_source,
                                  int intr_num,
                                  int intr_prio)
{
  /* There is no need to set up the interrupt matrix here - neither by
   * calling `esp_rom_route_intr_matrix` nor by directly setting the
   * related registers - because the callback `interrupt_on_wrapper` calls
   * `up_enable_irq`, which already set this up.
   */

  wlinfo("cpu_no=%d , intr_source=%d , intr_num=%d, intr_prio=%d\n",
         cpu_no, intr_source, intr_num, intr_prio);
}

/****************************************************************************
 * Name: interrupt_clear_wrapper
 *
 * Description:
 *   Not supported. Clear previously bound interrupt IRQ.
 *
 * Input Parameters:
 *   intr_source - The interrupt peripheral number.
 *   intr_num    - The interrupt CPU number.
 *
 * Returned Value:
 *   NuttX error code
 *
 ****************************************************************************/

static void IRAM_ATTR interrupt_clear_wrapper(int intr_source, int intr_num)
{
}

/****************************************************************************
 * Name: interrupt_handler_set_wrapper
 *
 * Description:
 *   Register interrupt function
 *
 * Input Parameters:
 *   intr_num - The interrupt CPU number.
 *   fn       - Interrupt function
 *   arg      - Function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void interrupt_handler_set_wrapper(int intr_num, void *fn, void *arg)
{
  struct irq_adpt_s *adapter;
  int irq = esp32s3_getirq(0, intr_num);

  wlinfo("intr_num=%d fn=%p arg=%p irq=%d\n", intr_num, fn, arg, irq);

  adapter = kmm_malloc(sizeof(struct irq_adpt_s));
  DEBUGASSERT(adapter);

  adapter->func = fn;
  adapter->arg = arg;

  DEBUGVERIFY(irq_attach(irq, esp_int_adpt_cb, adapter));
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
 *   Perform a solicited context switch on FreeRTOS. Do nothing in NuttX.
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

#ifdef CONFIG_ESP32S3_SPIFLASH
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
  nxsem_destroy(&bt_sem->sem);
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
 *   hptw   - Unused.
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

#ifdef CONFIG_ESP32S3_SPIFLASH
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

#ifdef CONFIG_ESP32S3_SPIFLASH
  if (queue_len <= BLE_TASK_EVENT_QUEUE_LEN &&
      item_size == BLE_TASK_EVENT_QUEUE_ITEM_SIZE)
    {
      esp_init_queuecache(&g_esp_queuecache,
                          &mq_adpt->mq,
                          g_esp_queuecache_buffer,
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

      if (ret < 0 && ret != -ETIMEDOUT)
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
 * Name: malloc_wrapper
 *
 * Description:
 *   Malloc buffer
 *
 * Input Parameters:
 *   size - buffer size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void *malloc_wrapper(size_t size)
{
  void * p = kmm_malloc(size);
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
 *   size - buffer size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void *malloc_internal_wrapper(size_t size)
{
  void * p = kmm_malloc(size);

  if (p != NULL)
    {
      if (esp32s3_ptr_extram(p))
        {
          kmm_free(p);
          return NULL;
        }
    }

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
 * Name: btdm_lpcycles_2_hus
 *
 * Description:
 *   Converts a number of low power clock cycles into a duration in half us.
 *
 * Input Parameters:
 *   cycles     - number of CPU cycles
 *   error_corr - local error corr
 *
 * Returned Value:
 *   res - value equivalent to the CPU cycles
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_lpcycles_2_hus(uint32_t cycles,
                                              uint32_t *error_corr)
{
  uint64_t local_error_corr;
  uint64_t res;

  local_error_corr = (error_corr == NULL) ? 0 : (uint64_t)(*error_corr);
  res = (uint64_t)g_btdm_lpcycle_us * cycles * 2;

  local_error_corr += res;
  res = (local_error_corr >> g_btdm_lpcycle_us_frac);
  local_error_corr -= (res << g_btdm_lpcycle_us_frac);
  if (error_corr)
    {
      *error_corr = (uint32_t)local_error_corr;
    }

  return (uint32_t)res;
}

/****************************************************************************
 * Name: btdm_hus_2_lpcycles
 *
 * Description:
 *   Converts a duration in half us into a number of low power clock cycles.
 *
 * Input Parameters:
 *   hus - sleep duration (us)
 *
 * Returned Value:
 *   cycles
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_hus_2_lpcycles(uint32_t hus)
{
  /* The number of sleep duration(us) should not lead to overflow. Thrs: 100s
   * Compute the sleep duration in us to low power clock cycles, with
   * calibration result applied clock measurement is conducted
   */

  uint64_t cycles;

  cycles = ((uint64_t)(hus) << g_btdm_lpcycle_us_frac) / g_btdm_lpcycle_us;
  cycles >>= 1;

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

static bool IRAM_ATTR btdm_sleep_check_duration(int32_t *half_slot_cnt)
{
  if (*half_slot_cnt < BTDM_MIN_SLEEP_DURATION)
    {
      return false;
    }

  *half_slot_cnt -= BTDM_MODEM_WAKE_UP_DELAY;
  return true;
}

/****************************************************************************
 * Name: btdm_sleep_enter_phase1_wrapper
 *
 * Description:
 *   ESP32-S3 BLE lightsleep callback function.
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
  uint32_t us_to_sleep;
  uint32_t uncertainty;

  if (g_lp_cntl.wakeup_timer_required == 0)
    {
      return;
    }

  /* start a timer to wake up and acquire the pm_lock before sleep awakes */

  us_to_sleep = btdm_lpcycles_2_hus(lpcycles, NULL) >> 1;

  DEBUGASSERT(us_to_sleep > BTDM_MIN_TIMER_UNCERTAINTY_US);
  uncertainty = (us_to_sleep >> 11);

  if (uncertainty < BTDM_MIN_TIMER_UNCERTAINTY_US)
    {
      uncertainty = BTDM_MIN_TIMER_UNCERTAINTY_US;
    }

  DEBUGASSERT(g_lp_stat.wakeup_timer_started == 0);

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
}

/****************************************************************************
 * Name: btdm_sleep_enter_phase2_wrapper
 *
 * Description:
 *   ESP32-S3 BLE lightsleep callback function.
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
  if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1)
    {
      if (g_lp_stat.phy_enabled)
        {
          esp_phy_disable();
          g_lp_stat.phy_enabled = 0;
        }
      else
        {
          DEBUGPANIC();
        }

      if (g_lp_stat.pm_lock_released == 0)
        {
#ifdef CONFIG_PM_ENABLE
          esp32s3_pm_lockrelease();
#endif
          g_lp_stat.pm_lock_released = 1;
        }
    }
}

/****************************************************************************
 * Name: btdm_sleep_exit_phase3_wrapper
 *
 * Description:
 *   ESP32-S3 BLE lightsleep callback function..
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btdm_sleep_exit_phase3_wrapper(void)
{
#ifdef CONFIG_PM_ENABLE
  if (g_lp_stat.pm_lock_released)
    {
      esp32s3_pm_lockacquire();
      g_lp_stat.pm_lock_released = 0;
    }
#endif

  if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1)
    {
      if (g_lp_stat.phy_enabled == 0)
        {
          esp_phy_enable();
          g_lp_stat.phy_enabled = 1;
        }
    }

  if (g_lp_cntl.wakeup_timer_required && g_lp_stat.wakeup_timer_started)
    {
      esp_timer_stop(g_btdm_slp_tmr);
      g_lp_stat.wakeup_timer_started = 0;
    }

  while (btdm_sleep_clock_sync());
}

/****************************************************************************
 * Name: coex_wifi_sleep_set_hook
 *
 * Description:
 *   Set Wi-Fi/BT coexistence sleep.
 *
 * Input Parameters:
 *   sleep - True to set sleep, false otherwise.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_wifi_sleep_set_hook(bool sleep)
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
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  coex_schm_status_bit_set(type, status);
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
#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  coex_schm_status_bit_clear(type, status);
#endif
}

/****************************************************************************
 * Name: interrupt_on_wrapper
 *
 * Description:
 *   Enable Wi-Fi interrupt
 *
 * Input Parameters:
 *   intr_num - The interrupt CPU number.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void interrupt_on_wrapper(int intr_num)
{
  int cpuint = intr_num;
  int irq = esp32s3_getirq(0, cpuint);

  DEBUGVERIFY(esp32s3_irq_set_iram_isr(irq));

  up_enable_irq(irq);
}

/****************************************************************************
 * Name: interrupt_off_wrapper
 *
 * Description:
 *   Disable Wi-Fi interrupt
 *
 * Input Parameters:
 *   intr_num - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void interrupt_off_wrapper(int intr_num)
{
  up_disable_irq(intr_num + XTENSA_IRQ_FIRSTPERIPH);
}

/****************************************************************************
 * Name: btdm_hw_mac_power_down_wrapper
 *
 * Description:
 *   Power down MAC and baseband of Wi-Fi and Bluetooth when PHY is disabled
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void IRAM_ATTR btdm_hw_mac_power_down_wrapper(void)
{
}

/****************************************************************************
 * Name: btdm_hw_mac_power_up_wrapper
 *
 * Description:
 *   Power up MAC and baseband of Wi-Fi and Bluetooth when PHY is disabled
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void IRAM_ATTR btdm_hw_mac_power_up_wrapper(void)
{
}

/****************************************************************************
 * Name: btdm_backup_dma_copy_wrapper
 *
 * Description:
 *   Copy btdm backup DMA when PHY is disabled
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void IRAM_ATTR btdm_backup_dma_copy_wrapper(uint32_t reg, uint32_t mem_addr,
                                            uint32_t num,  bool to_mem)
{
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

#ifdef CONFIG_ESP32S3_SPIFLASH
  if (!spi_flash_cache_enabled())
    {
      esp_send_queuecache(&g_esp_queuecache, item, mq_adpt->msgsize);
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
 * Name: btdm_slp_tmr_callback
 *
 * Description:
 *   ESP-S3 BLE sleep callback function.
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
#ifdef CONFIG_PM
  btdm_vnd_offload_post(BTDM_VND_OL_SIG_WAKEUP_TMR,
                        (void *)BTDM_ASYNC_WAKEUP_SRC_TMR);
#endif
}

/****************************************************************************
 * Name: esp_int_adpt_cb
 *
 * Description:
 *   BT interrupt adapter callback function
 *
 * Input Parameters:
 *   arg - interrupt adapter private data
 *
 * Returned Value:
 *   NuttX error code
 *
 ****************************************************************************/

static int IRAM_ATTR esp_int_adpt_cb(int irq, void *context, void *arg)
{
  struct irq_adpt_s *adapter = (struct irq_adpt_s *)arg;

  adapter->func(adapter->arg);

  return OK;
}

/****************************************************************************
 * Name: btdm_sleep_exit_phase0
 *
 * Description:
 *   Acquire PM lock and stop esp timer.
 *
 * Input Parameters:
 *   param - wakeup event
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR btdm_sleep_exit_phase0(void *param)
{
  DEBUGASSERT(g_lp_cntl.enable);

#ifdef CONFIG_PM
  if (g_lp_stat.pm_lock_released)
    {
      esp32s3_pm_lockacquire();
      g_lp_stat.pm_lock_released = 0;
    }
#endif

  int event = (int)param;

  if (event == BTDM_ASYNC_WAKEUP_SRC_VHCI ||
      event == BTDM_ASYNC_WAKEUP_SRC_DISA)
    {
      btdm_wakeup_request();
    }

  if (g_lp_cntl.wakeup_timer_required && g_lp_stat.wakeup_timer_started)
    {
      esp_timer_stop(g_btdm_slp_tmr);
      g_lp_stat.wakeup_timer_started = 0;
    }

  if (event == BTDM_ASYNC_WAKEUP_SRC_VHCI ||
      event == BTDM_ASYNC_WAKEUP_SRC_DISA)
    {
      semphr_give_wrapper(g_wakeup_req_sem);
    }
}

#if CONFIG_MAC_BB_PD

/****************************************************************************
 * Name: btdm_mac_bb_power_down_cb
 *
 * Description:
 *   This function is a callback that powers down the MAC and baseband (BB)
 *   of the Bluetooth module. It first checks if the power down control for
 *   the MAC and BB is enabled and if they are not already powered down. If
 *   these conditions are met, it powers down the DMA and sets the MAC and BB
 *   as powered down.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR btdm_mac_bb_power_down_cb(void)
{
  if (g_lp_cntl.mac_bb_pd && g_lp_stat.mac_bb_pd == 0)
    {
      btdm_ble_power_down_dma_copy(true);
      g_lp_stat.mac_bb_pd = 1;
    }
}

/****************************************************************************
 * Name: btdm_mac_bb_power_up_cb
 *
 * Description:
 *   This function is a callback that powers up the MAC and baseband (BB)
 *   of the Bluetooth module. It first checks if the power down control for
 *   the MAC and BB is enabled and if they are currently powered down. If
 *   these conditions are met, it powers up the DMA and sets the MAC and BB
 *   as powered up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR btdm_mac_bb_power_up_cb(void)
{
  if (g_lp_cntl.mac_bb_pd && g_lp_stat.mac_bb_pd)
    {
      btdm_ble_power_down_dma_copy(false);
      g_lp_stat.mac_bb_pd = 0;
    }
}
#endif

/****************************************************************************
 * Name: btdm_controller_mem_init
 *
 * Description:
 *   Initialize BT controller to allocate task and other resource.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void btdm_controller_mem_init(void)
{
  extern void btdm_controller_rom_data_init(void);
  btdm_controller_rom_data_init();
}

/****************************************************************************
 * Name: bt_controller_deinit_internal
 *
 * Description:
 *   Deinit BT internal controller.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void bt_controller_deinit_internal(void)
{
  periph_module_disable(PERIPH_BT_MODULE);

  if (g_lp_stat.phy_enabled)
    {
      esp_phy_disable();
      g_lp_stat.phy_enabled = 0;
    }

  /* deinit low power control resources */

#if CONFIG_MAC_BB_PD
  if (g_lp_cntl.mac_bb_pd)
    {
      btdm_deep_sleep_mem_deinit();
      g_lp_cntl.mac_bb_pd = 0;
    }
#endif

#ifdef CONFIG_PM
  g_lp_stat.pm_lock_released = 0;
#endif

  if (g_lp_cntl.wakeup_timer_required)
    {
      if (g_lp_stat.wakeup_timer_started)
        {
          esp_timer_stop(g_btdm_slp_tmr);
        }

      g_lp_stat.wakeup_timer_started = 0;
      esp_timer_delete(g_btdm_slp_tmr);
      g_btdm_slp_tmr = NULL;
    }

  if (g_lp_cntl.enable)
    {
      btdm_vnd_offload_task_deregister(BTDM_VND_OL_SIG_WAKEUP_TMR);
      if (g_wakeup_req_sem != NULL)
        {
          semphr_delete_wrapper(g_wakeup_req_sem);
          g_wakeup_req_sem = NULL;
        }
    }

  if (g_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL)
    {
#ifdef CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
      if (g_lp_cntl.main_xtal_pu)
        {
          esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
          g_lp_cntl.main_xtal_pu = 0;
        }
#endif

      btdm_lpclk_select_src(BTDM_LPCLK_SEL_RTC_SLOW);
      btdm_lpclk_set_div(0);
#if CONFIG_ESP32S3_WIFI_BT_COEXIST
      coex_update_lpclk_interval();
#endif
    }

  g_btdm_lpcycle_us = 0;

#if CONFIG_MAC_BB_PD
  esp_unregister_mac_bb_pd_callback(btdm_mac_bb_power_down_cb);
  esp_unregister_mac_bb_pu_callback(btdm_mac_bb_power_up_cb);
#endif

  esp_bt_power_domain_off();
#if CONFIG_MAC_BB_PD
  esp_mac_bb_pd_mem_deinit();
#endif
  esp_phy_modem_deinit();

  if (osi_funcs_p != NULL)
    {
      kmm_free(osi_funcs_p);
      osi_funcs_p = NULL;
    }

    g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;
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
  bool do_wakeup_request = false;

  if (g_lp_cntl.enable == 0)
    {
      return false;
    }

  switch (event)
    {
      case BTDM_ASYNC_WAKEUP_SRC_VHCI:
      case BTDM_ASYNC_WAKEUP_SRC_DISA:
        btdm_in_wakeup_requesting_set(true);

        if (btdm_power_state_active() == false)
          {
            r_btdm_vnd_offload_post(BTDM_VND_OL_SIG_WAKEUP_TMR,
                                    (void *)event);
            do_wakeup_request = true;
            semphr_take_wrapper(g_wakeup_req_sem, OSI_FUNCS_TIME_BLOCKING);
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
  bool allow_to_sleep;

  if (g_lp_cntl.enable == 0)
    {
      return;
    }

  switch (event)
    {
      case BTDM_ASYNC_WAKEUP_SRC_VHCI:
      case BTDM_ASYNC_WAKEUP_SRC_DISA:
        allow_to_sleep = true;
        break;
      default:
        allow_to_sleep = true;
        break;
    }

  if (allow_to_sleep)
    {
      btdm_in_wakeup_requesting_set(false);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_bt_controller_init
 *
 * Description:
 *   Init  BT controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s3_bt_controller_init(void)
{
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_config_t *cfg = &bt_cfg;
  bool select_src_ret;
  bool set_div_ret;
  int i;
  int err;

  sq_init(&g_ble_int_flags_free);
  sq_init(&g_ble_int_flags_used);

  for (i = 0; i < NR_IRQSTATE_FLAGS; i++)
    {
      sq_addlast((sq_entry_t *)&g_ble_int_flags[i], &g_ble_int_flags_free);
    }

  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE)
    {
      wlerr("Invalid controller status");
      return ERROR;
    }

  if (cfg == NULL)
    {
      return -EINVAL;
    }

  if (cfg->controller_task_prio != ESP_TASK_BT_CONTROLLER_PRIO ||
      cfg->controller_task_stack_size < ESP_TASK_BT_CONTROLLER_STACK)
    {
      wlerr("Invalid controller task prioriy or stack size");
      return ERROR;
    }

  if (cfg->bluetooth_mode != ESP_BT_MODE_BLE)
    {
      wlerr("Controller only support BLE only mode");
      return ERROR;
    }

  if (cfg->bluetooth_mode & ESP_BT_MODE_BLE)
    {
      if ((cfg->ble_max_act <= 0) ||
          (cfg->ble_max_act > BT_CTRL_BLE_MAX_ACT_LIMIT))
        {
          wlerr("Invalid value of ble_max_act");
          return ERROR;
        }
    }

  if (cfg->sleep_mode == ESP_BT_SLEEP_MODE_1)
    {
      if (cfg->sleep_clock == ESP_BT_SLEEP_CLOCK_NONE)
        {
          wlerr("SLEEP_MODE_1 enabled but sleep clock not configured");
          return ERROR;
        }
    }

  cfg->controller_task_stack_size = CONFIG_ESP32S3_BLE_TASK_STACK_SIZE;
  cfg->controller_task_prio       = CONFIG_ESP32S3_BLE_TASK_PRIORITY;
  cfg->controller_task_run_cpu    = CONFIG_BT_CTRL_PINNED_TO_CORE;
  cfg->magic                      = ESP_BT_CTRL_CONFIG_MAGIC_VAL;

#if CONFIG_MAC_BB_PD
  esp_mac_bb_pd_mem_init();
#endif

  esp_phy_modem_init();

  esp_bt_power_domain_on();

  btdm_controller_mem_init();

#if CONFIG_MAC_BB_PD
  if (esp_register_mac_bb_pd_callback(btdm_mac_bb_power_down_cb) != 0)
    {
      err = -EINVAL;
      goto error;
    }

  if (esp_register_mac_bb_pu_callback(btdm_mac_bb_power_up_cb) != 0)
    {
      err = -EINVAL;
      goto error;
    }
#endif

  osi_funcs_p = kmm_malloc(sizeof(struct osi_funcs_s));
  if (osi_funcs_p == NULL)
    {
      return -ENOMEM;
    }

  memcpy(osi_funcs_p, &g_osi_funcs, sizeof(struct osi_funcs_s));
  if (btdm_osi_funcs_register(osi_funcs_p) != 0)
    {
      wlerr("Error, probably invalid OSI Functions\n");
      return -EINVAL;
    }

  wlinfo("BT controller compile version [%s]\n",
         btdm_controller_get_compile_version());

  /* init low-power control resources */

  do
    {
      /* set default values for global states or resources */

      g_lp_cntl.val = 0;
      g_lp_stat.val = 0;
      g_wakeup_req_sem = NULL;
      g_btdm_slp_tmr = NULL;

      /* configure and initialize resources */

      g_lp_cntl.enable = (cfg->sleep_mode == ESP_BT_SLEEP_MODE_1) ? 1 : 0;
      g_lp_cntl.no_light_sleep = 0;

      if (g_lp_cntl.enable)
        {
#if CONFIG_MAC_BB_PD
          if (!btdm_deep_sleep_mem_init())
            {
              err = -ENOMEM;
              goto error;
            }

          g_lp_cntl.mac_bb_pd = 1;
#endif
#ifdef CONFIG_PM
          g_lp_cntl.wakeup_timer_required = true;
#endif
          /* async wakeup semaphore for VHCI */

          g_wakeup_req_sem = semphr_create_wrapper(1, 0);
          if (g_wakeup_req_sem == NULL)
            {
              goto error;
            }

          btdm_vnd_offload_task_register(BTDM_VND_OL_SIG_WAKEUP_TMR,
                                         btdm_sleep_exit_phase0);
        }

      if (g_lp_cntl.wakeup_timer_required)
        {
          esp_timer_create_args_t create_args =
            {
              .callback = btdm_slp_tmr_callback,
              .arg = NULL,
              .name = "btSlp",
            };

          if (esp_timer_create(&create_args, &g_btdm_slp_tmr) != OK)
            {
              wlerr("Failed to create timer");
              goto error;
            }
        }

      /* set default bluetooth sleep clock cycle and its fractional bits */

      g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
      g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

      /* set default bluetooth sleep clock source */

      g_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_XTAL; /* set default value */

#if CONFIG_BT_CTRL_LPCLK_SEL_EXT_32K_XTAL
      /* check whether or not EXT_CRYS is working */

      if (esp32s3_rtc_clk_slow_freq_get() == RTC_SLOW_FREQ_32K_XTAL) /* External 32 kHz XTAL */
        {
          g_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_XTAL32K;
        }
      else
        {
          wlwarn("32.768kHz XTAL not detected");
#if !CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
          g_lp_cntl.no_light_sleep = 1;
#endif
        }
#elif (CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL)
      wlwarn("Bluetooth will use main XTAL as Bluetooth sleep clock.");
#if !CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
      g_lp_cntl.no_light_sleep = 1;
#endif
#elif (CONFIG_BT_CTRL_LPCLK_SEL_RTC_SLOW)
      /* check whether or not internal 150 kHz RC oscillator is working */

      if (rtc_clk_slow_src_get() == RTC_SLOW_FREQ_RTC)
        {
          g_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_RTC_SLOW; /* Internal 150 kHz RC oscillator */
          wlwarn("Internal 150kHz RC osciallator. The accuracy of this "
                 "clock is a lot larger than 500ppm which is required in "
                 "Bluetooth communication, so don't select this option in "
                 "scenarios such as BLE connection state.");
        }
      else
        {
          wlerr("Internal 150kHz RC oscillator not detected.");
          PANIC();
        }
#endif

      if (g_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL)
        {
          uint32_t rtc_clk_xtal_freq = 0;
#ifdef CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
          ASSERT(esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON));
          g_lp_cntl.main_xtal_pu = 1;
#endif
          select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL);
          rtc_clk_xtal_freq = esp32s3_rtc_clk_xtal_freq_get();
          set_div_ret = btdm_lpclk_set_div(rtc_clk_xtal_freq);
          DEBUGASSERT(select_src_ret && set_div_ret);
          g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
          g_btdm_lpcycle_us = 1 << (g_btdm_lpcycle_us_frac);
        }
      else if (g_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL32K)
        {
          select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL32K);
          set_div_ret = btdm_lpclk_set_div(0);
          DEBUGASSERT(select_src_ret && set_div_ret);
          g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
          g_btdm_lpcycle_us = (RTC_CLK_CAL_FRACT > 15) ?
              (1000000 << (RTC_CLK_CAL_FRACT - 15)) :
              (1000000 >> (15 - RTC_CLK_CAL_FRACT));
          DEBUGASSERT(g_btdm_lpcycle_us != 0);
        }
      else if (g_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_RTC_SLOW)
        {
          select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_RTC_SLOW);
          set_div_ret = btdm_lpclk_set_div(0);
          DEBUGASSERT(select_src_ret && set_div_ret);
          g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
          g_btdm_lpcycle_us = esp32s3_clk_slowclk_cal_get();
        }
      else
        {
          UNUSED(select_src_ret);
          UNUSED(set_div_ret);
          goto error;
        }

#if CONFIG_ESP32S3_WIFI_BT_COEXIST
      coex_update_lpclk_interval();
#endif

#ifdef CONFIG_PM
      g_lp_stat.pm_lock_released = 1;
#endif
    }
  while (0);

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  coex_init();
#endif

  periph_module_enable(PERIPH_BT_MODULE);
  periph_module_reset(PERIPH_BT_MODULE);

  esp_phy_enable();
  g_lp_stat.phy_enabled = 1;

  if (btdm_controller_init(cfg) != 0)
    {
      esp_phy_disable();
      g_lp_stat.phy_enabled = 0;
      return -EIO;
    }

  coex_pti_v2();

  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

#ifdef CONFIG_ESP32S3_SPIFLASH
  if (esp_wireless_init() != OK)
    {
      return -EIO;
    }
#endif

  return OK;

error:

  bt_controller_deinit_internal ();

  return -ENOMEM;
}

/****************************************************************************
 * Name: esp32s3_bt_controller_deinit
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

int esp32s3_bt_controller_deinit(void)
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
 * Name: esp32s3_bt_controller_enable
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

int esp32s3_bt_controller_enable(esp_bt_mode_t mode)
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

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  coex_enable();
#endif

#ifdef CONFIG_PM
  /* enable low power mode */

  if (g_lp_cntl.no_light_sleep)
    {
      esp32s3_pm_lockacquire();
    }

  esp32s3_pm_lockacquire();
  g_lp_stat.pm_lock_released = 0;
#endif

  if (g_lp_cntl.enable)
    {
        btdm_controller_enable_sleep(true);
    }

  if (btdm_controller_enable(mode) != 0)
    {
      ret = ERROR;
      goto error;
    }

  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;

  return ret;

error:

  /* disable low power mode */

  btdm_controller_enable_sleep(false);

#ifdef CONFIG_PM
  if (g_lp_cntl.no_light_sleep)
    {
      esp32s3_pm_lockrelease();
    }

  if (g_lp_stat.pm_lock_released == 0)
    {
      esp32s3_pm_lockrelease();
      g_lp_stat.pm_lock_released = 1;
    }
#endif

#if CONFIG_ESP32S3_WIFI_BT_COEXIST
    coex_disable();
#endif

  return ret;
}

/****************************************************************************
 * Name: esp32s3_bt_controller_disable
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

int esp32s3_bt_controller_disable(void)
{
  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return ERROR;
    }

  async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_DISA);
  while (!btdm_power_state_active())
    {
      nxsig_usleep(1000); /* wait */
    }

  btdm_controller_disable();

  async_wakeup_request_end(BTDM_ASYNC_WAKEUP_SRC_DISA);

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  coex_disable();
#endif

  g_btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

#ifdef CONFIG_PM
  /* disable low power mode */

  if (g_lp_cntl.no_light_sleep)
    {
      esp_pm_lock_release(s_light_sleep_pm_lock);
    }

  if (g_lp_stat.pm_lock_released == 0)
    {
      esp32s3_pm_lockrelease();
      g_lp_stat.pm_lock_released = 1;
    }
  else
    {
      DEBUGPANIC();
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32s3_bt_controller_get_status
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

esp_bt_controller_status_t esp32s3_bt_controller_get_status(void)
{
  return g_btdm_controller_status;
}

/****************************************************************************
 * Name: esp32s3_vhci_host_check_send_available
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

bool esp32s3_vhci_host_check_send_available(void)
{
  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return false;
    }

  return api_vhci_host_check_send_available();
}

/****************************************************************************
 * Name: esp32s3_vhci_host_send_packet
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

void esp32s3_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return;
    }

  async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_VHCI);

  api_vhci_host_send_packet(data, len);

  async_wakeup_request_end(BTDM_ASYNC_WAKEUP_SRC_VHCI);
}

/****************************************************************************
 * Name: esp32s3_vhci_register_callback
 *
 * Description:
 *   Register the vhci reference callback.
 *
 * Input Parameters:
 *   callback - struct defined by vhci_host_callback structure.
 *
 * Returned Value:
 *   status - success or fail
 *
 ****************************************************************************/

int esp32s3_vhci_register_callback(const esp_vhci_host_callback_t *callback)
{
  int ret = ERROR;
  if (g_btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return ret;
    }

  ret = api_vhci_host_register_callback(
            (const vhci_host_callback_t *)callback) == 0 ? 0 : -1;
  return ret;
}
