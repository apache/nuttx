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
#include "espidf_wifi.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32_rt_timer.h"
#include "esp32_ble_adapter.h"

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
#  include "esp_coexist_internal.h"
#  include "esp_coexist_adapter.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef void (*xt_handler)(void *);
typedef void (*coex_func_cb_t)(uint32_t event, int sched_cnt);

#define XTHAL_SET_INTSET(v) \
do {\
  int __interrupt = (int)(v);\
  __asm__ __volatile__("wsr.intset %0" :: "a"(__interrupt):"memory");\
} while(0)

#define MSG_QUEUE_NAME_SIZE    16

#define ESP_ERR_INVALID_STATE       0x103

#define OSI_FUNCS_TIME_BLOCKING          0xffffffff
#define OSI_VERSION                      0x00010002
#define OSI_MAGIC_VALUE                  0xfadebead

#define BTDM_ASYNC_WAKEUP_REQ_HCI   0
#define BTDM_ASYNC_WAKEUP_REQ_COEX  1
#define BTDM_ASYNC_WAKEUP_REQMAX    2

#ifdef CONFIG_PM
#define BTDM_MIN_TIMER_UNCERTAINTY_US    (1800)

/* Low Power Clock Selection */

#define BTDM_LPCLK_SEL_XTAL              (0)
#define BTDM_LPCLK_SEL_XTAL32K           (1)
#define BTDM_LPCLK_SEL_RTC_SLOW          (2)
#define BTDM_LPCLK_SEL_8M                (3)

/* Sleep and wakeup interval control */

#define BTDM_MIN_SLEEP_DURATION          (24) /* Threshold of interval in half slots to allow to fall into sleep mode */
#define BTDM_MODEM_WAKE_UP_DELAY         (8)  /* delay in half slots of modem wake up procedure, including re-enable PHY/RF */
#endif

#define BTDM_MODEM_SLEEP_MODE_NONE 0

#define ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL  0x20200622

extern void btdm_controller_set_sleep_mode(uint8_t mode);

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
extern void coex_pti_v2(void);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Number of fractional bits in values returned by rtc_clk_cal */

#define RTC_CLK_CAL_FRACT  19

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

/* BLE low power control struct */

typedef struct btdm_lpcntl_s
{
  bool enable;                  /* whether low power mode is required */
  bool lpclk_sel;               /* low power clock source */
  bool mac_bb_pd;               /* whether hardware(MAC, BB) force-power-down is required during sleep */
  bool wakeup_timer_required;   /* whether system timer is needed */
  bool no_light_sleep;          /* do not allow system to enter light sleep after bluetooth is enabled */
} btdm_lpcntl_t;

/* low power control status */

typedef struct btdm_lpstat_s
{
  bool pm_lock_released;        /* whether power management lock is released */
  bool mac_bb_pd;               /* whether hardware(MAC, BB) is powered down */
  bool phy_enabled;             /* whether phy is switched on */
  bool wakeup_timer_started;    /* whether wakeup timer is started */
} btdm_lpstat_t;

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

typedef enum
{
  PERIPH_LEDC_MODULE = 0,
  PERIPH_UART0_MODULE,
  PERIPH_UART1_MODULE,
  PERIPH_UART2_MODULE,
  PERIPH_I2C0_MODULE,
  PERIPH_I2C1_MODULE,
  PERIPH_I2S0_MODULE,
  PERIPH_I2S1_MODULE,
  PERIPH_TIMG0_MODULE,
  PERIPH_TIMG1_MODULE,
  PERIPH_PWM0_MODULE,
  PERIPH_PWM1_MODULE,
  PERIPH_PWM2_MODULE,
  PERIPH_PWM3_MODULE,
  PERIPH_UHCI0_MODULE,
  PERIPH_UHCI1_MODULE,
  PERIPH_RMT_MODULE,
  PERIPH_PCNT_MODULE,
  PERIPH_SPI_MODULE,
  PERIPH_HSPI_MODULE,
  PERIPH_VSPI_MODULE,
  PERIPH_SPI_DMA_MODULE,
  PERIPH_SDMMC_MODULE,
  PERIPH_SDIO_SLAVE_MODULE,
  PERIPH_CAN_MODULE,
  PERIPH_EMAC_MODULE,
  PERIPH_RNG_MODULE,
  PERIPH_WIFI_MODULE,
  PERIPH_BT_MODULE,
  PERIPH_WIFI_BT_COMMON_MODULE,
  PERIPH_BT_BASEBAND_MODULE,
  PERIPH_BT_LC_MODULE,
  PERIPH_AES_MODULE,
  PERIPH_SHA_MODULE,
  PERIPH_RSA_MODULE,
} periph_module_e;

/* prototype of function to handle vendor dependent signals */

typedef void (*btdm_vnd_ol_task_func_t)(void *param);

/* VHCI function interface */

typedef struct vhci_host_callback_s
{
  void (*notify_host_send_available)(void);               /* callback used to notify that the host can send packet to controller */
  int (*notify_host_recv)(uint8_t *data, uint16_t len);   /* callback used to notify that the controller has a packet to send to the host */
} vhci_host_callback_t;

/* DRAM region */

typedef struct btdm_dram_available_region_s
{
  esp_bt_mode_t mode;
  intptr_t start;
  intptr_t end;
} btdm_dram_available_region_t;

typedef void (*osi_intr_handler)(void);

/* BLE OS function */

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
    void *(* _malloc)(uint32_t size);
    void *(* _malloc_internal)(uint32_t size);
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
    uint32_t _magic;
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

static xt_handler esp_ble_set_isr(int n, xt_handler f, void *arg);
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
static int mutex_lock_wrapper(void *mutex);
static int mutex_unlock_wrapper(void *mutex);
static int IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item,
                                                 void *hptw);
static int IRAM_ATTR queue_recv_from_isr_wrapper(void *queue, void *item,
                                                 void *hptw);
static int task_create_wrapper(void *task_func, const char *name,
                               uint32_t stack_depth, void *param,
                               uint32_t prio, void *task_handle,
                               uint32_t core_id);
static void task_delete_wrapper(void *task_handle);
static bool IRAM_ATTR is_in_isr_wrapper(void);
static void *malloc_wrapper(size_t size);
static void IRAM_ATTR cause_sw_intr(void *arg);
static int IRAM_ATTR cause_sw_intr_to_core_wrapper(int core_id, int intr_no);
static void *malloc_internal_wrapper(size_t size);
static int IRAM_ATTR read_mac_wrapper(uint8_t mac[6]);
static void IRAM_ATTR srand_wrapper(unsigned int seed);
static int IRAM_ATTR rand_wrapper(void);
static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles);
static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static int queue_send_wrapper(void *queue, void *item,
                              uint32_t block_time_ms);
static int queue_recv_wrapper(void *queue, void *item,
                              uint32_t block_time_ms);
static void queue_delete_wrapper(void *queue);
static void esp32_ints_on(uint32_t mask);
static int adapter_coex_register_bt_cb_wrapper(coex_func_cb_t cb);
static int adapter_coex_schm_register_btdm_callback(void *callback);
static int adapter_coex_register_wifi_channel_change_callback(void *cb);
static int adapter_coex_wifi_channel_get(uint8_t *primary,
                                         uint8_t *secondary);
static void adapter_coex_schm_status_bit_clear(uint32_t type,
                                               uint32_t status);
static void adapter_coex_schm_status_bit_set(uint32_t type, uint32_t status);
static uint32_t adapter_coex_schm_interval_get(void);
static uint8_t adapter_coex_schm_curr_period_get(void);
static void *adapter_coex_schm_curr_phase_get(void);

#ifdef CONFIG_PM
static bool IRAM_ATTR btdm_sleep_check_duration(int32_t *half_slot_cnt);
static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles);
static void btdm_sleep_enter_phase2_wrapper(void);
static void btdm_sleep_exit_phase3_wrapper(void);
#endif

static bool coex_bt_wakeup_request(void);
static void coex_bt_wakeup_request_end(void);
static int esp_int_adpt_cb(int irq, void *context, void *arg);

/****************************************************************************
 * Extern Functions declaration and value
 ****************************************************************************/

extern int btdm_osi_funcs_register(void *osi_funcs);
extern void btdm_controller_rom_data_init(void);
extern void coex_bt_high_prio(void);

/* Initialise and De-initialise */

extern int btdm_controller_init(uint32_t config_mask,
                                esp_bt_controller_config_t *config_opts);
extern void btdm_controller_deinit(void);
extern int btdm_controller_enable(esp_bt_mode_t mode);
extern void btdm_controller_disable(void);
extern uint8_t btdm_controller_get_mode(void);
extern const char *btdm_controller_get_compile_version(void);
extern void btdm_rf_bb_init_phase2(void); /* shall be called after PHY/RF is enabled */

/* Sleep */

extern void btdm_controller_enable_sleep(bool enable);
extern void btdm_controller_set_sleep_mode(uint8_t mode);
extern uint8_t btdm_controller_get_sleep_mode(void);
extern bool btdm_power_state_active(void);
extern void btdm_wakeup_request(bool request_lock);
extern void btdm_wakeup_request_end(void);

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
extern int bredr_txpwr_set(int min_power_level, int max_power_level);
extern int bredr_txpwr_get(int *min_power_level, int *max_power_level);
extern void bredr_sco_datapath_set(uint8_t data_path);
extern void btdm_controller_scan_duplicate_list_clear(void);

/* Coexistence */

int coex_bt_request_wrapper(uint32_t event,
                            uint32_t latency,
                            uint32_t duration);
int coex_bt_release_wrapper(uint32_t event);
uint32_t coex_bb_reset_lock_wrapper(void);
void coex_bb_reset_unlock_wrapper(uint32_t restore);
extern void coex_ble_adv_priority_high_set(bool high);

extern char _bss_start_btdm;
extern char _bss_end_btdm;
extern char _data_start_btdm;
extern char _data_end_btdm;
extern uint32_t _data_start_btdm_rom;
extern uint32_t _data_end_btdm_rom;

extern uint32_t _bt_bss_start;
extern uint32_t _bt_bss_end;
extern uint32_t _btdm_bss_start;
extern uint32_t _btdm_bss_end;
extern uint32_t _bt_data_start;
extern uint32_t _bt_data_end;
extern uint32_t _btdm_data_start;
extern uint32_t _btdm_data_end;

extern char _bt_tmp_bss_start;
extern char _bt_tmp_bss_end;

void intr_matrix_set(int cpu_no, uint32_t model_num, uint32_t intr_num);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Controller status */

static DRAM_ATTR esp_bt_controller_status_t btdm_controller_status =
                        ESP_BT_CONTROLLER_STATUS_IDLE;

/* low power control struct */

static DRAM_ATTR btdm_lpcntl_t g_lp_cntl;

/* low power status struct */

static DRAM_ATTR btdm_lpstat_t g_lp_stat;

/* measured average low power clock period in micro seconds */

static DRAM_ATTR uint32_t g_btdm_lpcycle_us = 0;

/* number of fractional bit for g_btdm_lpcycle_us */

static DRAM_ATTR uint8_t g_btdm_lpcycle_us_frac = 0;

#ifdef CONFIG_PM
/* semaphore used for blocking VHCI API to wait for controller to wake up */

static DRAM_ATTR void * g_wakeup_req_sem  = NULL;

/* wakeup timer */

static DRAM_ATTR esp_timer_handle_t g_btdm_slp_tmr;
#endif

/* BT interrupt private data */

static irqstate_t g_inter_flags;
static uint32_t g_phy_clk_en_cnt;
static int64_t g_phy_rf_en_ts;
static uint8_t g_phy_access_ref;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* BLE OS adapter data */

static struct osi_funcs_s g_osi_funcs =
{
  ._magic = OSI_MAGIC_VALUE,
  ._version = OSI_VERSION,
  ._set_isr = esp_ble_set_isr,
  ._ints_on = esp32_ints_on,
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
#ifdef CONFIG_PM
  ._btdm_sleep_check_duration = btdm_sleep_check_duration,
  ._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
  ._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
  ._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
#endif
  ._coex_bt_wakeup_request = coex_bt_wakeup_request,
  ._coex_bt_wakeup_request_end = coex_bt_wakeup_request_end,
  ._coex_bt_request = coex_bt_request_wrapper,
  ._coex_bt_release = coex_bt_release_wrapper,
  ._coex_register_bt_cb = adapter_coex_register_bt_cb_wrapper,
  ._coex_register_wifi_channel_change_callback =
                 adapter_coex_register_wifi_channel_change_callback,
  ._coex_wifi_channel_get = adapter_coex_wifi_channel_get,
  ._coex_schm_status_bit_clear = adapter_coex_schm_status_bit_clear,
  ._coex_schm_status_bit_set = adapter_coex_schm_status_bit_set,
  ._coex_schm_interval_get = adapter_coex_schm_interval_get,
  ._coex_schm_curr_period_get = adapter_coex_schm_curr_period_get,
  ._coex_schm_curr_phase_get = adapter_coex_schm_curr_phase_get,
  ._coex_schm_register_btdm_callback =
                         adapter_coex_schm_register_btdm_callback,
  ._coex_bb_reset_lock = coex_bb_reset_lock_wrapper,
  ._coex_bb_reset_unlock = coex_bb_reset_unlock_wrapper,
};

/* The mode column will be modified by release function to indicate the
 * available region
 */

static btdm_dram_available_region_t btdm_dram_available_region[] =
{
  /* following is .data */

  {
    ESP_BT_MODE_BTDM,
    SOC_MEM_BT_DATA_START,
    SOC_MEM_BT_DATA_END
  },

  /* following is memory which HW will use */

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

  /* following is .bss */

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

/****************************************************************************
 * Private Functions and Public Functions only used by libraries
 ****************************************************************************/

static int adapter_coex_register_bt_cb_wrapper(coex_func_cb_t cb)
{
  return ESP_ERR_INVALID_STATE;
}

static int adapter_coex_schm_register_btdm_callback(void *callback)
{
  return ESP_ERR_INVALID_STATE;
}

static int adapter_coex_register_wifi_channel_change_callback(void *cb)
{
  return ESP_ERR_INVALID_STATE;
}

static int adapter_coex_wifi_channel_get(uint8_t *primary,
                                         uint8_t *secondary)
{
  return -ERROR;
}

static void adapter_coex_schm_status_bit_clear(uint32_t type,
                                               uint32_t status)
{
}

static void adapter_coex_schm_status_bit_set(uint32_t type, uint32_t status)
{
}

static uint32_t adapter_coex_schm_interval_get(void)
{
  return ESP_ERR_INVALID_STATE;
}

static uint8_t adapter_coex_schm_curr_period_get(void)
{
  return OK;
}

static void *adapter_coex_schm_curr_phase_get(void)
{
  return NULL;
}

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

  return pid > 0 ? true : false;
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

static xt_handler esp_ble_set_isr(int n, xt_handler f, void *arg)
{
  int ret;
  uint32_t tmp;
  struct irq_adpt_s *adapter;
  int irq = n + XTENSA_IRQ_FIRSTPERIPH;

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
      assert(0);
      return NULL;
    }

  adapter->func = f;
  adapter->arg = arg;

  ret = irq_attach(irq, esp_int_adpt_cb, adapter);
  if (ret)
    {
      wlerr("Failed to attach IRQ %d\n", irq);
      assert(0);
      return NULL;
    }

  return NULL;
}

/****************************************************************************
 * Name: esp32_ints_on
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

static void esp32_ints_on(uint32_t mask)
{
  uint32_t bit;

  for (int i = 0; i < 32; i++)
    {
      bit = 1 << i;
      if (bit & mask)
      {
        wlinfo("Enabled bit %d\n", i);
        up_enable_irq(i);
      }
    }
}

/****************************************************************************
 * Name: is_wifi_clk_peripheral
 *
 * Description:
 *     Checks if the peripheral module needs WiFi Clock.
 *
 * Input Parameters:
 *     periph - The peripheral module
 *
 * Returned Value:
 *    true if it depends on WiFi clock or false otherwise.
 *
 ****************************************************************************/

static bool is_wifi_clk_peripheral(periph_module_e periph)
{
  /* A small subset of peripherals use WIFI_CLK_EN_REG and
   * CORE_RST_EN_REG for their clock & reset registers
   */

  switch (periph)
  {
    case PERIPH_SDMMC_MODULE:
    case PERIPH_SDIO_SLAVE_MODULE:
    case PERIPH_EMAC_MODULE:
    case PERIPH_RNG_MODULE:
    case PERIPH_WIFI_MODULE:
    case PERIPH_BT_MODULE:
    case PERIPH_WIFI_BT_COMMON_MODULE:
    case PERIPH_BT_BASEBAND_MODULE:
    case PERIPH_BT_LC_MODULE:
        return true;
    default:
        return false;
  }
}

/****************************************************************************
 * Name: get_clk_en_mask
 *
 * Description:
 *     Returns the WIFI_BT clock mask case it is BLE peripheral.
 *
 * Input Parameters:
 *     periph - The peripheral module
 *
 * Returned Value:
 *    The clock peripheral mask.
 *
 ****************************************************************************/

static uint32_t get_clk_en_mask(periph_module_e periph)
{
  switch (periph)
  {
    case PERIPH_BT_MODULE:
      return DPORT_WIFI_CLK_BT_EN_M;
    default:
      return 0;
  }
}

/****************************************************************************
 * Name: get_rst_en_mask
 *
 * Description:
 *     Returns the WIFI_BT reset mask
 *
 * Input Parameters:
 *     periph - The peripheral module
 *     enable - Enable/Disable
 *
 * Returned Value:
 *    The reset peripheral mask.
 *
 ****************************************************************************/

static uint32_t get_rst_en_mask(periph_module_e periph, bool enable)
{
  return 0;
}

/****************************************************************************
 * Name: get_clk_en_reg
 *
 * Description:
 *     Returns the WIFI_BT clock register
 *
 * Input Parameters:
 *     periph - The peripheral module
 *
 * Returned Value:
 *    The clock peripheral register.
 *
 ****************************************************************************/

static uint32_t get_clk_en_reg(periph_module_e periph)
{
  return is_wifi_clk_peripheral(periph) ? DPORT_WIFI_CLK_EN_REG :
                                          DPORT_PERIP_CLK_EN_REG;
}

/****************************************************************************
 * Name: get_rst_en_reg
 *
 * Description:
 *     Returns the WIFI_BT reset register
 *
 * Input Parameters:
 *     periph - The peripheral module
 *
 * Returned Value:
 *    The reset peripheral register.
 *
 ****************************************************************************/

static uint32_t get_rst_en_reg(periph_module_e periph)
{
  return is_wifi_clk_peripheral(periph) ? DPORT_CORE_RST_EN_REG :
                                          DPORT_PERIP_RST_EN_REG;
}

/****************************************************************************
 * Name: bt_periph_module_enable
 *
 * Description:
 *     Enable the bluetooth module
 *
 * Input Parameters:
 *     periph - The peripheral module
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void bt_periph_module_enable(periph_module_e periph)
{
  modifyreg32(get_clk_en_reg(periph), 0, get_clk_en_mask(periph));
  modifyreg32(get_rst_en_reg(periph), get_rst_en_mask(periph, true), 0);
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

static int esp_int_adpt_cb(int irq, void *context, void *arg)
{
  struct irq_adpt_s *adapter = (struct irq_adpt_s *)arg;

  adapter->func(adapter->arg);

  return OK;
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
  g_inter_flags = enter_critical_section();
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
  leave_critical_section(g_inter_flags);
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
  sem_t *sem = (sem_t *)semphr;
  sem_destroy(sem);
  kmm_free(sem);
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
  return esp_errno_trans(sem_trywait(semphr));
}

/****************************************************************************
 * Name: semphr_give_from_isr_wrapper
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

static int IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
  return semphr_give_wrapper(semphr);
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
  struct timespec timeout;
  sem_t *sem = (sem_t *)semphr;

  if (block_time_ms == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = sem_wait(sem);
      if (ret)
        {
          wlerr("Failed to wait sem %d\n", ret);
        }
    }
  else
    {
      ret = clock_gettime(CLOCK_REALTIME, &timeout);
      if (ret < 0)
        {
          wlerr("Failed to get time\n");
          return esp_errno_trans(ret);
        }

      if (block_time_ms)
        {
          esp_update_time(&timeout, MSEC2TICK(block_time_ms));
        }

      ret = sem_timedwait(sem, &timeout);
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
  sem_t *sem = (sem_t *)semphr;

  ret = sem_post(sem);
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
  struct mq_adpt_s *mq_adpt = (struct mq_adpt_s *)queue;

  if (ticks == OSI_FUNCS_TIME_BLOCKING || ticks == 0)
    {
      /**
       * BLE interrupt function will call this adapter function to send
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
 * Name: queue_recv_from_isr_wrapper
 *
 * Description:
 *   Receive message from queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   hptw - Unused
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int IRAM_ATTR queue_recv_from_isr_wrapper(void *queue,
                                                 void *item,
                                                 void *hptw)
{
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
                                        prio, task_handle, UINT32_MAX);
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
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static bool IRAM_ATTR is_in_isr_wrapper(void)
{
  return false;
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
 *  mac - mac address
 *
 * Returned Value:
 *   None
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
 *
 * Input Parameters:
 *  None
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
 *    Get random value.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Random value
 *
 ****************************************************************************/

static int IRAM_ATTR rand_wrapper(void)
{
  return random();
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
 *    us - value equivalent to the CPU cycles
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles)
{
  uint64_t us = (uint64_t)g_btdm_lpcycle_us * cycles;
  us = (us + (1 << (g_btdm_lpcycle_us_frac - 1))) >> g_btdm_lpcycle_us_frac;
  return (uint32_t)us;
}

/****************************************************************************
 * Name: btdm_us_2_lpcycles
 *
 * Description:
 * Converts a duration in half us into a number of low power clock cycles.
 *
 * Input Parameters:
 *  us
 *
 * Returned Value:
 *   cycles
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us)
{
  uint64_t cycles;
  cycles = ((uint64_t)(us) << g_btdm_lpcycle_us_frac) / g_btdm_lpcycle_us;
  return (uint32_t)cycles;
}

#ifdef CONFIG_PM
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
  int event = (int) param;

  DEBUGASSERT(g_lp_cntl.enable == true);

  if (g_lp_stat.pm_lock_released)
    {
      esp32_pm_lockacquire();
      g_lp_stat.pm_lock_released = false;
    }

  if (event == BTDM_ASYNC_WAKEUP_SRC_VHCI ||
      event == BTDM_ASYNC_WAKEUP_SRC_DISA)
    {
      btdm_wakeup_request();
    }

  if (g_lp_cntl.wakeup_timer_required && g_lp_stat.wakeup_timer_started)
    {
      esp_timer_stop(g_btdm_slp_tmr);
      g_lp_stat.wakeup_timer_started = false;
    }

  if (event == BTDM_ASYNC_WAKEUP_SRC_VHCI ||
      event == BTDM_ASYNC_WAKEUP_SRC_DISA)
    {
      semphr_give_wrapper(g_wakeup_req_sem);
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
  btdm_vnd_offload_post(BTDM_VND_OL_SIG_WAKEUP_TMR,
                        (void *)BTDM_ASYNC_WAKEUP_SRC_TMR);
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
      DEBUGASSERT(0);
    }
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
  if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1)
    {
      if (g_lp_stat.phy_enabled)
        {
          bt_phy_disable();
          g_lp_stat.phy_enabled = false;
        }
      else
        {
          DEBUGASSERT(0);
        }

      if (g_lp_stat.pm_lock_released == false)
        {
          esp32_pm_lockrelease();
          g_lp_stat.pm_lock_released = true;
        }
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

static void btdm_sleep_exit_phase3_wrapper(void)
{
  if (g_lp_stat.pm_lock_released)
    {
      esp32_pm_lockacquire();
      g_lp_stat.pm_lock_released = false;
    }

  if (btdm_sleep_clock_sync())
    {
      wlerr("sleep eco state err\n");
      DEBUGASSERT(0);
    }

  if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1)
    {
      if (g_lp_stat.phy_enabled == false)
        {
          bt_phy_enable();
          g_lp_stat.phy_enabled = true;
        }
    }

  if (g_lp_cntl.wakeup_timer_required && g_lp_stat.wakeup_timer_started)
    {
      esp_timer_stop(g_btdm_slp_tmr);
      g_lp_stat.wakeup_timer_started = false;
    }
}
#endif

/****************************************************************************
 * Name: btdm_controller_mem_init
 *
 * Description:
 *    Initialize BT controller to allocate task and other resource.
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

  memcpy(&_data_start_btdm, (void *)_data_start_btdm_rom,
         &_data_end_btdm - &_data_start_btdm);

  wlinfo(".data initialise [0x%08x] <== [0x%08x]\n",
         (uint32_t)&_data_start_btdm, _data_start_btdm_rom);

  /* initial em, .bss section */

  btdm_dram_regions = sizeof(btdm_dram_available_region)
                      / sizeof(btdm_dram_available_region_t);

  for (int i = 1; i < btdm_dram_regions; i++)
    {
      if (btdm_dram_available_region[i].mode != ESP_BT_MODE_IDLE)
        {
          memset((void *)btdm_dram_available_region[i].start, 0x0,
                 btdm_dram_available_region[i].end - \
                 btdm_dram_available_region[i].start);
          wlinfo(".bss initialise [0x%08x] - [0x%08x]\n",
                 btdm_dram_available_region[i].start,
                 btdm_dram_available_region[i].end);
        }
    }
}

/****************************************************************************
 * Name: phy_printf_ble
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

int phy_printf_ble(const char *format, ...)
{
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  va_list arg;

  va_start(arg, format);
  vsyslog(LOG_INFO, format, arg);
  va_end(arg);
#endif

  return 0;
}

int coexist_printf(const char *format, ...)
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
 * Name: bt_phy_enable_clock
 *
 * Description:
 *    Enable BT clock.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void bt_phy_enable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt == 0)
    {
      modifyreg32(DPORT_WIFI_CLK_EN_REG, 0,
                  DPORT_WIFI_CLK_WIFI_BT_COMMON_M);
    }

  g_phy_clk_en_cnt++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bt_phy_disable_clock
 *
 * Description:
 *    Disable BT clock.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void bt_phy_disable_clock(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_phy_clk_en_cnt)
    {
      g_phy_clk_en_cnt--;
      if (!g_phy_clk_en_cnt)
        {
          modifyreg32(DPORT_WIFI_CLK_EN_REG,
                      DPORT_WIFI_CLK_WIFI_BT_COMMON_M,
                      0);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bt_phy_disable
 *
 * Description:
 *    Disable BT phy.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void bt_phy_disable(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  g_phy_access_ref--;

  if (g_phy_access_ref == 0)
    {
      /* Disable PHY and RF. */

      phy_close_rf();

      /* Disable Wi-Fi/BT common peripheral clock.
       * Do not disable clock for hardware RNG.
       */

      bt_phy_disable_clock();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: bt_phy_enable
 *
 * Description:
 *    Enable BT phy.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void bt_phy_enable(void)
{
  irqstate_t flags;
  esp_phy_calibration_data_t *cal_data;

  cal_data = kmm_zalloc(sizeof(esp_phy_calibration_data_t));
  if (cal_data == NULL)
    {
      wlerr("Failed to kmm_zalloc");
      DEBUGASSERT(0);
    }

  flags = enter_critical_section();

  if (g_phy_access_ref == 0)
    {
      /* Update time stamp */

      g_phy_rf_en_ts = (int64_t)rt_timer_time_us();

      bt_phy_enable_clock();
      phy_set_wifi_mode_only(0);
      register_chipv7_phy(&phy_init_data, cal_data, PHY_RF_CAL_NONE);
#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
      coex_pti_v2();
#endif
    }

  g_phy_access_ref++;
  leave_critical_section(flags);
  kmm_free(cal_data);
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
      wlerr("Failed to create mqueue %d \n", ret);
      kmm_free(mq_adpt);
      return NULL;
    }

  mq_adpt->msgsize = item_size;
  return (void *)mq_adpt;
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

#ifdef CONFIG_ESP32_BLE_RUN_APP_CPU
    mask |= BTDM_CFG_CONTROLLER_RUN_APP_CPU;
#endif

#ifdef CONFIG_ESP32_BLE_FULL_SCAN
    mask |= BTDM_CFG_BLE_FULL_SCAN_SUPPORTED;
#endif

    mask |= BTDM_CFG_SCAN_DUPLICATE_OPTIONS;

    mask |= BTDM_CFG_SEND_ADV_RESERVED_SIZE;

    return mask;
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
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_init(void)
{
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_config_t *cfg = &bt_cfg;
  int err;
  uint32_t btdm_cfg_mask = 0;

  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE)
    {
      wlerr("Invalid controller status");
      return -ERROR;
    }

  if (btdm_osi_funcs_register(&g_osi_funcs) != 0)
    {
      wlinfo("Error, probably invalid OSI Functions\n");
      return -EINVAL;
    }

  wlinfo("BT controller compile version [%s]\n",
                              btdm_controller_get_compile_version());

  /* If all the bt available memory was already released,
   * cannot initialize bluetooth controller
   */

  if (btdm_dram_available_region[0].mode == ESP_BT_MODE_IDLE)
    {
      wlerr("Error, bt available memory was released\n");
      return ESP_ERR_INVALID_STATE;
    }

  if (cfg == NULL)
    {
      wlerr("%s  %d\n", __func__, __LINE__);
      return -EINVAL;
    }

  cfg->controller_task_stack_size  = CONFIG_ESP32_BLE_TASK_STACK_SIZE;
  cfg->controller_task_prio        = CONFIG_ESP32_BLE_TASK_PRIORITY;

  cfg->bt_max_sync_conn            = CONFIG_ESP32_BLE_MAX_CONN;
  cfg->magic                       = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL;

  if (((cfg->mode & ESP_BT_MODE_BLE) && (cfg->ble_max_conn <= 0 ||
        cfg->ble_max_conn > BTDM_CONTROLLER_BLE_MAX_CONN_LIMIT)) ||
      ((cfg->mode & ESP_BT_MODE_CLASSIC_BT) && (cfg->bt_max_acl_conn <= 0 ||
        cfg->bt_max_acl_conn > BTDM_CONTROLLER_BR_EDR_MAX_ACL_CONN_LIMIT)) ||
      ((cfg->mode & ESP_BT_MODE_CLASSIC_BT) &&
       (cfg->bt_max_sync_conn > BTDM_CONTROLLER_BR_EDR_MAX_SYNC_CONN_LIMIT)))
    {
      wlerr("%s  %d\n", __func__, __LINE__);
      return -EINVAL;
    }

  wlinfo("BT controller compile version [%s]",
         btdm_controller_get_compile_version());

  btdm_controller_mem_init();

  wlinfo("Memory initialized!\n");

  bt_periph_module_enable(PERIPH_BT_MODULE);

  /* set default sleep clock cycle and its fractional bits */

  g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
  g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

  btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE);

  btdm_cfg_mask = btdm_config_mask_load();

  wlinfo("Going to call btdm_controller_init\n");

  if (btdm_controller_init(btdm_cfg_mask, cfg) != 0)
    {
      wlerr("Failed to initialize the BLE Controller\n");
      err = -ENOMEM;
      goto error;
    }

  wlinfo("The btdm_controller_init was initialized\n");

#ifdef CONFIG_BTDM_COEX_BLE_ADV_HIGH_PRIORITY
  coex_ble_adv_priority_high_set(true);
#else
  coex_ble_adv_priority_high_set(false);
#endif

  btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

  return OK;

error:
  return err;
}

/****************************************************************************
 * Name: esp32_bt_controller_deinit
 *
 * Description:
 *    Deinit  BT controller.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_deinit(void)
{
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED)
    {
      return -ERROR;
    }

  btdm_controller_deinit();

  if (g_lp_stat.phy_enabled)
    {
      bt_phy_disable();
      g_lp_stat.phy_enabled = false;
    }
  else
    {
      DEBUGASSERT(0);
    }

#ifdef CONFIG_PM
  /* deinit low power control resources */

  g_lp_stat.pm_lock_released = false;

  if (g_lp_cntl.wakeup_timer_required)
    {
      if (g_lp_stat.wakeup_timer_started)
        {
          esp_timer_stop(g_btdm_slp_tmr);
        }

      g_lp_stat.wakeup_timer_started = false;
      esp_timer_delete(g_btdm_slp_tmr);
      g_btdm_slp_tmr = NULL;
    }

  if (g_lp_cntl.enable)
    {
      btdm_vnd_offload_task_deregister(BTDM_VND_OL_SIG_WAKEUP_TMR);
      semphr_delete_wrapper(g_wakeup_req_sem);
      g_wakeup_req_sem = NULL;
    }
#endif

  btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;
  g_btdm_lpcycle_us = 0;
  return 0;
}

/****************************************************************************
 * Name: esp32_bt_controller_disable
 *
 * Description:
 *    Disable  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_disable(void)
{
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return -ERROR;
    }

  while (!btdm_power_state_active())
    {
      usleep(1000); /* wait */
    }

  btdm_controller_disable();

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  coex_disable();
#endif

  btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

#ifdef CONFIG_PM
  /* disable low power mode */

  if (g_lp_stat.pm_lock_released == false)
    {
      esp32_pm_lockrelease();
      g_lp_stat.pm_lock_released = true;
    }
  else
    {
      DEBUGASSERT(0);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp32_bt_controller_enable
 *
 * Description:
 *    Enable  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32_bt_controller_enable(esp_bt_mode_t mode)
{
  int ret = 0;

  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED)
    {
      return -ERROR;
    }

  if (mode != btdm_controller_get_mode())
    {
      wlerr("invalid mode %d, controller support mode is %d",
                                  mode, btdm_controller_get_mode());
      return -1;
    }

  bt_phy_enable();

  btdm_rf_bb_init_phase2();

  coex_bt_high_prio();

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  coex_enable();
#endif

#ifdef CONFIG_PM
  /* enable low power mode */

  esp32_pm_lockacquire();
  g_lp_stat.pm_lock_released = false;

  if (g_lp_cntl.enable)
    {
      btdm_controller_enable_sleep(true);
    }
#endif

  if (g_lp_cntl.enable)
    {
        btdm_controller_enable_sleep(true);
    }

  if (btdm_controller_enable(mode) != 0)
    {
      ret = -1;
      goto error;
    }

  btdm_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;

  return ret;

error:

  /* disable low power mode */

  btdm_controller_enable_sleep(false);

#ifdef CONFIG_PM
  if (g_lp_stat.pm_lock_released == false)
    {
      esp32_pm_lockrelease();
      g_lp_stat.pm_lock_released = true;
    }
#endif

  return ret;
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
  return btdm_controller_status;
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
        request_lock = true;
        break;
      case BTDM_ASYNC_WAKEUP_REQ_COEX:
        request_lock = false;
        break;
      default:
        return false;
    }

  if (!btdm_power_state_active())
    {
      do_wakeup_request = true;
      btdm_wakeup_request(request_lock);
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
        request_lock = false;
        break;
      default:
        return;
    }

  if (request_lock)
    {
      btdm_wakeup_request_end();
    }

  return;
}

/****************************************************************************
 * Name: coex_bt_wakeup_request
 *
 * Description:
 *   Request a WiFi/BLE Coex wakeup request
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
 *   Finish WiFi/BLE Coex wakeup request
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
  return;
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
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return false;
    }

  return api_vhci_host_check_send_available();
}

/****************************************************************************
 * Name: esp32_vhci_host_send_packet
 *
 * Description:
 *    Host send packet to controller.
 * Input Parameters:
 *  data - the packet pointer
 *  len - the packet length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
  wlinfo("len: %d\n", len);
  for (uint16_t i = 0; i < len; i++)
    {
      wlinfo("%02x\n", data[i]);
    }

  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return;
    }

  api_vhci_host_send_packet(data, len);
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
  int ret = -ERROR;
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return ret;
    }

  ret = api_vhci_host_register_callback(
            (const vhci_host_callback_t *)callback) == 0 ? 0 : -1;
  return ret;
}

int coex_bt_request_wrapper(uint32_t event,
                            uint32_t latency,
                            uint32_t duration)
{
  return 0;
}

int coex_bt_release_wrapper(uint32_t event)
{
  return 0;
}

uint32_t coex_bb_reset_lock_wrapper(void)
{
  return 0;
}

void coex_bb_reset_unlock_wrapper(uint32_t restore)
{
  return;
}

