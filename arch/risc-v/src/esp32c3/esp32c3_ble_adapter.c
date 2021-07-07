/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_ble_adapter.c
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

#include "hardware/esp32c3_syscon.h"
#include "espidf_wifi.h"
#include "esp32c3.h"
#include "esp32c3_attr.h"
#include "esp32c3_irq.h"
#include "esp32c3_rt_timer.h"
#include "esp32c3_ble_adapter.h"

#ifdef CONFIG_ESP32C3_WIFI_BT_COEXIST
#  include "esp_coexist_internal.h"
#  include "esp_coexist_adapter.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OSI_FUNCS_TIME_BLOCKING          0xffffffff
#define OSI_VERSION                      0x00010006
#define OSI_MAGIC_VALUE                  0xfadebead

#ifdef CONFIG_PM
#define BTDM_MIN_TIMER_UNCERTAINTY_US    (1800)

/* Sleep and wakeup interval control */

#define BTDM_MIN_SLEEP_DURATION          (24) /* threshold of interval in half slots to allow to fall into modem sleep */
#define BTDM_MODEM_WAKE_UP_DELAY         (8)  /* delay in half slots of modem wake up procedure, including re-enable PHY/RF */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

/* BLE low power control struct */

typedef enum btdm_lpclk_sel_e
{
  BTDM_LPCLK_SEL_XTAL     = 0,
  BTDM_LPCLK_SEL_XTAL32K  = 1,
  BTDM_LPCLK_SEL_RTC_SLOW = 2,
  BTDM_LPCLK_SEL_8M       = 3,
} btdm_lpclk_sel_t;

typedef enum btdm_vnd_ol_sig_e
{
  BTDM_VND_OL_SIG_WAKEUP_TMR,
  BTDM_VND_OL_SIG_NUM,
} btdm_vnd_ol_sig_t;

typedef struct btdm_lpcntl_s
{
  bool enable;                  /* whether low power mode is required */
  bool wakeup_timer_required;   /* whether system timer is needed */
  btdm_lpclk_sel_t lpclk_sel;   /* low power clock source */
} btdm_lpcntl_t;

/* low power control status */

typedef struct btdm_lpstat_s
{
  bool pm_lock_released;        /* whether power management lock is released */
  bool phy_enabled;             /* whether phy is switched on */
  bool wakeup_timer_started;    /* whether wakeup timer is started */
} btdm_lpstat_t;

#ifdef CONFIG_PM
/* wakeup request sources */

enum btdm_wakeup_src_e
{
  BTDM_ASYNC_WAKEUP_SRC_VHCI,
  BTDM_ASYNC_WAKEUP_SRC_DISA,
  BTDM_ASYNC_WAKEUP_SRC_TMR,
  BTDM_ASYNC_WAKEUP_SRC_MAX,
};
#endif

/* prototype of function to handle vendor dependent signals */

typedef void (* btdm_vnd_ol_task_func_t)(void *param);

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

typedef void (* osi_intr_handler)(void);

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

/****************************************************************************
 * Private Function
 ****************************************************************************/

static void interrupt_set_wrapper(int cpu_no, int intr_source,
                                  int intr_num, int intr_prio);
static void interrupt_clear_wrapper(int intr_source, int intr_num);
static void interrupt_handler_set_wrapper(int n, void *fn, void *arg);
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
static void *malloc_internal_wrapper(size_t size);
static int IRAM_ATTR read_mac_wrapper(uint8_t mac[6]);
static void IRAM_ATTR srand_wrapper(unsigned int seed);
static int IRAM_ATTR rand_wrapper(void);
static uint32_t IRAM_ATTR btdm_lpcycles_2_hus(uint32_t cycles,
                                              uint32_t *error_corr);
static uint32_t IRAM_ATTR btdm_hus_2_lpcycles(uint32_t us);
static void coex_wifi_sleep_set_hook(bool sleep);
static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status);
static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                                               uint32_t status);
static void interrupt_on_wrapper(int intr_num);
static void interrupt_off_wrapper(int intr_num);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static int queue_send_wrapper(void *queue, void *item,
                              uint32_t block_time_ms);
static int queue_recv_wrapper(void *queue, void *item,
                              uint32_t block_time_ms);
static void queue_delete_wrapper(void *queue);

#ifdef CONFIG_PM
static bool IRAM_ATTR btdm_sleep_check_duration(int32_t *half_slot_cnt);
static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles);
static void btdm_sleep_enter_phase2_wrapper(void);
static void btdm_sleep_exit_phase3_wrapper(void);
#endif

/****************************************************************************
 * Extern Functions declaration and value
 ****************************************************************************/

extern int btdm_osi_funcs_register(void *osi_funcs);
extern void btdm_controller_rom_data_init(void);

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
extern int btdm_vnd_offload_post_from_isr(btdm_vnd_ol_sig_t sig,
                                          void *param, bool need_yield);
extern int btdm_vnd_offload_post(btdm_vnd_ol_sig_t sig, void *param);

/* Low Power Clock */

extern bool btdm_lpclk_select_src(uint32_t sel);
extern bool btdm_lpclk_set_div(uint32_t div);
extern int btdm_hci_tl_io_event_post(int event);

/* VHCI */

extern bool API_vhci_host_check_send_available(void); /* Functions in bt lib */
extern void API_vhci_host_send_packet(uint8_t * data, uint16_t len);
extern int API_vhci_host_register_callback(const vhci_host_callback_t
                                           *callback);

/* TX power */

extern int ble_txpwr_set(int power_type, int power_level);
extern int ble_txpwr_get(int power_type);

extern uint16_t l2c_ble_link_get_tx_buf_num(void);
extern int coex_core_ble_conn_dyn_prio_get(bool *low, bool *high);

extern bool btdm_deep_sleep_mem_init(void);
extern void btdm_deep_sleep_mem_deinit(void);
extern void btdm_ble_power_down_dma_copy(bool copy);
extern uint8_t btdm_sleep_clock_sync(void);

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

static bool g_ble_irq_bind;
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
  ._malloc = malloc_wrapper,
  ._malloc_internal = malloc_internal_wrapper,
  ._free = free,
  ._read_efuse_mac = read_mac_wrapper,
  ._srand = srand_wrapper,
  ._rand = rand_wrapper,
  ._btdm_lpcycles_2_hus = btdm_lpcycles_2_hus,
  ._btdm_hus_2_lpcycles = btdm_hus_2_lpcycles,
#ifdef CONFIG_PM
  ._btdm_sleep_check_duration = btdm_sleep_check_duration,
  ._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
  ._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
  ._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
#endif
  ._coex_wifi_sleep_set = coex_wifi_sleep_set_hook,
  ._coex_core_ble_conn_dyn_prio_get = coex_core_ble_conn_dyn_prio_get,
  ._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
  ._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
  ._interrupt_on = interrupt_on_wrapper,
  ._interrupt_off = interrupt_off_wrapper,
};

/****************************************************************************
 * Private Functions and Public Functions only used by libraries
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

  pid = kthread_create(name, prio, stack_depth, entry,
                      (char * const *)param);
  if (pid > 0)
    {
      if (task_handle)
        {
          *((int *)task_handle) = pid;
        }
    }
  else
    {
      wlerr("Failed to create task\n");
    }

  return pid > 0 ? true : false;
}

/****************************************************************************
 * Name: interrupt_set_wrapper
 *
 * Description:
 *   Bind IRQ and resource with given parameters.
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

static void interrupt_set_wrapper(int cpu_no,
                                  int intr_source,
                                  int intr_num,
                                  int intr_prio)
{
    wlinfo("cpu_no=%d , intr_source=%d , intr_num=%d, intr_prio=%d\n",
                        cpu_no, intr_source, intr_num, intr_prio);
    esp32c3_bind_irq(intr_num, intr_source, intr_prio, ESP32C3_INT_LEVEL);
}

/****************************************************************************
 * Name: interrupt_clear_wrapper
 *
 * Description:
 *   Not supported
 *
 ****************************************************************************/

static void interrupt_clear_wrapper(int intr_source, int intr_num)
{
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

static int esp_int_adpt_cb(int irq, void *context, FAR void *arg)
{
  struct irq_adpt_s *adapter = (struct irq_adpt_s *)arg;

  adapter->func(adapter->arg);

  return OK;
}

/****************************************************************************
 * Name: interrupt_handler_set_wrapper
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

static void interrupt_handler_set_wrapper(int n, void *fn, void *arg)
{
  int ret;
  struct irq_adpt_s *adapter;

    if (g_ble_irq_bind)
      {
        return;
      }

  adapter = kmm_malloc(sizeof(struct irq_adpt_s));
  DEBUGASSERT(adapter);

  adapter->func = fn;
  adapter->arg = arg;

  ret = irq_attach(n + ESP32C3_IRQ_FIRSTPERIPH, esp_int_adpt_cb, adapter);
  DEBUGASSERT(ret == OK);

  g_ble_irq_bind = true;
}

/****************************************************************************
 * Name: esp32c3_ints_on
 *
 * Description:
 *   Enable Wi-Fi interrupt
 *
 * Input Parameters:
 *   intr_num - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void interrupt_on_wrapper(int intr_num)
{
  up_enable_irq(intr_num);
}

/****************************************************************************
 * Name: esp32c3_ints_off
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
  up_disable_irq(intr_num);
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
  enter_critical_section();
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
  DEBUGASSERT(sem);

  ret = sem_init(sem, 0, init);
  DEBUGASSERT(ret == OK);

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
 *   take a semaphore from an ISR
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
  return semphr_take_wrapper(semphr, 0);
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
          wlerr("Failed to wait sem\n");
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
 *  szie - buffer size
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
 *  mac - mac address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int IRAM_ATTR read_mac_wrapper(uint8_t mac[6])
{
  return 0;
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
 *    Get random value
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
 * Name: btdm_lpcycles_2_hus
 *
 * Description:
 * Converts a number of low power clock cycles into a duration in half us.
 *
 * Input Parameters:
 *  cycles
 * error_corr
 *
 * Returned Value:
 *   us
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR btdm_lpcycles_2_hus(uint32_t cycles,
                                              uint32_t *error_corr)
{
  uint64_t us = (uint64_t)g_btdm_lpcycle_us * cycles;
  us = (us + (1 << (g_btdm_lpcycle_us_frac - 1))) >> g_btdm_lpcycle_us_frac;
  return (uint32_t)us;
}

/****************************************************************************
 * Name: btdm_hus_2_lpcycles
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

static uint32_t IRAM_ATTR btdm_hus_2_lpcycles(uint32_t us)
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
 *   acquire PM lock and stop esp timer.
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
  DEBUGASSERT(g_lp_cntl.enable == true);

  if (g_lp_stat.pm_lock_released)
    {
      esp32c3_pm_lockacquire();
      g_lp_stat.pm_lock_released = false;
    }

  int event = (int) param;

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
 *   Esp ble sleep callback function.
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
 *   ESP32C3 BLE lightsleep callback function.
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
  if (g_lp_cntl.wakeup_timer_required == false)
    {
      return;
    }

  /* start a timer to wake up and acquire the pm_lock before sleep awakes */

  uint32_t us_to_sleep = btdm_lpcycles_2_hus(lpcycles, NULL) >> 1;

  DEBUGASSERT(us_to_sleep > BTDM_MIN_TIMER_UNCERTAINTY_US);
  uint32_t uncertainty = (us_to_sleep >> 11);

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
 *   ESP32C3 BLE lightsleep callback function.
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
          esp32c3_pm_lockrelease();
          g_lp_stat.pm_lock_released = true;
        }
    }
}

/****************************************************************************
 * Name: btdm_sleep_exit_phase3_wrapper
 *
 * Description:
 *   ESP32C3 BLE lightsleep callback function..
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
      esp32c3_pm_lockacquire();
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
 * Name: coex_schm_status_bit_set_wrapper
 *
 * Description:
 *
 * Input Parameters:
 *  type
 * status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#ifdef CONFIG_ESP32C3_WIFI_BT_COEXIST
  coex_schm_status_bit_set(type, status);
#endif
}

/****************************************************************************
 * Name: coex_schm_status_bit_clear_wrapper
 *
 * Description:
 *
 * Input Parameters:
 *  szie
 *  status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void coex_schm_status_bit_clear_wrapper(uint32_t type,
                          uint32_t status)
{
#ifdef CONFIG_ESP32C3_WIFI_BT_COEXIST
  coex_schm_status_bit_clear(type, status);
#endif
}

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
  btdm_controller_rom_data_init();
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

#ifndef CONFIG_ESP32C3_WIFI
int phy_printf(const char *format, ...)
{
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  va_list arg;

  va_start(arg, format);
  vsyslog(LOG_INFO, format, arg);
  va_end(arg);
#endif

  return 0;
}
#endif

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
      modifyreg32(SYSTEM_WIFI_CLK_EN_REG, 0,
                  SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M);
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
          modifyreg32(SYSTEM_WIFI_CLK_EN_REG,
                      SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M,
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
  if (!cal_data)
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
      extern void coex_pti_v2(void);
      coex_pti_v2();
    }

  g_phy_access_ref++;
  leave_critical_section(flags);
  kmm_free(cal_data);
}

static void coex_wifi_sleep_set_hook(bool sleep)
{
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
      wlerr("Failed to create mqueue\n");
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
 *   item          - Message data pointerint
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
          wlerr("Failed to get time\n");
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_bt_controller_init
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

int esp32c3_bt_controller_init(void)
{
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_config_t *cfg = &bt_cfg;
#ifdef CONFIG_PM
  bool select_src_ret;
  bool set_div_ret;
#endif

  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE)
    {
      wlerr("Invalid controller status");
      return -1;
    }

  cfg->controller_task_stack_size       = CONFIG_ESP32C3_BLE_TASK_STACK_SIZE;
  cfg->controller_task_prio             = CONFIG_ESP32C3_BLE_TASK_PRIORITY;

  cfg->controller_task_run_cpu          = 0;
  cfg->ble_max_act                      = 10;
  cfg->sleep_mode                       = 0;
  cfg->coex_phy_coded_tx_rx_time_limit  = 0;
  cfg->bluetooth_mode                   = 1;
  cfg->sleep_clock                      = 0;
  cfg->ble_st_acl_tx_buf_nb             = 0;
  cfg->ble_hw_cca_check                 = 0;
  cfg->ble_adv_dup_filt_max             = 30;
  cfg->ce_len_type                      = 0;
  cfg->hci_tl_type                      = 1;
  cfg->hci_tl_funcs                     = NULL;
  cfg->txant_dft                        = 0;
  cfg->rxant_dft                        = 0;
  cfg->txpwr_dft                        = 7;
  cfg->cfg_mask                         = 1;
  cfg->scan_duplicate_mode              = 0;
  cfg->scan_duplicate_type              = 0;
  cfg->normal_adv_size                  = 20;
  cfg->mesh_adv_size                    = 0;

  btdm_controller_mem_init();

  if (btdm_osi_funcs_register(&g_osi_funcs) != 0)
    {
      return -EINVAL;
    }

  wlinfo("BT controller compile version [%s]\n",
                              btdm_controller_get_compile_version());

#ifdef CONFIG_PM
  /* init low-power control resources */

  memset(&g_lp_cntl, 0x0, sizeof(btdm_lpcntl_t));
  memset(&g_lp_stat, 0x0, sizeof(btdm_lpstat_t));
  g_wakeup_req_sem = NULL;
  g_btdm_slp_tmr = NULL;

  /* configure and initialize resources */

  g_lp_cntl.enable = (cfg->sleep_mode == ESP_BT_SLEEP_MODE_1) ? true : false;

  if (g_lp_cntl.enable)
    {
      g_lp_cntl.wakeup_timer_required = true;
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

      if ((err = esp_timer_create(&create_args, &g_btdm_slp_tmr)) != ESP_OK)
        {
          goto error;
        }
    }

  g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
  g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

  if (esp32c3_rtc_clk_slow_freq_get() == RTC_SLOW_FREQ_32K_XTAL)
    {
      g_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_XTAL32K;
    }
  else
    {
      wlwarn("32.768kHz XTAL not detected");
      g_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_XTAL;
    }

  if (g_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL)
    {
      select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL);
      set_div_ret = btdm_lpclk_set_div(esp32c3_rtc_clk_xtal_freq_get() * 2);
      DEBUGASSERT(select_src_ret && set_div_ret);
      g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
      g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);
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
      g_btdm_lpcycle_us = esp_clk_slowclk_cal_get_wrapper();
    }
  else
    {
      goto error;
    }

  g_lp_stat.pm_lock_released = true;
#endif

#ifdef CONFIG_ESP32C3_WIFI_BT_COEXIST
  coex_init();
#endif

  modifyreg32(SYSTEM_WIFI_CLK_EN_REG, 0, UINT32_MAX);

  bt_phy_enable();
  g_lp_stat.phy_enabled = true;

  if (btdm_controller_init(cfg) != 0)
    {
      bt_phy_disable();
      g_lp_stat.phy_enabled = false;
      return -EIO;
    }

  btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

  return 0;

#ifdef CONFIG_PM
error:
  if (g_lp_stat.phy_enabled)
    {
      bt_phy_disable();
      g_lp_stat.phy_enabled = false;
    }

  g_lp_stat.pm_lock_released = false;

  if (g_lp_cntl.wakeup_timer_required && g_btdm_slp_tmr != NULL)
    {
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

  return ENOMEM;
#endif
}

/****************************************************************************
 * Name: esp32c3_bt_controller_deinit
 *
 * Description:
 *    Deinit  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32c3_bt_controller_deinit(void)
{
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED)
    {
      return -1;
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
 * Name: esp32c3_bt_controller_disable
 *
 * Description:
 *    disable  BT controller.
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

int esp32c3_bt_controller_disable(void)
{
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return -1;
    }

  while (!btdm_power_state_active())
    {
      usleep(1000); /* wait */
    }

  btdm_controller_disable();

#ifdef CONFIG_ESP32C3_WIFI_BT_COEXIST
  coex_disable();
#endif

  btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

#ifdef CONFIG_PM
  /* disable low power mode */

  if (g_lp_stat.pm_lock_released == false)
    {
      esp32c3_pm_lockrelease();
      g_lp_stat.pm_lock_released = true;
    }
  else
    {
      DEBUGASSERT(0);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: esp32c3_bt_controller_enable
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

int esp32c3_bt_controller_enable(esp_bt_mode_t mode)
{
  int ret = 0;

  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED)
    {
      return -1;
    }

  if (mode != btdm_controller_get_mode())
    {
      wlerr("invalid mode %d, controller support mode is %d",
                                  mode, btdm_controller_get_mode());
      return -1;
    }

#ifdef CONFIG_ESP32C3_WIFI_BT_COEXIST
  coex_enable();
#endif

#ifdef CONFIG_PM
  /* enable low power mode */

  esp32c3_pm_lockacquire();
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
      esp32c3_pm_lockrelease();
      g_lp_stat.pm_lock_released = true;
    }
#endif

  return ret;
}

esp_bt_controller_status_t esp32c3_bt_controller_get_status(void)
{
  return btdm_controller_status;
}

/****************************************************************************
 * Name: esp32c3_vhci_host_check_send_available
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

bool esp32c3_vhci_host_check_send_available(void)
{
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return false;
    }

  return API_vhci_host_check_send_available();
}

/****************************************************************************
 * Name: esp32c3_vhci_host_send_packet
 *
 * Description:
 *    host send packet to controller.
 * Input Parameters:
 *  data - the packet point
 *  len - the packet length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return;
    }

  API_vhci_host_send_packet(data, len);
}

/****************************************************************************
 * Name: esp32c3_vhci_register_callback
 *
 * Description:
 *    register the vhci reference callback.
 * Input Parameters:
 *  callback - struct defined by vhci_host_callback structure.
 *
 * Returned Value:
 *   status - success or fail
 *
 ****************************************************************************/

int esp32c3_vhci_register_callback(const esp_vhci_host_callback_t *callback)
{
  int ret = -1;
  if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      return ret;
    }

  ret = API_vhci_host_register_callback(
            (const vhci_host_callback_t *)callback) == 0 ? 0 : -1;
  return ret;
}
