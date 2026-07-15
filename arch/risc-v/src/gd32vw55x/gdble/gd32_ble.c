/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdble/gd32_ble.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE 5.3 bring-up for the GD32VW55x, using the vendor's own BLE host.
 *
 * The prebuilt libble is an all-in-one controller + RivieraWaves host: the
 * full GAP/GATT/SMP stack is inside the blob (ble_adp_*, ble_adv_*,
 * ble_gap_*, ble_gatts_*), and the controller talks to it internally, not
 * over HCI.  The blob is built WITHOUT any HCI transport layer (its h4tl.o
 * is empty), so the NuttX native Bluetooth host, which speaks HCI, cannot
 * drive this controller.  This port therefore uses the vendor host directly,
 * the same way every GigaDevice BLE example does.
 *
 * Validated vendor HAL: SDK V1.0.3g (2026-04-23, commit 945c6e2).
 *
 * Init order (from the SDK ble_ibeacon example) is mandatory:
 *   ble_power_on() -> ble_sw_init() -> ble_adp_callback_register()
 *   -> ble_irq_enable() (only after ble_sw_init).
 * Advertising is then created from the adapter "enable complete" event and
 * started from the advertising "created" event.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/irq.h>

/* Vendor SDK BLE host headers */

#include "platform_def.h"     /* CFG_BLE_SUPPORT/CFG_WLAN_SUPPORT -> CFG_COEX */
#include "ble_export.h"
#include "ble_gap.h"
#include "ble_adapter.h"
#include "ble_adv.h"
#include "wrapper_os.h"
#include "gd32vw55x_platform.h"

#ifdef CONFIG_GD32VW55X_BLE_GATT_DEMO
#  include "ble_gatt.h"
#  include "ble_gatts.h"
#endif

/* Inside the blobs: the BLE scheduler (libble) and the MAC notifier
 * (libwifi).  There is no public header for them.
 */

extern void ble_coex_evt_notify_register(void (*cb)(uint32_t evt_start,
                                                    uint32_t evt_window,
                                                    uint32_t iso_evt));
extern void coex_ble_event_notify(uint32_t evt_start, uint32_t evt_window,
                                  uint32_t iso_evt);

/* The BLE radio interrupt handler bodies live in the SDK gd32vw55x_it.c
 * (compiled by Wireless.mk under CFG_BLE_SUPPORT); each calls the matching
 * ISR inside libble.  The vendor build reaches them through the ECLIC
 * hardware vector table; NuttX uses trap dispatch, so they must be attached
 * to the IRQ table below or the source fires into irq_unexpected_isr.
 */

extern void BLE_WKUP_IRQHandler(void);
extern void BLE_POWER_STATUS_IRQHandler(void);
extern void BLE_SW_TRIG_IRQHandler(void);
extern void BLE_FINE_TIMER_TARGET_IRQHandler(void);
extern void BLE_STAMP_TARGET1_IRQHandler(void);
extern void BLE_STAMP_TARGET2_IRQHandler(void);
extern void BLE_STAMP_TARGET3_IRQHandler(void);
extern void BLE_ENCRYPTION_ENGINE_IRQHandler(void);
extern void BLE_SLEEP_MODE_IRQHandler(void);
extern void BLE_HALF_SLOT_IRQHandler(void);
extern void BLE_FIFO_ACTIVITY_IRQHandler(void);
extern void BLE_ERROR_IRQHandler(void);
extern void BLE_FREQ_SELECT_IRQHandler(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BLE stack task parameters (same as the vendor examples).  Stack size is
 * in WORDS -- a FreeRTOS convention the SDK carries; wrapper_nuttx.c
 * converts it to bytes.
 */

#define GD32_BLE_TASK_STACK      1024
#define GD32_BLE_TASK_PRIO       OS_TASK_PRIORITY(2)
#define GD32_BLE_APP_TASK_STACK  1024
#define GD32_BLE_APP_TASK_PRIO   OS_TASK_PRIORITY(1)

/* Advertising: connectable, general discoverable, 100 ms interval.  The
 * device advertises the name below so it is easy to find with a scanner.
 */

#define GD32_BLE_ADV_NAME        "NuttX"
#define GD32_BLE_ADV_CH_MAP      0x07     /* channels 37, 38, 39 */
#define GD32_BLE_ADV_INTV_MIN    160      /* 160 * 0.625 ms = 100 ms */
#define GD32_BLE_ADV_INTV_MAX    160

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_ble_isr
 *
 * Description:
 *   Trampoline from the NuttX IRQ dispatch to the SDK's void(void) radio
 *   handler passed as 'arg' (same shape as the Wi-Fi glue).
 *
 ****************************************************************************/

static int gd32_ble_isr(int irq, void *context, void *arg)
{
  ((void (*)(void))arg)();
  return 0;
}

/****************************************************************************
 * Name: gd32_ble_irq_attach
 *
 * Description:
 *   Wire every BLE radio interrupt the SDK's ble_irq_enable() turns on to
 *   its SDK handler.  Attaching also keeps the handlers from being dropped
 *   by --gc-sections (nothing else references them in a NuttX build).  The
 *   ECLIC enable and edge/level trigger are done by ble_irq_enable() itself.
 *
 ****************************************************************************/

static void gd32_ble_irq_attach(void)
{
  irq_attach(GD32VW55X_IRQ_BLE_WKUP, gd32_ble_isr, BLE_WKUP_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_POWER_STAT, gd32_ble_isr,
             BLE_POWER_STATUS_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_SW_TRIG, gd32_ble_isr,
             BLE_SW_TRIG_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_FINE_TGT, gd32_ble_isr,
             BLE_FINE_TIMER_TARGET_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_STAMP_TGT1, gd32_ble_isr,
             BLE_STAMP_TARGET1_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_STAMP_TGT2, gd32_ble_isr,
             BLE_STAMP_TARGET2_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_STAMP_TGT3, gd32_ble_isr,
             BLE_STAMP_TARGET3_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_ENC_ENGINE, gd32_ble_isr,
             BLE_ENCRYPTION_ENGINE_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_SLEEP_MODE, gd32_ble_isr,
             BLE_SLEEP_MODE_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_HALF_SLOT, gd32_ble_isr,
             BLE_HALF_SLOT_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_FIFO, gd32_ble_isr,
             BLE_FIFO_ACTIVITY_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_ERROR, gd32_ble_isr, BLE_ERROR_IRQHandler);
  irq_attach(GD32VW55X_IRQ_BLE_FREQ_SEL, gd32_ble_isr,
             BLE_FREQ_SELECT_IRQHandler);
}

/****************************************************************************
 * Name: gd32_ble_adv_start
 *
 * Description:
 *   Set the advertising data (flags + complete local name) and start the
 *   set.  Called once the set has been created.
 *
 ****************************************************************************/

static void gd32_ble_adv_start(uint8_t adv_idx)
{
  ble_adv_data_set_t adv;
  ble_adv_data_t adv_data;

  memset(&adv, 0, sizeof(adv));
  memset(&adv_data, 0, sizeof(adv_data));

  adv_data.flags            = BLE_GAP_ADV_FLAG_LE_ONLY_GENERAL_DISC_MODE;
  adv_data.local_name.type   = BLE_ADV_DATA_FULL_NAME;
  adv_data.local_name.p_name = (uint8_t *)GD32_BLE_ADV_NAME;
  adv_data.local_name.name_len = sizeof(GD32_BLE_ADV_NAME) - 1;

  adv.data_force         = false;
  adv.data.p_data_enc    = &adv_data;

  ble_adv_start(adv_idx, &adv, NULL, NULL);
}

/****************************************************************************
 * Name: gd32_ble_adv_evt_handler
 *
 * Description:
 *   Advertising state machine: when the set finishes being created, push
 *   the advertising data; the stack then moves it to the started state.
 *
 ****************************************************************************/

static void gd32_ble_adv_evt_handler(ble_adv_evt_t adv_evt, void *p_data,
                                     void *p_context)
{
  ble_adv_state_chg_t *chg;

  if (adv_evt != BLE_ADV_EVT_STATE_CHG)
    {
      return;
    }

  chg = (ble_adv_state_chg_t *)p_data;

  if (chg->state == BLE_ADV_STATE_CREATE)
    {
      gd32_ble_adv_start(chg->adv_idx);
    }
  else if (chg->state == BLE_ADV_STATE_START)
    {
      wlinfo("BLE advertising '%s'\n", GD32_BLE_ADV_NAME);
    }
}

/****************************************************************************
 * Name: gd32_ble_adv_create
 *
 * Description:
 *   Create a legacy, connectable, general-discoverable advertising set.
 *
 ****************************************************************************/

static void gd32_ble_adv_create(void)
{
  ble_adv_param_t adv_param;

  memset(&adv_param, 0, sizeof(adv_param));

  adv_param.param.own_addr_type = BLE_GAP_LOCAL_ADDR_STATIC;
  adv_param.param.type          = BLE_GAP_ADV_TYPE_LEGACY;
  adv_param.param.prop          = BLE_GAP_ADV_PROP_UNDIR_CONN;
  adv_param.param.filter_pol    = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
  adv_param.param.disc_mode     = BLE_GAP_ADV_MODE_GEN_DISC;
  adv_param.param.primary_phy   = BLE_GAP_PHY_1MBPS;
  adv_param.param.ch_map        = GD32_BLE_ADV_CH_MAP;
  adv_param.param.adv_intv_min  = GD32_BLE_ADV_INTV_MIN;
  adv_param.param.adv_intv_max  = GD32_BLE_ADV_INTV_MAX;

  ble_adv_create(&adv_param, gd32_ble_adv_evt_handler, NULL);
}

/****************************************************************************
 * Name: gd32_ble_adp_evt_handler
 *
 * Description:
 *   Adapter events.  Once the stack reports "enable complete", it is safe
 *   to create the advertising set.
 *
 ****************************************************************************/

static void gd32_ble_adp_evt_handler(ble_adp_evt_t event,
                                     ble_adp_data_u *p_data)
{
  if (event == BLE_ADP_EVT_ENABLE_CMPL_INFO)
    {
      gd32_ble_adv_create();
    }
}

#ifdef CONFIG_GD32VW55X_BLE_GATT_DEMO

/****************************************************************************
 * Demo GATT service
 *
 * A minimal "transparent UART" service so a phone or PC can exercise the BLE
 * data path: the central writes bytes to the RX characteristic (central ->
 * board), the board logs them, and -- if the peer has subscribed -- echoes
 * them back through a TX notification (board -> central).  16-bit UUIDs in
 * the 0xffe0 vendor range, following the SDK app_blue_courier_link server
 * model: the write is handled straight from the GATT server callback (which
 * runs on the vendor BLE task) and the echo notification is issued from that
 * same context, so the demo never re-enters the stack from a foreign task or
 * work queue.
 *
 * Note: the write path is validated with write-without-response (a write
 * command).  The prebuilt vendor controller does not reliably complete a
 * write-request (write-with-response) or the CCCD subscribe that a Linux
 * BlueZ host issues, so notifications are best exercised from a phone app
 * such as nRF Connect.  See the board documentation.
 ****************************************************************************/

#define GD32_BLE_SVC_UUID     0xffe0
#define GD32_BLE_RX_UUID      0xffe1   /* central -> board (write)  */
#define GD32_BLE_TX_UUID      0xffe2   /* board -> central (notify) */
#define GD32_BLE_GATT_MAXLEN  244

enum
{
  GD32_BLE_IDX_SVC,
  GD32_BLE_IDX_CHAR_RX,
  GD32_BLE_IDX_RX,
  GD32_BLE_IDX_CHAR_TX,
  GD32_BLE_IDX_TX,
  GD32_BLE_IDX_TX_CCCD,
  GD32_BLE_IDX_NUM
};

static const ble_gatt_attr_desc_t g_gatt_db[GD32_BLE_IDX_NUM] =
{
  [GD32_BLE_IDX_SVC]     = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_PRIMARY_SERVICE),
                             PROP(RD), 0 },
  [GD32_BLE_IDX_CHAR_RX] = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_CHARACTERISTIC),
                             PROP(RD), 0 },
  [GD32_BLE_IDX_RX]      = { UUID_16BIT_TO_ARRAY(GD32_BLE_RX_UUID),
                             PROP(WR) | PROP(WC), GD32_BLE_GATT_MAXLEN },
  [GD32_BLE_IDX_CHAR_TX] = { UUID_16BIT_TO_ARRAY(BLE_GATT_DECL_CHARACTERISTIC),
                             PROP(RD), 0 },
  [GD32_BLE_IDX_TX]      = { UUID_16BIT_TO_ARRAY(GD32_BLE_TX_UUID),
                             PROP(NTF), GD32_BLE_GATT_MAXLEN },
  [GD32_BLE_IDX_TX_CCCD] = { UUID_16BIT_TO_ARRAY(BLE_GATT_DESC_CLIENT_CHAR_CFG),
                             PROP(RD) | PROP(WR),
                             OPT(NO_OFFSET) | sizeof(uint16_t) },
};

static uint8_t  g_gatt_svc_id;
static uint16_t g_gatt_tx_cccd;   /* != 0 once the peer enables notifications */

/****************************************************************************
 * Name: gd32_ble_gatts_cb
 *
 * Description:
 *   Server events.  A write to the RX characteristic is logged and echoed
 *   back through a TX notification; a write to the TX client-config
 *   descriptor toggles notifications.  No explicit read/write confirm is
 *   used: the stack auto-confirms from the callback return value and serves
 *   descriptor reads from its own storage (SDK courier_link model).
 *
 ****************************************************************************/

static ble_status_t gd32_ble_gatts_cb(ble_gatts_msg_info_t *info)
{
  ble_gatts_op_info_t *op;
  ble_gatts_write_req_t *w;

  if (info->srv_msg_type != BLE_SRV_EVT_GATT_OPERATION)
    {
      return BLE_ERR_NO_ERROR;
    }

  op = &info->msg_data.gatts_op_info;

  if (op->gatts_op_sub_evt != BLE_SRV_EVT_WRITE_REQ)
    {
      return BLE_ERR_NO_ERROR;
    }

  w = &op->gatts_op_data.write_req;

  if (w->att_idx == GD32_BLE_IDX_RX)
    {
      syslog(LOG_INFO, "BLE RX (%u): %.*s\n", w->val_len,
             (int)w->val_len, (const char *)w->p_val);

      /* Echo the bytes straight back through a TX notification, from this
       * same BLE-task context, if the peer has subscribed.
       */

      if (g_gatt_tx_cccd != 0)
        {
          ble_gatts_ntf_ind_send(op->conn_idx, g_gatt_svc_id,
                                 GD32_BLE_IDX_TX, w->p_val, w->val_len,
                                 BLE_GATT_NOTIFY);
        }
    }
  else if (w->att_idx == GD32_BLE_IDX_TX_CCCD &&
           w->val_len == sizeof(uint16_t))
    {
      g_gatt_tx_cccd = (uint16_t)(w->p_val[0] | (w->p_val[1] << 8));
    }

  return BLE_ERR_NO_ERROR;
}

/****************************************************************************
 * Name: gd32_ble_gatts_add
 *
 * Description:
 *   Register the demo service.  Called once the host is up (after
 *   ble_sw_init).
 *
 ****************************************************************************/

static void gd32_ble_gatts_add(void)
{
  uint8_t svc_uuid[BLE_GATT_UUID_128_LEN] =
      UUID_16BIT_TO_ARRAY(GD32_BLE_SVC_UUID);

  ble_gatts_svc_add(&g_gatt_svc_id, svc_uuid, 0, 0, g_gatt_db,
                    GD32_BLE_IDX_NUM, gd32_ble_gatts_cb);
}

#endif /* CONFIG_GD32VW55X_BLE_GATT_DEMO */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_ble_initialize
 *
 * Description:
 *   Bring the BLE controller and the vendor host up, and start advertising.
 *   Depends on the Wi-Fi platform bring-up (gdwifi_start): clocks, RF, TRNG
 *   and NVDS are shared, and coexistence lives inside the blobs.
 *
 ****************************************************************************/

int gd32_ble_initialize(void)
{
  static ble_os_api_t os_api =
    {
      .os_malloc                  = sys_malloc,
      .os_calloc                  = sys_calloc,
      .os_mfree                   = sys_mfree,
      .os_memset                  = sys_memset,
      .os_memcpy                  = sys_memcpy,
      .os_memcmp                  = sys_memcmp,
      .os_task_create             = sys_task_create,
      .os_task_init_notification  = sys_task_init_notification,
      .os_task_wait_notification  = sys_task_wait_notification,
      .os_task_notify             = sys_task_notify,
      .os_task_delete             = sys_task_delete,
      .os_ms_sleep                = sys_ms_sleep,
      .os_current_task_handle_get = sys_current_task_handle_get,
      .os_queue_init              = sys_queue_init,
      .os_queue_free              = sys_queue_free,
      .os_queue_write             = sys_queue_write,
      .os_queue_read              = sys_queue_read,
      .os_random_bytes_get        = sys_random_bytes_get,
    };

  ble_init_param_t param;
  ble_status_t status;

  memset(&param, 0, sizeof(param));

  /* Mandatory order: power on -> ble_sw_init -> callbacks -> IRQs */

  ble_power_on();

  param.role                    = BLE_GAP_ROLE_PERIPHERAL;
  param.ble_task_stack_size     = GD32_BLE_TASK_STACK;
  param.ble_task_priority       = GD32_BLE_TASK_PRIO;
  param.ble_app_task_stack_size = GD32_BLE_APP_TASK_STACK;
  param.ble_app_task_priority   = GD32_BLE_APP_TASK_PRIO;
  param.en_cfg                  = 0;
  param.p_os_api                = &os_api;
  param.p_hci_uart_func         = NULL;  /* internal host, no HCI transport */

  status = ble_sw_init(&param);
  if (status != BLE_ERR_NO_ERROR)
    {
      wlerr("ERROR: ble_sw_init failed: %d\n", status);
      return -EIO;
    }

  ble_adp_callback_register(gd32_ble_adp_evt_handler);

#ifdef CONFIG_GD32VW55X_BLE_GATT_DEMO
  /* Register the demo GATT service (RX write / TX notify) */

  gd32_ble_gatts_add();
#endif

  /* Attach the radio handlers before enabling the sources in the ECLIC */

  gd32_ble_irq_attach();

  ble_irq_enable();                 /* only after ble_sw_init */

#ifdef CFG_COEX
  /* Coexistence: the BLE scheduler (libble) tells the MAC (libwifi) about
   * every radio window.  Both sides are already in the blobs; only the
   * registration is ours.
   */

  ble_coex_evt_notify_register(coex_ble_event_notify);
#endif

  wlinfo("BLE stack started\n");
  return OK;
}
