/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_blehci.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_UART_BTH4)
#  include <nuttx/serial/uart_bth4.h>
#endif

#include "stm32wb_ipcc.h"
#include "stm32wb_mbox.h"
#include "stm32wb_mbox_shci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32WB_BLE_PREP_WRITE_NUM \
  STM32WB_MBOX_DEFAULT_BLE_PREP_WRITE_NUM(CONFIG_STM32WB_BLE_MAX_ATT_MTU)

#define STM32WB_C2_MEM_BLOCK_NUM \
  STM32WB_MBOX_DEFAULT_C2_MEM_BLOCK_NUM(CONFIG_STM32WB_BLE_MAX_ATT_MTU, \
                                        CONFIG_STM32WB_BLE_MAX_CONN, \
                                        STM32WB_BLE_PREP_WRITE_NUM)

#ifdef CONFIG_STM32WB_BLE_C2HOST
#  define STM32WB_BLE_C2HOST              STM32WB_SHCI_BLE_INIT_OPT_STACK_LL_HOST
#else
#  define STM32WB_BLE_C2HOST              STM32WB_SHCI_BLE_INIT_OPT_STACK_LL
#endif

#ifdef CONFIG_STM32WB_BLE_SVC_CHANGED_CHAR
#  define STM32WB_BLE_SVC_CHANGED_CHAR    STM32WB_SHCI_BLE_INIT_OPT_SVC_CHCHAR_ENABLED
#else
#  define STM32WB_BLE_SVC_CHANGED_CHAR    STM32WB_SHCI_BLE_INIT_OPT_SVC_CHCHAR_DISABLED
#endif

#ifdef CONFIG_STM32WB_BLE_WRITABLE_DEVICE_NAME
#  define STM32WB_BLE_DEVICE_NAME_MODE    STM32WB_SHCI_BLE_INIT_OPT_DEVICE_NAME_MODE_RW
#else
#  define STM32WB_BLE_DEVICE_NAME_MODE    STM32WB_SHCI_BLE_INIT_OPT_DEVICE_NAME_MODE_RO
#endif

#ifdef CONFIG_STM32WB_BLE_CHAN_SEL_ALG2
#  define STM32WB_BLE_CS_ALG2             STM32WB_SHCI_BLE_INIT_OPT_CS_ALG2_ENABLED
#else
#  define STM32WB_BLE_CS_ALG2             STM32WB_SHCI_BLE_INIT_OPT_CS_ALG2_DISABLED
#endif

#ifdef CONFIG_STM32WB_BLE_POWER_CLASS_1
#  define STM32WB_BLE_POWER_CLASS         STM32WB_SHCI_BLE_INIT_OPT_POWER_CLASS_1
#else
#  define STM32WB_BLE_POWER_CLASS         STM32WB_SHCI_BLE_INIT_OPT_POWER_CLASS_2_3
#endif

#define STM32WB_BLE_INIT_OPTIONS \
  (STM32WB_BLE_C2HOST | STM32WB_BLE_SVC_CHANGED_CHAR | \
   STM32WB_BLE_DEVICE_NAME_MODE | STM32WB_BLE_CS_ALG2 | \
   STM32WB_BLE_POWER_CLASS)

#ifdef CONFIG_STM32WB_BLE_AGC_RSSI_IMPROVED
#  define STM32WB_BLE_RXMOD_AGC_RSSI      STM32WB_SHCI_BLE_INIT_RXMOD_AGC_RSSI_IMPROVED
#else
#  define STM32WB_BLE_RXMOD_AGC_RSSI      STM32WB_SHCI_BLE_INIT_RXMOD_AGC_RSSI_LEGACY
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32wb_blehci_driveropen(struct bt_driver_s *btdev);
static int stm32wb_blehci_driversend(struct bt_driver_s *btdev,
                                     enum bt_buf_type_e type,
                                     void *data, size_t len);
static int stm32wb_blehci_rxevt(struct stm32wb_mbox_evt_s *evt);
static void stm32wb_blehci_bleinit(void);
static int stm32wb_blehci_driverinitialize(void);
static void stm32wb_blehci_drvinitworker(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_driver_s g_blehci_driver =
{
  .head_reserve = 0,
  .open         = stm32wb_blehci_driveropen,
  .send         = stm32wb_blehci_driversend
};

static sem_t  g_excl_sem = SEM_INITIALIZER(1);
struct work_s g_drv_init_work;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_blehci_driveropen
 ****************************************************************************/

static int stm32wb_blehci_driveropen(struct bt_driver_s *btdev)
{
  return 0;
}

/****************************************************************************
 * Name: stm32wb_blehci_driversend
 ****************************************************************************/

static int stm32wb_blehci_driversend(struct bt_driver_s *btdev,
                                     enum bt_buf_type_e type,
                                     void *data, size_t len)
{
  int ret = -EIO;

  if (type == BT_CMD || type == BT_ACL_OUT)
    {
      wlinfo("passing type %s to mailbox driver\n",
             (type == BT_CMD) ? "CMD" : "ACL");

      /* Ensure non-concurrent access */

      ret = nxsem_wait_uninterruptible(&g_excl_sem);
      if (ret < 0)
        {
          return ret;
        }

      if (type == BT_CMD)
        {
          ret = stm32wb_mbox_blecmd(data, len);
        }
      else
        {
          ret = stm32wb_mbox_bleacl(data, len);
        }

      nxsem_post(&g_excl_sem);
    }

  return ret < 0 ? ret : (int)len;
}

/****************************************************************************
 * Name: stm32wb_blehci_rxevt
 ****************************************************************************/

static int stm32wb_blehci_rxevt(struct stm32wb_mbox_evt_s *evt)
{
  size_t len;

  switch (evt->type)
    {
      case STM32WB_MBOX_HCIEVT:
        len = sizeof(evt->evt_hdr) + evt->evt_hdr.len;
        if (evt->evt_hdr.evt == BT_HCI_EVT_CMD_COMPLETE)
          {
            wlinfo("received CMD_COMPLETE from mailbox "
                   "(opcode: 0x%x, status: 0x%x)\n",
                   *(uint16_t *)((uint8_t *)&evt->evt_hdr + 3),
                   *((uint8_t *)&evt->evt_hdr + 5));
          }
        else
          {
            wlinfo("received HCI EVT from mailbox "
                   "(evt: %d, len: %zu)\n", evt->evt_hdr.evt, len);
          }

        bt_netdev_receive(&g_blehci_driver, BT_EVT, &evt->evt_hdr, len);
        break;

      case STM32WB_MBOX_HCIACL:
        wlinfo("received HCI ACL from mailbox (handle: %d)\n",
               evt->acl_hdr.handle);
        len = sizeof(evt->acl_hdr) + evt->acl_hdr.len;

        bt_netdev_receive(&g_blehci_driver, BT_ACL_IN, &evt->acl_hdr, len);
        break;

      case STM32WB_MBOX_SYSEVT:
        wlinfo("received SYS event from mailbox (evt: %d)\n",
               evt->evt_hdr.evt);
        if (evt->evt_hdr.evt == STM32WB_SHCI_ASYNC_EVT &&
            *(uint16_t *)(&evt->evt_hdr + 1) == STM32WB_SHCI_ASYNC_EVT_C2RDY)
          {
            stm32wb_blehci_bleinit();
          }
        break;

      case STM32WB_MBOX_SYSACK:

        /* CPU2 Ready is the only expected response */

        DEBUGASSERT(evt->evt_hdr.evt == STM32WB_SHCI_ACK_EVT_C2RDY);

        if (evt->evt_hdr.evt == STM32WB_SHCI_ACK_EVT_C2RDY)
          {
            wlinfo("system command ACK response");

            /* Make driver initialisation in low priority work queue */

            work_queue(LPWORK, &g_drv_init_work,
                       stm32wb_blehci_drvinitworker, NULL, 0);
          }
        break;

      default:
        break;
    }

  return 0;
}

/****************************************************************************
 * Name: stm32wb_blehci_bleinit
 ****************************************************************************/

static void stm32wb_blehci_bleinit(void)
{
  /* Prepare BLE configuration */

  struct stm32wb_shci_ble_init_cfg_s params =
  {
    .ble_buf =              NULL,
    .ble_buf_size =         0,
    .gatt_attr_num =        CONFIG_STM32WB_BLE_GATT_MAX_ATTR_NUM,
    .gatt_srv_num =         CONFIG_STM32WB_BLE_GATT_MAX_SVC_NUM,
    .gatt_attr_buf_size =   CONFIG_STM32WB_BLE_GATT_ATTR_BUF_SIZE,
    .max_conn =             CONFIG_STM32WB_BLE_MAX_CONN,
    .dle_enable =           CONFIG_STM32WB_BLE_DLE,
    .prep_write_op_num =    STM32WB_BLE_PREP_WRITE_NUM,
    .mem_block_num =        STM32WB_C2_MEM_BLOCK_NUM,
    .att_max_mtu_size =     CONFIG_STM32WB_BLE_MAX_ATT_MTU,
    .slave_sca =            CONFIG_STM32WB_BLE_SLAVE_SCA,
    .master_sca_range =     CONFIG_STM32WB_BLE_MASTER_SCA,
    .ls_clock_source =      CONFIG_STM32WB_BLE_LS_CLK_SRC,
    .conn_event_length =    CONFIG_STM32WB_BLE_MAX_CONN_EVT_LENGTH,
    .hse_startup =          CONFIG_STM32WB_BLE_HSE_STARTUP,
    .viterbi_enable =       CONFIG_STM32WB_BLE_VITERBI,
    .options =              STM32WB_BLE_INIT_OPTIONS,
    .hw_version =           0,
    .max_initor_coc_num =   CONFIG_STM32WB_BLE_MAX_INITOR_COC_NUM,
    .tx_power_min =         CONFIG_STM32WB_BLE_MIN_TX_POWER,
    .tx_power_max =         CONFIG_STM32WB_BLE_MAX_TX_POWER,
    .rx_model_config =      STM32WB_BLE_RXMOD_AGC_RSSI
  };

  /* Initialise BLE */

  stm32wb_mbox_bleinit(&params);
}

/****************************************************************************
 * Name: stm32wb_blehci_driverinitialize
 ****************************************************************************/

static int stm32wb_blehci_driverinitialize(void)
{
  int ret = 0;

#ifdef CONFIG_UART_BTH4
  /* Register UART BT H4 device */

  ret = uart_bth4_register(CONFIG_STM32WB_BLE_TTY_NAME, &g_blehci_driver);
  if (ret < 0)
    {
      wlerr("bt_bth4_register error: %d\n", ret);
      return ret;
    }
#elif defined(CONFIG_NET_BLUETOOTH)
  /* Register network device */

  ret = bt_netdev_register(&g_blehci_driver);
  if (ret < 0)
    {
      wlerr("bt_netdev_register error: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: stm32wb_blehci_drvinitworker
 ****************************************************************************/

static void stm32wb_blehci_drvinitworker(void *arg)
{
  stm32wb_blehci_driverinitialize();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_blehci_initialize
 *
 * Description:
 *   Initialize and register BLE HCI driver which interfaces a BLE host
 *   stack to a BLE controller running on CPU2 via HCI protocol.  Driver
 *   registration occurs later when CPU2 notifies its ready status.
 *
 ****************************************************************************/

void stm32wb_blehci_initialize(void)
{
  /* Initialize mbox internal data structures and set
   * event receive handler.
   */

  stm32wb_mboxinitialize(stm32wb_blehci_rxevt);

  /* Enable communication hardware and bootup CPU2 */

  stm32wb_mboxenable();
}
