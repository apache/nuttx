/****************************************************************************
 * drivers/wireless/bluetooth/bt_bridge.c
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

#include <debug.h>
#include <stdatomic.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/snoop.h>

#include <nuttx/wireless/bluetooth/bt_bridge.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_FILTER_CONN_COUNT   16
#define BT_FILTER_OPCODE_COUNT 16

#define BT_FILTER_TYPE_BT      0
#define BT_FILTER_TYPE_BLE     1
#define BT_FILTER_TYPE_COUNT   2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bt_filter_s
{
  int      type;
  uint16_t opcode[BT_FILTER_OPCODE_COUNT];
  uint16_t handle[BT_FILTER_CONN_COUNT];
};

struct bt_bridge_device_s
{
  struct bt_driver_s        driver;
  struct bt_filter_s        filter;
  FAR struct bt_bridge_s   *bridge;
};

struct bt_bridge_s
{
  struct bt_bridge_device_s device[BT_FILTER_TYPE_COUNT];
  FAR struct bt_driver_s   *driver;
#ifdef CONFIG_BLUETOOTH_BRIDGE_BTSNOOP
  FAR struct snoop_s       *snoop;
#endif /* CONFIG_BLUETOOTH_BRIDGE_BTSNOOP */
  atomic_uint               refs;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_bt_filter_bt_evt_table[] =
{
  BT_HCI_EVT_INQUIRY_COMPLETE,
  BT_HCI_EVT_CONN_COMPLETE,
  BT_HCI_EVT_CONN_REQUEST,
  BT_HCI_EVT_AUTH_COMPLETE,
  BT_HCI_EVT_REMOTE_NAME_REQ_COMPLETE,
  BT_HCI_EVT_REMOTE_FEATURES,
  BT_HCI_EVT_ROLE_CHANGE,
  BT_HCI_EVT_PIN_CODE_REQ,
  BT_HCI_EVT_LINK_KEY_NOTIFY,
  BT_HCI_EVT_LINK_KEY_REQ,
  BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI,
  BT_HCI_EVT_REMOTE_EXT_FEATURES,
  BT_HCI_EVT_SYNC_CONN_COMPLETE,
  BT_HCI_EVT_EXTENDED_INQUIRY_RESULT,
  BT_HCI_EVT_IO_CAPA_RESP,
  BT_HCI_EVT_IO_CAPA_REQ,
  BT_HCI_EVT_SSP_COMPLETE,
  BT_HCI_EVT_USER_CONFIRM_REQ,
  BT_HCI_EVT_USER_PASSKEY_REQ,
  BT_HCI_EVT_USER_PASSKEY_NOTIFY,
  BT_HCI_EVT_PAGE_SCAN_REP_MODE_CHNG,
  BT_HCI_EVT_CONN_PKT_TYPE_CHANGE,
  BT_HCI_EVT_READ_CLOCK_OFF_COMP,
  BT_HCI_EVT_MAX_SLOTS_CHANGED,
  BT_HCI_EVT_MODE_CHANGE,
  BT_HCI_EVT_QOS_SETUP_COMP,
  BT_HCI_EVT_LINK_SUPER_TOUT_CHANGED,
};

static const uint8_t g_bt_filter_ble_evt_table[] =
{
  BT_HCI_EVT_LE_META_EVENT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool bt_filter_set(FAR uint16_t *array, int size, uint16_t old,
                          uint16_t new, uint16_t mask)
{
  int i;

  for (i = 0; i < size; i++)
    {
      if (array[i] == old)
        {
          array[i] = new & mask;
          return true;
        }
    }

  return false;
}

static bool bt_filter_set_handle(FAR uint16_t *handle, int size,
                                 uint16_t old, uint16_t new)
{
  return bt_filter_set(handle, size, old, new, 0xfff);
}

static bool bt_filter_alloc_handle(FAR struct bt_filter_s *filter,
                                   uint16_t handle)
{
  return bt_filter_set_handle(filter->handle, BT_FILTER_CONN_COUNT,
                              0, handle);
}

static bool bt_filter_free_handle(FAR struct bt_filter_s *filter,
                                  uint16_t handle)
{
  return bt_filter_set_handle(filter->handle, BT_FILTER_CONN_COUNT,
                              handle, 0);
}

static bool bt_filter_has_handle(FAR struct bt_filter_s *filter,
                                 uint16_t handle)
{
  return bt_filter_set_handle(filter->handle, BT_FILTER_CONN_COUNT,
                              handle, handle);
}

static bool bt_filter_set_opcode(FAR uint16_t *opcode, int size,
                                 uint16_t old, uint16_t new)
{
  return bt_filter_set(opcode, size, old, new, 0xffff);
}

static bool bt_filter_alloc_opcode(FAR struct bt_filter_s *filter,
                                   uint16_t opcode)
{
  return bt_filter_set_opcode(filter->opcode, BT_FILTER_OPCODE_COUNT,
                              0, opcode);
}

static bool bt_filter_free_opcode(FAR struct bt_filter_s *filter,
                                  uint16_t opcode)
{
  return bt_filter_set_opcode(filter->opcode, BT_FILTER_OPCODE_COUNT,
                              opcode, 0);
}

static bool bt_filter_can_recv(FAR struct bt_filter_s *filter,
                               enum bt_buf_type_e type,
                               FAR const uint8_t *buffer, size_t buflen)
{
  FAR const uint8_t *evt_table;
  int size;
  int i;

  if (type == BT_EVT)
    {
      if (filter->type == BT_FILTER_TYPE_BLE)
        {
          evt_table = g_bt_filter_bt_evt_table;
          size = sizeof(g_bt_filter_bt_evt_table);
        }
      else
        {
          evt_table = g_bt_filter_ble_evt_table;
          size = sizeof(g_bt_filter_ble_evt_table);
        }

      for (i = 0; i < size; i++)
        {
          if (buffer[0] == evt_table[i])
            {
              return false;
            }
        }

      switch (buffer[0])
        {
        case BT_HCI_EVT_LE_META_EVENT:
          if (buffer[2] == BT_HCI_EVT_LE_CONN_COMPLETE)
            {
              FAR struct bt_hci_evt_le_conn_complete_s *evt;

              evt = (FAR void *)&buffer[3];
              if (evt->status == 0)
                {
                  return bt_filter_alloc_handle(filter, evt->handle);
                }
            }
          else if (buffer[2] == BT_HCI_EVT_LE_ENH_CONN_COMPLETE)
            {
              FAR struct bt_hci_evt_le_enh_conn_complete_s *evt;

              evt = (FAR void *)&buffer[3];
              if (evt->status == 0)
                {
                  return bt_filter_alloc_handle(filter, evt->handle);
                }
            }
          break;
        case BT_HCI_EVT_CONN_COMPLETE:
          {
            FAR struct bt_hci_evt_conn_complete_s *evt;

            evt = (FAR void *)&buffer[2];
            if (evt->status == 0)
              {
                return bt_filter_alloc_handle(filter, evt->handle);
              }
          }
          break;
        case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
          {
            FAR struct bt_hci_evt_num_completed_packets_s *evt;

            evt = (FAR void *)&buffer[2];
            return bt_filter_has_handle(filter, evt->h[0].handle);
          }
          break;
        case BT_HCI_EVT_REMOTE_VERSION_INFO:
          {
            FAR struct bt_hci_evt_remote_version_info_s *evt;

            evt = (FAR void *)&buffer[2];
            return bt_filter_has_handle(filter, evt->handle);
          }
          break;
        case BT_HCI_EVT_ENCRYPT_CHANGE:
          {
            FAR struct bt_hci_evt_encrypt_change_s *evt;

            evt = (FAR void *)&buffer[2];
            return bt_filter_has_handle(filter, evt->handle);
          }
          break;
        case BT_HCI_EVT_ENCRYPT_KEY_REFRESH_COMPLETE:
          {
            FAR struct bt_hci_evt_encrypt_key_refresh_complete_s *evt;

            evt = (FAR void *)&buffer[2];
            return bt_filter_has_handle(filter, evt->handle);
          }
          break;
        case BT_HCI_EVT_DISCONN_COMPLETE:
          {
            FAR struct bt_hci_evt_disconn_complete_s *evt;

            evt = (FAR void *)&buffer[2];
            return bt_filter_free_handle(filter, evt->handle);
          }
          break;
        case BT_HCI_EVT_CMD_COMPLETE:
          {
            FAR struct hci_evt_cmd_complete_s *evt;
            uint8_t ogf;

            evt = (FAR void *)&buffer[2];
            ogf = evt->opcode >> 10;

            if (BT_OGF_BASEBAND == ogf || BT_OGF_LINK_CTRL == ogf
                || BT_OGF_INFO == ogf || BT_OGF_LE == ogf)
              {
                return bt_filter_free_opcode(filter, evt->opcode);
              }
            else if (BT_OGF_LINK_POLICY == ogf || BT_OGF_STATUS == ogf)
              {
                if (filter->type == BT_FILTER_TYPE_BLE)
                  {
                    return false;
                  }
              }
          }
          break;
        case BT_HCI_EVT_CMD_STATUS:
          {
            FAR struct bt_hci_evt_cmd_status_s *stat;
            uint8_t ogf;

            stat = (FAR void *)&buffer[2];
            ogf = stat->opcode >> 10;

            if (BT_OGF_BASEBAND == ogf || BT_OGF_LINK_CTRL == ogf
                || BT_OGF_INFO == ogf || BT_OGF_LE == ogf)
              {
                return bt_filter_free_opcode(filter, stat->opcode);
              }

            if (BT_OGF_LINK_POLICY == ogf || BT_OGF_STATUS == ogf)
              {
                if (filter->type == BT_FILTER_TYPE_BLE)
                  {
                    return false;
                  }
              }
          }
          break;
        }
    }
  else if (type == BT_ACL_IN)
    {
      FAR struct bt_hci_acl_hdr_s *acl = (FAR void *)&buffer[0];

      return bt_filter_has_handle(filter, acl->handle);
    }

  return true;
}

static bool bt_filter_can_send(FAR struct bt_filter_s *filter,
                               enum bt_buf_type_e type,
                               FAR uint8_t *buffer, size_t buflen)
{
  uint16_t opcode;
  uint8_t ogf;
  int i;

  if (type == BT_CMD)
    {
      opcode = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[0];
      ogf = buffer[1] >> 2;

      switch (opcode)
        {
        case BT_HCI_OP_SET_EVENT_MASK:
          {
            /* Override the all event bits roughly to avoid the
             * bt/ble host specific mask deliver to controller.
             */

            memset(&buffer[3], 0xff, 7);
            buffer[10] = 0x3f;
          }
          break;
        }

      if (BT_OGF_BASEBAND == ogf || BT_OGF_LINK_CTRL == ogf
          || BT_OGF_INFO == ogf || BT_OGF_LE == ogf)
        {
          if (!bt_filter_alloc_opcode(filter, opcode))
            {
              nerr("Unable to set opcode 0x%04x.\n", opcode);

              for (i = 0; i < BT_FILTER_OPCODE_COUNT; i++)
                {
                  nerr("PENDING opcode: %04x.\n", filter->opcode[i]);
                }

              memset(filter->opcode, 0, sizeof(filter->opcode));

              bt_filter_alloc_opcode(filter, opcode);
            }
        }
    }

  return true;
}

static int bt_bridge_open(FAR struct bt_driver_s *drv)
{
  FAR struct bt_bridge_device_s *device =
          (FAR struct bt_bridge_device_s *)drv;
  FAR struct bt_bridge_s *bridge = device->bridge;
  FAR struct bt_driver_s *driver = bridge->driver;

  if (atomic_fetch_add(&bridge->refs, 1) == 0)
    {
      int ret  = driver->open(driver);
      if (ret < 0)
        {
          atomic_fetch_sub(&bridge->refs, 1);
        }

      return ret;
    };

  return OK;
}

static int bt_bridge_send(FAR struct bt_driver_s *drv,
                          enum bt_buf_type_e type,
                          FAR void *data, size_t len)
{
  FAR struct bt_bridge_device_s *device =
          (FAR struct bt_bridge_device_s *)drv;
  FAR struct bt_bridge_s *bridge = device->bridge;
  FAR struct bt_driver_s *driver = bridge->driver;
  irqstate_t flags;

  flags = enter_critical_section();
  if (bt_filter_can_send(&device->filter, type, data, len))
    {
      leave_critical_section(flags);
#ifdef CONFIG_BLUETOOTH_BRIDGE_BTSNOOP
      snoop_dump(bridge->snoop, data - drv->head_reserve,
                 len + drv->head_reserve, 0,
                 SNOOP_DIRECTION_FLAG_SENT);
#endif /* CONFIG_BLUETOOTH_BRIDGE_BTSNOOP */

      return driver->send(driver, type, data, len);
    }

  leave_critical_section(flags);
  return 0;
}

static void bt_bridge_close(FAR struct bt_driver_s *drv)
{
  FAR struct bt_bridge_device_s *device =
          (FAR struct bt_bridge_device_s *)drv;
  FAR struct bt_bridge_s *bridge = device->bridge;
  FAR struct bt_driver_s *driver = bridge->driver;

  if (atomic_fetch_sub(&bridge->refs, 1) == 1)
    {
      driver->close(driver);
    };
}

static int bt_bridge_receive(FAR struct bt_driver_s *drv,
                             enum bt_buf_type_e type,
                             FAR void *data, size_t len)
{
  FAR struct bt_bridge_s *bridge = (FAR struct bt_bridge_s *)drv->priv;
  FAR struct bt_bridge_device_s *device;
  FAR struct bt_driver_s *driver;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();
  for (i = 0; i < BT_FILTER_TYPE_COUNT; i++)
    {
      device = &bridge->device[i];
      driver = &device->driver;
      if (bt_filter_can_recv(&device->filter, type, data, len))
        {
          int ret;

          leave_critical_section(flags);
          ret = bt_netdev_receive(driver, type, data, len);

#ifdef CONFIG_BLUETOOTH_BRIDGE_BTSNOOP
          snoop_dump(bridge->snoop, data - drv->head_reserve,
                     len + drv->head_reserve, 0,
                     SNOOP_DIRECTION_FLAG_RECV);
#endif /* CONFIG_BLUETOOTH_BRIDGE_BTSNOOP */
          return ret;
        }
    }

  leave_critical_section(flags);
  return 0;
}

static int bt_bridge_ioctl(FAR struct bt_driver_s *drv, int cmd,
                           unsigned long arg)
{
  FAR struct bt_bridge_device_s *device =
          (FAR struct bt_bridge_device_s *)drv;
  FAR struct bt_bridge_s *bridge = device->bridge;
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_BLUETOOTH_BRIDGE_BTSNOOP
    case SIOCBTSNOOPOPEN:
      {
        FAR const char *filename = (FAR const char *)((uintptr_t)arg);
        FAR struct snoop_s *snoop = (FAR struct snoop_s *)
                                    kmm_zalloc(sizeof(struct snoop_s));
        if (!snoop)
          {
            return -ENOMEM;
          }

        ret = snoop_open(snoop, filename, SNOOP_DATALINK_HCI_UART, true);
        if (ret == OK)
          {
            bridge->snoop = snoop;
          }
        else
          {
            kmm_free(snoop);
          }
        break;
      }

    case SIOCBTSNOOPCLOSE:
      {
        FAR struct snoop_s *snoop = bridge->snoop;

        bridge->snoop = NULL;
        ret = snoop_close(snoop);
        kmm_free(snoop);
        break;
      }
#endif /* CONFIG_BLUETOOTH_BRIDGE_BTSNOOP */

      /* hci ioctl operation */

    default:
      {
        if (bridge->driver->ioctl)
          {
            ret = bridge->driver->ioctl(drv, cmd, arg);
          }
        else
          {
            ret = -ENOTTY;
          }
      }
    }

  return ret;
}

static void bt_device_init(FAR struct bt_bridge_s *bridge,
                           FAR struct bt_driver_s **drv, uint8_t type)
{
  FAR struct bt_bridge_device_s *device = &bridge->device[type];
  FAR struct bt_driver_s *driver = &device->driver;

  device->bridge = bridge;
  device->filter.type = type;

  *drv = &device->driver;

  driver->head_reserve = bridge->driver->head_reserve;
  driver->open = bt_bridge_open;
  driver->send = bt_bridge_send;
  driver->close = bt_bridge_close;
  driver->ioctl = bt_bridge_ioctl;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_bridge_register
 *
 * Description:
 *   Register the Bluetooth BT/BLE dual mode bridge driver
 *
 ****************************************************************************/

int bt_bridge_register(FAR struct bt_driver_s *hcidrv,
                       FAR struct bt_driver_s **btdrv,
                       FAR struct bt_driver_s **bledrv)
{
  FAR struct bt_bridge_s *bridge;

  if (!hcidrv || !btdrv || !bledrv)
    {
      return -EINVAL;
    }

  bridge = (FAR struct bt_bridge_s *)kmm_zalloc(sizeof(struct bt_bridge_s));
  if (!bridge)
    {
      return -ENOMEM;
    }

  bridge->driver = hcidrv;

  hcidrv->receive = bt_bridge_receive;
  hcidrv->priv = bridge;

  bt_device_init(bridge, btdrv, BT_FILTER_TYPE_BT);
  bt_device_init(bridge, bledrv, BT_FILTER_TYPE_BLE);

  return OK;
}
