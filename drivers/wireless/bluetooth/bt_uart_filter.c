/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_filter.c
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

#include <string.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "bt_uart_filter.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_bt_uart_filter_bt_evt_table[] =
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

static const uint8_t g_bt_uart_filter_ble_evt_table[] =
{
  BT_HCI_EVT_LE_META_EVENT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool bt_uart_filter_set_handle(FAR struct bt_uart_filter_s *filter,
                                      int old, int new)
{
  int i;

  for (i = 0; i < BT_UART_FILTER_CONN_COUNT; i++)
    {
      if (filter->handle[i] == old)
        {
          filter->handle[i] = new & 0xfff;
          return true;
        }
    }

  return false;
}

static bool bt_uart_filter_alloc_handle(FAR struct bt_uart_filter_s *filter,
                                        int handle)
{
  return bt_uart_filter_set_handle(filter, 0, handle);
}

static bool bt_uart_filter_free_handle(FAR struct bt_uart_filter_s *filter,
                                       int handle)
{
  return bt_uart_filter_set_handle(filter, handle, 0);
}

static bool bt_uart_filter_has_handle(FAR struct bt_uart_filter_s *filter,
                                      int handle)
{
  return bt_uart_filter_set_handle(filter, handle, handle);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool bt_uart_filter_forward_recv(FAR struct bt_uart_filter_s *filter,
                                 FAR char *buffer, size_t buflen)
{
  FAR const uint8_t *evt_table;
  int size;
  int i;

  if (buffer[0] == H4_EVT)
    {
      if (filter->type == BT_UART_FILTER_TYPE_BLE)
        {
          evt_table = g_bt_uart_filter_bt_evt_table;
          size = sizeof(g_bt_uart_filter_bt_evt_table);
        }
      else
        {
          evt_table = g_bt_uart_filter_ble_evt_table;
          size = sizeof(g_bt_uart_filter_ble_evt_table);
        }

      for (i = 0; i < size; i++)
        {
          if (buffer[1] == evt_table[i])
            {
              return false;
            }
        }

      switch (buffer[1])
        {
          case BT_HCI_EVT_LE_META_EVENT:
            if (buffer[3] == BT_HCI_EVT_LE_CONN_COMPLETE)
              {
                FAR struct bt_hci_evt_le_conn_complete_s *evt;

                evt = (FAR void *)&buffer[4];
                if (evt->status == 0)
                  {
                    return bt_uart_filter_alloc_handle(filter, evt->handle);
                  }
              }
            else if (buffer[3] == BT_HCI_EVT_LE_ENH_CONN_COMPLETE)
              {
                FAR struct bt_hci_evt_le_enh_conn_complete_s *evt;

                evt = (FAR void *)&buffer[4];
                if (evt->status == 0)
                  {
                    return bt_uart_filter_alloc_handle(filter, evt->handle);
                  }
              }
            break;
          case BT_HCI_EVT_CONN_COMPLETE:
            {
              FAR struct bt_hci_evt_conn_complete_s *evt;

              evt = (FAR void *)&buffer[3];
              if (evt->status == 0)
                {
                  return bt_uart_filter_alloc_handle(filter, evt->handle);
                }
            }
            break;
          case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
            {
              FAR struct bt_hci_evt_num_completed_packets_s *evt;

              evt = (FAR void *)&buffer[3];
              return bt_uart_filter_has_handle(filter, evt->h[0].handle);
            }
            break;
          case BT_HCI_EVT_REMOTE_VERSION_INFO:
            {
              FAR struct bt_hci_evt_remote_version_info_s *evt;

              evt = (FAR void *)&buffer[3];
              return bt_uart_filter_has_handle(filter, evt->handle);
            }
            break;
          case BT_HCI_EVT_ENCRYPT_CHANGE:
            {
              FAR struct bt_hci_evt_encrypt_change_s *evt;

              evt = (FAR void *)&buffer[3];
              return bt_uart_filter_has_handle(filter, evt->handle);
            }
            break;
          case BT_HCI_EVT_ENCRYPT_KEY_REFRESH_COMPLETE:
            {
              FAR struct bt_hci_evt_encrypt_key_refresh_complete_s *evt;

              evt = (FAR void *)&buffer[3];
              return bt_uart_filter_has_handle(filter, evt->handle);
            }
            break;
          case BT_HCI_EVT_DISCONN_COMPLETE:
            {
              FAR struct bt_hci_evt_disconn_complete_s *evt;

              evt = (FAR void *)&buffer[3];
              return bt_uart_filter_free_handle(filter, evt->handle);
            }
            break;
          default:
            break;
        }
    }
  else if (buffer[0] == H4_ACL)
    {
      FAR struct bt_hci_acl_hdr_s *acl = (FAR void *)&buffer[1];

      return bt_uart_filter_has_handle(filter, acl->handle);
    }

  return true;
}

bool bt_uart_filter_forward_send(FAR struct bt_uart_filter_s *filter,
                                 FAR char *buffer, size_t buflen)
{
  uint16_t opcode;

  if (buffer[0] == H4_CMD)
    {
      opcode = (uint16_t)buffer[2] << 8 | (uint16_t)buffer[1];

      switch (opcode)
        {
          case BT_HCI_OP_SET_EVENT_MASK:
            {
              /* Override the all event bits roughly to avoid the
               * bt/ble host specific mask deliver to controller.
               */

              memset(&buffer[4], 0xff, 7);
              buffer[11] = 0x3f;
            }
            break;

          default:
            break;
        }
    }

  return true;
}

void bt_uart_filter_init(FAR struct bt_uart_filter_s *filter, int type)
{
  memset(filter->handle, 0, sizeof(filter->handle));
  filter->type = type;
}
