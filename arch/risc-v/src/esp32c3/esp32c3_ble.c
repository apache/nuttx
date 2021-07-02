/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_ble.c
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

#include <sys/types.h>

#include <sys/socket.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#if defined(CONFIG_UART_BTH4)
  #include <nuttx/serial/uart_bth4.h>
#endif

#include "esp32c3_ble_adapter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BLE packet buffer max number */

#define BLE_BUF_NUM       CONFIG_ESP32C3_BLE_PKTBUF_NUM

/* BLE packet buffer max size */

#define BLE_BUF_SIZE      1024

/* Low-priority work queue process RX/TX */

#define BLE_WORK          LPWORK

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3_ble_priv_s
{
  struct bt_driver_s drv;         /* NuttX BT/BLE driver data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32c3_ble_open(struct bt_driver_s *drv);
static int esp32c3_ble_send(struct bt_driver_s *drv,
                            enum bt_buf_type_e type,
                            void *data, size_t len);
static void esp32c3_ble_close(struct bt_driver_s *drv);

static void esp32c3_ble_send_ready(void);
static int esp32c3_ble_recv_cb(uint8_t *data, uint16_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32c3_ble_priv_s g_ble_priv =
{
  .drv =
    {
      .head_reserve = H4_HEADER_SIZE,
      .open         = esp32c3_ble_open,
      .send         = esp32c3_ble_send,
      .close        = esp32c3_ble_close
    }
};

static esp_vhci_host_callback_t vhci_host_cb =
{
  .notify_host_send_available = esp32c3_ble_send_ready,
  .notify_host_recv           = esp32c3_ble_recv_cb
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_ble_send_ready
 *
 * Description:
 *   If the controller could send HCI comand will callback this function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3_ble_send_ready(void)
{
}

/****************************************************************************
 * Name: esp32c3_ble_recv_cb
 *
 * Description:
 *   BLE receive callback function when BLE hardware receive packet
 *
 * Input Parameters:
 *   data - BLE packet data pointer
 *   len  - BLE packet length
 *
 * Returned Value:
 *   0 on success or a negated value on failure.
 *
 ****************************************************************************/

static int esp32c3_ble_recv_cb(uint8_t *data, uint16_t len)
{
  int ret;
  bool valid;
  enum bt_buf_type_e type;
  struct esp32c3_ble_priv_s *priv = &g_ble_priv;

  switch (data[0])
    {
      case H4_EVT:
        type = BT_EVT;
        valid = true;
        break;
      case H4_ACL:
        type = BT_ACL_IN;
        valid = true;
        break;
      case H4_ISO:
        type = BT_ISO_IN;
        valid = true;
        break;
      default:
        valid = false;
        break;
    }

  if (!valid)
    {
      ret = ERROR;
    }
  else
    {
      /* send packet to host */

      ret = bt_netdev_receive(&priv->drv, type,
                              &data[H4_HEADER_SIZE],
                              len - H4_HEADER_SIZE);
      if (ret < 0)
        {
          wlerr("Failed to receive ret=%d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_ble_send
 *
 * Description:
 *   ESP32-C3 BLE send callback function for BT driver.
 *
 * Input Parameters:
 *   drv  - BT driver pointer
 *   type - BT packet type
 *   data - BT packte data buffer pointer
 *   len  - BT packte length
 *
 * Returned Value:
 *   Sent bytes on success or a negated value on failure.
 *
 ****************************************************************************/

static int esp32c3_ble_send(struct bt_driver_s *drv,
                            enum bt_buf_type_e type,
                            void *data, size_t len)
{
  uint8_t *hdr = (uint8_t *)data - drv->head_reserve;

  if ((len + H4_HEADER_SIZE) > BLE_BUF_SIZE)
    {
      return -EINVAL;
    }

  if (type == BT_CMD)
    {
      *hdr = H4_CMD;
    }
  else if (type == BT_ACL_OUT)
    {
      *hdr = H4_ACL;
    }
  else if (type == BT_ISO_OUT)
    {
      *hdr = H4_ISO;
    }
  else
    {
      return -EINVAL;
    }

  if (esp32c3_vhci_host_check_send_available())
    {
      esp32c3_vhci_host_send_packet(hdr, len + drv->head_reserve);
    }

  return len;
}

/****************************************************************************
 * Name: esp32c3_ble_close
 *
 * Description:
 *   ESP32-C3 BLE close callback function for BT driver.
 *
 * Input Parameters:
 *   drv  - BT driver pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32c3_ble_close(struct bt_driver_s *drv)
{
}

/****************************************************************************
 * Name: esp32c3_ble_open
 *
 * Description:
 *   ESP32-C3 BLE open callback function for BT driver.
 *
 * Input Parameters:
 *   drv - BT driver pointer
 *
 * Returned Value:
 *   OK on success or a negated value on failure.
 *
 ****************************************************************************/

static int esp32c3_ble_open(struct bt_driver_s *drv)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_ble_initialize
 *
 * Description:
 *   Init BT controller
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   success or fail
 *
 ****************************************************************************/

int esp32c3_ble_initialize(void)
{
  int ret;

  ret = esp32c3_bt_controller_init();
  if (ret)
    {
      wlerr("Failed to initialize BLE ret=%d\n", ret);
      return ERROR;
    }

  ret = esp32c3_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
    {
      wlerr("Failed to Enable BLE ret=%d\n", ret);
      return ERROR;
    }

  ret = esp32c3_vhci_register_callback(&vhci_host_cb);
  if (ret)
    {
      wlerr("Failed to register BLE callback ret=%d\n", ret);
      return ERROR;
    }

#if defined(CONFIG_UART_BTH4)
  ret = uart_bth4_register(CONFIG_ESP32C3_BLE_TTY_NAME, &g_ble_priv.drv);
#else
  ret = bt_netdev_register(&g_ble_priv.drv);
#endif
  if (ret < 0)
    {
      wlerr("bt_netdev_register error: %d\n", ret);
      return ret;
    }

  return OK;
}
