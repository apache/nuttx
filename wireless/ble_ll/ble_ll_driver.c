/****************************************************************************
 * wireless/ble_ll/ble_ll_driver.c
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

#include <nuttx/config.h>

#include <nuttx/queue.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "ble_ll_os.h"
#include "ble_ll.h"
#include "ble_ll_hci.h"
#include "ble_ll_tmr.h"
#include "ble_phy.h"
#include "hci_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HCI command buffer: header and maximum parameter length */

#define BLE_LL_CMD_SIZE      (sizeof(struct ble_hci_cmd) + 255)

/* The command buffer is reused for the command complete event and freed
 * after the event is passed to the host.  The second buffer accepts the
 * next command (or HCI_Host_Number_Of_Completed_Packets) in the meantime.
 */

#define BLE_LL_CMD_COUNT     2

#define BLE_LL_ACL_OUT_SIZE  (BLE_HCI_DATA_HDR_SZ + CONFIG_BLE_LL_ACL_SIZE)
#define BLE_LL_ACL_IN_SIZE   (BLE_HCI_DATA_HDR_SZ + BLE_LL_DATA_PDU_MAX_PYLD)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* HCI buffer pool */

struct ble_ll_hci_pool_s
{
  FAR uint8_t *base;     /* Buffer storage */
  size_t       size;     /* Size of one buffer */
  int          count;    /* Number of buffers */
  sq_queue_t   free;     /* Free buffers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ble_ll_drv_open(FAR struct bt_driver_s *btdev);
static int ble_ll_drv_send(FAR struct bt_driver_s *btdev,
                           enum bt_buf_type_e type,
                           FAR void *data, size_t len);

static int ble_ll_lower_rx_start(FAR uint8_t *rxbuf, uint8_t chan,
                                 FAR struct ble_mbuf_hdr *hdr);
static int ble_ll_lower_rx_end(FAR uint8_t *rxbuf,
                               FAR struct ble_mbuf_hdr *hdr);
static void ble_ll_lower_wfr_exp(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_driver_s g_ble_ll_btdev =
{
  .open = ble_ll_drv_open,
  .send = ble_ll_drv_send,
};

static const struct ble_ll_lowercb_s g_ble_ll_lowercb =
{
  .rx_start   = ble_ll_lower_rx_start,
  .rx_end     = ble_ll_lower_rx_end,
  .wfr_exp    = ble_ll_lower_wfr_exp,
  .tmr_exp    = ble_ll_tmr_isr,
  .rng_sample = ble_ll_rand_sample,
  .state_get  = ble_ll_state_get,
};

static uint32_t g_ble_ll_cmd_buf[BLE_LL_CMD_COUNT]
                                [ALIGN_UP(BLE_LL_CMD_SIZE, 4) / 4];
static uint32_t g_ble_ll_evt_buf[CONFIG_BLE_LL_EVT_COUNT]
                                [ALIGN_UP(CONFIG_BLE_LL_EVT_SIZE, 4) / 4];
static uint32_t g_ble_ll_evt_lo_buf[CONFIG_BLE_LL_EVT_DISCARDABLE_COUNT]
                                   [ALIGN_UP(CONFIG_BLE_LL_EVT_SIZE, 4) / 4];

static struct ble_ll_hci_pool_s g_ble_ll_cmd_pool =
{
  (FAR uint8_t *)g_ble_ll_cmd_buf, sizeof(g_ble_ll_cmd_buf[0]),
  BLE_LL_CMD_COUNT
};

static struct ble_ll_hci_pool_s g_ble_ll_evt_pool =
{
  (FAR uint8_t *)g_ble_ll_evt_buf, sizeof(g_ble_ll_evt_buf[0]),
  CONFIG_BLE_LL_EVT_COUNT
};

static struct ble_ll_hci_pool_s g_ble_ll_evt_lo_pool =
{
  (FAR uint8_t *)g_ble_ll_evt_lo_buf, sizeof(g_ble_ll_evt_lo_buf[0]),
  CONFIG_BLE_LL_EVT_DISCARDABLE_COUNT
};

/* Flat buffer for ACL data sent to the host.  Used only from the Link
 * Layer work queue.
 */

static uint8_t g_ble_ll_acl_in[BLE_LL_ACL_IN_SIZE];

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR struct ble_ll_lower_s *g_ble_ll_lower;
FAR struct kwork_wqueue_s *g_ble_ll_wq;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_pool_init
 ****************************************************************************/

static void ble_ll_pool_init(FAR struct ble_ll_hci_pool_s *pool)
{
  int i;

  sq_init(&pool->free);
  for (i = 0; i < pool->count; i++)
    {
      sq_addlast((FAR sq_entry_t *)(pool->base + i * pool->size),
                 &pool->free);
    }
}

/****************************************************************************
 * Name: ble_ll_pool_alloc
 ****************************************************************************/

static FAR void *ble_ll_pool_alloc(FAR struct ble_ll_hci_pool_s *pool)
{
  irqstate_t flags;
  FAR void *buf;

  flags = enter_critical_section();
  buf = sq_remfirst(&pool->free);
  leave_critical_section(flags);

  return buf;
}

/****************************************************************************
 * Name: ble_ll_pool_free
 ****************************************************************************/

static bool ble_ll_pool_free(FAR struct ble_ll_hci_pool_s *pool,
                             FAR void *buf)
{
  FAR uint8_t *ptr = buf;
  irqstate_t flags;

  if (ptr < pool->base || ptr >= pool->base + pool->count * pool->size)
    {
      return false;
    }

  DEBUGASSERT((ptr - pool->base) % pool->size == 0);

  flags = enter_critical_section();
  sq_addlast((FAR sq_entry_t *)buf, &pool->free);
  leave_critical_section(flags);

  return true;
}

/****************************************************************************
 * Name: ble_ll_drv_open
 ****************************************************************************/

static int ble_ll_drv_open(FAR struct bt_driver_s *btdev)
{
  return OK;
}

/****************************************************************************
 * Name: ble_ll_drv_send
 *
 * Description:
 *   Receive an HCI command or ACL data packet from the host.
 *
 ****************************************************************************/

static int ble_ll_drv_send(FAR struct bt_driver_s *btdev,
                           enum bt_buf_type_e type,
                           FAR void *data, size_t len)
{
  FAR struct iob_s *iob;
  FAR uint8_t *cmd;
  int ret;

  switch (type)
    {
      case BT_CMD:
        if (len < sizeof(struct ble_hci_cmd) || len > BLE_LL_CMD_SIZE ||
            ((FAR struct ble_hci_cmd *)data)->length !=
            len - sizeof(struct ble_hci_cmd))
          {
            return -EINVAL;
          }

        cmd = ble_ll_pool_alloc(&g_ble_ll_cmd_pool);
        if (cmd == NULL)
          {
            return -EBUSY;
          }

        memcpy(cmd, data, len);
        if (ble_ll_hci_cmd_rx(cmd) != 0)
          {
            ble_ll_hci_buf_free(cmd);
            return -EBUSY;
          }

        return len;

      case BT_ACL_OUT:
        if (len < BLE_HCI_DATA_HDR_SZ || len > BLE_LL_ACL_OUT_SIZE)
          {
            return -EINVAL;
          }

        iob = ble_ll_iob_alloc();
        if (iob == NULL)
          {
            return -ENOMEM;
          }

        ret = iob_trycopyin(iob, data, len, 0, false);
        if (ret != len)
          {
            iob_free_chain(iob);
            return -ENOMEM;
          }

        ble_ll_hci_acl_rx(iob);
        return len;

      default:
        return -ENOTSUP;
    }
}

/****************************************************************************
 * Name: ble_ll_lower_rx_start
 ****************************************************************************/

static int ble_ll_lower_rx_start(FAR uint8_t *rxbuf, uint8_t chan,
                                 FAR struct ble_mbuf_hdr *hdr)
{
  g_ble_ll_phy_rxhdr = hdr;
  return ble_ll_rx_start(rxbuf, chan, hdr);
}

/****************************************************************************
 * Name: ble_ll_lower_rx_end
 ****************************************************************************/

static int ble_ll_lower_rx_end(FAR uint8_t *rxbuf,
                               FAR struct ble_mbuf_hdr *hdr)
{
  g_ble_ll_phy_rxhdr = hdr;
  return ble_ll_rx_end(rxbuf, hdr);
}

/****************************************************************************
 * Name: ble_ll_lower_wfr_exp
 ****************************************************************************/

static void ble_ll_lower_wfr_exp(void)
{
  ble_ll_wfr_timer_exp(NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_hci_evt_alloc
 *
 * Description:
 *   Allocate an HCI event buffer.  Discardable events (like advertising
 *   reports) use only the low priority pool.
 *
 ****************************************************************************/

FAR void *ble_ll_hci_evt_alloc(int discardable)
{
  FAR void *buf;

  if (discardable)
    {
      return ble_ll_pool_alloc(&g_ble_ll_evt_lo_pool);
    }

  buf = ble_ll_pool_alloc(&g_ble_ll_evt_pool);
  if (buf == NULL)
    {
      buf = ble_ll_pool_alloc(&g_ble_ll_evt_lo_pool);
    }

  return buf;
}

/****************************************************************************
 * Name: ble_ll_hci_buf_free
 *
 * Description:
 *   Free an HCI command or event buffer.
 *
 ****************************************************************************/

void ble_ll_hci_buf_free(FAR void *buf)
{
  if (!ble_ll_pool_free(&g_ble_ll_cmd_pool, buf) &&
      !ble_ll_pool_free(&g_ble_ll_evt_pool, buf) &&
      !ble_ll_pool_free(&g_ble_ll_evt_lo_pool, buf))
    {
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: ble_ll_hci_evt_send
 *
 * Description:
 *   Send an HCI event to the host and free the event buffer.
 *
 ****************************************************************************/

int ble_ll_hci_evt_send(FAR void *buf)
{
  FAR struct ble_hci_ev *ev = buf;
  int ret = -ENODEV;

  if (g_ble_ll_btdev.receive != NULL)
    {
      ret = bt_netdev_receive(&g_ble_ll_btdev, BT_EVT, ev,
                              sizeof(*ev) + ev->length);
    }

  ble_ll_hci_buf_free(buf);
  return ret < 0 ? -1 : 0;
}

/****************************************************************************
 * Name: ble_ll_hci_acl_send
 *
 * Description:
 *   Send ACL data with the HCI ACL header to the host and free the PDU
 *   buffer.
 *
 ****************************************************************************/

int ble_ll_hci_acl_send(FAR struct iob_s *iob)
{
  unsigned int len = iob->io_pktlen;
  int ret = -ENODEV;

  DEBUGASSERT(len <= sizeof(g_ble_ll_acl_in));

  if (g_ble_ll_btdev.receive != NULL)
    {
      iob_copyout(g_ble_ll_acl_in, iob, len, 0);
      ret = bt_netdev_receive(&g_ble_ll_btdev, BT_ACL_IN, g_ble_ll_acl_in,
                              len);
    }

  iob_free_chain(iob);
  return ret < 0 ? -1 : 0;
}

/****************************************************************************
 * Name: ble_ll_register
 *
 * Description:
 *   Initialize the BLE Link Layer on top of a radio lower half and register
 *   it as a Bluetooth HCI driver.
 *
 * Input Parameters:
 *   lower - Radio lower half
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ble_ll_register(FAR struct ble_ll_lower_s *lower)
{
  int ret;

  DEBUGASSERT(lower != NULL && lower->ops != NULL);
  DEBUGASSERT(g_ble_ll_lower == NULL);

  g_ble_ll_wq = work_queue_create("ble_ll", CONFIG_BLE_LL_PRIORITY, NULL,
                                  CONFIG_BLE_LL_STACKSIZE, 1);
  if (g_ble_ll_wq == NULL)
    {
      wlerr("ERROR: failed to create work queue\n");
      return -ENOMEM;
    }

  ble_ll_pool_init(&g_ble_ll_cmd_pool);
  ble_ll_pool_init(&g_ble_ll_evt_pool);
  ble_ll_pool_init(&g_ble_ll_evt_lo_pool);

  g_ble_ll_lower = lower;
  lower->ops->bind(lower, &g_ble_ll_lowercb);

  ble_ll_init();

  ret = bt_driver_register(&g_ble_ll_btdev);
  if (ret < 0)
    {
      wlerr("ERROR: bt_driver_register failed: %d\n", ret);
    }

  return ret;
}
