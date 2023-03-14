/****************************************************************************
 * drivers/wireless/bluetooth/bt_rpmsghci.c
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

#include <sys/param.h>

#include <assert.h>
#include <debug.h>
#include <stdlib.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/semaphore.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/wireless/bluetooth/bt_rpmsghci.h>

#include "bt_rpmsghci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsghci_s
{
  /* This must be te first thung in the structure so we can simply cast from
   * struct rpmsghci_s to struct bt_driver_s.
   */

  struct bt_driver_s    btdev;
  struct rpmsg_endpoint ept;
  FAR const char       *cpuname;
  FAR const char       *name;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsghci_default_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);
static int rpmsghci_recv_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsghci_handler[] =
{
  rpmsghci_default_handler, /* RPMSGHCI_OPEN */
  rpmsghci_default_handler, /* RPMSGHCI_CLOSE */
  rpmsghci_default_handler, /* RPMSGHCI_SEND */
  rpmsghci_default_handler, /* RPMSGHCI_IOCTL */
  rpmsghci_recv_handler,    /* RPMSGHCI_RECV */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsghci_get_tx_payload_buffer
 ****************************************************************************/

static FAR void *
rpmsghci_get_tx_payload_buffer(FAR struct rpmsghci_s *priv,
                               FAR uint32_t *len)
{
  return rpmsg_get_tx_payload_buffer(&priv->ept, len, true);
}

/****************************************************************************
 * Name: rpmsghci_send
 *
 * Description:
 *   Send the rpmsg data and wait for ACK.
 *
 * Parameters:
 *   priv    - rpmsg device handle
 *   command - the command, RPMSGHCI_OPEN, RPMSGHCI_CLOSE, RPMSGHCI_SEND,
 *                          RPMSGHCI_IOCTL
 *   msg     - the message header
 *   len     - length of the payload
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsghci_send(FAR struct rpmsghci_s *priv, uint32_t command,
                         FAR struct rpmsghci_header_s *msg, int len)
{
  struct rpmsghci_cookie_s cookie;
  int                      ret = OK;

  memset(&cookie, 0, sizeof(struct rpmsghci_cookie_s));
  nxsem_init(&cookie.sem, 0, 0);

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)&cookie;

  ret = rpmsg_send_nocopy(&priv->ept, msg, len);
  if (ret < 0)
    {
      goto errout;
    }

  ret = rpmsg_wait(&priv->ept, &cookie.sem);
  if (ret >= 0)
    {
      ret = msg->result;
    }

errout:
  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: rpmsghci_bt_open
 *
 * Description:
 *   Rpmsg-HCI open operation
 *
 * Parameters:
 *   btdev  - the bt instance
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsghci_bt_open(struct bt_driver_s *btdev)
{
  FAR struct rpmsghci_s      *priv = NULL;
  FAR struct rpmsghci_open_s *msg  = NULL;
  uint32_t                    space = 0;

  /* Get RPMSG-HCI data */

  DEBUGASSERT(btdev != NULL);
  priv = (FAR struct rpmsghci_s *)btdev;

  /* Perform the rpmsg write */

  msg = rpmsghci_get_tx_payload_buffer(priv, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  return rpmsghci_send(priv, RPMSGHCI_OPEN, &msg->header,
                       sizeof(struct rpmsghci_open_s));
}

/****************************************************************************
 * Name: rpmsghci_bt_close
 *
 * Description:
 *   Rpmsg-HCI close operation
 *
 * Parameters:
 *   btdev  - the bt instance
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static void rpmsghci_bt_close(struct bt_driver_s *btdev)
{
  FAR struct rpmsghci_s       *priv  = NULL;
  FAR struct rpmsghci_close_s *msg   = NULL;
  uint32_t                     space = 0;

  /* Get RPMSG-HCI data */

  DEBUGASSERT(btdev != NULL);
  priv = (FAR struct rpmsghci_s *)btdev;

  /* Perform the rpmsg write */

  msg = rpmsghci_get_tx_payload_buffer(priv, &space);
  if (msg == NULL)
    {
      return;
    }

  rpmsghci_send(priv, RPMSGHCI_CLOSE, &msg->header,
                sizeof(struct rpmsghci_close_s));
}

/****************************************************************************
 * Name: rpmsghci_bt_send
 *
 * Description:
 *   Rpmsg-HCI send operation
 *
 * Parameters:
 *   btdev  - the bt instance
 *   type   - bt command type
 *   data   - bt command data
 *   len    - bt command data length
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsghci_bt_send(struct bt_driver_s *btdev, uint8_t type,
                            void *data, size_t len)
{
  FAR struct rpmsghci_s      *priv  = NULL;
  FAR struct rpmsghci_data_s *msg   = NULL;
  uint32_t                    space = 0;

  wlinfo("rpmsghci_bt_send %d\n", type);

  /* Get RPMSG-HCI data */

  DEBUGASSERT(btdev != NULL);
  priv = (FAR struct rpmsghci_s *)btdev;

  if (data == NULL)
    {
      return -EINVAL;
    }

  /* Perform the rpmsg write */

  msg = rpmsghci_get_tx_payload_buffer(priv, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(sizeof(struct rpmsghci_data_s) - 1 + len <= space);
  memcpy(msg->data, data, len);
  msg->type = type;
  return rpmsghci_send(priv, RPMSGHCI_SEND, &msg->header,
                       sizeof(struct rpmsghci_data_s) -1 + len);
}

/****************************************************************************
 * Name: rpmsghci_bt_ioctl
 *
 * Description:
 *   Rpmsg-HCI ioctl operation
 *
 * Parameters:
 *   btdev  - the bt instance
 *   cmd    - the ioctl command
 *   arg    - the ioctl arguments
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsghci_bt_ioctl(FAR struct bt_driver_s *btdev, int cmd,
                              unsigned long arg)
{
  /* TODO */

  return -ENOTTY;
}

/****************************************************************************
 * Name: rpmsghci_default_handler
 *
 * Description:
 *   Default rpmsg-HCI response handler, this function will be called to
 *   process the return message of rpmsghci_open(), rpmsghci_close(),
 *   rpmsghci_send(), rpmsghci_ioctl().
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *   src  - unknow
 *   priv - unknow
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsghci_default_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_header_s *header = data;
  FAR struct rpmsghci_cookie_s *cookie = NULL;

  if (header->cookie != 0)
    {
      cookie = (FAR struct rpmsghci_cookie_s *)(uintptr_t)header->cookie;
      return rpmsg_post(ept, &cookie->sem);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsghci_recv_handler
 *
 * Description:
 *   Rpmsg-HCI receive handler.
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *   src  - unknow
 *   priv - unknow
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsghci_recv_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_s      *client = NULL;
  FAR struct rpmsghci_data_s *msg    = data;
  int                         ret    = OK;

  DEBUGASSERT(priv);
  client = priv;

  wlinfo("rpmsghci_recv_handler %d\n", msg->type);

  ret = bt_netdev_receive(&client->btdev, msg->type, msg->data,
                          len - sizeof(struct rpmsghci_data_s) + 1);
  msg->header.result = ret;

  return rpmsg_send(ept, msg, sizeof(struct rpmsghci_header_s));
}

/****************************************************************************
 * Name: rpmsghci_client_ept_cb
 *
 * Description:
 *   Rpmsg HCI end point callback function, this function will be called
 *   when receive the remote cpu message.
 *
 * Parameters:
 *   ept  - The rpmsg-HCI end point
 *   data - The received data
 *   len  - The received data length
 *   src  - unknow
 *   priv - unknow
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsghci_client_ept_cb(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_header_s *header  = data;
  uint32_t                      command = header->command;

  if (command < nitems(g_rpmsghci_handler))
    {
      return g_rpmsghci_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: rpmsghci_client_created
 *
 * Description:
 *   Rpmsg HCI create function, this function will be called by rptun to
 *   create a rpmsg-hci end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-hci end point
 *   priv_ - Rpmsg-hci handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsghci_client_created(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsghci_s *priv = priv_;
  char                   eptname[RPMSG_NAME_SIZE];

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      snprintf(eptname, RPMSG_NAME_SIZE, "%s%s", RPMSGHCI_NAME_PREFIX,
               priv->name);
      priv->ept.priv = priv;
      rpmsg_create_ept(&priv->ept, rdev, eptname,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsghci_client_ept_cb, NULL);
    }
}

/****************************************************************************
 * Name: rpmsghci_client_destroy
 *
 * Description:
 *   Rpmsg HCI destroy function, this function will be called by rptun to
 *   destroy rpmsg-HCI end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-HCI end point
 *   priv_ - Rpmsg-HCI handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsghci_client_destroy(FAR struct rpmsg_device *rdev,
                                    FAR void *priv_)
{
  FAR struct rpmsghci_s *priv = priv_;

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsghci_register
 *
 * Description:
 *   Rpmsg-HCI client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Parameters:
 *   cpuname  - the server cpu name
 *   name     - the HCI device you want to access in the remote cpu
 *
 * Returned Values:
 *    A non-NULL handle is returned on success;
 *    A NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct bt_driver_s *rpmsghci_register(FAR const char *cpuname,
                                          FAR const char *name)
{
  FAR struct rpmsghci_s *priv = NULL;

  /* Validate input */

  if (!cpuname || !name)
    {
      return NULL;
    }

  /* Create RPMSG-HCI driver */

  priv = kmm_zalloc(sizeof(struct rpmsghci_s));
  if (!priv)
    {
      return NULL;
    }

  /* Initialize BT driver */

  priv->btdev.open  = rpmsghci_bt_open;
  priv->btdev.close = rpmsghci_bt_close;
  priv->btdev.send  = rpmsghci_bt_send;
  priv->btdev.ioctl = rpmsghci_bt_ioctl;

  /* Initialize RPMSG-HCI client data */

  priv->cpuname = cpuname;
  priv->name    = name;

  /* Register the device with the openamp */

  rpmsg_register_callback(priv,
                          rpmsghci_client_created,
                          rpmsghci_client_destroy,
                          NULL,
                          NULL);

  return &priv->btdev;
}
