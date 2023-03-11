/****************************************************************************
 * drivers/wireless/bluetooth/bt_rpmsghci_server.c
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

#include <nuttx/rptun/openamp.h>
#include <nuttx/wireless/bluetooth/bt_rpmsghci.h>

#include "bt_rpmsghci.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsghci_server_s
{
  struct rpmsg_endpoint   ept;
  FAR struct bt_driver_s *btdev;
  FAR const char         *name;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsghci_open_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsghci_close_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsghci_send_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsghci_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsghci_default_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsghci_handler[] =
{
  rpmsghci_open_handler,       /* RPMSGHCI_OPEN */
  rpmsghci_close_handler,      /* RPMSGHCI_CLOSE */
  rpmsghci_send_handler,       /* RPMSGHCI_SEND */
  rpmsghci_ioctl_handler,      /* RPMSGHCI_IOCTL */
  rpmsghci_default_handler,    /* RPMSGHCI_RECV */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsghci_get_tx_payload_buffer
 ****************************************************************************/

static FAR void *
rpmsghci_get_tx_payload_buffer(FAR struct rpmsghci_server_s *priv,
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

static int rpmsghci_send(FAR struct rpmsghci_server_s *priv,
                         uint32_t command, FAR struct rpmsghci_header_s *msg,
                         int len)
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
 * Name: rpmsghci_open_handler
 *
 * Description:
 *   Rpmsg-HCI device open handler.
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

static int rpmsghci_open_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_server_s *server = NULL;
  FAR struct bt_driver_s       *btdev  = NULL;
  FAR struct rpmsghci_data_s   *msg    = data;
  int                           ret    = OK;

  DEBUGASSERT(priv);
  server = priv;
  DEBUGASSERT(server->btdev);
  btdev = server->btdev;

  if (btdev->open)
    {
      ret = btdev->open(btdev);
    }

  msg->header.result = ret;

  return rpmsg_send(ept, msg, sizeof(struct rpmsghci_header_s));
}

/****************************************************************************
 * Name: rpmsghci_close_handler
 *
 * Description:
 *   Rpmsg-HCI close handler.
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

static int rpmsghci_close_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_server_s *server = NULL;
  FAR struct bt_driver_s       *btdev  = NULL;
  FAR struct rpmsghci_data_s   *msg    = data;
  int                           ret    = OK;

  DEBUGASSERT(priv);
  server = priv;
  DEBUGASSERT(server->btdev);
  btdev = server->btdev;

  if (btdev->close)
    {
      btdev->close(btdev);
    }

  msg->header.result = ret;

  return rpmsg_send(ept, msg, sizeof(struct rpmsghci_header_s));
}

/****************************************************************************
 * Name: rpmsghci_send_handler
 *
 * Description:
 *   Rpmsg-HCI send handler.
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

static int rpmsghci_send_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_server_s *server = NULL;
  FAR struct bt_driver_s       *btdev  = NULL;
  FAR struct rpmsghci_data_s   *msg    = data;
  int                           ret    = OK;

  DEBUGASSERT(priv);
  server = priv;
  DEBUGASSERT(server->btdev);
  btdev = server->btdev;

  wlinfo("rpmsghci_send_handler %d\n", msg->type);

  ret = btdev->send(btdev, msg->type, msg->data,
                    len - sizeof(struct rpmsghci_data_s) + 1);

  msg->header.result = ret;

  return rpmsg_send(ept, msg, sizeof(struct rpmsghci_header_s));
}

/****************************************************************************
 * Name: rpmsghci_ioctl_handler
 *
 * Description:
 *   Rpmsg-HCI ioctl handler.
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

static int rpmsghci_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsghci_server_s *server = NULL;
  FAR struct bt_driver_s       *btdev  = NULL;
  FAR struct rpmsghci_ioctl_s  *msg    = data;
  int                           ret    = OK;

  DEBUGASSERT(priv);
  server = priv;
  DEBUGASSERT(server->btdev);
  btdev = server->btdev;

  ret = btdev->ioctl(btdev, msg->request,
                     msg->arglen > 0 ? (unsigned long)msg->buf :
                     msg->arg);

  msg->header.result = ret;

  /* Send the entire frame so as not to lose ioctl returned data */

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsghci_default_handler
 *
 * Description:
 *   Default rpmsg-HCI response handler, this function will be called to
 *   process the return message of rpmsghci_recv()
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
 * Name: rpmsghci_ns_match
 *
 * Description:
 *   Check if we match to the rpmsg name service.
 *
 ****************************************************************************/

static bool rpmsghci_ns_match(FAR struct rpmsg_device *rdev, FAR void *priv,
                              FAR const char *name, uint32_t dest)
{
  return !strncmp(name, RPMSGHCI_NAME_PREFIX, RPMSGHCI_NAME_PREFIX_LEN);
}

/****************************************************************************
 * Name: rpmsghci_ept_cb
 *
 * Description:
 *   Rpmsg HCI end point callback function, this function will be called
 *   when receive the client cpu message.
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

static int rpmsghci_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                           size_t len, uint32_t src, FAR void *priv)
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
 * Name: rpmsghci_ns_unbind
 *
 * Description:
 *   Unbind from the rpmsg name service.
 *
 ****************************************************************************/

static void rpmsghci_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  rpmsg_destroy_ept(ept);
  kmm_free(ept);
}

/****************************************************************************
 * Name: rpmsghci_ns_bind
 *
 * Description:
 *   Bind to the rpmsg name service.
 *
 ****************************************************************************/

static void rpmsghci_ns_bind(FAR struct rpmsg_device *rdev, FAR void *priv,
                             FAR const char *name, uint32_t dest)
{
  FAR struct rpmsghci_server_s *server = priv;

  server->ept.priv = priv;
  rpmsg_create_ept(&server->ept, rdev, name,
                   RPMSG_ADDR_ANY, dest,
                   rpmsghci_ept_cb, rpmsghci_ns_unbind);
}

/****************************************************************************
 * Name: rpmsghci_bt_receive
 *
 * Description:
 *   Called by the Bluetooth low-level driver when new data is received from
 *   the radio.  Transfers data to the RPMSG client.
 *
 *   This callback should not be called from the interrupt context !
 *
 * Returned Value:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsghci_bt_receive(FAR struct bt_driver_s *btdev, uint8_t type,
                        FAR void *data, size_t len)
{
  FAR struct rpmsghci_server_s *priv  = NULL;
  FAR struct rpmsghci_data_s   *msg   = NULL;
  uint32_t                      space = 0;
  int                           ret   = OK;

  wlinfo("rpmsghci_bt_receive %d\n", type);

  /* Get RPMSG-HCI data */

  DEBUGASSERT(btdev != NULL);
  priv = btdev->priv;

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
  return rpmsghci_send(priv, RPMSGHCI_RECV, &msg->header,
                       sizeof(struct rpmsghci_data_s) -1 + len);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsghci_server_init
 *
 * Description:
 *   Rpmsg-HCI server initialize function, the server cpu should call
 *   this function.
 *
 * Parameters:
 *   name - RPMSG-HCI server name
 *   bt   - BT device handler
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmshci_server_init(FAR const char *name, FAR struct bt_driver_s *btdev)
{
  struct rpmsghci_server_s *priv = NULL;
  int                       ret  = OK;

  /* Validate input */

  if (!name || !btdev)
    {
      return -EINVAL;
    }

  /* Create RPMSG-HCI server */

  priv = kmm_zalloc(sizeof(struct rpmsghci_server_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Connect BT receive callback and RPMSG as priv */

  btdev->receive = rpmsghci_bt_receive;
  btdev->priv    = priv;

  /* Initialize RPMSG-HCI server data */

  priv->name  = name;
  priv->btdev = btdev;

  ret = rpmsg_register_callback(priv, NULL, NULL,
                                rpmsghci_ns_match,
                                rpmsghci_ns_bind);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}
