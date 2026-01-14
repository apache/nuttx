/****************************************************************************
 * drivers/rpmsg/rpmsg_router_edge.c
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

#include <debug.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <nuttx/kmalloc.h>
#include <rpmsg/rpmsg_internal.h>

#include "rpmsg_router.h"

/****************************************************************************
 * Rpmsg-router Model:
 *
 *  +------+       +------+       +------+
 *  | edge |<----->| hub  |<----->| edge |
 *  +------+       +------+       +------+
 *
 * Description:
 *    edge CPUs (edge) are physically linked to the central router cpu (hub),
 *    edge CPUs' communication reply on hub cpu message forwarding.
 *
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rpmsg_router_edge_from_rdev(d) \
  metal_container_of(d, struct rpmsg_router_edge_s, rdev)

#define RPMSG_ROUTER_USER_NAME_SIZE \
  (RPMSG_NAME_SIZE - RPMSG_ROUTER_NAME_PREFIX_LEN - RPMSG_ROUTER_CPUNAME_LEN)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_router_edge_s
{
  struct rpmsg_s      rpmsg;
  struct rpmsg_device rdev;
  struct rpmsg_device *hubdev;
  char                name[RPMSG_NAME_SIZE];
  char                localcpu[RPMSG_ROUTER_CPUNAME_LEN];
  char                remotecpu[RPMSG_ROUTER_CPUNAME_LEN];

  /* Tx/Rx buffer size */

  uint32_t            tx_len;
  uint32_t            rx_len;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR const char *
rpmsg_router_edge_get_local_cpuname(FAR struct rpmsg_s *rpmsg);
static FAR const char *
rpmsg_router_edge_get_cpuname(FAR struct rpmsg_s *rpmsg);

static int
rpmsg_router_edge_get_rx_buffer_size(FAR struct rpmsg_device *rdev);
static int
rpmsg_router_edge_get_tx_buffer_size(FAR struct rpmsg_device *rdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_ops_s g_rpmsg_router_edge_ops =
{
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  rpmsg_router_edge_get_local_cpuname,
  rpmsg_router_edge_get_cpuname,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_router_edge_get_local_cpuname
 ****************************************************************************/

static FAR const char *
rpmsg_router_edge_get_local_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_router_edge_s *edge =
      (FAR struct rpmsg_router_edge_s *)rpmsg;

  return edge->localcpu;
}

/****************************************************************************
 * Name: rpmsg_router_edge_get_cpuname
 ****************************************************************************/

static FAR const char *
rpmsg_router_edge_get_cpuname(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_router_edge_s *edge =
      (FAR struct rpmsg_router_edge_s *)rpmsg;

  return edge->remotecpu;
}

/****************************************************************************
 * Name: rpmsg_router_edge_get_tx_payload_buffer
 ****************************************************************************/

static FAR void *
rpmsg_router_edge_get_tx_payload_buffer(FAR struct rpmsg_device *rdev,
                                        FAR uint32_t *len, int wait)
{
  FAR struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);
  FAR struct rpmsg_device *hubdev = edge->hubdev;
  FAR void *buf;

  if (!hubdev->ops.get_tx_payload_buffer)
    {
      return NULL;
    }

  buf = hubdev->ops.get_tx_payload_buffer(hubdev, len, wait);
  *len = edge->tx_len;
  return buf;
}

/****************************************************************************
 * Name: rpmsg_router_edge_hold_rx_buffer
 ****************************************************************************/

static void rpmsg_router_edge_hold_rx_buffer(FAR struct rpmsg_device *rdev,
                                             FAR void *rxbuf)
{
  FAR struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);
  FAR struct rpmsg_device *hubdev = edge->hubdev;

  if (!hubdev->ops.hold_rx_buffer)
    {
      return;
    }

  hubdev->ops.hold_rx_buffer(hubdev, rxbuf);
}

/****************************************************************************
 * Name: rpmsg_router_edge_release_rx_buffer
 ****************************************************************************/

static void
rpmsg_router_edge_release_rx_buffer(FAR struct rpmsg_device *rdev,
                                    FAR void *rxbuf)
{
  struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);
  struct rpmsg_device *hubdev = edge->hubdev;

  if (!hubdev->ops.release_rx_buffer)
    {
      return;
    }

  hubdev->ops.release_rx_buffer(hubdev, rxbuf);
}

/****************************************************************************
 * Name: rpmsg_router_edge_release_tx_buffer
 ****************************************************************************/

static int rpmsg_router_edge_release_tx_buffer(FAR struct rpmsg_device *rdev,
                                               FAR void *txbuf)
{
  struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);
  struct rpmsg_device *hubdev = edge->hubdev;

  if (!hubdev->ops.release_tx_buffer)
    {
      return RPMSG_ERR_PERM;
    }

  return hubdev->ops.release_tx_buffer(hubdev, txbuf);
}

/****************************************************************************
 * Name: rpmsg_router_edge_send_nocopy
 ****************************************************************************/

static int rpmsg_router_edge_send_nocopy(FAR struct rpmsg_device *rdev,
                                         uint32_t src, uint32_t dst,
                                         FAR const void *data, int len)
{
  struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);
  struct rpmsg_device *hubdev = edge->hubdev;

  if (!hubdev->ops.send_offchannel_nocopy)
    {
      return RPMSG_ERR_PARAM;
    }

  return hubdev->ops.send_offchannel_nocopy(hubdev, src, dst, data, len);
}

/****************************************************************************
 * Name: rpmsg_router_edge_get_rx_buffer_size
 ****************************************************************************/

static int
rpmsg_router_edge_get_rx_buffer_size(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);

  return edge->rx_len;
}

/****************************************************************************
 * Name: rpmsg_router_edge_get_tx_buffer_size
 ****************************************************************************/

static int
rpmsg_router_edge_get_tx_buffer_size(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);

  return edge->tx_len;
}

/****************************************************************************
 * Name: rpmsg_router_edge_cb
 *
 * Description:
 *   This is the callback function for edge core.
 *   It will receive data from real rpmsg channel by ept(r:cpu:name),
 *   and find the corresponding user ept, then processing data through
 *   the user ept callback.
 *
 * Parameters:
 *   ept - rpmsg_endpoint for communicating with router core (r:cpu:name)
 *   data - received data
 *   len - received data length
 *   src - source address
 *   priv - save user rpmsg_endpoint generally
 *
 * Returned Values:
 *   0 on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsg_router_edge_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_endpoint *usr_ept = priv;

  if (!usr_ept)
    {
      return 0;
    }

  /* Processing data through the user ept callback */

  return usr_ept->cb(usr_ept, data, len, src, usr_ept->priv);
}

/****************************************************************************
 * Name: rpmsg_router_edge_bound
 *
 * Description:
 *   This is the callback function for edge core.
 *   It will be called when the edge core is bound to the router core
 *   by r:cpu:name, save the destination address of the edge core, and
 *   call the bound function of the user endpoint.
 *
 * Parameters:
 *   ept - rpmsg_endpoint for communicating with router core (r:cpu:name)
 *
 ****************************************************************************/

static void rpmsg_router_edge_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_endpoint *usr_ept = ept->priv;

  if (!usr_ept)
    {
      rpmsgerr("Try to get user ept failed.\n");
      return;
    }

  usr_ept->dest_addr = ept->dest_addr;
  if (usr_ept->ns_bound_cb)
    {
      usr_ept->ns_bound_cb(usr_ept);
    }
}

/****************************************************************************
 * Name: rpmsg_router_edge_unbind
 *
 * Description:
 *   This is the unbind callback function for edge core.
 *
 * Parameters:
 *   ept - rpmsg_endpoint for communicating with router core (r:cpu:name)
 *
 ****************************************************************************/

static void rpmsg_router_edge_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_endpoint *usr_ept = ept->priv;

  if (!usr_ept)
    {
      rpmsgerr("Try to get user ept failed.\n");
      return;
    }

  if (usr_ept->ns_unbind_cb)
    {
      usr_ept->ns_unbind_cb(usr_ept);
    }
}

/****************************************************************************
 * Name: rpmsg_router_edge_release
 ****************************************************************************/

static void rpmsg_router_edge_release(FAR struct rpmsg_endpoint *ept)
{
  kmm_free(ept);
}

/****************************************************************************
 * Name: rpmsg_router_edge_send_offchannel_raw
 *
 * Description:
 *   This function sends normal rpmsg message or ns message to remote device.
 *   If the destination address is RPMSG_NS_EPT_ADDR, it will create a new
 *   endpoint(r:cpu:name) for real communication, and save the user endpoint
 *   information in the private field of the new endpoint.
 *
 * Parameters:
 *   rdev - rpmsg_device for router core
 *   src - source address
 *   dst - destination address
 *   data - data to send
 *   len - data length
 *   wait - boolean, wait or not for buffer to become available
 *
 * Returned Values:
 *   size of data sent or negative value for failure.
 *
 ****************************************************************************/

static int
rpmsg_router_edge_send_offchannel_raw(FAR struct rpmsg_device *rdev,
                                      uint32_t src, uint32_t dst,
                                      FAR const void *data,
                                      int len, int wait)
{
  FAR struct rpmsg_router_edge_s *edge = rpmsg_router_edge_from_rdev(rdev);
  FAR struct rpmsg_ns_msg *ns_msg = (FAR struct rpmsg_ns_msg *)data;
  FAR struct rpmsg_device *hubdev = edge->hubdev;
  FAR struct rpmsg_endpoint *usr_ept;
  FAR struct rpmsg_endpoint *ept;
  char name[RPMSG_ROUTER_USER_NAME_SIZE];
  int ret;

  /* Send normal rpmsg "message" to remote device */

  if (dst != RPMSG_NS_EPT_ADDR)
    {
      if (!hubdev->ops.send_offchannel_raw)
        {
          return RPMSG_ERR_PARAM;
        }

      return hubdev->ops.send_offchannel_raw(hubdev, src,
                                             dst, data, len, wait);
    }

  /* Try to get user ept firstly */

  metal_mutex_acquire(&rdev->lock);
  usr_ept = rpmsg_get_endpoint(rdev, ns_msg->name, src, dst);
  metal_mutex_release(&rdev->lock);
  if (!usr_ept)
    {
      rpmsgerr("Try to get user ept failed.\n");
      return RPMSG_ERR_PARAM;
    }

  /* Set hub endpoint name(r:cpu:name) for real communication */

  strlcpy(name, ns_msg->name, sizeof(name));
  snprintf(ns_msg->name, sizeof(ns_msg->name),
           RPMSG_ROUTER_NAME_PREFIX"%s:%s", edge->remotecpu, name);

  if (ns_msg->flags == RPMSG_NS_DESTROY)
    {
      /* Processing RPMSG_NS_DESTROY message */

      metal_mutex_acquire(&hubdev->lock);
      ept = rpmsg_get_endpoint(hubdev, ns_msg->name,
                               RPMSG_ADDR_ANY, usr_ept->dest_addr);
      metal_mutex_release(&hubdev->lock);
      if (!ept)
        {
          rpmsgerr("Try to get router endpoint (r:ept) failed.\n");
          return RPMSG_ERR_PARAM;
        }

      /* Destroy endpoint(r:cpu:name) of real communication */

      rpmsg_destroy_ept(ept);
      return 0;
    }
  else
    {
      /* Processing RPMSG_NS_CREATE or RPMSG_NS_CREATE_ACK message */

      ept = kmm_zalloc(sizeof(*ept));
      if (!ept)
        {
          return -ENOMEM;
        }

      /* Save user endpoint */

      ept->priv = usr_ept;
      ept->ns_bound_cb = rpmsg_router_edge_bound;
      ept->release_cb = rpmsg_router_edge_release;

      /* Create endpoint (r:cpu:name) for real communication */

      ret = rpmsg_create_ept(ept, hubdev, ns_msg->name,
                             RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                             rpmsg_router_edge_cb,
                             rpmsg_router_edge_unbind);
      if (ret < 0)
        {
          rpmsgerr("Create router endpoint failed: %d\n", ret);
          kmm_free(ept);
        }

      return ret;
    }
}

/****************************************************************************
 * Name: rpmsg_router_edge_match
 *
 * Description:
 *   This function is used to match the edge core device.
 *   rpmsg_router_edge_bind will be called if the device is matched.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - rpmsg router device for edge core
 *   name - endpoint name (r:cpu:name)
 *   dest - destination address
 *
 * Returned Values:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static bool rpmsg_router_edge_match(FAR struct rpmsg_device *rdev,
                                    FAR void *priv, FAR const char *name,
                                    uint32_t dest)
{
  FAR struct rpmsg_router_edge_s *edge = priv;

  if (strncmp(name, RPMSG_ROUTER_NAME_PREFIX, RPMSG_ROUTER_NAME_PREFIX_LEN))
    {
      return false;
    }

  return !strncmp(name + RPMSG_ROUTER_NAME_PREFIX_LEN, edge->remotecpu,
                  strlen(edge->remotecpu));
}

/****************************************************************************
 * Name: rpmsg_router_edge_bind
 *
 * Description:
 *   This function is used to bind the edge core device.
 *   It will try to find rpmsg_user_ns_bind_cb by user ept name.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - rpmsg router device for edge core
 *   name - endpoint name (r:cpu:name)
 *   dest - destination address
 *
 ****************************************************************************/

static void rpmsg_router_edge_bind(FAR struct rpmsg_device *rdev,
                                   FAR void *priv, FAR const char *name,
                                   uint32_t dest)
{
  FAR struct rpmsg_router_edge_s *edge = priv;
  FAR struct rpmsg_device *edgedev = &edge->rdev;

  edgedev->ns_bind_cb(edgedev,
                      name + RPMSG_ROUTER_NAME_PREFIX_LEN +
                      strlen(edge->remotecpu) + 1, dest);
}

/****************************************************************************
 * Name: rpmsg_router_edge_destroy
 *
 * Description:
 *   This function is used to destroy the edge core device.
 *
 * Parameters:
 *   edge - rpmsg router edge device
 *
 ****************************************************************************/

static void rpmsg_router_edge_destroy(FAR struct rpmsg_router_edge_s *edge)
{
  rpmsg_unregister_callback(edge, NULL, NULL,
                            rpmsg_router_edge_match,
                            rpmsg_router_edge_bind);
  rpmsg_unregister(edge->name, &edge->rpmsg);
  rpmsg_device_destory(&edge->rpmsg);
  kmm_free(edge);
}

/****************************************************************************
 * Name: rpmsg_router_edge_create
 *
 * Description:
 *   This function is used to create the edge core device.
 *
 * Parameters:
 *   hubdev - rpmsg device for router hub
 *   msg - sync message from router hub
 *   remotecpu - remote edge cpu name
 *
 * Returned Values:
 *   edge device on success; NULL on failure.
 *
 ****************************************************************************/

static FAR struct rpmsg_router_edge_s *
rpmsg_router_edge_create(FAR struct rpmsg_device *hubdev,
                         FAR struct rpmsg_router_s *msg,
                         FAR const char *remotecpu)
{
  FAR struct rpmsg_router_edge_s *edge;
  FAR struct rpmsg_device *rdev;
  int ret;

  /* Create the router edge device */

  edge = kmm_zalloc(sizeof(*edge));
  if (!edge)
    {
      return NULL;
    }

  strlcpy(edge->remotecpu, remotecpu, sizeof(edge->remotecpu));
  strlcpy(edge->localcpu, msg->cpuname, sizeof(edge->localcpu));
  edge->rx_len = msg->rx_len;
  edge->tx_len = msg->tx_len;
  edge->hubdev = hubdev;

  /* Initialize router rpmsg device */

  rdev = &edge->rdev;
  metal_mutex_init(&rdev->lock);
  rdev->ns_bind_cb = rpmsg_ns_bind;
  rdev->ns_unbind_cb = rpmsg_ns_unbind;
  rdev->ops.hold_rx_buffer = rpmsg_router_edge_hold_rx_buffer;
  rdev->ops.release_rx_buffer = rpmsg_router_edge_release_rx_buffer;
  rdev->ops.release_tx_buffer = rpmsg_router_edge_release_tx_buffer;
  rdev->ops.send_offchannel_nocopy = rpmsg_router_edge_send_nocopy;
  rdev->ops.send_offchannel_raw = rpmsg_router_edge_send_offchannel_raw;
  rdev->ops.get_tx_payload_buffer = rpmsg_router_edge_get_tx_payload_buffer;
  rdev->ops.get_rx_buffer_size = rpmsg_router_edge_get_rx_buffer_size;
  rdev->ops.get_tx_buffer_size = rpmsg_router_edge_get_tx_buffer_size;

  metal_list_init(&rdev->endpoints);
  rdev->support_ack = true;
  rdev->support_ns = true;

  /* Register rpmsg for edge core */

  snprintf(edge->name, sizeof(edge->name), "/dev/rpmsg/%s", edge->remotecpu);
  ret = rpmsg_register(edge->name, &edge->rpmsg, &g_rpmsg_router_edge_ops);
  if (ret < 0)
    {
      rpmsgerr("rpmsg_register failed: %d\n", ret);
      goto free;
    }

  /* Register callback for edge core */

  ret = rpmsg_register_callback(edge, NULL, NULL,
                                rpmsg_router_edge_match,
                                rpmsg_router_edge_bind);

  if (ret < 0)
    {
      rpmsgerr("Register rpmsg callback failed: %d\n", ret);
      goto unregister;
    }

  /* Broadcast device_created to all registers */

  rpmsg_device_created(&edge->rpmsg);
  return edge;

unregister:
  rpmsg_unregister(edge->name, &edge->rpmsg);
free:
  kmm_free(edge);
  return NULL;
}

/****************************************************************************
 * Name: rpmsg_router_cb
 *
 * Description:
 *   This function is used to receive sync message from router core,
 *   and create or destroy the edge core device.
 *
 * Parameters:
 *   ept - endpoint for synchronizing ready messages
 *   data - received data
 *   len - received data length
 *   src - source address
 *   priv - private data
 *
 * Returned Values:
 *   0 on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsg_router_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len,
                           uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_router_s *msg = data;
  FAR struct rpmsg_router_edge_s *edge;

  /* Destroy the router edge device */

  if (msg->cmd == RPMSG_ROUTER_DESTROY)
    {
      edge = ept->priv;

      if (edge)
        {
          rpmsg_router_edge_destroy(edge);
          ept->priv = NULL;
          return 0;
        }

      return -EINVAL;
    }

  /* Create the router edge device */

  edge = rpmsg_router_edge_create(ept->rdev, msg,
                                  ept->name + RPMSG_ROUTER_NAME_LEN);
  if (!edge)
    {
      return -ENODEV;
    }

  ept->priv = edge;
  return 0;
}

/****************************************************************************
 * Name: rpmsg_router_unbind
 *
 * Description:
 *   This function is used to destroy the sync endpoint
 *   when another edge core is disconnected.
 *
 * Parameters:
 *   ept - rpmsg endpoint for synchronizing message.
 *
 ****************************************************************************/

static void rpmsg_router_unbind(FAR struct rpmsg_endpoint *ept)
{
  struct rpmsg_router_edge_s *edge = ept->priv;

  if (edge)
    {
      rpmsg_router_edge_destroy(edge);
      ept->priv = NULL;
    }

  rpmsg_destroy_ept(ept);
  kmm_free(ept);
}

/****************************************************************************
 * Name: rpmsg_router_match
 *
 * Description:
 *   This function is used to match the endpoint for
 *   synchronizing ready messages.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - rpmsg_router_priv
 *   name - endpoint name
 *   dest - destination address
 *
 * Returned Values:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static bool rpmsg_router_match(FAR struct rpmsg_device *rdev, FAR void *priv,
                               FAR const char *name, uint32_t dest)
{
  return !strncmp(name, RPMSG_ROUTER_NAME, RPMSG_ROUTER_NAME_LEN);
}

/****************************************************************************
 * Name: rpmsg_router_bind
 *
 * Description:
 *   This function is used to bind the endpoint for
 *   synchronizing ready messages.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - private data
 *   name - endpoint name
 *   dest - destination address
 *
 ****************************************************************************/

static void rpmsg_router_bind(FAR struct rpmsg_device *rdev, FAR void *priv,
                              FAR const char *name, uint32_t dest)
{
  FAR struct rpmsg_endpoint *ept;
  int ret;

  ept = kmm_zalloc(sizeof(*ept));
  DEBUGASSERT(ept);

  ret = rpmsg_create_ept(ept, rdev, name, RPMSG_ADDR_ANY, dest,
                         rpmsg_router_cb, rpmsg_router_unbind);
  if (ret < 0)
    {
      rpmsgerr("Create router endpoint failed: %d\n", ret);
      kmm_free(ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_router_edge_init
 *
 * Description:
 *   This function is used to initialize the edge core.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_router_edge_init(void)
{
  /* Register callback for listening sync message from router hub */

  return rpmsg_register_callback(NULL, NULL, NULL,
                                 rpmsg_router_match,
                                 rpmsg_router_bind);
}
