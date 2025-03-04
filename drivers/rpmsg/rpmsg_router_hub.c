/****************************************************************************
 * drivers/rpmsg/rpmsg_router_hub.c
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
#include <sys/param.h>

#include <nuttx/mutex.h>
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
 * Private Types
 ****************************************************************************/

struct rpmsg_router_hub_s
{
  struct rpmsg_endpoint ept[2];
  char                  cpuname[2][RPMSG_ROUTER_CPUNAME_LEN];
  mutex_t               lock;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_router_hub_cb
 *
 * Description:
 *   This is the callback function for router core.
 *   It will receive data from source edge core by ept(r:cpu:name), and find
 *   dest edge core communicating with it, send data to dest edge core.
 *
 * Parameters:
 *   ept - rpmsg_endpoint for communicating with edge core (r:dst_cpu:name)
 *   data - received data
 *   len - received data length
 *   src - source address
 *   priv - save dest edge core rpmsg_endpoint (r:src_cpu:name)
 *
 * Returned Values:
 *   Returns number of bytes it has sent or negative error value on failure.
 *
 ****************************************************************************/

static int rpmsg_router_hub_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len,
                               uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_endpoint *dst_ept = priv;

  /* Retransmit data to dest edge core */

  if (!dst_ept)
    {
      return -EINVAL;
    }

  return rpmsg_send(dst_ept, data, len);
}

/****************************************************************************
 * Name: rpmsg_router_hub_unbind
 *
 * Description:
 *   This is the unbind callback function for router core.
 *
 * Parameters:
 *   ept - rpmsg_endpoint for communicating with edge core (r:cpu:name)
 *
 ****************************************************************************/

static void rpmsg_router_hub_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_endpoint *dst_ept = ept->priv;

  /* Destroy dest edge ept firstly */

  if (dst_ept)
    {
      rpmsg_destroy_ept(dst_ept);
      kmm_free(dst_ept);
    }

  /* Destroy source edge ept */

  rpmsg_destroy_ept(ept);
  kmm_free(ept);
}

/****************************************************************************
 * Name: rpmsg_router_hub_bound
 *
 * Description:
 *   This is the bound callback function for router core.
 *   It will create endpoint to source edge after dest edge
 *   core is bound.
 *
 * Parameters:
 *   ept - rpmsg_endpoint for communicating with edge core (r:cpu:name)
 *
 ****************************************************************************/

static void rpmsg_router_hub_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_endpoint *src_ept = ept->priv;
  int ret;

  /* Create endpoint (r:dst_cpu:name) and send ACK to source edge core */

  ret = rpmsg_create_ept(src_ept, src_ept->rdev, src_ept->name,
                         RPMSG_ADDR_ANY, src_ept->dest_addr,
                         rpmsg_router_hub_cb, rpmsg_router_hub_unbind);
  DEBUGASSERT(ret == RPMSG_SUCCESS);
}

/****************************************************************************
 * Name: rpmsg_router_hub_match
 *
 * Description:
 *   This function is used to match the router core device.
 *   rpmsg_router_hub_bind will be called if the device is matched.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - rpmsg router hub for router core
 *   name - endpoint name (r:dst_cpu:name)
 *   dest - destination address
 *
 * Returned Values:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static bool rpmsg_router_hub_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv, FAR const char *name,
                                   uint32_t dest)
{
  FAR struct rpmsg_router_hub_s *hub = priv;
  int i;

  if (strncmp(name, RPMSG_ROUTER_NAME_PREFIX, RPMSG_ROUTER_NAME_PREFIX_LEN))
    {
      return false;
    }

  /* Must match both source edge CPU and dest edge CPU simultaneously */

  for (i = 0; i < 2; i++)
    {
      if (strcmp(rpmsg_get_cpuname(rdev), hub->cpuname[i]))
        {
          continue;
        }

      if (strncmp(name + RPMSG_ROUTER_NAME_PREFIX_LEN,
                  hub->cpuname[1 - i], strlen(hub->cpuname[1 - i])))
        {
          continue;
        }

      break;
    }

  return i < 2;
}

/****************************************************************************
 * Name: rpmsg_router_hub_bind
 *
 * Description:
 *   This function is used to bind the router core device.
 *   It will try to create endpoint (r:src_cpu:name) to another dest cpu.
 *   The source endpoint information will be saved in the private field of
 *   the dest endpoint.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - rpmsg router hub for router core
 *   name - source edge core endpoint name (r:dst_cpu:name)
 *   dest - destination address
 *
 ****************************************************************************/

static void rpmsg_router_hub_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv, FAR const char *name,
                                  uint32_t dest)
{
  FAR struct rpmsg_router_hub_s *hub = priv;
  FAR struct rpmsg_endpoint *src_ept;
  FAR struct rpmsg_endpoint *dst_ept;
  FAR struct rpmsg_device *dst_rdev;
  char dst_name[RPMSG_NAME_SIZE];
  int ret;
  int i;

  nxmutex_lock(&hub->lock);
  metal_mutex_acquire(&rdev->lock);
  if (rpmsg_get_endpoint(rdev, name, RPMSG_ADDR_ANY, dest))
    {
      metal_mutex_release(&rdev->lock);
      nxmutex_unlock(&hub->lock);
      return;
    }

  metal_mutex_release(&rdev->lock);

  /* Try to create endpoint name(r:src_cpu:name) of another dest cpu */

  for (i = 0; i < 2; i++)
    {
      if (!strcmp(hub->cpuname[i], rpmsg_get_cpuname(rdev)))
        {
          break;
        }
    }

  DEBUGASSERT(i < 2);

  dst_rdev = hub->ept[1 - i].rdev;
  snprintf(dst_name, RPMSG_NAME_SIZE,
           RPMSG_ROUTER_NAME_PREFIX"%s%s", hub->cpuname[i],
           name + RPMSG_ROUTER_NAME_PREFIX_LEN +
           strlen(hub->cpuname[1 - i]));

  src_ept = kmm_zalloc(sizeof(*src_ept));
  dst_ept = kmm_zalloc(sizeof(*dst_ept));

  DEBUGASSERT(src_ept && dst_ept);

  /* Save information for the ept(r:dst_cpu:name) of the source cpu */

  src_ept->priv = dst_ept;
  src_ept->rdev = rdev;
  src_ept->dest_addr = dest;
  strlcpy(src_ept->name, name, sizeof(src_ept->name));

  /* Create endpoint (r:src_cpu:name) to another dest cpu */

  dst_ept->priv = src_ept;
  dst_ept->ns_bound_cb = rpmsg_router_hub_bound;
  ret = rpmsg_create_ept(dst_ept, dst_rdev, dst_name,
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         rpmsg_router_hub_cb,
                         rpmsg_router_hub_unbind);
  if (ret < 0)
    {
      kmm_free(dst_ept);
      kmm_free(src_ept);
    }

  nxmutex_unlock(&hub->lock);
}

/****************************************************************************
 * Name: rpmsg_router_bound
 *
 * Description:
 *   This function is used to send tx/rx buffer size
 *   when both cores are ready
 *
 * Parameters:
 *   ept - rpmsg endpoint for communicating with edge core
 *
 ****************************************************************************/

static void rpmsg_router_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_router_hub_s *hub = ept->priv;
  struct rpmsg_router_s msg;
  int ret;
  int i;

  if (!is_rpmsg_ept_ready(&hub->ept[0]) ||
      !is_rpmsg_ept_ready(&hub->ept[1]))
    {
      return;
    }

  for (i = 0; i < 2; i++)
    {
      msg.cmd = RPMSG_ROUTER_CREATE;
      msg.tx_len = MIN(rpmsg_get_rx_buffer_size(&hub->ept[i]),
                       rpmsg_get_tx_buffer_size(&hub->ept[1 - i]));
      msg.rx_len = MIN(rpmsg_get_tx_buffer_size(&hub->ept[i]),
                       rpmsg_get_rx_buffer_size(&hub->ept[1 - i]));
      strlcpy(msg.cpuname, hub->cpuname[i], sizeof(msg.cpuname));
      ret = rpmsg_send(&hub->ept[i], &msg, sizeof(msg));
      DEBUGASSERT(ret >= 0);
    }
}

/****************************************************************************
 * Name: rpmsg_router_cb
 ****************************************************************************/

static int rpmsg_router_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                           size_t len, uint32_t src, FAR void *priv)
{
  return 0;
}

/****************************************************************************
 * Name: rpmsg_router_created
 *
 * Description:
 *   This function is used to create endpoint to edge core,
 *   for synchronizing ready messages.
 *
 * Parameters:
 *   rdev - real rpmsg device
 *   priv - rpmsg router hub
 *
 ****************************************************************************/

static void rpmsg_router_created(FAR struct rpmsg_device *rdev,
                                 FAR void *priv)
{
  FAR struct rpmsg_router_hub_s *hub = priv;
  char name[RPMSG_NAME_SIZE];
  int ret;
  int i;

  for (i = 0; i < 2; i++)
    {
      if (strcmp(rpmsg_get_cpuname(rdev), hub->cpuname[i]))
        {
          continue;
        }

      hub->ept[i].priv = hub;
      hub->ept[i].ns_bound_cb = rpmsg_router_bound;
      snprintf(name, RPMSG_NAME_SIZE, RPMSG_ROUTER_NAME"%s",
               hub->cpuname[1 - i]);

      ret = rpmsg_create_ept(&hub->ept[i], rdev, name,
                             RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                             rpmsg_router_cb, NULL);
      DEBUGASSERT(ret == RPMSG_SUCCESS);
      break;
    }
}

/****************************************************************************
 * Name: rpmsg_router_destroy
 *
 * Description:
 *   This function is used to destroy the rpmsg router hub.
 *
 * Parameters:
 *   priv - rpmsg router hub
 *
 ****************************************************************************/

static void rpmsg_router_destroy(FAR struct rpmsg_device *rdev,
                                 FAR void *priv)
{
  FAR struct rpmsg_router_hub_s *hub = priv;
  struct rpmsg_router_s msg;
  int i;

  for (i = 0; i < 2; i++)
    {
      if (strcmp(rpmsg_get_cpuname(rdev), hub->cpuname[i]))
        {
          continue;
        }

      rpmsg_destroy_ept(&hub->ept[i]);

      /* Notify the other edge core to destroy router device */

      msg.cmd = RPMSG_ROUTER_DESTROY;
      rpmsg_send(&hub->ept[1 - i], &msg, sizeof(msg));
      break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_router_hub_init
 *
 * Description:
 *   This function is used to initialize the rpmsg router hub.
 *
 * Parameters:
 *   edge0 - edge cpu name
 *   edge1 - edge cpu name
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_router_hub_init(FAR const char *edge0,
                          FAR const char *edge1)
{
  FAR struct rpmsg_router_hub_s *hub;
  int ret;

  if (!edge0 || !edge1)
    {
      return -EINVAL;
    }

  hub = kmm_zalloc(sizeof(*hub));
  if (!hub)
    {
      return -ENOMEM;
    }

  nxmutex_init(&hub->lock);
  strlcpy(hub->cpuname[0], edge0, sizeof(hub->cpuname[0]));
  strlcpy(hub->cpuname[1], edge1, sizeof(hub->cpuname[1]));

  /* Register callback for retranmitting data between edge cores */

  ret = rpmsg_register_callback(hub,
                                rpmsg_router_created,
                                rpmsg_router_destroy,
                                rpmsg_router_hub_match,
                                rpmsg_router_hub_bind);
  if (ret < 0)
    {
      rpmsgerr("Register rpmsg callback failed: %d\n", ret);
      nxmutex_destroy(&hub->lock);
      kmm_free(hub);
      return ret;
    }

  return 0;
}
