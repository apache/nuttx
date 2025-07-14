/****************************************************************************
 * include/nuttx/net/rpmsgdrv.h
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

#ifndef __INCLUDE_NUTTX_NET_RPMSGDRV_H
#define __INCLUDE_NUTTX_NET_RPMSGDRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/netdev_lowerhalf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_RPMSG_DRV

#define NET_RPMSG_EVENT_IF_UP       1
#define NET_RPMSG_EVENT_IF_DOWN     2
#define NET_RPMSG_EVENT_CARRIER_ON  3
#define NET_RPMSG_EVENT_CARRIER_OFF 4

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*net_rpmsg_drv_cb_t)(FAR struct netdev_lowerhalf_s *dev,
                                        int event);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: net_rpmsg_drv_init
 *
 * Description:
 *   Allocate a new network device instance for the RPMSG network and
 *   register it with the network device manager.  This is the client side of
 *   the RPMSG driver.  The RPMSG driver is the server side of the driver.
 *
 * Input Parameters:
 *   cpuname      - Remote CPU name
 *   devname      - Local and remote network device name
 *   lltype       - Link layer type
 *   priv         - Reference to the caller's private data
 *
 * Returned Value:
 *   A pointer to the allocated network device instance.  NULL is returned on
 *   failure.
 *
 ****************************************************************************/

FAR struct netdev_lowerhalf_s *
net_rpmsg_drv_init(FAR const char *cpuname, FAR const char *devname,
                   enum net_lltype_e lltype);

/****************************************************************************
 * Name: net_rpmsg_drv_priv
 *
 * Description:
 *   Get the private data associated with the network device instance.
 *
 * Input Parameters:
 *   dev - Reference to the network device instance.
 *
 ****************************************************************************/

FAR void *net_rpmsg_drv_priv(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: net_rpmsg_drv_set_callback
 *
 * Description:
 *   Set the callback function for the network device instance.
 *
 * Input Parameters:
 *   dev - Reference to the network device instance.
 *   cb  - Callback function to be set.
 *
 ****************************************************************************/

void net_rpmsg_drv_set_callback(FAR struct netdev_lowerhalf_s *dev,
                                net_rpmsg_drv_cb_t cb, FAR void *priv);

/****************************************************************************
 * Name: net_rpmsg_drv_server_init
 *
 * Description:
 *   Initialize the RPMSG network (for server side).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_RPMSG_DRV_SERVER
int net_rpmsg_drv_server_init(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif
#endif /* __INCLUDE_NUTTX_NET_RPMSGDRV_H */
