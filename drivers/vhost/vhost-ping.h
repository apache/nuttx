/****************************************************************************
 * drivers/vhost/vhost-ping.h
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

#ifndef __DRIVERS_VHOST_VHOST_PING_H
#define __DRIVERS_VHOST_VHOST_PING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_DRIVERS_VHOST_PING

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Wrapper the vhost ping API to the shared ping implementation */

#define vhost_ping_logout       virtio_ping_logout
#define vhost_ping_logout_rate  virtio_ping_logout_rate

#define VHOST_PING_HDRSIZE      VIRTIO_PING_HDRSIZE

/* The queue indices are named from the driver's point of view, so this side
 * mirrors the roles: what the driver calls TX is what this side receives on.
 * Aliasing them here keeps the vhost code readable instead of forcing every
 * reader to invert the names in their head.
 */

#define VHOST_PING_VQ_RX        VIRTIO_PING_VQ_TX  /* Peer -> us */
#define VHOST_PING_VQ_TX        VIRTIO_PING_VQ_RX  /* Us -> peer */
#define VHOST_PING_VQ_NUM       VIRTIO_PING_VQ_NUM

/* Wrapper the shared ping types to their vhost names */

#define vhost_ping_hdr_s        virtio_ping_hdr_s
#define vhost_ping_config_s     virtio_ping_config_s
#define vhost_ping_s            virtio_ping_s

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct vhost_ping_s;

int vhost_register_ping_driver(void);
int vhost_ping(FAR const struct vhost_ping_s *ping);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VHOST_PING */

#endif /* __DRIVERS_VHOST_VHOST_PING_H */
