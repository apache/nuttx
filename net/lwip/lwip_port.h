/****************************************************************************
 * net/lwip/lwip_port.h
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

#ifndef __NET_LWIP_LWIP_PORT_H
#define __NET_LWIP_LWIP_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_LWIP

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <semaphore.h>

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board/MAC driver callbacks expected by the lwIP glue layer.
 *
 * This interface isolates the imported stack from NuttX network drivers.
 */

struct nuttx_lwip_link_ops
{
  int  (*ifup)(void *priv);
  int  (*ifdown)(void *priv);
  int  (*get_hwaddr)(void *priv, uint8_t *mac, size_t maclen);
  int  (*linkoutput)(void *priv, struct pbuf *p);
  struct pbuf *(*linkinput)(void *priv);
  bool (*is_link_up)(void *priv);
  int (*get_mac)(void *priv, uint8_t *mac);
};

struct nuttx_lwip_if
{
  struct netif nif;
  const struct nuttx_lwip_link_ops *ops;
  void *priv;

  sem_t rx_sem;
  pthread_t rx_thread;

  volatile bool running;
  volatile bool default_if;
  volatile bool rx_pending;
  volatile bool link_pending;
  volatile bool link_up;
  volatile bool ready;

  char ifname[3];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int nuttx_lwip_initialize(void);

int nuttx_lwip_add_interface(struct nuttx_lwip_if *lif,
                             const struct nuttx_lwip_link_ops *ops,
                             void *priv,
                             const char *ifname,
                             bool default_if,
                             const ip4_addr_t *ipaddr,
                             const ip4_addr_t *netmask,
                             const ip4_addr_t *gw);

void nuttx_lwip_remove_interface(struct nuttx_lwip_if *lif);

int nuttx_lwipif_rx_ready(struct nuttx_lwip_if *lif);
int nuttx_lwipif_link_event(struct nuttx_lwip_if *lif, bool up);

/* Implemented by ethernetif_nuttx.c */

err_t nuttx_ethernetif_init(struct netif *netif);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_LWIP */
#endif /* __NET_LWIP_LWIP_PORT_H */
