/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/gdwifi_netif_compat.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implements the network functions that the SDK wifi_manager expects from
 * the lwIP port (wifi_netif.c), now on top of the NuttX network stack.
 * Only what the SDK actually calls -- the binary seam with the prebuilt
 * library lives in gdwifi_netdev.c.
 *
 * The IP address is no longer configured by the SDK: NuttX is in charge
 * (dhcpc / ifconfig / renew).  The DHCP functions become no-ops that report
 * the netdev state.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/net.h>

#include "gdwifi_netdev.h"

/****************************************************************************
 * SDK Externals
 ****************************************************************************/

int  macif_tx_start(void *net_if, struct pbuf *buf,
                    void (*cfm_cb)(uint32_t, bool, void *), void *cfm_arg);
int  sys_sema_init_ext(void **sema, int max_count, int init_count);
int  sys_sema_down(void **sema, uint32_t timeout_ms);
void sys_sema_up(void **sema);
int  sys_mutex_init(void **mutex);
int  sys_mutex_get(void **mutex);
void sys_mutex_put(void **mutex);

net_buf_tx_t *net_buf_tx_alloc(uint32_t length);
void          net_buf_tx_pbuf_free(net_buf_tx_t *buf);

struct net_driver_s *gdwifi_netdev_get(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* EAPOL has to be synchronous: the 4-way handshake installs the keys right
 * after the frame goes out, so we wait for the MAC confirm.
 */

static void *g_l2_sema;
static void *g_l2_mutex;
static bool  g_l2_acked;

static bool  g_static_ip;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void net_l2_send_cfm(uint32_t frame_id, bool acknowledged, void *arg)
{
  g_l2_acked = acknowledged;
  sys_sema_up(&g_l2_sema);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int net_init(void)
{
  sys_sema_init_ext(&g_l2_sema, 1, 0);
  sys_mutex_init(&g_l2_mutex);
  return 0;
}

void net_deinit(void)
{
  /* wifi_deinit() calls this.  The wlan0 netdev stays registered -- it is
   * the driver unregister that removes it, not the wifi_manager teardown.
   */
}

/****************************************************************************
 * Name: net_l2_send
 *
 * Description:
 *   Send an L2 frame (EAPOL) straight through the MAC, bypassing the IP
 *   stack.  Blocks until the TX confirm -- the key installation depends on
 *   it.
 *
 ****************************************************************************/

int net_l2_send(void *net_if, const uint8_t *data, int data_len,
                uint16_t ethertype, const uint8_t *dst_addr, bool *ack)
{
  struct net_driver_s *dev = gdwifi_netdev_get();
  net_buf_tx_t *buf;
  uint8_t *p;
  int len = data_len;
  int ret;

  if (dst_addr != NULL)
    {
      len += 14;                       /* Ethernet header */
    }

  buf = net_buf_tx_alloc(len);
  if (buf == NULL)
    {
      return -1;
    }

  p = buf->payload;

  if (dst_addr != NULL)
    {
      memcpy(p, dst_addr, 6);
      memcpy(p + 6, dev->d_mac.ether.ether_addr_octet, 6);
      p[12] = (uint8_t)(ethertype >> 8);
      p[13] = (uint8_t)(ethertype & 0xff);
      p += 14;
    }

  memcpy(p, data, data_len);

  sys_mutex_get(&g_l2_mutex);

  ret = macif_tx_start(net_if, buf, net_l2_send_cfm, NULL);
  if (ret == 0)
    {
      sys_sema_down(&g_l2_sema, 0);    /* Wait for the confirm */

      if (ack != NULL)
        {
          *ack = g_l2_acked;
        }
    }
  else
    {
      net_buf_tx_pbuf_free(buf);
    }

  sys_mutex_put(&g_l2_mutex);
  return ret;
}

int net_if_get_name(void *net_if, char *name, int len)
{
  struct net_driver_s *dev = gdwifi_netdev_get();

  if (name == NULL || len <= 0)
    {
      return -1;
    }

  strlcpy(name, dev->d_ifname, len);
  return 0;
}

const uint8_t *net_if_get_mac_addr(void *net_if)
{
  return gdwifi_netdev_get()->d_mac.ether.ether_addr_octet;
}

void *net_if_find_from_name(const char *name)
{
  return gdwifi_netdev_get();
}

int net_if_is_static_ip(void)
{
  return g_static_ip;
}

void net_if_use_static_ip(bool static_ip)
{
  g_static_ip = static_ip;
}

/****************************************************************************
 * IP configuration: with the NuttX network stack the user is the one who
 * configures it (ifconfig / renew / dhcpc).  We keep the entry points so
 * that the wifi_manager compiles, and we report the real netdev state.
 ****************************************************************************/

void net_if_set_ip(void *net_if, uint32_t ip, uint32_t mask, uint32_t gw)
{
  /* Deliberate no-op: with the NuttX network stack the IP address belongs
   * to the user (ifconfig / renew / dhcpc).  The wifi_manager used to call
   * this at the end of the association with values from its own DHCP client
   * -- which no longer runs.
   */
}

int net_if_get_ip(void *net_if, uint32_t *ip, uint32_t *mask, uint32_t *gw)
{
  struct net_driver_s *dev = gdwifi_netdev_get();

  if (ip != NULL)
    {
      *ip = dev->d_ipaddr;
    }

  if (mask != NULL)
    {
      *mask = dev->d_netmask;
    }

  if (gw != NULL)
    {
      *gw = dev->d_draddr;
    }

  return 0;
}

void net_if_set_default(void *net_if)
{
}

/* The DHCP client is now the NuttX one (the "renew" command): here we only
 * report whether an IP address is already configured.
 */

int net_dhcp_start(void *net_if)
{
  return 0;
}

void net_dhcp_stop(void *net_if)
{
}

int net_dhcp_release(void *net_if)
{
  return 0;
}

bool net_dhcp_address_obtained(void *net_if)
{
  return gdwifi_netdev_get()->d_ipaddr != 0;
}

void net_if_send_gratuitous_arp(void *net_if)
{
}

int net_set_dns(uint32_t dns_server)
{
  return 0;
}

int net_get_dns(uint32_t *dns_server)
{
  if (dns_server != NULL)
    {
      *dns_server = 0;
    }

  return 0;
}

uint16_t net_ip_chksum(const void *dataptr, int len)
{
  const uint8_t *p = dataptr;
  uint32_t sum = 0;

  while (len > 1)
    {
      sum += (uint32_t)((p[0] << 8) | p[1]);
      p   += 2;
      len -= 2;
    }

  if (len > 0)
    {
      sum += (uint32_t)(p[0] << 8);
    }

  while (sum >> 16)
    {
      sum = (sum & 0xffff) + (sum >> 16);
    }

  return (uint16_t)(~sum);
}

int netif_is_up(void *net_if)
{
  return IFF_IS_UP(gdwifi_netdev_get()->d_flags) ? 1 : 0;
}

/****************************************************************************
 * Name: net_if_add
 *
 * Description:
 *   The SDK calls this when it creates the VIF, with the MAC address read
 *   from the efuse -- that is the right moment to register wlan0 with the
 *   NuttX network stack.
 *
 ****************************************************************************/

int net_if_add(void *net_if, const uint8_t *mac_addr, const uint32_t *ipaddr,
               const uint32_t *netmask, const uint32_t *gw, void *vif)
{
  static bool registered;

  if (registered)
    {
      return 0;
    }

  if (gdwifi_netdev_register(mac_addr) < 0)
    {
      nerr("ERROR: failed to register wlan0\n");
      return -1;
    }

  registered = true;
  ninfo("wlan0 registered (%02x:%02x:%02x:%02x:%02x:%02x)\n",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);
  return 0;
}

void net_if_remove(void *net_if)
{
}

/* SDK DHCP server (softAP): out of scope for the STA port */

int net_dhcpd_start(void *net_if)
{
  return -1;
}

void net_dhcpd_stop(void *net_if)
{
}

uint32_t dhcpd_find_ipaddr_by_macaddr(uint8_t *mac_addr)
{
  return 0;
}

void dhcpd_set_dns_server(uint32_t dns)
{
}
