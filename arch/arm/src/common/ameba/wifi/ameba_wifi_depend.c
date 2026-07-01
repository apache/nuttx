/****************************************************************************
 * arch/arm/src/common/ameba/wifi/ameba_wifi_depend.c
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
 * NuttX backing for the lwIP/event glue that the Realtek WHC host WiFi
 * libraries call out to.  In the vendor SDK these live in the lwIP netif
 * adapter, the rtw_event dispatcher and the rtk_app auto-reconnect / fast-
 * connect helpers -- all of which pull in lwIP, wpa_supplicant, p2p and the
 * AT-command service.  NuttX replaces lwIP with its native BSD network stack
 * and does not use those helpers, so they are provided here.
 *
 * The symbols NuttX does not exercise (lwIP netif up/down, the AT-command
 * service, P2P/WPS) are minimal stubs here; the live RX/TX data path and the
 * WiFi event indications run through the NuttX netdev (ameba_wlan.c) and the
 * SDK's own rtw_event.c dispatcher.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* This file sits on the SDK side of the boundary: it is compiled with the
 * vendor WiFi include set, NOT the NuttX one (the two header trees collide
 * -- e.g. lwIP vs NuttX atomic.h).  So it pulls only SDK/shim headers and
 * uses the SDK's DiagPrintf (linked from swlib/log.c) for diagnostics rather
 * than NuttX syslog.
 */

#include <stdint.h>

#include "lwip_netconf.h"        /* shim: struct pbuf, pbuf_layer/type */
#include <wifi_api.h>            /* u8/u32/s32, struct rtw_network_info */
#include <wifi_intf_drv_to_app_internal.h> /* struct internal_block_param */
#include <os_wrapper.h>          /* rtos_mem_zmalloc / rtos_mem_free */
#include <rtw_wifi_common.h>     /* struct eth_drv_sg */
#include <diag.h>                /* DiagPrintf (SDK swlib) */

/* WHC host TX (whc_ipc_host_send, == whc_host_send macro in IPC mode). */

extern int whc_ipc_host_send(int idx, struct eth_drv_sg *sg_list,
                             int sg_len, int total_len, void *raw_para,
                             u8 is_special_pkt);

/* NuttX netdev RX entry (ameba_wlan.c) -- raw Ethernet frame into devif. */

extern void ameba_wlan_rxframe(int idx, const unsigned char *buf,
                               unsigned int len);

/* WHC host join-state globals (defined in lib_wifi_whc_ap).  wifi_connect()
 * blocks on join_block_param->sema until the join-status event posts it.
 */

extern u8                          rtw_join_status;
extern int                         join_fail_reason;
extern struct internal_block_param *join_block_param;

/* Host-side WPA supplicant event handlers (lib_wpa_lite).  In the IPC scheme
 * the NP does scan/auth/assoc and forwards the PSK/SAE handshake to the host
 * via these events; the host supplicant drives the 4-way handshake.  We
 * dispatch them directly (instead of compiling the heavyweight rtw_event.c).
 */

/* SET_PSK_INFO */

extern void rtw_psk_set_psk_info_evt_hdl(u8 *evt_info);

/* STA_4WAY_START */

extern void rtw_psk_sta_start_4way(u8 *evt_info);

/* STA_4WAY_RECV */

extern void rtw_psk_sta_recv_eapol(u8 *evt_info);

/* EXTERNAL_AUTH */

extern void rtw_sae_sta_start(u8 *evt_info);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Packet-buffer glue + data path.
 *
 * Allocate a packet buffer for the WHC RX copy path.  The WHC host RX
 * (whc_ipc_host_trx.c, non-SHARE_TO_PBUF mode) does pbuf_alloc(PBUF_RAW,
 * len, PBUF_POOL) then memcpy's the frame into pbuf->payload.  A single
 * contiguous pbuf (struct + payload in one block) covers Ethernet-sized
 * frames, so the copy loop runs once and never chains.
 */

struct pbuf *pbuf_alloc(pbuf_layer layer, u16_t length, pbuf_type type)
{
  struct pbuf *p;

  (void)layer;
  (void)type;

  p = rtos_mem_zmalloc(sizeof(struct pbuf) + length);
  if (p == NULL)
    {
      return NULL;
    }

  p->payload = (void *)(p + 1);
  p->tot_len = length;
  p->len     = length;
  p->ref     = 1;
  p->next    = NULL;
  return p;
}

void pbuf_free(struct pbuf *p)
{
  if (p != NULL)
    {
      rtos_mem_free(p);
    }
}

/* RX seam: the WHC host RX path calls this with a received Ethernet frame
 * from the NP.  Hand the raw bytes to the NuttX netdev (ameba_wlan.c), then
 * free the pbuf (we own it -- copy mode).  Runs in the WHC host RX task
 * context.
 */

void netif_adapter_wifi_recv_whc(uint8_t idx, struct pbuf *p_buf)
{
  if (p_buf == NULL)
    {
      return;
    }

  ameba_wlan_rxframe(idx, (const unsigned char *)p_buf->payload,
                     p_buf->tot_len);
  pbuf_free(p_buf);
}

/* TX seam: send one contiguous Ethernet frame to the NP over WHC IPC.
 * Called by the NuttX netdev TX path.  is_special flags DHCP frames (the NP
 * prioritises / does not power-save-defer them, mirroring the SDK lwIP
 * adapter).
 */

int ameba_wifi_txframe(int idx, const void *buf, unsigned int len,
                       unsigned char is_special)
{
  struct eth_drv_sg sg;

  sg.buf = (unsigned int)(uintptr_t)buf;
  sg.len = len;
  return whc_ipc_host_send(idx, &sg, 1, (int)len, NULL, is_special);
}

/* Query the STA MAC address (host API -> NP).  Thin wrapper so the NuttX
 * netdev can fill dev->d_mac without pulling SDK headers.  Referencing
 * wifi_get_mac_address here also keeps its real NP implementation (the NP
 * noused generator sees it in the NuttX image).  Returns 0 on success.
 */

int ameba_wifi_get_mac(int idx, unsigned char *mac)
{
  struct rtw_mac m;

  if (wifi_get_mac_address(idx, &m, 0) != RTK_SUCCESS)
    {
      return -1;
    }

  mac[0] = m.octet[0]; mac[1] = m.octet[1]; mac[2] = m.octet[2];
  mac[3] = m.octet[3]; mac[4] = m.octet[4]; mac[5] = m.octet[5];
  return 0;
}

/* Bring the WLAN netif administratively up.  In NuttX the netdev
 * (ameba_wlan.c) owns interface state, so this is a no-op here.
 */

void lwip_netif_set_up(uint8_t idx)
{
  (void)idx;
}

/* Counterpart called from wifi_stop_ap(); netdev state is owned by NuttX. */

void lwip_netif_set_down(uint8_t idx)
{
  (void)idx;
}

/* lwIP netif link hooks that the precompiled lib_wifi_whc_ap.a references
 * from wifi_start_ap()/wifi_stop_ap() (built with CONFIG_LWIP_LAYER on).
 * NuttX's netdev (ameba_wlan.c) owns carrier via ifup, so these are no-ops.
 */

void lwip_netif_set_link_up(unsigned char idx)
{
  (void)idx;
}

void lwip_netif_set_link_down(unsigned char idx)
{
  (void)idx;
}

/* WiFi event dispatch (wifi_event_init / wifi_indication /
 * wifi_event_handle and the per-event handlers) is provided by the SDK's own
 * table-driven dispatcher, component/wifi/common/rtw_event.c, which is
 * compiled into libameba_wifi.a.  Its event_internal_hdl[] table already
 * handles JOIN_STATUS (incl. posting join_block_param->sema for a
 * synchronous connect), the WPA 4-way handshake, SAE/OWE, etc., and stays in
 * sync with the SDK automatically.  We no longer hand-maintain a
 * wifi_event_handle() switch here.
 */

/* rtk_app helpers not used by the NuttX port. */

s32 wifi_set_autoreconnect(u8 enable)
{
  (void)enable;
  return 0;
}

int wifi_stop_autoreconnect(void)
{
  return 0;
}

/* Flash key-value store (rt_kv_set/get/delete, used for fast-connect / PMK
 * caching) is implemented NuttX-side in ameba_kv.c on top of the littlefs
 * data partition -- it cannot live here because this translation unit is
 * compiled with the lwIP-colliding SDK include set.
 */

void rtw_reconn_new_conn(struct rtw_network_info *connect_param)
{
  (void)connect_param;
}

void wifi_fast_connect_enable(unsigned char enable)
{
  (void)enable;
}

/* lwIP netif info accessors.
 *
 * The WHC host libs query the per-interface IP/GW/mask/MAC and netif state
 * through these lwIP-named accessors.  NuttX's netdev (ameba_wlan.c) owns
 * the interface state and IP config, so these return zeroed scratch -- the
 * SDK paths that would consume real values are not on the host's data path.
 */

static u8 g_ameba_ip_scratch[4];
static u8 g_ameba_mac_scratch[6];

u8 *lwip_get_ip(u8 idx)
{
  (void)idx;
  return g_ameba_ip_scratch;
}

u8 *lwip_get_gw(u8 idx)
{
  (void)idx;
  return g_ameba_ip_scratch;
}

u8 *lwip_get_mask(u8 idx)
{
  (void)idx;
  return g_ameba_ip_scratch;
}

u8 *lwip_get_mac(u8 idx)
{
  (void)idx;
  return g_ameba_mac_scratch;
}

u8 lwip_get_ipv6_enabled(void)
{
  return 0;
}

int lwip_is_valid_ip(int idx, unsigned char *ip_dest)
{
  (void)idx;
  (void)ip_dest;
  return 0;
}

void lwip_wlan_set_netif_info(int idx_wlan, void *dev,
                              unsigned char *dev_addr)
{
  (void)idx_wlan;
  (void)dev;
  (void)dev_addr;
}

/* AP-mode netif pointer; STA-only build leaves it NULL (AP path unused). */

void *pnetif_ap = NULL;

/* DHCP server helper (AP only). */

int dhcps_ip_in_table_check(void *pnetif, u8 gate, u8 d)
{
  (void)pnetif;
  (void)gate;
  (void)d;
  return 0;
}

/* atcmd stub + WTN/mesh.
 *
 * The SDK event handlers (rtw_event.c) emit AT-command status strings via
 * at_printf_indicate().  We do not link the AT-command service (it is the
 * NP's shell, disabled here), so provide a no-op: the WiFi event path needs
 * the symbol, but the indications themselves are unused on the NuttX host.
 */

void at_printf_indicate(const char *fmt, ...)
{
  (void)fmt;
}

/* The SDK event source (rtw_event.c) references handlers from subsystems we
 * do not link: EAP/802.1X enterprise auth (we do PSK/SAE) and the
 * auto-reconnect helper.  Their table entries / call sites are reachable
 * from the pulled-in rtw_event.o, so the symbols must resolve; the functions
 * themselves are never exercised (STA PSK/SAE).  Provide no-ops.
 *
 * (_sscanf_ss is supplied as the genuine SDK implementation, not here:
 * RTL8721Dx compiles swlib/sscanf_minimal.c, RTL8720F gets it from the
 * whole-archived ROM lib.  See the per-board Make.defs.)
 */

int get_eap_phase(void)
{
  return 0;
}

void eap_eapol_start_hdl(u8 *evt_info)
{
  (void)evt_info;
}

void eap_eapol_recvd_hdl(u8 *buf, s32 buf_len)
{
  (void)buf;
  (void)buf_len;
}

void eap_disconnected_hdl(void)
{
}

void rtw_reconn_join_status_hdl(u8 *evt_info)
{
  (void)evt_info;
}

void rtw_reconn_dhcp_status_hdl(u8 *evt_info)
{
  (void)evt_info;
}

void wtn_rnat_ap_init(u8 enable)
{
  (void)enable;
}

int wtn_socket_init(u8 enable, u8 rnat_ap_start)
{
  (void)enable;
  (void)rnat_ap_start;
  return 0;
}

int wtn_socket_send(u8 *buf, u32 len)
{
  (void)buf;
  (void)len;
  return 0;
}
