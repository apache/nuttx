/****************************************************************************
 * arch/arm/src/common/ameba/wifi/ameba_wifi.c
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
 * WHC (WiFi Host Controller) host bring-up for the Ameba AP core.
 *
 * This is the SDK-header side of the WiFi port (compiled with the vendor
 * WiFi include set into libameba_wifi.a, NOT through the NuttX CSRCS path --
 * the two header trees collide).  It performs the host init sequence the SDK
 * runs from its AP main()/wifi_init_thread(), minus everything NuttX already
 * owns (clocks, heap, scheduler):
 *
 *     wifi_set_rom2flash();        // remap ROM wifi hooks to flash
 *     wifi_set_task_size();        // populate g_rtw_task_size
 *     whc_host_init();             // == whc_ipc_host_init() in IPC mode
 *     wifi_on(RTW_MODE_STA);       // power on the MAC/PHY on the NP
 *
 * The IPC transport itself (AP<->NP on-chip IPC interrupt + ipc_table_init)
 * is wired on the NuttX side in ameba_wifi_init.c, which calls
 * ameba_wifi_start from a dedicated task once the scheduler is running.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include "lwip_netconf.h"        /* shim (pulled in transitively by headers) */
#include <wifi_api.h>            /* wifi_on, RTW_MODE_STA, RTK_SUCCESS, ...  */
#include <wifi_intf_drv_to_app_internal.h> /* g_rtw_task_size              */
#include <os_wrapper.h>          /* rtos_mem_zmalloc / rtos_mem_free         */
#include <diag.h>                /* DiagPrintf (SDK swlib)                   */

#include "ameba_wlan.h"          /* struct ameba_scan_ap (plain ABI types)  */

/****************************************************************************
 * Public Function Prototypes (resolved from SDK libs / compiled sources)
 ****************************************************************************/

extern void wifi_set_rom2flash(void);
extern void wifi_set_task_size(void);   /* rtw_task_size.c (compiled)        */
extern void whc_ipc_host_init(void);    /* lib_wifi_whc_ap                   */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_wifi_start
 *
 * Description:
 *   Run the AP host WiFi init sequence and power on the radio in STA mode.
 *   Must be called from task context (wifi_on blocks on the NP round-trip
 *   over IPC), after ameba_wifi_init.c has brought up the IPC interrupt and
 *   ipc_table_init().
 *
 * Returned Value:
 *   0 on success, a negative value on failure.
 *
 ****************************************************************************/

int ameba_wifi_start(void)
{
  int ret;

  DiagPrintf("[ameba-wifi] WHC host bring-up (STA) ...\n");

  wifi_set_rom2flash();
  wifi_set_task_size();

  /* The vendor sizes these WHC host tasks for the lwIP model, where the IPC
   * message-queue task only hands RX frames to the tcpip mailbox.  In this
   * NuttX port the same task (whc_msg_q_task) runs the full devif input path
   * (ipv4_input/arp_input/...) inline, which overflows the stock ~704-byte
   * stack.  Enlarge the host tasks here -- before whc_ipc_host_init()
   * creates them -- instead of patching the SDK headers.
   */

  if (g_rtw_task_size.ipc_msg_q_task < 4096)
    {
      g_rtw_task_size.ipc_msg_q_task = 4096;
    }

  if (g_rtw_task_size.ipc_blk_api_task < 2048)
    {
      g_rtw_task_size.ipc_blk_api_task = 2048;
    }

  if (g_rtw_task_size.ipc_unblk_api_task < 4096)
    {
      g_rtw_task_size.ipc_unblk_api_task = 4096;
    }

  /* Host TX skb pool: the SDK default (4) is too small for sustained TCP
   * TX.  The pool drains, whc_ipc_host_send() returns -2 (drops the frame),
   * TCP retransmits, and under load the retransmits exhaust and the link is
   * torn down (send -> ENOTCONN).  UDP tolerates the drops so is unaffected.
   * A modest pool keeps TCP TX stable; 16 skbs ~= 27 KB of AP heap (vs 53 KB
   * at 32).
   */

  if (wifi_user_config.skb_num_ap < 16)
    {
      wifi_user_config.skb_num_ap = 16;
    }

  whc_ipc_host_init();

  /* Keep the vendor's default power-save (IPS) config: the WHC driver wakes
   * the radio automatically on the next host API call, so no special
   * override is needed here.
   */

  ret = wifi_on(RTW_MODE_STA);
  if (ret != RTK_SUCCESS)
    {
      DiagPrintf("[ameba-wifi] wifi_on(STA) failed: %d\n", ret);
      return -1;
    }

  DiagPrintf("[ameba-wifi] wifi_on(STA) ok\n");
  return 0;
}

/****************************************************************************
 * Name: ameba_wifi_scan_start
 *
 * Description:
 *   Run one blocking active scan.  Returns the number of APs found
 *   (>= 0) or a negative value on error.
 *
 ****************************************************************************/

int ameba_wifi_scan_start(void)
{
  struct rtw_scan_param scan_param;
  int ret;

  memset(&scan_param, 0, sizeof(scan_param));
  scan_param.options = RTW_SCAN_ACTIVE;

  ret = wifi_scan_networks(&scan_param, 1);
  return (ret < RTK_SUCCESS) ? -1 : ret;
}

/****************************************************************************
 * Name: ameba_wifi_scan_results
 *
 * Description:
 *   Copy up to max scan records into the caller's plain ameba_scan_ap array.
 *   Returns the number copied, or a negative value on error.
 *
 ****************************************************************************/

int ameba_wifi_scan_results(struct ameba_scan_ap *out, int max)
{
  struct rtw_scan_result *list;
  u32 ap_num = (u32)max;
  u32 i;
  int n;

  if (out == NULL || max <= 0)
    {
      return -1;
    }

  list = rtos_mem_zmalloc(ap_num * sizeof(struct rtw_scan_result));
  if (list == NULL)
    {
      return -1;
    }

  if (wifi_get_scan_records(&ap_num, list) < 0)
    {
      rtos_mem_free(list);
      return -1;
    }

  for (i = 0, n = 0; i < ap_num && n < max; i++)
    {
      u8 slen = list[i].ssid.len;

      if (slen > AMEBA_WLAN_SSID_MAXLEN)
        {
          slen = AMEBA_WLAN_SSID_MAXLEN;
        }

      memcpy(out[n].ssid, list[i].ssid.val, slen);
      out[n].ssid[slen] = '\0';
      out[n].ssid_len   = slen;
      memcpy(out[n].bssid, list[i].bssid.octet, 6);
      out[n].rssi       = list[i].signal_strength;
      out[n].channel    = (uint8_t)list[i].channel;
      out[n].security   = (list[i].security == RTW_SECURITY_OPEN) ? 0 : 1;
      n++;
    }

  rtos_mem_free(list);
  return n;
}

/****************************************************************************
 * Name: ameba_wifi_connect
 *
 * Description:
 *   Connect (blocking) to ssid with the given PSK.  Empty password => open.
 *   Returns 0 on success, a negative value on failure.
 *
 ****************************************************************************/

int ameba_wifi_connect(const unsigned char *ssid, int ssid_len,
                       const unsigned char *pw, int pw_len)
{
  struct rtw_network_info info;
  int ret;

  if (ssid == NULL || ssid_len <= 0 || ssid_len > RTW_ESSID_MAX_SIZE)
    {
      return -1;
    }

  memset(&info, 0, sizeof(info));
  info.ssid.len = (u8)ssid_len;
  memcpy(info.ssid.val, ssid, ssid_len);
  info.channel = 0;   /* full scan to find the AP */

  if (pw != NULL && pw_len > 0)
    {
      info.password       = (u8 *)pw;
      info.password_len   = pw_len;
      info.security_type  = RTW_SECURITY_WPA_WPA2_MIXED_PSK;
    }
  else
    {
      info.security_type  = RTW_SECURITY_OPEN;
    }

  DiagPrintf("[ameba-wifi] connecting to \"%s\" ...\n", info.ssid.val);
  ret = wifi_connect(&info, 1);
  if (ret != RTK_SUCCESS)
    {
      DiagPrintf("[ameba-wifi] connect failed: %d\n", ret);
      return -1;
    }

  DiagPrintf("[ameba-wifi] connected\n");
  return 0;
}

/****************************************************************************
 * Name: ameba_wifi_disconnect
 ****************************************************************************/

int ameba_wifi_disconnect(void)
{
  return (wifi_disconnect() == RTK_SUCCESS) ? 0 : -1;
}

/****************************************************************************
 * Name: ameba_wifi_start_ap
 *
 * Description:
 *   Start a SoftAP on WiFi interface-1 with the given SSID/PSK/channel.
 *   An empty (or too-short) password starts an open AP; otherwise WPA2-AES
 *   PSK is used.  wifi_on() has already run (STA mode) in
 *   ameba_wifi_start(), which the SDK requires before wifi_start_ap(); the
 *   resulting STA+SoftAP concurrency is harmless when only the AP is used.
 *   When a STA link is up the SoftAP follows the STA channel (SDK
 *   behavior), so channel is only a hint for the standalone-AP case.
 *   Blocks on the NP round-trip, so it must run from task context (it does
 *   -- driven from the wapi ioctl path).
 *
 *   Returns 0 on success, a negative value on failure.
 *
 ****************************************************************************/

int ameba_wifi_start_ap(const unsigned char *ssid, int ssid_len,
                        const unsigned char *pw, int pw_len, int channel)
{
  struct rtw_softap_info ap;
  int ret;

  if (ssid == NULL || ssid_len <= 0 || ssid_len > RTW_ESSID_MAX_SIZE)
    {
      return -1;
    }

  memset(&ap, 0, sizeof(ap));
  ap.ssid.len = (u8)ssid_len;
  memcpy(ap.ssid.val, ssid, ssid_len);
  ap.channel  = (channel >= 1 && channel <= 165) ? (u8)channel : 1;

  if (pw != NULL && pw_len >= RTW_MIN_PSK_LEN)
    {
      ap.password      = (u8 *)pw;
      ap.password_len  = (u8)pw_len;
      ap.security_type = RTW_SECURITY_WPA2_AES_PSK;
    }
  else
    {
      ap.security_type = RTW_SECURITY_OPEN;
    }

  DiagPrintf("[ameba-wifi] start AP \"%s\" ch=%d %s ...\n",
             ap.ssid.val, ap.channel,
             ap.security_type == RTW_SECURITY_OPEN ? "open" : "wpa2");

  ret = wifi_start_ap(&ap);
  if (ret != RTK_SUCCESS)
    {
      DiagPrintf("[ameba-wifi] wifi_start_ap failed: %d\n", ret);
      return -1;
    }

  DiagPrintf("[ameba-wifi] SoftAP started\n");
  return 0;
}

/****************************************************************************
 * Name: ameba_wifi_stop_ap
 ****************************************************************************/

int ameba_wifi_stop_ap(void)
{
  return (wifi_stop_ap() == RTK_SUCCESS) ? 0 : -1;
}
