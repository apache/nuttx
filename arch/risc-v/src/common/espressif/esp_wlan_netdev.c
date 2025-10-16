/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_wlan_netdev.c
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

#include <errno.h>
#include <debug.h>
#include <arpa/inet.h>

#include <nuttx/wireless/wireless.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "esp_attr.h"

#include "esp_wifi_api.h"
#include "esp_wifi_utils.h"
#include "esp_wlan_netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only need 1 TX buf because packets are transmitted immediately and we can
 * reuse the same buffer for the next packet.
 * RX buf should match CONFIG_ESPRESSIF_WIFI_DYNAMIC_RX_BUFFER_NUM.
 */

 #define RX_BUF_COUNT CONFIG_ESPRESSIF_WIFI_DYNAMIC_RX_BUFFER_NUM
 #define TX_BUF_COUNT 1

 #define CONNECT_TIMEOUT CONFIG_ESPRESSIF_WIFI_CONNECT_TIMEOUT

/****************************************************************************
 * Private Types
 ****************************************************************************/

static int esp_wlan_ifup(struct netdev_lowerhalf_s *dev);
static int esp_wlan_ifdown(struct netdev_lowerhalf_s *dev);
static int esp_wlan_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);
static netpkt_t *esp_wlan_receive(struct netdev_lowerhalf_s *dev);
static void esp_wlan_reclaim(struct netdev_lowerhalf_s *dev);
static int esp_wlan_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                          unsigned long arg);

static int esp_wlan_connect(struct netdev_lowerhalf_s *dev);
static int esp_wlan_disconnect(struct netdev_lowerhalf_s *dev);
static int esp_wlan_essid(struct netdev_lowerhalf_s *dev,
                          struct iwreq *iwr, bool set);
static int esp_wlan_bssid(struct netdev_lowerhalf_s *dev,
                          struct iwreq *iwr, bool set);
static int esp_wlan_passwd(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set);
static int esp_wlan_mode(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set);
static int esp_wlan_auth(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set);
static int esp_wlan_freq(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set);
static int esp_wlan_bitrate(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set);
static int esp_wlan_txpower(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set);
static int esp_wlan_country(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set);
static int esp_wlan_sensitivity(struct netdev_lowerhalf_s *dev,
                                struct iwreq *iwr, bool set);
static int esp_wlan_scan(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set);
static int esp_wlan_range(struct netdev_lowerhalf_s *dev, struct iwreq *iwr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common Wi-Fi controller data */

struct esp_common_wifi_s
{
  bool driver_init;
};

/* Main driver control structure, contains the upper-half data structure
 * and lower-half driver information.
 */

struct esp_wlan_priv_s
{
  /* Upper-half interface */

  struct netdev_lowerhalf_s dev;

  /* Lower-half data */

  struct esp_common_wifi_s *common;
  uint32_t mode;

  netpkt_queue_t netdev_rx_queue;
  uint8_t flatbuf[CONFIG_NET_ETH_PKTSIZE];
};

/* Netdev operations */

static const struct netdev_ops_s g_netdev_ops =
{
    .ifup = esp_wlan_ifup,
    .ifdown = esp_wlan_ifdown,
    .transmit = esp_wlan_transmit,
    .receive = esp_wlan_receive,
    .ioctl = esp_wlan_ioctl,
    .reclaim = esp_wlan_reclaim,
};

/* Wireless operations */

static const struct wireless_ops_s g_wireless_ops =
{
  .connect = esp_wlan_connect,
  .disconnect = esp_wlan_disconnect,
  .essid = esp_wlan_essid,
  .bssid = esp_wlan_bssid,
  .passwd = esp_wlan_passwd,
  .mode = esp_wlan_mode,
  .auth = esp_wlan_auth,
  .freq = esp_wlan_freq,
  .bitrate = esp_wlan_bitrate,
  .txpower = esp_wlan_txpower,
  .country = esp_wlan_country,
  .sensitivity = esp_wlan_sensitivity,
  .scan = esp_wlan_scan,
  .range = esp_wlan_range,
};

/* Common Wi-Fi controller data */

struct esp_common_wifi_s g_common_wifi =
{
  .driver_init = false,
};

/* Station interface control structure */

#ifdef ESP_WLAN_HAS_STA
static struct esp_wlan_priv_s g_wlan_sta =
{
  .dev =
  {
    .ops = &g_netdev_ops,
    .iw_ops = &g_wireless_ops,
  },
  .common = &g_common_wifi,
  .mode = IW_MODE_INFRA,
};
#endif  /* ESP_WLAN_HAS_STA */

/* SoftAP interface control structure */

#ifdef ESP_WLAN_HAS_SOFTAP
static struct esp_wlan_priv_s g_wlan_softap =
{
  .dev =
  {
    .ops = &g_netdev_ops,
    .iw_ops = &g_wireless_ops,
  },
  .common = &g_common_wifi,
  .mode = IW_MODE_MASTER,
};
#endif  /* ESP_WLAN_HAS_SOFTAP */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wlan_ifup
 *
 * Description:
 *   Bring up the network device.
 *
 * Input Parameters:
 *   dev - The network device.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_ifup(struct netdev_lowerhalf_s *dev)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;
  struct net_driver_s *netdev = &priv->dev.netdev;
  int ret = OK;

#ifdef CONFIG_NET_IPv4
  wlinfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(netdev->d_ipaddr), ip4_addr2(netdev->d_ipaddr),
        ip4_addr3(netdev->d_ipaddr), ip4_addr4(netdev->d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  wlinfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        netdev->d_ipv6addr[0], netdev->d_ipv6addr[1], netdev->d_ipv6addr[2],
        netdev->d_ipv6addr[3], netdev->d_ipv6addr[4], netdev->d_ipv6addr[5],
        netdev->d_ipv6addr[6], netdev->d_ipv6addr[7]);
#endif

  /* Clear RX queue */

  netpkt_free_queue(&priv->netdev_rx_queue);

  /* Start Wi-Fi interface */

  ret = esp_wifi_api_start(priv->mode);
  if (ret)
    {
      wlerr("ERROR: Failed to start Wi-Fi station\n");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wlan_ifdown
 *
 * Description:
 *   Bring down the network device.
 *   If using STA + SoftAP, both interfaces are stopped, followed by another
 *   bringup of the other interface.
 *
 *   If using only STA, only the STA interface is stopped.
 *   If using only SoftAP, only the SoftAP interface is stopped.
 *   If using STA + SoftAP and request to stop SoftAP:
 *     Both are disabled and STA is restarted.
 *   If using STA + SoftAP and request to stop STA:
 *     Both are disabled and SoftAP is restarted.
 *
 * Input Parameters:
 *   dev - The network device.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;
  int ret = OK;

  ret = esp_wifi_api_stop(priv->mode);
  if (ret)
    {
      wlerr("ERROR: Failed to stop Wi-Fi station\n");
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wlan_transmit
 *
 * Description:
 *   Transmit function required by the netdev ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   pkt - The packet to transmit.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_transmit(struct netdev_lowerhalf_s *dev,
                             netpkt_t *pkt)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;
  unsigned int len = netpkt_getdatalen(dev, pkt);
  int ret = OK;

  /* Copy data from the packet to the flat buffer, then push it
   * to the Wi-Fi driver.
   */

  netpkt_copyout(dev, priv->flatbuf, pkt, len, 0);
#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      ret = esp_wifi_sta_send_data(priv->flatbuf, len);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      ret = esp_wifi_softap_send_data(priv->flatbuf, len);
    }
#endif

  if (ret < 0)
    {
      return ret;
    }

  /* Free the packet after sending */

  netpkt_free(dev, pkt, NETPKT_TX);

  return ret;
}

/****************************************************************************
 * Name: esp_wlan_receive
 *
 * Description:
 *   Receive function required by the netdev ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *
 * Returned Value:
 *   A pointer to the received packet.
 *
 ****************************************************************************/

static netpkt_t *esp_wlan_receive(struct netdev_lowerhalf_s *dev)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;
  netpkt_t *pkt = netpkt_remove_queue(&priv->netdev_rx_queue);
  return pkt;
}

/****************************************************************************
 * Name: esp_wlan_ioctl
 *
 * Description:
 *   Ioctl function required by the netdev ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   cmd - The command.
 *   arg - The argument.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int esp_wlan_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  /* TODO: Implement ioctl handler */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: esp_wlan_reclaim
 *
 * Description:
 *   Reclaim function required by the netdev ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_wlan_reclaim(struct netdev_lowerhalf_s *dev)
{
  /* TODO: Implement reclaim logic */
}

/* Skeleton implementations for wireless_ops_s functions */

/****************************************************************************
 * Name: esp_wlan_connect
 *
 * Description:
 *   Connect function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_connect(struct netdev_lowerhalf_s *dev)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  int ret;
  useconds_t timeout = 1000 * 1000;  /* 1 second */
  int timeout_count = CONNECT_TIMEOUT;
  volatile bool connection_success = false;
  volatile bool timeout_reached = false;

  if (priv->mode == IW_MODE_INFRA)
    {
      ret = esp_wifi_sta_connect();
      if (ret < 0)
        {
          wlerr("Failed to connect to Wi-Fi\n");
          return ret;
        }

      /* Wait for connection success or timeout.
       *
       * Note: IFF_RUNNING is set when WIFI_ADPT_EVT_STA_CONNECT event is
       *       received.
       */

      while (timeout_count >= 0 && !connection_success)
        {
          if (priv->dev.netdev.d_flags & IFF_RUNNING)
            {
              connection_success = true;
              break;
            }

          nxsched_usleep(timeout);
          timeout_count--;
        }

      if (!connection_success)
        {
          wlerr("Connection timeout after %d seconds\n", CONNECT_TIMEOUT);
          return -ETIMEDOUT;
        }

      return ret;
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_connect();
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_disconnect
 *
 * Description:
 *   Disconnect function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_disconnect(struct netdev_lowerhalf_s *dev)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_disconnect(false);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_disconnect();
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_essid
 *
 * Description:
 *   ESSID function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the ESSID is to be set, false if the ESSID is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_essid(struct netdev_lowerhalf_s *dev,
                          struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_essid(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_essid(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_bssid
 *
 * Description:
 *   BSSID function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the BSSID is to be set, false if the BSSID is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_bssid(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_bssid(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_bssid(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_passwd
 *
 * Description:
 *   Password function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the password is to be set, false if the password is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_passwd(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_password(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_password(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_mode
 *
 * Description:
 *   Mode function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if mode is to be set, false if the mode is to be retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_mode(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_mode(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_mode(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_auth
 *
 * Description:
 *   Authentication function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if authentication is to be set, false if the authentication
 *         is to be retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_auth(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_auth(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_auth(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_freq
 *
 * Description:
 *   Frequency function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if frequency is to be set, false if the frequency is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_freq(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_freq(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_freq(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_bitrate
 *
 * Description:
 *   Bitrate function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the bitrate is to be set, false if the bitrate is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_bitrate(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_bitrate(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_bitrate(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_txpower
 *
 * Description:
 *   TX power function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the TX power is to be set, false if the TX power is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_txpower(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_txpower(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_txpower(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_country
 *
 * Description:
 *   Country function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the country is to be set, false if the country is to be
 *         retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_country(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_country(iwr, set);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (priv->mode == IW_MODE_MASTER)
    {
      return esp_wifi_softap_country(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_sensitivity
 *
 * Description:
 *   Sensitivity function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if the sensitivity is to be set, false if the sensitivity is
 *         to be retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_sensitivity(struct netdev_lowerhalf_s *dev,
                                struct iwreq *iwr, bool set)
{
  struct esp_wlan_priv_s *priv = (struct esp_wlan_priv_s *)dev;

#ifdef ESP_WLAN_HAS_STA
  if (priv->mode == IW_MODE_INFRA)
    {
      return esp_wifi_sta_rssi(iwr, set);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_scan
 *
 * Description:
 *   Scan function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *   set - True if scan is to be started, false if the scan results are to
 *         be retrieved.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_scan(struct netdev_lowerhalf_s *dev,
                         struct iwreq *iwr, bool set)
{
#ifdef ESP_WLAN_HAS_STA
  if (set)
    {
      return esp_wifi_start_scan(iwr);
    }
  else
    {
      return esp_wifi_get_scan_results(iwr);
    }
#endif

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wlan_range
 *
 * Description:
 *   Range get function required by the wireless ops.
 *
 * Input Parameters:
 *   dev - The network device.
 *   iwr - The wireless request.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_range(struct netdev_lowerhalf_s *dev,
                          struct iwreq *iwr)
{
  /* TODO: Implement range get */

  return -ENOSYS;
}

/****************************************************************************
 * Name: esp_wifi_tx_done_cb
 *
 * Description:
 *   TX done callback. Called when the TX packet is sent. Informs the upper
 *   layer that the TX packet has been sent.
 *
 * Input Parameters:
 *   ifidx - The interface index.
 *   data  - The data of the TX packet.
 *   len   - The length of the TX packet.
 *   txstatus - The status of the TX packet.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void IRAM_ATTR esp_wifi_tx_done_cb(uint8_t ifidx,
                                   uint8_t *data,
                                   uint16_t *len,
                                   bool txstatus)
{
#ifdef ESP_WLAN_HAS_STA
  if (ifidx == ESP_IF_WIFI_STA)
    {
      netdev_lower_txdone(&g_wlan_sta.dev);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (ifidx == ESP_IF_WIFI_AP)
    {
      netdev_lower_txdone(&g_wlan_softap.dev);
    }
#endif
}

/****************************************************************************
 * Name: wlan_rx_done
 *
 * Description:
 *   RX done function. Allocates a new packet and copies the received
 *   data into a packet, which is then added to the receive queue.
 *
 *   When done, informs the upper layer that the packet is ready to be
 *   processed.
 *
 * Input Parameters:
 *   priv - The private data structure for the specified mode.
 *   buffer - The buffer containing the received packet.
 *   len - The length of the received packet.
 *   eb - The event buffer.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int wlan_rx_done(struct esp_wlan_priv_s *priv,
                        void *buffer, uint16_t len, void *eb)
{
  int ret = OK;
  netpkt_t *pkt = NULL;

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      ret = -ENOMEM;
      goto out;
    }

  ret = netpkt_copyin(&priv->dev, pkt, buffer, len, 0);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to copy packet\n");
      goto out;
    }

  ret = netpkt_tryadd_queue(pkt, &priv->netdev_rx_queue);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to add packet to queue\n");
      goto out;
    }

out:
  if (eb != NULL)
    {
      esp_wifi_api_free_rx_buffer(eb);
    }

  if (ret != OK && pkt != NULL)
    {
      netpkt_free(&priv->dev, pkt, NETPKT_RX);
    }

  netdev_lower_rxready(&priv->dev);

  return ret;
}

/****************************************************************************
 * Name: wlan_sta_rx_done
 *
 * Description:
 *   RX done callback for station mode. This call back is specified by
 *   IDF's API.
 *
 * Input Parameters:
 *   buffer - The buffer containing the received packet.
 *   len - The length of the received packet.
 *   eb - The event buffer.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
static int wlan_sta_rx_done(void *buffer, uint16_t len, void *eb)
{
  struct esp_wlan_priv_s *priv = &g_wlan_sta;

  return wlan_rx_done(priv, buffer, len, eb);
}
#endif  /* ESP_WLAN_HAS_STA */

/****************************************************************************
 * Name: wlan_softap_rx_done
 *
 * Description:
 *   RX done callback for SoftAP mode. This call back is specified by
 *   IDF's API.
 *
 * Input Parameters:
 *   buffer - The buffer containing the received packet.
 *   len - The length of the received packet.
 *   eb - The event buffer.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
static int wlan_softap_rx_done(void *buffer, uint16_t len, void *eb)
{
  struct esp_wlan_priv_s *priv = &g_wlan_softap;

  return wlan_rx_done(priv, buffer, len, eb);
}
#endif  /* ESP_WLAN_HAS_SOFTAP */

/****************************************************************************
 * Name: esp_wifi_initialize
 *
 * Description:
 *   Initialize the Wi-Fi controller.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wifi_initialize(void)
{
  int ret;

  wlinfo("INFO: Bring up Wi-Fi adapter\n");

  if (!g_common_wifi.driver_init)
    {
      ret = esp_wifi_api_adapter_init();
      if (ret < 0)
        {
          wlerr("ERROR: Initialize Wi-Fi adapter error: %d\n", ret);
          return ret;
        }

      g_common_wifi.driver_init = true;
    }
  else
    {
      wlwarn("WARN: Wi-Fi adapter is already initialized\n");
    }

  return OK;
}

/****************************************************************************
 * Name: esp_wlan_initialize
 *
 * Description:
 *   Initialize the Wi-Fi adapter for the specified mode.
 *
 *   1. Calls esp_wifi_initialize() to initialize the Wi-Fi controller. This
 *   should be done only once.
 *
 *   2. Reads the MAC address from the Wi-Fi controller.
 *
 *   3. Copies the MAC address to the netdev.
 *
 *   4. Sets the number of RX and TX buffers.
 *
 *   5. Registers the TX and RX callback.
 *
 *   6. Registers the network device at /dev/wlanN.
 *
 * Input Parameters:
 *   mode - The Wi-Fi mode to initialize (from wireless.h).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_wlan_initialize(uint32_t mode)
{
  int ret;
  uint8_t mac[6];
  struct esp_wlan_priv_s *priv = NULL;

  ret = esp_wifi_initialize();
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize Wi-Fi adapter\n");
      return ret;
    }

  switch (mode)
    {
      case IW_MODE_INFRA:
#ifdef ESP_WLAN_HAS_STA
        wlinfo("INFO: Initialize Wi-Fi station\n");
        priv = &g_wlan_sta;
        ret = esp_wifi_sta_read_mac(mac);
#endif
        break;
        case IW_MODE_MASTER:
#ifdef ESP_WLAN_HAS_SOFTAP
        wlinfo("INFO: Initialize Wi-Fi SoftAP\n");
        priv = &g_wlan_softap;
        ret = esp_wifi_softap_read_mac(mac);
#endif
        break;
      default:
        wlerr("ERROR: Invalid Wi-Fi mode\n");
        return -EINVAL;
    }

  if (ret < 0)
    {
      wlerr("ERROR: Failed to read MAC address\n");
      return ret;
    }

  wlinfo("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
         mac[0], mac[1], mac[2],
         mac[3], mac[4], mac[5]);

  /* Copy the ESP MAC Address to the netdev */

  memcpy(priv->dev.netdev.d_mac.ether.ether_addr_octet, mac, sizeof(mac));

  /* Set the number of RX and TX buffers. */

  priv->dev.quota[NETPKT_RX] = RX_BUF_COUNT;
  priv->dev.quota[NETPKT_TX] = TX_BUF_COUNT;

  IOB_QINIT(&priv->netdev_rx_queue);

  /* Register RX done callback. Called when the RX packet is received. */

#ifdef ESP_WLAN_HAS_STA
  if (mode == IW_MODE_INFRA)
    {
      ret = esp_wifi_api_sta_register_rx_callback(wlan_sta_rx_done);
    }
#endif

#ifdef ESP_WLAN_HAS_SOFTAP
  if (mode == IW_MODE_MASTER)
    {
      ret = esp_wifi_api_softap_register_rx_callback(wlan_softap_rx_done);
    }
#endif

  if (ret < 0)
    {
      wlerr("ERROR: Failed to register RX callback\n");
      return ret;
    }

  /* Register TX done callback. Called when the TX packet is sent. */

  ret = esp_wifi_api_register_tx_done_callback(esp_wifi_tx_done_cb);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to register TX done callback\n");
      return ret;
    }

  /* Registers the network device at /dev/wlanN */

  ret = netdev_lower_register((FAR struct netdev_lowerhalf_s *) priv,
                              NET_LL_IEEE80211);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to register IEEE 802.11 netdev: %d\n", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wlan_sta_connect_success_hook
 *
 * Description:
 *   Notify the networking layer that connection has succeeded.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
void esp_wlan_sta_connect_success_hook(void)
{
  struct esp_wlan_priv_s *priv = &g_wlan_sta;
  netdev_lower_carrier_on(&priv->dev);
}
#endif

/****************************************************************************
 * Name: esp_wlan_sta_disconnect_hook
 *
 * Description:
 *   Notify the networking layer that connection has been disconnected.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
void esp_wlan_sta_disconnect_hook(void)
{
  struct esp_wlan_priv_s *priv = &g_wlan_sta;
  netdev_lower_carrier_off(&priv->dev);
}
#endif

/****************************************************************************
 * Name: esp_wlan_softap_connect_success_hook
 *
 * Description:
 *   Notify the networking layer that connection has succeeded.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
void esp_wlan_softap_connect_success_hook(void)
{
  struct esp_wlan_priv_s *priv = &g_wlan_softap;
  netdev_lower_carrier_on(&priv->dev);
}
#endif

/****************************************************************************
 * Name: esp_wlan_softap_disconnect_hook
 *
 * Description:
 *   Notify the networking layer that connection has been disconnected.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
void esp_wlan_softap_disconnect_hook(void)
{
  struct esp_wlan_priv_s *priv = &g_wlan_softap;
  netdev_lower_carrier_off(&priv->dev);
}
#endif

/****************************************************************************
 * Name: esp_wlan_sta_initialize
 *
 * Description:
 *   Initialize the Wi-Fi adapter for station mode.
 *   Called from board level initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
int esp_wlan_sta_initialize(void)
{
  return esp_wlan_initialize(IW_MODE_INFRA);
}
#endif

/****************************************************************************
 * Name: esp_wlan_softap_initialize
 *
 * Description:
 *   Initialize the Wi-Fi adapter for SoftAP mode.
 *   Called from board level initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_SOFTAP
int esp_wlan_softap_initialize(void)
{
  return esp_wlan_initialize(IW_MODE_MASTER);
}
#endif  /* ESP_WLAN_HAS_SOFTAP */
