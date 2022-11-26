/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_netdev.c
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
#include <string.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/net/arp.h>
#include <nuttx/kmalloc.h>
#include "amebaz_netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void amebaz_netdev_notify_tx_done(struct amebaz_dev_s *priv)
{
  work_queue(LPWORK, &priv->pollwork, amebaz_txavail_work, priv, 0);
}

static int amebaz_txpoll(struct net_driver_s *dev)
{
  struct amebaz_dev_s *priv = (struct amebaz_dev_s *)dev->d_private;

  net_lock();
  if (!priv->curr)
    {
      net_unlock();
      amebaz_netdev_notify_tx_done(priv);
      return false;
    }

  DEBUGASSERT(priv->curr->tail == priv->dev.d_buf);
  skb_put(priv->curr, priv->dev.d_len);
  rltk_wlan_send_skb(priv->devnum, priv->curr);
  priv->dev.d_buf = NULL;
  priv->curr = NULL;
  net_unlock();
  NETDEV_TXPACKETS(&priv->dev);
  amebaz_netdev_notify_tx_done(priv);
  return true;
}

static int amebaz_transmit(struct amebaz_dev_s *priv)
{
  struct sk_buff *skb;
  skb = rltk_wlan_alloc_skb(priv->dev.d_len);
  if (!skb)
    {
      NETDEV_TXERRORS(&priv->dev);
      return -ENOMEM;
    }

  NETDEV_TXPACKETS(&priv->dev);
  memcpy(skb->tail, priv->dev.d_buf, priv->dev.d_len);
  skb_put(skb, priv->dev.d_len);
  rltk_wlan_send_skb(priv->devnum, skb);
  NETDEV_TXDONE(&priv->dev);
  return OK;
}

static void amebaz_reply(struct amebaz_dev_s *priv)
{
  if (priv->dev.d_len > 0)
    {
      amebaz_transmit(priv);
    }
}

void amebaz_netdev_notify_receive(struct amebaz_dev_s *priv,
                                  int index, unsigned int len)
{
  struct net_driver_s *dev = &priv->dev;
  struct eth_hdr_s *hdr;
  struct sk_buff *skb;
  void *oldbuf;
  skb = rltk_wlan_get_recv_skb(index);
  if (skb == NULL)
    {
      return;
    }

  if (!IFF_IS_UP(dev->d_flags))
    {
      skb_pull(skb, len);
      return;
    }

  NETDEV_RXPACKETS(&priv->dev);
  net_lock();
  oldbuf = priv->dev.d_buf;
  hdr = (struct eth_hdr_s *)skb->data;
  priv->dev.d_buf = (void *)skb->data;
  priv->dev.d_len = len;
#ifdef CONFIG_NET_PKT
  pkt_input(&priv->dev);
#endif
  if (hdr->type == HTONS(TPID_8021QVLAN))
    {
      uint8_t temp_buffer[12];
      memcpy(temp_buffer, skb->data, 12);
      memcpy(skb->data + 4, temp_buffer, 12);
      priv->dev.d_buf = skb->data = skb->data + 4;
      priv->dev.d_len -= 4;
    }

#ifdef CONFIG_NET_IPv4
  if (hdr->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(&priv->dev);
      ipv4_input(&priv->dev);
      amebaz_reply(priv);
    }

  else
    {
#endif
#ifdef CONFIG_NET_IPv6
      if (hdr->type == HTONS(ETHTYPE_IP6))
        {
          NETDEV_RXIPV6(&priv->dev);
          ipv6_input(&priv->dev);
          amebaz_reply(priv);
        }

      else
        {
#endif
#ifdef CONFIG_NET_ARP
          if (hdr->type == HTONS(ETHTYPE_ARP))
            {
              arp_arpin(&priv->dev);
              NETDEV_RXARP(&priv->dev);
              if (priv->dev.d_len > 0)
                {
                  amebaz_transmit(priv);
                }
            }

          else
#endif
            {
              NETDEV_RXDROPPED(&priv->dev);
            }
#ifdef CONFIG_NET_IPv6
        }
#endif
#ifdef CONFIG_NET_IPv4
    }
#endif

  skb_pull(skb, len);
  priv->dev.d_buf = oldbuf;
  net_unlock();
}

static void amebaz_txavail_work(void *arg)
{
  struct amebaz_dev_s *priv = (struct amebaz_dev_s *)arg;
  struct net_driver_s *dev = &priv->dev;
  net_lock();
  if (IFF_IS_UP(dev->d_flags))
    {
      if (!priv->curr && rltk_wlan_check_isup(priv->devnum))
        {
          priv->curr = rltk_wlan_alloc_skb(MAX_NETDEV_PKTSIZE);
          if (priv->curr)
            {
              priv->dev.d_buf = priv->curr->tail;
              priv->dev.d_len = 0;
            }
        }

      if (priv->dev.d_buf)
        {
          devif_poll(&priv->dev, amebaz_txpoll);
        }
    }

  net_unlock();
}

static int amebaz_txavail(struct net_driver_s *dev)
{
  struct amebaz_dev_s *priv = (struct amebaz_dev_s *)dev->d_private;
  if (work_available(&priv->pollwork))
    {
      work_queue(LPWORK, &priv->pollwork, amebaz_txavail_work, priv, 0);
    }

  return OK;
}

int amebaz_ioctl(struct net_driver_s *dev, int cmd,
                 unsigned long arg)
{
  struct amebaz_dev_s *priv = (struct amebaz_dev_s *)dev->d_private;
  int ret;
  if (!IFF_IS_UP(dev->d_flags) ||
      (!rltk_wlan_running(priv->devnum) && cmd != SIOCSIWMODE))
    {
      return -EINVAL;
    }

  switch (cmd)
    {
    case SIOCSIWSCAN:
      ret = amebaz_wl_start_scan(priv, (void *)arg);
      break;
    case SIOCGIWSCAN:
      ret = amebaz_wl_get_scan_results(priv, (void *)arg);
      break;
    case SIOCSIWENCODEEXT:
      ret = amebaz_wl_set_encode_ext(priv, (void *)arg);
      break;
    case SIOCGIWENCODEEXT:
      ret = amebaz_wl_get_encode_ext(priv, (void *)arg);
      break;
    case SIOCSIWESSID:
      ret = amebaz_wl_set_ssid(priv, (void *)arg);
      break;
    case SIOCSIWAP:
      ret = amebaz_wl_set_bssid(priv, (void *)arg);
      break;
    case SIOCSIWMODE:
      ret = amebaz_wl_set_mode(priv, (void *)arg);
      break;
    case SIOCSIWCOUNTRY:
      ret = amebaz_wl_set_country(priv, (void *)arg);
      break;
    case SIOCGIWFREQ:
      ret = amebaz_wl_get_freq(priv, (void *)arg);
      break;
    case SIOCSIWFREQ:
      ret = amebaz_wl_set_freq(priv, (void *)arg);
      break;
    case SIOCGIWAP:
    case SIOCGIWMODE:
    case SIOCGIWESSID:
    case SIOCGIWSENS:
    case SIOCSIWAUTH:
    case SIOCGIWAUTH:
    case SIOCSIFHWADDR:
    case SIOCGIFHWADDR:
    case SIOCSIWRATE:
    case SIOCGIWRATE:
    case SIOCSIWTXPOW:
    case SIOCGIWTXPOW:
      ret = amebaz_wl_process_command(priv, cmd, (void *)arg);
      break;
    default:
      wlwarn("ERROR: Unrecognized IOCTL command: %d\n", cmd);
      ret = -ENOTTY;  /* Special return value for this case */
      break;
    }

  return ret;
}

static int amebaz_ifup(struct net_driver_s *dev)
{
  struct amebaz_dev_s *priv = (struct amebaz_dev_s *)dev->d_private;
  if (!IFF_IS_UP(dev->d_flags))
    {
      priv->mode = RTW_MODE_NONE;
      priv->conn.status = AMEBAZ_STATUS_DISABLED;
    }

  return OK;
}

static int amebaz_ifdown(struct net_driver_s *dev)
{
  int ret = 0;
  struct amebaz_dev_s *priv = (struct amebaz_dev_s *)dev->d_private;
  irqstate_t flags;
  if (priv->devnum == 0 && rltk_wlan_running(1))
    {
      printf("must ifdown wlan 1 first\r\n");
      return ERROR;
    }

  flags = enter_critical_section();
  if (IFF_IS_UP(dev->d_flags))
    {
      if (priv->curr)
        {
          skb_put(priv->curr, 0);
          rltk_wlan_send_skb(priv->devnum, priv->curr);

          priv->curr = NULL;
        }

      if (priv->devnum == 0)
        {
          rltk_wlan_deinit();
        }

      else if (priv->mode == RTW_MODE_STA_AP)
        {
          ret = rltk_set_mode_prehandle(RTW_MODE_STA_AP,
                                        RTW_MODE_STA, "wlan0");
          rtw_msleep_os(50);
          ret = rltk_set_mode_posthandle(RTW_MODE_STA_AP,
                                         RTW_MODE_STA, "wlan0");
          while (rltk_wlan_running(1))
            {
              rtw_msleep_os(50);
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

int amebaz_netdev_register(struct amebaz_dev_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  dev->d_ifup    = amebaz_ifup;
  dev->d_ifdown  = amebaz_ifdown;
  dev->d_txavail = amebaz_txavail;
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl   = amebaz_ioctl;
#endif
  dev->d_private = priv;
  return netdev_register(dev, NET_LL_IEEE80211);
}

