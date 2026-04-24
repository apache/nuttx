/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_emac.c
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

#ifdef CONFIG_ESPRESSIF_EMAC

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_mac.h"

#include "esp_emac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETH_HEADER_LEN          (14)
#define ETH_VLANTAG_LEN         (4)
#define ETH_MAXPAYLOAD_LEN      (1500)
#define ETH_CRC_LEN             (4)

#define ETH_MAXPKT_LEN          (ETH_HEADER_LEN + ETH_VLANTAG_LEN + \
                                 ETH_MAXPAYLOAD_LEN + ETH_CRC_LEN)

#define EMAC_PHY_ADDR           (CONFIG_ESPRESSIF_ETH_PHY_ADDR)
#define NET2PRIV(_dev)          ((struct esp_emac_s *)(_dev))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_emac_s
{
  struct netdev_lowerhalf_s dev;

  esp_eth_handle_t    handle;

  bool                installed;
  bool                ifup;
  bool                link_up;

  spinlock_t          lock;
  netpkt_queue_t      rxq;
  uint8_t             txbuf[ETH_MAXPKT_LEN];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_emac_s g_esp_emac;
static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  emac_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);
static netpkt_t *emac_receive(struct netdev_lowerhalf_s *dev);
static esp_err_t emac_stack_input(esp_eth_handle_t hdl,
                                  uint8_t *buffer, uint32_t length,
                                  void *priv_arg);
static void emac_eth_event_handler(void *arg, esp_event_base_t base,
                                   int32_t id, void *data);
static int  emac_ifup(struct netdev_lowerhalf_s *dev);
static int  emac_ifdown(struct netdev_lowerhalf_s *dev);
#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  emac_addmac(struct netdev_lowerhalf_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  emac_rmmac(struct netdev_lowerhalf_s *dev, const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  emac_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                       unsigned long arg);
#endif

static const struct netdev_ops_s g_emac_ops =
{
  .ifup     = emac_ifup,
  .ifdown   = emac_ifdown,
  .transmit = emac_transmit,
  .receive  = emac_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = emac_addmac,
  .rmmac    = emac_rmmac,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = emac_ioctl,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emac_transmit
 *
 * Description:
 *   Push one outbound packet from the netdev lower-half TX path to esp_eth.
 *
 *   Backpressure handling:
 *
 *   When a TCP burst is in flight the EMAC DMA TX descriptor ring may be
 *   fully owned by the DMA engine.  In that case esp_eth_transmit() returns
 *   ESP_ERR_NO_MEM and we must wait until the hardware releases at least
 *   one descriptor before submitting the frame, otherwise the upper layer
 *   would treat the frame as lost and only recover via TCP retransmission,
 *   which collapses throughput.
 *
 *   The wait is implemented with sched_yield() rather than a fixed-time
 *   delay: it relinquishes the CPU to any runnable peer (most importantly
 *   the RX worker that consumes inbound TCP ACKs) without imposing an
 *   arbitrary sleep, while the EMAC DMA hardware drains its TX descriptors
 *   in parallel with no CPU involvement.
 *
 * Input Parameters:
 *   dev - Lower-half network device state.
 *   pkt - Packet to transmit.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int emac_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct esp_emac_s *priv = NET2PRIV(dev);
  unsigned int len;
  esp_err_t err;
  unsigned int retries = 0;
  int ret;

  len = netpkt_getdatalen(dev, pkt);

  if (!priv->installed || !priv->ifup)
    {
      return -ENETDOWN;
    }

  if (len == 0 || len > ETH_MAXPKT_LEN)
    {
      return -EMSGSIZE;
    }

  ret = netpkt_copyout(dev, priv->txbuf, pkt, len, 0);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      err = esp_eth_transmit(priv->handle, priv->txbuf, len);
      if (err != ESP_ERR_NO_MEM)
        {
          break;
        }

      sched_yield();
    }
  while (++retries < CONFIG_ESPRESSIF_ETH_TX_MAX_RETRIES);

  if (err != ESP_OK)
    {
      nerr("ERROR: esp_eth_transmit failed: %d\n", (int)err);
      return -EIO;
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  netdev_lower_txdone(dev);
  return OK;
}

/****************************************************************************
 * Name: emac_receive
 *
 * Description:
 *   Pop one received packet from the RX queue and hand it to upper-half.
 *
 * Input Parameters:
 *   dev - Lower-half network device state.
 *
 * Returned Value:
 *   Received packet pointer on success; NULL when no packet is queued.
 *
 ****************************************************************************/

static netpkt_t *emac_receive(struct netdev_lowerhalf_s *dev)
{
  struct esp_emac_s *priv = NET2PRIV(dev);
  irqstate_t flags;
  netpkt_t *pkt;

  flags = spin_lock_irqsave(&priv->lock);
  pkt = netpkt_remove_queue(&priv->rxq);
  spin_unlock_irqrestore(&priv->lock, flags);
  return pkt;
}

/****************************************************************************
 * Name: emac_stack_input
 *
 * Description:
 *   esp_eth input-path callback. Convert the received frame to netpkt and
 *   queue it to be consumed by netdev upper-half.
 *
 * Input Parameters:
 *   hdl      - esp_eth driver handle.
 *   buffer   - Frame buffer allocated by esp_eth (ownership transferred).
 *   length   - Frame length in bytes.
 *   priv_arg - Opaque pointer to struct esp_emac_s.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_ERR_NO_MEM on allocation or queue failures.
 *
 ****************************************************************************/

static esp_err_t emac_stack_input(esp_eth_handle_t hdl,
                                  uint8_t *buffer, uint32_t length,
                                  void *priv_arg)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)priv_arg;
  netpkt_t *pkt;
  irqstate_t flags;
  int ret;

  UNUSED(hdl);

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      nerr("ERROR: emac RX netpkt_alloc failed\n");
      free(buffer);
      return ESP_ERR_NO_MEM;
    }

  ret = netpkt_copyin(&priv->dev, pkt, buffer, length, 0);
  free(buffer);
  if (ret < 0)
    {
      netpkt_free(&priv->dev, pkt, NETPKT_RX);
      return ESP_ERR_NO_MEM;
    }

  flags = spin_lock_irqsave(&priv->lock);
  ret = netpkt_tryadd_queue(pkt, &priv->rxq);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (ret < 0)
    {
      nerr("ERROR: emac RX queue full\n");
      netpkt_free(&priv->dev, pkt, NETPKT_RX);
      return ESP_ERR_NO_MEM;
    }

  netdev_lower_rxready(&priv->dev);
  return ESP_OK;
}

/****************************************************************************
 * Name: emac_eth_event_handler
 *
 * Description:
 *   Receive ETH_EVENT notifications posted by esp_eth (link up/down,
 *   start/stop) and update NuttX carrier state accordingly.
 *
 * Input Parameters:
 *   arg  - Opaque pointer to struct esp_emac_s.
 *   base - Event base (unused; expected ETH_EVENT).
 *   id   - Event identifier (ETHERNET_EVENT_*).
 *   data - Event payload.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void emac_eth_event_handler(void *arg, esp_event_base_t base,
                                   int32_t id, void *data)
{
  struct esp_emac_s *priv = (struct esp_emac_s *)arg;

  UNUSED(base);
  UNUSED(data);

  switch (id)
    {
      case ETHERNET_EVENT_CONNECTED:
        ninfo("Ethernet link up\n");
        priv->link_up = true;
        netdev_lower_carrier_on(&priv->dev);
        break;

      case ETHERNET_EVENT_DISCONNECTED:
        ninfo("Ethernet link down\n");
        priv->link_up = false;
        netdev_lower_carrier_off(&priv->dev);
        break;

      case ETHERNET_EVENT_START:
        ninfo("Ethernet started\n");
        break;

      case ETHERNET_EVENT_STOP:
        ninfo("Ethernet stopped\n");
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: emac_ifup
 *
 * Description:
 *   Bring the network interface up and start the esp_eth state machine.
 *
 * Input Parameters:
 *   dev - NuttX network interface state.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int emac_ifup(struct netdev_lowerhalf_s *dev)
{
  struct esp_emac_s *priv = NET2PRIV(dev);
  esp_err_t err;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->netdev.d_ipaddr & 0xff),
        (int)((dev->netdev.d_ipaddr >> 8) & 0xff),
        (int)((dev->netdev.d_ipaddr >> 16) & 0xff),
        (int)((dev->netdev.d_ipaddr >> 24) & 0xff));
#endif

  if (!priv->installed)
    {
      nerr("ERROR: esp_eth driver not installed\n");
      return -ENODEV;
    }

  if (priv->ifup)
    {
      return OK;
    }

  err = esp_eth_start(priv->handle);
  if (err != ESP_OK)
    {
      nerr("ERROR: esp_eth_start failed: %d\n", (int)err);
      return -EIO;
    }

  priv->ifup = true;
  return OK;
}

/****************************************************************************
 * Name: emac_ifdown
 *
 * Description:
 *   Bring the network interface down and stop the esp_eth state machine.
 *
 * Input Parameters:
 *   dev - NuttX network interface state.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int emac_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct esp_emac_s *priv = NET2PRIV(dev);
  esp_err_t err;

  ninfo("Taking the network down\n");

  if (!priv->ifup)
    {
      return OK;
    }

  err = esp_eth_stop(priv->handle);
  if (err != ESP_OK)
    {
      nerr("ERROR: esp_eth_stop failed: %d\n", (int)err);
    }

  priv->ifup = false;
  priv->link_up = false;
  netdev_lower_carrier_off(dev);

  return OK;
}

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
/****************************************************************************
 * Name: emac_addmac
 *
 * Description:
 *   Add a multicast MAC address filter entry to the underlying ESP Ethernet
 *   driver.
 *
 * Input Parameters:
 *   dev - NuttX network interface state.
 *   mac - MAC address to add to the multicast filter list.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int emac_addmac(struct netdev_lowerhalf_s *dev, const uint8_t *mac)
{
  struct esp_emac_s *priv = NET2PRIV(dev);

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  esp_eth_ioctl(priv->handle, ETH_CMD_ADD_MAC_FILTER, (void *)mac);
  return OK;
}
#endif

#ifdef CONFIG_NET_MCASTGROUP
/****************************************************************************
 * Name: emac_rmmac
 *
 * Description:
 *   Remove a multicast MAC address filter entry from the underlying ESP
 *   Ethernet driver.
 *
 * Input Parameters:
 *   dev - NuttX network interface state.
 *   mac - MAC address to remove from the multicast filter list.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int emac_rmmac(struct netdev_lowerhalf_s *dev, const uint8_t *mac)
{
  struct esp_emac_s *priv = NET2PRIV(dev);

  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  esp_eth_ioctl(priv->handle, ETH_CMD_DEL_MAC_FILTER, (void *)mac);
  return OK;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Name: emac_ioctl
 *
 * Description:
 *   Handle network device ioctl requests supported by the ESP EMAC driver.
 *
 * Input Parameters:
 *   dev - NuttX network interface state.
 *   cmd - Ioctl command code.
 *   arg - Command-specific argument.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int emac_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                      unsigned long arg)
{
  struct esp_emac_s *priv = NET2PRIV(dev);
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
      case SIOCGMIIPHY:
        {
          struct mii_ioctl_data_s *req =
              (struct mii_ioctl_data_s *)((uintptr_t)arg);
          uint32_t phy_addr = 0;

          if (esp_eth_ioctl(priv->handle, ETH_CMD_G_PHY_ADDR, &phy_addr)
              == ESP_OK)
            {
              req->phy_id = (uint16_t)phy_addr;
              ret = OK;
            }
          else
            {
              ret = -EIO;
            }
        }
        break;

      case SIOCGMIIREG:
        {
          struct mii_ioctl_data_s *req =
              (struct mii_ioctl_data_s *)((uintptr_t)arg);
          esp_eth_phy_reg_rw_data_t rw;
          uint32_t val = 0;

          rw.reg_addr = req->reg_num;
          rw.reg_value_p = &val;
          if (esp_eth_ioctl(priv->handle, ETH_CMD_READ_PHY_REG, &rw)
              == ESP_OK)
            {
              req->val_out = (uint16_t)val;
              ret = OK;
            }
          else
            {
              ret = -EIO;
            }
        }
        break;

      case SIOCSMIIREG:
        {
          struct mii_ioctl_data_s *req =
              (struct mii_ioctl_data_s *)((uintptr_t)arg);
          esp_eth_phy_reg_rw_data_t rw;
          uint32_t val = req->val_in;

          rw.reg_addr = req->reg_num;
          rw.reg_value_p = &val;
          if (esp_eth_ioctl(priv->handle, ETH_CMD_WRITE_PHY_REG, &rw)
              == ESP_OK)
            {
              ret = OK;
            }
          else
            {
              ret = -EIO;
            }
        }
        break;
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&g_lock);
  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_emac_init
 *
 * Description:
 *   Create the esp_eth MAC+PHY+driver stack, register our RX callback and
 *   ETH_EVENT handler, pull the factory-programmed MAC address from eFuse
 *   and register the interface with the NuttX network stack.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_emac_init(void)
{
  struct esp_emac_s *priv = &g_esp_emac;
  esp_eth_mac_t *mac = NULL;
  esp_eth_phy_t *phy = NULL;
  unsigned int quota;
  uint8_t mac_addr[6];
  esp_err_t err;
  int ret;

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_esp32_emac_config_t esp32_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();

  memset(priv, 0, sizeof(*priv));
  IOB_QINIT(&priv->rxq);
  spin_lock_init(&priv->lock);

  /* Override default RMII/SMI pin configuration from Kconfig. */

  esp32_cfg.smi_gpio.mdc_num  = CONFIG_ESPRESSIF_ETH_MDC_GPIO;
  esp32_cfg.smi_gpio.mdio_num = CONFIG_ESPRESSIF_ETH_MDIO_GPIO;

#if defined(SOC_EMAC_USE_MULTI_IO_MUX) || defined(SOC_EMAC_MII_USE_GPIO_MATRIX)
  esp32_cfg.emac_dataif_gpio.rmii.tx_en_num  =
      CONFIG_ESPRESSIF_ETH_RMII_TX_EN_GPIO;
  esp32_cfg.emac_dataif_gpio.rmii.txd0_num   =
      CONFIG_ESPRESSIF_ETH_RMII_TXD0_GPIO;
  esp32_cfg.emac_dataif_gpio.rmii.txd1_num   =
      CONFIG_ESPRESSIF_ETH_RMII_TXD1_GPIO;
  esp32_cfg.emac_dataif_gpio.rmii.crs_dv_num =
      CONFIG_ESPRESSIF_ETH_RMII_CRS_DV_GPIO;
  esp32_cfg.emac_dataif_gpio.rmii.rxd0_num   =
      CONFIG_ESPRESSIF_ETH_RMII_RXD0_GPIO;
  esp32_cfg.emac_dataif_gpio.rmii.rxd1_num   =
      CONFIG_ESPRESSIF_ETH_RMII_RXD1_GPIO;
#endif

  esp32_cfg.clock_config.rmii.clock_gpio =
      CONFIG_ESPRESSIF_ETH_RMII_CLK_GPIO;

  phy_cfg.phy_addr = EMAC_PHY_ADDR;
#ifdef CONFIG_ESPRESSIF_ETH_ENABLE_PHY_RSTPIN
  phy_cfg.reset_gpio_num = CONFIG_ESPRESSIF_ETH_PHY_RST_GPIO;
#else
  phy_cfg.reset_gpio_num = -1;
#endif

  mac = esp_eth_mac_new_esp32(&esp32_cfg, &mac_config);
  if (mac == NULL)
    {
      nerr("ERROR: esp_eth_mac_new_esp32 failed\n");
      return -EIO;
    }

  phy = esp_eth_phy_new_generic(&phy_cfg);
  if (phy == NULL)
    {
      nerr("ERROR: esp_eth_phy_new_generic failed\n");
      mac->del(mac);
      return -EIO;
    }

    {
      esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);

      err = esp_eth_driver_install(&eth_cfg, &priv->handle);
      if (err != ESP_OK)
        {
          nerr("ERROR: esp_eth_driver_install failed: %d\n", (int)err);
          phy->del(phy);
          mac->del(mac);
          return -EIO;
        }
    }

  priv->installed = true;

  err = esp_eth_update_input_path(priv->handle, emac_stack_input, priv);
  if (err != ESP_OK)
    {
      nerr("ERROR: esp_eth_update_input_path failed: %d\n", (int)err);
      ret = -EIO;
      goto errout;
    }

  err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                   emac_eth_event_handler, priv);
  if (err != ESP_OK)
    {
      nerr("ERROR: esp_event_handler_register(ETH_EVENT) failed: %d\n",
           (int)err);
      ret = -EIO;
      goto errout;
    }

  /* Apply the factory-programmed MAC address (eFuse) to the MAC. */

  err = esp_read_mac(mac_addr, ESP_MAC_ETH);
  if (err != ESP_OK)
    {
      nwarn("WARNING: esp_read_mac(ESP_MAC_ETH) failed: %d\n", (int)err);
      mac_addr[0] = 0x02;
      mac_addr[1] = 0x00;
      mac_addr[2] = 0x00;
      mac_addr[3] = 0x00;
      mac_addr[4] = 0x00;
      mac_addr[5] = 0x01;
    }

  esp_eth_ioctl(priv->handle, ETH_CMD_S_MAC_ADDR, mac_addr);
  memcpy(priv->dev.netdev.d_mac.ether.ether_addr_octet, mac_addr, 6);
  priv->dev.ops = &g_emac_ops;

  /* Keep quotas well below the global netpkt pool size so lower-half
   * registration remains valid across different board configurations.
   */

  quota = NETPKT_BUFNUM / 4;
  if (quota == 0)
    {
      quota = 1;
    }

  priv->dev.quota[NETPKT_RX] = quota;
  priv->dev.quota[NETPKT_TX] = quota;
  priv->dev.rxtype = NETDEV_RX_WORK;
  priv->dev.priority = LPWORK;

  ret = netdev_lower_register(&priv->dev, NET_LL_ETHERNET);
  if (ret != 0)
    {
      nerr("ERROR: netdev_lower_register failed: %d\n", ret);
      goto errout_event;
    }

  ninfo("esp_emac: MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);

  return OK;

errout_event:
  esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID,
                               emac_eth_event_handler);

errout:
  if (priv->installed)
    {
      esp_eth_driver_uninstall(priv->handle);
      priv->installed = false;
    }

  return ret;
}

#endif /* CONFIG_ESPRESSIF_EMAC */
