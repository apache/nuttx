/****************************************************************************
 * drivers/net/oa_tc6/oa_tc6.c
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

#include <assert.h>
#include <debug.h>
#include <sys/endian.h>

#include <nuttx/net/ioctl.h>

#ifdef CONFIG_NET_OA_TC6_NCV7410
#include "oa_tc6_ncv7410.h"
#endif

#ifdef CONFIG_NET_OA_TC6_LAN865X
#include "oa_tc6_lan865x.h"
#endif

#include "oa_tc6.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OA_TC6_WORK LPWORK

#define OA_TC6_N_TRIES 5

#define OA_TC6_RECOVERY_WORK_INTERVAL_MS 1000

/* Maximum frame size = (MTU + LL heaader size) + FCS size */

#define OA_TC6_MAX_FRAME_SIZE(p) (p->dev.netdev.d_pktsize + 4)

/* Packet Memory ************************************************************/

/* Maximum number of allocated TX and RX netpackets */

#define OA_TC6_TX_QUOTA 1
#define OA_TC6_RX_QUOTA 2

#if CONFIG_IOB_NBUFFERS < (OA_TC6_TX_QUOTA + OA_TC6_RX_QUOTA)
#  error "CONFIG_IOB_NBUFFERS must be > (OA_TC6_TX_QUOTA + OA_TC6_RX_QUOTA)"
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error "CONFIG_SCHED_LPWORK is needed by the OA-TC6 driver"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Bit calculations */

static int oa_tc6_get_parity(uint32_t word);

/* SPI transfers */

static int oa_tc6_poll_footer(FAR struct oa_tc6_driver_s *priv,
                              FAR uint32_t *footer);

static int oa_tc6_exchange_chunk(FAR struct oa_tc6_driver_s *priv,
                                 FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                                 uint32_t header, FAR uint32_t *footer);

/* Interrupt handling */

static int oa_tc6_interrupt(int irq, FAR void *context, FAR void *arg);
static void oa_tc6_interrupt_work(FAR void *arg);

/* SPI recovery */

static void oa_tc6_recovery_work(FAR void *arg);
static void oa_tc6_enter_recovery(FAR struct oa_tc6_driver_s *priv);
static void oa_tc6_exit_recovery(FAR struct oa_tc6_driver_s *priv);

/* Data Transaction Protocol logic */

static void oa_tc6_io_work(FAR void *arg);
static uint32_t oa_tc6_prep_chunk_exchange(FAR struct oa_tc6_driver_s *priv,
                                           FAR uint8_t *txbuf);
static bool oa_tc6_can_rx(FAR struct oa_tc6_driver_s *priv);
static void oa_tc6_try_finish_tx_packet(FAR struct oa_tc6_driver_s *priv);
static void oa_tc6_handle_rx_chunk(FAR struct oa_tc6_driver_s *priv,
                                   uint32_t footer, FAR uint8_t *rxbuf);
static void oa_tc6_finalize_rx_packet(FAR struct oa_tc6_driver_s *priv);
static void oa_tc6_release_tx_packet(FAR struct oa_tc6_driver_s *priv);
static void oa_tc6_release_rx_packet(FAR struct oa_tc6_driver_s *priv);

/* SPI utility functions */

static void oa_tc6_select_spi_raw(FAR struct spi_dev_s *spi,
                                  FAR const struct oa_tc6_config_s *config);
static void oa_tc6_deselect_spi_raw(FAR struct spi_dev_s *spi,
                                  FAR const struct oa_tc6_config_s *config);
#define oa_tc6_select_spi(priv) \
    oa_tc6_select_spi_raw((priv)->spi, (priv)->config)
#define oa_tc6_deselect_spi(priv) \
    oa_tc6_deselect_spi_raw((priv)->spi, (priv)->config)

/* OA-TC6 reset and configuration */

static int oa_tc6_read_reg_raw(FAR struct spi_dev_s *spi,
                               FAR const struct oa_tc6_config_s *config,
                               oa_tc6_regid_t regid, FAR uint32_t *word);
static int oa_tc6_get_phyid(FAR struct spi_dev_s *spi,
                            FAR const struct oa_tc6_config_s *config,
                            FAR uint32_t *phyid);
static int oa_tc6_init_by_id(FAR struct spi_dev_s *spi,
                             FAR const struct oa_tc6_config_s *config,
                             uint32_t phyid);

static int oa_tc6_reset(FAR struct oa_tc6_driver_s *priv);
static int oa_tc6_config(FAR struct oa_tc6_driver_s *priv);
static int oa_tc6_enable(FAR struct oa_tc6_driver_s *priv);
static int oa_tc6_disable(FAR struct oa_tc6_driver_s *priv);
static int oa_tc6_update_mac_filter(FAR struct oa_tc6_driver_s *priv);

/* Driver buffer manipulation */

static void oa_tc6_reset_driver_buffers(FAR struct oa_tc6_driver_s *priv);

/* MDIO */

static int oa_tc6_miireg_get_regid(FAR struct mii_ioctl_data_s *req,
                                   FAR oa_tc6_regid_t *regid);
static int oa_tc6_get_mmd_base(uint8_t mmd,
                               FAR uint8_t *mms, FAR uint16_t *addr);

/* NuttX callback functions */

static int oa_tc6_ifup(FAR struct netdev_lowerhalf_s *dev);
static int oa_tc6_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int oa_tc6_transmit(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt);
static FAR netpkt_t *oa_tc6_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int oa_tc6_addmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac);
static int oa_tc6_rmmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int oa_tc6_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                        unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_oa_tc6_ops =
{
  .ifup     = oa_tc6_ifup,
  .ifdown   = oa_tc6_ifdown,
  .transmit = oa_tc6_transmit,
  .receive  = oa_tc6_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = oa_tc6_addmac,
  .rmmac    = oa_tc6_rmmac,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = oa_tc6_ioctl
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oa_tc6_get_parity
 *
 * Description:
 *   Obtain parity of a 32-bit word.
 *
 * Input Parameters:
 *   word - 32-bit word, subject to the parity calculation
 *
 * Returned Value:
 *   If the parity of the word is even, zero is returned.
 *   Otherwise one is returned.
 *
 ****************************************************************************/

static int oa_tc6_get_parity(uint32_t word)
{
  /* www-graphics.stanford.edu/~seander/bithacks.html */

  word ^= word >> 1;
  word ^= word >> 2;
  word = (word & 0x11111111u) * 0x11111111u;
  return (word >> 28) & 1;
}

/****************************************************************************
 * Name: oa_tc6_poll_footer
 *
 * Description:
 *   Poll a data transaction chunk footer.
 *
 * Input Parameters:
 *   priv   - pointer to the driver-specific state structure
 *   footer - pointer to a 32-bit footer destination variable
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_poll_footer(FAR struct oa_tc6_driver_s *priv,
                              FAR uint32_t *footer)
{
  uint8_t *txbuf = priv->txbuf;
  uint8_t *rxbuf = priv->rxbuf;
  uint32_t header;

  header =   (1 << OA_TC6_DNC_POS)   /* Data Not Control */
           | (1 << OA_TC6_NORX_POS); /* No Read */

  if (oa_tc6_exchange_chunk(priv, txbuf, rxbuf, header, footer))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_exchange_chunk
 *
 * Description:
 *   Send a data chunk to MAC-PHY and simultaneously receive chunk.
 *
 *   Computing header parity, checking footer parity, converting to proper
 *   endianness and setting DNC flag is done by this function.
 *
 * Input Parameters:
 *   priv   - pointer to the driver-specific state structure
 *   txbuf  - buffer with transmit chunk data
 *   rxbuf  - buffer to save the received chunk to
 *   header - header controlling the transaction
 *   footer - pointer to a 32-bit value for the footer
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_exchange_chunk(FAR struct oa_tc6_driver_s *priv,
                                 FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                                 uint32_t header, FAR uint32_t *footer)
{
  int parity;
  int hdrb;
  int sync;

  header |= (1 << OA_TC6_DNC_POS);
  header |= (!oa_tc6_get_parity(header) << OA_TC6_P_POS);
  header = htobe32(header);

  ((uint32_t *)txbuf)[0] = header;

  oa_tc6_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txbuf, rxbuf, OA_TC6_CHUNK_SIZE(priv));
  oa_tc6_deselect_spi(priv);

  *footer = *((uint32_t *)(&rxbuf[priv->config->chunk_payload_size]));

  *footer = be32toh(*footer);

  parity = oa_tc6_get_parity(*footer);
  hdrb   = oa_tc6_header_bad(*footer);
  sync   = oa_tc6_mac_phy_sync(*footer);

  if (!parity || hdrb || !sync)
    {
      if (!parity)
        {
          nerr("Error: Wrong parity in the footer\n");
        }

      if (hdrb)
        {
          nerr("Error: HDRB set in the footer\n");
        }

      if (!sync)
        {
          nerr("Error: MAC-PHY lost configuration, "
               "SYNC cleared in the footer\n");
        }

      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_interrupt
 *
 * Description:
 *   Schedule interrupt work when the interrupt signal from MAC-PHY is
 *   received.
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *   arg     - oa_tc6_driver_s priv structure to be passed to the interrupt
 *             worker
 *
 * Returned Value:
 *   OK is always returned.
 *
 ****************************************************************************/

static int oa_tc6_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)arg;

  ninfo("Info: OA-TC6 interrupt!\n");

  /* Schedule interrupt work */

  work_queue(OA_TC6_WORK, &priv->interrupt_work,
             oa_tc6_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_interrupt_work
 *
 * Description:
 *   Identify the interrupt source and perform necessary work.
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_interrupt_work(FAR void *arg)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)arg;
  uint32_t footer;
  int tries = OA_TC6_N_TRIES;

  if (nxmutex_lock(&priv->lock))
    {
      nerr("Error: Failed to acquire lock\n");
      return;
    }

  if (priv->ifstate != OA_TC6_IFSTATE_UP)
    {
      ninfo("Info: Interrupt work invoked when the interface is not up\n");
      nxmutex_unlock(&priv->lock);
      return;
    }

  ninfo("Info: OA-TC6 interrupt worker invoked!\n");

  do
    {
      if (!oa_tc6_poll_footer(priv, &footer))
        {
          break;
        }

      nerr("Error: Polling footer unsuccessful\n");
    }
  while (--tries);

  if (!tries)
    {
      nerr("Error: Failed to poll footer in %d tries, carrier down\n",
           OA_TC6_N_TRIES);

      oa_tc6_enter_recovery(priv);

      nxmutex_unlock(&priv->lock);
      return;
    }

  if (oa_tc6_ext_status(footer))
    {
      /* Device-specific driver may implement special functionality on EXST */

      priv->ops->action(priv, OA_TC6_ACTION_EXST);
    }

  /* Update MAC-PHY buffer status */

  priv->txc = oa_tc6_tx_credits(footer);
  priv->rca = oa_tc6_rx_available(footer);

  if ((priv->tx_pkt && priv->txc) || priv->rca)
    {
      /* Schedule IO work */

      ninfo("Info: Scheduled io_work from interrupt_work\n");
      work_queue(OA_TC6_WORK, &priv->io_work, oa_tc6_io_work, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: oa_tc6_recovery_work
 *
 * Description:
 *   Every OA_TC6_RECOVERY_WORK_INTERVAL_MS milliseconds check whether the
 *   SPI is already available.
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_recovery_work(FAR void *arg)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)arg;
  uint32_t regval;

  if (nxmutex_lock(&priv->lock))
    {
      nerr("Error: Failed to acquire lock\n");
      return;
    }

  if (priv->ifstate != OA_TC6_IFSTATE_UP_RECOVERY)
    {
      ninfo("Info: Trying to recover when not in recovery\n");
      nxmutex_unlock(&priv->lock);
      return;
    }

  if (oa_tc6_read_reg(priv, OA_TC6_CONFIG0_REGID, &regval))
    {
      nwarn("Warning: MAC-PHY still not available\n");

      work_queue(OA_TC6_WORK, &priv->recovery_work,
                 oa_tc6_recovery_work, priv,
                 MSEC2TICK(OA_TC6_RECOVERY_WORK_INTERVAL_MS));

      nxmutex_unlock(&priv->lock);
      return;
    }

  ninfo("Info: Reading register successful during recovery\n");

  if (!oa_tc6_get_field(regval, CONFIG0_SYNC))
    {
      /* MAC-PHY has lost config. Shut down */

      nwarn("Warning: MAC-PHY has lost config (SYNC = '0'), "
            "shutting down\n");

      priv->ifstate = OA_TC6_IFSTATE_UP_UNKNOWN;

      /* Unlock before shutting down, otherwise would result in deadlock */

      nxmutex_unlock(&priv->lock);

      net_lock();
      netdev_ifdown(&priv->dev.netdev);
      net_unlock();

      return;
    }

  ninfo("Info: MAC-PHY SPI contact successful, exiting recovery\n");

  oa_tc6_exit_recovery(priv);

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: oa_tc6_enter_recovery
 *
 * Description:
 *   Disable the interface after the contact over the SPI is lost.
 *   Enter recovery mode in waiting for contact over SPI.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_enter_recovery(FAR struct oa_tc6_driver_s *priv)
{
  /* Disable interrupt on the board level */

  priv->config->enable(priv->config, false);

  work_cancel(OA_TC6_WORK, &priv->interrupt_work);
  work_cancel(OA_TC6_WORK, &priv->io_work);

  oa_tc6_reset_driver_buffers(priv);

  priv->ifstate = OA_TC6_IFSTATE_UP_RECOVERY;

  net_lock();
  netdev_lower_carrier_off(&priv->dev);
  net_unlock();

  work_queue(OA_TC6_WORK, &priv->recovery_work,
             oa_tc6_recovery_work, priv,
             MSEC2TICK(OA_TC6_RECOVERY_WORK_INTERVAL_MS));
}

/****************************************************************************
 * Name: oa_tc6_exit_recovery
 *
 * Description:
 *   Re-enable the interface after the contact over the SPI is recovered.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_exit_recovery(FAR struct oa_tc6_driver_s *priv)
{
  priv->ifstate = OA_TC6_IFSTATE_UP;

  net_lock();
  netdev_lower_carrier_on(&priv->dev);
  net_unlock();

  work_queue(OA_TC6_WORK, &priv->interrupt_work,
             oa_tc6_interrupt_work, priv, 0);

  /* Enable interrupt on the board level */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: oa_tc6_io_work
 *
 * Description:
 *   Exchange data chunk with the MAC-PHY.
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_io_work(FAR void *arg)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)arg;

  uint8_t *txbuf = priv->txbuf;
  uint8_t *rxbuf = priv->rxbuf;

  uint32_t header;
  uint32_t footer;

  if (nxmutex_lock(&priv->lock))
    {
      nerr("Error: Failed to acquire lock\n");
      return;
    }

  if (priv->ifstate != OA_TC6_IFSTATE_UP)
    {
      nerr("Error: Trying to work when the interface is not up\n");
      nxmutex_unlock(&priv->lock);
      return;
    }

  header = oa_tc6_prep_chunk_exchange(priv, txbuf);

  /* Perform the SPI exchange */

  if (oa_tc6_exchange_chunk(priv, txbuf, rxbuf, header, &footer))
    {
      nerr("Error: Chunk exchange failed\n");

      /* Reset buffers, effectively dropping frames */

      oa_tc6_reset_driver_buffers(priv);

      /* Plan the interrupt work to try and find out what's going on */

      work_queue(OA_TC6_WORK, &priv->interrupt_work,
                 oa_tc6_interrupt_work, priv, 0);

      nxmutex_unlock(&priv->lock);
      return;
    }

  if (oa_tc6_ext_status(footer))
    {
      priv->ops->action(priv, OA_TC6_ACTION_EXST);
    }

  oa_tc6_try_finish_tx_packet(priv);

  oa_tc6_handle_rx_chunk(priv, footer, rxbuf);

  /* Schedule further work if needed */

  if ((priv->tx_pkt && priv->txc) || priv->rca)
    {
      work_queue(OA_TC6_WORK, &priv->io_work, oa_tc6_io_work, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: oa_tc6_prep_chunk_exchange
 *
 * Description:
 *   Determine whether there is data to transmit or receive.
 *   Set the appropriate header bitfields and fill the txbuf accordingly.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   txbuf - pointer to the transmit chunk buffer
 *
 * Returned Value:
 *   Returns the prepared chunk header.
 *
 ****************************************************************************/

static uint32_t oa_tc6_prep_chunk_exchange(FAR struct oa_tc6_driver_s *priv,
                                           FAR uint8_t *txbuf)
{
  uint32_t header = 0;
  int txlen;

  if (priv->tx_pkt && priv->txc)
    {
      header |= (1 << OA_TC6_DV_POS);  /* Data Valid */

      if (priv->tx_pkt_idx == 0)
        {
          header |=   (1 << OA_TC6_SV_POS)   /* Start Valid           */
                    | (0 << OA_TC6_SWO_POS); /* Start Word Offset = 0 */
        }

      txlen = priv->tx_pkt_len - priv->tx_pkt_idx;

      if (txlen <= priv->config->chunk_payload_size)
        {
          header |=   (1 << OA_TC6_EV_POS)             /* End Valid       */
                    | ((txlen - 1) << OA_TC6_EBO_POS); /* End Byte Offset */
        }
      else
        {
          txlen = priv->config->chunk_payload_size;
        }

      /* Copy data from network to txbuf, leave 4 bytes for header */

      netpkt_copyout(&priv->dev, &txbuf[4], priv->tx_pkt,
                     txlen, priv->tx_pkt_idx);
      priv->tx_pkt_idx += txlen;
    }

  if (oa_tc6_can_rx(priv) == false)
    {
      header |= (1 << OA_TC6_NORX_POS);  /* No RX */
    }

  return header;
}

/****************************************************************************
 * Name: oa_tc6_can_rx
 *
 * Description:
 *   Determine whether rx data is available and whether it can be received.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   If it is possible to receive an rx chunk, true is returned,
 *   otherwise false is returned.
 *
 ****************************************************************************/

static bool oa_tc6_can_rx(FAR struct oa_tc6_driver_s *priv)
{
  if (!priv->rca)
    {
      return false;
    }

  if (priv->rx_pkt_ready)
    {
      return false;
    }

  if (priv->rx_pkt)
    {
      return true;
    }

  /* No RX packet, try to alloc */

  priv->rx_pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (priv->rx_pkt)
    {
      priv->rx_pkt_idx = 0;
      return true;
    }

  ninfo("Info: Failed to alloc rx netpkt\n");

  /* There is no buffer for RX data */

  return false;
}

/****************************************************************************
 * Name: oa_tc6_try_finish_tx_packet
 *
 * Description:
 *   Check whether the entire packet has been transmitted.
 *   If so, free the tx netpkt and notify the upperhalf.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_try_finish_tx_packet(FAR struct oa_tc6_driver_s *priv)
{
  if (priv->tx_pkt && (priv->tx_pkt_idx == priv->tx_pkt_len))
    {
      oa_tc6_release_tx_packet(priv);
      netdev_lower_txdone(&priv->dev);
    }
}

/****************************************************************************
 * Name: oa_tc6_handle_rx_chunk
 *
 * Description:
 *   Parse the received footer, update buffer status and handle data
 *   in the rxbuf.
 *
 * Input Parameters:
 *   priv   - pointer to the driver-specific state structure
 *   footer - the received footer
 *   rxbuf  - pointer to the received data buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_handle_rx_chunk(FAR struct oa_tc6_driver_s *priv,
                                   uint32_t footer, FAR uint8_t *rxbuf)
{
  int rxlen;
  int newlen;

  /* Update buffer status */

  priv->txc = oa_tc6_tx_credits(footer);
  priv->rca = oa_tc6_rx_available(footer);

  /* Check rx_pkt && !rx_pkt_ready,
   * oa_tc6_data_valid flag might have been set due to an SPI error
   */

  if (oa_tc6_data_valid(footer) && priv->rx_pkt && !priv->rx_pkt_ready)
    {
      if (oa_tc6_start_valid(footer))
        {
          /* When the end chunk is lost, this will save the upcoming frame */

          priv->rx_pkt_idx = 0;
        }
      else if (priv->rx_pkt_idx == 0)
        {
          /* Skip to the start of the next frame */

          return;
        }

      if (oa_tc6_end_valid(footer))
        {
          if (oa_tc6_frame_drop(footer))
            {
              nwarn("Warning: Dropping frame (FD)\n");
              oa_tc6_release_rx_packet(priv);
              return;
            }

          if (priv->rx_pkt_idx > OA_TC6_MAX_FRAME_SIZE(priv))
            {
              nwarn("Warning: Dropping frame (too long)\n");
              oa_tc6_release_rx_packet(priv);
              return;
            }

          rxlen = oa_tc6_end_byte_offset(footer) + 1;
        }
      else
        {
          rxlen = priv->config->chunk_payload_size;
        }

      newlen = priv->rx_pkt_idx + rxlen;

      if (newlen > OA_TC6_MAX_FRAME_SIZE(priv))
        {
          nwarn("Dropping chunk of a packet that is too long");

          /* Set index so that a subsequent chunk with
           * smaller payload won't pass
           */

          priv->rx_pkt_idx = OA_TC6_MAX_FRAME_SIZE(priv) + 1;
          return;
        }

      netpkt_copyin(&priv->dev, priv->rx_pkt, rxbuf,
                    rxlen, priv->rx_pkt_idx);
      priv->rx_pkt_idx = newlen;

      if (oa_tc6_end_valid(footer))
        {
          oa_tc6_finalize_rx_packet(priv);
        }
    }
}

/****************************************************************************
 * Name: oa_tc6_finalize_rx_packet
 *
 * Description:
 *   Strip down last 4 bytes (FCS) from the rx packet, mark it ready
 *   and notify upper.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_finalize_rx_packet(FAR struct oa_tc6_driver_s *priv)
{
  netpkt_setdatalen(&priv->dev, priv->rx_pkt,
                    netpkt_getdatalen(&priv->dev, priv->rx_pkt) - 4);
  priv->rx_pkt_ready = true;
  netdev_lower_rxready(&priv->dev);
}

/****************************************************************************
 * Name: oa_tc6_release_tx_packet
 *
 * Description:
 *   Release the tx packet.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_release_tx_packet(FAR struct oa_tc6_driver_s *priv)
{
  netpkt_free(&priv->dev, priv->tx_pkt, NETPKT_TX);
  priv->tx_pkt = NULL;
}

/****************************************************************************
 * Name: oa_tc6_release_rx_packet
 *
 * Description:
 *   Release the rx packet.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_release_rx_packet(FAR struct oa_tc6_driver_s *priv)
{
  netpkt_free(&priv->dev, priv->rx_pkt, NETPKT_RX);
  priv->rx_pkt = NULL;
}

/****************************************************************************
 * Name: oa_tc6_(select/deselect)_spi
 *
 * Description:
 *   Helper functions to setup SPI hardware.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_select_spi_raw(FAR struct spi_dev_s *spi,
                                  FAR const struct oa_tc6_config_s *config)
{
  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, OA_TC6_SPI_MODE);
  SPI_SETBITS(spi, OA_TC6_SPI_NBITS);
  SPI_HWFEATURES(spi, 0);  /* disable HW features */
  SPI_SETFREQUENCY(spi, config->frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
  SPI_SETDELAY(spi, 0, 0, 0, 0);
#endif

  SPI_SELECT(spi, config->id, true);
}

static void oa_tc6_deselect_spi_raw(FAR struct spi_dev_s *spi,
                                    FAR const struct oa_tc6_config_s *config)
{
  SPI_SELECT(spi, config->id, false);

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: oa_tc6_read_reg_raw
 *
 * Description:
 *   Read a MAC-PHY register without the need for the whole oa_tc6_driver_s
 *   structure.
 *
 * Input Parameters:
 *   spi    - pointer to the spi device instance
 *   config - pointer to the MAC-PHY configuration structure
 *
 * Returned Value:
 *   On successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_read_reg_raw(FAR struct spi_dev_s *spi,
                               FAR const struct oa_tc6_config_s *config,
                               oa_tc6_regid_t regid, FAR uint32_t *word)
{
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_TC6_REGID_GET_MMS(regid);
  uint16_t addr = OA_TC6_REGID_GET_ADDR(regid);
  int parity;
  uint32_t header;

  /* Prepare header */

  header =   (mms  << OA_TC6_MMS_POS)
           | (addr << OA_TC6_ADDR_POS);
  parity = oa_tc6_get_parity(header);
  header |= parity ? 0 : OA_TC6_P_MASK;  /* Make header odd parity */

  /* Convert to big endian */

  header = htobe32(header);

  /* Prepare exchange */

  txdata[0] = header;

  oa_tc6_select_spi_raw(spi, config);
  SPI_EXCHANGE(spi, txdata, rxdata, 12);
  oa_tc6_deselect_spi_raw(spi, config);

  *word = be32toh(rxdata[2]);
  if (rxdata[1] != header)
    {
      nerr("Error: Reading register failed, MMS: %d, ADDR: 0x%x\n",
           mms, addr);
      return ERROR;
    }

  ninfo("Info: Reading register OK, MMS: %d, ADDR: 0x%x\n", mms, addr);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_get_phyid
 *
 * Description:
 *   Read the device type from the PHYID register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   phyid - pointer to the destination of the PHYID value
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_get_phyid(FAR struct spi_dev_s *spi,
                            FAR const struct oa_tc6_config_s *config,
                            FAR uint32_t *phyid)
{
  return oa_tc6_read_reg_raw(spi, config, OA_TC6_PHYID_REGID, phyid);
}

/****************************************************************************
 * Name: oa_tc6_init_by_id
 *
 * Description:
 *   Initialize OA-TC6 device driver based on the given PHYID
 *
 * Input Parameters:
 *   spi    - reference to the SPI driver state data
 *   config - reference to the predefined configuration of the driver
 *   phyid  - identification of the OA-TC6 MAC-PHY chip
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_init_by_id(FAR struct spi_dev_s *spi,
                             FAR const struct oa_tc6_config_s *config,
                             uint32_t phyid)
{
  switch (phyid)
    {
#ifdef CONFIG_NET_OA_TC6_NCV7410
      case OA_TC6_NCV7410_PHYID:
          ninfo("Info: Detected NCV7410 or NCN26010\n");
          return ncv7410_initialize(spi, config);
#endif
#ifdef CONFIG_NET_OA_TC6_LAN865X
      case OA_TC6_LAN865X_PHYID:
          ninfo("Info: Detected LAN865x\n");
          return lan865x_initialize(spi, config);
#endif
      default:
          nerr("Error: Unknown PHYID 0x%08lX. "
               "Is the support enabled in Kconfig? "
               "Does the revision match?\n", phyid);
          return -EINVAL;
    }
}

/****************************************************************************
 * Name: oa_tc6_reset
 *
 * Description:
 *   Perform SW reset of the MAC-PHY.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On a successful reset OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_reset(FAR struct oa_tc6_driver_s *priv)
{
  int tries = OA_TC6_N_TRIES;
  uint32_t regval = (1 << OA_TC6_RESET_SWRESET_POS);

  if (oa_tc6_write_reg(priv, OA_TC6_RESET_REGID, regval))
    {
      return ERROR;
    }

  /* Check whether the RESET bit cleared itself */

  do
    {
      if (oa_tc6_read_reg(priv, OA_TC6_RESET_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (--tries && (regval & OA_TC6_RESET_SWRESET_MASK));

  if (regval & OA_TC6_RESET_SWRESET_MASK)
    {
      return ERROR;
    }

  /* Check whether the reset complete flag is set */

  tries = OA_TC6_N_TRIES;

  do
    {
      if (oa_tc6_read_reg(priv, OA_TC6_STATUS0_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (--tries && !(regval & OA_TC6_STATUS0_RESETC_MASK));

  if (!(regval & OA_TC6_STATUS0_RESETC_MASK))
    {
      return ERROR;
    }

  /* Clear reset complete flag */

  if (oa_tc6_write_reg(priv, OA_TC6_STATUS0_REGID,
                       1 << OA_TC6_STATUS0_RESETC_POS))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_config
 *
 * Description:
 *   Configure the MAC-PHY into promiscuous mode and set the SYNC flag.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 * Assumptions:
 *   The function is called after the MAC address is initialized.
 *
 ****************************************************************************/

static int oa_tc6_config(FAR struct oa_tc6_driver_s *priv)
{
  uint32_t regval;
  uint8_t chunk_payload_size = priv->config->chunk_payload_size;
  uint8_t cps = 0;

  ninfo("Info: Configuring OA-TC6\n");

  /* Call the MAC-PHY type specific config hook */

  if (priv->ops->action(priv, OA_TC6_ACTION_CONFIG))
    {
      nerr("Error: Device-specific config hook failed\n");
      return ERROR;
    }

  /* Add the MAC address to the address filter */

  if (priv->ops->addmac(priv, priv->dev.netdev.d_mac.ether.ether_addr_octet))
    {
      nerr("Error: Setting the address filter failed\n");
      return ERROR;
    }

  /* Setup SPI protocol and set SYNC flag */

  regval =   (1 << OA_TC6_CONFIG0_SYNC_POS)
           | (1 << OA_TC6_CONFIG0_CSARFE_POS)
           | (1 << OA_TC6_CONFIG0_ZARFE_POS)
           | (3 << OA_TC6_CONFIG0_TXCTHRESH_POS);

  if (priv->config->rx_cut_through)
    {
      regval |= 1 << OA_TC6_CONFIG0_RXCTE_POS; /* Enable RX cut-through */
    }

  /* Calculate and set CPS: 2^(CPS) = chunk_payload_size */

  while (chunk_payload_size != 1)
    {
      chunk_payload_size >>= 1;
      cps += 1;
    }

  regval |= (cps << OA_TC6_CONFIG0_CPS_POS);

  if (oa_tc6_write_reg(priv, OA_TC6_CONFIG0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_enable
 *
 * Description:
 *   Enable TX and RX on the MAC-PHY.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_enable(FAR struct oa_tc6_driver_s *priv)
{
  ninfo("Info: Enabling OA-TC6\n");

  /* Call device-specific code */

  if (priv->ops->action(priv, OA_TC6_ACTION_ENABLE))
    {
      nerr("Error: Enable on the device-specific level failed\n");
      return ERROR;
    }

  /* Enable interrupt on the board level */

  priv->config->enable(priv->config, true);

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_disable
 *
 * Description:
 *   Disable TX and RX on the MAC-PHY.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_disable(FAR struct oa_tc6_driver_s *priv)
{
  /* Disable interrupt on the board level */

  priv->config->enable(priv->config, false);

  ninfo("Info: Disabling OA-TC6\n");

  /* Call device-specific code */

  if (priv->ops->action(priv, OA_TC6_ACTION_DISABLE))
    {
      nerr("Error: Disable on the device-specific level failed\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_update_mac_filter
 *
 * Description:
 *   To be called during the ifup procedure. Checks whether the address
 *   saved in the base net_driver_s structure differs from the one saved in
 *   the mac_addr field of priv. If yes, the rmmac and addmac calls are
 *   issued in order to update the MAC address filter in the MAC-PHY
 *   to match the one in the base structure.
 *   The difference in addresses may be caused by performing
 *   the SIOCSIFHWADDR ioctl call.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_update_mac_filter(FAR struct oa_tc6_driver_s *priv)
{
  if (!memcmp(&priv->mac_addr, &priv->dev.netdev.d_mac.ether, IFHWADDRLEN))
    {
      return OK;
    }

  ninfo("Info: A new MAC address is present, updating\n");

  if (priv->ops->rmmac(priv, priv->mac_addr))
    {
      return ERROR;
    }

  if (priv->ops->addmac(priv, priv->dev.netdev.d_mac.ether.ether_addr_octet))
    {
      return ERROR;
    }

  memcpy(priv->mac_addr, &priv->dev.netdev.d_mac.ether, IFHWADDRLEN);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_reset_driver_buffers
 *
 * Description:
 *   If allocated, release both tx and rx netpackets and reset buffer status
 *   to the default.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_tc6_reset_driver_buffers(FAR struct oa_tc6_driver_s *priv)
{
  priv->txc = 0;
  priv->rca = 0;

  if (priv->tx_pkt)
    {
      oa_tc6_release_tx_packet(priv);
    }

  if (priv->rx_pkt)
    {
      oa_tc6_release_rx_packet(priv);
    }

  priv->tx_pkt_idx = 0;
  priv->rx_pkt_idx = 0;
  priv->tx_pkt_len = 0;
  priv->rx_pkt_ready = false;
}

/****************************************************************************
 * Name: oa_tc6_miireg_get_regid
 *
 * Description:
 *   Decode the phy_id field in the mii_ioctl_data_s structure and store
 *   the corresponding regid to destination referenced by the regid argument
 *
 * Input Parameters:
 *   req   - reference to the mii_ioctl_data_s request data
 *   regid - reference to the destination of the decoded regid
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_miireg_get_regid(FAR struct mii_ioctl_data_s *req,
                                   FAR oa_tc6_regid_t *regid)
{
  if (mdio_phy_id_is_c45(req->phy_id))
    {
      uint16_t prtad = mdio_phy_id_prtad(req->phy_id);
      uint8_t  mms;
      uint16_t base_addr;

      if (oa_tc6_get_mmd_base(prtad, &mms, &base_addr))
        {
          return ERROR;
        }

      *regid = OA_TC6_MAKE_REGID(mms, base_addr + req->reg_num);
    }
  else if (req->phy_id < 32)
    {
      *regid =
        OA_TC6_MAKE_REGID(OA_TC6_MII_BASE_MMS,
                          OA_TC6_MII_BASE_ADDR + (req->reg_num & 0x1f));
    }
  else
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_get_mmd_base
 *
 * Description:
 *   Get the MMS and the base address of the specified MMD within the
 *   MAC-PHY memory.
 *
 * Input Parameters:
 *   mmd  - the MMD to be given base to
 *   mms  - pointer to the destination of the MMS of the given MMD
 *   addr - pointer to the destination of the MMD's base address in the MMS
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_tc6_get_mmd_base(uint8_t mmd,
                               FAR uint8_t *mms, FAR uint16_t *addr)
{
  switch (mmd)
    {
      case 1:
          *mms  = OA_TC6_MMD_1_BASE_MMS;
          *addr = OA_TC6_MMD_1_BASE_ADDR;
          return OK;
      case 3:
          *mms  = OA_TC6_MMD_3_BASE_MMS;
          *addr = OA_TC6_MMD_3_BASE_ADDR;
          return OK;
      case 7:
          *mms  = OA_TC6_MMD_7_BASE_MMS;
          *addr = OA_TC6_MMD_7_BASE_ADDR;
          return OK;
      case 13:
          *mms  = OA_TC6_MMD_13_BASE_MMS;
          *addr = OA_TC6_MMD_13_BASE_ADDR;
          return OK;
      case 29:
          *mms  = OA_TC6_MMD_29_BASE_MMS;
          *addr = OA_TC6_MMD_29_BASE_ADDR;
          return OK;
      case 31:
          *mms  = OA_TC6_MMD_31_BASE_MMS;
          *addr = OA_TC6_MMD_31_BASE_ADDR;
          return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Netdev upperhalf callbacks
 ****************************************************************************/

/****************************************************************************
 * Name: oa_tc6_ifup
 *
 * Description:
 *   NuttX callback: Bring up the Ethernet interface
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;

  int err = nxmutex_lock(&priv->lock);
  if (err)
    {
      nerr("Error: Failed to acquire lock\n");
      return err;
    }

  if (priv->ifstate != OA_TC6_IFSTATE_RESET &&
      priv->ifstate != OA_TC6_IFSTATE_DOWN  &&
      priv->ifstate != OA_TC6_IFSTATE_DOWN_UNKNOWN)
    {
      nerr("Error: Tried to bring OA-TC6 interface up when not down\n");
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  ninfo("Info: Bringing up OA-TC6\n");

  if (priv->ifstate == OA_TC6_IFSTATE_DOWN_UNKNOWN)
    {
      if (oa_tc6_reset(priv))
        {
          nerr("Error: Reset of the OA-TC6 failed\n");
          nxmutex_unlock(&priv->lock);
          return -EIO;
        }

      priv->ifstate = OA_TC6_IFSTATE_RESET;
    }

  if (priv->ifstate == OA_TC6_IFSTATE_RESET)
    {
      if (oa_tc6_config(priv))
        {
          nerr("Error: Config of the OA-TC6 failed\n");
          nxmutex_unlock(&priv->lock);
          return -EIO;
        }

      priv->ifstate = OA_TC6_IFSTATE_DOWN;
    }

  if (oa_tc6_update_mac_filter(priv))
    {
      nerr("Error: Error during MAC address filter update\n");
      nxmutex_unlock(&priv->lock);
      return -EIO;
    }

  if (oa_tc6_enable(priv))
    {
      nerr("Error: Enabling of the OA-TC6 interface failed\n");
      nxmutex_unlock(&priv->lock);
      return -EIO;
    }

  priv->ifstate = OA_TC6_IFSTATE_UP;

  /* Schedule interrupt work to initialize txc and rca */

  work_queue(OA_TC6_WORK, &priv->interrupt_work,
             oa_tc6_interrupt_work, priv, 0);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_ifdown
 *
 * Description:
 *   NuttX callback: Shut down the Ethernet interface.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;

  int err = nxmutex_lock(&priv->lock);
  if (err)
    {
      nerr("Error: Failed to acquire lock\n");
      return err;
    }

  if (priv->ifstate != OA_TC6_IFSTATE_UP          &&
      priv->ifstate != OA_TC6_IFSTATE_UP_RECOVERY &&
      priv->ifstate != OA_TC6_IFSTATE_UP_UNKNOWN)
    {
      nerr("Error: Tried to bring the OA-TC6 interface down when not up\n");
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  work_cancel(OA_TC6_WORK, &priv->interrupt_work);
  work_cancel(OA_TC6_WORK, &priv->io_work);

  /* Allow disable to fail in recovery or unknown */

  if (oa_tc6_disable(priv) && priv->ifstate == OA_TC6_IFSTATE_UP)
    {
      nerr("Error: Disabling the OA-TC6 interface failed\n");
      nxmutex_unlock(&priv->lock);
      return -EIO;
    }

  oa_tc6_reset_driver_buffers(priv);

  if (priv->ifstate == OA_TC6_IFSTATE_UP_UNKNOWN)
    {
      priv->ifstate = OA_TC6_IFSTATE_DOWN_UNKNOWN;
    }
  else
    {
      priv->ifstate = OA_TC6_IFSTATE_DOWN;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_transmit
 *
 * Description:
 *   NuttX callback: Transmit the given packet.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *   pkt - network packet to be transmitted
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_transmit(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;

  int err = nxmutex_lock(&priv->lock);
  if (err)
    {
      nerr("Error: Failed to acquire lock\n");
      return err;
    }

  if (priv->tx_pkt || priv->ifstate != OA_TC6_IFSTATE_UP)
    {
      /* Previous TX packet was not yet sent to the network
       * or the interface has been shut down while waiting for the lock
       */

      nxmutex_unlock(&priv->lock);
      return -EAGAIN;
    }

  priv->tx_pkt_idx = 0;
  priv->tx_pkt_len = netpkt_getdatalen(dev, pkt);
  priv->tx_pkt = pkt;

  work_queue(OA_TC6_WORK, &priv->io_work, oa_tc6_io_work, priv, 0);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_receive
 *
 * Description:
 *   NuttX callback: Claims an RX packet if available.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   If the RX packet is ready, its pointer is returned.
 *   NULL is returned otherwise.
 *
 ****************************************************************************/

static FAR netpkt_t *oa_tc6_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;

  if (nxmutex_lock(&priv->lock))
    {
      nerr("Error: Failed to acquire lock\n");
      return NULL;
    }

  if (priv->rx_pkt_ready)
    {
      FAR netpkt_t *retval = priv->rx_pkt;

      ninfo("Info: Received RX packet %d bytes long\n",
            netpkt_getdatalen(&priv->dev, priv->rx_pkt));

      priv->rx_pkt_ready = false;
      priv->rx_pkt = NULL;

      nxmutex_unlock(&priv->lock);
      return retval;
    }

  nxmutex_unlock(&priv->lock);
  return NULL;
}

#ifdef CONFIG_NET_MCASTGROUP
/****************************************************************************
 * Name: oa_tc6_addmac
 *
 * Description:
 *   NuttX callback: Add multicast MAC address to the HW address filter.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_addmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;

  if (priv->ops->addmac)
    {
      return priv->ops->addmac(priv, mac);
    }

  return -ENOSYS;
}

/****************************************************************************
 * Name: oa_tc6_rmmac
 *
 * Description:
 *   NuttX callback: Remove multicast MAC address from the HW address filter.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_rmmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;

  if (priv->ops->rmmac)
    {
      return priv->ops->rmmac(priv, mac);
    }

  return -ENOSYS;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Name: oa_tc6_ioctl
 *
 * Description:
 *   NuttX callback: Remove multicast MAC address from the HW address filter.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_tc6_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct oa_tc6_driver_s *priv = (FAR struct oa_tc6_driver_s *)dev;
  int retval = 0;

  if (priv->ops->ioctl)
    {
      retval = priv->ops->ioctl(priv, cmd, arg);
    }

  /* This mechanism allows device-specific drivers to provide implementation
   * of ioctl commands and possibly override the following generic ones
   */

  if (retval != OA_TC6_IOCTL_CMD_NOT_IMPLEMENTED)
    {
      return retval;
    }

  /* If the device-specific command is not implemented, try if the OA generic
   * ioctl is present
   */

  switch (cmd)
    {
      case SIOCGMIIREG:
      case SIOCSMIIREG:
        {
          struct mii_ioctl_data_s *req =
              (struct mii_ioctl_data_s *)((uintptr_t)arg);
          oa_tc6_regid_t regid;
          uint32_t regval;

          if (oa_tc6_miireg_get_regid(req, &regid))
            {
              return -EINVAL;
            }

          if (cmd == SIOCGMIIREG)
            {
              if (oa_tc6_read_reg(priv, regid, &regval))
                {
                  return -EIO;
                }

              req->val_out = (uint16_t)regval;
            }
          else
            {
              regval = req->val_in;
              if (oa_tc6_write_reg(priv, regid, regval))
                {
                  return -EIO;
                }
            }

          return OK;
        }
    }

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oa_tc6_write_reg
 *
 * Description:
 *   Write to a MAC-PHY register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   regid - Register id encapsulating MMS and ADDR
 *   word  - 32-bit word to be written to the register
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_tc6_write_reg(FAR struct oa_tc6_driver_s *priv,
                     oa_tc6_regid_t regid, uint32_t word)
{
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_TC6_REGID_GET_MMS(regid);
  uint16_t addr = OA_TC6_REGID_GET_ADDR(regid);

  /* Prepare header */

  uint32_t header =   (1    << OA_TC6_WNR_POS)   /* Write Not Read */
                    | (mms  << OA_TC6_MMS_POS)
                    | (addr << OA_TC6_ADDR_POS);
  int parity = oa_tc6_get_parity(header);
  header |= parity ? 0 : OA_TC6_P_MASK;  /* Make header odd parity */

  /* Convert to big endian */

  header = htobe32(header);
  word = htobe32(word);

  /* Prepare exchange */

  txdata[0] = header;
  txdata[1] = word;

  oa_tc6_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  oa_tc6_deselect_spi(priv);
  if (rxdata[1] != header)
    {
      nerr("Error: Writing register failed, MMS: %d, ADDR: 0x%x\n",
           mms, addr);
      return ERROR;
    }

  ninfo("Info: Writing register OK, MMS: %d, ADDR: 0x%x\n", mms, addr);
  return OK;
}

/****************************************************************************
 * Name: oa_tc6_read_reg
 *
 * Description:
 *   Read a MAC-PHY register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   regid - register id encapsulating MMS and ADDR
 *   word  - pointer to a 32-bit destination variable
 *
 * Returned Value:
 *   On successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_tc6_read_reg(FAR struct oa_tc6_driver_s *priv,
                    oa_tc6_regid_t regid, FAR uint32_t *word)
{
  return oa_tc6_read_reg_raw(priv->spi, priv->config, regid, word);
}

/****************************************************************************
 * Name: oa_tc6_set_clear_bits
 *
 * Description:
 *   Perform a read-modify-write operation on a given register
 *   while setting bits from the setbits argument and clearing bits from
 *   the clearbits argument.
 *
 * Input Parameters:
 *   priv      - pointer to the driver-specific state structure
 *   regid     - register id of the register to be modified
 *   setbits   - bits set to one will be set in the register
 *   clearbits - bits set to one will be cleared in the register
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_tc6_set_clear_bits(FAR struct oa_tc6_driver_s *priv,
                          oa_tc6_regid_t regid,
                          uint32_t setbits, uint32_t clearbits)
{
  uint32_t regval;

  if (oa_tc6_read_reg(priv, regid, &regval))
    {
      return ERROR;
    }

  regval |= setbits;
  regval &= ~clearbits;

  if (oa_tc6_write_reg(priv, regid, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_store_mac_addr
 *
 * Description:
 *   Store the given MAC address into the net driver structure.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *   mac  - pointer to an array containing the MAC address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void oa_tc6_store_mac_addr(FAR struct oa_tc6_driver_s *priv,
                           FAR const uint8_t *mac)
{
  memcpy(&priv->dev.netdev.d_mac.ether.ether_addr_octet, mac, IFHWADDRLEN);
  memcpy(&priv->mac_addr, mac, IFHWADDRLEN);
}

/****************************************************************************
 * Name: oa_tc6_bitrev8
 *
 * Description:
 *   Perform a bit reverse of a byte.
 *
 * Input Parameters:
 *   byte - byte to be reversed
 *
 * Returned Value:
 *   Byte with reversed bits is returned.
 *
 ****************************************************************************/

uint8_t oa_tc6_bitrev8(uint8_t byte)
{
  /* https://stackoverflow.com/a/2602885 */

  byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
  byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
  byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
  return byte;
}

/****************************************************************************
 * Name: oa_tc6_initialize
 *
 * Description:
 *   Read the PHYID of the MAC-PHY device and initialize the matching
 *   driver.
 *
 * Input Parameters:
 *   spi    - pointer to the initialized SPI interface
 *   config - pointer to the initialized MAC-PHY configuration
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int oa_tc6_initialize(FAR struct spi_dev_s *spi,
                      FAR const struct oa_tc6_config_s *config)
{
  uint32_t phyid;

  /* Get device type from MAC-PHY OA-TC6 common registers */

  if (oa_tc6_get_phyid(spi, config, &phyid))
    {
      nerr("Error: Reading of the PHYID failed\n");
      return -EIO;
    }

  return oa_tc6_init_by_id(spi, config, phyid);
}

/****************************************************************************
 * Name: oa_tc6_common_init
 *
 * Description:
 *   Initialize the upper-half part of the device structure and reset
 *   the MAC-PHY.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   spi    - reference to the SPI driver state data
 *   config - reference to the predefined configuration of the driver
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int oa_tc6_common_init(FAR struct oa_tc6_driver_s *priv,
                       FAR struct spi_dev_s *spi,
                       FAR const struct oa_tc6_config_s *config)
{
  FAR struct netdev_lowerhalf_s *netdev;
  uint32_t stdcap;
  uint8_t mincps; /* Minimum supported chunk payload size */
  bool ctc;       /* Cut-through capability */

  /* Validate spi and config */

  DEBUGASSERT(spi);
  DEBUGASSERT(config);

  DEBUGASSERT((config->chunk_payload_size == 8)  ||
              (config->chunk_payload_size == 16) ||
              (config->chunk_payload_size == 32) ||
              (config->chunk_payload_size == 64));

  priv->spi = spi;       /* Save the reference to the SPI instance  */
  priv->config = config; /* Save the reference to the configuration */

  if (oa_tc6_read_reg(priv, OA_TC6_STDCAP_REGID, &stdcap))
    {
      nerr("Error: Reading STDCAP register failed\n");
      return -EIO;
    }

  mincps = 1 << oa_tc6_get_field(stdcap, STDCAP_MINCPS);
  ctc = oa_tc6_get_field(stdcap, STDCAP_CTC);
  DEBUGASSERT(mincps <= config->chunk_payload_size);
  DEBUGASSERT((config->rx_cut_through == false) || ctc);

  /* Check if the MDIO access is supported */

  if (!(stdcap & OA_TC6_STDCAP_DPRAC_MASK))
    {
      nwarn("Warning: Direct PHY register access is not supported "
            "by the MAC-PHY, SIOCxMMDREG ioctls won't work\n");
    }

  /* Check for mandatory callbacks */

  DEBUGASSERT(priv->ops);
  DEBUGASSERT(priv->ops->action);
  DEBUGASSERT(priv->config->attach);
  DEBUGASSERT(priv->config->enable);

  /* Attach ISR */

  if (priv->config->attach(priv->config, oa_tc6_interrupt, priv))
    {
      nerr("Error: Attaching ISR failed\n");
      return -EINVAL;
    }

  /* Init lock */

  nxmutex_init(&priv->lock);

  /* Fill the fields for the netdev upperhalf driver */

  netdev = &priv->dev;
  netdev->quota[NETPKT_TX] = OA_TC6_TX_QUOTA;
  netdev->quota[NETPKT_RX] = OA_TC6_RX_QUOTA;
  netdev->ops = &g_oa_tc6_ops;

  /* Allocate SPI buffers based on the config */

  priv->txbuf = kmm_malloc(OA_TC6_CHUNK_SIZE(priv));
  priv->rxbuf = kmm_malloc(OA_TC6_CHUNK_SIZE(priv));
  if ((priv->txbuf == NULL) || (priv->rxbuf == NULL))
    {
      if (priv->txbuf)
        {
          kmm_free(priv->txbuf);
        }

      if (priv->rxbuf)
        {
          kmm_free(priv->rxbuf);
        }

      nerr("Error: Could not allocate memory for SPI buffers\n");
      return -ENOMEM;
    }

  /* Reset the MAC-PHY */

  if (oa_tc6_reset(priv))
    {
      nerr("Error: Resetting OA-TC6 device failed\n");
      kmm_free(priv->txbuf);
      kmm_free(priv->rxbuf);
      return -EIO;
    }

  priv->ifstate = OA_TC6_IFSTATE_RESET;

  return OK;
}

/****************************************************************************
 * Name: oa_tc6_register
 *
 * Description:
 *   Register the OA-TC6 lower-half driver.
 *
 * Input Parameters:
 *   oa_tc6_dev - reference to the initialized oa_tc6_driver_s structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int oa_tc6_register(FAR struct oa_tc6_driver_s *oa_tc6_dev)
{
  return netdev_lower_register(&oa_tc6_dev->dev, NET_LL_ETHERNET);
}
