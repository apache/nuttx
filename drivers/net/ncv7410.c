/****************************************************************************
 * drivers/net/ncv7410.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <sys/endian.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/net/ncv7410.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "ncv7410.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NCVWORK LPWORK

#define NCV_RESET_TRIES 5

/* Maximum frame size = (MTU + LL heaader size) + FCS size */

#define NCV_MAX_FRAME_SIZE(p) (p->dev.netdev.d_pktsize + 4)

/* Packet Memory ************************************************************/

/* Maximum number of allocated tx and rx packets */

#define NCV7410_TX_QUOTA        1
#define NCV7410_RX_QUOTA        2

#if CONFIG_IOB_NBUFFERS < (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)
#  error "CONFIG_IOB_NBUFFERS must be > (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)"
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error "CONFIG_SCHED_LPWORK is needed by NCV7410 driver"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The ncv7410_driver_s encapsulates all state information for a single
 * hardware interface
 */

enum ncv_ifstate_e
{
  NCV_RESET,
  NCV_INIT_DOWN,
  NCV_INIT_UP
};

struct ncv7410_driver_s
{
  struct netdev_lowerhalf_s dev;   /* Driver data visible by the net stack
                                    * (must be placed first)                */
  mutex_t lock;                    /* Lock for data race prevention         */
  FAR struct spi_dev_s *spi;       /* The SPI device instance               */
  int irqnum;                      /* irq number of the interrupt pin       */
  struct ncv7410_config_s *config; /* NCV7410 configuration                 */
  uint8_t ifstate;                 /* Driver state from ncv_ifstate_e enum  */

  struct work_s interrupt_work;    /* wq handle for the interrupt work      */
  struct work_s io_work;           /* wq handle for the io work             */

  int txc;                         /* TX credits                            */
  int rca;                         /* RX chunks available                   */

  FAR netpkt_t *tx_pkt;            /* Pointer to the TX netpacket           */
  FAR netpkt_t *rx_pkt;            /* Pointer to the RX netpacket           */
  int tx_pkt_idx;                  /* Position in the TX netpacket          */
  int rx_pkt_idx;                  /* Position in the RX netpacket          */
  int tx_pkt_len;                  /* Length of the TX packet               */
  bool rx_pkt_ready;               /* RX packet ready to be received flag   */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Bit calculations */

static int ncv_get_parity(uint32_t word);
static uint8_t ncv_bitrev8(uint8_t byte);

/* SPI transfers */

static int ncv_write_reg(FAR struct ncv7410_driver_s *priv,
                         oa_regid_t regid, uint32_t word);

static int ncv_read_reg(FAR struct ncv7410_driver_s *priv,
                        oa_regid_t regid, FAR uint32_t *word);

static int ncv_set_clear_bits(FAR struct ncv7410_driver_s *priv,
                              oa_regid_t regid,
                              uint32_t setbits, uint32_t clearbits);

static int ncv_poll_footer(FAR struct ncv7410_driver_s *priv,
                           FAR uint32_t *footer);

static int ncv_exchange_chunk(FAR struct ncv7410_driver_s *priv,
                              FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                              uint32_t header, uint32_t *footer);

/* Interrupt handling */

static int ncv_interrupt(int irq, FAR void *context, FAR void *arg);
static void ncv_interrupt_work(FAR void *arg);

/* Data Transaction Protocol logic */

static void ncv_io_work(FAR void *arg);
static uint32_t ncv_prepare_chunk_exchange(FAR struct ncv7410_driver_s *priv,
                                           FAR uint8_t *txbuf);
static bool ncv_can_rx(FAR struct ncv7410_driver_s *priv);
static void ncv_try_finish_tx_packet(FAR struct ncv7410_driver_s *priv);
static void ncv_handle_rx_chunk(FAR struct ncv7410_driver_s *priv,
                                uint32_t footer, FAR uint8_t *rxbuf);
static void ncv_finalize_rx_packet(FAR struct ncv7410_driver_s *priv);
static void ncv_release_tx_packet(FAR struct ncv7410_driver_s *priv);
static void ncv_release_rx_packet(FAR struct ncv7410_driver_s *priv);

/* SPI inline utility functions */

static inline void ncv_select_spi(FAR struct ncv7410_driver_s *priv);
static inline void ncv_deselect_spi(FAR struct ncv7410_driver_s *priv);

/* ncv7410 reset and configuration */

static int ncv_reset(FAR struct ncv7410_driver_s *priv);
static int ncv_config(FAR struct ncv7410_driver_s *priv);
static int ncv_enable(FAR struct ncv7410_driver_s *priv);
static int ncv_disable(FAR struct ncv7410_driver_s *priv);
static int ncv_init_mac_addr(FAR struct ncv7410_driver_s *priv);

/* Driver buffer manipulation */

static void ncv_reset_driver_buffers(FAR struct ncv7410_driver_s *priv);

/* NuttX callback functions */

static int ncv7410_ifup(FAR struct netdev_lowerhalf_s *dev);
static int ncv7410_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int ncv7410_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt);
static FAR netpkt_t *ncv7410_receive(FAR struct netdev_lowerhalf_s *dev);

/* Debug */

#ifdef CONFIG_DEBUG_NET_INFO
static void ncv_print_footer(uint32_t footer);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_ncv7410_ops =
{
  .ifup     = ncv7410_ifup,
  .ifdown   = ncv7410_ifdown,
  .transmit = ncv7410_transmit,
  .receive  = ncv7410_receive,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv_interrupt
 *
 * Description:
 *   Schedule interrupt work when the interrupt signal from MAC-PHY is
 *   received.
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *   arg     - ncv7410_driver_s priv structure to be passed to the interrupt
 *             worker
 *
 * Returned Value:
 *   OK is always returned.
 *
 ****************************************************************************/

static int ncv_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)arg;

  ninfo("NCV7410 interrupt!\n");

  /* schedule interrupt work */

  work_queue(NCVWORK, &priv->interrupt_work, ncv_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: ncv_interrupt_work
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

static void ncv_interrupt_work(FAR void *arg)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)arg;
  uint32_t footer;

  nxmutex_lock(&priv->lock);

  if (priv->ifstate != NCV_INIT_UP)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  ninfo("NCV7410 interrupt worker invoked!\n");

  /* poll the data chunk footer */

  if (ncv_poll_footer(priv, &footer))
    {
      nerr("Polling footer unsuccessful\n");

      /* TODO: don't */

      PANIC();
    }

#ifdef CONFIG_DEBUG_NET_INFO
  ncv_print_footer(footer);
#endif

  /* if EXST in the footer, check enabled sources
   * STATUS0, link-status in clause 22 phy registers
   * (not yet implemented)
   */

  /* update MAC-PHY buffer status */

  priv->txc = oa_tx_credits(footer);
  priv->rca = oa_rx_available(footer);

  if ((priv->tx_pkt && priv->txc) || priv->rca)
    {
      /* schedule IO work */

      work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: ncv_io_work
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

static void ncv_io_work(FAR void *arg)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)arg;

  uint8_t txbuf[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE];
  uint8_t rxbuf[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE];

  uint32_t header;
  uint32_t footer;

  nxmutex_lock(&priv->lock);

  if (priv->ifstate != NCV_INIT_UP)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  header = ncv_prepare_chunk_exchange(priv, txbuf);

  /* Perform the SPI exchange */

  if (ncv_exchange_chunk(priv, txbuf, rxbuf, header, &footer))
    {
      nerr("Error during chunk exchange\n");

      /* TODO: do not panic, the best is probably to report the error
       * and reset MAC to some defined state and reset driver
       */

      PANIC();
    }

  ncv_try_finish_tx_packet(priv);

  ncv_handle_rx_chunk(priv, footer, rxbuf);

  /* schedule further work if needed */

  if ((priv->tx_pkt && priv->txc) || priv->rca)
    {
      work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: ncv_prepare_chunk_exchange
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

static uint32_t ncv_prepare_chunk_exchange(FAR struct ncv7410_driver_s *priv,
                                           FAR uint8_t *txbuf)
{
  uint32_t header = 0;
  int txlen;

  if (priv->tx_pkt && priv->txc)
    {
      header |= (1 << OA_DV_POS);  /* Data Valid */

      if (priv->tx_pkt_idx == 0)
        {
          header |=   (1 << OA_SV_POS)   /* Start Valid */
                    | (0 << OA_SWO_POS); /* Start Word Offset = 0 */
        }

      txlen = priv->tx_pkt_len - priv->tx_pkt_idx;

      if (txlen <= NCV_CHUNK_DEFAULT_PAYLOAD_SIZE)
        {
          header |=   (1 << OA_EV_POS)             /* End Valid */
                    | ((txlen - 1) << OA_EBO_POS); /* End Byte Offset */
        }
      else
        {
          txlen = NCV_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }

      /* copy data from network to txbuf */

      netpkt_copyout(&priv->dev, txbuf, priv->tx_pkt,
                     txlen, priv->tx_pkt_idx);
      priv->tx_pkt_idx += txlen;
    }

  if (ncv_can_rx(priv) == false)
    {
      header |= (1 << OA_NORX_POS);  /* no rx */
    }

  return header;
}

/****************************************************************************
 * Name: ncv_can_rx
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

static bool ncv_can_rx(FAR struct ncv7410_driver_s *priv)
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

  /* no RX packet, try to alloc */

  priv->rx_pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (priv->rx_pkt)
    {
      return true;
    }

  ninfo("INFO: Failed to alloc rx netpkt\n");

  /* there is no buffer for rx data */

  return false;
}

/****************************************************************************
 * Name: ncv_try_finish_tx_packet
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

static void ncv_try_finish_tx_packet(FAR struct ncv7410_driver_s *priv)
{
  if (priv->tx_pkt && (priv->tx_pkt_idx == priv->tx_pkt_len))
    {
      ncv_release_tx_packet(priv);
      netdev_lower_txdone(&priv->dev);
    }
}

/****************************************************************************
 * Name: ncv_handle_rx_chunk
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

static void ncv_handle_rx_chunk(FAR struct ncv7410_driver_s *priv,
                                uint32_t footer, FAR uint8_t *rxbuf)
{
  int rxlen;
  int newlen;

  /* update buffer status */

  priv->txc = oa_tx_credits(footer);
  priv->rca = oa_rx_available(footer);

  /* check rx_pkt && !rx_pkt_ready,
   * oa_data_valid flag might have been set due to an SPI error
   */

  if (oa_data_valid(footer) && priv->rx_pkt && !priv->rx_pkt_ready)
    {
      if (oa_start_valid(footer))
        {
          priv->rx_pkt_idx = 0;
        }

      if (oa_end_valid(footer))
        {
          if (oa_frame_drop(footer))
            {
              ncv_release_rx_packet(priv);
              return;
            }

          rxlen = oa_end_byte_offset(footer) + 1;
        }
      else
        {
          rxlen = NCV_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }

      newlen = priv->rx_pkt_idx + rxlen;

      if (newlen > NCV_MAX_FRAME_SIZE(priv))
        {
          nwarn("Dropping chunk of a packet that is too long");

          /* set index so that a subsequent chunk with
           * smaller payload won't pass
           */

          priv->rx_pkt_idx = NCV_MAX_FRAME_SIZE(priv) + 1;
          return;
        }

      netpkt_copyin(&priv->dev, priv->rx_pkt, rxbuf,
                    rxlen, priv->rx_pkt_idx);
      priv->rx_pkt_idx = newlen;

      if (oa_end_valid(footer))
        {
          /* finalize packet and notify the upper */

          ncv_finalize_rx_packet(priv);
          netdev_lower_rxready(&priv->dev);
        }
    }
}

/****************************************************************************
 * Name: ncv_finalize_rx_packet
 *
 * Description:
 *   Strip down last 4 bytes (FCS) from the rx packet and mark it ready.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ncv_finalize_rx_packet(FAR struct ncv7410_driver_s *priv)
{
  netpkt_setdatalen(&priv->dev, priv->rx_pkt,
                    netpkt_getdatalen(&priv->dev, priv->rx_pkt) - 4);
  priv->rx_pkt_ready = true;
}

/****************************************************************************
 * Name: ncv_release_tx_packet
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

static void ncv_release_tx_packet(FAR struct ncv7410_driver_s *priv)
{
  netpkt_free(&priv->dev, priv->tx_pkt, NETPKT_TX);
  priv->tx_pkt = NULL;
}

/****************************************************************************
 * Name: ncv_release_rx_packet
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

static void ncv_release_rx_packet(FAR struct ncv7410_driver_s *priv)
{
  netpkt_free(&priv->dev, priv->rx_pkt, NETPKT_RX);
  priv->rx_pkt = NULL;
}

/****************************************************************************
 * Name: ncv_get_parity
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

static int ncv_get_parity(uint32_t word)
{
  /* www-graphics.stanford.edu/~seander/bithacks.html */

  word ^= word >> 1;
  word ^= word >> 2;
  word = (word & 0x11111111u) * 0x11111111u;
  return (word >> 28) & 1;
}

/****************************************************************************
 * Name: ncv_bitrev8
 *
 * Description:
 *   Perform a bit reverse of a byte.
 *
 * Input Parameters:
 *   b - byte to be reversed
 *
 * Returned Value:
 *   Byte with reversed bits is returned.
 *
 ****************************************************************************/

static uint8_t ncv_bitrev8(uint8_t byte)
{
  /* https://stackoverflow.com/a/2602885 */

  byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
  byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
  byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
  return byte;
}

/****************************************************************************
 * Name: ncv_(select/deselect)_spi
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

static inline void ncv_select_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_LOCK(priv->spi, true);

  SPI_SETMODE(priv->spi, OA_SPI_MODE);
  SPI_SETBITS(priv->spi, OA_SPI_NBITS);
  SPI_HWFEATURES(priv->spi, 0);  /* disable HW features */
  SPI_SETFREQUENCY(priv->spi, CONFIG_NCV7410_FREQUENCY);

  SPI_SELECT(priv->spi, priv->config->id, true);
}

static inline void ncv_deselect_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_SELECT(priv->spi, priv->config->id, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: ncv_write_reg
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

static int ncv_write_reg(FAR struct ncv7410_driver_s *priv,
                         oa_regid_t regid, uint32_t word)
{
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);

  /* prepare header */

  uint32_t header =   (1    << OA_WNR_POS)   /* Write Not Read */
                    | (mms  << OA_MMS_POS)
                    | (addr << OA_ADDR_POS);
  int parity = ncv_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  /* make header odd parity */

  /* convert to big endian */

  header = htobe32(header);
  word = htobe32(word);

  /* prepare exchange */

  txdata[0] = header;
  txdata[1] = word;

  ncv_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  ncv_deselect_spi(priv);
  if (rxdata[1] != header)
    {
      nerr("Error writing register\n");
      return ERROR;
    }

  ninfo("Writing register OK\n");
  return OK;
}

/****************************************************************************
 * Name: ncv_read_reg
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

static int ncv_read_reg(FAR struct ncv7410_driver_s *priv,
                        oa_regid_t regid, FAR uint32_t *word)
{
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);
  int parity;
  uint32_t header;

  /* prepare header */

  header =   (mms  << OA_MMS_POS)
           | (addr << OA_ADDR_POS);
  parity = ncv_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  /* make header odd parity */

  /* convert to big endian */

  header = htobe32(header);

  /* prepare exchange */

  txdata[0] = header;

  ncv_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  ncv_deselect_spi(priv);

  *word = be32toh(rxdata[2]);
  if (rxdata[1] != header)
    {
      nerr("Error reading register\n");
      return ERROR;
    }

  ninfo("Reading register OK\n");
  return OK;
}

/****************************************************************************
 * Name: ncv_set_clear_bits
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

static int ncv_set_clear_bits(FAR struct ncv7410_driver_s *priv,
                              oa_regid_t regid,
                              uint32_t setbits, uint32_t clearbits)
{
  uint32_t regval;

  if (ncv_read_reg(priv, regid, &regval))
    {
      return ERROR;
    }

  regval |= setbits;
  regval &= ~clearbits;

  if (ncv_write_reg(priv, regid, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_exchange_chunk
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

static int ncv_exchange_chunk(FAR struct ncv7410_driver_s *priv,
                              FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                              uint32_t header, uint32_t *footer)
{
  header |= (1 << OA_DNC_POS);
  header |= (!ncv_get_parity(header) << OA_P_POS);
  header = htobe32(header);

  ncv_select_spi(priv);

  /* this depends on SW Chip Select */

  SPI_EXCHANGE(priv->spi, (uint8_t *) &header, rxbuf, 4);
  SPI_EXCHANGE(priv->spi, txbuf,
               &rxbuf[4], NCV_CHUNK_DEFAULT_PAYLOAD_SIZE - 4);
  SPI_EXCHANGE(priv->spi, &txbuf[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE - 4],
               (uint8_t *)footer, 4);
  ncv_deselect_spi(priv);

  *footer = be32toh(*footer);
  if (!ncv_get_parity(*footer))
    {
      nerr("Wrong parity in the footer\n");
      return ERROR;
    }

  if (oa_header_bad(*footer))
    {
      nerr("HDRB set in the footer\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_poll_footer
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

static int ncv_poll_footer(FAR struct ncv7410_driver_s *priv,
                           FAR uint32_t *footer)
{
  uint8_t txdata[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE];
  uint8_t rxdata[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE];
  uint32_t header;

  header =   (1 << OA_DNC_POS)   /* Data Not Control */
           | (1 << OA_NORX_POS); /* No Read */

  if (ncv_exchange_chunk(priv, txdata, rxdata, header, footer))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_reset
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

static int ncv_reset(FAR struct ncv7410_driver_s *priv)
{
  int tries = NCV_RESET_TRIES;
  uint32_t regval = (1 << OA_RESET_SWRESET_POS);

  if (ncv_write_reg(priv, OA_RESET_REGID, regval))
    {
      return ERROR;
    }

  /* check whether the RESET bit cleared itself */

  do
    {
      if (ncv_read_reg(priv, OA_RESET_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (tries-- && (regval & OA_RESET_SWRESET_MASK));

  if (regval & OA_RESET_SWRESET_MASK)
    {
      return ERROR;
    }

  /* check whether the reset complete flag is set */

  tries = NCV_RESET_TRIES;

  do
    {
      if (ncv_read_reg(priv, OA_STATUS0_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (tries-- && !(regval & OA_STATUS0_RESETC_MASK));

  if (!(regval & OA_STATUS0_RESETC_MASK))
    {
      return ERROR;
    }

  /* clear HDRE in STATUS0 (due to a bug in NCV7410) */

  if (ncv_write_reg(priv, OA_STATUS0_REGID, (1 << OA_STATUS0_HDRE_POS)))
    {
      return ERROR;
    }

  /* clear reset complete flag */

  if (ncv_write_reg(priv, OA_STATUS0_REGID, (1 << OA_STATUS0_RESETC_POS)))
    {
      return ERROR;
    }

  /* blink with LEDs for debugging purposes */

  for (int i = 0; i < 4; i++)
    {
      regval = 0x0302;
      if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
        {
          return ERROR;
        }

      nxsig_usleep(250000);
      regval = 0x0203;
      if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
        {
          return ERROR;
        }

      nxsig_usleep(250000);
    }

  /* set DIOs to default */

  regval = NCV_DIO_CONFIG_DEF;
  if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_config
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

static int ncv_config(FAR struct ncv7410_driver_s *priv)
{
  uint32_t regval;
#ifndef CONFIG_NET_PROMISCUOUS
  uint8_t *mac = priv->dev.netdev.d_mac.ether.ether_addr_octet;
#endif

  ninfo("Configuring NCV7410\n");

  /* setup LEDs DIO0: txrx blink
   *            DIO1: link enabled and link status up
   */

  regval =   (NCV_DIO_TXRX_FUNC << NCV_DIO0_FUNC_POS)
           | (NCV_DIO_LINK_CTRL_FUNC << NCV_DIO1_FUNC_POS)
           | (1 << NCV_DIO0_OUT_VAL_POS)
           | (1 << NCV_DIO1_OUT_VAL_POS);

  if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
    {
      return ERROR;
    }

  /* enable MAC TX, RX, enable transmit FCS computation on MAC,
   * enable MAC address filtering
   */

  regval =   (1 << NCV_MAC_CONTROL0_FCSA_POS)
           | (1 << NCV_MAC_CONTROL0_TXEN_POS)
           | (1 << NCV_MAC_CONTROL0_RXEN_POS)
           | (1 << NCV_MAC_CONTROL0_ADRF_POS);

#ifdef CONFIG_NET_PROMISCUOUS
  /* disable MAC address filtering */

  regval &= ~(1 << NCV_MAC_CONTROL0_ADRF_POS);
#endif

  if (ncv_write_reg(priv, NCV_MAC_CONTROL0_REGID, regval))
    {
      return ERROR;
    }

#ifndef CONFIG_NET_PROMISCUOUS
  /* setup MAC address filter */

  regval =   (mac[2] << 24)
           | (mac[3] << 16)
           | (mac[4] << 8)
           | (mac[5]);

  if (ncv_write_reg(priv, NCV_ADDRFILT0L_REGID, regval))
    {
      return ERROR;
    }

  regval =   (1 << 31)  /* enable filter */
           | (mac[0] << 8)
           | (mac[1]);

  if (ncv_write_reg(priv, NCV_ADDRFILT0H_REGID, regval))
    {
      return ERROR;
    }

  regval = 0xffffffff;

  if (ncv_write_reg(priv, NCV_ADDRMASK0L_REGID, regval))
    {
      return ERROR;
    }

  regval = 0x0000ffff;

  if (ncv_write_reg(priv, NCV_ADDRMASK0H_REGID, regval))
    {
      return ERROR;
    }

#endif

  /* enable rx buffer overflow interrupt */

  regval = OA_IMSK0_DEF & ~(1 << OA_IMSK0_RXBOEM_POS);

  if (ncv_write_reg(priv, OA_IMSK0_REGID, regval))
    {
      return ERROR;
    }

  /* setup SPI protocol and set SYNC flag */

  regval =   (1 << OA_CONFIG0_SYNC_POS)
           | (1 << OA_CONFIG0_CSARFE_POS)
           | (1 << OA_CONFIG0_ZARFE_POS)
           | (1 << OA_CONFIG0_RXCTE_POS)  /* a bit lower latency */
           | (3 << OA_CONFIG0_TXCTHRESH_POS)
           | (6 << OA_CONFIG0_CPS_POS);

  if (ncv_write_reg(priv, OA_CONFIG0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_enable
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

static int ncv_enable(FAR struct ncv7410_driver_s *priv)
{
  /* enable PHY */

  uint32_t setbits;

  ninfo("Enabling NCV7410\n");

  /* enable RX and TX in PHY */

  setbits = (1 << OA_PHY_CONTROL_LCTL_POS);

  if (ncv_set_clear_bits(priv, OA_PHY_CONTROL_REGID, setbits, 0))
    {
      return ERROR;
    }

  /* enable PHY interrupt */

  setbits = (1 << OA_IMSK0_PHYINTM_POS);

  if (ncv_set_clear_bits(priv, OA_IMSK0_REGID, setbits, 0))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_disable
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

static int ncv_disable(FAR struct ncv7410_driver_s *priv)
{
  /* disable PHY */

  uint32_t clearbits;

  ninfo("Disabling NCV7410\n");

  /* disable PHY interrupt */

  clearbits = (1 << OA_IMSK0_PHYINTM_POS);

  if (ncv_set_clear_bits(priv, OA_IMSK0_REGID, 0, clearbits))
    {
      return ERROR;
    }

  /* disable RX and TX in PHY */

  clearbits = (1 << OA_PHY_CONTROL_LCTL_POS);

  if (ncv_set_clear_bits(priv, OA_PHY_CONTROL_REGID, 0, clearbits))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_init_mac_addr
 *
 * Description:
 *   Read the MAC-PHY's factory-assigned MAC address and copy it into
 *   the network device state structure.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int ncv_init_mac_addr(FAR struct ncv7410_driver_s *priv)
{
  uint32_t regval;
  uint8_t  mac[6];

  if (ncv_read_reg(priv, OA_PHYID_REGID, &regval))
    {
      return ERROR;
    }

  mac[0] = ncv_bitrev8(regval >> 26);
  mac[1] = ncv_bitrev8(regval >> 18);
  mac[2] = ncv_bitrev8(regval >> 10);

  if (ncv_read_reg(priv, NCV_MACID1_REGID, &regval))
    {
      return ERROR;
    }

  mac[3] = regval;

  if (ncv_read_reg(priv, NCV_MACID0_REGID, &regval))
    {
      return ERROR;
    }

  mac[4] = regval >> 8;
  mac[5] = regval;

  memcpy(&priv->dev.netdev.d_mac.ether, &mac, sizeof(struct ether_addr));

  return OK;
}

/****************************************************************************
 * Name: ncv_reset_driver_buffers
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

static void ncv_reset_driver_buffers(FAR struct ncv7410_driver_s *priv)
{
  priv->txc = 0;
  priv->rca = 0;

  if (priv->tx_pkt)
    {
      ncv_release_tx_packet(priv);
    }

  if (priv->rx_pkt)
    {
      ncv_release_rx_packet(priv);
    }

  priv->tx_pkt_idx = 0;
  priv->rx_pkt_idx = 0;
  priv->tx_pkt_len = 0;
  priv->rx_pkt_ready = false;
}

/****************************************************************************
 * Name: ncv_print_footer
 *
 * Description:
 *   print individual bitfield of a receive chunk footer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_NET_INFO
static void ncv_print_footer(uint32_t footer)
{
  ninfo("Footer:\n");
  ninfo("  EXST: %d\n", oa_ext_status(footer));
  ninfo("  HDRB: %d\n", oa_header_bad(footer));
  ninfo("  SYNC: %d\n", oa_mac_phy_sync(footer));
  ninfo("  RCA:  %d\n", oa_rx_available(footer));
  ninfo("  DV:   %d\n", oa_data_valid(footer));
  ninfo("  SV:   %d\n", oa_start_valid(footer));
  ninfo("  SWO:  %d\n", oa_start_word_offset(footer));
  ninfo("  FD:   %d\n", oa_frame_drop(footer));
  ninfo("  EV:   %d\n", oa_end_valid(footer));
  ninfo("  EBO:  %d\n", oa_end_byte_offset(footer));
  ninfo("  RTSA: %d\n", oa_rx_frame_timestamp_added(footer));
  ninfo("  RTSP: %d\n", oa_rx_frame_timestamp_parity(footer));
  ninfo("  TXC:  %d\n", oa_tx_credits(footer));
}
#endif

/****************************************************************************
 * Netdev upperhalf callbacks
 ****************************************************************************/

/****************************************************************************
 * Name: ncv7410_ifup
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

static int ncv7410_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;

  if (priv->ifstate == NCV_INIT_UP)
    {
      nerr("Tried to bring NCV7410 interface up when already up\n");
      return -EINVAL;
    }

  ninfo("Bringing up NCV7410\n");

  if (priv->ifstate == NCV_RESET)
    {
      if (ncv_config(priv) == ERROR)
        {
          nerr("Error configuring NCV7410\n");
          return -EIO;
        }

      priv->ifstate = NCV_INIT_DOWN;
    }

  /* set NCV_INIT_UP prior to enabling to allow ncv_interrupt_work right
   * after MAC-PHY enable
   */

  priv->ifstate = NCV_INIT_UP;

  if (ncv_enable(priv) == ERROR)
    {
      nerr("Error enabling NCV7410\n");
      priv->ifstate = NCV_INIT_DOWN;
      return -EIO;
    }

  /* schedule interrupt work to initialize txc and rca */

  work_queue(NCVWORK, &priv->interrupt_work, ncv_interrupt_work, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: ncv7410_ifdown
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

static int ncv7410_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;

  nxmutex_lock(&priv->lock);

  if (priv->ifstate != NCV_INIT_UP)
    {
      nxmutex_unlock(&priv->lock);
      nerr("Tried to bring the NCV7410 interface down but it is not up\n");
      return -EINVAL;
    }

  work_cancel(NCVWORK, &priv->interrupt_work);
  work_cancel(NCVWORK, &priv->io_work);

  if (ncv_disable(priv) == ERROR)
    {
      nxmutex_unlock(&priv->lock);
      nerr("Error disabling NCV7410\n");
      return -EIO;
    }

  ncv_reset_driver_buffers(priv);

  priv->ifstate = NCV_INIT_DOWN;

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: ncv7410_transmit
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

static int ncv7410_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;

  nxmutex_lock(&priv->lock);

  if (priv->tx_pkt || priv->ifstate != NCV_INIT_UP)
    {
      /* previous tx packet was not yet sent to the network
       * or the interface was shut down while waiting for the lock
       */

      nxmutex_unlock(&priv->lock);
      return -EAGAIN;
    }

  priv->tx_pkt_idx = 0;
  priv->tx_pkt_len = netpkt_getdatalen(dev, pkt);
  priv->tx_pkt = pkt;

  nxmutex_unlock(&priv->lock);

  work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: ncv7410_receive
 *
 * Description:
 *   NuttX callback: Claims an rx packet if available.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   If the rx packet is ready, its pointer is returned.
 *   NULL is returned otherwise.
 *
 ****************************************************************************/

static FAR netpkt_t *ncv7410_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;

  nxmutex_lock(&priv->lock);

  if (priv->rx_pkt_ready)
    {
      netpkt_t *retval = priv->rx_pkt;
      priv->rx_pkt_ready = false;
      priv->rx_pkt = NULL;
      nxmutex_unlock(&priv->lock);
      return retval;
    }

  nxmutex_unlock(&priv->lock);

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv7410_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.
 *
 * Input Parameters:
 *   spi - reference to the SPI driver state data
 *   irq - irq number of the pin connected to MAC-PHY's interrupt signal
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int ncv7410_initialize(FAR struct spi_dev_s *spi, int irq,
                       struct ncv7410_config_s *config)
{
  FAR struct ncv7410_driver_s   *priv   = NULL;
  FAR struct netdev_lowerhalf_s *netdev = NULL;
  int retval;

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Could not allocate data for ncv7410 priv\n");
      return -ENOMEM;
    }

  priv->spi = spi;       /* Save the SPI instance                   */
  priv->irqnum = irq;    /* Save the Interrupt Request Number       */
  priv->config = config; /* Save the reference to the configuration */

  /* Reset NCV7410 chip */

  if (ncv_reset(priv))
    {
      nerr("Error resetting NCV7410\n");
      retval = -EIO;
      goto errout;
    }

  priv->ifstate = NCV_RESET;
  ninfo("Resetting NCV7410 OK\n");

  if (ncv_init_mac_addr(priv))
    {
      nerr("Error initializing NCV7410 MAC address\n");
      retval = -EIO;
      goto errout;
    }

  ninfo("Initializing MAC address OK\n");

  /* Attach ISR */

  irq_attach(priv->irqnum, ncv_interrupt, priv);

  /* Init lock */

  nxmutex_init(&priv->lock);

  /* Register the device with the OS */

  netdev = &priv->dev;
  netdev->quota[NETPKT_TX] = NCV7410_TX_QUOTA;
  netdev->quota[NETPKT_RX] = NCV7410_RX_QUOTA;
  netdev->ops = &g_ncv7410_ops;

  retval = netdev_lower_register(netdev, NET_LL_ETHERNET);
  if (retval == OK)
    {
      ninfo("Successfully registered NCV7410 network driver\n");
      return OK;
    }

  nerr("Error registering NCV7410 network driver: %d\n", retval);

errout:
  kmm_free(priv);
  return retval;
}
