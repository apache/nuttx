/****************************************************************************
 * drivers/wireless/spirit/drivers/spirit_netdev.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <assert.h>
#include <debug.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mm/iob.h>
#include <nuttx/spi/spi.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/sixlowpan.h>

#include <nuttx/wireless/spirit.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "spirit_types.h"
#include "spirit_general.h"
#include "spirit_irq.h"
#include "spirit_spi.h"
#include "spirit_gpio.h"
#include "spirit_linearfifo.h"
#include "spirit_commands.h"
#include "spirit_radio.h"
#include "spirit_pktbasic.h"
#include "spirit_qi.h"
#include "spirit_management.h"
#include "spirit_timer.h"
#include "spirit_csma.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_HPWORK) || !defined(CONFIG_SCHED_HPWORK)
#  error Both high and low priority work queues required in this driver
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

#if !defined(CONFIG_NET) || !defined(CONFIG_NET_6LOWPAN)
#  error 6LoWPAN network support is required.
#endif

#ifndef CONFIG_SPIRIT_PKTLEN
#  define CONFIG_SPIRIT_PKTLEN 96
#endif

#ifndef CONFIG_SPIRIT_MTU
#  define CONFIG_SPIRIT_MTU 596
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define SPIRIT_WDDELAY   (1*CLK_TCK)

/* TX timeout = 1 minute */

#define SPIRIT_TXTIMEOUT (60*CLK_TCK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum spirit_driver_state_e
{
  DRIVER_STATE_IDLE = 0,
  DRIVER_STATE_SENDING,
  DRIVER_STATE_RECEIVING
};

/* SPIRIT1 device instance
 *
 * Make sure that struct ieee802154_radio_s remains first.  If not it will break the
 * code
 */

struct spirit_driver_s
{
  struct ieee802154_driver_s       ieee;      /* Interface understood by the network */
  struct spirit_library_s          spirit;    /* Spirit library state */
  FAR const struct spirit_lower_s *lower;     /* Low-level MCU-specific support */
  FAR struct iob_s                *txhead;    /* Head of pending TX transfers */
  FAR struct iob_s                *txtail;    /* Tail of pending TX transfers */
  FAR struct iob_s                *rxhead;    /* Head of completed RX transfers */
  FAR struct iob_s                *rxtail;    /* Tail of completed RX transfers */
  struct work_s                    hpwork;    /* Interrupt continuation work queue support */
  struct work_s                    lpwork;    /* Net poll work queue support */
  WDOG_ID                          txpoll;    /* TX poll timer */
  WDOG_ID                          txtimeout; /* TX timeout timer */
  sem_t                            exclsem;   /* Mutually exclusive access */
  bool                             ifup;      /* Spirit is on and interface is up */
  uint8_t                          state;     /* See  enum spirit_driver_state_e */
  uint8_t                          panid[2];  /* PAN identifier, ffff = not set */
  uint16_t                         saddr;     /* Short address, ffff = not set */
  uint8_t                          eaddr[8];  /* Extended address, ffffffffffffffff = not set */
  uint8_t                          channel;   /* 11 to 26 for the 2.4 GHz band */
  uint8_t                          devmode;   /* Device mode: device, coord, pancoord */
  uint8_t                          paenabled; /* Enable usage of PA */
  uint8_t                          rxmode;    /* Reception mode: Main, no CRC, promiscuous */
  int32_t                          txpower;   /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s          cca;       /* Clear channel assessement method */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void spirit_lock(FAR struct spirit_driver_s *priv);
#define spirit_unlock(priv) sem_post(&priv->exclsem);

static int spirit_waitstatus(FAR struct spirit_library_s *spirit,
                             enum spirit_state_e state, unsigned int msec);
static int spirit_set_readystate(FAR struct spirit_driver_s *priv);

/* TX-related logic */

static int  spirit_transmit(FAR struct spirit_driver_s *priv);
static void sprit_transmit_work(FAR void *arg);
static void spirit_schedule_transmit_work(FAR struct spirit_driver_s *priv);

static int  spirit_txpoll_callback(FAR struct net_driver_s *dev);

/* RX-related logic */

static void sprit_receive_work(FAR void *arg);
static void spirit_schedule_receive_work(FAR struct spirit_driver_s *priv);

/* Interrupt handling */

static void spirit_interrupt_work(FAR void *arg);
static int  spirit_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void spirit_txtimeout_work(FAR void *arg);
static void spirit_txtimeout_expiry(int argc, wdparm_t arg, ...);

static void spirit_poll_work(FAR void *arg);
static void spirit_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  spirit_ifup(FAR struct net_driver_s *dev);
static int  spirit_ifdown(FAR struct net_driver_s *dev);

static void spirit_txavail_work(FAR void *arg);
static int  spirit_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int  spirit_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  spirit_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void spirit_ipv6multicast(FAR struct spirit_driver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  spirit_ioctl(FAR struct net_driver_s *dev, int cmd,
            unsigned long arg);
#endif
static int spirit_get_mhrlen(FAR struct ieee802154_driver_s *netdev,
            FAR const struct ieee802154_frame_meta_s *meta);
static int spirit_req_data(FAR struct ieee802154_driver_s *netdev,
            FAR const struct ieee802154_frame_meta_s *meta,
            FAR struct iob_s *framelist);

/* Initialization */

int spirit_hw_initialize(FAR struct spirit_driver_s *dev,
            FAR struct spi_dev_s *spi);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Spirit radio initialization */

static const struct radio_init_s g_radio_init =
{
  SPIRIT_BASE_FREQUENCY,              /* base_frequency */
  SPIRIT_CHANNEL_SPACE,               /* chspace */
  SPIRIT_XTAL_OFFSET_PPM,             /* xtal_offset_ppm */
  SPIRIT_CHANNEL_NUMBER,              /* chnum */
  SPIRIT_MODULATION_SELECT,           /* modselect */
  SPIRIT_DATARATE,                    /* datarate */
  SPIRIT_FREQ_DEVIATION,              /* freqdev */
  SPIRIT_BANDWIDTH                    /* bandwidth */
};

/* Spirit PktBasic initialization */

static const struct pktbasic_init_s g_pktbasic_init =
{
  SPIRIT_SYNC_WORD,                   /* syncwords */
  SPIRIT_PREAMBLE_LENGTH,             /* premblen */
  SPIRIT_SYNC_LENGTH,                 /* synclen */
  SPIRIT_LENGTH_TYPE,                 /* fixvarlen */
  SPIRIT_LENGTH_WIDTH,                /* pktlenwidth */
  SPIRIT_CRC_MODE,                    /* crcmode */
  SPIRIT_CONTROL_LENGTH,              /* ctrllen */
  SPIRIT_EN_ADDRESS,                  /* txdestaddr */
  SPIRIT_EN_FEC,                      /* fec */
  SPIRIT_EN_WHITENING                 /* datawhite */
 };

static const struct spirit_gpio_init_s g_gpioinit =
{
  SPIRIT_GPIO_3,                      /* gpiopin */
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP, /* gpiomode */
  SPIRIT_GPIO_DIG_OUT_IRQ             /* gpioio */
};

static const struct spirit_csma_init_s g_csma_init =
{
  1,                /* BU counter seed */
  S_ENABLE,         /* enable persistent mode */
  TBIT_TIME_64,     /* Tcca time */
  TCCA_TIME_3,      /* Lcca length */
  3,                /* max nr of backoffs (<8) */
  8                 /* BU prescaler */
};

#ifdef CONFIG_SPIRIT_MULTICAST
static struct pktbasic_addr_s g_addrinit =
{
  S_DISABLE,                         /* Disable filtering on node address */
  0x34,                              /* Note address (Temporary, until assigned) */
  S_DISABLE,                         /* Disable filtering on multicast address */
  0xee,                              /* Multicast address */
  S_DISABLE,                         /* Disable filtering on broadcast address */
  0xff                               /* Broadcast address */
};
#else
static struct pktbasic_addr_s g_addrinit =
{
  S_ENABLE,                          /* Enable filtering on node address */
  0x34,                              /* Note address (Temporary, until assigned) */
#ifdef CONFIG_SPIRIT_MULTICAST
  S_ENABLE,                          /* Enable filtering on multicast address */
#else
  S_DISABLE,                         /* Disable filtering on multicast address */
#endif
  0xee,                              /* Multicast address */
#ifdef CONFIG_SPIRIT_BROADCAST
  S_ENABLE,                          /* Enable filtering on broadcast address */
#else
  S_DISABLE,                         /* Disable filtering on broadcast address */
#endif
  0xff                               /* Broadcast address */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spirit_waitstatus
 *
 * Description:
 *   Get exclusive access to the driver instance and to the spirit library.
 *
 * Parameters:
 *   priv - Reference to a driver state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spirit_lock(FAR struct spirit_driver_s *priv)
{
  while (sem_wait(&priv->exclsem) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: spirit_waitstatus
 *
 * Description:
 *   Poll until the Spirit status is the requested value or until a timeout
 *   occurs.
 *
 * Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   state  - That that we are waiting for.
 *   msec   - Timeout in millisedonds
 *
 * Returned Value:
 *   OK on success; a negated errno on a timeout
 *
 * Assumptions:
 *   We have exclusive access to the driver state and to the spirit library.
 *
 ****************************************************************************/

static int spirit_waitstatus(FAR struct spirit_library_s *spirit,
                             enum spirit_state_e state, unsigned int msec)
{
  systime_t start;
  unsigned int ticks;
  unsigned int elapsed;
  int ret;

  /* MSEC to clock ticks (add one to make sure we wait at least requested
   * timeout)
   */

  ticks = MSEC2TICK(msec);
  if (ticks == 0)
    {
      /* The timeout is below the current timer resolution */

      ticks = 1;
    }

  /* The time that we started the wait */

  start = clock_systimer();

  /* Loop until the status change occurs (or the wait times out) */

  do
    {
      ret = spirit_update_status(spirit);
      if (ret < 0)
        {
          return ret;
        }

      elapsed = clock_systimer() - start;
    }
  while (spirit->u.state.MC_STATE != state && elapsed <= ticks);

  return (spirit->u.state.MC_STATE == state) ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: spirit_set_readystate
 *
 * Description:
 *   Got to the READY state (if possible).
 *
 * Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   OK on success; a negated errno on a timeout
 *
 * Assumptions:
 *   We have exclusive access to the driver state and to the spirit library.
 *
 ****************************************************************************/

static int spirit_set_readystate(FAR struct spirit_driver_s *priv)
{
  FAR struct spirit_library_s *spirit = &priv->spirit;
  int ret;

  DEBUGASSERT(priv->lower != NULL && priv->lower->enable != NULL);
  priv->lower->enable(priv->lower, false);

  ret = spirit_command(spirit, CMD_FLUSHRXFIFO);
  if (ret < 0)
    {
      goto errout_with_irqdisable;
    }

  ret = spirit_update_status(spirit);
  if (ret < 0)
    {
      goto errout_with_irqdisable;
    }

  if (spirit->u.state.MC_STATE == MC_STATE_STANDBY)
    {
      ret = spirit_command(spirit, CMD_READY);
    }
  else if(spirit->u.state.MC_STATE == MC_STATE_RX)
    {
      ret = spirit_command(spirit, CMD_SABORT);
    }

  if (ret < 0)
    {
      goto errout_with_irqdisable;
    }

  ret = spirit_irq_clr_pending(spirit);

errout_with_irqdisable:
  priv->lower->enable(priv->lower, true);
  return ret;
}

/****************************************************************************
 * Name: spirit_transmit
 *
 * Description:
 *   Start hardware transmission.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int spirit_transmit(FAR struct spirit_driver_s *priv)
{
  FAR struct spirit_library_s *spirit = &priv->spirit;
  FAR struct iob_s *iob;
  int ret;

  /* Check if there are any pending transfers in the TX AND if the hardware
   * is not busy with another reception or transmission.
   */

  spirit_lock(priv);
  while (priv->txhead != NULL && priv->state == DRIVER_STATE_IDLE)
    {
      /* Remove the IOB from the head of the TX queue */

      iob          = priv->txhead;
      priv->txhead = iob->io_flink;

      if (priv->txhead == NULL)
        {
          priv->txtail = NULL;
        }

      DEBUGASSERT(iob != NULL);

      /* Checks if the payload length is supported */

      if (iob->io_len > CONFIG_SPIRIT_PKTLEN)
        {
          NETDEV_RXDROPPED(&priv->ieee.i_dev);
          iob_free(iob);
          continue;
        }

      priv->state = DRIVER_STATE_SENDING;

      /* Reset state to ready */

      ret = spirit_set_readystate(priv);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

      /* Make sure that the TX linear FIFO is completely empty */

      ret = spirit_command(spirit, COMMAND_FLUSHTXFIFO);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

      /* Sets the length of the packet to send */

      ret = spirit_pktbasic_set_payloadlen(spirit, iob->io_len);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

#ifndef CONFIG_SPIRIT_PROMISCOUS
      /* Set the destination address */

#warning Missing logic
#if 0
      ret = spirit_pktcommon_set_txdestaddr(spirit, txdestaddr);
      if (ret < 0)
        {
          goto errout_with_iob;
        }
#endif
#endif

      /* Enable CSMA */

      ret = spirit_csma_enable(spirit, S_ENABLE);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

      /* Write the packet to the linear FIFO */

      ret = spirit_fifo_write(spirit, iob->io_data, iob->io_len);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

      /* Put the SPIRIT1 into TX state.  This starts the transmission */

      ret = spirit_command(spirit, COMMAND_TX);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

      /* Wait until we have successfully entered the TX state */

      ret = spirit_waitstatus(spirit, MC_STATE_TX, 5);
      if (ret < 0)
        {
          goto errout_with_iob;
        }

      /* Setup the TX timeout watchdog (perhaps restarting the timer) */

      (void)wd_start(priv->txtimeout, SPIRIT_TXTIMEOUT,
                     spirit_txtimeout_expiry, 1, (wdparm_t)priv);
    }

  spirit_unlock(priv);
  return OK;

errout_with_iob:
  spirit_unlock(priv);
  NETDEV_RXDROPPED(&priv->ieee.i_dev);
  iob_free(iob);
  return ret;
}

/****************************************************************************
 * Name: sprit_transmit_work
 *
 * Description:
 *   Send data on the LP work queue.
 *
 * Parameters:
 *   arg - Reference to driver state structure (cast to void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sprit_transmit_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  net_lock();
  spirit_transmit(priv);
  net_unlock();
}

/****************************************************************************
 * Name: spirit_schedule_transmit_work
 *
 * Description:
 *   Schedule to send data on the LP work queue.
 *
 * Parameters:
 *   priv - Reference to driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from logic running the HP work queue.
 *
 ****************************************************************************/

static void spirit_schedule_transmit_work(FAR struct spirit_driver_s *priv)
{
  if(priv->txhead != NULL && priv->state == DRIVER_STATE_IDLE)
    {
      /* Schedule to perform the TX processing on the worker thread. */

      work_queue(LPWORK, &priv->lpwork, sprit_transmit_work, priv, 0);
    }
}

/****************************************************************************
 * Name: spirit_txpoll_callback
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int spirit_txpoll_callback(FAR struct net_driver_s *dev)
{
  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: sprit_receive_work
 *
 * Description:
 *   Pass received packets to the network on the LP work queue.
 *
 * Parameters:
 *   arg - Reference to driver state structure (cast to void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sprit_receive_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  net_lock();
#warning Missing logic
  //spirit_receive(priv);
  net_unlock();
}

/****************************************************************************
 * Name: spirit_schedule_receive_work
 *
 * Description:
 *   Schedule to receive data on the LP work queue.
 *
 * Parameters:
 *   priv - Reference to driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from logic running the HP work queue.
 *
 ****************************************************************************/

static void spirit_schedule_receive_work(FAR struct spirit_driver_s *priv)
{
  if(priv->txhead != NULL && priv->state == DRIVER_STATE_IDLE)
    {
      /* Schedule to perform the TX processing on the worker thread. */

      work_queue(LPWORK, &priv->lpwork, sprit_receive_work, priv, 0);
    }
}

/****************************************************************************
 * Name: spirit_interrupt_work
 *
 * Description:
 *   Actual thread to handle the irq outside of privaleged mode.
 *
 ****************************************************************************/

static void spirit_interrupt_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;
  FAR struct spirit_library_s *spirit;
  struct spirit_irqset_s irqstatus;

  DEBUGASSERT(priv != NULL);
  spirit = &priv->spirit;

  /* Get the interrupt source from radio */

  spirit_lock(priv);
  DEBUGVERIFY(spirit_irq_get_pending(spirit, &irqstatus));
  DEBUGVERIFY(spirit_irq_clr_pending(spirit));

  /* Process the Spirit1 interrupt */
  /* First check for errors */

  if (irqstatus.IRQ_RX_FIFO_ERROR != 0)
    {
      wlwarn("WARNING: Rx FIFO Error\n");
      DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));

      priv->state = DRIVER_STATE_IDLE;
      NETDEV_RXERRORS(&priv->ieee.i_dev);

      /* Send any pending packets */

      spirit_csma_enable(spirit, S_DISABLE);
      spirit_schedule_transmit_work(priv);
    }

  if (irqstatus.IRQ_TX_FIFO_ERROR != 0)
    {
      wlwarn("WARNING: Tx FIFO Error\n");
      DEBUGVERIFY(spirit_command(spirit, COMMAND_FLUSHTXFIFO));

      priv->state = DRIVER_STATE_IDLE;
      NETDEV_TXERRORS(&priv->ieee.i_dev);

      /* Send any pending packets */

      spirit_csma_enable(spirit, S_DISABLE);
      spirit_schedule_transmit_work(priv);
    }

  /* The IRQ_TX_DATA_SENT bit notifies that a packet was sent. */

  if (irqstatus.IRQ_TX_DATA_SENT != 0)
    {
      /* Put the Spirit back in the receiving state */

      DEBUGVERIFY(spirit_management_rxstrobe(spirit));
      DEBUGVERIFY(spirit_command(spirit, CMD_RX));

      NETDEV_TXDONE(&priv->ieee.i_dev)
      spirit_csma_enable(spirit, S_DISABLE);

      /* Check if there are more packets to send */

      DEBUGASSERT(priv->state == DRIVER_STATE_SENDING);
      priv->state = DRIVER_STATE_IDLE;
      spirit_schedule_transmit_work(priv);
    }

#ifdef CONFIG_SPIRIT_FIFOS
  /* The IRQ_TX_FIFO_ALMOST_EMPTY notifies an nearly empty TX fifo.
   * Necessary for sending large packets > sizeof(TX FIFO).
   */

  if (irqstatus.IRQ_TX_FIFO_ALMOST_EMPTY != 0)
    {
#warning Missing logic
    }
#endif

  /* The IRQ_VALID_SYNC bit is used to notify a new packet is coming */

  if (irqstatus.IRQ_VALID_SYNC != 0)
    {
      DEBUGASSERT(priv->state == DRIVER_STATE_IDLE);
      priv->state = DRIVER_STATE_RECEIVING;
    }

  /* The IRQ_RX_DATA_READY notifies that a new packet has been received */

  if (irqstatus.IRQ_RX_DATA_READY != 0)
    {
      FAR struct iob_s *iob;
      uint8_t count;

      /* Check the packet size */

      count = spirit_fifo_get_rxcount(spirit);
      if (count > CONFIG_IOB_BUFSIZE)
        {
          wlwarn("WARNING:  Packet too large... dropping\n");
          DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
          priv->state = DRIVER_STATE_IDLE;

          /* Send any pending packets */

          spirit_schedule_transmit_work(priv);
        }
      else
        {
          /* Allocate an I/O buffer to hold the received packet.
           * REVISIT: Not a good place to wait.  Perhaps we should pre-
           * allocate a few I/O buffers?
           */

          iob = iob_alloc(0);
          if (iob == NULL)
            {
              wlerr("ERROR:  Packet too large... dropping\n");

              DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
              priv->state = DRIVER_STATE_IDLE;
              NETDEV_RXDROPPED(&priv->ieee.i_dev);

              /* Send any pending packets */

              spirit_schedule_transmit_work(priv);
            }
          else
            {

              /* Read the packet into the I/O buffer */

              DEBUGVERIFY(spirit_fifo_read(spirit, iob->io_data, count));
              iob->io_len = spirit_pktbasic_get_rxpktlen(spirit);

              DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
              priv->state = DRIVER_STATE_IDLE;

              NETDEV_RXPACKETS(&priv->ieee.i_dev);

              /* Add the IO buffer to the tail of the completed RX transfers */

              iob->io_flink = priv->rxtail;
              priv->rxtail  = iob;

              if (priv->rxhead == NULL)
                {
                  priv->rxhead = iob;
                }

              /* Create the packet meta data and forward to the network.  This
               * must be done on the LP work queue with the network lockes.
               */
#warning Missing logic

              /* Forward the packet to the network.  This must be done on the
               * LP work queue with the network locked.
               */

              spirit_schedule_receive_work(priv);

              /* Try sending the next packet */

              spirit_schedule_transmit_work(priv);
            }
        }
    }

#ifdef CONFIG_SPIRIT_FIFOS
  /* The IRQ_RX_FIFO_ALMOST_FULL notifies an nearly full RX fifo.
   * Necessary for receiving large packets > sizeof(RX FIFO).
   */

  if (irqstatus.IRQ_RX_FIFO_ALMOST_FULL != 0)
    {
#warning Missing logic
    }
#endif

  /* IRQ_RX_DATA_DISC indicates that Rx data was discarded */

  if (irqstatus.IRQ_RX_DATA_DISC)
    {
      DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
      priv->state = DRIVER_STATE_IDLE;
      NETDEV_RXDROPPED(&priv->ieee.i_dev);
    }

  /* Check the Spirit status.  If it is IDLE, the setup to receive more */

  DEBUGVERIFY(spirit_update_status(spirit));

  if (spirit->u.state.MC_STATE == MC_STATE_READY)
    {
      /* Set up to receive */

      DEBUGVERIFY(spirit_command(spirit, CMD_RX));

      /* Wait for Spirit to enter the Tx state (or timeut) */

      DEBUGVERIFY(spirit_waitstatus(spirit, MC_STATE_RX, 1));
    }

  /* Re-enable the interrupt. */

  DEBUGASSERT(priv->lower != NULL && priv->lower->enable != NULL);
  priv->lower->enable(priv->lower, true);
  spirit_unlock(priv);
}

/****************************************************************************
 * Name: spirit_interrupt
 *
 * Description:
 *   Actual interrupt handler ran inside privileged space.
 *
 ****************************************************************************/

static int spirit_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* TODO: Determine if a TX transfer just completed .
   * If a TX transfer just completed, then cancel the TX timeout so
   * there will be no race condition between any subsequent timeout
   * expiration and the deferred interrupt processing.
   */

  //wd_cancel(priv->txtimeout);

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   *
   * Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  priv->lower->enable(priv->lower, false);

  return work_queue(HPWORK, &priv->hpwork, spirit_interrupt_work,
                    (FAR void *)priv, 0);
}

/****************************************************************************
 * Name: spirit_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void spirit_txtimeout_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(&priv->ieee.i_dev);

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->ieee.i_dev, spirit_txpoll_callback);
  net_unlock();
}

/****************************************************************************
 * Name: spirit_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void spirit_txtimeout_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  DEBUGASSERT(priv != NULL && priv->lower != NULL);

  /* Disable further Spirit interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  DEBUGASSERT(priv->lower->enable != NULL);
  priv->lower->enable(priv->lower, false);

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(LPWORK, &priv->hpwork, spirit_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: spirit_poll_process
 *
 * Description:
 *   Perform the periodic poll.  This may be called either from watchdog
 *   timer logic or from the worker thread, depending upon the configuration.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void spirit_poll_process(FAR struct spirit_driver_s *priv)
{
}

/****************************************************************************
 * Name: spirit_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Scheduled on the LP worker thread  from the poll timer expiration
 *   handler.
 *
 ****************************************************************************/

static void spirit_poll_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Perform the periodic poll */

  (void)devif_timer(&priv->ieee.i_dev, spirit_txpoll_callback);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, SPIRIT_WDDELAY, spirit_poll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: spirit_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void spirit_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(LPWORK, &priv->lpwork, spirit_poll_work, priv, 0);
}

/****************************************************************************
 * Name: spirit_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Spirit interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 *   Called from the network layer and running on the LP working thread.
 *
 ****************************************************************************/

static int spirit_ifup(FAR struct net_driver_s *dev)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)dev->d_private;
  FAR struct spirit_library_s *spirit;
  int ret;

  DEBUGASSERT(priv != NULL);
  spirit = &priv->spirit;

  if (!priv->ifup)
    {
      ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
            dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
            dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
            dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

      /* Disable spirit interrupts */

      DEBUGASSERT(priv->lower->enable != NULL);
      priv->lower->enable(priv->lower, false);

      /* Ensure we are in READY state as we go from there to Rx.
       * Since spirit interrupts are disabled, we don't need to be concerned
       * about mutual exclusion.
       */

      ret = spirit_command(spirit, CMD_READY);
      if (ret < 0)
        {
          return ret;
        }

      ret = spirit_waitstatus(spirit, MC_STATE_READY, 5);
      if (ret < 0)
        {
          goto error_with_ifalmostup;
        }

      /* Now we go to Rx */

      ret = spirit_command(spirit, CMD_RX);
      if (ret < 0)
        {
          goto error_with_ifalmostup;
        }

      ret = spirit_waitstatus(spirit, MC_STATE_RX, 5);
      if (ret < 0)
        {
          goto error_with_ifalmostup;
        }

#ifndef CONFIG_SPIRIT_PROMISCOUS
      /* Instantiate the assigned node address */

#warning Missing logic
#if 0
      ret = spirit_pktcommon_set_nodeaddress(spirit, node_address);
      if (ret < 0)
        {
          goto error_with_ifalmostup;
        }
#endif
#endif

      /* Set and activate a timer process */

      (void)wd_start(priv->txpoll, SPIRIT_WDDELAY, spirit_poll_expiry, 1,
                     (wdparm_t)priv);

      /* Enables the interrupts from the SPIRIT1 */

      DEBUGASSERT(priv->lower->enable != NULL);
      priv->lower->enable(priv->lower, true);

      /* We are up! */

      priv->ifup = true;
  }

  return OK;

error_with_ifalmostup:
  priv->ifup = true;
  (void)spirit_ifdown(dev);
  return ret;
}

/****************************************************************************
 * Name: spirit_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the network layer and running on the LP working thread.
 *
 ****************************************************************************/

static int spirit_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)dev->d_private;
  FAR struct spirit_library_s *spirit;
  int ret = OK;

  DEBUGASSERT(priv != NULL);
  spirit = &priv->spirit;

  if (priv->ifup)
    {
      irqstate_t flags;
      int status;

      /* Disable the Spirit interrupt */

      flags = enter_critical_section();

      DEBUGASSERT(priv->lower->enable != NULL);
      priv->lower->enable(priv->lower, false);

      /* Cancel the TX poll timer and TX timeout timers */

      wd_cancel(priv->txpoll);
      wd_cancel(priv->txtimeout);
      leave_critical_section(flags);

      /* First stop Rx/Tx
       * Since spirit interrupts are disabled, we don't need to be concerned
       * about mutual exclusion.
       */

      status = spirit_command(spirit, CMD_SABORT);
      if (status < 0 && ret == 0)
        {
          ret = status;
        }

      /* Clear any pending irqs */

      status = spirit_irq_clr_pending(spirit);
      if (status < 0 && ret == 0)
        {
          ret = status;
        }

      status = spirit_waitstatus(spirit, MC_STATE_READY, 5);
      if (status < 0 && ret == 0)
        {
          ret = status;
        }

      /* Put the SPIRIT1 in STANDBY */

      status = spirit_command(spirit, CMD_STANDBY);
      if (status < 0 && ret == 0)
        {
          ret = status;
        }

      status = spirit_waitstatus(spirit, MC_STATE_STANDBY, 5);
      if (status < 0 && ret == 0)
        {
          ret = status;
        }

      priv->ifup = false;
    }

  return ret;
}

/****************************************************************************
 * Name: spirit_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void spirit_txavail_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->ifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll the network for new XMIT data */

      (void)devif_poll(&priv->ieee.i_dev, spirit_txpoll_callback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: spirit_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network logic running on the LP work queue
 *
 ****************************************************************************/

static int spirit_txavail(FAR struct net_driver_s *dev)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  spirit_lock(priv);
  if (work_available(&priv->lpwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPWORK, &priv->lpwork, spirit_txavail_work, priv, 0);
    }

  spirit_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: spirit_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int spirit_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spirit_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int spirit_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spirit_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void spirit_ipv6multicast(FAR struct spirit_driver_s *priv)
{
  return -ENOSYS;
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: spirit_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int spirit_ioctl(FAR struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)dev->d_private;
  int ret;

  /* Decode and dispatch the driver-specific IOCTL command */

  spirit_lock(priv);
  switch (cmd)
    {
      /* Add cases here to support the IOCTL commands */

      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", command);
        ret = -ENOTTY;  /* Special return value for this case */
    }

  spirit_unlock(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: spirit_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Meta data needed to recreate the MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Called from network logic running on the low-priority work queue.
 *
 ****************************************************************************/

static int spirit_get_mhrlen(FAR struct ieee802154_driver_s *netdev,
                             FAR const struct ieee802154_frame_meta_s *meta)
{
  FAR struct spirit_driver_s *priv;

  DEBUGASSERT(netdev != NULL && netdev->i_dev.d_private != NULL && meta != NULL);
  priv = (FAR struct spirit_driver_s *)netdev->i_dev.d_private;

  spirit_lock(priv);
#warning Missing logic
  spirit_unlock(priv);
  return -ENOSYS;
}

/****************************************************************************
 * Name: spirit_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.
 *
 * Input parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Meta data needed to recreate the MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from network logic with the network locked.
 *
 ****************************************************************************/

static int spirit_req_data(FAR struct ieee802154_driver_s *netdev,
                           FAR const struct ieee802154_frame_meta_s *meta,
                           FAR struct iob_s *framelist)
{
  FAR struct spirit_driver_s *priv;
  FAR struct iob_s *iob;

  wlinfo("Received framelist\n");

  DEBUGASSERT(netdev != NULL && netdev->i_dev.d_private != NULL);
  priv = (FAR struct spirit_driver_s *)netdev->i_dev.d_private;

  DEBUGASSERT(meta != NULL && framelist != NULL);

  /* Add the incoming list of frames to the MAC's outgoing queue */

  spirit_lock(priv);
  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->md_dev.i_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      /* Apply the Spirit header to the frame */
# warning Missing logic

      /* Add the IOB to the queue of outgoing IOBs. */

      iob->io_flink = priv->txtail;
      priv->txtail  = iob;

      if (priv->txhead == NULL)
        {
          priv->txhead = iob;
        }

      /* If there are no transmissions or receptions in progress, then start
       * tranmission of the frame in the IOB at the head of the IOB queue.
       */

       spirit_transmit(priv);
    }

  spirit_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: spirit_hw_initialize
 *
 * Description:
 *   Initialize the Spirit1 radio.
 *
 ****************************************************************************/

int spirit_hw_initialize(FAR struct spirit_driver_s *priv,
                         FAR struct spi_dev_s *spi)
{
  FAR struct spirit_library_s *spirit = &priv->spirit;
  int ret;

  /* Configures the Spirit1 radio library */

  spirit->spi            = spi;
  spirit->xtal_frequency = SPIRIT_XTAL_FREQUENCY;

  /* Reset the Spirit1 radio part */

  DEBUGASSERT(priv->lower != NULL && priv->lower->reset != NULL);
  ret = priv->lower->reset(priv->lower) ;
  if (ret < 0)
    {
      return ret;
    }

  /* Soft reset of Spirit1 core */

  ret = spirit_command(spirit, COMMAND_SRES);
  if (ret < 0)
    {
      return ret;
    }

  priv->ifup = false;

  /* Perform VCO calibration WA when the radio is initialized */

  spirit_radio_enable_wavco_calibration(spirit, S_ENABLE);

  /* Configure the Spirit1 radio part */

  ret = spirit_radio_initialize(spirit, &g_radio_init);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_radio_set_palevel_dbm(spirit, 0, SPIRIT_POWER_DBM);
  if (ret < 0)
    {
      return ret;
    }

  ret =spirit_radio_set_palevel_maxindex(spirit, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Configures the SPIRIT1 packet handling logic */

  ret = spirit_pktbasic_initialize(spirit, &g_pktbasic_init);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure address filtering */

  ret = spirit_pktbasic_addr_initialize(spirit, &g_addrinit);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the following interrupt sources, routed to GPIO */

  ret = spirit_irq_disable_all(spirit);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_clr_pending(spirit);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, TX_DATA_SENT, S_ENABLE);
   if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_DATA_READY, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, VALID_SYNC, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_DATA_DISC, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, TX_FIFO_ERROR, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_FIFO_ERROR, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_SPIRIT_FIFOS
  ret = spirit_irq_enable(spirit, TX_FIFO_ALMOST_EMPTY, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_FIFO_ALMOST_FULL, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Configure Spirit1 */

  ret = spirit_radio_persistentrx(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_qi_set_sqithreshold(spirit, SQI_TH_0);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_qi_enable_sqicheck(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_qi_set_rssithreshold_dbm(spirit, SPIRIT_CCA_THRESHOLD);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_timer_set_rxtimeout_stopcondition(spirit,
                                                 SQI_ABOVE_THRESHOLD);
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_timer_set_rxtimeout_counter(spirit, 0); /* 0=No timeout */
  if (ret < 0)
    {
      return ret;
    }

  ret = spirit_radio_afcfreezeonsync(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }

#if 0
  ret = spirit_calibration_rco(spirit, S_ENABLE);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Configure the radio to route the IRQ signal to its GPIO 3 */

  ret = spirit_gpio_initialize(spirit, &g_gpioinit);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup CSMA/CA */

  ret = spirit_csma_initialize(spirit, &g_csma_init);
  if (ret < 0)
    {
      return ret;
    }

  /* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */

  ret = spirit_command(spirit, CMD_STANDBY);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: spirit_netdev_initialize
 *
 * Description:
 *   Initialize the IEEE802.15.4 driver and register it as a network device.
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the spirit1
 *   lower - The MCU-specific interrupt used to control low-level MCU
 *           functions (i.e., spirit1 GPIO interrupts).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/


int spirit_netdev_initialize(FAR struct spi_dev_s *spi,
                             FAR const struct spirit_lower_s *lower)
{
  FAR struct spirit_driver_s *priv;
  FAR struct ieee802154_driver_s *ieee;
  FAR struct net_driver_s *dev;
  FAR uint8_t *pktbuf;
  int ret;

  /* Allocate a driver state structure instance */

  priv = (FAR struct spirit_driver_s *)kmm_zalloc(sizeof(struct spirit_driver_s));
  if (priv == NULL)
    {
      wlerr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Allocate a packet buffer */

  pktbuf = (uint8_t *)kmm_zalloc(CONFIG_SPIRIT_MTU + CONFIG_NET_GUARDSIZE);
  if (priv == NULL)
    {
      wlerr("ERROR: Failed to allocate a packet buffer\n");
      ret = -ENOMEM;
      goto errout_with_alloc;
    }

  /* Attach the interface, lower driver, and devops */

  priv->lower = lower;

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll        = wd_create();   /* Create periodic poll timer */
  priv->txtimeout     = wd_create();   /* Create TX timeout timer */

  DEBUGASSERT(priv->txpoll != NULL && priv->txtimeout != NULL);

  sem_init(&priv->exclsem, 0, 1);

  /* Initialize the IEEE 802.15.4 network device fields */

  ieee                = &priv->ieee;
  ieee->i_get_mhrlen  = spirit_get_mhrlen; /* Get MAC header length */
  ieee->i_req_data    = spirit_req_data;   /* Enqueue frame for transmission */

  /* Initialize the common network device fields */

  dev                 = &ieee->i_dev;
  dev->d_buf          = pktbuf;            /* Single packet buffer */
  dev->d_ifup         = spirit_ifup;       /* I/F up (new IP address) callback */
  dev->d_ifdown       = spirit_ifdown;     /* I/F down callback */
  dev->d_txavail      = spirit_txavail;    /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac       = spirit_addmac;     /* Add multicast MAC address */
  dev->d_rmmac        = spirit_rmmac;      /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl        = spirit_ioctl;      /* Handle network IOCTL commands */
#endif
  dev->d_private      = (FAR void *)priv;  /* Used to recover private state from dev */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling spirit_ifdown().
   */

  /* Read the MAC address from the hardware into dev->d_mac.ether.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed.
   * REVISIT:  What kind of a device is this?
   */

  (void)netdev_register(dev, NET_LL_IEEE802154);

  /* Attach irq */

  ret = lower->attach(lower, spirit_interrupt, priv);
  if (ret < 0)
    {
      goto errout_with_pktbuf;
    }

  /* Initialize device */

  ret = spirit_hw_initialize(priv, spi);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_hw_initialize failed: %d\n", ret);
      goto errout_with_attach;
    }

  /* Put the Device to RX ON Mode */

  /* Enable Radio IRQ */

  lower->enable(lower, true);
  return OK;

errout_with_attach:
  (void)lower->attach(lower, NULL, NULL);

errout_with_pktbuf:
#if 0
  kmm_free(pktbuf);
#endif

errout_with_alloc:
  kmm_free(priv);
  return ret;
}
