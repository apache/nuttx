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
#include <nuttx/fs/fs.h>
#include <nuttx/mm/iob.h>
#include <nuttx/spi/spi.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/sixlowpan.h>

#include <nuttx/wireless/spirit.h>
#include <nuttx/wireless/pktradio.h>

#include "spirit_config.h"
#include "spirit_types.h"
#include "spirit_general.h"
#include "spirit_irq.h"
#include "spirit_spi.h"
#include "spirit_gpio.h"
#include "spirit_linearfifo.h"
#include "spirit_commands.h"
#include "spirit_radio.h"
#include "spirit_pktstack.h"
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
#  define CONFIG_SPIRIT_PKTLEN SPIRIT_MAX_FIFO_LEN
#endif

#if CONFIG_SPIRIT_PKTLEN > SPIRIT_MAX_FIFO_LEN && \
    !defined(CONFIG_SPIRIT_FIFOS)
#  error Without CONFIG_SPIRIT_FIFOS, need CONFIG_SPIRIT_PKTLEN <= SPIRIT_MAX_FIFO_LEN
#  undef  CONFIG_SPIRIT_PKTLEN
#  define CONFIG_SPIRIT_PKTLEN SPIRIT_MAX_FIFO_LEN
#endif

/* The packet length width field specifies log2 or the number of bits used to
 * transfer the packet length.
 */

#if CONFIG_SPIRIT_PKTLEN < 2
#  define PKT_LENGTH_WIDTH 1      /* 0 - 1 */
#elif CONFIG_SPIRIT_PKTLEN < 4
#  define PKT_LENGTH_WIDTH 2      /* 2 - 3 */
#elif CONFIG_SPIRIT_PKTLEN < 8
#  define PKT_LENGTH_WIDTH 3      /* 4 - 7 */
#elif CONFIG_SPIRIT_PKTLEN < 16
#  define PKT_LENGTH_WIDTH 4      /* 8 - 15 */
#elif CONFIG_SPIRIT_PKTLEN < 32
#  define PKT_LENGTH_WIDTH 5      /* 16 - 31 */
#elif CONFIG_SPIRIT_PKTLEN < 64
#  define PKT_LENGTH_WIDTH 6      /* 32 - 63 */
#elif CONFIG_SPIRIT_PKTLEN < 128
#  define PKT_LENGTH_WIDTH 7      /* 63 - 127 */
#elif CONFIG_SPIRIT_PKTLEN < 256
#  define PKT_LENGTH_WIDTH 8      /* 128 - 255 */
#elif CONFIG_SPIRIT_PKTLEN < 512
#  define PKT_LENGTH_WIDTH 9      /* 256 - 255 */
#elif CONFIG_SPIRIT_PKTLEN < 1024
#  define PKT_LENGTH_WIDTH 10     /* 512 - 1023 */
#elif CONFIG_SPIRIT_PKTLEN < 2048
#  define PKT_LENGTH_WIDTH 11     /* 1024 - 2047 */
#elif CONFIG_SPIRIT_PKTLEN < 4096
#  define PKT_LENGTH_WIDTH 12     /* 2048 - 4095 */
#elif CONFIG_SPIRIT_PKTLEN < 8192
#  define PKT_LENGTH_WIDTH 13     /* 4096 - 8191 */
#elif CONFIG_SPIRIT_PKTLEN < 16384
#  define PKT_LENGTH_WIDTH 14     /* 8192 - 16383 */
#elif CONFIG_SPIRIT_PKTLEN < 32768
#  define PKT_LENGTH_WIDTH 15     /* 16384 - 32767 */
#elif CONFIG_SPIRIT_PKTLEN < 65536
#  define PKT_LENGTH_WIDTH 16     /* 32768 - 65535 */
#else
#  error Invalid CONFIG_SPIRIT_PKTLEN
#endif

/* Default node address */

#define SPIRIT_NODE_ADDR    0x34

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define SPIRIT_WDDELAY      (1*CLK_TCK)

/* TX timeout = 5 seconds */

#define SPIRIT_TXTIMEOUT    (5*CLK_TCK)

/* RX timeout = 1.5 seconds */

#define SPIRIT_RXTIMEOUT    1500.0

/* Return values from spirit_transmit() */

#define SPIRIT_TX_IDLE      0
#define SPIRIT_TX_INFLIGHT  1

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
  struct sixlowpan_driver_s        radio;      /* Interface understood by the network */
  struct spirit_library_s          spirit;    /* Spirit library state */
  FAR const struct spirit_lower_s *lower;     /* Low-level MCU-specific support */
  FAR struct pktradio_metadata_s  *txhead;    /* Head of pending TX transfers */
  FAR struct pktradio_metadata_s  *txtail;    /* Tail of pending TX transfers */
  FAR struct pktradio_metadata_s  *rxhead;    /* Head of completed RX transfers */
  FAR struct pktradio_metadata_s  *rxtail;    /* Tail of completed RX transfers */
  struct work_s                    irqwork;   /* Interrupt continuation work queue support */
  struct work_s                    txwork;    /* TX / Network poll work queue support */
  struct work_s                    rxwork;    /* RX work queue support */
  WDOG_ID                          txpoll;    /* TX poll timer */
  WDOG_ID                          txtimeout; /* TX timeout timer */
  sem_t                            exclsem;   /* Mutually exclusive access */
  bool                             ifup;      /* Spirit is on and interface is up */
  bool                             needpoll;  /* Timer poll needed */
  uint8_t                          state;     /* See  enum spirit_driver_state_e */
  uint8_t                          counter;   /* Count used with TX timeout */
  uint8_t                          prescaler; /* Prescaler used with TX timeout */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void spirit_lock(FAR struct spirit_driver_s *priv);
static inline void spirit_unlock(FAR struct spirit_driver_s *priv);

static void spirit_set_ipaddress(FAR struct net_driver_s *dev);
static int  spirit_set_readystate(FAR struct spirit_driver_s *priv);

/* TX-related logic */

static int  spirit_transmit(FAR struct spirit_driver_s *priv);
static void spirit_transmit_work(FAR void *arg);
static void spirit_schedule_transmit_work(FAR struct spirit_driver_s *priv);

static int  spirit_txpoll_callback(FAR struct net_driver_s *dev);

/* RX-related logic */

static void spirit_receive_work(FAR void *arg);
static void spirit_schedule_receive_work(FAR struct spirit_driver_s *priv);

/* Interrupt handling */

static void spirit_interrupt_work(FAR void *arg);
static int  spirit_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void spirit_txtimeout_work(FAR void *arg);
static void spirit_txtimeout_expiry(int argc, wdparm_t arg, ...);

static void spirit_txpoll_work(FAR void *arg);
static void spirit_txpoll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  spirit_ifup(FAR struct net_driver_s *dev);
static int  spirit_ifdown(FAR struct net_driver_s *dev);
static int  spirit_txavail(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_IGMP
static int  spirit_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
static int  spirit_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int  spirit_ioctl(FAR struct net_driver_s *dev, int cmd,
            unsigned long arg);
#endif

static int spirit_get_mhrlen(FAR struct sixlowpan_driver_s *netdev,
            FAR const void *meta);
static int spirit_req_data(FAR struct sixlowpan_driver_s *netdev,
            FAR const void *meta, FAR struct iob_s *framelist);
static int spirit_properties(FAR struct sixlowpan_driver_s *netdev,
            FAR struct sixlowpan_properties_s *properties);

/* Initialization */

int spirit_hw_initialize(FAR struct spirit_driver_s *dev,
            FAR struct spi_dev_s *spi);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Spirit radio initialization */

static const struct radio_init_s g_radio_init =
{
  SPIRIT_BASE_FREQUENCY,              /* base_frequency selected in board.h */
  SPIRIT_CHANNEL_SPACE,               /* chspace        selected in board.h */
  SPIRIT_XTAL_OFFSET_PPM,             /* foffset        selected in board.h */
  SPIRIT_CHANNEL_NUMBER,              /* chnum          selected in board.h */
  SPIRIT_MODULATION_SELECT,           /* modselect      selected in board.h */
  SPIRIT_DATARATE,                    /* datarate       selected in board.h */
  SPIRIT_FREQ_DEVIATION,              /* freqdev        selected in board.h */
  SPIRIT_BANDWIDTH                    /* bandwidth      selected in board.h */
};

/* Spirit PktSTack initialization */

static const struct spirit_pktstack_init_s g_pktstack_init =
{
  SPIRIT_SYNC_WORD,                   /* syncword       selected in board.h */
  SPIRIT_PREAMBLE_LENGTH,             /* premblen       selected in board.h*/
  SPIRIT_SYNC_LENGTH,                 /* synclen        selected in board.h */
  PKT_LENGTH_VAR,                     /* fixvarlen      variable packet length */
  PKT_LENGTH_WIDTH,                   /* pktlenwidth    from CONFIG_SPIRIT_PKTLEN */
#ifdef CONFIG_SPIRIT_CRCDISABLE
  PKT_NO_CRC,                         /* crcmode        none */
#else
  SPIRIT_CRC_MODE,                    /* crcmode        selected in board.h */
#endif
  SPIRIT_CONTROL_LENGTH,              /* ctrllen        selected in board.h */
  SPIRIT_EN_FEC,                      /* fec            selected in board.h */
  SPIRIT_EN_WHITENING                 /* datawhite      selected in board.h */
 };

/* LLP Configuration */

static const struct spirit_pktstack_llp_s g_llp_init =
{
  S_ENABLE,                           /* autoack */
  S_DISABLE,                          /* piggyback */
  PKT_N_RETX_10                       /* maxretx */
};

/* GPIO Configuration.
 *
 * REVISIT:  Assumes interrupt is on GPIO3.  Might need to be configurable.
 */

static const struct spirit_gpio_init_s g_gpioinit =
{
  SPIRIT_GPIO_3,                      /* gpiopin */
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP, /* gpiomode */
  SPIRIT_GPIO_DIG_OUT_IRQ             /* gpioio */
};

/* CSMA initialization */

static const struct spirit_csma_init_s g_csma_init =
{
  1,                                  /* BU counter seed */
  S_ENABLE,                           /* enable persistent mode */
  TBIT_TIME_64,                       /* Tcca time */
  TCCA_TIME_3,                        /* Lcca length */
  3,                                  /* max nr of backoffs (<8) */
  8                                   /* BU prescaler */
};

#ifdef CONFIG_SPIRIT_PROMISICUOUS
static struct spirit_pktstack_address_s g_addrinit =
{
  S_DISABLE,                          /* Disable filtering on node address */
  SPIRIT_NODE_ADDR,                   /* Node address (Temporary, until assigned) */
  S_DISABLE,                          /* Disable filtering on multicast address */
  SPIRIT_MCAST_ADDRESS,               /* Multicast address */
  S_DISABLE,                          /* Disable filtering on broadcast address */
  SPIRIT_BCAST_ADDRESS                /* Broadcast address */
};
#else
static struct spirit_pktstack_address_s g_addrinit =
{
  S_ENABLE,                           /* Enable filtering on node address */
  SPIRIT_NODE_ADDR,                   /* Node address (Temporary, until assigned) */
#ifdef CONFIG_SPIRIT_MULTICAST
  S_ENABLE,                           /* Enable filtering on multicast address */
#else
  S_DISABLE,                          /* Disable filtering on multicast address */
#endif
  SPIRIT_MCAST_ADDRESS,               /* Multicast address */
#ifdef CONFIG_SPIRIT_BROADCAST
  S_ENABLE,                           /* Enable filtering on broadcast address */
#else
  S_DISABLE,                          /* Disable filtering on broadcast address */
#endif
  SPIRIT_BCAST_ADDRESS                /* Broadcast address */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spirit_lock
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
 * Name: spirit_unlock
 *
 * Description:
 *   Relinquish exclusive access to the driver instance and to the spirit
 *   library.
 *
 * Parameters:
 *   priv - Reference to a driver state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spirit_unlock(FAR struct spirit_driver_s *priv)
{
  sem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: spirit_set_ipaddress
 *
 * Description:
 *   Set the advertised node addressing.  External logic must set a unique
 *   8-bit node-address for the radio.  We will then derive the IPv6
 *   address for that.
 *
 * Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   OK on success; a negated errno on a timeout
 *
 ****************************************************************************/

static void spirit_set_ipaddress(FAR struct net_driver_s *dev)
{
  FAR struct netdev_varaddr_s *addr;

  /* Get a convenient pointer to the PktRadio variable length address struct */

  addr = (FAR struct netdev_varaddr_s *)&dev->d_mac.sixlowpan;

  /* Has a node address been assigned? */

  if (addr->nv_addrlen == 0)
    {
      /* No.. Use the default address */

      wlwarn("WARNING: No address assigned.  Using %02x\n",
             SPIRIT_NODE_ADDR);

      addr->nv_addrlen = 1;
      addr->nv_addr[0] = SPIRIT_NODE_ADDR;
    }

  /* Then set the IP address derived from the node address */

  dev->d_ipv6addr[0]  = HTONS(0xfe80);
  dev->d_ipv6addr[1]  = 0;
  dev->d_ipv6addr[2]  = 0;
  dev->d_ipv6addr[3]  = 0;
  dev->d_ipv6addr[4]  = 0;
  dev->d_ipv6addr[5]  = HTONS(0x00ff);
  dev->d_ipv6addr[6]  = HTONS(0xfe00);
  dev->d_ipv6addr[7]  = (uint16_t)addr->nv_addr[0] << 8 ^ 0x0200;
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
 *   0 - on success with nothing to be sent (SPIRIT_TX_IDLE)
 *   1 - on success if a packet is in-flight (SPIRIT_TX_INFLIGHT)
 *  <0 - a negated errno on failure
 *
 ****************************************************************************/

static int spirit_transmit(FAR struct spirit_driver_s *priv)
{
  FAR struct spirit_library_s *spirit = &priv->spirit;
  FAR struct pktradio_metadata_s *pktmeta;
  FAR struct iob_s *iob;
  int ret;

  /* Check if there are any pending transfers in the TX AND if the hardware
   * is not busy with another reception or transmission.
   */

  spirit_lock(priv);
  wlinfo("txhead=%p state=%u\n", priv->txhead, priv->state);

  while (priv->txhead != NULL && priv->state == DRIVER_STATE_IDLE)
    {
      /* Remove the contained IOB from the head of the TX queue */

      pktmeta      = priv->txhead;
      priv->txhead = pktmeta->pm_flink;

      if (priv->txhead == NULL)
        {
          priv->txtail = NULL;
        }

      /* Remove the IOB from metadata container */

      iob             = pktmeta->pm_iob;
      pktmeta->pm_iob = NULL;

      DEBUGASSERT(iob != NULL);

      /* Checks if the payload length is supported */

      if (iob->io_len > CONFIG_SPIRIT_PKTLEN)
        {
          NETDEV_RXDROPPED(&priv->radio.r_dev);
          iob_free(iob);
          continue;
        }

      priv->state = DRIVER_STATE_SENDING;

      /* Reset state to ready */

      ret = spirit_set_readystate(priv);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to set READY state: %d\n", ret);
          goto errout_with_iob;
        }

      /* Make sure that the TX linear FIFO is completely empty */

      ret = spirit_command(spirit, COMMAND_FLUSHTXFIFO);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to flush TX FIFO\n");
          goto errout_with_iob;
        }

      /* Sets the length of the packet to send */

      wlinfo("Payload length=%u\n", iob->io_len);

      ret = spirit_pktstack_set_payloadlen(spirit, iob->io_len);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to set payload length: %d\n", ret);
          goto errout_with_iob;
        }

      /* Set the destination address */

      DEBUGASSERT(pktmeta->pm_dest.pa_addrlen == 1);
      wlinfo("txdestaddr=%02x\n", pktmeta->pm_dest.pa_addr[0]);

      ret = spirit_pktcommon_set_txdestaddr(spirit,
              pktmeta->pm_dest.pa_addr[0]);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to set TX destaddr to %02x: %d\n",
                pktmeta->pm_dest.pa_addr[0], ret);
          goto errout_with_iob;
        }

      /* Enable CSMA */

      wlinfo("Enable CSMA and send packet\n");

      ret = spirit_csma_enable(spirit, S_ENABLE);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to enable CSMA: %d\n", ret);
          goto errout_with_iob;
        }

      /* Write the packet to the linear FIFO */

      ret = spirit_fifo_write(spirit, iob->io_data, iob->io_len);
      if (ret < 0)
        {
          wlerr("ERROR: Write to linear FIFO failed: %d\n", ret);
          goto errout_with_iob;
        }

      /* We can free the IOB now */

      iob_free(iob);

      /* Start the RX timeout */

       ret = spirit_timer_setup_rxtimeout(spirit, priv->counter,
                                          priv->prescaler);
      if (ret < 0)
        {
          wlerr("ERROR: spirit_timer_setup_rxtimeout failed: %d\n", ret);
          goto errout_with_iob;
        }

      /* Put the SPIRIT1 into TX state.  This starts the transmission */

      ret = spirit_command(spirit, COMMAND_TX);
      if (ret < 0)
        {
          wlerr("ERROR: Write to send TX command: %d\n", ret);
          goto errout_with_lock;
        }

      /* Wait until we have successfully entered the TX state */

      ret = spirit_waitstatus(spirit, MC_STATE_TX, 5);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to go to TX state: %d\n", ret);
          goto errout_with_lock;
        }

      /* Setup the TX timeout watchdog (perhaps restarting the timer) */

      (void)wd_start(priv->txtimeout, SPIRIT_TXTIMEOUT,
                     spirit_txtimeout_expiry, 1, (wdparm_t)priv);
    }

  /* Return 0 or 1, depending upon if the Spirit is IDLE or is in the TX (or
   * possibly RX) states.
   */

  ret = (priv->state == DRIVER_STATE_IDLE) ? SPIRIT_TX_IDLE : SPIRIT_TX_INFLIGHT;
  spirit_unlock(priv);
  return ret;

errout_with_iob:
  iob_free(iob);

errout_with_lock:
  spirit_unlock(priv);
  NETDEV_TXERRORS(&priv->radio.r_dev);
  return ret;
}

/****************************************************************************
 * Name: spirit_transmit_work
 *
 * Description:
 *   Send data on the LP work queue.  This function scheduled by interrupt
 *   handling logic when a TX transfer completes and, more generally, on
 *   all transitions to the IDLE state.
 *
 * Parameters:
 *   arg - Reference to driver state structure (cast to void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spirit_transmit_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* If the driver is IDLE and there are packets to be sent, then send them
   * now.  This will cause a transition to the DRIVER_STATE_SENDING state.
   */

  ret = spirit_transmit(priv);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_transmit failed: %d\n", ret);
    }
  else if (ret == SPIRIT_TX_IDLE)
    {
      /* Nothing was sent.  Try, instead, to poll for new TX data,  We do
       * this here because TX and TX poll share the sam work structure so
       * the receipt of TX data could caue the loss of polls.
       */

      spirit_txpoll_work(arg);
    }
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
  if (priv->txhead != NULL && priv->state == DRIVER_STATE_IDLE)
    {
      /* Schedule to perform the TX processing on the worker thread. */

      work_queue(LPWORK, &priv->txwork, spirit_transmit_work, priv, 0);
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
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)dev->d_private;
  int ret;

  /* NOTE: It is not necessary to call spirit_transmit because that works
   * though different mechanism, through a backdoor 6LoWPAN interface.  We
   * call it here only to may sure that it has not stalled for some reason.
   */

  ret = spirit_transmit(priv);

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   *
   * REVISIT:  Should we halt polling if there are packets in flight.
   */

  UNUSED(ret);
  return 0;
}

/****************************************************************************
 * Name: spirit_receive_work
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

static void spirit_receive_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;
  FAR struct pktradio_metadata_s *pktmeta;
  FAR struct iob_s *iob;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* We need to have exclusive access to the RX queue */

  spirit_lock(priv);
  while (priv->rxhead != NULL)
    {
      /* Remove the contained IOB from the RX queue */

      pktmeta           = priv->rxhead;
      priv->rxhead      = pktmeta->pm_flink;
      pktmeta->pm_flink = NULL;

      /* Did the RX queue become empty? */

      if (priv->rxhead == NULL)
        {
          priv->rxtail = NULL;
        }

      spirit_unlock(priv);

      /* Remove the IOB from the container */

      iob             = pktmeta->pm_iob;
      pktmeta->pm_iob = NULL;

      /* Send the next frame to the network */

      wlinfo("Send frame %p to the network:  Offset=%u Length=%u\n",
             iob, iob->io_offset, iob->io_len);

      net_lock();
      ret = sixlowpan_input(&priv->radio, iob, (FAR void *)pktmeta);
      if (ret < 0)
        {
          wlerr("ERROR: sixlowpan_input returned %d\n", ret);
          NETDEV_RXERRORS(&priv->radio.r_dev);
          NETDEV_ERRORS(&priv->radio.r_dev);
        }

      net_unlock();

      /* sixlowpan_input() will free the IOB, but we must free the struct
       * pktradio_metadata_s container here.
       */

      pktradio_metadata_free(pktmeta);

      /* Get exclusive access as needed at the top of the loop */

      spirit_lock(priv);
    }

  spirit_unlock(priv);
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

static inline void spirit_schedule_receive_work(FAR struct spirit_driver_s *priv)
{
  /* Schedule to perform the RX processing on the worker thread. */

  work_queue(LPWORK, &priv->rxwork, spirit_receive_work, priv, 0);
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

  /* Get the set of pending ineterrupts from the radio.
   * NOTE: The pending interrupts are cleared as a side-effect of reading
   * the IRQ status register.
   */

  spirit_lock(priv);
  DEBUGVERIFY(spirit_irq_get_pending(spirit, &irqstatus));
  wlinfo("Pending: %08lx\n", *(FAR unsigned long *)&irqstatus);

  /* Process the Spirit1 interrupt */
  /* First check for errors */

  if (irqstatus.IRQ_RX_FIFO_ERROR != 0)
    {
      wlwarn("WARNING: Rx FIFO Error\n");
      DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));

      priv->state = DRIVER_STATE_IDLE;
      NETDEV_RXERRORS(&priv->radio.r_dev);
      NETDEV_ERRORS(&priv->radio.r_dev);

      /* Send any pending packets */

      DEBUGVERIFY(spirit_timer_set_rxtimeout_counter(spirit, 0));
      DEBUGVERIFY(spirit_csma_enable(spirit, S_DISABLE));
      spirit_schedule_transmit_work(priv);
    }

  if (irqstatus.IRQ_TX_FIFO_ERROR != 0 ||
      irqstatus.IRQ_MAX_RE_TX_REACH != 0)
    {
      wlwarn("WARNING: Tx FIFO Error/Max retries\n");
      DEBUGVERIFY(spirit_command(spirit, COMMAND_FLUSHTXFIFO));

      priv->state = DRIVER_STATE_IDLE;
      NETDEV_TXERRORS(&priv->radio.r_dev);
      NETDEV_ERRORS(&priv->radio.r_dev);

      /* Send any pending packets */

      DEBUGVERIFY(spirit_timer_set_rxtimeout_counter(spirit, 0));
      DEBUGVERIFY(spirit_csma_enable(spirit, S_DISABLE));
      spirit_schedule_transmit_work(priv);
    }

  /* The IRQ_TX_DATA_SENT bit notifies that a packet was sent. */

  if (irqstatus.IRQ_TX_DATA_SENT != 0)
    {
      /* Put the Spirit back in the receiving state */

      wlinfo("Data sent\n");
      DEBUGVERIFY(spirit_management_rxstrobe(spirit));
      DEBUGVERIFY(spirit_command(spirit, CMD_RX));

      NETDEV_TXDONE(&priv->radio.r_dev);
      DEBUGVERIFY(spirit_timer_set_rxtimeout_counter(spirit, 0));
      DEBUGVERIFY(spirit_csma_enable(spirit, S_DISABLE));

      /* Disable the RX timeout */

      DEBUGVERIFY(spirit_timer_set_rxtimeout_counter(spirit, 0));

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
      wlinfo("TX FIFO almost empty\n");
#warning Missing logic
    }
#endif

  /* The IRQ_VALID_SYNC bit is used to notify a new packet is coming */

  if (irqstatus.IRQ_VALID_SYNC != 0)
    {
      wlinfo("Valid sync\n");
      DEBUGASSERT(priv->state == DRIVER_STATE_IDLE);
      priv->state = DRIVER_STATE_RECEIVING;
    }

  /* The IRQ_RX_DATA_READY notifies that a new packet has been received */

  if (irqstatus.IRQ_RX_DATA_READY != 0)
    {
      FAR struct pktradio_metadata_s *pktmeta;
      FAR struct iob_s *iob;
      uint8_t count;

      wlinfo("Data ready\n");
      NETDEV_RXPACKETS(&priv->radio.r_dev);

      /* Check the packet size */

      count = spirit_fifo_get_rxcount(spirit);
      if (count > CONFIG_IOB_BUFSIZE)
        {
          wlwarn("WARNING:  Packet too large... dropping\n");
          DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
          priv->state = DRIVER_STATE_IDLE;
          NETDEV_RXDROPPED(&priv->radio.r_dev);
        }
      else
        {
          wlinfo("Receiving %u bytes\n", count);

          /* Allocate an I/O buffer to hold the received packet.
           * REVISIT: Not a good place to wait.  Perhaps we should pre-
           * allocate a few I/O buffers?
           */

          iob = iob_alloc(0);
          if (iob == NULL)
            {
              wlerr("ERROR: Failed to allocate IOB... dropping\n");

              DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
              priv->state = DRIVER_STATE_IDLE;
              NETDEV_RXDROPPED(&priv->radio.r_dev);

              /* Send any pending packets */

              spirit_schedule_transmit_work(priv);
            }
          else
            {
              /* Read the packet into the I/O buffer */

              DEBUGVERIFY(spirit_fifo_read(spirit, iob->io_data, count));
              iob->io_len    = spirit_pktstack_get_rxpktlen(spirit);
              iob->io_offset = 0;
              iob->io_pktlen = iob->io_len;
              iob->io_flink  = NULL;

              DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
              priv->state = DRIVER_STATE_IDLE;

              /* Create the packet meta data and forward to the network.  This
               * must be done on the LP work queue with the network lockes.
               */

               pktmeta = pktradio_metadata_allocate();
               if (pktmeta == NULL)
                 {
                   wlerr("ERROR: Failed to allocate metadata... dropping\n");
                   NETDEV_RXDROPPED(&priv->radio.r_dev);
                   iob_free(iob);
                 }
               else
                 {
                   /* Get the packet meta data.  This consists only of the
                    * source and destination addresses.
                    */

                  pktmeta->pm_iob             = iob;

                  pktmeta->pm_src.pa_addrlen  = 1;
                  pktmeta->pm_src.pa_addr[0]  =
                    spirit_pktcommon_get_rxsrcaddr(spirit);

                  pktmeta->pm_dest.pa_addrlen = 1;
                  pktmeta->pm_dest.pa_addr[0] =
                    spirit_pktcommon_get_nodeaddress(spirit);

                  wlinfo("RX srcaddr=%02x destaddr=%02x\n",
                         pktmeta->pm_src.pa_addr[0],
                         pktmeta->pm_dest.pa_addr[0]);

                  /* Add the contained IOB to the tail of the queue of
                   * completed RX transfers.
                   */

                  pktmeta->pm_flink          = NULL;
                  if (priv->rxtail == NULL)
                    {
                      priv->rxhead           = pktmeta;
                    }
                  else
                    {
                      priv->rxtail->pm_flink = pktmeta;
                    }

                  priv->rxtail               = pktmeta;

                  /* Forward the packet to the network.  This must be done
                   * on the LP work queue with the network locked.
                   */

                  spirit_schedule_receive_work(priv);
                }
            }

          /* Send any pending packets */

          spirit_schedule_transmit_work(priv);
        }
    }

#ifdef CONFIG_SPIRIT_FIFOS
  /* The IRQ_RX_FIFO_ALMOST_FULL notifies an nearly full RX fifo.
   * Necessary for receiving large packets > sizeof(RX FIFO).
   */

  if (irqstatus.IRQ_RX_FIFO_ALMOST_FULL != 0)
    {
      wlinfo("RX FIFO almost full\n");
#warning Missing logic
    }
#endif

  /* IRQ_RX_DATA_DISC indicates that Rx data was discarded */

  if (irqstatus.IRQ_RX_DATA_DISC != 0)
    {
      wlinfo("Data discarded: Node addr=%02x RX dest addr=%02x\n",
             spirit_pktcommon_get_nodeaddress(spirit),
             spirit_pktcommon_get_rxdestaddr(spirit));
      wlinfo("                CRC error=%u RX timeout=%u\n",
             irqstatus.IRQ_CRC_ERROR, irqstatus.IRQ_RX_TIMEOUT);

      DEBUGVERIFY(spirit_command(spirit, CMD_FLUSHRXFIFO));
      priv->state = DRIVER_STATE_IDLE;
      NETDEV_RXDROPPED(&priv->radio.r_dev);
    }

  /* Check the Spirit status.  If it is READY, the setup the RX state */

  DEBUGVERIFY(spirit_update_status(spirit));
  wlinfo("MC_STATE=%02x\n", spirit->u.state.MC_STATE);

  if (spirit->u.state.MC_STATE == MC_STATE_READY)
    {
      wlinfo("Go to RX state (%02x)\n", MC_STATE_RX);

      /* Set up to receive */

      DEBUGVERIFY(spirit_command(spirit, CMD_RX));

      /* Wait for Spirit to enter the Rx state (or timeut) */

      DEBUGVERIFY(spirit_waitstatus(spirit, MC_STATE_RX, 1));
    }

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
   */

  return work_queue(HPWORK, &priv->irqwork, spirit_interrupt_work,
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

  NETDEV_TXTIMEOUTS(&priv->radio.r_dev);

  /* Then reset the hardware */

  spirit_ifdown(&priv->radio.r_dev);
  spirit_ifup(&priv->radio.r_dev);

  /* Then poll the network for new XMIT data */

  (void)devif_poll(&priv->radio.r_dev, spirit_txpoll_callback);
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

  work_queue(HPWORK, &priv->irqwork, spirit_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: spirit_txpoll_work
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

static void spirit_txpoll_work(FAR void *arg)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Do nothing if the network is not yet UP */

  if (!priv->ifup)
    {
      priv->needpoll = false;
    }

  /* Skip the poll if the Spirit busy in the TX or RX states.  In this case,
   * spirit_transmit_work will run when the state transitions back to IDLE
   * we will perform the poll at that time.
   */

  else if (priv->state == DRIVER_STATE_IDLE)
    {
      /* Is a periodic poll needed? */

       if (priv->needpoll)
        {
          /* Perform the periodic poll */

          priv->needpoll = false;
          (void)devif_timer(&priv->radio.r_dev, spirit_txpoll_callback);

          /* Setup the watchdog poll timer again */

          (void)wd_start(priv->txpoll, SPIRIT_WDDELAY, spirit_txpoll_expiry, 1,
                         (wdparm_t)priv);
        }
      else
        {
          /* Perform a normal, asynchronous poll for new TX data */

          (void)devif_poll(&priv->radio.r_dev, spirit_txpoll_callback);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: spirit_txpoll_expiry
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

static void spirit_txpoll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct spirit_driver_s *priv = (FAR struct spirit_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  priv->needpoll = true;
  work_queue(LPWORK, &priv->txwork, spirit_txpoll_work, priv, 0);
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
      /* Set the node IP address based on the assigned 8-bit node address */

      spirit_set_ipaddress(dev);

      wlinfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
             dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
             dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
             dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

      /* Disable spirit interrupts */

      DEBUGASSERT(priv->lower->enable != NULL);
      priv->lower->enable(priv->lower, false);

      /* Ensure we are in READY state before we go from there to Rx.
       * Since spirit interrupts are disabled, we don't need to be concerned
       * about mutual exclusion.
       */

      wlinfo("Go to the ready state\n");
      ret = spirit_command(spirit, CMD_READY);
      if (ret < 0)
        {
          return ret;
        }

      ret = spirit_waitstatus(spirit, MC_STATE_READY, 5);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to go to READY state: %d\n", ret);
          goto error_with_ifalmostup;
        }

      /* Now we go to Rx */

      wlinfo("Go to RX state (%02x)\n", MC_STATE_RX);

      ret = spirit_command(spirit, CMD_RX);
      if (ret < 0)
        {
          goto error_with_ifalmostup;
        }

      ret = spirit_waitstatus(spirit, MC_STATE_RX, 5);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to go to RX state: %d\n", ret);
          goto error_with_ifalmostup;
        }

      /* Instantiate the assigned node address in hardware */

      DEBUGASSERT(dev->d_mac.sixlowpan.nv_addrlen == 1);
      wlinfo("Set node address to %02x\n",
              dev->d_mac.sixlowpan.nv_addr[0]);

      ret = spirit_pktcommon_set_nodeaddress(spirit,
               dev->d_mac.sixlowpan.nv_addr[0]);
      if (ret < 0)
        {
          wlerr("ERROR: Failed to set node address: %d\n", ret);
          goto error_with_ifalmostup;
        }

      /* Set and activate a timer process */

      (void)wd_start(priv->txpoll, SPIRIT_WDDELAY, spirit_txpoll_expiry, 1,
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
          wlerr("ERROR: Failed to go to READY state: %d\n", ret);
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
          wlerr("ERROR: Failed to go to STANBY state: %d\n", ret);
          ret = status;
        }

      priv->ifup = false;
    }

  return ret;
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
  if (work_available(&priv->txwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPWORK, &priv->txwork, spirit_txpoll_work, priv, 0);
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

#ifdef CONFIG_NET_IGMP
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
  FAR struct pktradio_ifreq_s *cmddata;
  FAR struct spirit_driver_s *priv;
  int ret = -ENOTTY;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL && arg != 0ul);
  priv    = (FAR struct spirit_driver_s *)dev->d_private;
  cmddata = (FAR struct pktradio_ifreq_s *)((uintptr_t)arg);

  spirit_lock(priv);
  switch (cmd)
    {
      /* SIOCPKTRADIOGGPROPS
       *   Description:   Get the radio properties
       *   Input:         Pointer to read-write instance of struct
       *                  pktradio_ifreq_s
       *   Output:        Properties returned in struct pktradio_ifreq_s
       *                  instance
       */

      case SIOCPKTRADIOGGPROPS:
        {
          FAR struct sixlowpan_driver_s *radio =
            (FAR struct sixlowpan_driver_s *)dev;
          FAR struct sixlowpan_properties_s *props =
            (FAR struct sixlowpan_properties_s *)&cmddata->pifr_props;

          ret = spirit_properties(radio, props);
        }
        break;

      /* SIOCPKTRADIOSNODE
       *   Description:   Set the radio node address
       *   Input:         Pointer to read-only instance of struct
       *                  pktradio_ifreq_s
       *   Output:        None
       */

      case SIOCPKTRADIOSNODE:
        {
          FAR const struct pktradio_addr_s *newaddr =
            (FAR const struct pktradio_addr_s *)&cmddata->pifr_hwaddr;

          if (newaddr->pa_addrlen != 1)
            {
              ret = -EINVAL;
            }
          else
            {
              FAR struct netdev_varaddr_s *devaddr = &dev->d_mac.sixlowpan;

              devaddr->nv_addrlen = 1;
              devaddr->nv_addr[0] = newaddr->pa_addr[0];
#if CONFIG_PKTRADIO_ADDRLEN > 1
              memset(&devaddr->pa_addr[1], 0, CONFIG_PKTRADIO_ADDRLEN - 1);
#endif
              ret = OK;
            }
        }
        break;

      /* SIOCPKTRADIOGNODE
       *   Description:   Get the radio node address
       *   Input:         Pointer to read-write instance of
       *                  struct pktradio_ifreq_s
       *   Output:        Node address return in struct pktradio_ifreq_s
       *                  instance
       */

      case SIOCPKTRADIOGNODE:
        {
          FAR struct pktradio_addr_s *retaddr =
            (FAR struct pktradio_addr_s *)&cmddata->pifr_hwaddr;
          FAR struct netdev_varaddr_s *devaddr = &dev->d_mac.sixlowpan;

          retaddr->pa_addrlen = devaddr->nv_addrlen;
          retaddr->pa_addr[0] = devaddr->nv_addr[0];
#if CONFIG_PKTRADIO_ADDRLEN > 1
          memset(&addr->pa_addr[1], 0, CONFIG_PKTRADIO_ADDRLEN - 1);
#endif
          ret = OK;
        }
        break;

      default:
        wlwarn("WARNING: Unrecognized IOCTL command: %02x\n", cmd);
        break;
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
 *   netdev    - The network device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Called from network logic running on the low-priority work queue.
 *
 ****************************************************************************/

static int spirit_get_mhrlen(FAR struct sixlowpan_driver_s *netdev,
                             FAR const void *meta)
{
  DEBUGASSERT(netdev != NULL && netdev->r_dev.d_private != NULL && meta != NULL);

  /* There is no header on the Spirit radio payload */

  return 0;
}

/****************************************************************************
 * Name: spirit_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.  We get here
 *   indirectly as a consequence of a TX poll that generates a series of
 *   6LoWPAN radio packets.
 *
 * Input parameters:
 *   netdev    - The network device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
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

static int spirit_req_data(FAR struct sixlowpan_driver_s *netdev,
                           FAR const void *meta, FAR struct iob_s *framelist)
{
  FAR struct spirit_driver_s *priv;
  FAR const struct pktradio_metadata_s *metain;
  FAR struct pktradio_metadata_s *pktmeta;
  FAR struct iob_s *iob;

  wlinfo("Received framelist\n");

  DEBUGASSERT(netdev != NULL && netdev->r_dev.d_private != NULL);
  priv = (FAR struct spirit_driver_s *)netdev->r_dev.d_private;

  DEBUGASSERT(meta != NULL && framelist != NULL);
  metain = (FAR const struct pktradio_metadata_s *)meta;

  /* Add the incoming list of frames to the MAC's outgoing queue */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      spirit_lock(priv);
      NETDEV_TXPACKETS(&priv->radio.r_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      /* Note that there is no header applied to the outgoing payload */

      DEBUGASSERT(iob->io_offset == 0 && iob->io_len > 0);

      /* Allocate a metadata container to hold the IOB.  This is just like
       * the we got from the network, be we will copy it so that we can
       * control the life of the container.
       *
       * REVISIT:  To bad we cound not just simply allocate the structure
       * on the network side.  But that behavior is incompatible with how
       * IEEE 802.15.4 works.
       */

       pktmeta = pktradio_metadata_allocate();
       if (pktmeta == NULL)
         {
           wlerr("ERROR: Failed to allocate metadata... dropping\n");
           NETDEV_RXDROPPED(&priv->radio.r_dev);
           spirit_unlock(priv);
           iob_free(iob);
           continue;
         }

       /* Save the IOB and addressing information in the newly allocated
        * container.
        */

       memcpy(&pktmeta->pm_src, &metain->pm_src,
              sizeof(struct pktradio_addr_s));
       memcpy(&pktmeta->pm_dest, &metain->pm_dest,
              sizeof(struct pktradio_addr_s));
       pktmeta->pm_iob  = iob;

      /* Add the IOB container to tail of the queue of outgoing IOBs. */

      pktmeta->pm_flink          = NULL;
      if (priv->txtail == NULL)
        {
          priv->txhead           = pktmeta;
        }
      else
        {
          priv->txtail->pm_flink = pktmeta;
        }

      priv->txtail               = pktmeta;

      /* If there are no transmissions or receptions in progress, then start
       * tranmission of the frame in the IOB at the head of the IOB queue.
       */

       spirit_unlock(priv);
       spirit_transmit(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: spirit_properties
 *
 * Description:
 *   Different packet radios may have different properties.  If there are
 *   multiple packet radios, then those properties have to be queried at
 *   run time.  This information is provided to the 6LoWPAN network via the
 *   following structure.
 *
 * Input parameters:
 *   netdev     - The network device to be queried
 *   properties - Location where radio properities will be returned.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spirit_properties(FAR struct sixlowpan_driver_s *netdev,
                             FAR struct sixlowpan_properties_s *properties)
{
  DEBUGASSERT(netdev != NULL && properties != NULL);
  memset(properties, 0, sizeof(struct sixlowpan_properties_s));

  /* General */

  properties->sp_addrlen = 1;                    /* Length of an address */
  properties->sp_pktlen  = CONFIG_SPIRIT_PKTLEN; /* Fixed packet length */

  /* Multicast address */

  properties->sp_mcast.nv_addrlen = 1;
  properties->sp_mcast.nv_addr[0] = SPIRIT_MCAST_ADDRESS;

  /* Broadcast address */

  properties->sp_bcast.nv_addrlen = 1;
  properties->sp_bcast.nv_addr[0] = SPIRIT_BCAST_ADDRESS;
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

  wlinfo("Initialize spirit hardware\n");

  /* Configures the Spirit1 radio library */

  spirit->spi            = spi;
  spirit->xtal_frequency = SPIRIT_XTAL_FREQUENCY;

  priv->ifup = false;

  /* Reset the Spirit1 radio part */

  wlinfo("Spirit Reset\n");
  DEBUGASSERT(priv->lower != NULL && priv->lower->reset != NULL);
  ret = priv->lower->reset(priv->lower) ;
  if (ret < 0)
    {
      wlerr("ERROR: SDN reset failed: %d\n", ret);
      return ret;
    }

  /* Soft reset of Spirit1 core */

#if 0
  ret = spirit_command(spirit, COMMAND_SRES);
  if (ret < 0)
    {
      wlerr("ERROR: Soft reset failed: %d\n", ret);
      return ret;
    }
#endif

  ret = spirit_waitstatus(spirit, MC_STATE_READY, 100);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to go to READY state: %d\n", ret);
      return ret;
    }

  /* Perform VCO calibration WA when the radio is initialized */

  wlinfo("Peform VCO calibration\n");
  spirit_radio_enable_wavco_calibration(spirit, S_ENABLE);

  /* Configure the Spirit1 radio part */

  wlinfo("Configure the spirit radio\n");
  ret = spirit_radio_initialize(spirit, &g_radio_init);
  if (ret < 0)
    {
      wlerr("ERROR: pirit_radio_initialize failed: %d\n", ret);
      return ret;
    }

  ret = spirit_radio_set_palevel_dbm(spirit, 0, SPIRIT_POWER_DBM);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_radio_set_palevel_dbm failed: %d\n", ret);
      return ret;
    }

  ret =spirit_radio_set_palevel_maxindex(spirit, 0);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_radio_set_palevel_maxindex failed: %d\n", ret);
      return ret;
    }

  /* Configures the SPIRIT1 packet handling logic */

  wlinfo("Configure STack packets\n");
  ret = spirit_pktstack_initialize(spirit, &g_pktstack_init);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_radio_set_palevel_maxindex failed: %d\n", ret);
      return ret;
    }

  ret = spirit_pktstack_llp_initialize(spirit, &g_llp_init);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_pktstack_llp_initialize failed: %d\n", ret);
      return ret;
    }

  /* Configure address filtering */

  wlinfo("Configure address filtering\n");
  ret = spirit_pktstack_address_initialize(spirit, &g_addrinit);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_pktstack_address_initialize failed: %d\n", ret);
      return ret;
    }

  /* Enable the following interrupt sources, routed to GPIO */

  wlinfo("Configure Interrupts\n");
  ret = spirit_irq_disable_all(spirit);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_irq_disable_all failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_clr_pending(spirit);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_irq_clr_pending failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, TX_DATA_SENT, S_ENABLE);
   if (ret < 0)
    {
      wlerr("ERROR: Enable TX_DATA_SENT failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_DATA_READY, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable RX_DATA_READY failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, VALID_SYNC, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable VALID_SYNC failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_DATA_DISC, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable RX_DATA_DISC failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, MAX_RE_TX_REACH, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable MAX_RE_TX_REACH failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, TX_FIFO_ERROR, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable TX_FIFO_ERROR failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_FIFO_ERROR, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable RX_FIFO_ERROR failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_SPIRIT_FIFOS
  ret = spirit_irq_enable(spirit, TX_FIFO_ALMOST_EMPTY, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable TX_FIFO_ALMOST_EMPTY failed: %d\n", ret);
      return ret;
    }

  ret = spirit_irq_enable(spirit, RX_FIFO_ALMOST_FULL, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: Enable RX_FIFO_ALMOST_FULL failed: %d\n", ret);
      return ret;
    }
#endif

  /* Configure Spirit1 */

  wlinfo("Configure Spriti1\n");
  ret = spirit_radio_persistentrx(spirit, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_radio_persistentrx failed: %d\n", ret);
      return ret;
    }

  ret = spirit_qi_set_sqithreshold(spirit, SQI_TH_0);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_qi_set_sqithreshold failed: %d\n", ret);
      return ret;
    }

  ret = spirit_qi_enable_sqicheck(spirit, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_qi_enable_sqicheck failed: %d\n", ret);
      return ret;
    }

  ret = spirit_qi_set_rssithreshold_dbm(spirit, SPIRIT_CCA_THRESHOLD);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_qi_set_rssithreshold_dbm failed: %d\n", ret);
      return ret;
    }

  /* Initialize counter and prescaler values that will be used with
   * with RX timeouts for ACKs.
   */

  spirit_timer_calc_rxtimeout_values(spirit, SPIRIT_RXTIMEOUT,
                                     &priv->counter, &priv->prescaler);

  ret = spirit_timer_set_rxtimeout_stopcondition(spirit,
                                                 SQI_ABOVE_THRESHOLD);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_timer_set_rxtimeout_stopcondition failed: %d\n", ret);
      return ret;
    }

  ret = spirit_timer_set_rxtimeout_counter(spirit, 0); /* 0=Disables timeout */
  if (ret < 0)
    {
      wlerr("ERROR: spirit_timer_set_rxtimeout_counter failed: %d\n", ret);
      return ret;
    }

  ret = spirit_radio_afcfreezeonsync(spirit, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_radio_afcfreezeonsync failed: %d\n", ret);
      return ret;
    }

#if 0
  ret = spirit_calibration_rco(spirit, S_ENABLE);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_calibration_rco failed: %d\n", ret);
      return ret;
    }
#endif

  /* Configure the radio to route the IRQ signal to its GPIO 3 */

  wlinfo("Configure Spirt GPIOs\n");
  ret = spirit_gpio_initialize(spirit, &g_gpioinit);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_gpio_initialize failed: %d\n", ret);
      return ret;
    }

  /* Setup CSMA/CA */

  wlinfo("Configure Spirt CSMA\n");
  ret = spirit_csma_initialize(spirit, &g_csma_init);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_csma_initialize failed: %d\n", ret);
      return ret;
    }

  /* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */

  wlinfo("Go to STANDBY\n");
  ret = spirit_command(spirit, CMD_STANDBY);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to go to STANDBY: %d\n", ret);
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
  FAR struct sixlowpan_driver_s *radio;
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

  pktbuf = (uint8_t *)kmm_zalloc(CONFIG_NET_6LOWPAN_MTU + CONFIG_NET_GUARDSIZE);
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

  radio               = &priv->radio;
  radio->r_get_mhrlen = spirit_get_mhrlen; /* Get MAC header length */
  radio->r_req_data   = spirit_req_data;   /* Enqueue frame for transmission */
  radio->r_properties = spirit_properties; /* Return radio properties */

  /* Initialize the common network device fields */

  dev                 = &radio->r_dev;
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

  /* Make sure that the PktRadio common logic has been initialized */

  pktradio_metadata_initialize();

  /* Initialize device */

  ret = spirit_hw_initialize(priv, spi);
  if (ret < 0)
    {
      wlerr("ERROR: spirit_hw_initialize failed: %d\n", ret);
      goto errout_with_attach;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed. */

  ret = netdev_register(dev, NET_LL_PKTRADIO);
  if (ret < 0)
    {
      wlerr("ERROR: netdev_register failed: %d\n", ret);
      goto errout_with_attach;
    }

  /* Attach irq */

  ret = lower->attach(lower, spirit_interrupt, priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to attach interrupt: %d\n", ret);
      goto errout_with_pktbuf;
    }

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
