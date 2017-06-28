/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40.c
 *
 *   Copyright (C) 2015-2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mrf24j40.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_SPIMODE
#  define CONFIG_IEEE802154_MRF24J40_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_FREQUENCY
#  define CONFIG_IEEE802154_MRF24J40_FREQUENCY 8000000
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

/* Definitions for the device structure */

#define MRF24J40_RXMODE_NORMAL  0
#define MRF24J40_RXMODE_PROMISC 1
#define MRF24J40_RXMODE_NOCRC   2

#define MRF24J40_MODE_DEVICE    0
#define MRF24J40_MODE_COORD     1
#define MRF24J40_MODE_PANCOORD  2

/* Definitions for PA control on high power modules */

#define MRF24J40_PA_AUTO   1
#define MRF24J40_PA_ED     2
#define MRF24J40_PA_SLEEP  3

#define MRF24J40_GTS_SLOTS 2

/* Clock configuration macros */

#define MRF24J40_SLPCLKPER_100KHZ ((1000 * 1000 * 1000)/100000) /* 10ns */
#define MRF24J40_SLPCLKPER_32KHZ  ((1000 * 1000 * 1000)/32000)  /* 31.25ns */

#define MRF24J40_BEACONINTERVAL_NSEC(beaconorder) \
  (IEEE802154_BASE_SUPERFRAME_DURATION * (1 << beaconorder) * (16 *1000))

/* For now I am just setting the REMCNT to the maximum while staying in multiples
 * of 10000 (100khz period) */

#define MRF24J40_REMCNT 60000
#define MRF24J40_REMCNT_NSEC (MRF24J40_REMCNT * 50)

#define MRF24J40_MAINCNT(bo, clkper) \
  ((MRF24J40_BEACONINTERVAL_NSEC(bo) - MRF24J40_REMCNT_NSEC) / \
    clkper)

/* Formula for calculating default macMaxFrameWaitTime is on pg. 130
 *
 * For PHYs other than CSS and UWB, the attribute phyMaxFrameDuration is given by:
 *
 * phyMaxFrameDuration = phySHRDuration +
 *                       ceiling([aMaxPHYPacketSize + 1] x phySymbolsPerOctet)
 *
 * where ceiling() is a function that returns the smallest integer value greater
 * than or equal to its argument value. [1] pg. 158
*/

#define MRF24J40_DEFAULT_MAX_FRAME_WAITTIME 1824

#define MRF24J40_SYMBOL_DURATION_PS 16000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A MRF24J40 device instance */

struct mrf24j40_radio_s
{
  struct ieee802154_radio_s radio;  /* The public device instance */
  FAR struct ieee802154_radiocb_s *radiocb; /* Registered callbacks */

  /* MAC Attributes */

  bool rxonidle : 1;

  /* Low-level MCU-specific support */

  FAR const struct mrf24j40_lower_s *lower;
  FAR struct spi_dev_s *spi; /* Saved SPI interface instance */

  struct work_s irqwork;       /* For deferring interrupt work to work queue */
  struct work_s csma_pollwork; /* For deferring poll work to the work queue */
  struct work_s gts_pollwork;  /* For deferring poll work to the work queue */

  sem_t         exclsem;       /* Exclusive access to this struct */

  struct ieee802154_addr_s addr;

  uint8_t         chan;       /* 11 to 26 for the 2.4 GHz band */
  uint8_t         devmode;     /* device mode: device, coord, pancoord */
  uint8_t         paenabled;   /* enable usage of PA */
  uint8_t         rxmode;      /* Reception mode: Main, no CRC, promiscuous */
  int32_t         txpower;     /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s cca; /* Clear channel assessement method */

  /* MAC PIB attributes */

  uint32_t max_frame_waittime;

  struct ieee802154_txdesc_s *txdelayed_desc;
  struct ieee802154_txdesc_s *csma_desc;
  bool txdelayed_busy : 1;
  bool csma_busy : 1;
  bool reschedule_csma : 1;

  bool rxenabled : 1;

  struct ieee802154_txdesc_s *gts_desc[MRF24J40_GTS_SLOTS];
  bool gts_busy[MRF24J40_GTS_SLOTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal operations */

static void mrf24j40_spi_lock(FAR struct spi_dev_s *spi);

static void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr,
              uint8_t val);
static uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr);

static int  mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev);
static int  mrf24j40_pacontrol(FAR struct mrf24j40_radio_s *dev, int mode);
static int  mrf24j40_initialize(FAR struct mrf24j40_radio_s *dev);

static int  mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode);
static int  mrf24j40_regdump(FAR struct mrf24j40_radio_s *dev);

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_radio_s *dev);
static void mrf24j40_irqwork_txnorm(FAR struct mrf24j40_radio_s *dev);
static void mrf24j40_irqwork_txgts(FAR struct mrf24j40_radio_s *dev,
              uint8_t gts_num);

static void mrf24j40_irqworker(FAR void *arg);
static int  mrf24j40_interrupt(int irq, FAR void *context, FAR void *arg);

static void mrf24j40_dopoll_csma(FAR void *arg);
static void mrf24j40_dopoll_gts(FAR void *arg);

static void mrf24j40_norm_setup(FAR struct mrf24j40_radio_s *dev,
              FAR struct iob_s *frame, bool csma);
static void  mrf24j40_gts_setup(FAR struct mrf24j40_radio_s *dev, uint8_t gts,
              FAR struct iob_s *frame);
static void mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *buf, uint8_t length, uint32_t fifo_addr);

static inline void mrf24j40_norm_trigger(FAR struct mrf24j40_radio_s *dev);

static int  mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev,
              uint8_t chan);
static int  mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *panid);
static int  mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *saddr);
static int  mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *eaddr);
static int mrf24j40_setcoordsaddr(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *saddr);
static int mrf24j40_setcoordeaddr(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *eaddr);
static int  mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev,
              uint8_t mode);
static int  mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev,
              int32_t txpwr);
static int  mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
              FAR struct ieee802154_cca_s *cca);
static int  mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev,
              FAR uint8_t *energy);
static void mrf24j40_mactimer(FAR struct mrf24j40_radio_s *dev, int numsymbols);

/* Driver operations */

static int  mrf24j40_bind(FAR struct ieee802154_radio_s *radio,
              FAR struct ieee802154_radiocb_s *radiocb);
static int  mrf24j40_txnotify(FAR struct ieee802154_radio_s *radio, bool gts);
static int  mrf24j40_txdelayed(FAR struct ieee802154_radio_s *radio,
                               FAR struct ieee802154_txdesc_s *txdesc,
                               uint32_t symboldelay);
static int  mrf24j40_reset_attrs(FAR struct ieee802154_radio_s *radio);
static int  mrf24j40_get_attr(FAR struct ieee802154_radio_s *radio,
              enum ieee802154_attr_e attr,
              FAR union ieee802154_attr_u *attrval);
static int  mrf24j40_set_attr(FAR struct ieee802154_radio_s *radio,
              enum ieee802154_attr_e attr,
              FAR const union ieee802154_attr_u *attrval);
static int  mrf24j40_rxenable(FAR struct ieee802154_radio_s *dev, bool enable);
static int  mrf24j40_req_rxenable(FAR struct ieee802154_radio_s *radio,
              FAR struct ieee802154_rxenable_req_s *req);
static int  mrf24j40_beaconstart(FAR struct ieee802154_radio_s *radio,
               FAR const struct ieee802154_superframespec_s *sfspec,
               FAR struct ieee802154_beaconframe_s *beacon);
static int  mrf24j40_beaconupdate(FAR struct ieee802154_radio_s *radio,
               FAR struct ieee802154_beaconframe_s *beacon);
static int  mrf24j40_beaconstop(FAR struct ieee802154_radio_s *radio);
static int  mrf24j40_sfupdate(FAR struct ieee802154_radio_s *radio,
               FAR const struct ieee802154_superframespec_s *sfspec);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_allones[8] =
{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/****************************************************************************
 * Radio Interface Functions
 ****************************************************************************/

static int mrf24j40_bind(FAR struct ieee802154_radio_s *radio,
                         FAR struct ieee802154_radiocb_s *radiocb)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  DEBUGASSERT(dev != NULL);
  dev->radiocb = radiocb;
  return OK;
}

/****************************************************************************
 * Function: mrf24j40_txnotify
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_txnotify(FAR struct ieee802154_radio_s *radio, bool gts)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  if (gts)
    {
      /* Is our single work structure available?  It may not be if there are
       * pending interrupt actions and we will have to ignore the Tx
       * availability action.
       */

      if (work_available(&dev->gts_pollwork))
        {
          /* Schedule to serialize the poll on the worker thread. */

          work_queue(HPWORK, &dev->gts_pollwork, mrf24j40_dopoll_gts, dev, 0);
        }
    }
  else
    {
      /* Is our single work structure available?  It may not be if there are
       * pending interrupt actions and we will have to ignore the Tx
       * availability action.
       */

      if (work_available(&dev->csma_pollwork))
        {
          /* Schedule to serialize the poll on the worker thread. */

          work_queue(HPWORK, &dev->csma_pollwork, mrf24j40_dopoll_csma, dev, 0);
        }
    }

  return OK;
}

/****************************************************************************
 * Function: mrf24j40_txdelayed
 *
 * Description:
 *   Transmit a packet without regard to supeframe structure after a certain
 *   number of symbols.  This function is used to send Data Request responses.
 *   It can also be used to send data immediately if the delay is set to 0.
 *
 * Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_txdelayed(FAR struct ieee802154_radio_s *radio,
                              FAR struct ieee802154_txdesc_s *txdesc,
                              uint32_t symboldelay)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  uint8_t reg;

  /* Get exclusive access to the radio device */

  if (sem_wait(&dev->exclsem) != 0)
    {
      return -EINTR;
    }

  /* There should never be more than one of these transactions at once. */

  DEBUGASSERT(!dev->txdelayed_busy);

  dev->txdelayed_desc = txdesc;
  dev->txdelayed_busy = true;

  /* Disable the TX norm interrupt and clear it */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* If after disabling the interrupt, the irqworker is not scheduled, there
   * are no interrupts to worry about. However, if there is work scheduled,
   * we need to process it before going any further.
   */

  if (!work_available(&dev->irqwork))
    {
      work_cancel(HPWORK, &dev->irqwork);
      sem_post(&dev->exclsem);
      mrf24j40_irqworker((FAR void *)dev);

      /* Get exclusive access to the radio device */

      if (sem_wait(&dev->exclsem) != 0)
        {
          return -EINTR;
        }
    }

  if (dev->csma_busy)
    {
      dev->reschedule_csma = true;
    }

  mrf24j40_norm_setup(dev, txdesc->frame, false);

  if (symboldelay == 0)
    {
      mrf24j40_norm_trigger(dev);
    }
  else
    {
      mrf24j40_mactimer(dev, symboldelay);
    }

  sem_post(&dev->exclsem);

  return OK;
}

static int  mrf24j40_reset_attrs(FAR struct ieee802154_radio_s *radio)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  dev->max_frame_waittime = MRF24J40_DEFAULT_MAX_FRAME_WAITTIME;

  return OK;
}

static int mrf24j40_get_attr(FAR struct ieee802154_radio_s *radio,
                             enum ieee802154_attr_e attr,
                             FAR union ieee802154_attr_u *attrval)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int ret;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_EADDR:
        {
          memcpy(&attrval->mac.eaddr[0], &dev->addr.eaddr[0], 8);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

     case IEEE802154_ATTR_MAC_MAX_FRAME_WAITTIME:
        {
          attrval->mac.max_frame_waittime = dev->max_frame_waittime;
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_PHY_SYMBOL_DURATION:
        {
          attrval->phy.symdur_picosec = MRF24J40_SYMBOL_DURATION_PS;
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_PHY_CHAN:
        {
          attrval->phy.chan = dev->chan;
          ret = IEEE802154_STATUS_SUCCESS;
        }

      default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
    }

  return ret;
}

static int mrf24j40_set_attr(FAR struct ieee802154_radio_s *radio,
                             enum ieee802154_attr_e attr,
                             FAR const union ieee802154_attr_u *attrval)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int ret;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          mrf24j40_setpanid(dev, attrval->mac.panid);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_MAC_SADDR:
        {
          mrf24j40_setsaddr(dev, attrval->mac.saddr);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_MAC_EADDR:
        {
          mrf24j40_seteaddr(dev, attrval->mac.eaddr);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
          mrf24j40_setcoordsaddr(dev, attrval->mac.coordsaddr);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
          mrf24j40_setcoordeaddr(dev, attrval->mac.coordeaddr);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_MAC_PROMISCUOUS_MODE:
        {
          if (attrval->mac.promisc_mode)
            {
              mrf24j40_setrxmode(dev, MRF24J40_RXMODE_PROMISC);
            }
          else
            {
              mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL);
            }

          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_MAC_RX_ON_WHEN_IDLE:
        {
          dev->rxonidle = attrval->mac.rxonidle;
          mrf24j40_rxenable(radio, dev->rxonidle);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      case IEEE802154_ATTR_PHY_CHAN:
        {
          mrf24j40_setchannel(dev, attrval->phy.chan);
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
        break;
    }

  return ret;
}

static int  mrf24j40_req_rxenable(FAR struct ieee802154_radio_s *radio,
                                  FAR struct ieee802154_rxenable_req_s *req)
{
  return -ENOTTY;
}

static int  mrf24j40_beaconstart(FAR struct ieee802154_radio_s *radio,
               FAR const struct ieee802154_superframespec_s *sfspec,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  uint32_t maincnt = 0;
  uint32_t slpcal = 0;
  int reg;

  if (sfspec->pancoord)
    {
      /* Set the PANCOORD (RXMCR 0x00<3>) bit = 1to configure as PAN coordinator */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
      reg |= MRF24J40_RXMCR_PANCOORD;
      mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);

      /* Set the SLOTTED (TXMCR 0x11<5>) bit = 1 to use Slotted CSMA-CA mode */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);
      reg |= MRF24J40_TXMCR_SLOTTED;
      mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);

      /* Load the beacon frame into the TXBFIFO (0x080-0x0FF). */

      mrf24j40_setup_fifo(dev, beacon->bf_data, beacon->bf_len, MRF24J40_BEACON_FIFO);

      /* Set the TXBMSK (TXBCON1 0x25<7>) bit = 1 to mask the beacon interrupt
       * mask
       */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_TXBCON1);
      reg |= MRF24J40_TXBCON1_TXBMSK;
      mrf24j40_setreg(dev->spi, MRF24J40_TXBCON1, reg);

      /* Set INTL (WAKECON 0x22<5:0>) value to 0x03. */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_WAKECON);
      reg &= ~MRF24J40_WAKECON_INTL;
      reg |= 0x03 & MRF24J40_WAKECON_INTL;
      mrf24j40_setreg(dev->spi, MRF24J40_WAKECON, reg);

      /* Program the CAP end slot (ESLOTG1 0x13<3:0>) value. */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_ESLOTG1);
      reg &= ~MRF24J40_ESLOTG1_CAP;
      reg |= sfspec->final_capslot & MRF24J40_ESLOTG1_CAP;
      mrf24j40_setreg(dev->spi, MRF24J40_ESLOTG1, reg);

      /* TODO: Add GTS related code. See pg 100 of datasheet */

      /* Calibrate the Sleep Clock (SLPCLK) frequency. Refer to Section 3.15.1.2
       * “Sleep Clock Calibration”.
       */

      /* If the Sleep Clock Selection, SLPCLKSEL (0x207<7:6), is the internal
       * oscillator (100 kHz), set SLPCLKDIV to a minimum value of 0x01.
       */

      mrf24j40_setreg(dev->spi, MRF24J40_SLPCON1, 0x01);

      /* Select the source of SLPCLK (internal 100kHz) */

      mrf24j40_setreg(dev->spi, MRF24J40_RFCON7, MRF24J40_RFCON7_SEL_100KHZ);

      /* Begin calibration by setting the SLPCALEN bit (SLPCAL2 0x20B<4>) to
       * ‘1’. Sixteen samples of the SLPCLK are counted and stored in the
       * SLPCAL register. No need to mask, this is the only writable bit
       */

      mrf24j40_setreg(dev->spi, MRF24J40_SLPCAL2, MRF24J40_SLPCAL2_SLPCALEN);

      /* Calibration is complete when the SLPCALRDY bit (SLPCAL2 0x20B<7>) is
       * set to ‘1’.
       */

      while (!(mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL2) &
             MRF24J40_SLPCAL2_SLPCALRDY))
        {
          usleep(1);
        }

      slpcal  = mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL0);
      slpcal |= (mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL1) << 8);
      slpcal |= ((mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL2) << 16) & 0x0F);

      /* Set WAKECNT (SLPACK 0x35<6:0>) value = 0x5F to set the main oscillator
       * (20 MHz) start-up timer value.
       */

      mrf24j40_setreg(dev->spi, MRF24J40_SLPACK, 0x5F);

      /* Program the Beacon Interval into the Main Counter, MAINCNT (0x229<1:0>,
       * 0x228, 0x227, 0x226), and Remain Counter, REMCNT (0x225, 0x224),
       * according to BO and SO values. Refer to Section 3.15.1.3 “Sleep Mode
       * Counters”
       */

      mrf24j40_setreg(dev->spi, MRF24J40_REMCNTL, (MRF24J40_REMCNT & 0xFF));
      mrf24j40_setreg(dev->spi, MRF24J40_REMCNTH, ((MRF24J40_REMCNT >> 8) & 0xFF));

      maincnt = MRF24J40_MAINCNT(sfspec->beaconorder, (slpcal * 50 / 16));

      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT0, (maincnt & 0xFF));
      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT1, ((maincnt >> 8)  & 0xFF));
      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT2, ((maincnt >> 16) & 0xFF));
      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT3, ((maincnt >> 24) & 0x03));

      /* Enable the SLPIF and WAKEIF flags */

      reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg &= ~(MRF24J40_INTCON_SLPIE | MRF24J40_INTCON_WAKEIE);
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

      /* Configure the BO (ORDER 0x10<7:4>) and SO (ORDER 0x10<3:0>) values.
       * After configuring BO and SO, the beacon frame will be sent immediately.
       */

      mrf24j40_setreg(dev->spi, MRF24J40_ORDER,
        ((sfspec->beaconorder << 4) & 0xF0) | (sfspec->sforder & 0x0F));
    }
  else
    {
      return -ENOTTY;
    }

  return OK;
}

static int  mrf24j40_beaconupdate(FAR struct ieee802154_radio_s *radio,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  mrf24j40_setup_fifo(dev, beacon->bf_data, beacon->bf_len, MRF24J40_BEACON_FIFO);

  return OK;
}

static int  mrf24j40_beaconstop(FAR struct ieee802154_radio_s *radio)
{
  return -ENOTTY;
}

static int mrf24j40_sfupdate(FAR struct ieee802154_radio_s *radio,
             FAR const struct ieee802154_superframespec_s *sfspec)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int reg;

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);

  if (sfspec->pancoord)
    {
      reg |= MRF24J40_RXMCR_PANCOORD;
    }
  else
    {
      reg &= ~MRF24J40_RXMCR_PANCOORD;
    }
  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);

  /* Program the CAP end slot (ESLOTG1 0x13<3:0>) value. */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_ESLOTG1);
  reg &= ~MRF24J40_ESLOTG1_CAP;
  reg |= sfspec->final_capslot & MRF24J40_ESLOTG1_CAP;
  mrf24j40_setreg(dev->spi, MRF24J40_ESLOTG1, reg);

  /* Configure the BO (ORDER 0x10<7:4>) and SO (ORDER 0x10<3:0>) values.
    * After configuring BO and SO, the beacon frame will be sent immediately.
    */

  mrf24j40_setreg(dev->spi, MRF24J40_ORDER,
    ((sfspec->beaconorder << 4) & 0xF0) | (sfspec->sforder & 0x0F));

  return OK;
}

/****************************************************************************
 * Internal Functions
 ****************************************************************************/

static void mrf24j40_mactimer(FAR struct mrf24j40_radio_s *dev, int numsymbols)
{
  uint16_t nhalfsym;
  uint8_t reg;

  nhalfsym = (numsymbols << 1);

  /* Disable the interrupt, clear the timer count */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_HSYMTMRIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  mrf24j40_setreg(dev->spi, MRF24J40_HSYMTMRL, 0x00);
  mrf24j40_setreg(dev->spi, MRF24J40_HSYMTMRH, 0x00);

  reg &= ~MRF24J40_INTCON_HSYMTMRIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Set the timer count and enable interrupts */

  reg = (nhalfsym & 0xFF);
  mrf24j40_setreg(dev->spi, MRF24J40_HSYMTMRL, reg);

  reg = (nhalfsym >> 8) & 0xFF;
  mrf24j40_setreg(dev->spi, MRF24J40_HSYMTMRH, reg);
}

/****************************************************************************
 * Function: mrf24j40_dopoll_csma
 *
 * Description:
 *   This function is called in order to preform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mrf24j40_txdone_csma),
 *   2. When new TX data is available (mrf24j40_txnotify), and
 *   3. After a TX timeout to restart the sending process
 *      (mrf24j40_txtimeout_csma).
 *
 * Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_dopoll_csma(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int len = 0;

  /* Get exclusive access to the driver */

  while (sem_wait(&dev->exclsem) != 0) { }

  /* If this a CSMA transaction and we have room in the CSMA fifo */

  if (!dev->csma_busy)
    {
      len = dev->radiocb->poll(dev->radiocb, false, &dev->csma_desc);

      if (len > 0)
        {
          /* Now the txdesc is in use */

          dev->csma_busy = 1;

          /* Setup the transaction on the device in the CSMA FIFO */

          mrf24j40_norm_setup(dev, dev->csma_desc->frame, true);
          mrf24j40_norm_trigger(dev);
        }
    }

  sem_post(&dev->exclsem);
}

/****************************************************************************
 * Function: mrf24j40_dopoll_gts
 *
 * Description:
 *   This function is called in order to preform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mrf24j40_txdone_gts),
 *   2. When new TX data is available (mrf24j40_txnotify), and
 *   3. After a TX timeout to restart the sending process
 *      (mrf24j40_txtimeout_gts).
 *
 * Parameters:
 *   arg  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_dopoll_gts(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int gts = 0;
  int len = 0;

  /* Get exclusive access to the driver */

  while (sem_wait(&dev->exclsem) != 0) { }

  for (gts = 0; gts < MRF24J40_GTS_SLOTS; gts++)
    {
      if (!dev->gts_busy[gts])
        {
          len = dev->radiocb->poll(dev->radiocb, true, &dev->gts_desc[gts]);

          if (len > 0)
            {
              /* Now the txdesc is in use */

              dev->gts_busy[gts]= 1;

              /* Setup the transaction on the device in the open GTS FIFO */

              mrf24j40_gts_setup(dev, gts, dev->gts_desc[gts]->frame);
            }
        }
    }

  sem_post(&dev->exclsem);
}

/****************************************************************************
 * Name: mrf24j40_spi_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static void mrf24j40_spi_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 1);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, CONFIG_IEEE802154_MRF24J40_SPIMODE);
  SPI_SETFREQUENCY(spi, CONFIG_IEEE802154_MRF24J40_FREQUENCY);
}

/****************************************************************************
 * Name: mrf24j40_spi_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static inline void mrf24j40_spi_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi,0);
}

/****************************************************************************
 * Name: mrf24j40_setreg
 *
 * Description:
 *   Define the value of an MRF24J40 device register
 *
 ****************************************************************************/

static void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr,
                           uint8_t val)
{
  uint8_t buf[3];
  int     len;

  if (!(addr&0x80000000))
    {
      addr  &= 0x3F; /* 6-bit address */
      addr <<= 1;
      addr  |= 0x01; /* writing */
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      addr  &= 0x3FF; /* 10-bit address */
      addr <<= 5;
      addr  |= 0x8010; /* writing long */
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xFF);
      len    = 2;
    }

  buf[len++] = val;

  mrf24j40_spi_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);
  SPI_SNDBLOCK(spi, buf, len);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);
}

/****************************************************************************
 * Name: mrf24j40_getreg
 *
 * Description:
 *   Return the value of an MRF24J40 device register*
 *
 ****************************************************************************/

static uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr)
{
  uint8_t buf[3];
  uint8_t rx[3];
  int     len;

  if (!(addr&0x80000000))
    {
      /* 6-bit address */

      addr  &= 0x3F;
      addr <<= 1;
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      /* 10-bit address */

      addr  &= 0x3FF;
      addr <<= 5;
      addr  |= 0x8000;
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xFF);
      len    = 2;
    }

  buf[len++] = 0xFF; /* dummy */

  mrf24j40_spi_lock  (spi);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), true);
  SPI_EXCHANGE   (spi, buf, rx, len);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);

  /* wlinfo("r[%04X]=%02X\n", addr, rx[len - 1]); */
  return rx[len - 1];
}

/****************************************************************************
 * Name: mrf24j40_resetrfsm
 *
 * Description:
 *   Reset the RF state machine. Required at boot, after channel change,
 *   and probably after PA settings.
 *
 ****************************************************************************/

static int mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RFCTL);
  reg |= 0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);

  reg &= ~0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);
  up_udelay(200);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_pacontrol
 *
 * Description:
 *   Control the external LNA/PA on the MRF24J40MB/MC/MD/ME modules
 *   GPIO 1: PA enable
 *   GPIO 2: LNA enable
 *   GPIO 3: PA power enable (not required on MB)
 ****************************************************************************/

static int mrf24j40_pacontrol(FAR struct mrf24j40_radio_s *dev, int mode)
{
  if (!dev->paenabled)
    {
      return OK;
    }

  if (mode == MRF24J40_PA_AUTO)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x0F);
    }
  else if (mode == MRF24J40_PA_ED)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x0F);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x0C);
    }
  else if (mode == MRF24J40_PA_SLEEP)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_TESTMODE, 0x08);
      mrf24j40_setreg(dev->spi, MRF24J40_TRISGPIO, 0x0F);
      mrf24j40_setreg(dev->spi, MRF24J40_GPIO    , 0x00);
    }
  else
    {
      return -EINVAL;
    }

  mrf24j40_resetrfsm(dev);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_initialize
 *
 * Description:
 *   Reset the device and put in in order of operation
 *
 ****************************************************************************/

static int mrf24j40_initialize(FAR struct mrf24j40_radio_s *dev)
{
  /* Software reset */

  mrf24j40_setreg(dev->spi, MRF24J40_SOFTRST  , 0x07); /* 00000111 Reset */
  while(mrf24j40_getreg(dev->spi, MRF24J40_SOFTRST) & 0x07);

  /* Apply recommended settings */

  mrf24j40_setreg(dev->spi, MRF24J40_PACON2 , 0x98); /* 10011000 Enable FIFO (default), TXONTS=6 (recommended), TXONT<8:7>=0 */
  mrf24j40_setreg(dev->spi, MRF24J40_TXSTBL , 0x95); /* 10010101 set the SIFS period. RFSTBL=9, MSIFS=5, aMinSIFSPeriod=14 (min 12) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXPEND , 0x7C); /* 01111100 set the LIFS period, MLIFS=1Fh=31 aMinLIFSPeriod=40 (min 40) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXTIME , 0x30); /* 00110000 set the turnaround time, TURNTIME=3 aTurnAroundTime=12 */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON1 , 0x02); /* 00000010 VCO optimization, recommended value */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON2 , 0x80); /* 10000000 Enable PLL */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON6 , 0x90); /* 10010000 TX filter enable, fast 20M recovery, No bat monitor*/
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON7 , 0x80); /* 10000000 Sleep clock on internal 100 kHz */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON8 , 0x10); /* 00010000 VCO control bit, as recommended */
  mrf24j40_setreg(dev->spi, MRF24J40_SLPCON1, 0x01); /* 00000001 no CLKOUT, default divisor */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6 , 0x40); /* 01000000 Append RSSI to rx packets */

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setrxmode
 *
 * Description:
 *   Set the RX mode (normal, promiscuous, no CRC)
 *
 ****************************************************************************/

static int mrf24j40_setrxmode(FAR struct mrf24j40_radio_s *dev, int mode)
{
  uint8_t reg;

  if (mode < MRF24J40_RXMODE_NORMAL || mode > MRF24J40_RXMODE_NOCRC)
    {
      return -EINVAL;
    }

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
  reg &= ~0x03;
  reg |= mode;

  /* Set mode options */

  if (mode != MRF24J40_RXMODE_NORMAL)
    {
      /* Promisc and error modes: Disable auto ACK */

      reg |= MRF24J40_RXMCR_NOACKRSP;
    }
  else
    {
      /* Normal mode : enable auto-ACK */

      reg &= ~MRF24J40_RXMCR_NOACKRSP;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);

  dev->rxmode = mode;
  wlinfo("%u\n", (unsigned)mode);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *   In the 2.4 GHz, there are 16 channels, each 2 MHz wide, 5 MHz spacing:
 *   Chan   MHz       Chan   MHz       Chan   MHz       Chan   MHz
 *     11  2405         15  2425         19  2445         23  2465
 *     12  2410         16  2430         20  2450         24  2470
 *     13  2415         17  2435         21  2455         25  2475
 *     14  2420         18  2440         22  2460         26  2480
 *
 ****************************************************************************/

static int mrf24j40_setchannel(FAR struct mrf24j40_radio_s *dev, uint8_t chan)
{
  if (chan < 11 || chan > 26)
    {
      wlerr("ERROR: Invalid chan: %d\n",chan);
      return -EINVAL;
    }

  /* 15. Set channel – See Section 3.4 “Channel Selection”. */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON0, (chan - 11) << 4 | 0x03);

  /* 17. RFCTL (0x36) = 0x04 – Reset RF state machine.
   * 18. RFCTL (0x36) = 0x00.
   */

  mrf24j40_resetrfsm(dev);

  dev->chan = chan;
  wlinfo("%u\n", (unsigned)chan);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setpanid
 *
 * Description:
 *   Define the PAN ID the device is operating on.
 *
 ****************************************************************************/

static int mrf24j40_setpanid(FAR struct mrf24j40_radio_s *dev,
                             FAR const uint8_t *panid)
{
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDL, panid[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_PANIDH, panid[1]);

  IEEE802154_PANIDCOPY(dev->addr.panid, panid);
  wlinfo("%02X:%02X\n", panid[1], panid[0]);

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setsaddr
 *
 * Description:
 *   Define the device short address. The following addresses are special:
 *   FFFEh : Broadcast
 *   FFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_setsaddr(FAR struct mrf24j40_radio_s *dev,
                             FAR const uint8_t *saddr)
{
  mrf24j40_setreg(dev->spi, MRF24J40_SADRL, saddr[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_SADRH, saddr[1]);

  IEEE802154_SADDRCOPY(dev->addr.saddr, saddr);

  wlinfo("%02X:%02X\n", saddr[1], saddr[0]);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_seteaddr
 *
 * Description:
 *   Define the device extended address. The following addresses are special:
 *   FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_seteaddr(FAR struct mrf24j40_radio_s *dev,
                             FAR const uint8_t *eaddr)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_EADR0 + i, eaddr[i]);
      dev->addr.eaddr[i] = eaddr[i];
    }

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setcoordsaddr
 *
 * Description:
 *   Define the coordinator short address. The following addresses are special:
 *   FFFEh : Broadcast
 *   FFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_setcoordsaddr(FAR struct mrf24j40_radio_s *dev,
                                  FAR const uint8_t *saddr)
{
  mrf24j40_setreg(dev->spi, MRF24J40_ASSOSADR0, saddr[0]);
  mrf24j40_setreg(dev->spi, MRF24J40_ASSOSADR1, saddr[1]);

  IEEE802154_SADDRCOPY(dev->addr.saddr, saddr);

  wlinfo("%02X:%02X\n", saddr[1], saddr[0]);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setcoordeaddr
 *
 * Description:
 *   Define the coordinator extended address. The following addresses are special:
 *   FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

static int mrf24j40_setcoordeaddr(FAR struct mrf24j40_radio_s *dev,
                                  FAR const uint8_t *eaddr)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      mrf24j40_setreg(dev->spi, MRF24J40_ASSOEADR0 + i, eaddr[i]);
      dev->addr.eaddr[i] = eaddr[i];
    }

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setdevmode
 *
 * Description:
 *   Define the device behaviour: normal end device or coordinator
 *
 ****************************************************************************/

static int mrf24j40_setdevmode(FAR struct mrf24j40_radio_s *dev,
                               uint8_t mode)
{
  int ret = OK;
  uint8_t reg;

  /* Disable slotted mode until I decide to implement slotted mode */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);
  reg &= ~MRF24J40_TXMCR_SLOTTED;
  mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);
  mrf24j40_setreg(dev->spi, MRF24J40_ORDER, 0xFF);

  /* Define dev mode */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);

  if (mode == MRF24J40_MODE_PANCOORD)
    {
      reg |=  MRF24J40_RXMCR_PANCOORD;
      reg &= ~MRF24J40_RXMCR_COORD;
    }
  else if (mode == MRF24J40_MODE_COORD)
    {
      reg |=  MRF24J40_RXMCR_COORD;
      reg &= ~MRF24J40_RXMCR_PANCOORD;
    }
  else if (mode == MRF24J40_MODE_DEVICE)
    {
      reg &= ~MRF24J40_RXMCR_PANCOORD;
      reg &= ~MRF24J40_RXMCR_COORD;
    }
  else
    {
      return -EINVAL;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);
  dev->devmode = mode;
  return ret;
}

/****************************************************************************
 * Name: mrf24j40_settxpower
 *
 * Description:
 *   Define the transmit power. Value is passed in mBm, it is rounded to
 *   the nearest value. Some MRF modules have a power amplifier, this routine
 *   does not care about this. We only change the CHIP output power.
 *
 ****************************************************************************/

static int mrf24j40_settxpower(FAR struct mrf24j40_radio_s *dev,
                               int32_t txpwr)
{
  uint8_t reg;
  int save_txpwr = txpwr;

  if (txpwr <= -3000 && txpwr > -3630)
    {
      reg = 0xC0;
      txpwr += 3000;
    }
  else if (txpwr <= -2000)
    {
      reg = 0x80;
      txpwr += 2000;
    }
  else if (txpwr <= -1000)
    {
      reg = 0x40;
      txpwr += 1000;
    }
  else if (txpwr <= 0)
    {
      reg = 0x00;
    }
  else
    {
      return -EINVAL;
    }

  wlinfo("remaining attenuation: %d mBm\n",txpwr);

  switch(txpwr/100)
    {
      case -9:
      case -8:
      case -7:
      case -6:
        reg |= 0x07;
        break;

      case -5:
        reg |= 0x06;
        break;

      case -4:
        reg |= 0x05;
        break;

      case -3:
        reg |= 0x04;
        break;

      case -2:
        reg |= 0x03;
        break;

      case -1:
        reg |= 0x02;
        break;

      case  0:
        reg |= 0x00;  /* value 0x01 is 0.5 db, not used */
        break;

      default:
        return -EINVAL;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON3, reg);
  dev->txpower = save_txpwr;
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_setcca
 *
 * Description:
 *   Define the Clear Channel Assessement method.
 *
 ****************************************************************************/

static int mrf24j40_setcca(FAR struct mrf24j40_radio_s *dev,
                           FAR struct ieee802154_cca_s *cca)
{
  uint8_t mode;

  if (!cca->use_ed && !cca->use_cs)
    {
      return -EINVAL;
    }

  if (cca->use_cs && cca->csth > 0x0f)
    {
      return -EINVAL;
    }

  mode  = mrf24j40_getreg(dev->spi, MRF24J40_BBREG2);
  mode &= 0x03;

  if (cca->use_ed)
    {
      mode |= MRF24J40_BBREG2_CCAMODE_ED;
      mrf24j40_setreg(dev->spi, MRF24J40_CCAEDTH, cca->edth);
    }

  if (cca->use_cs)
    {
      mode |= MRF24J40_BBREG2_CCAMODE_CS;
      mode |= cca->csth << 2;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG2, mode);

  memcpy(&dev->cca, cca, sizeof(struct ieee802154_cca_s));
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_regdump
 *
 * Description:
 *   Display the value of all registers.
 *
 ****************************************************************************/

static int mrf24j40_regdump(FAR struct mrf24j40_radio_s *dev)
{
  uint32_t i;
  char buf[4+16*3+2+1];
  int len = 0;

  wlinfo("Short regs:\n");

  for (i = 0; i < 0x40; i++)
    {
      if ((i & 15) == 0)
        {
          len=sprintf(buf, "%02x: ",i&0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          wlinfo("%s", buf);
        }
    }

  wlinfo("Long regs:\n");
  for (i = 0x80000200; i < 0x80000250; i++)
    {
      if ((i & 15) == 0)
        {
          len=sprintf(buf, "%02x: ",i&0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          wlinfo("%s", buf);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mrf24j40_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

static int mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev,
                                 FAR uint8_t *energy)
{
  uint8_t reg;

  /* Manually enable the LNA*/

  mrf24j40_pacontrol(dev, MRF24J40_PA_ED);

  /* Set RSSI average duration to 8 symbols */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXBCON1);
  reg |= 0x30;
  mrf24j40_setreg(dev->spi, MRF24J40_TXBCON1, reg);

  /* 1. Set RSSIMODE1 0x3E<7> – Initiate RSSI calculation. */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x80);

  /* 2. Wait until RSSIRDY 0x3E<0> is set to ‘1’ – RSSI calculation is
   *    complete.
   */

  while(!(mrf24j40_getreg(dev->spi, MRF24J40_BBREG6) & 0x01));

  /* 3. Read RSSI 0x210<7:0> – The RSSI register contains the averaged RSSI
   *    received power level for 8 symbol periods.
   */

  *energy = mrf24j40_getreg(dev->spi, MRF24J40_RSSI);

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x40);

  /* Back to automatic control */

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_norm_setup
 *
 * Description:
 *   Setup a transaction in the normal TX FIFO
 *
 ****************************************************************************/

static void mrf24j40_norm_setup(FAR struct mrf24j40_radio_s *dev,
                                FAR struct iob_s *frame, bool csma)
{
  uint8_t reg;

  /* Enable tx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg &= ~MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Enable/Disable CSMA mode */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);

  if (csma)
    {
      reg &= ~MRF24J40_TXMCR_NOCSMA;
    }
  else
    {
      reg |= MRF24J40_TXMCR_NOCSMA;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);

  /* Setup the FIFO */

  mrf24j40_setup_fifo(dev, frame->io_data, frame->io_len, MRF24J40_TXNORM_FIFO);

  /* If the frame control field contains an acknowledgment request, set the
   * TXNACKREQ bit. See IEEE 802.15.4/2003 7.2.1.1 page 112 for info.
   */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXNCON);

  if (frame->io_data[0] & IEEE802154_FRAMECTRL_ACKREQ)
    {
      reg |= MRF24J40_TXNCON_TXNACKREQ;
    }
  else
    {
      reg &= ~MRF24J40_TXNCON_TXNACKREQ;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_TXNCON, reg);
}

/****************************************************************************
 * Name: mrf24j40_norm_trigger
 *
 * Description:
 *   Trigger the normal TX FIFO
 *
 ****************************************************************************/

static inline void mrf24j40_norm_trigger(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXNCON);

  reg |= MRF24J40_TXNCON_TXNTRIG;

  mrf24j40_setreg(dev->spi, MRF24J40_TXNCON, reg);
}

/****************************************************************************
 * Name: mrf24j40_gts_setup
 *
 * Description:
 *   Setup a GTS transaction in one of the GTS FIFOs
 *
 ****************************************************************************/

static void mrf24j40_gts_setup(FAR struct mrf24j40_radio_s *dev, uint8_t fifo,
                               FAR struct iob_s *frame)
{

}

/****************************************************************************
 * Name: mrf24j40_setup_fifo
 *
 * Description:
 *
 ****************************************************************************/

static void mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev,
                               FAR const uint8_t *buf, uint8_t length,
                               uint32_t fifo_addr)
{
  int hlen = 3; /* Include frame control and seq number */
  int i;
  uint16_t frame_ctrl;

  /* Analyze frame control to compute header length */

  frame_ctrl = buf[0];
  frame_ctrl |= (buf[1] << 8);

  if ((frame_ctrl & IEEE802154_FRAMECTRL_DADDR)== IEEE802154_ADDRMODE_SHORT)
    {
      hlen += 2 + 2; /* Destination PAN + shortaddr */
    }
  else if ((frame_ctrl & IEEE802154_FRAMECTRL_DADDR) == IEEE802154_ADDRMODE_EXTENDED)
    {
      hlen += 2 + 8; /* Destination PAN + extaddr */
    }

  if (!(frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP))
    {
      hlen += 2; /* No PAN compression, source PAN is different from dest PAN */
    }

  if ((frame_ctrl & IEEE802154_FRAMECTRL_SADDR)== IEEE802154_ADDRMODE_SHORT)
    {
      hlen += 2; /* Source saddr */
    }
  else if ((frame_ctrl & IEEE802154_FRAMECTRL_SADDR) == IEEE802154_ADDRMODE_EXTENDED)
    {
      hlen += 8; /* Ext saddr */
    }

  /* Header len, 0, TODO for security modes */

  mrf24j40_setreg(dev->spi, fifo_addr++, hlen);

  /* Frame length */

  mrf24j40_setreg(dev->spi, fifo_addr++, length);

  /* Frame data */

  for (i = 0; i < length; i++)
    {
      mrf24j40_setreg(dev->spi, fifo_addr++, buf[i]);
    }
}

/****************************************************************************
 * Name: mrf24j40_irqwork_txnorm
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_txnorm(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;
  enum ieee802154_status_e status;
  bool framepending;

  /* Disable tx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Get the status from the device and copy the status into the tx desc.
   * The status for the normal FIFO is represented with bit TXNSTAT where
   * 0=success, 1= failure.
   */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_TXSTAT);

  /* TXNSTAT = 0: Transmission was successful
   * TXNSTAT = 1: Transmission failed, retry count exceeded
   */

  if (reg & MRF24J40_TXSTAT_TXNSTAT)
    {
      /* The number of retries of the most recent transmission is contained in the
       * TXNRETRY (TXSTAT 0x24<7:6>) bits. The CCAFAIL (TXSTAT 0x24<5>) bit = 1
       * indicates if the failed transmission was due to the channel busy
       * (CSMA-CA timed out).
       */

      if (reg & MRF24J40_TXSTAT_CCAFAIL)
        {
          status = IEEE802154_STATUS_CHANNEL_ACCESS_FAILURE;
        }
      else
        {
          status = IEEE802154_STATUS_NO_ACK;
        }
    }
  else
    {
      status = IEEE802154_STATUS_SUCCESS;
    }

  framepending = (mrf24j40_getreg(dev->spi, MRF24J40_TXNCON) &
                  MRF24J40_TXNCON_FPSTAT);

  if (dev->txdelayed_busy)
    {
      /* Inform the next layer of the transmission success/failure */

      dev->txdelayed_desc->conf->status = status;
      dev->txdelayed_desc->framepending = framepending;
      dev->radiocb->txdone(dev->radiocb, dev->txdelayed_desc);

      dev->txdelayed_busy = false;

      if (dev->reschedule_csma)
        {
          mrf24j40_norm_setup(dev, dev->csma_desc->frame, true);
          mrf24j40_norm_trigger(dev);
          dev->reschedule_csma = false;
        }
    }
  else
    {
      /* Inform the next layer of the transmission success/failure */

      dev->csma_desc->conf->status = status;
      dev->csma_desc->framepending = framepending;
      dev->radiocb->txdone(dev->radiocb, dev->csma_desc);

      /* We are now done with the transaction */

      dev->csma_busy = 0;

      /* Must unlock the radio before calling poll */

      sem_post(&dev->exclsem);
      mrf24j40_dopoll_csma(dev);
      while (sem_wait(&dev->exclsem) != 0) { }
    }
}

/****************************************************************************
 * Name: mrf24j40_irqwork_gts
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_txgts(FAR struct mrf24j40_radio_s *dev,
                                   uint8_t gts)
{
  uint8_t txstat;

  /* Disable tx int */

  txstat  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  txstat |= MRF24J40_INTCON_TXNIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, txstat);

  /* Get the status from the device and copy the status into the tx desc.
   * The status for the normal FIFO is represented with bit TXNSTAT where
   * 0=success, 1= failure.
   */

  txstat = mrf24j40_getreg(dev->spi, MRF24J40_TXSTAT);

  if (gts == 0)
    {
      dev->csma_desc->conf->status = txstat & MRF24J40_TXSTAT_TXG1STAT;
    }
  else if (gts == 1)
    {
      dev->csma_desc->conf->status = txstat & MRF24J40_TXSTAT_TXG2STAT;
    }

  /* Inform the next layer of the transmission success/failure */

  dev->radiocb->txdone(dev->radiocb, dev->gts_desc[gts]);

  /* We are now done with the transaction */

  dev->gts_busy[gts]= 0;

  mrf24j40_dopoll_gts(dev);
}

/****************************************************************************
 * Name: mrf24j40_rxenable
 *
 * Description:
 *  Enable/Disable receiver.
 *
 ****************************************************************************/

static int mrf24j40_rxenable(FAR struct ieee802154_radio_s *radio, bool enable)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  uint8_t reg;

  dev->rxenabled = enable;


  if (enable)
    {
      /* Disable packet reception. See pg. 109 of datasheet */

      mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, MRF24J40_BBREG1_RXDECINV);

      /* Enable rx int */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg &= ~MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

      /* Purge the RX buffer */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_RXFLUSH);
      reg |= MRF24J40_RXFLUSH_RXFLUSH;
      mrf24j40_setreg(dev->spi, MRF24J40_RXFLUSH, reg);

      /* Re-enable packet reception. See pg. 109 of datasheet */

      mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, 0);
    }
  else
    {
      /* Disable rx int */

      reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg |= MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }

  return OK;
}

/****************************************************************************
 * Name: mrf24j40_irqwork_rx
 *
 * Description:
 *   Manage packet reception.
 *
 ****************************************************************************/

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_radio_s *dev)
{
  FAR struct ieee802154_data_ind_s *ind;
  uint32_t addr;
  uint32_t index;
  uint8_t  reg;

  wlinfo("RX interrupt\n");

  /* Disable rx int */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg |= MRF24J40_INTCON_RXIE;
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  /* Disable packet reception. See pg. 109 of datasheet */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, MRF24J40_BBREG1_RXDECINV);

  /* Allocate a data_ind to put the frame in */

  ind = ieee802154_ind_allocate();
  if (ind == NULL)
    {
      wlerr("ERROR: Unable to allocate data_ind. Discarding frame\n");
      goto done;
    }

  /* Read packet */

  addr = MRF24J40_RXBUF_BASE;

  ind->frame->io_len = mrf24j40_getreg(dev->spi, addr++);

  /* TODO: This needs to be changed.  It is inefficient to do the SPI read byte
   * by byte */

  for (index = 0; index < ind->frame->io_len; index++)
    {
      ind->frame->io_data[index] = mrf24j40_getreg(dev->spi, addr++);
    }

  ind->lqi  = mrf24j40_getreg(dev->spi, addr++);
  ind->rssi = mrf24j40_getreg(dev->spi, addr++);

  /* Reduce len by 2, we only receive frames with correct crc, no check
   * required.
   */

  ind->frame->io_len -= 2;

  /* Callback the receiver in the next highest layer */

  dev->radiocb->rxframe(dev->radiocb, ind);

done:
  /* Enable reception of next packet by flushing the fifo.
   * This is an MRF24J40 errata (no. 1).
   */

  mrf24j40_setreg(dev->spi, MRF24J40_RXFLUSH, 1);

  /* Only enable RX interrupt if we are to be listening when IDLE */

  if (dev->rxenabled)
    {
      /* Enable packet reception */

      mrf24j40_setreg(dev->spi, MRF24J40_BBREG1, 0);

      reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg &= ~MRF24J40_INTCON_RXIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }
}

/****************************************************************************
 * Name: mrf24j40_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mrf24j40_irqworker(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  uint8_t intstat;
  uint8_t reg;

  DEBUGASSERT(dev);
  DEBUGASSERT(dev->spi);

  /* Get exclusive access to the driver */

  while (sem_wait(&dev->exclsem) != 0) { }

  /* Read and store INTSTAT - this clears the register. */

  intstat = mrf24j40_getreg(dev->spi, MRF24J40_INTSTAT);

  /* Do work according to the pending interrupts */

  if ((intstat & MRF24J40_INTSTAT_HSYMTMRIF))
    {
      /* As of now the only use for the MAC timer is for delayed transactions.
       * Therefore, all we do here is trigger the TX norm FIFO
       */

      mrf24j40_norm_trigger(dev);

      /* Timers are one-shot, so disable the interrupt */

      reg  = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
      reg |= MRF24J40_INTCON_HSYMTMRIE;
      mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);
    }

  if ((intstat & MRF24J40_INTSTAT_RXIF) && dev->rxenabled)
    {
      /* A packet was received, retrieve it */

      mrf24j40_irqwork_rx(dev);
    }

  if ((intstat & MRF24J40_INTSTAT_TXNIF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_txnorm(dev);
    }

  if ((intstat & MRF24J40_INTSTAT_TXG1IF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_txgts(dev, 0);
    }

  if ((intstat & MRF24J40_INTSTAT_TXG1IF))
    {
      /* A packet was transmitted or failed*/

      mrf24j40_irqwork_txgts(dev, 1);
    }

  if ((intstat & MRF24J40_INTSTAT_SLPIF))
    {
      dev->radiocb->sfevent(dev->radiocb, IEEE802154_SFEVENT_ENDOFACTIVE);

      /* Acknowledge the alert and put the device to sleep */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_SLPACK);
      reg |= MRF24J40_SLPACK_SLPACK;
      mrf24j40_setreg(dev->spi, MRF24J40_SLPACK, reg);
    }

  /* Unlock the radio device */

  sem_post(&dev->exclsem);

  /* Re-enable GPIO interrupts */

  dev->lower->enable(dev->lower, true);
}

/****************************************************************************
 * Name: mrf24j40_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int mrf24j40_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;

  DEBUGASSERT(dev != NULL);

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  DEBUGASSERT(work_available(&dev->irqwork));

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in enc_irqworker() when the work is completed.
   */

  dev->lower->enable(dev->lower, false);
  return work_queue(HPWORK, &dev->irqwork, mrf24j40_irqworker, (FAR void *)dev, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_init
 *
 * Description:
 *   Return an mrf24j40 device for use by other drivers.
 *
 ****************************************************************************/

FAR struct ieee802154_radio_s *mrf24j40_init(FAR struct spi_dev_s *spi,
                                      FAR const struct mrf24j40_lower_s *lower)
{
  FAR struct mrf24j40_radio_s *dev;
  struct ieee802154_cca_s   cca;

  dev = kmm_zalloc(sizeof(struct mrf24j40_radio_s));
  if (dev == NULL)
    {
      return NULL;
    }

  /* Attach irq */

  if (lower->attach(lower, mrf24j40_interrupt, dev) != OK)
    {
#if 0
      free(dev);
#endif
      return NULL;
    }

  /* Allow exclusive access to the privmac struct */

  sem_init(&dev->exclsem, 0, 1);

  dev->radio.bind         = mrf24j40_bind;
  dev->radio.txnotify     = mrf24j40_txnotify;
  dev->radio.txdelayed    = mrf24j40_txdelayed;
  dev->radio.reset_attrs  = mrf24j40_reset_attrs;
  dev->radio.get_attr     = mrf24j40_get_attr;
  dev->radio.set_attr     = mrf24j40_set_attr;
  dev->radio.rxenable     = mrf24j40_rxenable;
  dev->radio.req_rxenable = mrf24j40_req_rxenable;
  dev->radio.beaconstart  = mrf24j40_beaconstart;
  dev->radio.beaconupdate = mrf24j40_beaconupdate;
  dev->radio.beaconstop   = mrf24j40_beaconstop;
  dev->radio.sfupdate     = mrf24j40_sfupdate;

  dev->lower    = lower;
  dev->spi      = spi;


  dev->rxenabled = false;
  mrf24j40_initialize(dev);

  mrf24j40_setchannel(dev, 11);
  mrf24j40_setpanid(dev, g_allones);
  mrf24j40_setsaddr(dev, g_allones);
  mrf24j40_seteaddr(dev, g_allones);

  /* Default device params */

  cca.use_ed = 1;
  cca.use_cs = 0;
  cca.edth   = 0x60; /* CCA mode ED, no carrier sense, recommenced ED threshold -69 dBm */
  mrf24j40_setcca(dev, &cca);

  mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL);

  mrf24j40_settxpower(dev, 0); /*16. Set transmitter power .*/

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);

  /* For now, we want to always just have the frame pending bit set when
   * acknowledging a Data Request command. The standard says that the coordinator
   * can do this if it needs time to figure out whether it has data or not
   */

  mrf24j40_setreg(dev->spi, MRF24J40_ACKTMOUT, 0x39 | MRF24J40_ACKTMOUT_DRPACK);

  dev->lower->enable(dev->lower, true);
  return &dev->radio;
}
