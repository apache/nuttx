/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_radif.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mrf24j40.h"
#include "mrf24j40_reg.h"
#include "mrf24j40_getset.h"
#include "mrf24j40_regops.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mrf24j40_mactimer(FAR struct mrf24j40_radio_s *dev,
                              int numsymbols);
static void mrf24j40_setorder(FAR struct mrf24j40_radio_s *dev, uint8_t bo,
                              uint8_t so);
static void mrf24j40_slpclkcal(FAR struct mrf24j40_radio_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_allones[8] =
{
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mrf24j40_mactimer(FAR struct mrf24j40_radio_s *dev,
                              int numsymbols)
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

  reg = (nhalfsym & 0xff);
  mrf24j40_setreg(dev->spi, MRF24J40_HSYMTMRL, reg);

  reg = (nhalfsym >> 8) & 0xff;
  mrf24j40_setreg(dev->spi, MRF24J40_HSYMTMRH, reg);
}

/****************************************************************************
 * Name: mrf24j40_setorder
 *
 * Description:
 *   Configures the timers and sets the ORDER register
 ****************************************************************************/

static void mrf24j40_setorder(FAR struct mrf24j40_radio_s *dev, uint8_t bo,
                              uint8_t so)
{
  uint32_t bi = MRF24J40_BEACONINTERVAL_NSEC(bo);
  uint32_t sfduration = MRF24J40_SUPERFRAMEDURATION_NSEC(so);
  uint32_t maincnt;
  uint32_t remcnt;

  wlinfo("bo: %d, so: %d\n", bo, so);

  if (bo < 15)
    {
      if (dev->devmode == IEEE802154_DEVMODE_ENDPOINT)
        {
          wlinfo("Configuring sleep for inactive period\n");
          maincnt = (bi - sfduration) / dev->slpclkper;
          remcnt  = ((bi - sfduration) - (maincnt * dev->slpclkper)) / 50;
        }
      else
        {
          wlinfo("Configuring sleep for beacon interval\n");
          maincnt = bi / dev->slpclkper;
          remcnt  = (bi - (maincnt * dev->slpclkper)) / 50;
        }

      wlinfo("MAINCNT: %" PRIu32 ", REMCNT: %" PRIu32 "\n", maincnt, remcnt);

      /* Program the Main Counter, MAINCNT (0x229<1:0>, 0x228, 0x227,
       * 0x226), and Remain Counter, REMCNT (0x225, 0x224), according to BO
       * and SO values.  Refer to Section 3.15.1.3 "Sleep Mode * Counters"
       */

      mrf24j40_setreg(dev->spi, MRF24J40_REMCNTL, (remcnt & 0xff));
      mrf24j40_setreg(dev->spi, MRF24J40_REMCNTH, ((remcnt >> 8) & 0xff));

      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT0, (maincnt & 0xff));
      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT1, ((maincnt >> 8)  & 0xff));
      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT2, ((maincnt >> 16) & 0xff));
      mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT3, ((maincnt >> 24) & 0x03));
    }

  /* Configure the BO (ORDER 0x10<7:4>) and SO (ORDER 0x10<3:0>) values.
   * After configuring BO and SO, the beacon frame will be sent immediately.
   */

  mrf24j40_setreg(dev->spi, MRF24J40_ORDER,
                 ((bo << 4) & 0xf0) | (so & 0x0f));
}

static void mrf24j40_slpclkcal(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  /* Select the source of SLPCLK (internal 100kHz) */

  mrf24j40_setreg(dev->spi, MRF24J40_RFCON7, MRF24J40_RFCON7_SEL_100KHZ);

  /* If the Sleep Clock Selection, SLPCLKSEL (0x207<7:6), is the internal
   * oscillator (100 kHz), set SLPCLKDIV to a minimum value of 0x01.
   */

  mrf24j40_setreg(dev->spi, MRF24J40_SLPCON1,
                  0x01 | MRF24J40_SLPCON1_CLKOUT_DISABLED);

  /* Begin calibration by setting the SLPCALEN bit (SLPCAL2 0x20b<4>) to
   * ‘1’. Sixteen samples of the SLPCLK are counted and stored in the
   * SLPCAL register. No need to mask, this is the only writable bit
   */

  mrf24j40_setreg(dev->spi, MRF24J40_SLPCAL2, MRF24J40_SLPCAL2_SLPCALEN);

  /* Calibration is complete when the SLPCALRDY bit (SLPCAL2 0x20b<7>) is
   * set to ‘1’.
   */

  while (!(mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL2) &
         MRF24J40_SLPCAL2_SLPCALRDY))
    {
      up_udelay(1);
    }

  reg = mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL0);
  dev->slpclkper  = reg;
  reg = mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL1);
  dev->slpclkper |= (reg << 8);
  reg = mrf24j40_getreg(dev->spi, MRF24J40_SLPCAL2) & 0x0f;
  dev->slpclkper |= (reg << 16);

  dev->slpclkper  = (dev->slpclkper * 50 / 16);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mrf24j40_bind(FAR struct ieee802154_radio_s *radio,
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
 * Input Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

int mrf24j40_txnotify(FAR struct ieee802154_radio_s *radio, bool gts)
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

          work_queue(HPWORK, &dev->gts_pollwork,
                     mrf24j40_dopoll_gts, dev, 0);
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

          work_queue(HPWORK, &dev->csma_pollwork, mrf24j40_dopoll_csma,
                     dev, 0);
        }
    }

  return OK;
}

/****************************************************************************
 * Function: mrf24j40_txdelayed
 *
 * Description:
 *   Transmit a packet without regard to supeframe structure after a certain
 *   number of symbols.  This function is used to send Data Request
 *   responses.  It can also be used to send data immediately if the delay
 *   is set to 0.
 *
 * Input Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

int mrf24j40_txdelayed(FAR struct ieee802154_radio_s *radio,
                              FAR struct ieee802154_txdesc_s *txdesc,
                              uint32_t symboldelay)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  uint8_t reg;
  int ret;

  /* Get exclusive access to the radio device */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
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
   * FIXME: I think this could be done cleaner.
   */

  if (!work_available(&dev->irqwork))
    {
      nxmutex_unlock(&dev->lock);
      mrf24j40_irqworker((FAR void *)dev);

      /* Get exclusive access to the radio device */

      ret = nxmutex_lock(&dev->lock);
      if (ret < 0)
        {
          return ret;
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

  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Name: mrf24j40_rxenable
 *
 * Description:
 *  Enable/Disable receiver.
 *
 ****************************************************************************/

int mrf24j40_rxenable(FAR struct ieee802154_radio_s *radio, bool enable)
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

int mrf24j40_energydetect(FAR struct ieee802154_radio_s *radio,
                          uint32_t nsymbols)
{
  return -ENOTTY;
}

int mrf24j40_reset(FAR struct ieee802154_radio_s *radio)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  struct ieee802154_cca_s   cca;
  int reg;

  /* Software reset */

  mrf24j40_setreg(dev->spi, MRF24J40_SOFTRST  , 0x07); /* 00000111 Reset */
  while (mrf24j40_getreg(dev->spi, MRF24J40_SOFTRST) & 0x07);

  /* Apply recommended settings */

  mrf24j40_setreg(dev->spi, MRF24J40_PACON2 , 0x98); /* 10011000 Enable FIFO (default), TXONTS=6 (recommended), TXONT<8:7>=0 */
  mrf24j40_setreg(dev->spi, MRF24J40_TXSTBL , 0x95); /* 10010101 set the SIFS period. RFSTBL=9, MSIFS=5, aMinSIFSPeriod=14 (min 12) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXPEND , 0x7c); /* 01111100 set the LIFS period, MLIFS=1Fh=31 aMinLIFSPeriod=40 (min 40) */
  mrf24j40_setreg(dev->spi, MRF24J40_TXTIME , 0x30); /* 00110000 set the turnaround time, TURNTIME=3 aTurnAroundTime=12 */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON1 , 0x02); /* 00000010 VCO optimization, recommended value */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON2 , 0x80); /* 10000000 Enable PLL */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON6 , 0x90); /* 10010000 TX filter enable, fast 20M recovery, No bat monitor */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON7 , 0x80); /* 10000000 Sleep clock on internal 100 kHz */
  mrf24j40_setreg(dev->spi, MRF24J40_RFCON8 , 0x10); /* 00010000 VCO control bit, as recommended */
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6 , 0x40); /* 01000000 Append RSSI to rx packets */

  /* Calibrate the Sleep Clock (SLPCLK) frequency. Refer to Section 3.15.1.2
   * “Sleep Clock Calibration”.
   */

  mrf24j40_slpclkcal(dev);

  /* For now, we want to always just have the frame pending bit set when
   * acknowledging a Data Request command. The standard says that the
   * coordinator* can do this if it needs time to figure out whether it has
   * data or not
   */

  mrf24j40_setreg(dev->spi, MRF24J40_ACKTMOUT,
                  0x39 | MRF24J40_ACKTMOUT_DRPACK);

  /* Set WAKETIME to recommended value for 100kHz SLPCLK Source.
   *
   * NOTE!!!: The datasheet specifies that WAKETIME > WAKECNT. It appears
   * that it is even sensitive to the order in which you set WAKECNT and
   * WAKETIME.  Meaning, if you set WAKECNT first and it goes higher than
   * WAKETIME, and then raise WAKETIME above WAKECNT, the device will not
   * function correctly.  Therefore, be careful when changing these registers
   */

  mrf24j40_setreg(dev->spi, MRF24J40_WAKETIMEL, 0xd2);
  mrf24j40_setreg(dev->spi, MRF24J40_WAKETIMEH, 0x00);

  /* Set WAKECNT (SLPACK 0x35<6:0>) value = 0x5f to set the main oscillator
   * (20 MHz) start-up timer value.
   */

  mrf24j40_setreg(dev->spi, MRF24J40_SLPACK,
                  (0x0c8 & MRF24J40_SLPACK_WAKECNT0_6));
  reg = mrf24j40_getreg(dev->spi, MRF24J40_RFCTL);
  reg &= ~MRF24J40_RFCTRL_WAKECNT7_8;
  reg |= ((0x0c8 >> 7) & 0x03) << 3;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);

  /* Enable the SLPIF and WAKEIF flags */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg &= ~(MRF24J40_INTCON_SLPIE | MRF24J40_INTCON_WAKEIE);
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

  mrf24j40_setorder(dev, 15, 15);

  dev->rxenabled = false;

  mrf24j40_setchannel(dev, 11);
  mrf24j40_setpanid(dev, g_allones);
  mrf24j40_setsaddr(dev, g_allones);
  mrf24j40_seteaddr(dev, g_allones);

  dev->max_frame_waittime = MRF24J40_DEFAULT_MAX_FRAME_WAITTIME;
  dev->bsn = 0;

  /* Default device params */

  cca.use_ed = 1;
  cca.use_cs = 0;
  cca.edth   = 0x60; /* CCA mode ED, no carrier sense, recommenced ED threshold -69 dBm */
  mrf24j40_setcca(dev, &cca);

  mrf24j40_setrxmode(dev, MRF24J40_RXMODE_NORMAL);

  mrf24j40_settxpower(dev, 0); /* 16. Set transmitter power. */

  mrf24j40_setpamode(dev, MRF24J40_PA_AUTO);

  return OK;
}

int mrf24j40_getattr(FAR struct ieee802154_radio_s *radio,
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
        break;

      case IEEE802154_ATTR_PHY_FCS_LEN:
        {
          attrval->phy.fcslen = 2;
          ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

      default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
    }

  return ret;
}

int mrf24j40_setattr(FAR struct ieee802154_radio_s *radio,
                            enum ieee802154_attr_e attr,
                            FAR const union ieee802154_attr_u *attrval)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int ret = IEEE802154_STATUS_SUCCESS;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          mrf24j40_setpanid(dev, attrval->mac.panid);
        }
        break;

      case IEEE802154_ATTR_MAC_SADDR:
        {
          mrf24j40_setsaddr(dev, attrval->mac.saddr);
        }
        break;

      case IEEE802154_ATTR_MAC_EADDR:
        {
          mrf24j40_seteaddr(dev, attrval->mac.eaddr);
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
          mrf24j40_setcoordsaddr(dev, attrval->mac.coordsaddr);
        }
        break;

      case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
          mrf24j40_setcoordeaddr(dev, attrval->mac.coordeaddr);
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
        }
        break;

      case IEEE802154_ATTR_PHY_CHAN:
        {
          mrf24j40_setchannel(dev, attrval->phy.chan);
        }
        break;

      case IEEE802154_ATTR_MAC_DEVMODE:
        {
          mrf24j40_setdevmode(dev, attrval->mac.devmode);
        }
        break;

      default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
        break;
    }

  return ret;
}

int mrf24j40_beaconstart(FAR struct ieee802154_radio_s *radio,
               FAR const struct ieee802154_superframespec_s *sfspec,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int reg;

  if (sfspec->pancoord)
    {
      /* Set the PANCOORD (RXMCR 0x00<3>) bit = 1to configure as
       * PAN coordinator
       */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_RXMCR);
      reg |= MRF24J40_RXMCR_PANCOORD;
      mrf24j40_setreg(dev->spi, MRF24J40_RXMCR, reg);

      /* Set the SLOTTED (TXMCR 0x11<5>) bit = 1 to use
       * Slotted CSMA-CA mode
       */

      reg = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);
      reg |= MRF24J40_TXMCR_SLOTTED;
      mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);

      /* Load the beacon frame into the TXBFIFO (0x080-0x0ff). */

      mrf24j40_setup_fifo(dev, beacon->bf_data, beacon->bf_len,
                          MRF24J40_BEACON_FIFO);

      /* The radio layer is responsible for setting the BSN. */

      dev->bsn = 0;
      mrf24j40_setreg(dev->spi, MRF24J40_BEACON_FIFO + 4, dev->bsn++);

      /* Set the TXBMSK (TXBCON1 0x25<7>) bit = 1 to mask the beacon
       * interrupt mask
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

      mrf24j40_setorder(dev, sfspec->beaconorder, sfspec->sforder);
    }
  else
    {
      return -ENOTTY;
    }

  return OK;
}

int mrf24j40_beaconupdate(FAR struct ieee802154_radio_s *radio,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;

  mrf24j40_setup_fifo(dev, beacon->bf_data, beacon->bf_len,
                      MRF24J40_BEACON_FIFO);
  mrf24j40_beacon_trigger(dev);

  return OK;
}

int mrf24j40_beaconstop(FAR struct ieee802154_radio_s *radio)
{
  return -ENOTTY;
}

int mrf24j40_sfupdate(FAR struct ieee802154_radio_s *radio,
             FAR const struct ieee802154_superframespec_s *sfspec)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  int reg;

  /* If we are operating on a beacon-enabled network, use slotted CSMA */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_TXMCR);
  if (sfspec->beaconorder < 15)
    {
      reg |= MRF24J40_TXMCR_SLOTTED;

      if (dev->devmode == IEEE802154_DEVMODE_ENDPOINT)
        {
          mrf24j40_setreg(dev->spi, MRF24J40_FRMOFFSET, 0x15);
        }
      else
        {
          mrf24j40_setreg(dev->spi, MRF24J40_FRMOFFSET, 0x00);
        }
    }
  else
    {
      reg &= ~MRF24J40_TXMCR_SLOTTED;
    }

  mrf24j40_setreg(dev->spi, MRF24J40_TXMCR, reg);

  mrf24j40_setorder(dev, sfspec->beaconorder, sfspec->sforder);

  /* Program the CAP end slot (ESLOTG1 0x13<3:0>) value. */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_ESLOTG1);
  reg &= ~MRF24J40_ESLOTG1_CAP;
  reg |= sfspec->final_capslot & MRF24J40_ESLOTG1_CAP;
  mrf24j40_setreg(dev->spi, MRF24J40_ESLOTG1, reg);

  return OK;
}
