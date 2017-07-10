/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_radif.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>

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
 * Private Data
 ****************************************************************************/

static const uint8_t g_allones[8] =
{
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/****************************************************************************
 * Radio Interface Functions
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
 * Parameters:
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

int mrf24j40_txdelayed(FAR struct ieee802154_radio_s *radio,
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
   * FIXME: I think this could be done cleaner.
   */

  if (!work_available(&dev->irqwork))
    {
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

int mrf24j40_reset(FAR struct ieee802154_radio_s *radio)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
  struct ieee802154_cca_s   cca;
  int reg;

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

  /* Set this in reset since it can exist for all device modes. See pg 101 */

  mrf24j40_setreg(dev->spi, MRF24J40_FRMOFFSET, 0x15);

  /* For now, we want to always just have the frame pending bit set when
   * acknowledging a Data Request command. The standard says that the coordinator
   * can do this if it needs time to figure out whether it has data or not
   */

  mrf24j40_setreg(dev->spi, MRF24J40_ACKTMOUT, 0x39 | MRF24J40_ACKTMOUT_DRPACK);

  /* Set WAKECNT (SLPACK 0x35<6:0>) value = 0xC8 to set the main oscillator
   * (20 MHz) start-up timer value.
   */

  mrf24j40_setreg(dev->spi, MRF24J40_SLPACK, 0xC8);

  /* Set WAKETIME to recommended value for 100kHz SLPCLK Source */

  mrf24j40_setreg(dev->spi, MRF24J40_WAKETIMEL, 0xD2);
  mrf24j40_setreg(dev->spi, MRF24J40_WAKETIMEH, 0x00);

  /* Enable the SLPIF and WAKEIF flags */

  reg = mrf24j40_getreg(dev->spi, MRF24J40_INTCON);
  reg &= ~(MRF24J40_INTCON_SLPIE | MRF24J40_INTCON_WAKEIE);
  mrf24j40_setreg(dev->spi, MRF24J40_INTCON, reg);

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

  mrf24j40_settxpower(dev, 0); /*16. Set transmitter power .*/

  mrf24j40_pacontrol(dev, MRF24J40_PA_AUTO);

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

int mrf24j40_beaconstart(FAR struct ieee802154_radio_s *radio,
               FAR const struct ieee802154_superframespec_s *sfspec,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)radio;
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

      /* The radio layer is responsible for setting the BSN. */

      dev->bsn = 0;
      mrf24j40_setreg(dev->spi, MRF24J40_BEACON_FIFO + 4, dev->bsn++);

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

  mrf24j40_setup_fifo(dev, beacon->bf_data, beacon->bf_len, MRF24J40_BEACON_FIFO);
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
