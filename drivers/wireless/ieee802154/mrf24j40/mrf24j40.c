/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40.c
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mrf24j40.h"
#include "mrf24j40_reg.h"
#include "mrf24j40_radif.h"
#include "mrf24j40_getset.h"
#include "mrf24j40_regops.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if 0
static int mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev,
                                 FAR uint8_t *energy);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

#if 0
static int mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev,
                                 FAR uint8_t *energy)
{
  uint8_t reg;

  /* Manually enable the LNA */

  mrf24j40_setpamode(dev, MRF24J40_PA_ED);

  /* Set RSSI average duration to 8 symbols */

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXBCON1);
  reg |= 0x30;
  mrf24j40_setreg(dev->spi, MRF24J40_TXBCON1, reg);

  /* 1. Set RSSIMODE1 0x3E<7> – Initiate RSSI calculation. */

  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x80);

  /* 2. Wait until RSSIRDY 0x3E<0> is set to ‘1’ – RSSI calculation is
   *    complete.
   */

  while (!(mrf24j40_getreg(dev->spi, MRF24J40_BBREG6) & 0x01));

  /* 3. Read RSSI 0x210<7:0> – The RSSI register contains the averaged RSSI
   *    received power level for 8 symbol periods.
   */

  *energy = mrf24j40_getreg(dev->spi, MRF24J40_RSSI);
  mrf24j40_setreg(dev->spi, MRF24J40_BBREG6, 0x40);

  /* Back to automatic control */

  mrf24j40_setpamode(dev, MRF24J40_PA_AUTO);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: mrf24j40_dopoll_csma
 *
 * Description:
 *   This function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mrf24j40_txdone_csma),
 *   2. When new TX data is available (mrf24j40_txnotify), and
 *   3. After a TX timeout to restart the sending process
 *      (mrf24j40_txtimeout_csma).
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

void mrf24j40_dopoll_csma(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int len = 0;

  /* Get exclusive access to the driver */

  while (nxsem_wait(&dev->exclsem) < 0)
    {
    }

  /* If this a CSMA transaction and we have room in the CSMA fifo */

  if (!dev->csma_busy)
    {
      wlinfo("Polling for frame\n");
      len = dev->radiocb->poll(dev->radiocb, false, &dev->csma_desc);

      if (len > 0)
        {
          wlinfo("Frame received. Frame length: %d\n", len);

          /* Now the txdesc is in use */

          dev->csma_busy = 1;

          /* Setup the transaction on the device in the CSMA FIFO */

          mrf24j40_norm_setup(dev, dev->csma_desc->frame, true);
          mrf24j40_norm_trigger(dev);
        }
    }

  nxsem_post(&dev->exclsem);
}

/****************************************************************************
 * Function: mrf24j40_dopoll_gts
 *
 * Description:
 *   This function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. After completion of a transmission (mrf24j40_txdone_gts),
 *   2. When new TX data is available (mrf24j40_txnotify), and
 *   3. After a TX timeout to restart the sending process
 *      (mrf24j40_txtimeout_gts).
 *
 * Input Parameters:
 *   arg  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mrf24j40_dopoll_gts(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int gts = 0;
  int len = 0;

  /* Get exclusive access to the driver */

  while (nxsem_wait(&dev->exclsem) < 0)
    {
    }

  for (gts = 0; gts < MRF24J40_GTS_SLOTS; gts++)
    {
      if (!dev->gts_busy[gts])
        {
          len = dev->radiocb->poll(dev->radiocb, true, &dev->gts_desc[gts]);

          if (len > 0)
            {
              /* Now the txdesc is in use */

              dev->gts_busy[gts] = 1;

              /* Setup the transaction on the device in the open GTS FIFO */

              mrf24j40_gts_setup(dev, gts, dev->gts_desc[gts]->frame);
            }
        }
    }

  nxsem_post(&dev->exclsem);
}

/****************************************************************************
 * Name: mrf24j40_norm_setup
 *
 * Description:
 *   Setup a transaction in the normal TX FIFO
 *
 ****************************************************************************/

void mrf24j40_norm_setup(FAR struct mrf24j40_radio_s *dev,
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

  mrf24j40_setup_fifo(dev, frame->io_data, frame->io_len,
                      MRF24J40_TXNORM_FIFO);

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

void mrf24j40_norm_trigger(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXNCON);
  reg |= MRF24J40_TXNCON_TXNTRIG;
  mrf24j40_setreg(dev->spi, MRF24J40_TXNCON, reg);
}

/****************************************************************************
 * Name: mrf24j40_beacon_trigger
 *
 * Description:
 *   Trigger the beacon TX FIFO
 *
 ****************************************************************************/

void mrf24j40_beacon_trigger(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg  = mrf24j40_getreg(dev->spi, MRF24J40_TXBCON0);

  reg |= MRF24J40_TXBCON0_TXBTRIG;

  mrf24j40_setreg(dev->spi, MRF24J40_TXBCON0, reg);
}

/****************************************************************************
 * Name: mrf24j40_gts_setup
 *
 * Description:
 *   Setup a GTS transaction in one of the GTS FIFOs
 *
 ****************************************************************************/

void mrf24j40_gts_setup(FAR struct mrf24j40_radio_s *dev, uint8_t fifo,
                        FAR struct iob_s *frame)
{
}

/****************************************************************************
 * Name: mrf24j40_setup_fifo
 *
 * Description:
 *
 ****************************************************************************/

void mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev,
                         FAR const uint8_t *buf, uint8_t length,
                         uint32_t fifo_addr)
{
  uint16_t frame_ctrl;
  uint16_t addrmode;
  int hlen = 3; /* Include frame control and seq number */
  int i;

  /* Analyze frame control to compute header length */

  frame_ctrl = buf[0];
  frame_ctrl |= (buf[1] << 8);

  addrmode = (frame_ctrl & IEEE802154_FRAMECTRL_DADDR) >>
             IEEE802154_FRAMECTRL_SHIFT_DADDR;

  if (addrmode == IEEE802154_ADDRMODE_SHORT)
    {
      hlen += 2 + 2; /* Destination PAN + shortaddr */
    }
  else if (addrmode == IEEE802154_ADDRMODE_EXTENDED)
    {
      hlen += 2 + 8; /* Destination PAN + extaddr */
    }

  if (!(frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP))
    {
      hlen += 2; /* No PAN compression, source PAN is different from dest PAN */
    }

  addrmode = (frame_ctrl & IEEE802154_FRAMECTRL_SADDR) >>
             IEEE802154_FRAMECTRL_SHIFT_SADDR;

  if (addrmode == IEEE802154_ADDRMODE_SHORT)
    {
      hlen += 2; /* Source saddr */
    }
  else if (addrmode == IEEE802154_ADDRMODE_EXTENDED)
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_init
 *
 * Description:
 *   Return an mrf24j40 device for use by other drivers.
 *
 ****************************************************************************/

FAR struct ieee802154_radio_s *
  mrf24j40_init(FAR struct spi_dev_s *spi,
                FAR const struct mrf24j40_lower_s *lower)
{
  FAR struct mrf24j40_radio_s *dev;

  dev = kmm_zalloc(sizeof(struct mrf24j40_radio_s));
  if (dev == NULL)
    {
      return NULL;
    }

  /* Attach irq */

  if (lower->attach(lower, mrf24j40_interrupt, dev) != OK)
    {
#if 0
      kmm_free(dev);
#endif
      return NULL;
    }

  /* Allow exclusive access to the privmac struct */

  nxsem_init(&dev->exclsem, 0, 1);

  dev->radio.bind         = mrf24j40_bind;
  dev->radio.reset        = mrf24j40_reset;
  dev->radio.getattr      = mrf24j40_getattr;
  dev->radio.setattr      = mrf24j40_setattr;
  dev->radio.txnotify     = mrf24j40_txnotify;
  dev->radio.txdelayed    = mrf24j40_txdelayed;
  dev->radio.rxenable     = mrf24j40_rxenable;
  dev->radio.energydetect = mrf24j40_energydetect;
  dev->radio.beaconstart  = mrf24j40_beaconstart;
  dev->radio.beaconupdate = mrf24j40_beaconupdate;
  dev->radio.beaconstop   = mrf24j40_beaconstop;
  dev->radio.sfupdate     = mrf24j40_sfupdate;

  dev->lower    = lower;
  dev->spi      = spi;

  mrf24j40_reset(&dev->radio);

  dev->lower->enable(dev->lower, true);
  return &dev->radio;
}
