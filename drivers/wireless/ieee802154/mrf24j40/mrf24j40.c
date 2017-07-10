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

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mrf24j40.h"
#include "mrf24j40_reg.h"
#include "mrf24j40_radif.h"
#include "mrf24j40_regops.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Internal Functions
 ****************************************************************************/

void mrf24j40_mactimer(FAR struct mrf24j40_radio_s *dev, int numsymbols)
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

void mrf24j40_dopoll_csma(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  int len = 0;

  /* Get exclusive access to the driver */

  while (sem_wait(&dev->exclsem) != 0) { }

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

void mrf24j40_dopoll_gts(FAR void *arg)
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
 * Name: mrf24j40_setorder
 *
 * Description:
 *   Configures the timers and sets the ORDER register
 ****************************************************************************/

void mrf24j40_setorder(FAR struct mrf24j40_radio_s *dev, uint8_t bo, uint8_t so)
{
  uint32_t maincnt = 0;
  uint32_t slpcal = 0;

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

  /* Program the Beacon Interval into the Main Counter, MAINCNT (0x229<1:0>,
    * 0x228, 0x227, 0x226), and Remain Counter, REMCNT (0x225, 0x224),
    * according to BO and SO values. Refer to Section 3.15.1.3 “Sleep Mode
    * Counters”
    */


  mrf24j40_setreg(dev->spi, MRF24J40_REMCNTL, (MRF24J40_REMCNT & 0xFF));
  mrf24j40_setreg(dev->spi, MRF24J40_REMCNTH, ((MRF24J40_REMCNT >> 8) & 0xFF));

  maincnt = MRF24J40_MAINCNT(bo, (slpcal * 50 / 16));

  mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT0, (maincnt & 0xFF));
  mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT1, ((maincnt >> 8)  & 0xFF));
  mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT2, ((maincnt >> 16) & 0xFF));
  mrf24j40_setreg(dev->spi, MRF24J40_MAINCNT3, ((maincnt >> 24) & 0x03));

  /* Configure the BO (ORDER 0x10<7:4>) and SO (ORDER 0x10<3:0>) values.
    * After configuring BO and SO, the beacon frame will be sent immediately.
    */

  mrf24j40_setreg(dev->spi, MRF24J40_ORDER, ((bo << 4) & 0xF0) | (so & 0x0F));
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

int mrf24j40_pacontrol(FAR struct mrf24j40_radio_s *dev, int mode)
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
 * Name: mrf24j40_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

int mrf24j40_energydetect(FAR struct mrf24j40_radio_s *dev, FAR uint8_t *energy)
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

void mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev, FAR const uint8_t *buf,
                         uint8_t length, uint32_t fifo_addr)
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

/****************************************************************************
 * Name: mrf24j40_resetrfsm
 *
 * Description:
 *   Reset the RF state machine. Required at boot, after channel change,
 *   and probably after PA settings.
 *
 ****************************************************************************/

void mrf24j40_resetrfsm(FAR struct mrf24j40_radio_s *dev)
{
  uint8_t reg;

  reg = mrf24j40_getreg(dev->spi, MRF24J40_RFCTL);
  reg |= 0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);

  reg &= ~0x04;
  mrf24j40_setreg(dev->spi, MRF24J40_RFCTL, reg);
  up_udelay(200);
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
  dev->radio.reset        = mrf24j40_reset;
  dev->radio.getattr      = mrf24j40_getattr;
  dev->radio.setattr      = mrf24j40_setattr;
  dev->radio.txnotify     = mrf24j40_txnotify;
  dev->radio.txdelayed    = mrf24j40_txdelayed;
  dev->radio.rxenable     = mrf24j40_rxenable;
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

