/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_interrupt.c
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

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>

#include "mrf24j40.h"
#include "mrf24j40_reg.h"
#include "mrf24j40_regops.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mrf24j40_irqwork_rx(FAR struct mrf24j40_radio_s *dev);
static void mrf24j40_irqwork_txnorm(FAR struct mrf24j40_radio_s *dev);
static void mrf24j40_irqwork_txgts(FAR struct mrf24j40_radio_s *dev,
              uint8_t gts_num);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
      /* The number of retries of the most recent transmission is contained
       * in the TXNRETRY (TXSTAT 0x24<7:6>) bits.
       * The CCAFAIL (TXSTAT 0x24<5>) bit = 1 indicates if the failed
       * transmission was due to the channel busy (CSMA-CA timed out).
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

      nxsem_post(&dev->exclsem);
      mrf24j40_dopoll_csma(dev);
      while (nxsem_wait(&dev->exclsem) < 0)
        {
        }
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

  dev->gts_busy[gts] = 0;

  mrf24j40_dopoll_gts(dev);
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
  FAR struct ieee802154_primitive_s *primitive;
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

  primitive = ieee802154_primitive_allocate();
  ind = (FAR struct ieee802154_data_ind_s *)primitive;
  if (ind == NULL)
    {
      wlerr("ERROR: Unable to allocate data_ind. Discarding frame\n");
      goto done;
    }

  primitive->type = IEEE802154_PRIMITIVE_IND_DATA;

  /* Allocate an IOB to put the frame into */

  ind->frame = iob_alloc(false, IOBUSER_WIRELESS_RAD802154);
  ind->frame->io_flink = NULL;
  ind->frame->io_len = 0;
  ind->frame->io_pktlen = 0;
  ind->frame->io_offset = 0;

  /* Read packet */

  addr = MRF24J40_RXBUF_BASE;

  ind->frame->io_len = mrf24j40_getreg(dev->spi, addr++);

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Input Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mrf24j40_irqworker(FAR void *arg)
{
  FAR struct mrf24j40_radio_s *dev = (FAR struct mrf24j40_radio_s *)arg;
  uint8_t intstat;
  uint8_t reg;

  DEBUGASSERT(dev);
  DEBUGASSERT(dev->spi);

  /* Get exclusive access to the driver */

  while (nxsem_wait(&dev->exclsem) < 0)
    {
    }

  /* Read and store INTSTAT - this clears the register. */

  intstat = mrf24j40_getreg(dev->spi, MRF24J40_INTSTAT);

  /* Do work according to the pending interrupts */

  if ((intstat & MRF24J40_INTSTAT_HSYMTMRIF))
    {
      /* As of now the only use for the MAC timer is for delayed
       * transactions.  Therefore, all we do here is trigger the TX norm FIFO
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
      /* A packet was transmitted or failed */

      mrf24j40_irqwork_txnorm(dev);
    }

  if ((intstat & MRF24J40_INTSTAT_TXG1IF))
    {
      /* A packet was transmitted or failed */

      mrf24j40_irqwork_txgts(dev, 0);
    }

  if ((intstat & MRF24J40_INTSTAT_TXG1IF))
    {
      /* A packet was transmitted or failed */

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

  if ((intstat & MRF24J40_INTSTAT_WAKEIF))
    {
#ifdef CONFIG_MAC802154_SFEVENT_VERBOSE
      wlinfo("Wake Interrupt\n");
#endif

      if (dev->devmode != IEEE802154_DEVMODE_ENDPOINT)
        {
          /* This is right before the beacon, we set the bsn here, since the
           * MAC uses the SLPIF (end of active portion of superframe). to
           * make any changes to the beacon.  This assumes that any changes
           * to the beacon be in by the time that this interrupt fires.
           */

          mrf24j40_setreg(dev->spi, MRF24J40_BEACON_FIFO + 4, dev->bsn++);
          mrf24j40_beacon_trigger(dev);
          wlinfo("Beacon triggered. BSN: 0x%02X\n", dev->bsn - 1);
        }
    }

  /* Unlock the radio device */

  nxsem_post(&dev->exclsem);

  /* Re-enable GPIO interrupts */

  dev->lower->enable(dev->lower, true);
}

/****************************************************************************
 * Name: mrf24j40_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

int mrf24j40_interrupt(int irq, FAR void *context, FAR void *arg)
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
  return work_queue(HPWORK, &dev->irqwork,
                    mrf24j40_irqworker, (FAR void *)dev, 0);
}
