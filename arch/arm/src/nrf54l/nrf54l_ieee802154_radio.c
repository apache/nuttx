/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ieee802154_radio.c
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

#include <nuttx/debug.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>

#include "nrf54l_ieee802154_radio.h"
#include "nrf54l_ieee802154_trace.h"

#include "nrf54l_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Constant from NRF54L manual */

#define NRF54L_ED_RSSISCALE            (4)

/* One ED iteration lasts 8 symbols (128 us) */

#define NRF54L_ED_ITER_SYMBOLS         (8)

/* ED configuration:
 *   - IRQ on EDEND event (sampling of energy detection complete)
 *   - shortcut between READY and EDSTART
 *   - shortcut between EDEND and DISABLE
 *
 * Set in nrf54l_radioi8_energydetect() reset in nrf54l_radioi8_isr_radio().
 *
 * EDSTART task set in nrf54l_radioi8_energydetect().
 */

#define IEEE802154_ED_INT             (RADIO_INT_EDEND)
#define IEEE802154_ED_SHORTS          (RADIO_SHORTS_READY_EDSTART | \
                                       RADIO_SHORTS_EDEND_DISABLE)

/* RX configuration:
 *   - IRQ on END event (packet recveived)
 *   - shortcut between RXREADY and START
 *   - shortcut between ADDRESS and RSSISTART
 *
 * Set in nrf54l_radioi8_rxenable() reset in nrf54l_radioi8_rxenable().
 *
 * RXEN task set in nrf54l_radioi8_rxenable() and
 * nrf54l_radioi8_isr_radio() after RX handled and no pending ACKTX.
 */

#define IEEE802154_RX_INT              (RADIO_INT_END)
/* There is no END to DISABLE shortcut, RADIO is disabled in the ISR */

#define IEEE802154_RX_SHORTS           (RADIO_SHORTS_RXREADY_START |  \
                                        RADIO_SHORTS_ADDRESS_RSSISTART)

/* NOTE: for TX we trigger interrupts on PHYEND event, not END! */

/* TX CCA un-slotted configuration:
 *   - IRQ on PHYEND event
 *   - IRQ on CCABUSY event
 *   - shortcut between TXREADY and START
 *   - shortcut between PHYEND and DISABLE
 *   - shortcut between RXREADY and CCASTART
 *   - shortcut between CCAIDLE and STOP
 *   - shortcut between CCAIDLE and TXEN
 *
 * Set in nrf54l_radioi8_radio_norm_setup() reset in
 * nrf54l_radioi8_isr_radio().
 *
 * CCASTART task set in nrf54l_radioi8_isr_tim() when CCA transfer.
 */

#define IEEE802154_TXCCAUNSLT_INT      (RADIO_INT_PHYEND |              \
                                        RADIO_INT_CCABUSY)
#define IEEE802154_TXCCAUNSLT_SHORTS   (RADIO_SHORTS_TXREADY_START |    \
                                        RADIO_SHORTS_PHYEND_DISABLE |   \
                                        RADIO_SHORTS_RXREADY_CCASTART | \
                                        RADIO_SHORTS_CCAIDLE_STOP |     \
                                        RADIO_SHORTS_CCAIDLE_TXEN)

/* TX CCA slotted configuration:
 *   - IRQ on PHYEND event
 *   - IRQ on CCABUSY event
 *   - IRQ on CCAIDLE event
 *   - shortcut between TXREADY and START
 *   - shortcut between PHYEND and DISABLE
 *
 * Set in nrf54l_radioi8_radio_norm_setup() reset in
 * nrf54l_radioi8_isr_radio().
 *
 * CCASTART task set in nrf54l_radioi8_isr_tim() when CCA transfer.
 */

#define IEEE802154_TXCCASLT_INT        (RADIO_INT_PHYEND |              \
                                        RADIO_INT_CCAIDLE |             \
                                        RADIO_INT_CCABUSY)
#define IEEE802154_TXCCASLT_SHORTS     (RADIO_SHORTS_TXREADY_START |    \
                                        RADIO_SHORTS_PHYEND_DISABLE |   \
                                        RADIO_SHORTS_RXREADY_CCASTART)

/* TX no-CCA configuration:
 *   - IRQ on PHYEND event
 *   - shortcut between TXREADY and START
 *   - shortcut between PHYEND and DISABLE
 *
 * Set in nrf54l_radioi8_radio_norm_setup() reset in
 * nrf54l_radioi8_isr_radio().
 *
 * TXEN task set in nrf54l_radioi8_radio_norm_trigger() when non-CCA
 * transfer or in nrf54l_radioi8_isr_tim() for TXDELAY.
 */

#define IEEE802154_TX_INT              (RADIO_INT_PHYEND)
#define IEEE802154_TX_SHORTS           (RADIO_SHORTS_TXREADY_START |  \
                                        RADIO_SHORTS_PHYEND_DISABLE)

/* ACK configuration:
 *   - IRQ on PHYEND event
 *   - shortcut between PHYEND and DISABLE
 *
 * Set in nrf54l_radioi8_ack_transmit reset in nrf54l_radioi8_isr_radio().
 *
 * TXEN task set in nrf54l_radioi8_ack_transmit().
 * START task set in nrf54l_radioi8_isr_tim().
 */

#define IEEE802154_ACKTX_INT            (RADIO_INT_PHYEND)
#define IEEE802154_ACKTX_SHORTS         (RADIO_SHORTS_PHYEND_DISABLE)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf54l_radioi8_rx_parse(struct nrf54l_radioi8_dev_s *dev,
                                   uint8_t *ftype, uint8_t *cmdtype);
static void nrf54l_radioi8_ack_transmit(struct nrf54l_radioi8_dev_s *dev);
static bool nrf54l_radioi8_filter(struct nrf54l_radioi8_dev_s *dev);

/* Ops */

static void nrf54l_radioi8_txstart(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_ccastart(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_notify_noack(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_rxenable(struct nrf54l_radioi8_dev_s *dev,
                                  bool enable);
static int nrf54l_radioi8_energydetect(struct nrf54l_radioi8_dev_s *dev,
                                      uint32_t nsymbols);
static int nrf54l_radioi8_setchannel(struct nrf54l_radioi8_dev_s *dev,
                                    uint8_t chan);
static int nrf54l_radioi8_setcca(struct nrf54l_radioi8_dev_s *dev,
                                struct ieee802154_cca_s *cca);
static void nrf54l_radioi8_norm_setup_buf(struct nrf54l_radioi8_dev_s *dev,
                                         uint8_t *buf, bool csma);
static void nrf54l_radioi8_norm_setup(struct nrf54l_radioi8_dev_s *dev,
                                     struct iob_s *frame, bool csma);
static void nrf54l_radioi8_norm_trigger(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_reset(struct nrf54l_radioi8_radio_s *dev);
static int nrf54l_radioi8_csmapoll(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_gtspoll(struct nrf54l_radioi8_dev_s *dev);

/* Interrupts logic */

static void nrf54l_radioi8_txnext(struct nrf54l_radioi8_dev_s *dev,
                                 uint32_t mindelay);
static void nrf54l_radioi8_work_noack(void *arg);
static void nrf54l_radioi8_work_rx(void *arg);
static void nrf54l_radioi8_work_tx(void *arg);
static void nrf54l_radioi8_work_busy(void *arg);
static void nrf54l_radioi8_work_ed(void *arg);
static void nrf54l_radioi8_state_rx(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_state_tx(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_state_acktx(struct nrf54l_radioi8_dev_s *dev);
static void nrf54l_radioi8_state_ed(struct nrf54l_radioi8_dev_s *dev);
static int nrf54l_radioi8_isr_radio(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Packet buffers - must be byte aligned in RAM.
 *
 * NOTE: The first byte is PHR, the last byte is LQI.
 */

static uint8_t aligned_data(8)
  g_nrf54l_radioi8_rxbuf[IEEE802154_MAX_PHY_PACKET_SIZE + 2];

static uint8_t aligned_data(8)
  g_nrf54l_radioi8_txbuf[IEEE802154_MAX_PHY_PACKET_SIZE + 2];

static uint8_t aligned_data(8)
  g_nrf54l_radioi8_ackbuf[IEEE802154_ACK_FRAME_SIZE + 1];

/* Radio ops */

static struct nrf54l_radioi8_radio_ops_s g_radioi8_radio_ops =
{
  .txstart      = nrf54l_radioi8_txstart,
  .ccastart     = nrf54l_radioi8_ccastart,
  .notify_noack = nrf54l_radioi8_notify_noack,
  .rxenable     = nrf54l_radioi8_rxenable,
  .energydetect = nrf54l_radioi8_energydetect,
  .setchannel   = nrf54l_radioi8_setchannel,
  .setcca       = nrf54l_radioi8_setcca,
  .norm_setup   = nrf54l_radioi8_norm_setup,
  .norm_trigger = nrf54l_radioi8_norm_trigger,
  .txnext       = nrf54l_radioi8_txnext,
  .reset        = nrf54l_radioi8_reset,
  .csma_poll    = nrf54l_radioi8_csmapoll,
  .gts_poll     = nrf54l_radioi8_gtspoll
};

/* Radio interface */

static struct nrf54l_radioi8_radio_s g_radioi8_radio;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_rx_parse
 *
 * Description:
 *   Get the RX frame type and command type if present.
 *
 ****************************************************************************/

static void nrf54l_radioi8_rx_parse(struct nrf54l_radioi8_dev_s *dev,
                                   uint8_t *ftype, uint8_t *cmdtype)
{
  uint16_t *frame_ctrl = NULL;
  uint8_t   smode      = 0;
  uint8_t   dmode      = 0;
  uint8_t   panid_comp = 0;
  int       i          = 0;

  /* Frame starts from offset 1 */

  i = 1;

  /* Frame ctrl */

  frame_ctrl = (uint16_t *)&dev->radio->rxbuf[i];
  i += sizeof(uint16_t);

  /* Frame seq */

  i += 1;

  /* Return now if a given frame is not a command frame */

  *ftype = ((*frame_ctrl & IEEE802154_FRAMECTRL_FTYPE) >>
            IEEE802154_FRAMECTRL_SHIFT_FTYPE);
  if (*ftype != IEEE802154_FRAME_COMMAND)
    {
      return;
    }

  /* Now we have to find the offset for the frame command type */

  smode = ((*frame_ctrl & IEEE802154_FRAMECTRL_SADDR) >>
           IEEE802154_FRAMECTRL_SHIFT_SADDR);

  dmode = ((*frame_ctrl & IEEE802154_FRAMECTRL_DADDR) >>
           IEEE802154_FRAMECTRL_SHIFT_DADDR);

  panid_comp = ((*frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP) >>
                IEEE802154_FRAMECTRL_SHIFT_PANIDCOMP);

  if (dmode != IEEE802154_ADDRMODE_NONE)
    {
      i += IEEE802154_PANIDSIZE;

      if (dmode == IEEE802154_ADDRMODE_SHORT)
        {
          i += IEEE802154_SADDRSIZE;
        }
      else if (dmode == IEEE802154_ADDRMODE_EXTENDED)
        {
          i += IEEE802154_EADDRSIZE;
        }
    }

  if (smode != IEEE802154_ADDRMODE_NONE)
    {
      /* PAN ID compression applies only when both addresses are present */

      if (!panid_comp || dmode == IEEE802154_ADDRMODE_NONE)
        {
          i += IEEE802154_PANIDSIZE;
        }

      if (smode == IEEE802154_ADDRMODE_SHORT)
        {
          i += IEEE802154_SADDRSIZE;
        }
      else if (smode == IEEE802154_ADDRMODE_EXTENDED)
        {
          i += IEEE802154_EADDRSIZE;
        }
    }

  *cmdtype = dev->radio->rxbuf[i];
}

/****************************************************************************
 * Name: nrf54l_radioi8_ack_transmit
 *
 * Description:
 *   Transmit ACK.
 *
 ****************************************************************************/

static void nrf54l_radioi8_ack_transmit(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radio_dev_s *lower   = NULL;
  uint8_t                  *ackbuf  = dev->radio->ackbuf;
  uint32_t                  regval  = 0;
  uint8_t                   ftype   = 0;
  uint8_t                   cmdtype = 0;
  bool                      pending = false;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  /* Restore RX after ACK - we already are in DISABLE state so no need to
   * disable RX
   */

  if (dev->radio->state.rxenabled)
    {
      dev->radio->state.rxrestore = true;
    }

  /* Clear events */

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_READY_OFFSET, 0);
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_TXREADY_OFFSET, 0);
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_RXREADY_OFFSET, 0);

  /* Fill ACK buffer */

  ackbuf[0] = IEEE802154_ACK_FRAME_SIZE;
  ackbuf[1] = 0x02;
  ackbuf[2] = 0;
  ackbuf[3] = dev->radio->rxbuf[3];

  /* Get RX frame type */

  nrf54l_radioi8_rx_parse(dev, &ftype, &cmdtype);

  /* If this is ACK for Data Request command - set frame pending flag */

  if (dev->state.devmode != NRF54L_DEVMODE_ENDPOINT &&
      ftype == IEEE802154_FRAME_COMMAND &&
      cmdtype == IEEE802154_CMD_DATA_REQ)
    {
      pending = true;
    }

  /* Set frame pending flag for this ACK */

  if (pending)
    {
      ackbuf[1] |= 0x10;
    }

  /* Configure shorts and interrupts */

  NRF54L_RADIO_SHRTSET(lower, IEEE802154_ACKTX_SHORTS);
  NRF54L_RADIO_INTEN(lower, IEEE802154_ACKTX_INT);

  /* Set packet pointer - buffer must be aligned */

  regval = (uint32_t)ackbuf;
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_PACKETPTR_OFFSET, regval);

  /* Set state to ACK */

  dev->radio->state.state = NRF54L_RADIO_STATE_ACKTX;

  /* Enable TX - start TX is called from timer interrupt */

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_TXEN_OFFSET, 1);

  /* Run TIMER - TX is handled in timer isr */

  nrf54l_radioi8_trace_put(RADIO_TRACE_ACKTX, pending);
  dev->tim->ops->setup(dev, NRF54L_TIMER_CHAN_ACK,
                       IEEE802154_ACKIFS_SYMBOLS);
}

/****************************************************************************
 * Name: nrf54l_radioi8_filter
 *
 * Description:
 *   Filter received frames.
 *
 ****************************************************************************/

static bool nrf54l_radioi8_filter(struct nrf54l_radioi8_dev_s *dev)
{
  struct ieee802154_addr_s  addr;
  uint8_t                  *rx = &dev->radio->rxbuf[1];
  uint16_t                 *fc = (uint16_t *)&rx[0];
  int                       i;

  /* Promiscuous mode */

  if (dev->state.rxmode == NRF54L_RXMODE_PROMISC)
    {
      return true;
    }

  /* Beacon frame */

  if (((*fc & IEEE802154_FRAMECTRL_FTYPE) == IEEE802154_FRAME_BEACON) &&
      IEEE802154_PANIDCMP(IEEE802154_PANID_UNSPEC, &dev->state.addr.panid))
    {
      return true;
    }

  /* Get destination address */

  IEEE802154_PANIDCOPY(addr.panid, &rx[3]);
  addr.mode = ((*fc & IEEE802154_FRAMECTRL_DADDR) >>
               IEEE802154_FRAMECTRL_SHIFT_DADDR);
  if (addr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      IEEE802154_SADDRCOPY(addr.saddr, &rx[5]);
    }
  else if (addr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      /* Sent least significant octet first, eaddr[0] is the most
       * significant octet
       */

      for (i = 0; i < IEEE802154_EADDRSIZE; i++)
        {
          addr.eaddr[i] = rx[5 + IEEE802154_EADDRSIZE - 1 - i];
        }
    }

  /* PAN ID match */

  if (!IEEE802154_PANIDCMP(IEEE802154_PANID_UNSPEC, dev->state.addr.panid) &&
      !IEEE802154_PANIDCMP(addr.panid, &dev->state.addr.panid))
    {
      return false;
    }

  /* Destination address match */

  if (addr.mode == IEEE802154_ADDRMODE_SHORT)
    {
      if (IEEE802154_SADDRCMP(addr.saddr,
                              &dev->state.addr.saddr))
        {
          return true;
        }

      else if (IEEE802154_SADDRCMP(addr.saddr,
                                   IEEE802154_SADDR_UNSPEC))
        {
          return true;
        }
    }
  else if (addr.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      if (IEEE802154_EADDRCMP(addr.eaddr,
                              &dev->state.addr.eaddr))
        {
          return true;
        }

      else if (IEEE802154_EADDRCMP(addr.eaddr,
                                   IEEE802154_EADDR_UNSPEC))
        {
          return true;
        }
    }

  /* Otherwise drop this frame */

  return false;
}

/****************************************************************************
 * Name: nrf54l_radioi8_txstart
 *
 * Description:
 *   Start transmission, TX must be armed (TXEN must be set).
 *
 ****************************************************************************/

static void nrf54l_radioi8_txstart(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radio_dev_s *lower = NULL;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_START_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf54l_radioi8_ccastart
 *
 * Description:
 *   Start CCA, RX must be armed (RXEN must be set).
 *
 ****************************************************************************/

static void nrf54l_radioi8_ccastart(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radio_dev_s *lower = NULL;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_CCASTART_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf54l_radioi8_notify_noack
 *
 * Description:
 *   Notify Radio layer about noack event.
 *
 ****************************************************************************/

static void nrf54l_radioi8_notify_noack(struct nrf54l_radioi8_dev_s *dev)
{
  /* If flag is set - no ACK was received */

  if (dev->radio->state.waitack == true)
    {
      /* Notify MAC layer */

      DEBUGASSERT(work_available(&dev->radio->noackwork));
      work_queue(HPWORK, &dev->radio->noackwork,
                 nrf54l_radioi8_work_noack, (void *)dev, 0);

      /* Clear flag */

      dev->radio->state.waitack = false;
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_rxenable
 *
 * Description:
 *  Enable/Disable receiver.
 *
 ****************************************************************************/

static int nrf54l_radioi8_rxenable(struct nrf54l_radioi8_dev_s *dev,
                                  bool enable)
{
  struct nrf54l_radio_dev_s *lower  = NULL;
  uint32_t                  regval = 0;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  if (enable)
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_RXENABLE, 0);

      /* Clear events */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_END_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_READY_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_PHYEND_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_TXREADY_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_RXREADY_OFFSET, 0);

      /* Configure shorts and interrupts */

      NRF54L_RADIO_SHRTSET(lower, IEEE802154_RX_SHORTS);
      NRF54L_RADIO_INTEN(lower, IEEE802154_RX_INT);

      /* Set packet pointer - buffer must be aligned */

      regval = (uint32_t)dev->radio->rxbuf;
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_PACKETPTR_OFFSET, regval);

      /* Set state to RX */

      dev->radio->state.state = NRF54L_RADIO_STATE_RX;

      /* Start RX */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_RXEN_OFFSET, 1);
    }
  else
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_RXDISABLE, 0);

      /* Disable interrupts and shorts */

      NRF54L_RADIO_INTCLR(lower, IEEE802154_RX_INT);
      NRF54L_RADIO_SHRTSET(lower, 0);

      /* Disable RX */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_DISABLE_OFFSET, 1);
    }

  /* Store state */

  dev->radio->state.rxenabled = enable;

  /* Data returned by callback in ISR */

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_energydetect
 *
 * Description:
 *   Start the energy detect measurement.
 *
 ****************************************************************************/

static int nrf54l_radioi8_energydetect(struct nrf54l_radioi8_dev_s *dev,
                                      uint32_t nsymbols)
{
  struct nrf54l_radio_dev_s *lower = NULL;
  uint32_t                  edcnt = 0;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  /* ED lasts (EDCNT + 1) iterations */

  edcnt = nsymbols / NRF54L_ED_ITER_SYMBOLS;
  if (edcnt > 0)
    {
      edcnt -= 1;
    }

  if (edcnt > RADIO_EDCNT_MASK)
    {
      edcnt = RADIO_EDCNT_MASK;
    }

  /* ED starts on the READY event, RXEN has no effect if RX is active */

  if (dev->radio->state.rxenabled)
    {
      dev->radio->state.edrestore = true;
      dev->radio->ops->rxenable(dev, false);
    }

  while (NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_STATE_OFFSET) !=
         RADIO_STATE_DISABLED);

  /* Clear events */

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_EDEND_OFFSET, 0);
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_READY_OFFSET, 0);
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_RXREADY_OFFSET, 0);

  /* Configure shortucts and interrupts */

  NRF54L_RADIO_SHRTSET(lower, IEEE802154_ED_SHORTS);
  NRF54L_RADIO_INTEN(lower, IEEE802154_ED_INT);

  /* Configure ED symbols */

  NRF54L_RADIO_EDCNTSET(lower, edcnt);

  /* Set state to ENERGY_DETECT (ED) */

  dev->radio->state.state = NRF54L_RADIO_STATE_ED;

  /* Start RX */

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_RXEN_OFFSET, 1);

  /* Data returned by callback in ISR */

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *
 ****************************************************************************/

static int nrf54l_radioi8_setchannel(struct nrf54l_radioi8_dev_s *dev,
                                   uint8_t chan)
{
  struct nrf54l_radio_dev_s *lower = NULL;
  uint32_t                  freq  = 2405 + 5 * (chan - 11);
  int                       ret;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  ret = NRF54L_RADIO_FREQSET(lower, freq);
  if (ret < 0)
    {
      return ret;
    }

  /* Restart an active receiver on the new channel */

  if (dev->radio->state.state == NRF54L_RADIO_STATE_RX &&
      dev->radio->state.rxenabled)
    {
      dev->radio->ops->rxenable(dev, false);

      while (NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_STATE_OFFSET) !=
             RADIO_STATE_DISABLED);

      dev->radio->ops->rxenable(dev, true);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setcca
 *
 * Description:
 *   Configure the Clear Channel Assessement.
 *
 ****************************************************************************/

static int nrf54l_radioi8_setcca(struct nrf54l_radioi8_dev_s *dev,
                                      struct ieee802154_cca_s *cca)
{
  struct nrf54l_radio_dev_s *lower = NULL;
  struct nrf54l_radio_cca_s  c;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  /* Fill radio structure */

  memset(&c, 0, sizeof(struct nrf54l_radio_cca_s));

  if (cca->use_ed && !cca->use_cs)
    {
      c.mode = NRF54L_RADIO_CCA_ED;
    }
  else if (!cca->use_ed && cca->use_cs)
    {
      c.mode = NRF54L_RADIO_CCA_CARRIER;
    }
  else if (cca->use_ed && cca->use_cs)
    {
      c.mode = NRF54L_RADIO_CCA_CARRIER_AND_ED;
    }

  c.edthres   = cca->edth;
  c.corrthres = cca->csth;
  c.corrcnt   = 5;

  /* Configure radio */

  NRF54L_RADIO_CCACFG(lower, &c);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_norm_setup_buf
 *
 * Description:
 *   Setup a normal transaction (non GTS) from buffer.
 *
 ****************************************************************************/

static void nrf54l_radioi8_norm_setup_buf(struct nrf54l_radioi8_dev_s *dev,
                                         uint8_t *buf, bool csma)
{
  struct nrf54l_radio_dev_s *lower  = NULL;
  uint32_t                  regval = 0;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  nrf54l_radioi8_trace_put(RADIO_TRACE_CSMASETUP, csma);

  /* If RX is enabled - we have to temporarily disable it */

  if (dev->radio->state.rxenabled)
    {
      dev->macops.rxenable((struct ieee802154_radio_s *)dev, false);

      /* Restore RX after TX completed */

      dev->radio->state.rxrestore = true;
    }

  /* Set packet pointer - buffer must be aligned */

  regval = (uint32_t)buf;
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_PACKETPTR_OFFSET, regval);

  if (csma)
    {
      /* Clear events */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_END_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_READY_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_PHYEND_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_TXREADY_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_CCABUSY_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_CCAIDLE_OFFSET, 0);

      /* Configure shorts and interrupts */

      if (dev->radio->state.slotted)
        {
          ASSERT(0);
        }
      else
        {
          NRF54L_RADIO_SHRTSET(lower, IEEE802154_TXCCAUNSLT_SHORTS);
          NRF54L_RADIO_INTEN(lower, IEEE802154_TXCCAUNSLT_INT);
        }

      dev->radio->state.state = NRF54L_RADIO_STATE_TX_CSMA;
    }
  else
    {
      /* Clear events */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_END_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_READY_OFFSET, 0);
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_PHYEND_OFFSET, 0);

      /* Configure shorts and interrupts */

      NRF54L_RADIO_SHRTSET(lower, IEEE802154_TX_SHORTS);
      NRF54L_RADIO_INTEN(lower, IEEE802154_TX_INT);
      dev->radio->state.state = NRF54L_RADIO_STATE_TX_NOCSMA;
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_norm_setup
 *
 * Description:
 *   Setup a normal transaction (non GTS).
 *
 ****************************************************************************/

static void nrf54l_radioi8_norm_setup(struct nrf54l_radioi8_dev_s *dev,
                                     struct iob_s *frame, bool csma)
{
  /* The first byte in TX buffer is PHR */

  dev->radio->txbuf[0] = frame->io_len;

  /* Allocate space for CRC */

  dev->radio->txbuf[0] += 2;

  /* Copy frame to RX buffer - we must send from aligned buffer */

  memcpy(&dev->radio->txbuf[1], &frame->io_data[0], frame->io_len);

  /* Setup buffer */

  nrf54l_radioi8_norm_setup_buf(dev, dev->radio->txbuf, csma);
}

/****************************************************************************
 * Name: nrf54l_radioi8_trg_csma
 *
 * Description:
 *   Trigger transaction with CSMA-CA.
 *
 ****************************************************************************/

static void nrf54l_radioi8_trg_csma(struct nrf54l_radioi8_dev_s *dev)
{
  uint32_t delay = 0;

  /* Set state to TX */

  dev->radio->state.state = NRF54L_RADIO_STATE_TX;

  /* Need slotted CSMA-CA */

  if (dev->radio->state.slotted)
    {
      ASSERT(0);
    }

  /* Need un-slotted CSMA-CA */

  else
    {
      if (!dev->radio->state.csma_busy)
        {
          /* Initial backoff state */

          dev->radio->state.NB = 0;
          dev->radio->state.BE = dev->radio->state.min_be;
        }

      /* Get random unit backoff period delay */

      delay = rand() % (1 << dev->radio->state.BE);
    }

  /* CSMA is in use */

  dev->radio->state.csma_busy = true;
  nrf54l_radioi8_trace_put(RADIO_TRACE_CSMATRIGGER, delay);

  dev->tim->ops->setup(dev, NRF54L_TIMER_CHAN_CSMADELAY,
                       delay * IEEE802154_UNIT_BACKOFF_PERIOD);
}

/****************************************************************************
 * Name: nrf54l_radioi8_trg_nocsma
 *
 * Description:
 *   Trigger transaction without CSMA-CA;
 *
 ****************************************************************************/

static void nrf54l_radioi8_trg_nocsma(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radio_dev_s *lower = NULL;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  /* Set state to TX */

  dev->radio->state.state = NRF54L_RADIO_STATE_TX;

  /* Start TX - we transmit immediately */

  nrf54l_radioi8_trace_put(RADIO_TRACE_NOCSMATRIGGER,
                          dev->radio->state.state);
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_TXEN_OFFSET, 1);
}

/****************************************************************************
 * Name: nrf54l_radioi8_norm_trigger
 *
 * Description:
 *   Trigger normal transaction (non GTS).
 *
 ****************************************************************************/

static void nrf54l_radioi8_norm_trigger(struct nrf54l_radioi8_dev_s *dev)
{
  /* Wait for ACK */

  if (dev->radio->txbuf[1] & IEEE802154_FRAMECTRL_ACKREQ)
    {
      dev->radio->state.waitack = true;
    }

  /* CSMA transmission */

  if (dev->radio->state.state == NRF54L_RADIO_STATE_TX_CSMA)
    {
      nrf54l_radioi8_trg_csma(dev);
    }

  /* No-CSMA transmission */

  else if (dev->radio->state.state == NRF54L_RADIO_STATE_TX_NOCSMA)
    {
      nrf54l_radioi8_trg_nocsma(dev);
    }

  /* Invalid state */

  else
    {
      /* We should not be there */

      DEBUGASSERT(0);
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_reset
 *
 * Description:
 *   Reset radio state to work with IEEE 802.15.4.
 *
 ****************************************************************************/

static int nrf54l_radioi8_reset(struct nrf54l_radioi8_radio_s *dev)
{
  struct nrf54l_radio_dev_s    *radio = NULL;
  struct nrf54l_radio_crc_s     crc;
  struct nrf54l_radio_pktcfg_s  pcfg;
  int                          ret   = OK;

  DEBUGASSERT(dev);
  radio = dev->lower;

  /* Reset radio state */

  memset(&dev->state, 0, sizeof(struct nrf54l_radioi8_radio_data_s));

  /* Reset radio */

  NRF54L_RADIO_RESET(radio);

  /* MAC parameters */

  dev->state.max_frame_waittime = IEEE802154_MAX_FRAME_WAITTIME;
  dev->state.max_csma_backoffs  = IEEE802154_MAX_CSMA_BACKOFFS;
  dev->state.min_be             = IEEE802154_MIN_BE;
  dev->state.max_be             = IEEE802154_MAX_BE;

  /* Set radio IEEE 802.15.4 mode */

  ret = NRF54L_RADIO_MODESET(radio, NRF54L_RADIO_MODE_IEEE802154);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure CRC */

  crc.len  = 2;
  crc.skip = NRF54L_RADIO_CRC_SKIPADDR_IEEE802154;
  crc.poly = 0x011021;
  crc.init = 0;

  ret = NRF54L_RADIO_CRCCFG(radio, &crc);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure packet for IEEE 802.15.4 mode */

  pcfg.max_len  = IEEE802154_MAX_PHY_PACKET_SIZE;
  pcfg.stat_len = 0;
  pcfg.bal_len  = 0;
  pcfg.lf_len   = 8;
  pcfg.s0_len   = 0;
  pcfg.s1_len   = 0;
  pcfg.ci_len   = 0;
  pcfg.pl_len   = NRF54L_RADIO_PREAMBLE_32BITZERO;
  pcfg.term_len = 0;
  pcfg.crcinc   = true;
  pcfg.endian   = false;
  pcfg.whiteen  = false;

  ret = NRF54L_RADIO_PKTCFG(radio, &pcfg);
  if (ret < 0)
    {
      goto errout;
    }

  /* Disable hardware TIFS */

  NRF54L_RADIO_PUTREG(radio, NRF54L_RADIO_TIMING_OFFSET, RADIO_TIMING_RU);

  /* Set TX power */

  NRF54L_RADIO_TXPWRSET(radio, RADIO_TXPOWER_0DBM);

  /* TODO: Configure LNA/PA */

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rxrestore
 *
 * Description:
 *   Enable RX if the radio is idle.
 *
 ****************************************************************************/

static void nrf54l_radioi8_rxrestore(struct nrf54l_radioi8_dev_s *dev)
{
  /* Enable RX only if no transaction is armed or in progress */

  if (dev->radio->state.state == NRF54L_RADIO_STATE_DISABLED)
    {
      dev->macops.rxenable((struct ieee802154_radio_s *)dev, true);
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_txnext
 *
 * Description:
 *   Start the next frame if the radio is free: the delayed TX that waits,
 *   else the CSMA TX that waits, else restore RX.  mindelay is the IFS a
 *   delayed TX keeps after the ACK TX that deferred it.
 *
 ****************************************************************************/

static void nrf54l_radioi8_txnext(struct nrf54l_radioi8_dev_s *dev,
                                 uint32_t mindelay)
{
  irqstate_t flags;
  uint32_t   delay;

  flags = enter_critical_section();

  /* A frame on the air, an armed frame, an ACK TX, an energy detection
   * or a CSMA transaction between its backoffs owns the radio.
   */

  if (dev->radio->state.csma_busy ||
      (dev->state.txdelayed_busy && !dev->state.txdelayed_pending) ||
      (dev->radio->state.state != NRF54L_RADIO_STATE_DISABLED &&
       dev->radio->state.state != NRF54L_RADIO_STATE_RX))
    {
      leave_critical_section(flags);
      return;
    }

  /* The delayed TX first, it can be the response to a received frame */

  if (dev->state.txdelayed_pending)
    {
      dev->state.txdelayed_pending = false;
      nrf54l_radioi8_norm_setup(dev, dev->state.txdelayed_desc->frame,
                               false);

      delay = MAX(dev->state.txdelayed_delay, mindelay);
      if (delay == 0)
        {
          nrf54l_radioi8_norm_trigger(dev);
        }
      else
        {
          dev->tim->ops->setup(dev, NRF54L_TIMER_CHAN_TXDELAY, delay);
        }
    }

  /* Then the CSMA TX, its backoff keeps the IFS */

  else if (dev->state.csma_pending)
    {
      dev->state.csma_pending = false;
      nrf54l_radioi8_norm_setup(dev, dev->state.csma_desc->frame, true);
      nrf54l_radioi8_norm_trigger(dev);
    }

  /* Nothing to send, restore RX */

  else if (dev->radio->state.rxrestore)
    {
      dev->macops.rxenable((struct ieee802154_radio_s *)dev, true);
      dev->radio->state.rxrestore = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_radioi8_dopoll_csma
 *
 * Description:
 *   Out-of-sequence TX poll.
 *
 ****************************************************************************/

void nrf54l_radioi8_dopoll_csma(void *arg)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)arg;
  int                         len = 0;

  /* Get exclusive access to the driver */

  while (nxmutex_lock(&dev->lock) < 0)
    {
    }

  /* If this a CSMA transaction and we have room in the CSMA */

  if (!dev->radio->state.csma_busy && !dev->state.csma_pending &&
      !dev->state.txdelayed_busy)
    {
      wlinfo("Polling for frame\n");
      len = dev->radiocb->poll(dev->radiocb, false, &dev->state.csma_desc);

      if (len > 0)
        {
          wlinfo("Frame received. Frame length: %d\n", len);

          dev->state.csma_pending = true;
          nrf54l_radioi8_txnext(dev, 0);
        }
    }

  nrf54l_radioi8_rxrestore(dev);
  nxmutex_unlock(&dev->lock);
}

/****************************************************************************
 * Name: nrf54l_radioi8_csmapoll
 *
 * Description:
 *   Handle CSMA poll.
 *
 ****************************************************************************/

static int nrf54l_radioi8_csmapoll(struct nrf54l_radioi8_dev_s *dev)
{
  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&dev->radio->csma_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(HPWORK, &dev->radio->csma_pollwork,
                 nrf54l_radioi8_dopoll_csma, dev, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_gtspoll
 *
 * Description:
 *   Handle GTS poll.
 *
 ****************************************************************************/

static int nrf54l_radioi8_gtspoll(struct nrf54l_radioi8_dev_s *dev)
{
  /* GTS not supported */

  return -ENOTSUP;
}

/****************************************************************************
 * Name: nrf54l_radioi8_work_noack
 *
 * Description:
 *   Handle no ACK work.
 *
 ****************************************************************************/

static void nrf54l_radioi8_work_noack(void *arg)
{
  struct nrf54l_radioi8_dev_s *dev   = (struct nrf54l_radioi8_dev_s *)arg;
  struct ieee802154_txdesc_s *tx    = NULL;
  bool                       *busy  = NULL;
  bool                        csma  = false;

  DEBUGASSERT(dev);

  nrf54l_radioi8_trace_put(RADIO_TRACE_WORK_NOACK, 0);

  /* Get exclusive access to the driver */

  while (nxmutex_lock(&dev->lock) < 0)
    {
    }

  /* The frame on the air, a delayed TX may still wait for the radio */

  if (dev->state.txdelayed_busy && !dev->state.txdelayed_pending)
    {
      tx   = dev->state.txdelayed_desc;
      busy = &dev->state.txdelayed_busy;
      csma = false;
    }
  else
    {
      tx   = dev->state.csma_desc;
      busy = &dev->radio->state.csma_busy;
      csma = true;
    }

  if (tx->retrycount > 0)
    {
      /* Try again as a new transaction once the radio is free, CSMA-CA
       * starts again for each attempt
       */

      tx->retrycount -= 1;
      nrf54l_radioi8_trace_put(RADIO_TRACE_TXRETRY, tx->retrycount);

      if (csma)
        {
          dev->radio->state.csma_busy = false;
          dev->state.csma_pending     = true;
        }
      else
        {
          dev->state.txdelayed_pending = true;
          dev->state.txdelayed_delay   = 0;
        }

      nrf54l_radioi8_txnext(dev, 0);
      nxmutex_unlock(&dev->lock);
      return;
    }

  nrf54l_radioi8_trace_put(RADIO_TRACE_NOACK, 0);

  tx->conf->status = IEEE802154_STATUS_NO_ACK;
  tx->framepending = false;
  dev->radiocb->txdone(dev->radiocb, tx);
  *busy = false;

  if (csma)
    {
      dev->state.csma_desc = NULL;
    }
  else
    {
      dev->state.txdelayed_desc = NULL;
    }

  nrf54l_radioi8_txnext(dev, 0);

  /* Unlock the radio device and poll for the next CSMA transaction */

  nxmutex_unlock(&dev->lock);
  nrf54l_radioi8_dopoll_csma(dev);
}

/****************************************************************************
 * Name: nrf54l_radioi8_work_irq
 *
 * Description:
 *   Handle RX work.
 *
 ****************************************************************************/

static void nrf54l_radioi8_work_rx(void *arg)
{
  struct nrf54l_radioi8_dev_s    *dev   = (struct nrf54l_radioi8_dev_s *)arg;
  struct nrf54l_radio_dev_s      *lower = NULL;
  struct ieee802154_primitive_s *prim  = NULL;
  struct ieee802154_data_ind_s  *ind   = NULL;
  uint8_t                        index = 0;
  bool                           crcok = false;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  nrf54l_radioi8_trace_put(RADIO_TRACE_WORK_RX, 0);

  /* Get exclusive access to the driver */

  while (nxmutex_lock(&dev->lock) < 0)
    {
    }

  /* Check CRC */

  crcok = NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_CRCSTATUS_OFFSET);
  if (dev->state.rxmode != NRF54L_RXMODE_NOCRC && crcok == false)
    {
      goto out;
    }

  /* Allocate a data_ind to put the frame in */

  prim = ieee802154_primitive_allocate();
  ind = (struct ieee802154_data_ind_s *)prim;
  if (ind == NULL)
    {
      wlerr("ERROR: Unable to allocate data_ind. Discarding frame\n");
      goto out;
    }

  prim->type = IEEE802154_PRIMITIVE_IND_DATA;

  /* Allocate an IOB to put the frame into, never block the work queue */

  ind->frame = iob_tryalloc(false);
  if (ind->frame == NULL)
    {
      wlerr("ERROR: Unable to allocate IOB. Discarding frame\n");
      ieee802154_primitive_free(prim);
      goto out;
    }

  /* First byte is the PHR */

  ind->frame->io_len = dev->radio->rxbuf[0];

  /* Reduce len by 2, we only receive frames with correct crc, no check
   * required.
   */

  ind->frame->io_len -= 2;

  for (index = 0; index < ind->frame->io_len; index++)
    {
      ind->frame->io_data[index] = dev->radio->rxbuf[index + 1];
    }

  /* Set CRC fields to 0 */

  ind->frame->io_data[index + 1] = 0;
  ind->frame->io_data[index + 2] = 0;

  /* LQI is the last byte in RAM, scaled to 0-255 like the ED result */

  ind->lqi = MIN(dev->radio->rxbuf[index + 1] * NRF54L_ED_RSSISCALE, 255);

  /* RSSI sampled after the SFD, the signal strength is -rssi dBm */

  ind->rssi = NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_RSSISAMPLE_OFFSET) &
              RADIO_RSSISAMPLE_MASK;

  /* Callback the receiver in the next highest layer */

  dev->radiocb->rxframe(dev->radiocb, ind);

out:

  /* Re-Enable RX if the radio is idle */

  nrf54l_radioi8_rxrestore(dev);

  /* Unlock the radio device */

  nxmutex_unlock(&dev->lock);
}

/****************************************************************************
 * Name: nrf54l_radioi8_work_tx
 *
 * Description:
 *   Handle TX work.
 *
 ****************************************************************************/

static void nrf54l_radioi8_work_tx(void *arg)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)arg;
  enum ieee802154_status_e    status;
  bool                        fpending = false;

  nrf54l_radioi8_trace_put(RADIO_TRACE_WORK_TX, 0);

  /* Get exclusive access to the driver */

  while (nxmutex_lock(&dev->lock) < 0)
    {
    }

  /* Get frame pending flag from the last TX ACK */

  fpending = dev->radio->state.framepending;
  dev->radio->state.framepending = false;

  status = IEEE802154_STATUS_SUCCESS;

  /* The frame on the air, a delayed TX may still wait for the radio */

  if (dev->state.txdelayed_busy && !dev->state.txdelayed_pending)
    {
      /* Inform the next layer of the transmission success/failure */

      dev->state.txdelayed_desc->conf->status = status;
      dev->state.txdelayed_desc->framepending = fpending;
      dev->radiocb->txdone(dev->radiocb, dev->state.txdelayed_desc);
      dev->state.txdelayed_desc = NULL;
      dev->state.txdelayed_busy = false;
    }
  else
    {
      /* Inform the next layer of the transmission success/failure */

      dev->state.csma_desc->conf->status = status;
      dev->state.csma_desc->framepending = fpending;
      dev->radiocb->txdone(dev->radiocb, dev->state.csma_desc);
      dev->state.csma_desc = NULL;

      /* We are now done with the transaction */

      dev->radio->state.csma_busy = false;
    }

  nrf54l_radioi8_txnext(dev, 0);

  /* Unlock the radio device and poll for the next CSMA transaction, RX is
   * restored there if the radio is idle.
   */

  nxmutex_unlock(&dev->lock);
  nrf54l_radioi8_dopoll_csma(dev);
}

/****************************************************************************
 * Name: nrf54l_radioi8_work_busy
 *
 * Description:
 *   Handle CCABUSY work.
 *
 ****************************************************************************/

static void nrf54l_radioi8_work_busy(void *arg)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)arg;

  nrf54l_radioi8_trace_put(RADIO_TRACE_WORK_BUSY, 0);

  /* Get exclusive access to the driver */

  while (nxmutex_lock(&dev->lock) < 0)
    {
    }

  /* CSMA-CA failed for the CSMA transaction */

  dev->state.csma_desc->conf->status =
    IEEE802154_STATUS_CHANNEL_ACCESS_FAILURE;
  dev->state.csma_desc->framepending = false;
  dev->radiocb->txdone(dev->radiocb, dev->state.csma_desc);
  dev->state.csma_desc = NULL;
  dev->radio->state.csma_busy = false;
  nrf54l_radioi8_txnext(dev, 0);

  /* Unlock the radio device and poll for the next CSMA transaction */

  nxmutex_unlock(&dev->lock);
  nrf54l_radioi8_dopoll_csma(dev);
}

/****************************************************************************
 * Name: nrf54l_radioi8_work_ed
 *
 * Description:
 *   Handle ED work.
 *
 ****************************************************************************/

static void nrf54l_radioi8_work_ed(void *arg)
{
  struct nrf54l_radioi8_dev_s *dev   = (struct nrf54l_radioi8_dev_s *)arg;
  struct nrf54l_radio_dev_s   *lower = NULL;
  int                         ed    = 0;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  nrf54l_radioi8_trace_put(RADIO_TRACE_WORK_ED, 0);

  /* Get result */

  ed = NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_EDSAMPLE_OFFSET);

  /* Convert to IEEE 802.15.4 scale */

  ed = ((ed > 63) ? 255: (ed * NRF54L_ED_RSSISCALE));

  /* Callback the receiver in the next highest layer */

  dev->radiocb->edresult(dev->radiocb, ed);

  /* Restore RX if it was enabled and no next ED was started */

  if (dev->radio->state.edrestore &&
      dev->radio->state.state != NRF54L_RADIO_STATE_ED)
    {
      dev->radio->state.edrestore = false;
      dev->radio->ops->rxenable(dev, true);
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_state_rx
 *
 * Description:
 *   Handle radio interrupt for RX state.
 *
 ****************************************************************************/

static void nrf54l_radioi8_state_rx(struct nrf54l_radioi8_dev_s *dev)
{
  uint8_t *rxbuf = dev->radio->rxbuf;

  /* Frame filter, a frame with a CRC error is not acknowledged */

  if (dev->radio->state.waitack == false)
    {
      if ((dev->state.rxmode != NRF54L_RXMODE_NOCRC &&
           !NRF54L_RADIO_GETREG(dev->radio->lower,
                                NRF54L_RADIO_CRCSTATUS_OFFSET)) ||
          !nrf54l_radioi8_filter(dev))
        {
          nrf54l_radioi8_trace_put(RADIO_TRACE_DROPFRAME, 0);

          /* Enable RX and ignore this frame */

          dev->macops.rxenable((struct ieee802154_radio_s *)dev,
                              true);
          return;
        }
    }

  /* Start ACK handling now - this must be done in isr.  Frames are not
   * filtered while we wait for our ACK, so they are not acknowledged.
   */

  if (dev->radio->state.waitack == false &&
      dev->state.rxmode == NRF54L_RXMODE_NORMAL &&
      rxbuf[1] & IEEE802154_FRAMECTRL_ACKREQ &&
      rxbuf[0] != IEEE802154_ACK_FRAME_SIZE)
    {
      nrf54l_radioi8_ack_transmit(dev);
    }

  /* Store RX frame long flag */

  dev->radio->state.rxlong = rxbuf[0] > 18 ? true : false;

  /* We don't wait for TX ACK - forward this frame to MAC */

  if (dev->radio->state.waitack == false)
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_RXDONE, 0);

      /* Receive DONE */

      if (work_available(&dev->radio->irqwork))
        {
          work_queue(HPWORK, &dev->radio->irqwork,
                     nrf54l_radioi8_work_rx, (void *)dev, 0);
        }
    }

  /* We wait for TX ACK and this is not a valid ACK frame - ignore it and
   * keep waiting for the ACK (IEEE 802.15.4-2011 5.1.6.2, 5.1.6.4.3).
   */

  else if (rxbuf[0] != IEEE802154_ACK_FRAME_SIZE ||
           (rxbuf[1] & IEEE802154_FRAMECTRL_FTYPE) != IEEE802154_FRAME_ACK ||
           !NRF54L_RADIO_GETREG(dev->radio->lower,
                               NRF54L_RADIO_CRCSTATUS_OFFSET))
    {
      dev->macops.rxenable((struct ieee802154_radio_s *)dev, true);
    }

  /* We wait for TX ACK and this is our ACK - confirm TX to MAC */

  else if (rxbuf[3] == dev->radio->txbuf[3])
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_RXACKDONE, 0);

      /* ACK frame received */

      dev->radio->state.waitack = false;

      /* Get frame pending flag */

      dev->radio->state.framepending = rxbuf[1] & 0x10;

      /* Clear timer */

      dev->tim->ops->stop(dev);

      /* This is ACK - notify MAC that TX complete */

      if (work_available(&dev->radio->irqwork))
        {
          work_queue(HPWORK, &dev->radio->irqwork,
                     nrf54l_radioi8_work_tx, (void *)dev, 0);
        }
    }

  /* We wait for TX ACK and the ACK has another sequence number - notify TX
   * failure
   */

  else
    {
      /* Clear flag and stop the ACK wait timer */

      dev->radio->state.waitack = false;
      dev->tim->ops->stop(dev);

      /* And notify MAC */

      DEBUGASSERT(work_available(&dev->radio->noackwork));
      work_queue(HPWORK, &dev->radio->noackwork,
                 nrf54l_radioi8_work_noack, (void *)dev, 0);
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_state_tx
 *
 * Description:
 *   Handle radio interrupt for TX state.
 *
 ****************************************************************************/

static void nrf54l_radioi8_state_tx(struct nrf54l_radioi8_dev_s *dev)
{
  struct nrf54l_radio_dev_s *lower   = NULL;

  DEBUGASSERT(dev);
  lower = dev->radio->lower;

  /* CCABUSY event for TX */

  if (NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_EVENTS_CCABUSY_OFFSET))
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_TXCCABUSY, 0);

      /* Clear event */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_CCABUSY_OFFSET, 0);

      /* Slotted CSMA-CA */

      if (dev->radio->state.slotted == true)
        {
          ASSERT(0);
        }

      /* Un-slotted CSMA-CA */

      else
        {
          /* Update backoff */

          dev->radio->state.NB += 1;
          dev->radio->state.BE = MIN(dev->radio->state.BE + 1,
                                     dev->radio->state.max_be);

          if (dev->radio->state.NB > dev->radio->state.max_csma_backoffs)
            {
              /* Return failure to MAC, no frame was sent */

              dev->radio->state.waitack = false;

              if (work_available(&dev->radio->irqwork))
                {
                  work_queue(HPWORK, &dev->radio->irqwork,
                             nrf54l_radioi8_work_busy, (void *)dev, 0);
                }

              goto out;
            }
          else
            {
              /* Try again, shorts and interrupts were reset in ISR */

              nrf54l_radioi8_norm_setup_buf(dev, dev->radio->txbuf, true);
              nrf54l_radioi8_norm_trigger(dev);

              /* Do not restore RX */

              return;
            }
        }
    }

  /* CCAIDLE event for TX */

  if (NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_EVENTS_CCAIDLE_OFFSET))
    {
      /* Clear event */

      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_CCAIDLE_OFFSET, 0);

      /* Slotted CSMA-CA */

      if (dev->radio->state.slotted == true)
        {
          ASSERT(0);
        }

      /* Un-slotted CSMA-CA */

      else
        {
          /* Nothing here */
        }
    }

  /* TX done event */

  if (dev->radio->state.waitack == true)
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_WAITACK, 0);

      /* Start timer and wait for ACK */

      dev->tim->ops->setup(dev, NRF54L_TIMER_CHAN_WAITACK,
                           IEEE802154_ACK_WAIT);
    }
  else
    {
      nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_TXDONE, 0);

      /* Transmit DONE */

      if (work_available(&dev->radio->irqwork))
        {
          work_queue(HPWORK, &dev->radio->irqwork,
                     nrf54l_radioi8_work_tx, (void *)dev, 0);
        }
    }

out:

  /* Restore RX */

  if (dev->radio->state.rxrestore | dev->radio->state.waitack)
    {
      dev->macops.rxenable((struct ieee802154_radio_s *)dev, true);
      dev->radio->state.rxrestore = false;
    }
}

/****************************************************************************
 * Name: nrf54l_radioi8_state_acktx
 *
 * Description:
 *   Handle radio interrupt for ACKTX state.
 *
 ****************************************************************************/

static void nrf54l_radioi8_state_acktx(struct nrf54l_radioi8_dev_s *dev)
{
  nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_ACKTX, 0);

  /* Start the TX that waited for the ACK TX, after the IFS */

  nrf54l_radioi8_txnext(dev, dev->radio->state.rxlong ?
                            IEEE802154_LIFS_SYMBOLS :
                            IEEE802154_SIFS_SYMBOLS);
}

/****************************************************************************
 * Name: nrf54l_radioi8_state_ed
 *
 * Description:
 *   Handle radio interrupt for ED state.
 *
 ****************************************************************************/

static void nrf54l_radioi8_state_ed(struct nrf54l_radioi8_dev_s *dev)
{
  /* Energy detect DONE */

  DEBUGASSERT(work_available(&dev->radio->irqwork));
  work_queue(HPWORK, &dev->radio->irqwork,
             nrf54l_radioi8_work_ed, (void *)dev, 0);
}

/****************************************************************************
 * Name: nrf54l_radioi8_isr_radio
 *
 * Description:
 *   Radio IEEE 802.15.4 interrupt handler.
 *
 ****************************************************************************/

static int nrf54l_radioi8_isr_radio(int irq, void *context, void *arg)
{
  struct nrf54l_radioi8_dev_s *dev   = (struct nrf54l_radioi8_dev_s *)arg;
  struct nrf54l_radio_dev_s   *lower = NULL;
  irqstate_t                  flags;
  uint8_t                     state;

  DEBUGASSERT(dev != NULL);
  lower = dev->radio->lower;

  flags = enter_critical_section();

  nrf54l_radioi8_trace_put(RADIO_TRACE_IRQ_RADIO, 0);

  /* Get state */

  state = dev->radio->state.state;
  dev->radio->state.state = NRF54L_RADIO_STATE_DISABLED;

  /* Always clear END and PHYEND events */

  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_END_OFFSET, 0);
  NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_PHYEND_OFFSET, 0);

  /* Clear interrupts and shorts */

  NRF54L_RADIO_INTCLR(lower, 0xffffffff);
  NRF54L_RADIO_SHRTSET(lower, 0);

  /* If by chance radio is not disabled - disable it now. This should be
   * handled by SHORTS, but this not always work...
   */

  if (NRF54L_RADIO_GETREG(lower, NRF54L_RADIO_STATE_OFFSET)
      != RADIO_STATE_DISABLED)
    {
      NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_TASKS_DISABLE_OFFSET, 1);

      /* Don't wait for the DISABLED event, hopefully the radio will be
       * disabled by the time we use it again. Max delay is TX->DISABLED
       * and takes 21us.
       */
    }

  /* Handle IRQ depending on the current state */

  switch (state)
    {
      case NRF54L_RADIO_STATE_RX:
        {
          nrf54l_radioi8_state_rx(dev);

          break;
        }

      case NRF54L_RADIO_STATE_TX:
        {
          nrf54l_radioi8_state_tx(dev);

          break;
        }

      case NRF54L_RADIO_STATE_ACKTX:
        {
          nrf54l_radioi8_state_acktx(dev);

          break;
        }

      case NRF54L_RADIO_STATE_ED:
        {
          /* Clear event */

          NRF54L_RADIO_PUTREG(lower, NRF54L_RADIO_EVENTS_EDEND_OFFSET, 0);

          nrf54l_radioi8_state_ed(dev);

          break;
        }

      default:
        {
          ASSERT(0);
          break;
        }
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_radio_init
 *
 * Description:
 *   Initialize RADIO for IEEE802154 operations.
 *
 ****************************************************************************/

struct nrf54l_radioi8_radio_s *
nrf54l_radioi8_radio_init(struct nrf54l_radioi8_dev_s *dev,
                         struct nrf54l_radio_board_s *board)
{
  struct nrf54l_radio_dev_s *radio = NULL;

  /* Initialize lower-half radio */

  radio = nrf54l_radio_initialize(0, board);
  if (radio == NULL)
    {
      wlerr("nrf54l_radio_initialize failed %d\n", -errno);
      return NULL;
    }

  /* Attach custom RADIO interrupt */

  irq_attach(radio->irq, nrf54l_radioi8_isr_radio, dev);
  up_enable_irq(radio->irq);

  /* Connect radioer */

  memset(&g_radioi8_radio, 0, sizeof(struct nrf54l_radioi8_radio_s));
  g_radioi8_radio.ops   = &g_radioi8_radio_ops;
  g_radioi8_radio.lower = radio;

  /* Connect buffers */

  g_radioi8_radio.rxbuf     = g_nrf54l_radioi8_rxbuf;
  g_radioi8_radio.txbuf     = g_nrf54l_radioi8_txbuf;
  g_radioi8_radio.ackbuf    = g_nrf54l_radioi8_ackbuf;

  return &g_radioi8_radio;
}
