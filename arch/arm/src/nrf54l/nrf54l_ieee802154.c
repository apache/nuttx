/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ieee802154.c
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>

#include <nuttx/mm/iob.h>

#include "arm_internal.h"
#include "hardware/nrf54l_ficr.h"

#include "nrf54l_ieee802154_trace.h"

#include "nrf54l_ieee802154_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF54L_RADIO_CUSTOM
#  error RADIO custom interrupts must be enabled
#endif

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* IEEE 802.15.4 radio data */

static struct nrf54l_radioi8_dev_s g_nrf54l_radioi8;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Set/get radio attributes */

static int nrf54l_radioi8_setrxmode(struct nrf54l_radioi8_dev_s *dev,
                                   int mode);
static int nrf54l_radioi8_setpanid(struct nrf54l_radioi8_dev_s *dev,
                                  const uint8_t *panid);
static int nrf54l_radioi8_setsaddr(struct nrf54l_radioi8_dev_s *dev,
                                  const uint8_t *saddr);
static int nrf54l_radioi8_seteaddr(struct nrf54l_radioi8_dev_s *dev,
                                  const uint8_t *eaddr);
static int nrf54l_radioi8_setcoordsaddr(struct nrf54l_radioi8_dev_s *dev,
                                        const uint8_t *saddr);
static int nrf54l_radioi8_setdevmode(struct nrf54l_radioi8_dev_s *dev,
                                    uint8_t mode);

/* Radio ops */

static int nrf54l_radioi8_bind(struct ieee802154_radio_s *radio,
                              struct ieee802154_radiocb_s *radiocb);
static int nrf54l_radioi8_reset(struct ieee802154_radio_s *radio);
static int nrf54l_radioi8_getattr(struct ieee802154_radio_s *radio,
                                 enum ieee802154_attr_e attr,
                                 union ieee802154_attr_u *attrval);
static int nrf54l_radioi8_setattr(struct ieee802154_radio_s *radio,
                                 enum ieee802154_attr_e attr,
                                 const union ieee802154_attr_u *attrval);
static int nrf54l_radioi8_txnotify(struct ieee802154_radio_s *radio,
                                  bool gts);
static int nrf54l_radioi8_txdelayed(struct ieee802154_radio_s *radio,
                                   struct ieee802154_txdesc_s *txdesc,
                                   uint32_t symboldelay);
static int nrf54l_radioi8_rxenable(struct ieee802154_radio_s *radio,
                                  bool enable);
static int nrf54l_radioi8_setchannel(struct nrf54l_radioi8_dev_s *dev,
                                    uint8_t chan);
static int nrf54l_radioi8_setcca(struct nrf54l_radioi8_dev_s *dev,
                                struct ieee802154_cca_s *cca);
static int nrf54l_radioi8_energydetect(struct ieee802154_radio_s *radio,
                                      uint32_t nsymbols);

static int
nrf54l_radioi8_beaconstart(struct ieee802154_radio_s *radio,
                          const struct ieee802154_superframespec_s *sfspec,
                          struct ieee802154_beaconframe_s *beacon);
static int
nrf54l_radioi8_beaconupdate(struct ieee802154_radio_s *radio,
                           struct ieee802154_beaconframe_s *beacon);
static int nrf54l_radioi8_beaconstop(struct ieee802154_radio_s *radio);
static int
nrf54l_radioi8_sfupdate(struct ieee802154_radio_s *radio,
                       const struct ieee802154_superframespec_s *sfspec);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_setrxmode
 *
 * Description:
 *   Set the RX mode (normal, promiscuous, no CRC).
 *
 ****************************************************************************/

static int nrf54l_radioi8_setrxmode(struct nrf54l_radioi8_dev_s *dev,
                                   int mode)
{
  wlinfo("setrxmode %d\n", mode);

  if (mode < NRF54L_RXMODE_NORMAL || mode > NRF54L_RXMODE_NOCRC)
    {
      return -EINVAL;
    }

  dev->state.rxmode = mode;

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setpanid
 *
 * Description:
 *   Define the PAN ID the device is operating on.
 *
 ****************************************************************************/

static int nrf54l_radioi8_setpanid(struct nrf54l_radioi8_dev_s *dev,
                                  const uint8_t *panid)
{
  wlinfo("setpanid: %02X:%02X\n", panid[0], panid[1]);
  IEEE802154_PANIDCOPY(dev->state.addr.panid, panid);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setsaddr
 *
 * Description:
 *   Define the device short address. The following addresses are special:
 *
 *     FFFEh : Broadcast
 *     FFFFh : Unspecified
 *
 ****************************************************************************/

static int nrf54l_radioi8_setsaddr(struct nrf54l_radioi8_dev_s *dev,
                                  const uint8_t *saddr)
{
  wlinfo("setsaddr: %02X:%02X\n", saddr[0], saddr[1]);
  IEEE802154_SADDRCOPY(dev->state.addr.saddr, saddr);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_seteaddr
 *
 * Description:
 *   Define the device extended address. The following addresses are special:
 *
 *     ffffffffffffffffh : Unspecified
 *
 ****************************************************************************/

static int nrf54l_radioi8_seteaddr(struct nrf54l_radioi8_dev_s *dev,
                                  const uint8_t *eaddr)
{
  int i = 0;

  wlinfo("seteaddr: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
         eaddr[0], eaddr[1], eaddr[2], eaddr[3], eaddr[4], eaddr[5],
         eaddr[6], eaddr[7]);

  for (i = 0; i < 8; i++)
    {
      dev->state.addr.eaddr[i] = eaddr[i];
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setcoordsaddr
 *
 * Description:
 *   Define the coordinator short address. The following addresses are
 *   special:
 *
 *     FFFEh : Broadcast
 *     FFFFh : Unspecified
 *
 ****************************************************************************/

static int nrf54l_radioi8_setcoordsaddr(struct nrf54l_radioi8_dev_s *dev,
                                        const uint8_t *saddr)
{
  IEEE802154_SADDRCOPY(dev->state.coordaddr.saddr, saddr);

  wlinfo("setcoordsaddr: %02X:%02X\n", saddr[0], saddr[1]);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setcoordeaddr
 *
 * Description:
 *   Define the coordinator extended address. The following addresses are
 *   special:
 *
 *     FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

static int nrf54l_radioi8_setcoordeaddr(struct nrf54l_radioi8_dev_s *dev,
                                        const uint8_t *eaddr)
{
  int i = 0;

  wlinfo("setcoordeaddr: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
         eaddr[0], eaddr[1], eaddr[2], eaddr[3], eaddr[4], eaddr[5],
         eaddr[6], eaddr[7]);

  for (i = 0; i < 8; i++)
    {
      dev->state.coordaddr.eaddr[i] = eaddr[i];
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setdevmode
 *
 * Description:
 *   Define the device behaviour: endpoint, coord or PAN coord.
 *
 ****************************************************************************/

static int nrf54l_radioi8_setdevmode(struct nrf54l_radioi8_dev_s *dev,
                                    uint8_t mode)
{
  wlinfo("setdevmode %d\n", mode);

  if (mode < IEEE802154_DEVMODE_ENDPOINT ||
      mode > IEEE802154_DEVMODE_PANCOORD)
    {
      return -EINVAL;
    }

  dev->state.devmode = mode;
  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_bind
 *
 * Description:
 *   Bind radio callbacks.
 *
 ****************************************************************************/

static int nrf54l_radioi8_bind(struct ieee802154_radio_s *radio,
                              struct ieee802154_radiocb_s *radiocb)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;

  DEBUGASSERT(dev != NULL);
  dev->radiocb = radiocb;
  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_eui64
 *
 * Description:
 *   EUI-64 from the Nordic OUI and the FICR device identifier.
 *
 ****************************************************************************/

static void nrf54l_radioi8_eui64(uint8_t *eaddr)
{
  uint32_t id0 = getreg32(NRF54L_FICR_INFO_DEVICEID(0));
  uint32_t id1 = getreg32(NRF54L_FICR_INFO_DEVICEID(1));

  eaddr[0] = 0xf4;
  eaddr[1] = 0xce;
  eaddr[2] = 0x36;
  eaddr[3] = id1 & 0xff;
  eaddr[4] = (id1 >> 8) & 0xff;
  eaddr[5] = (id1 >> 16) & 0xff;
  eaddr[6] = (id1 >> 24) & 0xff;
  eaddr[7] = id0 & 0xff;
}

/****************************************************************************
 * Name: nrf54l_radioi8_reset
 *
 * Description:
 *   Reset radio.
 *
 ****************************************************************************/

static int nrf54l_radioi8_reset(struct ieee802154_radio_s *radio)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;
  struct ieee802154_cca_s     cca;
  int                         ret = OK;

#ifdef CONFIG_NRF54L_RADIO_IEEE802154_TRACE
  /* Reset trace */

  nrf54l_radioi8_trace_init();
#endif

  /* Reset radio state */

  memset(&dev->state, 0, sizeof(struct nrf54l_radioi8_state_s));

  /* Unique default extended address */

  nrf54l_radioi8_eui64(dev->state.addr.eaddr);

  /* Initialize radio in IEEE 802.15.4 mode */

  ret = dev->radio->ops->reset(dev->radio);
  if (ret < 0)
    {
      goto errout;
    }

  /* Reset TIMER */

  dev->tim->ops->reset(dev);

  /* Set channel */

  nrf54l_radioi8_setchannel(dev, 11);

  /* Configure default CCA:
   *   - CCA mode ED
   *   - no carrier sense
   *   - recommenced ED threshold -69 dBm
   */

  cca.use_ed = 1;
  cca.use_cs = 0;
  cca.edth   = 0x60;
  cca.csth   = 0x00;
  nrf54l_radioi8_setcca(dev, &cca);

  /* Configure initial RX mode */

  nrf54l_radioi8_setrxmode(dev, NRF54L_RXMODE_NORMAL);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf54l_radioi8_getattr
 *
 * Description:
 *   Get radio attribute.
 *
 ****************************************************************************/

static int nrf54l_radioi8_getattr(struct ieee802154_radio_s *radio,
                                 enum ieee802154_attr_e attr,
                                 union ieee802154_attr_u *attrval)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;
  int                         ret = OK;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_EADDR:
        {
          memcpy(&attrval->mac.eaddr[0], &dev->state.addr.eaddr[0], 8);
          ret = IEEE802154_STATUS_SUCCESS;
          break;
        }

      case IEEE802154_ATTR_MAC_MAX_FRAME_WAITTIME:
        {
          attrval->mac.max_frame_waittime =
            dev->radio->state.max_frame_waittime;
          ret                             = IEEE802154_STATUS_SUCCESS;
          break;
        }

      case IEEE802154_ATTR_PHY_SYMBOL_DURATION:
        {
          attrval->phy.symdur_picosec = (IEEE802154_SYMBOL_US * 1000000);
          ret                         = IEEE802154_STATUS_SUCCESS;
          break;
        }

      case IEEE802154_ATTR_PHY_CHAN:
        {
          attrval->phy.chan = dev->state.chan;
          ret               = IEEE802154_STATUS_SUCCESS;
          break;
        }

      case IEEE802154_ATTR_PHY_FCS_LEN:
        {
          attrval->phy.fcslen = 2;
          ret                 = IEEE802154_STATUS_SUCCESS;
          break;
        }

      case IEEE802154_ATTR_PHY_REGDUMP:
        {
          NRF54L_RADIO_DUMPREGS(dev->radio->lower);
          break;
        }

#ifdef CONFIG_NRF54L_RADIO_IEEE802154_TRACE
      case IEEE802154_ATTR_PHY_TRACEDUMP:
        {
          nrf54l_radioi8_trace_dump();
          break;
        }
#endif

      default:
        {
          ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setattr
 *
 * Description:
 *   Set radio attribute.
 *
 ****************************************************************************/

static int nrf54l_radioi8_setattr(struct ieee802154_radio_s *radio,
                                 enum ieee802154_attr_e attr,
                                 const union ieee802154_attr_u *attrval)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;
  int                         ret = IEEE802154_STATUS_SUCCESS;

  switch (attr)
    {
      case IEEE802154_ATTR_MAC_PANID:
        {
          nrf54l_radioi8_setpanid(dev, attrval->mac.panid);
          break;
        }

      case IEEE802154_ATTR_MAC_SADDR:
        {
          nrf54l_radioi8_setsaddr(dev, attrval->mac.saddr);
          break;
        }

      case IEEE802154_ATTR_MAC_EADDR:
        {
          nrf54l_radioi8_seteaddr(dev, attrval->mac.eaddr);
          break;
        }

      case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
          nrf54l_radioi8_setcoordsaddr(dev, attrval->mac.coordsaddr);
          break;
        }

      case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
          nrf54l_radioi8_setcoordeaddr(dev, attrval->mac.coordeaddr);
          break;
        }

      case IEEE802154_ATTR_MAC_PROMISCUOUS_MODE:
        {
          if (attrval->mac.promisc_mode)
            {
              nrf54l_radioi8_setrxmode(dev, NRF54L_RXMODE_PROMISC);
            }
          else
            {
              nrf54l_radioi8_setrxmode(dev, NRF54L_RXMODE_NORMAL);
            }
          break;
        }

      case IEEE802154_ATTR_PHY_CHAN:
        {
          nrf54l_radioi8_setchannel(dev, attrval->phy.chan);
          break;
        }

      case IEEE802154_ATTR_MAC_DEVMODE:
        {
          nrf54l_radioi8_setdevmode(dev, attrval->mac.devmode);
          break;
        }

      default:
        {
          ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_radioi8_txnotify
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 ****************************************************************************/

static int nrf54l_radioi8_txnotify(struct ieee802154_radio_s *radio,
                                  bool gts)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;
  int                         ret = OK;

  if (gts)
    {
      ret = dev->radio->ops->gts_poll(dev);
    }
  else
    {
      ret = dev->radio->ops->csma_poll(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf54l_radioi8_txdelayed
 *
 * Description:
 *   Transmit a packet without regard to supeframe structure after a certain
 *   number of symbols.  This function is used to send Data Request
 *   responses.  It can also be used to send data immediately if the delay
 *   is set to 0.
 *
 ****************************************************************************/

static int nrf54l_radioi8_txdelayed(struct ieee802154_radio_s *radio,
                                   struct ieee802154_txdesc_s *txdesc,
                                   uint32_t symboldelay)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;
  irqstate_t                  flags;
  int                         ret = OK;

  /* Get exclusive access to the radio device */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* There should never be more than one of these transactions at once. */

  DEBUGASSERT(!dev->state.txdelayed_busy && !dev->state.txdelayed_pending);

  dev->state.txdelayed_desc = txdesc;

  /* The radio sends one frame at a time, start this transaction when the
   * CSMA transaction is done.
   */

  if (dev->radio->state.csma_busy)
    {
      dev->state.txdelayed_pending = true;
      dev->state.txdelayed_delay   = symboldelay;
      goto out;
    }

  /* Not interrupted by the ACK TX interrupt that starts a busy transaction */

  flags = enter_critical_section();

  dev->state.txdelayed_busy = true;

  /* Wait for ACKTX done - we start transmission in
   * nrf54l_radioi8_state_acktx()
   */

  if (dev->radio->state.state == NRF54L_RADIO_STATE_ACKTX)
    {
      leave_critical_section(flags);
      goto out;
    }

  /* Setup TX */

  dev->radio->ops->norm_setup(dev, txdesc->frame, false);

  if (symboldelay == 0)
    {
      /* Send now */

      dev->radio->ops->norm_trigger(dev);
    }
  else
    {
      /* Run TIMER - TX is handled in timer isr */

      dev->tim->ops->setup(dev, NRF54L_TIMER_CHAN_TXDELAY, symboldelay);
    }

  leave_critical_section(flags);

out:
  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_rxenable
 *
 * Description:
 *  Enable/Disable receiver.
 *
 ****************************************************************************/

static int nrf54l_radioi8_rxenable(struct ieee802154_radio_s *radio,
                                  bool enable)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;

  wlinfo("rxenable %d\n", enable);
  return dev->radio->ops->rxenable(dev, enable);
}

/****************************************************************************
 * Name: nrf54l_radioi8_energydetect
 *
 * Description:
 *   Start the energy detect measurement.
 *
 ****************************************************************************/

static int nrf54l_radioi8_energydetect(struct ieee802154_radio_s *radio,
                                      uint32_t nsymbols)
{
  struct nrf54l_radioi8_dev_s *dev = (struct nrf54l_radioi8_dev_s *)radio;

  return dev->radio->ops->energydetect(dev, nsymbols);
}

/****************************************************************************
 * Name: nrf54l_radioi8_setchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *   In the 2.4 GHz, there are 16 channels, each 2 MHz wide, 5 MHz spacing:
 *
 *   Chan   MHz       Chan   MHz       Chan   MHz       Chan   MHz
 *     11  2405         15  2425         19  2445         23  2465
 *     12  2410         16  2430         20  2450         24  2470
 *     13  2415         17  2435         21  2455         25  2475
 *     14  2420         18  2440         22  2460         26  2480
 *
 ****************************************************************************/

static int nrf54l_radioi8_setchannel(struct nrf54l_radioi8_dev_s *dev,
                                    uint8_t chan)
{
  int ret = OK;

  wlinfo("setchannel: %u\n", (unsigned)chan);

  if (dev->state.chan == chan)
    {
      return OK;
    }

  if (chan < 11 || chan > 26)
    {
      wlerr("Invalid chan: %d\n", chan);
      return -EINVAL;
    }

  ret = dev->radio->ops->setchannel(dev, chan);
  if (ret < 0)
    {
      wlerr("dev->radio->ops->setchannel failed %d\n", ret);
      return -EINVAL;
    }

  dev->state.chan = chan;

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_setcca
 *
 * Description:
 *   Define the Clear Channel Assessement method.
 *
 ****************************************************************************/

static int nrf54l_radioi8_setcca(struct nrf54l_radioi8_dev_s *dev,
                                struct ieee802154_cca_s *cca)
{
  int ret = OK;

  if (!cca->use_ed && !cca->use_cs)
    {
      return -EINVAL;
    }

  if (cca->use_cs && cca->csth > 0x0f)
    {
      return -EINVAL;
    }

  /* Configure CCA */

  ret = dev->radio->ops->setcca(dev, cca);
  if (ret < 0)
    {
      wlerr("dev->radio->ops->setcca failed %d\n", ret);
      return -EINVAL;
    }

  memcpy(&dev->state.cca, cca, sizeof(struct ieee802154_cca_s));

  return OK;
}

/****************************************************************************
 * Name: nrf54l_radioi8_beaconstart
 *
 * Description:
 *   Start beacon.
 *
 ****************************************************************************/

static int
nrf54l_radioi8_beaconstart(struct ieee802154_radio_s *radio,
                          const struct ieee802154_superframespec_s *sfspec,
                          struct ieee802154_beaconframe_s *beacon)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: nrf54l_radioi8_beaconupdate
 *
 * Description:
 *   Update beacon.
 *
 ****************************************************************************/

static int
nrf54l_radioi8_beaconupdate(struct ieee802154_radio_s *radio,
                           struct ieee802154_beaconframe_s *beacon)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: nrf54l_radioi8_beaconstop
 *
 * Description:
 *   Stop beacon.
 *
 ****************************************************************************/

static int nrf54l_radioi8_beaconstop(struct ieee802154_radio_s *radio)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: nrf54l_radioi8_sfupdate
 *
 * Description:
 *   Update super frame.
 *
 ****************************************************************************/

static int
nrf54l_radioi8_sfupdate(struct ieee802154_radio_s *radio,
                       const struct ieee802154_superframespec_s *sfspec)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_radioi8_register
 *
 * Description:
 *   Register NRF54L radio in IEE802154 mode.
 *
 ****************************************************************************/

struct ieee802154_radio_s *
nrf54l_radioi8_register(struct nrf54l_radio_board_s *board)
{
  struct nrf54l_radioi8_dev_s *dev = &g_nrf54l_radioi8;

  /* Reset data */

  memset(&g_nrf54l_radioi8, 0, sizeof(struct nrf54l_radioi8_dev_s));

  /* Allow exclusive access to the privmac struct */

  nxmutex_init(&dev->lock);

  /* Initialize lower-half radio */

  dev->radio = nrf54l_radioi8_radio_init(dev, board);
  if (dev->radio == NULL)
    {
      wlerr("nrf54l_radioi8_radio_init failed %d\n", -errno);
      return NULL;
    }

  DEBUGASSERT(dev->radio->ops->txstart);
  DEBUGASSERT(dev->radio->ops->notify_noack);
  DEBUGASSERT(dev->radio->ops->ccastart);
  DEBUGASSERT(dev->radio->ops->rxenable);
  DEBUGASSERT(dev->radio->ops->energydetect);
  DEBUGASSERT(dev->radio->ops->setchannel);
  DEBUGASSERT(dev->radio->ops->setcca);
  DEBUGASSERT(dev->radio->ops->norm_setup);
  DEBUGASSERT(dev->radio->ops->norm_trigger);
  DEBUGASSERT(dev->radio->ops->reset);
  DEBUGASSERT(dev->radio->ops->csma_poll);
  DEBUGASSERT(dev->radio->ops->gts_poll);

  /* Initialize TIMER */

  dev->tim = nrf54l_radioi8_tim_init(dev);
  if (dev->tim == NULL)
    {
      wlerr("nrf54l_radioi8_tim_init failed %d\n", -errno);
      return NULL;
    }

  DEBUGASSERT(dev->tim->ops->setup);
  DEBUGASSERT(dev->tim->ops->stop);
  DEBUGASSERT(dev->tim->ops->reset);

  /* Connect MAC ops */

  dev->macops.bind         = nrf54l_radioi8_bind;
  dev->macops.reset        = nrf54l_radioi8_reset;
  dev->macops.getattr      = nrf54l_radioi8_getattr;
  dev->macops.setattr      = nrf54l_radioi8_setattr;
  dev->macops.txnotify     = nrf54l_radioi8_txnotify;
  dev->macops.txdelayed    = nrf54l_radioi8_txdelayed;
  dev->macops.rxenable     = nrf54l_radioi8_rxenable;
  dev->macops.energydetect = nrf54l_radioi8_energydetect;
  dev->macops.beaconstart  = nrf54l_radioi8_beaconstart;
  dev->macops.beaconupdate = nrf54l_radioi8_beaconupdate;
  dev->macops.beaconstop   = nrf54l_radioi8_beaconstop;
  dev->macops.sfupdate     = nrf54l_radioi8_sfupdate;

  return &dev->macops;
}
