/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ieee802154_priv.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_IEEE802154_PRIV_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_IEEE802154_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "nrf54l_ieee802154_radio.h"
#include "nrf54l_ieee802154_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RX modes */

enum nrf54l_ieee802154_rxmode_e
{
  NRF54L_RXMODE_NORMAL,
  NRF54L_RXMODE_PROMISC,
  NRF54L_RXMODE_NOCRC
};

/* Device modes */

enum nrf54l_ieee802154_devmode_e
{
  NRF54L_DEVMODE_ENDPOINT,
  NRF54L_DEVMODE_COORD,
  NRF54L_DEVMODE_PANCOORD
};

/* IEEE 802.15.4 device state */

struct nrf54l_radioi8_state_s
{
  /* MAC Attributes */

  struct ieee802154_addr_s addr;      /* Address */
  struct ieee802154_addr_s coordaddr; /* Coordinator address */
  struct ieee802154_cca_s  cca;       /* Clear channel assessement method */

  /* TX CSMA */

  struct ieee802154_txdesc_s *txdelayed_desc;
  struct ieee802154_txdesc_s *csma_desc;

  /* Radio state */

  uint8_t  chan;                /* 11 to 26 for the 2.4 GHz band */
  uint8_t  devmode;             /* device mode: endpoint, coord, PAN coord */
  uint8_t  rxmode;              /* Reception mode: Normal, no CRC, promiscuous */

  /* TX state */

  bool     txdelayed_busy;      /* No CSMA transfer owned by the radio */
  bool     txdelayed_pending;   /* No CSMA transfer not started yet */
  bool     csma_pending;        /* CSMA transfer not started yet */
  uint32_t txdelayed_delay;     /* Delay of the pending no CSMA transfer */
};

/* IEEE 802.15.4 radio device */

struct nrf54l_radioi8_dev_s
{
  /* The public device instance - must be first */

  struct ieee802154_radio_s macops;

  /* Registered callbacks */

  struct ieee802154_radiocb_s *radiocb;

  /* Radio interface */

  struct nrf54l_radioi8_radio_s *radio;

  /* High resolution timer */

  struct nrf54l_radioi8_tim_s *tim;

  /* Low resolution, low power timer */

  /* Radio state */

  struct nrf54l_radioi8_state_s state;

  /* Exclusive access to this struct */

  mutex_t lock;
};

#endif  /* __ARCH_ARM_SRC_NRF54L_NRF54L_IEEE802154_PRIV_H */
