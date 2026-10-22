/****************************************************************************
 * arch/arm/src/nrf53/nrf53_ieee802154_radio.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_IEEE802154_RADIO_H
#define __ARCH_ARM_SRC_NRF53_NRF53_IEEE802154_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wqueue.h>

#include "hardware/nrf53_radio.h"

#include "nrf53_radio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IEEE 802.15.4 constants */

#define IEEE802154_MAX_FRAME_WAITTIME (16 + 32 + 4064)
#define IEEE802154_MAX_CSMA_BACKOFFS  (4)
#define IEEE802154_MIN_BE             (3)
#define IEEE802154_MAX_BE             (5)
#define IEEE802154_CW0                (2)

/* macAckWaitDuration: aUnitBackoffPeriod + aTurnaroundTime +
 * phySHRDuration (10) + ceiling(6 * phySymbolsPerOctet (2)), in symbols
 * (IEEE 802.15.4-2011 6.4.3)
 */

#define IEEE802154_ACK_WAIT           (IEEE802154_UNIT_BACKOFF_PERIOD + \
                                       IEEE802154_TURN_AROUND_TIME +    \
                                       10 + 12)

/* RADIO ramp-up time in fast ramp-up mode */

#define NRF53_RADIOI8_RAMPUP_US       (40)

/* Air time of a frame with the given PHR: SHR, PHR and PSDU, 32 us/octet */

#define NRF53_RADIOI8_FRAME_US(phr)   (((phr) + 6) * 2 * IEEE802154_SYMBOL_US)

/* IFS after a frame with the given PHR */

#define NRF53_RADIOI8_IFS_US(phr)                       \
  (((phr) > IEEE802154_MAX_SIFS_FRAME_SIZE ?            \
    IEEE802154_LIFS_SYMBOLS : IEEE802154_SIFS_SYMBOLS) * \
   IEEE802154_SYMBOL_US)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Radio state */

enum nrf53_radioi8_state_e
{
  NRF53_RADIO_STATE_DISABLED  = 0, /* Radio disabled */
  NRF53_RADIO_STATE_TX_CSMA,       /* CSMA TX armed */
  NRF53_RADIO_STATE_TX_NOCSMA,     /* Non CSMA TX armed */
  NRF53_RADIO_STATE_TX,            /* TX on the air */
  NRF53_RADIO_STATE_RX,            /* RX active */
  NRF53_RADIO_STATE_ACKTX,         /* Transmitting ACK now */
  NRF53_RADIO_STATE_ED,            /* Energy detection now */
  NRF53_RADIO_STATE_BEACONTX,      /* Transmitting beacon now */
};

/* Radio data */

struct nrf53_radioi8_radio_data_s
{
  /* Radio state */

  uint8_t  state;

  /* CSMA-CA */

  uint8_t  NB;                  /* Number of required back offs */
  uint8_t  CW;                  /* Contention window length */
  uint8_t  BE;                  /* Backoff exponent */

  /* RX state */

  bool     rxenabled;           /* RX enabled now */
  bool     rxrestore;           /* RX needs to be restored */
  bool     edrestore;           /* RX needs to be restored after ED */
  bool     rxlong;              /* Last RX frame was long */

  /* TX state */

  bool     waitack;             /* TX needs ACK */
  bool     framepending;        /* Frame pending from the last TX ACK */
  bool     csma_busy;           /* CSMA-CA transaction in progress */
  bool     csma_paused;         /* Slotted CSMA-CA waits for the next CAP */
  bool     slotted;             /* Slotted CSMA-CA */
  uint8_t  bsn;                 /* macBSN, the beacon sequence number */

  /* Radio configuration */

  uint32_t max_frame_waittime;  /* Max Frame wait time */
  uint8_t  max_csma_backoffs;   /* Max CSMA backoffs */
  uint8_t  min_be;              /* Min backoff exponent (BE) */
  uint8_t  max_be;              /* Max backoff exponent (BE) */

#ifdef CONFIG_NRF53_RADIO_IEEE802154_SUPERFRAME
  /* Superframe data */

  bool     sftrack;             /* Device tracks the beacons */
  bool     ble;                 /* Battery life extension */
  bool     beacon_pending;      /* A beacon waits for the radio */
  uint8_t  backoffs;            /* Backoff periods before the next CCA */
  uint32_t boundary;            /* Backoff period boundary of the next step */
#endif
};

/* Forward reference */

struct nrf53_radioi8_radio_s;
struct ieee802154_radio_s;
struct nrf53_radioi8_dev_s;
struct ieee802154_cca_s;

/* Radio ops */

struct nrf53_radioi8_radio_ops_s
{
  /* Start transmission - TX must be armed (TXEN set) */

  void (*txstart)(struct nrf53_radioi8_dev_s *dev);

  /* Start CCA - RX must be armed (RXEN set) */

  void (*ccastart)(struct nrf53_radioi8_dev_s *dev);

  /* Notify MAC about no ACK */

  void (*notify_noack)(struct nrf53_radioi8_dev_s *dev);

  /* Enable/Disable receiver */

  int (*rxenable)(struct nrf53_radioi8_dev_s *dev, bool enable);

  /* Start the energy detect measurement */

  int (*energydetect)(struct nrf53_radioi8_dev_s *dev, uint32_t nsymbols);

  /* Define the current radio channel the device is operating on */

  int (*setchannel)(struct nrf53_radioi8_dev_s *dev, uint8_t chan);

  /* Configure the Clear Channel Assessement */

  int (*setcca)(struct nrf53_radioi8_dev_s *dev,
                struct ieee802154_cca_s *cca);

  /* Setup a normal transaction (non GTS) */

  void (*norm_setup)(struct nrf53_radioi8_dev_s *dev,
                     struct iob_s *frame, bool csma);

  /* Trigger normal transaction (non GTS) */

  void (*norm_trigger)(struct nrf53_radioi8_dev_s *dev);

  /* Start the next frame that waits for the radio, after mindelay */

  void (*txnext)(struct nrf53_radioi8_dev_s *dev, uint32_t mindelay);

#ifdef CONFIG_NRF53_RADIO_IEEE802154_SUPERFRAME
  /* Setup a beacon frame transfe */

  void (*beacon_setup)(struct nrf53_radioi8_dev_s *dev,
                       uint8_t *data, uint8_t len);

  /* Transmit a beacon frame (non CSMA-CA transfer) */

  void (*beacon_tx)(struct nrf53_radioi8_dev_s *dev);
#endif

  /* Reset radio state to work with IEEE 802.15.4 */

  int (*reset)(struct nrf53_radioi8_radio_s *dev);

  /* Handle TX poll (no GTS) */

  int (*csma_poll)(struct nrf53_radioi8_dev_s *dev);

  /* Handle GTS poll */

  int (*gts_poll)(struct nrf53_radioi8_dev_s *dev);
};

/* Radio interface */

struct nrf53_radioi8_radio_s
{
  /* Radio lower-half */

  struct nrf53_radio_dev_s *lower;

  /* IEEE 802.15.4 radio operations */

  struct nrf53_radioi8_radio_ops_s *ops;

  /* Packet buffers */

  uint8_t *rxbuf;
  uint8_t *txbuf;
  uint8_t *ackbuf;
#ifdef CONFIG_NRF53_RADIO_IEEE802154_SUPERFRAME
  uint8_t *beaconbuf;
#endif

  /* For deferring interrupts work */

  struct work_s irqwork;

  /* For deferring no ACK work */

  struct work_s noackwork;

  /* For deferring poll work to the work queue */

  struct work_s csma_pollwork;

#ifdef CONFIG_NRF53_RADIO_IEEE802154_SUPERFRAME
  /* For deferring poll work to the work queue */

  struct work_s gts_pollwork;
#endif

  /* Radio data */

  struct nrf53_radioi8_radio_data_s state;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_radioi8_radio_init
 *
 * Description:
 *   Initialize RADIO for IEEE802154 operations.
 *
 ****************************************************************************/

struct nrf53_radioi8_radio_s *
nrf53_radioi8_radio_init(struct nrf53_radioi8_dev_s *dev,
                         struct nrf53_radio_board_s *board);

#endif  /* __ARCH_ARM_SRC_NRF53_NRF53_IEEE802154_RADIO_H */
