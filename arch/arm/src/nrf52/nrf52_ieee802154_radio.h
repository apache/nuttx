/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ieee802154_radio.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RADIO_H
#define __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wqueue.h>

#include "hardware/nrf52_radio.h"

#include "nrf52_radio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IEEE 802.15.4 constants */

#define IEEE802154_MAX_FRAME_WAITTIME (16 + 32 + 4064)
#define IEEE802154_MAX_CSMA_BACKOFFS  (1)
#define IEEE802154_MIN_BE             (3)
#define IEEE802154_MAX_BE             (5)
#define IEEE802154_CW0                (2)

/* ACK_SYM + turnaround symbols + backoff */

#define IEEE802154_ACK_WAIT           (12+                              \
                                       IEEE802154_TURN_AROUND_TIME+     \
                                       IEEE802154_UNIT_BACKOFF_PERIOD)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Radio state */

enum nrf52_radioi8_state_e
{
  NRF52_RADIO_STATE_DISABLED  = 0, /* Radio disabled */
  NRF52_RADIO_STATE_TX_CSMA,       /* CSMA TX armed */
  NRF52_RADIO_STATE_TX_NOCSMA,     /* Non CSMA TX armed */
  NRF52_RADIO_STATE_TX,            /* TX on the air */
  NRF52_RADIO_STATE_RX,            /* RX active */
  NRF52_RADIO_STATE_ACKTX,         /* Transmiting ACK now */
  NRF52_RADIO_STATE_ED,            /* Energy detection now */
};

/* Radio data */

struct nrf52_radioi8_radio_data_s
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
  bool     rxlong;              /* Last RX frame was long */

  /* TX state */

  bool     waitack;             /* TX needs ACK */
  bool     framepending;        /* Frame pending from the last TX ACK */
  bool     csma_busy;           /* Un-slotted CSMA busy */
  bool     slotted;             /* Slotted CSMA-CA */

  /* Radio configuration */

  uint32_t max_frame_waittime;  /* Max Frame wait time */
  uint8_t  max_csma_backoffs;   /* Max CSMA backoffs */
  uint8_t  min_be;              /* Min backoff exponent (BE) */
  uint8_t  max_be;              /* Max backoff exponent (BE) */

#ifdef CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME
  /* Superframe data */

  bool    wait_for_beacon;      /* Device wait for beacon */
  bool    ble;                  /* Batter life extension */
#endif
};

/* Forward reference */

struct nrf52_radioi8_radio_s;
struct ieee802154_radio_s;
struct nrf52_radioi8_dev_s;
struct ieee802154_cca_s;

/* Radio ops */

struct nrf52_radioi8_radio_ops_s
{
  /* Start transmition - TX must be armed (TXEN set) */

  void (*txstart)(struct nrf52_radioi8_dev_s *dev);

  /* Start CCA - RX must be armed (RXEN set) */

  void (*ccastart)(struct nrf52_radioi8_dev_s *dev);

  /* Notify MAC about no ACK */

  void (*notify_noack)(struct nrf52_radioi8_dev_s *dev);

  /* Enable/Disable receiver */

  int (*rxenable)(struct nrf52_radioi8_dev_s *dev, bool enable);

  /* Start the energy detect measurement */

  int (*energydetect)(struct nrf52_radioi8_dev_s *dev, uint32_t nsymbols);

  /* Define the current radio channel the device is operating on */

  int (*setchannel)(struct nrf52_radioi8_dev_s *dev, uint8_t chan);

  /* Configure the Clear Channel Assessement */

  int (*setcca)(struct nrf52_radioi8_dev_s *dev,
                struct ieee802154_cca_s *cca);

  /* Setup a normal transaction (non GTS) */

  void (*norm_setup)(struct nrf52_radioi8_dev_s *dev,
                     struct iob_s *frame, bool csma);

  /* Trigger normal transaction (non GTS) */

  void (*norm_trigger)(struct nrf52_radioi8_dev_s *dev);

#ifdef CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME
  /* Setup a beacon frame transfe */

  void (*beacon_setup)(struct nrf52_radioi8_dev_s *dev,
                       uint8_t *data, uint8_t len);

  /* Transmit a beacon frame (non CSMA-CA transfer) */

  void (*beacon_tx)(struct nrf52_radioi8_dev_s *dev);
#endif

  /* Reset radio state to work with IEEE 802.15.4 */

  int (*reset)(struct nrf52_radioi8_radio_s *dev);

  /* Handle TX poll (no GTS) */

  int (*csma_poll)(struct nrf52_radioi8_dev_s *dev);

  /* Handle GTS poll */

  int (*gts_poll)(struct nrf52_radioi8_dev_s *dev);
};

/* Radio interface */

struct nrf52_radioi8_radio_s
{
  /* Radio lower-half */

  struct nrf52_radio_dev_s *lower;

  /* IEEE 802.15.4 radio operations */

  struct nrf52_radioi8_radio_ops_s *ops;

  /* Packet buffers */

  uint8_t *rxbuf;
  uint8_t *txbuf;
  uint8_t *ackbuf;
#ifdef CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME
  uint8_t *beaconbuf;
#endif

  /* For deferring interrupts work */

  struct work_s irqwork;

  /* For deferring no ACK work */

  struct work_s noackwork;

  /* For deferring poll work to the work queue */

  struct work_s csma_pollwork;

#ifdef CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME
  /* For deferring poll work to the work queue */

  struct work_s gts_pollwork;
#endif

  /* Radio data */

  struct nrf52_radioi8_radio_data_s state;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_radio_init
 *
 * Description:
 *   Initialize RADIO for IEEE802154 operations.
 *
 ****************************************************************************/

struct nrf52_radioi8_radio_s *
nrf52_radioi8_radio_init(struct nrf52_radioi8_dev_s *dev,
                         struct nrf52_radio_board_s *board);

#endif  /* __ARCH_ARM_SRC_NRF52_NRF52_IEEE802154_RADIO_H */
