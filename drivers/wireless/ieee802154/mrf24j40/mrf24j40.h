/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_MRF24J40_MRF24J40_H
#define __DRIVERS_WIRELESS_IEEE802154_MRF24J40_MRF24J40_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MRF24J40_GTS_SLOTS 2

/* Definitions for the device structure */

#define MRF24J40_RXMODE_NORMAL  0
#define MRF24J40_RXMODE_PROMISC 1
#define MRF24J40_RXMODE_NOCRC   2

#define MRF24J40_MODE_DEVICE    0
#define MRF24J40_MODE_COORD     1
#define MRF24J40_MODE_PANCOORD  2

/* Definitions for PA control on high power modules */

#define MRF24J40_PA_AUTO   1
#define MRF24J40_PA_ED     2
#define MRF24J40_PA_SLEEP  3

/* Formula for calculating default macMaxFrameWaitTime is on pg. 130
 *
 * For PHYs other than CSS and UWB, the attribute phyMaxFrameDuration
 * is given by:
 *
 * phyMaxFrameDuration = phySHRDuration +
 *                       ceiling([aMaxPHYPacketSize + 1] x
 *                       phySymbolsPerOctet)
 *
 * where ceiling() is a function that returns the smallest integer value
 * greater than or equal to its argument value. [1] pg. 158
 */

#define MRF24J40_DEFAULT_MAX_FRAME_WAITTIME 1824

#define MRF24J40_SYMBOL_DURATION_PS 16000000

/* Clock configuration macros */

#define MRF24J40_BEACONINTERVAL_NSEC(beaconorder) \
  (IEEE802154_BASE_SUPERFRAME_DURATION * (1 << beaconorder) * (16 * 1000))

#define MRF24J40_SUPERFRAMEDURATION_NSEC(sforder) \
  (IEEE802154_BASE_SUPERFRAME_DURATION * (1 << sforder) * (16 * 1000))

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue required in this driver
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_SPIMODE
#  define CONFIG_IEEE802154_MRF24J40_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_IEEE802154_MRF24J40_FREQUENCY
#  define CONFIG_IEEE802154_MRF24J40_FREQUENCY 8000000
#endif

#ifndef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE required for this driver
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A MRF24J40 device instance */

struct mrf24j40_radio_s
{
  struct ieee802154_radio_s radio;          /* The public device instance */
  FAR struct ieee802154_radiocb_s *radiocb; /* Registered callbacks */

  /* Low-level MCU-specific support */

  FAR const struct mrf24j40_lower_s *lower;
  FAR struct spi_dev_s *spi; /* Saved SPI interface instance */

  struct work_s irqwork;       /* For deferring interrupt work to work queue */
  struct work_s csma_pollwork; /* For deferring poll work to the work queue */
  struct work_s gts_pollwork;  /* For deferring poll work to the work queue */

  mutex_t       lock;          /* Exclusive access to this struct */

  /* MAC Attributes */

  struct ieee802154_addr_s addr;

  uint8_t chan;                /* 11 to 26 for the 2.4 GHz band */
  uint8_t devmode;             /* device mode: device, coord, pancoord */
  uint8_t paenabled;           /* enable usage of PA */
  uint8_t rxmode;              /* Reception mode: Main, no CRC, promiscuous */
  int32_t txpower;             /* TX power in mBm = dBm/100 */
  struct ieee802154_cca_s cca; /* Clear channel assessement method */
  uint32_t slpclkper;          /* Sleep clock period (nanoseconds) */

  /* MAC PIB attributes */

  uint32_t max_frame_waittime;

  struct ieee802154_txdesc_s *txdelayed_desc;
  struct ieee802154_txdesc_s *csma_desc;
  bool txdelayed_busy : 1;
  bool csma_busy : 1;
  bool reschedule_csma : 1;

  bool rxenabled : 1;

  uint8_t bsn;

  struct ieee802154_txdesc_s *gts_desc[MRF24J40_GTS_SLOTS];
  bool gts_busy[MRF24J40_GTS_SLOTS];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_spi_lock
 *
 * Description:
 *   Acquire exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static inline void mrf24j40_spi_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 1);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, CONFIG_IEEE802154_MRF24J40_SPIMODE);
  SPI_SETFREQUENCY(spi, CONFIG_IEEE802154_MRF24J40_FREQUENCY);
}

/****************************************************************************
 * Name: mrf24j40_spi_unlock
 *
 * Description:
 *   Release exclusive access to the shared SPI bus.
 *
 ****************************************************************************/

static inline void mrf24j40_spi_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, 0);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  mrf24j40_interrupt(int irq, FAR void *context, FAR void *arg);
void mrf24j40_irqworker(FAR void *arg);

void mrf24j40_dopoll_csma(FAR void *arg);
void mrf24j40_dopoll_gts(FAR void *arg);

void mrf24j40_norm_setup(FAR struct mrf24j40_radio_s *dev,
              FAR struct iob_s *frame, bool csma);
void  mrf24j40_gts_setup(FAR struct mrf24j40_radio_s *dev, uint8_t gts,
              FAR struct iob_s *frame);
void mrf24j40_setup_fifo(FAR struct mrf24j40_radio_s *dev,
              FAR const uint8_t *buf, uint8_t length, uint32_t fifo_addr);

void mrf24j40_norm_trigger(FAR struct mrf24j40_radio_s *dev);
void mrf24j40_beacon_trigger(FAR struct mrf24j40_radio_s *dev);

#endif /* __DRIVERS_WIRELESS_IEEE802154_MRF24J40_MRF24J40_H */
