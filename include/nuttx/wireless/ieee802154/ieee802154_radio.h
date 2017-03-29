/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_radio.h
 *
 *   Copyright (C) 2014-2016 Sebastien Lorquet. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* IEEE 802.15.4 Radio Interface **********************************************/

/* This layer only knows radio frames. There are no 802.15.4 specific bits
 * at this layer. */

/* Device modes */

#define IEEE802154_MODE_DEVICE        0x00
#define IEEE802154_MODE_COORD         0x01 /* avail in mrf24j40, but why? */
#define IEEE802154_MODE_PANCOORD      0x02

/* IEEE 802.15.4 Radio Character Driver IOCTL ********************************/

#define PHY802154IOC_SET_CHAN         _PHY802154IOC(0x0001)
#define PHY802154IOC_GET_CHAN         _PHY802154IOC(0x0002)

#define PHY802154IOC_SET_PANID        _PHY802154IOC(0x0003)
#define PHY802154IOC_GET_PANID        _PHY802154IOC(0x0004)

#define PHY802154IOC_SET_SADDR        _PHY802154IOC(0x0005)
#define PHY802154IOC_GET_SADDR        _PHY802154IOC(0x0006)

#define PHY802154IOC_SET_EADDR        _PHY802154IOC(0x0007)
#define PHY802154IOC_GET_EADDR        _PHY802154IOC(0x0008)

#define PHY802154IOC_SET_PROMISC      _PHY802154IOC(0x0009)
#define PHY802154IOC_GET_PROMISC      _PHY802154IOC(0x000A)

#define PHY802154IOC_SET_DEVMODE      _PHY802154IOC(0x000B)
#define PHY802154IOC_GET_DEVMODE      _PHY802154IOC(0x000C)

#define PHY802154IOC_SET_TXPWR        _PHY802154IOC(0x000D)
#define PHY802154IOC_GET_TXPWR        _PHY802154IOC(0x000E)

#define PHY802154IOC_SET_CCA          _PHY802154IOC(0x000F)
#define PHY802154IOC_GET_CCA          _PHY802154IOC(0x0010)

#define PHY802154IOC_ENERGYDETECT     _PHY802154IOC(0x0011)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ieee802154_cca_s
{
  uint8_t use_ed  : 1; /* CCA using ED */
  uint8_t use_cs  : 1; /* CCA using carrier sense */
  uint8_t edth;        /* Energy detection threshold for CCA */
  uint8_t csth;        /* Carrier sense threshold for CCA */
};

struct ieee802154_packet_s 
{ 
  uint8_t len; 
  uint8_t data[127]; 
  uint8_t lqi; 
  uint8_t rssi; 
};

struct ieee802154_radio_s;

struct ieee802154_radioops_s
{
  CODE int (*setchannel)(FAR struct ieee802154_radio_s *dev,
             uint8_t channel);
  CODE int (*getchannel)(FAR struct ieee802154_radio_s *dev,
             FAR uint8_t *channel);

  CODE int (*setpanid)(FAR struct ieee802154_radio_s *dev, uint16_t panid);
  CODE int (*getpanid)(FAR struct ieee802154_radio_s *dev,
             FAR uint16_t *panid);

  CODE int (*setsaddr)(FAR struct ieee802154_radio_s *dev, uint16_t saddr);
  CODE int (*getsaddr)(FAR struct ieee802154_radio_s *dev,
             FAR uint16_t *saddr);

  CODE int (*seteaddr)(FAR struct ieee802154_radio_s *dev,
             FAR uint8_t *laddr);
  CODE int (*geteaddr)(FAR struct ieee802154_radio_s *dev,
             FAR uint8_t *laddr);

  CODE int (*setpromisc)(FAR struct ieee802154_radio_s *dev, bool promisc);
  CODE int (*getpromisc)(FAR struct ieee802154_radio_s *dev,
             FAR bool *promisc);

  CODE int (*setdevmode)(FAR struct ieee802154_radio_s *dev,
             uint8_t devmode);
  CODE int (*getdevmode)(FAR struct ieee802154_radio_s *dev,
             FAR uint8_t *devmode);

  CODE int (*settxpower)(FAR struct ieee802154_radio_s *dev,
             int32_t txpwr);  /* unit = 1 mBm = 1/100 dBm */
  CODE int (*gettxpower)(FAR struct ieee802154_radio_s *dev,
             FAR int32_t *txpwr);

  CODE int (*setcca)(FAR struct ieee802154_radio_s *dev,
             FAR struct ieee802154_cca_s *cca);
  CODE int (*getcca)(FAR struct ieee802154_radio_s *dev,
             FAR struct ieee802154_cca_s *cca);

  CODE int (*ioctl)(FAR struct ieee802154_radio_s *ieee, int cmd,
             unsigned long arg);
  CODE int (*energydetect)(FAR struct ieee802154_radio_s *dev,
             FAR uint8_t *energy);
  CODE int (*rxenable)(FAR struct ieee802154_radio_s *dev, bool state,
             FAR struct ieee802154_packet_s *packet);
  CODE int (*transmit)(FAR struct ieee802154_radio_s *dev,
             FAR struct ieee802154_packet_s *packet);

  /*TODO beacon/sf order*/
};

struct ieee802154_radio_s
{
  FAR const struct ieee802154_radioops_s *ops;

  /* Packet reception management */

  struct ieee802154_packet_s *rxbuf; /* packet reception buffer, filled by
                                      * rx interrupt, NULL if rx not enabled */
  sem_t rxsem;                       /* Semaphore posted after reception of
                                      * a packet */

  /* Packet transmission management */

  bool txok;                         /* Last transmission status, filled by
                                      * tx interrupt */
  bool txbusy;                       /* Last transmission failed because
                                      * channel busy */
  uint8_t txretries;                 /* Last transmission required this much
                                      * retries */
  sem_t txsem;                       /* Semaphore posted after transmission
                                      * of a packet */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: radio802154dev_register
 *
 * Description:
 *   Register a character driver to access the IEEE 802.15.4 radio from
 *   user-space
 *
 * Input Parameters:
 *   radio - Pointer to the radio struct to be registerd.
 *   devname - The name of the IEEE 802.15.4 radio to be registered.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IEEE802154_DEV
int radio802154dev_register(FAR struct ieee802154_radio_s *radio,
                            FAR char *devname);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H */
