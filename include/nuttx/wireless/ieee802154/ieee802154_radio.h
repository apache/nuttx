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

#ifdef CONFIG_NET_6LOWPAN
#  include <net/if.h>
#endif

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

#define EADDR_SIZE                    8  /* REVISIT */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structures used with IEEE802.15.4 radio interface operations *************/

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

/* IOCTL command data argument **********************************************/

/* A pointer to this structure is passed as the argument of each IOCTL
 * command.
 */

union ieee802154_radioarg_u
{
  uint8_t  channel;            /* PHY802154IOC_GET/SET_CHAN */
  uint16_t panid;              /* PHY802154IOC_GET/SET_PANID */
  uint16_t saddr;              /* PHY802154IOC_GET/SET_SADDR */
  uint8_t  eaddr[EADDR_SIZE];  /* PHY802154IOC_GET/SET_EADDR */
  bool     promisc;            /* PHY802154IOC_GET/SET_EADDR */
  uint8_t  devmode;            /* PHY802154IOC_GET/SET_DEVMODE */
  int32_t  txpwr;              /* PHY802154IOC_GET/SET_TXPWR */
  bool     energy;             /* PHY802154IOC_ENERGYDETECT */
  struct ieee802154_cca_s cca; /* PHY802154IOC_GET/SET_CCA */
};

#ifdef CONFIG_NET_6LOWPAN
/* For the case of network IOCTLs, the network IOCTL to the MAC network
 * driver will include a device name like "wpan0" as the destination of
 * the IOCTL command.  The MAC layer will forward only the payload union
 * to the radio IOCTL method.
 */

struct ieee802154_netradio_s
{
  char ifr_name[IFNAMSIZ];       /* Interface name, e.g. "wpan0" */
  union ieee802154_radioarg_u u; /* Data payload */
};
#endif

/* IEEE802.15.4 Radio Interface Operations **********************************/
struct ieee802154_trans_s
{
  uint8_t retry_count;  /* The number of retries remaining */ 
  uint8_t msdu_handle;  /* The msdu handle identifying the transaction */

  uint16_t psdu_length; /* The length of the PSDU */

  /* The PHY Service Data Unit (PSDU) buffer representing the frame to be 
   * transmitted. This must be at the end of the struct to allow the array
   * to continue and make the struct "variable length". Users should allocate
   * memory using the SIZEOF_MAC802154_TRANSACTION_S macro below */

  uint8_t psdu[CONFIG_IEEE802154_MTU];
};

struct ieee802154_radio_s; /* Forward reference */

struct ieee802154_phyif_s
{
  CODE int (*poll_csma) (FAR struct ieee802154_phyif_s *phyif,
                         FAR struct ieee802154_txdesc_s *tx_desc,
                         uint8_t *buf);

  CODE int (*poll_gts) (FAR struct ieee802154_phyif_s *phyif,
                        FAR struct ieee802154_txdesc_s *tx_desc,
                        uint8_t *buf);

  /* Driver-specific information */

  void * priv;
};

struct ieee802154_radioops_s
{
  CODE int (*bind) (FAR struct ieee802154_radio_s *dev,
                    FAR const struct ieee802154_phyif_s *phyif);

  CODE int (*ioctl)(FAR struct ieee802154_radio_s *ieee, int cmd,
             unsigned long arg);

  CODE int (*rxenable)(FAR struct ieee802154_radio_s *dev, bool state,
             FAR struct ieee802154_packet_s *packet);

  CODE int (*transmit)(FAR struct ieee802154_radio_s *dev,
             FAR struct ieee802154_packet_s *packet);
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
