/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_radio.h
 *
 *   Copyright (C) 2014-2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* None at the moment */

/* IEEE 802.15.4 MAC Interface **********************************************/

/* Frame control field masks, 2 bytes 
 * Seee IEEE 802.15.4/2003 7.2.1.1 page 112
 */

#define IEEE802154_FC1_FTYPE   0x03 /* Frame type, bits 0-2 */
#define IEEE802154_FC1_SEC     0x08 /* Security Enabled, bit 3 */
#define IEEE802154_FC1_PEND    0x10 /* Frame pending, bit 4 */
#define IEEE802154_FC1_ACKREQ  0x20 /* Acknowledge request, bit 5 */
#define IEEE802154_FC1_INTRA   0x40 /* Intra PAN, bit 6 */
#define IEEE802154_FC2_DADDR   0x0C /* Dest   addressing mode, bits 10-11 */
#define IEEE802154_FC2_VERSION 0x30 /* Source addressing mode, bits 12-13 */
#define IEEE802154_FC2_SADDR   0xC0 /* Source addressing mode, bits 14-15 */

/* Frame Type */

#define IEEE802154_FRAME_BEACON  0x00
#define IEEE802154_FRAME_DATA    0x01
#define IEEE802154_FRAME_ACK     0x02
#define IEEE802154_FRAME_COMMAND 0x03

/* Security Enabled */

#define IEEE802154_SEC_OFF       0x00
#define IEEE802154_SEC_ON        0x08

/* Flags */

#define IEEE802154_PEND          0x10
#define IEEE802154_ACK_REQ       0x20
#define IEEE802154_INTRA         0x40

/* Dest Addressing modes */

#define IEEE802154_DADDR_NONE    0x00
#define IEEE802154_DADDR_SHORT   0x08
#define IEEE802154_DADDR_EXT     0x0A

/* Src Addressing modes */

#define IEEE802154_SADDR_NONE    0x00
#define IEEE802154_SADDR_SHORT   0x80
#define IEEE802154_SADDR_EXT     0xA0

/* Some addresses */

#define IEEE802154_PAN_DEFAULT  (uint16_t)0xFFFF
#define IEEE802154_SADDR_UNSPEC (uint16_t)0xFFFF
#define IEEE802154_SADDR_BCAST  (uint16_t)0xFFFE
#define IEEE802154_EADDR_UNSPEC (uint8_t*)"\xff\xff\xff\xff\xff\xff\xff\xff"

#define IEEE802154_CMD_ASSOC_REQ      0x01
#define IEEE802154_CMD_ASSOC_RSP      0x02
#define IEEE802154_CMD_DIS_NOT        0x03
#define IEEE802154_CMD_DATA_REQ       0x04
#define IEEE802154_CMD_PANID_CONF_NOT 0x05
#define IEEE802154_CMD_ORPHAN_NOT     0x06
#define IEEE802154_CMD_BEACON_REQ     0x07
#define IEEE802154_CMD_COORD_REALIGN  0x08
#define IEEE802154_CMD_GTS_REQ        0x09

/* Device modes */

#define IEEE802154_MODE_DEVICE        0x00
#define IEEE802154_MODE_COORD         0x01 /* avail in mrf24j40, but why? */
#define IEEE802154_MODE_PANCOORD      0x02

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ieee802154_packet_s
{
  uint8_t len;
  uint8_t data[127];
  uint8_t lqi;
  uint8_t rssi;
};

struct ieee802154_cca_s
{
  uint8_t use_ed  : 1; /* CCA using ED */
  uint8_t use_cs  : 1; /* CCA using carrier sense */
  uint8_t edth;     /* Energy detection threshold for CCA */
  uint8_t csth;     /* Carrier sense threshold for CCA */
};

struct ieee802154_dev_s;

struct ieee802154_devops_s
{
  CODE int (*setchannel)(FAR struct ieee802154_dev_s *dev, uint8_t channel);
  CODE int (*getchannel)(FAR struct ieee802154_dev_s *dev,
             FAR uint8_t *channel);

  CODE int (*setpanid)(FAR struct ieee802154_dev_s *dev, uint16_t panid);
  CODE int (*getpanid)(FAR struct ieee802154_dev_s *dev,
             FAR uint16_t *panid);

  CODE int (*setsaddr)(FAR struct ieee802154_dev_s *dev, uint16_t saddr);
  CODE int (*getsaddr)(FAR struct ieee802154_dev_s *dev,
             FAR uint16_t *saddr);

  CODE int (*seteaddr)(FAR struct ieee802154_dev_s *dev,
             FAR uint8_t *laddr);
  CODE int (*geteaddr)(FAR struct ieee802154_dev_s *dev,
             FAR uint8_t *laddr);

  CODE int (*setpromisc)(FAR struct ieee802154_dev_s *dev, bool promisc);
  CODE int (*getpromisc)(FAR struct ieee802154_dev_s *dev,
             FAR bool *promisc);

  CODE int (*setdevmode)(FAR struct ieee802154_dev_s *dev, uint8_t devmode);
  CODE int (*getdevmode)(FAR struct ieee802154_dev_s *dev,
             FAR uint8_t *devmode);

  CODE int (*settxpower)(FAR struct ieee802154_dev_s *dev,
             int32_t txpwr);  /* unit = 1 mBm = 1/100 dBm */
  CODE int (*gettxpower)(FAR struct ieee802154_dev_s *dev,
             FAR int32_t *txpwr);

  CODE int (*setcca)(FAR struct ieee802154_dev_s *dev,
             FAR struct ieee802154_cca_s *cca);
  CODE int (*getcca)(FAR struct ieee802154_dev_s *dev,
             FAR struct ieee802154_cca_s *cca);

  CODE int (*ioctl)(FAR struct ieee802154_dev_s *ieee, int cmd,
             unsigned long arg);
  CODE int (*energydetect)(FAR struct ieee802154_dev_s *dev,
             FAR uint8_t *energy);
  CODE int (*rxenable)(FAR struct ieee802154_dev_s *dev, bool state,
             FAR struct ieee802154_packet_s *packet);
  CODE int (*transmit)(FAR struct ieee802154_dev_s *dev,
             FAR struct ieee802154_packet_s *packet);

  /*TODO beacon/sf order*/
};

struct ieee802154_dev_s
{
  FAR const struct ieee802154_devops_s *ops;

  /* Packet reception management */

  struct ieee802154_packet_s *rxbuf;
  sem_t rxsem;

  /* Packet transmission management */

  sem_t txsem;
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_MRF24J40_H */
