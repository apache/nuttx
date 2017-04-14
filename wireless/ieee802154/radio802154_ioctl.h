/****************************************************************************
 * wireless/ieee802154/radio802154_ioctl.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __WIRELESS_IEEE802154_RADIO802154_IOCTL_H
#define __WIRELESS_IEEE802154_RADIO802154_IOCTL_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "radio802154_ioctl.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: radio802154_setchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *   In the 2.4 GHz, there are 16 channels, each 2 MHz wide, 5 MHz spacing:
 *   Chan   MHz       Chan   MHz       Chan   MHz       Chan   MHz
 *     11  2405         15  2425         19  2445         23  2465
 *     12  2410         16  2430         20  2450         24  2470
 *     13  2415         17  2435         21  2455         25  2475
 *     14  2420         18  2440         22  2460         26  2480
 *
 ****************************************************************************/

int radio802154_setchannel(FAR struct ieee802154_radio_s *radio,
                           uint8_t chan);

/****************************************************************************
 * Name: radio802154_getchannel
 *
 * Description:
 *   Define the current radio channel the device is operating on.
 *
 ****************************************************************************/

int radio802154_getchannel(FAR struct ieee802154_radio_s *radio,
                           FAR uint8_t *chan);

/****************************************************************************
 * Name: radio802154_setpanid
 *
 * Description:
 *   Define the PAN ID the device is operating on.
 *
 ****************************************************************************/

int radio802154_setpanid(FAR struct ieee802154_radio_s *radio,
                         uint16_t panid);

/****************************************************************************
 * Name: radio802154_getpanid
 *
 * Description:
 *   Define the current PAN ID the device is operating on.
 *
 ****************************************************************************/

int radio802154_getpanid(FAR struct ieee802154_radio_s *radio,
                         FAR uint16_t *panid);

/****************************************************************************
 * Name: radio802154_setsaddr
 *
 * Description:
 *   Define the device short address. The following addresses are special:
 *   FFFEh : Broadcast
 *   FFFFh : Unspecified
 *
 ****************************************************************************/

int radio802154_setsaddr(FAR struct ieee802154_radio_s *radio,
                         uint16_t saddr);

/****************************************************************************
 * Name: radio802154_getsaddr
 *
 * Description:
 *   Define the current short address the device is using.
 *
 ****************************************************************************/

int radio802154_getsaddr(FAR struct ieee802154_radio_s *radio,
                         FAR uint16_t *saddr);

/****************************************************************************
 * Name: radio802154_seteaddr
 *
 * Description:
 *   Define the device extended address. The following addresses are special:
 *   FFFFFFFFFFFFFFFFh : Unspecified
 *
 ****************************************************************************/

int radio802154_seteaddr(FAR struct ieee802154_radio_s *radio,
                         FAR uint8_t *eaddr);

/****************************************************************************
 * Name: radio802154_geteaddr
 *
 * Description:
 *   Define the current extended address the device is using.
 *
 ****************************************************************************/

int radio802154_geteaddr(FAR struct ieee802154_radio_s *radio,
                         FAR uint8_t *eaddr);

/****************************************************************************
 * Name: radio802154_setpromisc
 *
 * Description:
 *   Set the device into promiscuous mode, e.g do not filter any incoming
 *   frame.
 *
 ****************************************************************************/

int radio802154_setpromisc(FAR struct ieee802154_radio_s *radio,
                           bool promisc);

/****************************************************************************
 * Name: radio802154_getpromisc
 *
 * Description:
 *   Get the device receive mode.
 *
 ****************************************************************************/

int radio802154_getpromisc(FAR struct ieee802154_radio_s *radio,
                           FAR bool *promisc);

/****************************************************************************
 * Name: radio802154_setdevmode
 *
 * Description:
 *   Define the device behaviour: normal end device or coordinator
 *
 ****************************************************************************/

int radio802154_setdevmode(FAR struct ieee802154_radio_s *radio,
                           uint8_t mode);

/****************************************************************************
 * Name: radio802154_setdevmode
 *
 * Description:
 *   Return the current device mode
 *
 ****************************************************************************/

int radio802154_getdevmode(FAR struct ieee802154_radio_s *radio,
                           FAR uint8_t *mode);

/****************************************************************************
 * Name: radio802154_settxpower
 *
 * Description:
 *   Define the transmit power. Value is passed in mBm, it is rounded to
 *   the nearest value. Some MRF modules have a power amplifier, this routine
 *   does not care about this. We only change the CHIP output power.
 *
 ****************************************************************************/

int radio802154_settxpower(FAR struct ieee802154_radio_s *radio,
                           int32_t txpwr);

/****************************************************************************
 * Name: radio802154_gettxpower
 *
 * Description:
 *   Return the actual transmit power, in mBm.
 *
 ****************************************************************************/

int radio802154_gettxpower(FAR struct ieee802154_radio_s *radio,
                           FAR int32_t *txpwr);

/****************************************************************************
 * Name: radio802154_setcca
 *
 * Description:
 *   Define the Clear Channel Assessement method.
 *
 ****************************************************************************/

int radio802154_setcca(FAR struct ieee802154_radio_s *radio,
                       FAR struct ieee802154_cca_s *cca);

/****************************************************************************
 * Name: radio802154_getcca
 *
 * Description:
 *   Return the Clear Channel Assessement method.
 *
 ****************************************************************************/

int radio802154_getcca(FAR struct ieee802154_radio_s *radio,
                       FAR struct ieee802154_cca_s *cca);

/****************************************************************************
 * Name: radio802154_energydetect
 *
 * Description:
 *   Measure the RSSI level for the current channel.
 *
 ****************************************************************************/

int radio802154_energydetect(FAR struct ieee802154_radio_s *radio,
                             FAR uint8_t *energy);

#endif /* __WIRELESS_IEEE802154_RADIO802154_IOCTL_H */
