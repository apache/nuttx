/****************************************************************************
 * include/nuttx/wireless/ieee80211/bcmf_board.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_BOARD_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <nuttx/irq.h>
#include <net/ethernet.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_board_initialize
 *
 * Description:
 *   Board specific function called from Broadcom FullMAC driver
 *   that must be implemented to configure WLAN chip GPIOs
 *
 * Input Parameters:
 *   minor - zero based minor device number which is unique
 *           for each wlan device.
 *
 ****************************************************************************/

void bcmf_board_initialize(int minor);

/****************************************************************************
 * Name: bcmf_board_power
 *
 * Description:
 *   Board specific function called from Broadcom FullMAC driver
 *   that must be implemented to power WLAN chip
 *
 * Input Parameters:
 *   minor - zero based minor device number which is unique
 *           for each wlan device.
 *   power - true to power WLAN chip else false
 *
 ****************************************************************************/

void bcmf_board_power(int minor, bool power);

/****************************************************************************
 * Name: bcmf_board_reset
 *
 * Description:
 *   Board specific function called from Broadcom FullMAC driver
 *   that must be implemented to reset WLAN chip
 *
 * Input Parameters:
 *   minor - zero based minor device number which is unique
 *           for each wlan device.
 *   reset - true to set WLAN chip in reset state else false
 *
 ****************************************************************************/

void bcmf_board_reset(int minor, bool reset);

/****************************************************************************
 * Function: bcmf_board_setup_oob_irq
 *
 * Description:
 *   Board specific function called from Broadcom FullMAC driver
 *   that must be implemented to use WLAN chip interrupt signal
 *
 * Input Parameters:
 *   minor - zero based minor device number which is unique
 *           for each wlan device.
 *   func  - WLAN chip callback function that must be called on gpio event
 *   arg   - WLAN chip internal structure that must be passed to callback
 *
 ****************************************************************************/

void bcmf_board_setup_oob_irq(int minor, int (*func)(void *), void *arg);

/****************************************************************************
 * Name: bcmf_board_etheraddr
 *
 * Description:
 *   Board specific function called from Broadcom FullMAC driver
 *   that must be implemented to get the customized MAC address
 *
 * Returned Value:
 *   Return true if customized MAC address is set,
 *   otherwise use firmware default MAC address
 *
 * Input Parameters:
 *   ethaddr - Pointer to MAC address
 *
 ****************************************************************************/

bool bcmf_board_etheraddr(FAR struct ether_addr *ethaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_BOARD_H */
