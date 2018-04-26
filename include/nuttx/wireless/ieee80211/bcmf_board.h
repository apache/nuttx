/****************************************************************************
 * include/nuttx/wireless/ieee80211/bcmf_board.h
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_BOARD_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <nuttx/irq.h>

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

/************************************************************************************
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
 ************************************************************************************/

void bcmf_board_initialize(int minor);

/************************************************************************************
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
 ************************************************************************************/

void bcmf_board_power(int minor, bool power);

/************************************************************************************
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
 ************************************************************************************/

void bcmf_board_reset(int minor, bool reset);

/************************************************************************************
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
 ************************************************************************************/

void bcmf_board_setup_oob_irq(int minor, int (*func)(void *), void *arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE80211_BCMF_BOARD_H */
