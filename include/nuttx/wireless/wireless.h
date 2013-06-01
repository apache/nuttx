/************************************************************************************
 * include/nuttx/wireless/wireless.h
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <gnutt@nuttx.org>
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
 ************************************************************************************/

/* This file includes common definitions to be used in all wireless drivers
 * (when applicable).
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_H
#define __INCLUDE_NUTTX_WIRELESS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_WIRELESS

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* IOCTL Commands *******************************************************************/

#define WLIOC_SETRADIOFREQ     _WLIOC(0x0001)  /* arg: Pointer to uint32_t, frequency value (in Mhz) */
#define WLIOC_GETRADIOFREQ     _WLIOC(0x0002)  /* arg: Pointer to uint32_t, frequency value (in Mhz) */
#define WLIOC_SETADDR          _WLIOC(0x0003)  /* arg: Pointer to address value, format of the address is driver specific */
#define WLIOC_GETADDR          _WLIOC(0x0004)  /* arg: Pointer to address value, format of the address is driver specific */
#define WLIOC_SETTXPOWER       _WLIOC(0x0005)  /* arg: Pointer to int32_t, output power (in dBm) */
#define WLIOC_GETTXPOWER       _WLIOC(0x0006)  /* arg: Pointer to int32_t, output power (in dBm) */

/* Wireless drivers can provide additional, device specific ioctl
 * commands, beginning with this value:
 */

#define WLIOC_USER              0x0007         /* Lowest, unused WL ioctl command */

#define _WLIOC_USER(nr)         _WLIOC(nr + WLIOC_USER)

#endif

#endif  /* __INCLUDE_NUTTX_WIRELESS_H */
