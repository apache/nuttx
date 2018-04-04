/************************************************************************************
 * include/nuttx/wireless/ioctl.h
 * Wireless character driver IOCTL commands
 *
 *   Copyright (C) 2011-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <gnutt@nuttx.org>
 *           Gregory Nutt <gnutt@nuttx.org>
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

/* This file includes common definitions to be used in all wireless character drivers
 * (when applicable).
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_IOCTL_H
#define __INCLUDE_NUTTX_WIRELESS_IOCTL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_DRIVERS_WIRELESS

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Character Driver IOCTL commands *************************************************/
/* Non-compatible, NuttX only IOCTL definitions for use with low-level wireless
 * drivers that are accessed via a character device.  Use of these IOCTL commands
 * requires a file descriptor created by the open() interface.
 */

#define WLIOC_SETRADIOFREQ  _WLCIOC(0x0001)  /* arg: Pointer to uint32_t, frequency
                                             * value (in Mhz) */
#define WLIOC_GETRADIOFREQ  _WLCIOC(0x0002)  /* arg: Pointer to uint32_t, frequency
                                             * value (in Mhz) */
#define WLIOC_SETADDR       _WLCIOC(0x0003)  /* arg: Pointer to address value, format
                                             * of the address is driver specific */
#define WLIOC_GETADDR       _WLCIOC(0x0004)  /* arg: Pointer to address value, format
                                             * of the address is driver specific */
#define WLIOC_SETTXPOWER    _WLCIOC(0x0005)  /* arg: Pointer to int32_t, output power
                                             * (in dBm) */
#define WLIOC_GETTXPOWER    _WLCIOC(0x0006)  /* arg: Pointer to int32_t, output power
                                             * (in dBm) */

/* Device-specific IOCTL commands **************************************************/

#define WL_FIRST            0x0001          /* First common command */
#define WL_NCMDS            0x0006          /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half QE driver to the lower-half QE driver via the ioctl()
 * method fo the QE lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */


/* See include/nuttx/wireless/nrf24l01.h */

#define NRF24L01_FIRST      (WL_FIRST + WL_NCMDS)
#define NRF24L01_NCMDS      14

/************************************************************************************
 * Public Types
 ************************************************************************************/

#endif /* CONFIG_DRIVERS_WIRELESS */
#endif /* __INCLUDE_NUTTX_WIRELESS_IOCTL_H */
