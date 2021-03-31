/****************************************************************************
 * include/nuttx/wireless/ioctl.h
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

/* This file includes common definitions to be used in all wireless
 * character drivers (when applicable).
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_IOCTL_H
#define __INCLUDE_NUTTX_WIRELESS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_DRIVERS_WIRELESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Character Driver IOCTL commands
 * Non-compatible, NuttX only IOCTL definitions for use with low-level
 * wireless drivers that are accessed via a character device.
 * Use of these IOCTL commands requires a file descriptor created by
 * the open() interface.
 ****************************************************************************/

#define WLIOC_SETRADIOFREQ  _WLCIOC(0x0001)  /* arg: Pointer to uint32_t, */
                                             /* frequency value (in MHz) */
#define WLIOC_GETRADIOFREQ  _WLCIOC(0x0002)  /* arg: Pointer to uint32_t, */
                                             /* frequency value (in MHz) */
#define WLIOC_SETADDR       _WLCIOC(0x0003)  /* arg: Pointer to address value, format
                                             * of the address is driver specific */
#define WLIOC_GETADDR       _WLCIOC(0x0004)  /* arg: Pointer to address value, format
                                             * of the address is driver specific */
#define WLIOC_SETTXPOWER    _WLCIOC(0x0005)  /* arg: Pointer to int32_t, */
                                             /* output power (in dBm) */
#define WLIOC_GETTXPOWER    _WLCIOC(0x0006)  /* arg: Pointer to int32_t, */
                                             /* output power (in dBm) */

/****************************************************************************
 * Device-specific IOCTL commands
 ****************************************************************************/

#define WL_FIRST            0x0001          /* First common command */
#define WL_NCMDS            0x0006          /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half driver to the lower-half driver via the ioctl()
 * method of the lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See include/nuttx/wireless/nrf24l01.h */

#define NRF24L01_FIRST      (WL_FIRST + WL_NCMDS)
#define NRF24L01_NCMDS      16

/* See include/nuttx/wireless/lpwan/sx127x.h */

#define SX127X_FIRST        (NRF24L01_FIRST + NRF24L01_NCMDS)
#define SX127X_NCMDS        11

/* See include/nuttx/wireless/gs2200m.h */

#define GS2200M_FIRST       (SX127X_FIRST + SX127X_NCMDS)
#define GS2200M_NCMDS       9

/****************************************************************************
 * Public Types
 ****************************************************************************/

#endif /* CONFIG_DRIVERS_WIRELESS */
#endif /* __INCLUDE_NUTTX_WIRELESS_IOCTL_H */
