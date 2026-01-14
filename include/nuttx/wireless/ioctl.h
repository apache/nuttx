/****************************************************************************
 * include/nuttx/wireless/ioctl.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * RF common IOCTL commands
 ****************************************************************************/

/* Offsets */

#define _WLIOC_COM_OFFS     1
#define _WLIOC_COM(x)       _WLCIOC(_WLIOC_COM_OFFS+x)

/* Commands */

#define WLIOC_SETRADIOFREQ  _WLIOC_COM(0) /* arg: Pointer to uint32_t, */
                                          /* frequency value (in Hz) */
#define WLIOC_GETRADIOFREQ  _WLIOC_COM(1) /* arg: Pointer to uint32_t, */
                                          /* frequency value (in Hz) */

#define WLIOC_SETADDR       _WLIOC_COM(2) /* arg: Pointer to address value, format
                                           * of the address is driver specific */
#define WLIOC_GETADDR       _WLIOC_COM(3) /* arg: Pointer to address value, format
                                           * of the address is driver specific */

#define WLIOC_SETTXPOWER    _WLIOC_COM(4) /* arg: Pointer to int32_t, */
                                          /* Set output power in dBm */
#define WLIOC_GETTXPOWER    _WLIOC_COM(5) /* arg: Pointer to int32_t, */
                                          /* Get output power in dBm */

#define WLIOC_SETFINEPOWER  _WLIOC_COM(6) /* arg: Pointer to int32_t,
                                           * Set power in steps of 0.01 dBm */
#define WLIOC_GETFINEPOWER  _WLIOC_COM(7) /* arg: Pointer to int32_t,
                                           * Get power in steps of 0.01 dBm */

#define WLIOC_SETMODU       _WLIOC_COM(8) /* arg: Pointer to enum wlioc_modulation_e.
                                           * This sets the modulation technology. */
#define WLIOC_GETMODU       _WLIOC_COM(9) /* arg: Pointer to enum wlioc_modulation_e.
                                           * Get current modulation technology. */

/* Number of commands */

#define _WLIOC_COM_COMMANDS 10 /* ! Must be corrected after changes to commands.
                                * ! Equal to the amount of commands above. */

/****************************************************************************
 * LoRa common IOCTL commands
 ****************************************************************************/

/* Offsets */

#define _WLIOC_LORA_OFFS        _WLIOC_COM_OFFS+_WLIOC_COM_COMMANDS
#define _WLIOC_LORA(x)          _WLCIOC(_WLIOC_LORA_OFFS+x)

/* Commands */

#define WLIOC_LORA_SETSF        _WLIOC_LORA(0) /* arg: Pointer to uint8_t */
                                               /* Spreading factor */
#define WLIOC_LORA_GETSF        _WLIOC_LORA(1) /* arg: Pointer to uint8_t */
                                               /* Spreading factor */

#define WLIOC_LORA_SETBW        _WLIOC_LORA(2) /* arg: Pointer to uint32_t */
                                               /* Bandwidth Hz */
#define WLIOC_LORA_GETBW        _WLIOC_LORA(3) /* arg: Pointer to uint32_t */
                                               /* Bandwidth Hz */

#define WLIOC_LORA_SETCR        _WLIOC_LORA(4) /* arg: Pointer to wlioc_lora_cr_e */
                                               /* Coding rate */
#define WLIOC_LORA_GETCR        _WLIOC_LORA(5) /* arg: Pointer to wlioc_lora_cr_e */
                                               /* Coding rate */

#define WLIOC_LORA_SETCRC       _WLIOC_LORA(6) /* arg: Pointer to uint8_t */ 
                                               /* Enable/disable CRC */
#define WLIOC_LORA_GETCRC       _WLIOC_LORA(7) /* arg: Pointer to uint8_t */ 
                                               /* Enabled/disabled CRC */

#define WLIOC_LORA_SETFIXEDHDR  _WLIOC_LORA(8) /* arg: Pointer to uint8_t */
                                               /* Enable/disable length byte */
#define WLIOC_LORA_GETFIXEDHDR  _WLIOC_LORA(9) /* arg: Pointer to uint8_t */
                                               /* Enabled/disabled length byte */

#define WLIOC_LORA_SETSYNCWORD  _WLIOC_LORA(10) /* arg: Pointer to wlioc_lora_syncword_s */
                                                /* Sets custom length syncword */
#define WLIOC_LORA_GETSYNCWORD  _WLIOC_LORA(11) /* arg: Pointer to wlioc_lora_syncword_s */
                                                /* Gets custom length syncword */

/* Number of commands */

#define _WLIOC_LORA_COMMANDS    12 /* ! Must be corrected after changes to commands.
                                    * ! Equal to the amount of commands above. */

/****************************************************************************
 * FSK common IOCTL commands (Including GFSK and similar)
 ****************************************************************************/

/* Offsets. Must follow WLIOC_LORA */

#define _WLIOC_FSK_OFFS         _WLIOC_LORA_OFFS+_WLIOC_LORA_COMMANDS
#define _WLIOC_FSK(x)           _WLCIOC(_WLIOC_FSK_OFFS+x)

/* Commands */

#define WLIOC_FSK_SETBITRATE    _WLIOC_FSK(0) /* arg: uint32_t ptr */
                                              /* In bits per second */
#define WLIOC_FSK_GETBITRATE    _WLIOC_FSK(1) /* arg: uint32_t ptr */
                                              /* In bits per second */

#define WLIOC_FSK_SETFDEV       _WLIOC_FSK(2) /* arg: uint32_t ptr */
                                              /* Frequency deviation Hz */
#define WLIOC_FSK_GETFDEV       _WLIOC_FSK(3) /* arg: uint32_t ptr */
                                              /* Frequency deviation Hz */

/* Number of commands */

#define _WLIOC_FSK_COMMANDS     4   /* ! Must be corrected after changes to commands.
                                     * ! Equal to the amount of commands above. */

/****************************************************************************
 * OOK (Also called "binary" ASK) common IOCTL commands
 ****************************************************************************/

/* Offsets. Must follow WLIOC_FSK */

#define _WLIOC_OOK_OFFS         _WLIOC_FSK_OFFS+_WLIOC_FSK_COMMANDS
#define _WLIOC_OOK(x)           _WLCIOC(_WLIOC_OOK_OFFS+x)

/* Commands */

#define WLIOC_OOK_SETBITRATE    _WLIOC_OOK(0) /* arg: uint32_t ptr */
                                              /* In bits per second */
#define WLIOC_OOK_GETBITRATE    _WLIOC_OOK(1) /* arg: uint32_t ptr */
                                              /* In bits per second */

/* Number of commands */

#define _WLIOC_OOK_COMMANDS     2   /* ! Must be corrected after changes to commands.
                                     * ! Equal to the amount of commands above. */

/****************************************************************************
 * Device-specific IOCTL commands
 ****************************************************************************/

/* WARNING: The following WLIOC command are EXPERIMENTAL and unstable. They
 * may be removed or modified in upcoming changes that introduce a common
 * LoRa API. These commands are currently only used by the RN2XX3 driver.
 */

/* Offsets. Must follow WLIOC_OOK */

#define _WLIOC_RN2XX3_OFFS  _WLIOC_OOK_OFFS+_WLIOC_OOK_COMMANDS
#define _WLIOC_RN2XX3(x)    _WLCIOC(_WLIOC_RN2XX3_OFFS+x)

/* Commands */

#define WLIOC_SETBANDWIDTH  _WLIOC_RN2XX3(0) /* arg: Pointer to uint32_t, */
                                             /* bandwidth in Hz */
#define WLIOC_GETBANDWIDTH  _WLIOC_RN2XX3(1) /* arg: Pointer to uint32_t, */
                                             /* bandwidth in Hz */

#define WLIOC_SETSPREAD     _WLIOC_RN2XX3(2) /* arg: Pointer to uint8_t, */
                                             /* spread factor */
#define WLIOC_GETSPREAD     _WLIOC_RN2XX3(3) /* arg: Pointer to uint8_t, */
                                             /* spread factor */

#define WLIOC_GETSNR        _WLIOC_RN2XX3(4) /* arg: Pointer to int8_t, */
                                             /* signal to noise ratio */

#define WLIOC_SETPRLEN      _WLIOC_RN2XX3(5) /* arg: uint16_t, */
                                             /* preamble length */
#define WLIOC_GETPRLEN      _WLIOC_RN2XX3(6) /* arg: Pointer to uint16_t, */
                                             /* preamble length */

#define WLIOC_SETMOD        _WLIOC_RN2XX3(7) /* arg: enum, */
                                             /* modulation type */
#define WLIOC_GETMOD        _WLIOC_RN2XX3(8) /* arg: enum pointer, */
                                             /* modulation type */

#define WLIOC_RESET         _WLIOC_RN2XX3(9) /* arg: none */

#define WLIOC_SETSYNC       _WLIOC_RN2XX3(10) /* arg: uint64_t pointer */
                                              /* sync word */
#define WLIOC_GETSYNC       _WLIOC_RN2XX3(11) /* arg: uint64_t pointer, */
                                              /* sync word */

#define WLIOC_SETBITRATE    _WLIOC_RN2XX3(12) /* arg: uint32_t */
                                              /* sync word */
#define WLIOC_GETBITRATE    _WLIOC_RN2XX3(13) /* arg: uint32_t pointer, */
                                              /* sync word */

#define WLIOC_IQIEN         _WLIOC_RN2XX3(14) /* arg: bool, enable invert IQ */
#define WLIOC_CRCEN         _WLIOC_RN2XX3(15) /* arg: bool, enable CRC */

#define WLIOC_SETCODERATE   _WLIOC_RN2XX3(16) /* arg: enum, coding rate */
#define WLIOC_GETCODERATE   _WLIOC_RN2XX3(17) /* arg: enum pointer, */
                                              /* coding rate */

#define WLIOC_SETTXPOWERF   _WLIOC_RN2XX3(18) /* arg: Pointer to float, */
                                              /* output power (in dBm) */
#define WLIOC_GETTXPOWERF   _WLIOC_RN2XX3(19) /* arg: Pointer to float, */
                                              /* output power (in dBm) */

/* Number of commands */

#define _WLIOC_RN2XX3_COMMANDS 20 /* ! Must be corrected after changes to commands.
                                   * ! Equal to the amount of commands above. */

/* End of RN2XX3 experimental warning */

/* Offsets */

#define WL_FIRST            0x0001 /* First common command */
#define WL_NCMDS            _WLIOC_RN2XX3_OFFS+_WLIOC_RN2XX3_COMMANDS

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

/* See include/nuttx/wireless/lpwan/sx126x.h */

#define SX126X_FIRST        (SX127X_FIRST + SX127X_NCMDS)
#define SX126X_NCMDS        11

/* See include/nuttx/wireless/gs2200m.h */

#define GS2200M_FIRST       (SX127X_FIRST + SX127X_NCMDS)
#define GS2200M_NCMDS       9

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* wlioc_mod_type_e is the RF modualtion technology to be used. */

enum wlioc_modulation_e
{
  WLIOC_LORA,
  WLIOC_FSK,
  WLIOC_GFSK,
  WLIOC_OOK
};

/* LoRa common types ********************************************************/

/* wlioc_lora_cr_e is the coding rate, which is commonly
 * supported by LoRa devices to correct errors.
 */

enum wlioc_lora_cr_e
{
  WLIOC_LORA_CR_4_5 = 0x01,
  WLIOC_LORA_CR_4_6 = 0x02,
  WLIOC_LORA_CR_4_7 = 0x03,
  WLIOC_LORA_CR_4_8 = 0x04
};

/* wlioc_lora_syncword_s is used to separate lora networks.
 * Radios will try to catch only packets with the specified syncword.
 */

struct wlioc_lora_syncword_s
{
  size_t syncword_length;
  FAR uint8_t *syncword;
};

/* Common RF types **********************************************************/

/* wlioc_rx_hdr_s gets written to by reading the character device.
 * Contains information about the payload.
 */

struct wlioc_rx_hdr_s
{
  /* Length of payload in bytes.
   * The amount written to the
   * payload buffer
   */

  size_t payload_length;

  /* Pointer to user buffer
   * This will be filled in with
   * the payload
   */

  FAR uint8_t *payload_buffer;

  /* When error detection is supported and enabled,
   * this will be greater than 0 when an error is
   * detected. The payload is still returned which
   * allows the user to reconstruct the payload
   */

  uint8_t error;

  /* RSSI dBm in 1/100 dBm */

  int32_t rssi_dbm;

  /* SNR dB in 1/100 dB  */

  int32_t snr_db;
};

#endif /* CONFIG_DRIVERS_WIRELESS */
#endif /* __INCLUDE_NUTTX_WIRELESS_IOCTL_H */
