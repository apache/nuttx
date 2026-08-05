/****************************************************************************
 * include/nuttx/wireless/lpwan/lora_gw.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_LORA_GW_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_LORA_GW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device independent interface of a LoRa gateway, that is, of a concentrator
 * demodulating several channels at once instead of one channel at a time as
 * an end device does.  A driver of this class registers a character device
 * where read() returns whole struct lora_gw_rxpkt_s records, write() takes
 * one struct lora_gw_txpkt_s, and the channel plan and the state of the chip
 * are reached with the WLIOC_GW_* commands of nuttx/wireless/ioctl.h.
 *
 * The layout below follows the userspace HAL that Semtech publishes for this
 * family of chips, which is what gateway software is written against on
 * other systems, so that such an application ports by replacing its
 * lgw_receive() with read() and its lgw_send() with write().  Two things
 * deliberately differ: the signal levels are integers scaled by ten instead
 * of floats, and the spreading factor is the plain number, not a bit mask.
 *
 * Whether the units should instead follow the ones of the end device
 * commands of nuttx/wireless/ioctl.h, that is, bandwidth in Hz and levels
 * scaled by a hundred, is a question for the common LoRa API rather than for
 * one driver, and is left as it is until that API materialises.
 */

#define LORA_GW_MULTI_NB       8   /* Multi-SF IF chains (IF0..IF7) */
#define LORA_GW_IF_CHAIN_NB    10  /* 8 multi-SF + LoRa standard + FSK */
#define LORA_GW_RF_CHAIN_NB    2   /* Radio A and radio B */
#define LORA_GW_MAX_PAYLOAD    256 /* Maximum PHY payload */
#define LORA_GW_REGION_NAMELEN 12
#define LORA_GW_REGION_DESCLEN 48

/* Bandwidth codes */

#define LORA_GW_BW_UNDEFINED   0x00
#define LORA_GW_BW_125K        0x04
#define LORA_GW_BW_250K        0x05
#define LORA_GW_BW_500K        0x06

/* Modulation codes */

#define LORA_GW_MOD_LORA       0x10
#define LORA_GW_MOD_FSK        0x20

/* Packet status.  A concentrator distinguishes a packet whose CRC was
 * checked and passed from one that failed and from one that carried no CRC
 * at all.  A gateway must never forward LORA_GW_STAT_CRC_BAD as if it were
 * valid: those are mostly correlator false triggers.
 */

#define LORA_GW_STAT_UNDEFINED 0x00
#define LORA_GW_STAT_NO_CRC    0x01
#define LORA_GW_STAT_CRC_OK    0x10
#define LORA_GW_STAT_CRC_BAD   0x11

/* TX modes */

#define LORA_GW_TX_IMMEDIATE   0
#define LORA_GW_TX_TIMESTAMPED 1

/* Channel types reported by WLIOC_GW_GETREGION */

#define LORA_GW_CHAN_OFF       0
#define LORA_GW_CHAN_MULTI_SF  1  /* One of IF0..IF7, SF7..SF12 */
#define LORA_GW_CHAN_STD       2  /* IF8, single SF/BW (LoRa standard) */
#define LORA_GW_CHAN_FSK       3  /* IF9 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A packet received by the concentrator.  read() returns whole multiples of
 * this structure, newest last.
 *
 * RSSI and SNR are scaled integers (tenths of a dBm/dB) so that no floating
 * point is needed in the driver.
 */

struct lora_gw_rxpkt_s
{
  uint32_t freq_hz;     /* Absolute frequency of the receiving channel */
  uint32_t count_us;    /* Concentrator timestamp, corrected as per the
                         * reference HAL */
  int16_t  rssi_dbm10;  /* RSSI in 0.1 dBm units */
  int16_t  snr_db10;    /* Signal to noise ratio in 0.1 dB units */
  uint16_t size;        /* Payload size in bytes */
  uint8_t  if_chain;    /* Demodulator index 0..9 */
  uint8_t  rf_chain;    /* Radio 0 (A) or 1 (B) */
  uint8_t  status;      /* One of LORA_GW_STAT_* */
  uint8_t  modulation;  /* LORA_GW_MOD_* */
  uint8_t  bandwidth;   /* LORA_GW_BW_* */
  uint8_t  datarate;    /* Spreading factor, 7..12 */
  uint8_t  coderate;    /* enum wlioc_lora_cr_e */
  uint8_t  payload[LORA_GW_MAX_PAYLOAD];
};

/* A packet to transmit.  write() takes exactly one of these. */

struct lora_gw_txpkt_s
{
  uint32_t freq_hz;
  uint32_t count_us;    /* On-air time for LORA_GW_TX_TIMESTAMPED */
  uint16_t size;
  uint16_t preamble;    /* 0 selects the LoRaWAN default of 8 symbols */
  uint8_t  tx_mode;     /* LORA_GW_TX_IMMEDIATE or _TIMESTAMPED */
  int8_t   rf_power;    /* Requested antenna power in dBm */
  uint8_t  rf_chain;
  uint8_t  modulation;
  uint8_t  bandwidth;
  uint8_t  datarate;
  uint8_t  coderate;    /* enum wlioc_lora_cr_e */
  bool     invert_pol;  /* True for LoRaWAN downlinks */
  bool     no_crc;
  bool     no_header;
  uint8_t  payload[LORA_GW_MAX_PAYLOAD];
};

/* Concentrator counters and state */

struct lora_gw_status_s
{
  bool     started;
  uint32_t rx_ok;       /* Packets with a valid CRC */
  uint32_t rx_bad;      /* Packets dropped because the CRC failed */
  uint32_t rx_nocrc;    /* Packets received without CRC */
  uint32_t rx_err;      /* FIFO read errors */
  uint32_t tx_ok;
  uint32_t tx_err;
};

/* One entry of the channel plan of a region */

struct lora_gw_chaninfo_s
{
  uint32_t freq_hz;
  uint8_t  rf_chain;
  uint8_t  type;        /* LORA_GW_CHAN_* */
  uint8_t  bandwidth;   /* LORA_GW_BW_* */
  uint8_t  datarate;    /* SF for LORA_GW_CHAN_STD, 0 for multi-SF */
  bool     enable;
};

/* A complete region description */

struct lora_gw_regioninfo_s
{
  char     name[LORA_GW_REGION_NAMELEN];
  char     desc[LORA_GW_REGION_DESCLEN];
  uint32_t radio_freq[LORA_GW_RF_CHAIN_NB];
  struct lora_gw_chaninfo_s channels[LORA_GW_IF_CHAIN_NB];
};

/* Argument of WLIOC_GW_GETREGION */

struct lora_gw_regionreq_s
{
  int index;                            /* -1: active region, else 0..n */
  struct lora_gw_regioninfo_s info;     /* Returned description */
};

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_LORA_GW_H */
