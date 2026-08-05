/****************************************************************************
 * drivers/wireless/lpwan/sx1301/sx1301_priv.h
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

#ifndef __DRIVERS_WIRELESS_LPWAN_SX1301_SX1301_PRIV_H
#define __DRIVERS_WIRELESS_LPWAN_SX1301_SX1301_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/lpwan/lora_gw.h>
#include <nuttx/wireless/lpwan/sx1301.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register pages.  The page is selected by the low bits of address 0x00 and
 * a few registers (the FIFO and PROM ports) are page independent; those are
 * accessed with SX1301_PAGE_ANY, which does not switch the page at all.
 */

#define SX1301_PAGE_0            0
#define SX1301_PAGE_1            1
#define SX1301_PAGE_2            2
#define SX1301_PAGE_3            3
#define SX1301_PAGE_ANY          0xff

/* SPI protocol of the SX1301: two byte transactions, MSB of the first byte
 * selects a write.  There is no FPGA/mux on this class of shield, so the
 * radios are reached through the internal bridge on page 2 instead.
 */

#define SX1301_SPI_WRITE_FLAG    0x80
#define SX1301_SPI_ADDR_MASK     0x7f

/* Register map -- page independent */

#define SX1301_REG_PAGE          0x00 /* Bits 1:0 page, bit 7 soft reset */
#define SX1301_REG_VERSION       0x01
#define SX1301_REG_RX_BUF_ADDR   0x02 /* 16 bit read pointer (0x02..0x03) */
#define SX1301_REG_RX_BUF_DATA   0x04 /* Auto incrementing data port */
#define SX1301_REG_TX_BUF_ADDR   0x05
#define SX1301_REG_TX_BUF_DATA   0x06 /* Write port; reads give the counter */
#define SX1301_REG_TIMESTAMP     0x06 /* 32 bit concentrator counter (RO) */
#define SX1301_REG_PROM_ADDR     0x09
#define SX1301_REG_PROM_DATA     0x0a
#define SX1301_REG_RX_NUM_STORED 0x0b /* Packets pending; write 0 to pop */
#define SX1301_REG_RX_ADDR_PTR   0x0c /* 16 bit (0x0c..0x0d), RO */
#define SX1301_REG_RX_STATUS     0x0e /* CRC status of the pending packet */
#define SX1301_REG_RX_PLD_SIZE   0x0f
#define SX1301_REG_GLOBAL_EN     0x10 /* b0 MBWSSF, b1 CONC, b2 FSK, b3 EN */
#define SX1301_REG_CLK_CTRL      0x11 /* b0 CLK32M, b1 CLKHS */
#define SX1301_REG_AGC_STATUS    0x20
#define SX1301_REG_CHIP_ID       0x7e
#define SX1301_REG_EMERGENCY     0x7f /* b0 force host control */

/* Register map -- page 0 */

#define SX1301_REG_GPIO_SELECT   0x1c
#define SX1301_REG_GPIO_MODE     0x1d
#define SX1301_REG_RADIO_SELECT  0x23
#define SX1301_REG_IF_FREQ_BASE  0x24 /* IF0..IF7, two bytes each */
#define SX1301_REG_IF_FREQ_8     0x34
#define SX1301_REG_IF_FREQ_9     0x36
#define SX1301_REG_CORR_EN_BASE  0x41 /* CORR0..CORR7 detect enable */
#define SX1301_REG_CORR_TUNE     0x4e /* b3:0 same peak, b6:4 mac gain */
#define SX1301_REG_FRAME_SYNCH   0x5f /* b3:0 peak1, b7:4 peak2 */
#define SX1301_REG_FREQ_DRIFT    0x5d
#define SX1301_REG_PPM_OFFSET    0x64
#define SX1301_REG_GAIN_OFFSET   0x68 /* b3:0 dec, b7:4 chan */
#define SX1301_REG_FORCE_CTRL    0x69 /* b1 radio, b2 front end, b3 filter */
#define SX1301_REG_MCU_CTRL      0x6a /* b0/b1 reset, b2/b3 PROM mux */
#define SX1301_REG_RSSI_BB_DFLT  0x6c
#define SX1301_REG_RSSI_DEC_DFLT 0x6d
#define SX1301_REG_RSSI_CHAN_DFT 0x6e
#define SX1301_REG_RSSI_BB_ALPHA 0x6f
#define SX1301_REG_RSSI_DEC_ALPH 0x70
#define SX1301_REG_RSSI_CHN_ALPH 0x71

/* Register map -- page 1 (transmitter and LoRa standard demodulator) */

#define SX1301_REG_TX_TRIG       0x21 /* b0 immediate, b1 timestamped */
#define SX1301_REG_TX_START_DLY  0x22 /* 16 bit (0x22..0x23) */
#define SX1301_REG_TX_FRAME_SYNC 0x24
#define SX1301_REG_TX_OFFSET_I   0x27
#define SX1301_REG_TX_OFFSET_Q   0x28
#define SX1301_REG_TX_GAIN       0x2a /* b1:0 digital gain, b7 swap IQ */
#define SX1301_REG_MBWSSF_SYNCH  0x2e /* b3:0 peak1, b7:4 peak2 */
#define SX1301_REG_MBWSSF_DRIFT  0x35
#define SX1301_REG_MBWSSF_BW_SEL 0x3a /* b1:0 bandwidth, b2 radio select */
#define SX1301_REG_MBWSSF_PPM    0x3b
#define SX1301_REG_MBWSSF_SF     0x3c
#define SX1301_REG_TX_STATUS     0x3e

/* Register map -- page 2 (radio bridge and MCU RAM debug ports) */

#define SX1301_REG_RADIO_A_DATA  0x21
#define SX1301_REG_RADIO_A_RB    0x22
#define SX1301_REG_RADIO_A_ADDR  0x23
#define SX1301_REG_RADIO_A_CS    0x25
#define SX1301_REG_RADIO_B_DATA  0x26
#define SX1301_REG_RADIO_B_RB    0x27
#define SX1301_REG_RADIO_B_ADDR  0x28
#define SX1301_REG_RADIO_B_CS    0x2a
#define SX1301_REG_RADIO_CFG     0x2b /* b0 A enable, b1 B enable, b2 reset */
#define SX1301_REG_DBG_ARB_DATA  0x40
#define SX1301_REG_DBG_AGC_DATA  0x41
#define SX1301_REG_DBG_ARB_ADDR  0x50
#define SX1301_REG_DBG_AGC_ADDR  0x51

/* Expected identification values */

#define SX1301_CHIP_VERSION      0x67
#define SX1301_CHIP_ID_VALUE     0x01

/* Sixteen bytes of metadata follow the payload in the RX data buffer */

#define SX1301_RX_METADATA_NB    16

/* Largest burst that can be issued with a zero filled MOSI buffer.  Big
 * enough for a full payload plus its metadata.
 */

#define SX1301_ZEROBUF_SIZE      (LORA_GW_MAX_PAYLOAD + SX1301_RX_METADATA_NB)

/* Convert an intermediate frequency in Hz into the 13 bit signed register
 * value: value = (if_hz << 5) / 15625.
 */

#define SX1301_IF_HZ_TO_REG(f)   ((int32_t)(((int64_t)(f) << 5) / 15625))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Radio front-end configuration, derived from the active region */

struct sx1301_rfconf_s
{
  bool     enable;
  bool     tx_enable;
  uint32_t freq_hz;
  int16_t  rssi_offset_dbm10; /* RSSI offset in 0.1 dBm units */
};

/* Multi-SF demodulator configuration (IF0..IF7) */

struct sx1301_ifconf_s
{
  bool    enable;
  uint8_t rf_chain;
  int32_t freq_hz;            /* Offset from the radio centre frequency */
};

/* LoRa standard demodulator configuration (IF8) */

struct sx1301_stdconf_s
{
  bool    enable;
  uint8_t rf_chain;
  uint8_t bandwidth;
  uint8_t datarate;
  int32_t freq_hz;
};

/* Driver state */

struct sx1301_dev_s
{
  FAR struct spi_dev_s *spi;
  FAR const struct sx1301_lower_s *lower;

  mutex_t  lock;              /* Exclusive access to the chip and state */
  uint8_t  page;              /* Currently selected register page */
  uint8_t  crefs;             /* Number of open references */
  bool     connected;         /* SPI probed and chip identified */
  bool     started;           /* Concentrator running */
  int      region;            /* Active region index */
  uint8_t  radio_select;      /* Radio mapping bitmask of IF0..IF7 */

  struct lora_gw_status_s  status;
  struct sx1301_rfconf_s  rf[LORA_GW_RF_CHAIN_NB];
  struct sx1301_ifconf_s  ifc[LORA_GW_MULTI_NB];
  struct sx1301_stdconf_s std;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Register access, sx1301_reg.c ********************************************/

int sx1301_reg_probe(FAR struct sx1301_dev_s *priv);
int sx1301_reg_write(FAR struct sx1301_dev_s *priv, uint8_t page,
                     uint8_t addr, uint8_t value);
int sx1301_reg_read(FAR struct sx1301_dev_s *priv, uint8_t page,
                    uint8_t addr, FAR uint8_t *value);
int sx1301_reg_wrburst(FAR struct sx1301_dev_s *priv, uint8_t page,
                       uint8_t addr, FAR const uint8_t *buffer,
                       size_t buflen);
int sx1301_reg_rdburst(FAR struct sx1301_dev_s *priv, uint8_t page,
                       uint8_t addr, FAR uint8_t *buffer, size_t buflen);
int sx1301_reg_setbit(FAR struct sx1301_dev_s *priv, uint8_t page,
                      uint8_t addr, uint8_t bit, bool value);
int sx1301_reg_write13s(FAR struct sx1301_dev_s *priv, uint8_t page,
                        uint8_t addr, int32_t value);

/* Region handling, sx1301_region.c *****************************************/

int sx1301_region_count(void);
int sx1301_region_byname(FAR const char *name);
int sx1301_region_getinfo(int region,
                          FAR struct lora_gw_regioninfo_s *info);
int sx1301_region_apply(FAR struct sx1301_dev_s *priv, int region);

#endif /* __DRIVERS_WIRELESS_LPWAN_SX1301_SX1301_PRIV_H */
