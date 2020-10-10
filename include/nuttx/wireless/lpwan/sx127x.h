/****************************************************************************
 * include/wireless/sx127x.c
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

#ifndef __INCLUDE_NUTTX_SX127X_H
#define __INCLUDE_NUTTX_SX127X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Constants to SX127X */

/* PA BOOST threshold power */

#define SX127X_PASELECT_POWER (14)

/* BAND3 (HF) -> 862(779) - 1020(960) MHz
 * BAND2 (LF) -> 410      - 525(480) MHz
 * BAND1 (LF) -> 137      - 175(160) MHz
 */

#define SX127X_HFBAND_THR                 (525000000)

/* IOCTL commands ***********************************************************/

/* arg: Pointer to int */

#define SX127XIOC_RSSIGET       _WLCIOC(SX127X_FIRST+0)

/* arg: Pointer to enum sx127x_modulation_e */

#define SX127XIOC_MODULATIONSET _WLCIOC(SX127X_FIRST+1)
#define SX127XIOC_MODULATIONGET _WLCIOC(SX127X_FIRST+2)

/* arg: Pointer to enum sx127x_opmode_fskook_e */

#define SX127XIOC_OPMODESET     _WLCIOC(SX127X_FIRST+3)
#define SX127XIOC_OPMODEGET     _WLCIOC(SX127X_FIRST+4)

/* arg: Pointer to struct sx127x_chanscan_ioc_s */

#define SX127XIOC_CHANSCAN      _WLCIOC(SX127X_FIRST+5)

/* arg: Pointer to uint32_t */

#define SX127XIOC_PREAMBLESET   _WLCIOC(SX127X_FIRST+6)
#define SX127XIOC_PREAMBLEGET   _WLCIOC(SX127X_FIRST+7)

/* arg: Pointer to struct sx127x_syncword_ioc_s */

#define SX127XIOC_SYNCWORDSET   _WLCIOC(SX127X_FIRST+8)
#define SX127XIOC_SYNCWORDGET   _WLCIOC(SX127X_FIRST+9)

/* arg: Pointer to uint32_t */

#define SX127XIOC_RANDOMGET     _WLCIOC(SX127X_FIRST+10)

/* TODO SX127X IOC:
 * - CRC ON/OFF
 * - FDEV
 * - RX_BW + AFC_BW
 * - BW for LORA
 * - SF for LORA
 * - CR for LORA
 * - FREQ HOPPING
 * - IMPLICIT HEADER for LORA
 */

/* RX FIFO data *************************************************************/

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
#  define SX127X_READ_DATA_HEADER_LEN (sizeof(struct sx127x_read_hdr_s) - \
                                       SX127X_READ_DATA_MAX)
#  define SX127X_READ_DATA_MAX        (CONFIG_LPWAN_SX127X_RXFIFO_DATA_LEN)
#  define SX127X_RXFIFO_ITEM_SIZE     (sizeof(struct sx127x_read_hdr_s))
#endif

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X_RXSUPPORT
/* RX FIFO data */

struct sx127x_read_hdr_s
{
  uint16_t datalen;
  int8_t   snr;
  int16_t  rssi;
  uint8_t  reserved[3];
  uint8_t  data[SX127X_READ_DATA_MAX];
};
#endif

/* Channel scan data */

struct sx127x_chanscan_ioc_s
{
  uint32_t freq;
  int16_t  rssi_thr;
  int16_t  rssi_max;
  int16_t  rssi_min;
  uint16_t stime;
  bool     free;
};

/* SyncWord data */

struct sx127x_syncword_ioc_s
{
  uint8_t syncword[8];
  uint8_t len;
};

/* Modulation types */

enum sx127x_modulation_e
{
  SX127X_MODULATION_INVALID =  0, /* Initial state */
  SX127X_MODULATION_FSK     =  1,
  SX127X_MODULATION_OOK     =  2,
  SX127X_MODULATION_LORA    =  3
};

/* Operating modes */

enum sx127x_opmode_fskook_e
{
  SX127X_OPMODE_INVALID      = 0, /* Initial state */
  SX127X_OPMODE_SLEEP        = 1,
  SX127X_OPMODE_STANDBY      = 2,
  SX127X_OPMODE_FSTX         = 3,
  SX127X_OPMODE_TX           = 4,
  SX127X_OPMODE_FSRX         = 5,
  SX127X_OPMODE_RX           = 6,    /* RXCONT for LORA */
  SX127X_OPMODE_RXSINGLE     = 7,    /* Only LORA */
  SX127X_OPMODE_CAD          = 8,    /* Only LORA */
};

/* FSK bandwidth */

enum sx127x_fskook_bw_e
{
  FSKOOK_BANDWIDTH_2P6KHZ   = 0x17,
  FSKOOK_BANDWIDTH_3P1KHZ   = 0x0f,
  FSKOOK_BANDWIDTH_3P9KHZ   = 0x07,
  FSKOOK_BANDWIDTH_5P2KHZ   = 0x16,
  FSKOOK_BANDWIDTH_6P3KHZ   = 0x0e,
  FSKOOK_BANDWIDTH_7P8KHZ   = 0x06,
  FSKOOK_BANDWIDTH_10P4KHZ  = 0x15,
  FSKOOK_BANDWIDTH_12P5KHZ  = 0x0d,
  FSKOOK_BANDWIDTH_15P6KHZ  = 0x05,
  FSKOOK_BANDWIDTH_20P8KHZ  = 0x14,
  FSKOOK_BANDWIDTH_25KHZ    = 0x0c,
  FSKOOK_BANDWIDTH_31P3KHZ  = 0x04,
  FSKOOK_BANDWIDTH_41P7KHZ  = 0x13,
  FSKOOK_BANDWIDTH_50KHZ    = 0x0b,
  FSKOOK_BANDWIDTH_62P5KHZ  = 0x03,
  FSKOOK_BANDWIDTH_83P3KHZ  = 0x12,
  FSKOOK_BANDWIDTH_100KHZ   = 0x0a,
  FSKOOK_BANDWIDTH_125KHZ   = 0x02,
  FSKOOK_BANDWIDTH_166P7KHZ = 0x11,
  FSKOOK_BANDWIDTH_200KHZ   = 0x09,
  FSKOOK_BANDWIDTH_250KHZ   = 0x01,

  /* Other settings reserved */
};

/* LORA bandwidth */

enum sx127x_lora_bw_e
{
  LORA_BANDWIDTH_7P8KHZ  = 0,
  LORA_BANDWIDTH_10P4KHZ = 1,
  LORA_BANDWIDTH_15P6KHZ = 2,
  LORA_BANDWIDTH_20P8KHZ = 3,
  LORA_BANDWIDTH_31P2KHZ = 4,
  LORA_BANDWIDTH_41P4KHZ = 5,
  LORA_BANDWIDTH_62P5KHZ = 6,
  LORA_BANDWIDTH_125KHZ  = 7,
  LORA_BANDWIDTH_250KHZ  = 9
};

/* LORA SF */

enum sx127x_lora_sf_e
{
  LORA_SF_6  = 6,
  LORA_SF_7  = 7,
  LORA_SF_8  = 8,
  LORA_SF_9  = 9,
  LORA_SF_10 = 10,
  LORA_SF_11 = 11,
  LORA_SF_12 = 12
};

/* LORA coding rate */

enum sx127x_lora_cr_e
{
  LORA_CR_4d5 = 1,
  LORA_CR_4d6 = 2,
  LORA_CR_4d7 = 3,
  LORA_CR_4d8 = 4
};

/* Board-specific operations and configuration */

struct sx127x_lower_s
{
  /* Attach DIO0 interrupt - RXREADY/TXREADY/CADREADY */

  CODE int (*irq0attach)(xcpt_t handler, FAR void *arg);

#ifdef CONFIG_LPWAN_SX127X_DIO1
  /* Not supported: Attach DIO1 interrupt */

  CODE int (*irq1attach)(xcpt_t handler, FAR void *arg);
#endif
#ifdef CONFIG_LPWAN_SX127X_DIO2
  /* Not supported: Attach DIO2 interrupt */

  CODE int (*irq2attach)(xcpt_t handler, FAR void *arg);
#endif
#ifdef CONFIG_LPWAN_SX127X_DIO3
  /* Not supported: Attach DIO3 interrupt */

  CODE int (*irq3attach)(xcpt_t handler, FAR void *arg);
#endif
#ifdef CONFIG_LPWAN_SX127X_DIO4
  /* Not supported: Attach DIO4 interrupt */

  CODE int (*irq4attach)(xcpt_t handler, FAR void *arg);
#endif
#ifdef CONFIG_LPWAN_SX127X_DIO5
  /* Not supported: Attach DIO5 interrupt */

  CODE int (*irq5attach)(xcpt_t handler, FAR void *arg);
#endif

  /* Reset radio module */

  CODE void (*reset)(void);

  /* Change radio operation mode */

  CODE int (*opmode_change)(int opmode);

  /* Change radio frequency */

  CODE int (*freq_select)(uint32_t freq);

  /* Set PA BOOST output */

  CODE int (*pa_select)(bool enable);

  /* Force PA BOOST output */

  bool pa_force;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int sx127x_register(FAR struct spi_dev_s *spi,
                    FAR const struct sx127x_lower_s *lower);

#endif /* __INCLUDE_NUTTX_SX127X_H */
