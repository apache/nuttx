/****************************************************************************
 * include/wireless/sx127x.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Mateusz Szafoni <raiden00@railab.me>
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
 * - IMPLICT HEADER for LORA
 */

/* RX FIFO data *************************************************************/

#define SX127X_READ_DATA_HEADER_LEN (sizeof(struct sx127x_read_hdr_s) - SX127X_READ_DATA_MAX)
#define SX127X_READ_DATA_MAX        (CONFIG_LPWAN_SX127X_RXFIFO_DATA_LEN)
#define SX127X_RXFIFO_ITEM_SIZE     (sizeof(struct sx127x_read_hdr_s))

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* RX FIFO data */

struct sx127x_read_hdr_s
{
  uint16_t datalen;
  int8_t   snr;
  int16_t  rssi;
  uint8_t  reserved[3];
  uint8_t  data[SX127X_READ_DATA_MAX];
};

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
  FSKOOK_BANDWIDTH_2p6kHz   = 0x17,
  FSKOOK_BANDWIDTH_3p1kHz   = 0x0f,
  FSKOOK_BANDWIDTH_3p9kHz   = 0x07,
  FSKOOK_BANDWIDTH_5p2kHz   = 0x16,
  FSKOOK_BANDWIDTH_6p3kHz   = 0x0e,
  FSKOOK_BANDWIDTH_7p8kHz   = 0x06,
  FSKOOK_BANDWIDTH_10p4kHz  = 0x15,
  FSKOOK_BANDWIDTH_12p5kHz  = 0x0d,
  FSKOOK_BANDWIDTH_15p6kHz  = 0x05,
  FSKOOK_BANDWIDTH_20p8kHz  = 0x14,
  FSKOOK_BANDWIDTH_25kHz    = 0x0c,
  FSKOOK_BANDWIDTH_31p3kHz  = 0x04,
  FSKOOK_BANDWIDTH_41p7kHz  = 0x13,
  FSKOOK_BANDWIDTH_50kHz    = 0x0b,
  FSKOOK_BANDWIDTH_62p5kHz  = 0x03,
  FSKOOK_BANDWIDTH_83p3kHz  = 0x12,
  FSKOOK_BANDWIDTH_100kHz   = 0x0a,
  FSKOOK_BANDWIDTH_125kHz   = 0x02,
  FSKOOK_BANDWIDTH_166p7kHz = 0x11,
  FSKOOK_BANDWIDTH_200kHz   = 0x09,
  FSKOOK_BANDWIDTH_250kHz   = 0x01,
  /* Other settings reserved */
};

/* LORA bandwidth */

enum sx127x_lora_bw_e
{
  LORA_BANDWIDTH_7p8kHz  = 0,
  LORA_BANDWIDTH_10p4kHz = 1,
  LORA_BANDWIDTH_15p6kHz = 2,
  LORA_BANDWIDTH_20p8kHz = 3,
  LORA_BANDWIDTH_31p2kHz = 4,
  LORA_BANDWIDTH_41p4kHz = 5,
  LORA_BANDWIDTH_62p5kHz = 6,
  LORA_BANDWIDTH_125kHz  = 7,
  LORA_BANDWIDTH_250kHz  = 9
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
 * Public Functions
 ****************************************************************************/

int sx127x_register(FAR struct spi_dev_s *spi,
                    FAR const struct sx127x_lower_s *lower);

#endif /* __INCLUDE_NUTTX_SX127X_H */

