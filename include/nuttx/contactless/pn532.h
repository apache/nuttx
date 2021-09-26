/****************************************************************************
 * include/nuttx/contactless/pn532.h
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

#ifndef __INCLUDE_NUTTX_CONTACTLESS_PN532_H
#define __INCLUDE_NUTTX_CONTACTLESS_PN532_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <sys/ioctl.h>

#include <nuttx/contactless/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define PN532_MIFARE_ISO14443A          (0x00)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum pn532_state_e
{
  PN532_STATE_NOT_INIT,
  PN532_STATE_IDLE,
  PN532_STATE_CMD_SENT,
  PN532_STATE_DATA_READY,
};

struct pn532_dev_s;
struct pn532_config_s
{
  int (*reset)(uint8_t enable);

  /* External CS, if NULL then SPIDEV_WIRELESS(n) CS is used */

  int (*select)(struct pn532_dev_s *dev, bool sel);
  int (*irqattach)(void *dev, xcpt_t isr);
};

enum PN_SAM_MODE
{
  PN_SAM_NORMAL_MODE = 0x01,
  PN_SAM_VIRTUAL_CARD,
  PN_SAM_WIRED_CARD,
  SAM_DUAL_CARD
};

struct pn_sam_settings_s
{
  enum PN_SAM_MODE mode;  /* Mode */
  uint8_t timeout;        /* Timeout: LSB=50ms 0x14*50ms = 1sec */
  uint8_t irq_en;         /* If 1 - enable P-70, IRQ */
};

enum PN_RF_CONFIG_ITEM
{
  PN_RF_CONFIG_RF_FIELD         = 0x01,
  PN_RF_CONFIG_VARIOUS_TIMINGS  = 0x02,
  PN_RF_CONFIG_ITEM_ANALOG_106A = 0x0a,
  PN_RF_CONFIG_ITEM_ANALOG_212  = 0x0b,
};

struct pn_rf_config_s
{
  uint8_t cfg_item;       /* Item */
  uint8_t data_size;      /* number of config items */
  uint8_t config[11];     /* Item config data */
};

struct pn_mifare_tag_data_s
{
  uint32_t data;
  uint8_t address;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pn532_register
 *
 * Description:
 *   Register the PN532 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/nfc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             PN532
 *   config  - Device persistent board data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pn532_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                   FAR struct pn532_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CONTACTLESS_PN532_H */
