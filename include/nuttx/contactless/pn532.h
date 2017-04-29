/****************************************************************************
 * include/contactless/pn532.h
 *
 *   Copyright(C) 2012, 2013, 2016 Offcode Ltd. All rights reserved.
 *   Authors: Janne Rosberg <janne@offcode.fi>
 *            Teemu Pirinen <teemu@offcode.fi>
 *            Juho Grundström <juho@offcode.fi>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  int (*irqattach)(void* dev, xcpt_t isr);
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

  PN_RF_CONFIG_ITEM_ANALOG_106A = 0x0A,
  PN_RF_CONFIG_ITEM_ANALOG_212  = 0x0B,
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
 * Public Functions
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
 *   spi     - An instance of the SPI interface to use to communicate with PN532
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
