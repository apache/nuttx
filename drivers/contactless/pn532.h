/****************************************************************************
 * drivers/contactless/pn532.h
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

#ifndef __DRIVERS_CONTACTLESS_PN532_H
#define __DRIVERS_CONTACTLESS_PN532_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/contactless/pn532.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define PN532_PREAMBLE                      0x00
#define PN532_STARTCODE1                    0x00
#define PN532_STARTCODE2                    0xFF
#define PN532_POSTAMBLE                     0x00

#define PN532_SOF                           0xFF00

#define PN532_HOSTTOPN532                   0xD4
#define PN532_PN532TOHOST                   0xD5

#define PN532_SPI_STATREAD                  0x02
#define PN532_SPI_DATAWRITE                 0x01
#define PN532_SPI_DATAREAD                  0x03
#define PN532_SPI_READY                     0x01

/* PN532 Commands */

#define PN532_COMMAND_DIAGNOSE              0x00
#define PN532_COMMAND_GETFIRMWAREVERSION    0x02
#define PN532_COMMAND_GETGENERALSTATUS      0x04
#define PN532_COMMAND_READREGISTER          0x06
#define PN532_COMMAND_WRITEREGISTER         0x08
#define PN532_COMMAND_READGPIO              0x0C
#define PN532_COMMAND_WRITEGPIO             0x0E
#define PN532_COMMAND_SETSERIALBAUDRATE     0x10
#define PN532_COMMAND_SETPARAMETERS         0x12
#define PN532_COMMAND_SAMCONFIGURATION      0x14
#define PN532_COMMAND_POWERDOWN             0x16
#define PN532_COMMAND_RFCONFIGURATION       0x32
#define PN532_COMMAND_RFREGULATIONTEST      0x58
#define PN532_COMMAND_INJUMPFORDEP          0x56
#define PN532_COMMAND_INJUMPFORPSL          0x46
#define PN532_COMMAND_INLISTPASSIVETARGET   0x4A
#define PN532_COMMAND_INATR                 0x50
#define PN532_COMMAND_INPSL                 0x4E
#define PN532_COMMAND_INDATAEXCHANGE        0x40
#define PN532_COMMAND_INCOMMUNICATETHRU     0x42
#define PN532_COMMAND_INDESELECT            0x44
#define PN532_COMMAND_INRELEASE             0x52
#define PN532_COMMAND_INSELECT              0x54
#define PN532_COMMAND_INAUTOPOLL            0x60
#define PN532_COMMAND_TGINITASTARGET        0x8C
#define PN532_COMMAND_TGSETGENERALBYTES     0x92
#define PN532_COMMAND_TGGETDATA             0x86
#define PN532_COMMAND_TGSETDATA             0x8E
#define PN532_COMMAND_TGSETMETADATA         0x94
#define PN532_COMMAND_TGGETINITIATORCOMMAND 0x88
#define PN532_COMMAND_TGRESPONSETOINITIATOR 0x90
#define PN532_COMMAND_TGGETTARGETSTATUS     0x8A

#define PN532_WAKEUP                        0x55

#define PN532_SAM_NORMAL_MODE               0x01
#define PN532_SAM_VIRTUAL_CARD              0x02
#define PN532_SAM_WIRED_CARD                0x03
#define PN532_SAM_DUAL_CARD                 0x04

#ifndef CONFIG_PN532_SPI_FREQ
#  define CONFIG_PN532_SPI_FREQ             (5000000)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct pn532_frame
{
  uint8_t  preamble;    /* 0x00 */
  uint16_t start_code;  /* 0x00FF (BE) -> 0xFF00 (LE) */
  uint8_t  len;         /* 1 byte indicating the number of bytes in
                         * the data field */
  uint8_t  lcs;         /* 1 Packet Length Checksum LCS byte that satisfies
                         * the relation:  Lower byte of [LEN + LCS] = 00h */
  uint8_t  tfi;         /* Frame identifier 0xD4, 0xD5 */
  uint8_t  data[];      /* LEN-1 bytes of Packet Data Information.
                         * The first byte PD0 is the Command Code */
} end_packed_struct;

begin_packed_struct struct pn_poll_response
{
  uint8_t nbtg;
  uint8_t tg;
  uint8_t target_data[];
} end_packed_struct;

begin_packed_struct struct pn_target_type_a
{
  uint16_t sens_res;
  uint8_t  sel_res;
  uint8_t  nfcid_len;
  uint8_t  nfcid_data[];
} end_packed_struct;

struct pn_firmware_version
{
  uint8_t ic;
  uint8_t ver;
  uint8_t rev;
  uint8_t support;
};

struct pn532_dev_s
{
  uint8_t state;
  FAR struct spi_dev_s *spi;          /* SPI interface */
  FAR struct pn532_config_s *config;  /* Board configuration data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool pn532_set_config(struct pn532_dev_s *dev, uint8_t flags);

#endif /* __DRIVERS_CONTACTLESS_PN532_H */
