/****************************************************************************
 * include/contactless/ioctl.h
 *
 *   Copyright(C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_CONTACTLESS_IOCTL_H
#define __INCLUDE_NUTTX_CONTACTLESS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* MFRC522 IOCTL Commands ***************************************************/

#define MFRC522IOC_GET_PICC_UID         _CLIOC(0x0001)
#define MFRC522IOC_GET_STATE            _CLIOC(0x0002)

/* PN532 IOCTL Commands *****************************************************/

#define PN532IOC_SET_SAM_CONF           _CLIOC(0x0003)
#define PN532IOC_READ_PASSIVE           _CLIOC(0x0004)
#define PN532IOC_SET_RF_CONF            _CLIOC(0x0005)
#define PN532IOC_SEND_CMD_READ_PASSIVE  _CLIOC(0x0006)
#define PN532IOC_GET_DATA_READY         _CLIOC(0x0007)
#define PN532IOC_GET_TAG_ID             _CLIOC(0x0008)
#define PN532IOC_GET_STATE              _CLIOC(0x0009)
#define PN532IOC_READ_TAG_DATA          _CLIOC(0x000a)
#define PN532IOC_WRITE_TAG_DATA         _CLIOC(0x000b)

/* Contactless common IOCTL Commands ****************************************/

#define CLIOC_READ_MIFARE_DATA          _CLIOC(0x000c)
#define CLIOC_WRITE_MIFARE_DATA         _CLIOC(0x000d)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct picc_uid_s
{
  uint8_t  size;         /* Number of bytes in the UID. 4, 7 or 10 */
  uint8_t  uid_data[10];
  uint8_t  sak;          /* The SAK (Select Acknowledge) return by the PICC */
};

/* Coding of Select Acknowledge (SAK) according to:
 *   http://www.nxp.com/documents/application_note/AN10833.pdf
 */

enum picc_cardid_e
{
  PICC_TYPE_NOT_COMPLETE = 0x04, /* UID not complete */
  PICC_TYPE_ISO_14443_4  = 0x20, /* PICC compliant with ISO/IEC 14443-4 */
  PICC_TYPE_ISO_18092    = 0x40, /* PICC compliant with ISO/IEC 18092 (NFC) */
  PICC_TYPE_MIFARE_MINI  = 0x09,
  PICC_TYPE_MIFARE_1K    = 0x08,
  PICC_TYPE_MIFARE_4K    = 0x18,
  PICC_TYPE_MIFARE_UL    = 0x00,
  PICC_TYPE_MIFARE_PLUS  = 0x11,
  PICC_TYPE_TNP3XXX      = 0x01
};

struct mifare_tag_data_s
{
  uint8_t  data[16];
  uint8_t  address;
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


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CONTACTLESS_IOCTL_H */
