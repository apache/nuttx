/****************************************************************************
 * include/nuttx/contactless/mfrc522.h
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

#ifndef __INCLUDE_NUTTX_CONTACTLESS_MFRC522_H
#define __INCLUDE_NUTTX_CONTACTLESS_MFRC522_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/contactless/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define MFRC522_MIFARE_ISO14443A          (0x00)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mfrc522_state_e
{
  MFRC522_STATE_NOT_INIT,
  MFRC522_STATE_IDLE,
  MFRC522_STATE_CMD_SENT,
  MFRC522_STATE_DATA_READY,
};

struct mfrc522_dev_s;

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
 * Name: mfrc522_register
 *
 * Description:
 *   Register the MFRC522 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rfid0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MFRC522
 *   config  - Device persistent board data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mfrc522_register(FAR const char *devpath, FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CONTACTLESS_MFRC522_H */
