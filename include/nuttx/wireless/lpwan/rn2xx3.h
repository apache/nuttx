/****************************************************************************
 * include/nuttx/wireless/lpwan/rn2xx3.h
 *
 * NOTE: EXPERIMENTAL DRIVER
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_RN2XX3_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_RN2XX3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/ioctl.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* Modulation types */

enum rn2xx3_mod_e
{
  RN2XX3_MOD_LORA = 0, /* LoRa modulation */
  RN2XX3_MOD_FSK = 1,  /* FSK modulation */
};

/* Coding rates */

enum rn2xx3_cr_e
{
  RN2XX3_CR_4_5 = 0, /* 4/5 coding rate */
  RN2XX3_CR_4_6 = 1, /* 4/6 coding rate */
  RN2XX3_CR_4_7 = 2, /* 4/7 coding rate */
  RN2XX3_CR_4_8 = 3, /* 4/8 coding rate */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rn2xx3_register
 *
 * Description:
 *   Register the RN2xx3 LoRa transceiver driver.
 *
 * Arguments:
 *    devpath - The device path to use for the driver
 *    uartpath - The path to the UART character driver connected to the
 *               transceiver
 *
 ****************************************************************************/

int rn2xx3_register(FAR const char *devpath, FAR const char *uartpath);

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_RN2XX3_H */
