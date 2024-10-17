/****************************************************************************
 * include/nuttx/wireless/lpwan/rn2483.h
 *
 * Contributed by Carleton University InSpace
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_RN2483_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_RN2483_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/ioctl.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RN2483 modulation modes */

enum rn2483_mod_e
{
  RN2483_MOD_FSK = 0,  /* FSK modulation */
  RN2483_MOD_LORA = 1, /* LoRa modulation */
};

/* RN2483 coding rates */

enum rn2483_cr_e
{
  RN2483_CR_4_5 = 5, /* 4/5 coding rate */
  RN2483_CR_4_6 = 6, /* 4/6 coding rate */
  RN2483_CR_4_7 = 7, /* 4/7 coding rate */
  RN2483_CR_4_8 = 8, /* 4/8 coding rate */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int rn2483_register(FAR const char *devpath, FAR const char *uartpath);

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_RN2483_H */
