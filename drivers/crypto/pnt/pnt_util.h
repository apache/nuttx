/****************************************************************************
 * drivers/crypto/pnt/pnt_util.h
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

/* Copyright 2023 NXP */

#ifndef __INCLUDE_NUTTX_CRYPTO_PNT_PNT_UTIL_H_
#define __INCLUDE_NUTTX_CRYPTO_PNT_PNT_UTIL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "se05x_types.h"
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SE050_MODULE_UNIQUE_ID_LEN 18
#define KSE05X_APPLETRESID_UNIQUE_ID 0x7fff0206
#define CLA_ISO7816 (0x00)     /* ISO7816-4 defined CLA byte */
#define CLA_GP_7816 (0x80)     /* GP 7816-4 defined CLA byte */
#define INS_GP_SELECT (0xa4)   /* Global platform defined instruction */
#define INS_GP_GET_DATA (0xcA) /* Global platform defined instruction */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  uint8_t tag_value_proprietary_data;
  uint8_t length_of_following_data;
  uint8_t tag_card_identification_data[0x02];
  uint8_t length_of_card_identification_data;
  uint8_t tag_configuration_id;
  uint8_t length_configuration_id;
  uint8_t configuration_id[0x0c];
  uint8_t tag_patch_id;
  uint8_t length_patch_id;
  uint8_t patch_id[0x08];
  uint8_t tag_platform_build_id1;
  uint8_t length_platform_build_id;
  uint8_t platform_build_id[0x18];
  uint8_t tag_fips_mode;
  uint8_t length_fips_mode;
  uint8_t fips_mode;
  uint8_t tag_pre_perso_state;
  uint8_t length_pre_perso_state;
  uint8_t bitmask_of_pre_perso_state;
  uint8_t tag_rom_id;
  uint8_t length_rom_id;
  uint8_t rom_id[0x08];
  uint8_t status_word_sw_[0x02];
} identify_rsp_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool select_card_manager(pSe05xSession_t session_ctx);
bool se05x_identify(pSe05xSession_t session_ctx,
        FAR identify_rsp_t *response);

#endif /* __INCLUDE_NUTTX_CRYPTO_PNT_PNT_UTIL_H_ */
