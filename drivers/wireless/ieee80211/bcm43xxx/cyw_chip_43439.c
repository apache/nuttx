/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/cyw_chip_43439.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "bcmf_interface.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WRAPPER_REGISTER_OFFSET  0x100000

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uint8_t       g_cyw43439_nvram_image[];
extern unsigned int        g_cyw43439_nvram_len;

#ifndef CONFIG_IEEE80211_BROADCOM_FWFILES

extern const uint8_t       g_cyw43439_firmware_image[];
extern const unsigned int  g_cyw43439_firmware_len;

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM

extern const uint8_t       g_cyw43439_clm_blob_image[];
extern const unsigned int  g_cyw43439_clm_blob_len;

#endif
#endif

const struct bcmf_chip_data g_cyw43439_config_data =
{
  /* General chip stats */

  .ram_base = 0,
  .ram_size = 512 * 1024,

  /* Backplane architecture */

  .core_base =
  {
    [CHIPCOMMON_CORE_ID]  = 0x18000000,  /* Chipcommon core register base   */
    [DOT11MAC_CORE_ID]    = 0x18001000,  /* dot11mac core register base     */
    [SDIOD_CORE_ID]       = 0x18002000,  /* SDIOD Device core register base */
    [WLAN_ARMCM3_CORE_ID] = 0x18003000 + /* ARMCM3 core register base       */
                            WRAPPER_REGISTER_OFFSET,
    [SOCSRAM_CORE_ID]     = 0x18004000 + /* SOCSRAM core register base      */
                            WRAPPER_REGISTER_OFFSET
  },

  /* Firmware images */

  .nvram_image         = (FAR uint8_t *)g_cyw43439_nvram_image,
  .nvram_image_size    = (FAR unsigned int *)&g_cyw43439_nvram_len,

#ifndef CONFIG_IEEE80211_BROADCOM_FWFILES
  .firmware_image      = (FAR uint8_t *)g_cyw43439_firmware_image,
  .firmware_image_size = (FAR unsigned int *)&g_cyw43439_firmware_len,

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM
  .clm_blob_image      = (FAR uint8_t *)g_cyw43439_clm_blob_image,
  .clm_blob_image_size = (FAR unsigned int *)&g_cyw43439_clm_blob_len
#endif
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
