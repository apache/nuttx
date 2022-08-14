/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_chip_data.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CHIP_DATA_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CHIP_DATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

enum
{
  CHIPCOMMON_CORE_ID = 0,
  DOT11MAC_CORE_ID,
  SDIOD_CORE_ID,
#if defined(CONFIG_IEEE80211_BROADCOM_BCM4301X) || \
    defined(CONFIG_IEEE80211_BROADCOM_BCM43362) || \
    defined(CONFIG_IEEE80211_BROADCOM_BCM43438) || \
    defined(CONFIG_IEEE80211_INFINEON_CYW43439)
  WLAN_ARMCM3_CORE_ID,
  SOCSRAM_CORE_ID,
#endif
#if defined(CONFIG_IEEE80211_BROADCOM_BCM43455)
  WLAN_ARMCR4_CORE_ID,
#endif
  MAX_CORE_ID
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SDIO chip configuration structure */

struct bcmf_chip_data
{
  uint32_t ram_base;
  uint32_t ram_size;
  uint32_t core_base[MAX_CORE_ID];

  /* In-memory file images */

  FAR uint8_t      *nvram_image;
  FAR unsigned int *nvram_image_size;

#ifndef CONFIG_IEEE80211_BROADCOM_FWFILES
  FAR uint8_t      *firmware_image;
  FAR unsigned int *firmware_image_size;

#ifdef CONFIG_IEEE80211_BROADCOM_HAVE_CLM
  FAR uint8_t      *clm_blob_image;
  FAR unsigned int *clm_blob_image_size;
#endif
#endif
};

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CHIP_DATA_H */
