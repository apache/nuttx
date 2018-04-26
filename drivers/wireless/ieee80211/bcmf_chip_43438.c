/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_chip_43438.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Simon Piriou <spiriou31@gmail.com>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "bcmf_sdio.h"
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WRAPPER_REGISTER_OFFSET  0x100000

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const char ap6212_nvram_image[];
extern const unsigned int ap6212_nvram_image_len;

extern const uint8_t ap6212_firmware_image[];
extern const unsigned int ap6212_firmware_len;

extern const uint8_t ap6212_clm_blob[];
extern const unsigned int ap6212_clm_blob_len;

const struct bcmf_sdio_chip bcmf_43438_config_sdio =
{

  /* General chip stats */

  .ram_size = 512*1024,

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
  /* TODO find something smarter than using image_len references */

  .firmware_image      = (uint8_t *)ap6212_firmware_image,
  .firmware_image_size = (unsigned int *)&ap6212_firmware_len,

  .nvram_image         = (uint8_t *)ap6212_nvram_image,
  .nvram_image_size    = (unsigned int *)&ap6212_nvram_image_len,

  .clm_blob_image      = (uint8_t *)ap6212_clm_blob,
  .clm_blob_image_size = (unsigned int *)&ap6212_clm_blob_len,
};
