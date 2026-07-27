/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ldo.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_LDO_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_LDO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Description of an on-chip LDO regulator channel. The channel ID and the
 * output voltage are board/schematic dependent and must be provided by the
 * caller. Only ESP32-P4 provides these regulators.
 */

struct esp_ldo_config_t
{
  uint8_t  chan_id;     /* LDO channel ID, e.g. 3 for LDO_VO3 */
  uint16_t voltage_mv;  /* Output voltage, in millivolts */
  void    *handler;     /* Opaque channel handle, set by acquire */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp_ldo_channel_acquire
 *
 * Description:
 *   Acquire an on-chip LDO regulator channel and enable its output at the
 *   requested voltage. The channel is configured as non-adjustable, so the
 *   same channel may be acquired more than once by different consumers.
 *
 * Input Parameters:
 *   config - Channel description. chan_id and voltage_mv are inputs;
 *            handler receives the channel handle on success and must be
 *            preserved for the matching esp_ldo_channel_release() call.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -EIO is returned if the channel
 *   could not be acquired.
 *
 ****************************************************************************/

int esp_ldo_channel_acquire(struct esp_ldo_config_t *config);

/****************************************************************************
 * Name: esp_ldo_channel_release
 *
 * Description:
 *   Release a channel previously acquired with
 *   esp_ldo_channel_acquire(). The regulator output is turned off once the
 *   last consumer of the channel releases it.
 *
 * Input Parameters:
 *   config - The same structure passed to esp_ldo_channel_acquire(). Its
 *            handler field is cleared on success.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -EINVAL if the channel was not
 *   acquired, or -EIO if the release failed.
 *
 ****************************************************************************/

int esp_ldo_channel_release(struct esp_ldo_config_t *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_LDO_H */
