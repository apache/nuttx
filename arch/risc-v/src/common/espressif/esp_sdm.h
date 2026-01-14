/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_sdm.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_SDM_H
#define __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_SDM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/dac.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Invert the output signal flag */

#define INVERT_OUT 1

/* For debug/test, the signal output from the GPIO
 * will be fed to the input path as well
 */

#define IO_LOOPBACK 2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct esp_sdm_chan_config_s
{
    int gpio_num;               /* GPIO number */
    uint32_t sample_rate_hz;    /* Over sample rate in Hz, it determines the frequency of the carrier pulses */
    uint32_t flags;             /* Flags for configuration */
};

/****************************************************************************
 * Public functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_sdm_create_channel
 *
 * Description:
 *   This function initializes a SDM (Sigma Delta Modulator) channel and
 *   register to related group with the provided configuration.
 *   Each group can be used as independent DAC and each channel in a group
 *   can be used as DAC channel.
 *
 * Input Parameters:
 *   config  - A structure containing the configuration settings
 *   dac_dev - Pointer to DAC structure which
 *             is returned from esp_sdminitialize
 *
 * Returned Value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

int esp_sdm_create_channel(struct esp_sdm_chan_config_s config,
                           struct dac_dev_s *dac_dev);

/****************************************************************************
 * Name: esp_sdminitialize
 *
 * Description:
 *   This function initializes SDM (Sigma Delta Modulator) group and a
 *   channel with the provided configuration.
 *
 * Input Parameters:
 *   config - A structure containing the configuration settings
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the SDM device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

struct dac_dev_s *esp_sdminitialize(struct esp_sdm_chan_config_s config);

#endif /* __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_SDM_H */
