/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_qencoder.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_QENCODER_H
#define __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * as a quadrature encoder input device.  If CONFIG_ESP_PCNT_Un is defined
 * then the CONFIG_ESP_PCNT_Un_QE must also be defined to indicate that
 * pcnt "n" is intended to be used for as a quadrature encoder.
 */

#ifndef CONFIG_ESP_PCNT_U0
#  undef CONFIG_ESP_PCNT_U0_QE
#endif
#ifndef CONFIG_ESP_PCNT_U1
#  undef CONFIG_ESP_PCNT_U1_QE
#endif
#ifndef CONFIG_ESP_PCNT_U2
#  undef CONFIG_ESP_PCNT_U2_QE
#endif
#ifndef CONFIG_ESP_PCNT_U3
#  undef CONFIG_ESP_PCNT_U3_QE
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Name: esp_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface. This function must be
 *   called from board-specific logic.
 *
 * Input Parameters:
 *   devpath  - The full path to the driver to register. E.g., "/dev/qe0"
 *   pcnt     - Pointer to the pcnt driver struct
 *   pcnt_num - The PCNT number to used.  'pcnt' must be an element of
 *              {0,1,2,3}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int esp_qeinitialize(const char *devpath, struct cap_lowerhalf_s *pcnt,
                     int pcnt_num);

#endif /* CONFIG_SENSORS_QENCODER */
#endif /* __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_QENCODER_H */
