/****************************************************************************
 * boards/risc-v/esp32h2/common/include/esp_board_pcnt.h
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

#ifndef __BOARDS_RISC_V_ESP32H2_COMMON_INCLUDE_ESP_BOARD_PCNT_H
#define __BOARDS_RISC_V_ESP32H2_COMMON_INCLUDE_ESP_BOARD_PCNT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_pcnt_initialize
 *
 * Description:
 *   Initialize the pulse counter/quadrature encoder driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; errno on failure.
 *
 ****************************************************************************/

int board_pcnt_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_RISC_V_ESP32H2_COMMON_INCLUDE_ESP_BOARD_PCNT_H */

