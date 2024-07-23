/****************************************************************************
 * boards/xtensa/esp32s3/common/include/esp32s3_board_mcpwm.h
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

#ifndef __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32S3_BOARD_MCPWM_H
#define __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32S3_BOARD_MCPWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE

/****************************************************************************
 * Name: board_capture_initialize
 *
 * Description:
 *   Initialize and register the capture driver using the MCPWM peripheral.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_capture_initialize(void);

#endif /* CONFIG_ESP_MCPWM_CAPTURE */
#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32S3_BOARD_MCPWM_H */
