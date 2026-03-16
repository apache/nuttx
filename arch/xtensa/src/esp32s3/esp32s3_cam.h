/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_cam.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CAM_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/video/imgdata.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_cam_initialize
 *
 * Description:
 *   Initialize the ESP32-S3 CAM imgdata driver.
 *   Starts XCLK output for sensor communication.
 *
 * Returned Value:
 *   Pointer to imgdata_s on success; NULL on failure.
 *
 ****************************************************************************/

struct imgdata_s *esp32s3_cam_initialize(void);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_CAM_H */
