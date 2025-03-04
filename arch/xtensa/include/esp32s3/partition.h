/****************************************************************************
 * arch/xtensa/include/esp32s3/partition.h
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

#ifndef __ARCH_XTENSA_INCLUDE_ESP32S3_PARTITION_H
#define __ARCH_XTENSA_INCLUDE_ESP32S3_PARTITION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* OTA image operation code */

enum ota_img_ctrl_e
{
  OTA_IMG_GET_BOOT        = 0xe1,
  OTA_IMG_SET_BOOT        = 0xe2,
  OTA_IMG_INVALIDATE_BOOT = 0xe3,
  OTA_IMG_IS_MAPPED_AS_TEXT = 0xe4,
};

/* OTA image boot sequency */

enum ota_img_bootseq_e
{
  OTA_IMG_BOOT_FACTORY    = 0,
  OTA_IMG_BOOT_OTA_0      = 1,
  OTA_IMG_BOOT_OTA_1      = 2,
  OTA_IMG_BOOT_SEQ_MAX
};

#endif /* __ARCH_XTENSA_INCLUDE_ESP32S3_PARTITION_H */
