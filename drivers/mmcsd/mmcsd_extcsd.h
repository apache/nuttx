/****************************************************************************
 * drivers/mmcsd/mmcsd_extcsd.h
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

#ifndef __DRIVERS_MMCSD_MMCSD_EXTCSD_H
#define __DRIVERS_MMCSD_MMCSD_EXTCSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Extended CSD register defines the Device properties and selected
 * modes. It is 512 bytes long. The most significant 320 bytes are the
 * Properties segment, that defines the Device capabilities and cannot be
 * modified by the host. The lower 192 bytes are the Modes segment, that
 * defines the configuration the Device is working in. These modes can be
 * changed by the host by means of the SWITCH command.
 * Multi bytes field is interpreted in little endian byte order.
 */

#define MMCSD_PART_SETTING_COMPLETED               0x1
#define MMCSD_PART_SUPPORT_PART_EN                 0x1

#define MMCSD_EXTCSD_GP_SIZE_MULT                  143  /* R/W */
#define MMCSD_EXTCSD_PARTITION_SETTING_COMPLETED   155  /* R/W */
#define MMCSD_EXTCSD_PARTITION_SUPPORT             160  /* RO */
#define MMCSD_EXTCSD_RPMB_SIZE_MULT                168  /* RO */
#define MMCSD_EXTCSD_HC_WP_GRP_SIZE                221  /* RO */
#define MMCSD_EXTCSD_HC_ERASE_GRP_SIZE             224  /* RO */
#define MMCSD_EXTCSD_BOOT_SIZE_MULT                226  /* RO */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_EXTCSD_H */
