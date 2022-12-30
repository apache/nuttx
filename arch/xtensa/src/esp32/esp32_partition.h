/****************************************************************************
 * arch/xtensa/src/esp32/esp32_partition.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_PARTITION_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_PARTITION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/mtd/mtd.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Partition APP type and subtype */

#define PARTITION_TYPE_APP                (0x00)
#define PARTITION_SUBTYPE_FACTORY         (0x00)
#define PARTITION_SUBTYPE_OTA_FLAG        (0x10)
#define PARTITION_SUBTYPE_OTA_MASK        (0x0f)
#define PARTITION_SUBTYPE_TEST            (0x20)

/* Partition DATA type and subtype */

#define PARTITION_TYPE_DATA               (0x01)
#define PARTITION_SUBTYPE_DATA_OTA        (0x00)
#define PARTITION_SUBTYPE_DATA_RF         (0x01)
#define PARTITION_SUBTYPE_DATA_WIFI       (0x02)
#define PARTITION_SUBTYPE_DATA_NVS_KEYS   (0x04)
#define PARTITION_SUBTYPE_DATA_EFUSE_EM   (0x05)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_partition_init
 *
 * Description:
 *   Initialize ESP32 partition. Read partition information of esp-idf,
 *   and create MTD by these data
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32_partition_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_PARTITION_H */
