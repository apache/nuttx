/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_partition.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PARTITION_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PARTITION_H

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_partition_init
 *
 * Description:
 *   Initialize ESP32-S3 partition. Read partition information
 *   and use these data for creating MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_partition_init(void);

/****************************************************************************
 * Name: esp32s3_partition_read
 *
 * Description:
 *   Read data from SPI Flash at designated address.
 *
 * Input Parameters:
 *   label  - Partition label
 *   offset - Offset in SPI Flash
 *   buf    - Data buffer pointer
 *   size   - Data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_partition_read(const char *label, size_t offset, void *buf,
                           size_t size);

/****************************************************************************
 * Name: esp32s3_partition_write
 *
 * Description:
 *   Write data to SPI Flash at designated address.
 *
 * Input Parameters:
 *   label  - Partition label
 *   offset - Offset in SPI Flash
 *   buf    - Data buffer pointer
 *   size   - Data number
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_partition_write(const char *label, size_t offset, void *buf,
                            size_t size);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_PARTITION_H */
