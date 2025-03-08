/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_sha.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SHA_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SHA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "soc/soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SHA1_START_REG register
 * Starts the SHA accelerator for SHA1 operation
 */

#define SHA1_START_REG (DR_REG_SHA_BASE + 0x80)

/* SHA256_START_REG register
 * Starts the SHA accelerator for SHA256 operation
 */

#define SHA256_START_REG (DR_REG_SHA_BASE + 0x90)

/* SHA384_START_REG register
 * Starts the SHA accelerator for SHA384 operation
 */

#define SHA384_START_REG (DR_REG_SHA_BASE + 0xA0)

/* SHA512_START_REG register
 * Starts the SHA accelerator for SHA512 operation
 */

#define SHA512_START_REG (DR_REG_SHA_BASE + 0xB0)

/* SHA1_CONTINUE_REG register
 * Continues SHA1 operation
 */

#define SHA1_CONTINUE_REG (DR_REG_SHA_BASE + 0x84)

/* SHA256_CONTINUE_REG register
 * Continues SHA256 operation
 */

#define SHA256_CONTINUE_REG (DR_REG_SHA_BASE + 0x94)

/* SHA384_CONTINUE_REG register
 * Continues SHA384 operation
 */

#define SHA384_CONTINUE_REG (DR_REG_SHA_BASE + 0xA4)

/* SHA512_CONTINUE_REG register
 * Continues SHA512 operation
 */

#define SHA512_CONTINUE_REG (DR_REG_SHA_BASE + 0xB4)

/* SHA1_LOAD_REG register
 * Triggers the final calculation of the hash
 */

#define SHA1_LOAD_REG (DR_REG_SHA_BASE + 0x88)

/* SHA256_LOAD_REG register
 * Triggers the final calculation of the hash
 */

#define SHA256_LOAD_REG (DR_REG_SHA_BASE + 0x98)

/* SHA384_LOAD_REG register
 * Triggers the final calculation of the hash
 */

#define SHA384_LOAD_REG (DR_REG_SHA_BASE + 0xA8)

/* SHA512_LOAD_REG register
 * Triggers the final calculation of the hash
 */

#define SHA512_LOAD_REG (DR_REG_SHA_BASE + 0xB8)

/* SHA1_BUSY_REG register
 * Indicates if Accelerator is busy with SHA1 operation or not
 */

#define SHA1_BUSY_REG (DR_REG_SHA_BASE + 0x8C)

/* SHA256_BUSY_REG register
 * Indicates if Accelerator is busy with SHA256 operation or not
 */

#define SHA256_BUSY_REG (DR_REG_SHA_BASE + 0x9C)

/* SHA384_BUSY_REG register
 * Indicates if Accelerator is busy with SHA384 operation or not
 */

#define SHA384_BUSY_REG (DR_REG_SHA_BASE + 0xAC)

/* SHA512_BUSY_REG register
 * Indicates if Accelerator is busy with SHA512 operation or not
 */

#define SHA512_BUSY_REG (DR_REG_SHA_BASE + 0xBC)

/* SHA_TEXT_0_REG register
 * Input/output text
 */

#define SHA_TEXT_0_REG (DR_REG_SHA_BASE + 0x0)

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SHA_H */
