/****************************************************************************
 * arch/xtensa/src/esp32s3/chip_macros.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_CHIP_MACROS_H
#define __ARCH_XTENSA_SRC_ESP32S3_CHIP_MACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the name of the section containing the Xtensa low level handlers
 * that is used by the board linker scripts.
 */

#define HANDLER_SECTION .iram1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* Macro to get the current core ID. Only uses the reg given as an argument.
 * Reading PRID on the ESP108 architecture gives us 0xcdcd on the PRO
 * processor and 0xabab on the APP CPU. We distinguish between the two by
 * simply checking bit 1: it's 1 on the APP and 0 on the PRO processor.
 */

    .macro      getcoreid reg
    rsr.prid    \reg
    bbci        \reg, 1, 1f
    movi        \reg, 1
    j           2f
1:
    movi        \reg, 0
2:
    .endm

#endif /* __ASSEMBLY */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_CHIP_MACROS_H */
