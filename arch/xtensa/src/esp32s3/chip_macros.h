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

#ifndef __ASSEMBLY__
#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
#include <stdint.h>
#endif
#endif

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

#ifdef __ASSEMBLY__

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
  .global	g_cpu_intstack_top
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 15 */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* Macro to get the current core ID. Only uses the reg given as an argument.
 * Reading PRID on the ESP32 gives us 0xCDCD on the PRO processor (0)
 * and 0xABAB on the APP CPU (1). We can distinguish between the two by
 * checking bit 13: it's 1 on the APP and 0 on the PRO processor.
 */

    .macro getcoreid reg
    rsr.prid \reg
    extui \reg,\reg,13,1
    .endm

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the "top" of the correct interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
    .macro setintstack tmp1 tmp2
    getcoreid \tmp1                   /* tmp1 = Core ID (0 or 1) */
    movi  \tmp2, g_cpu_intstack_top   /* tmp2 = Array of stack pointers */
    addx4 \tmp2, \tmp1, \tmp2         /* tmp2 = tmp2 + (tmp1 << 2) */
    l32i  a1, \tmp2, 0                /* a1   = *tmp2 */
    .endm
#endif
#endif /* __ASSEMBLY__ */

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

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
uintptr_t xtensa_intstack_alloc(void);
uintptr_t xtensa_intstack_top(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_CHIP_MACROS_H */
