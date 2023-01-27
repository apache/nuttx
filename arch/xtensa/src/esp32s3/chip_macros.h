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

#if defined(CONFIG_ESP32S3_WCL) && defined(CONFIG_BUILD_PROTECTED)
#include "hardware/esp32s3_wcl_core.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the name of the section containing the Xtensa low level handlers
 * that is used by the board linker scripts.
 */

#define HANDLER_SECTION .iram1

#if defined(CONFIG_ESP32S3_WCL) && defined(CONFIG_BUILD_PROTECTED)

/* Definitions for the Worlds reserved for Kernel and Userspace */

#define WCL_WORLD_KERNEL                    0   /* Privileged */
#define WCL_WORLD_USER                      1   /* Non-privileged */

/* Macros for privilege handling with the World Controller peripheral */

#define xtensa_saveprivilege(regs,var)      ((var) = (regs)[REG_INT_CTX])
#define xtensa_restoreprivilege(regs,var)   ((regs)[REG_INT_CTX] = (var))

#define xtensa_lowerprivilege(regs) ((regs)[REG_INT_CTX] = WCL_WORLD_USER)
#define xtensa_raiseprivilege(regs) ((regs)[REG_INT_CTX] = WCL_WORLD_KERNEL)

#endif

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

/****************************************************************************
 * Name: clear_wcl_write_buffer
 *
 * Description:
 *   Clear World Controller write buffer upon World 0 entry.
 *
 * Entry Conditions:
 *   tmp    - Temporary register
 *
 ****************************************************************************/

    .macro clear_wcl_write_buffer tmp
  /* Refer to ESP32-S3 Technical Reference Manual, section 16.4.3, for a
   * detailed description of the write buffer clearing process.
   */

    wsr     a2, DEPC
    movi    \tmp, SOC_RTC_DATA_LOW
    movi    a2, 0
    s32i    a2, \tmp, 0
    addi    a2, a2, 1
    s32i    a2, \tmp, 0
    addi    a2, a2, 1
    s32i    a2, \tmp, 0
    addi    a2, a2, 1
    s32i    a2, \tmp, 0
    addi    a2, a2, 1
    s32i    a2, \tmp, 0
    addi    a2, a2, 1
    s32i    a2, \tmp, 0
    addi    a2, a2, 1
    s32i    a2, \tmp, 0
    l32i    \tmp, \tmp, 0
    memw
    rsr     a2, DEPC
    .endm

/****************************************************************************
 * Name: get_prev_world
 *
 * Description:
 *   Retrieve World information from interruptee.
 *
 * Entry Conditions:
 *   level  - Interrupt level
 *   out    - Temporary and output register
 *
 * Exit Conditions:
 *   World number to be returned will be written to "out" register.
 *
 ****************************************************************************/

    .macro get_prev_world level out
    .ifeq (\level - 1) * (\level - 3)
    .ifeq (\level - 1)
    movi    \out, WCL_CORE_0_STATUSTABLE1_REG
    .endif
    .ifeq (\level - 3)
    movi    \out, WCL_CORE_0_STATUSTABLE2_REG
    .endif
    l32i    \out, \out, 0
    extui   \out, \out, 0, 1
    .endif
    .endm

/****************************************************************************
 * Name: set_next_world
 *
 * Description:
 *   Configure the World Controller for the new execution context.
 *
 * Entry Conditions:
 *   reg_sp - Stack pointer
 *   tmp1   - Temporary register 1
 *   tmp2   - Temporary register 2
 *
 ****************************************************************************/

    .macro set_next_world reg_sp tmp1 tmp2
    movi    \tmp1, BIT(1)
    movi    \tmp2, WCL_CORE_0_WORLD_PREPARE_REG
    s32i    \tmp1, \tmp2, 0       /* Prepare execution on World 1 */

    l32i    \tmp1, \reg_sp, (4 * REG_PC)
    movi    \tmp2, WCL_CORE_0_WORLD_TRIGGER_ADDR_REG
    s32i    \tmp1, \tmp2, 0

    movi    \tmp1, BIT(0)
    movi    \tmp2, WCL_CORE_0_WORLD_UPDATE_REG
    s32i    \tmp1, \tmp2, 0
    .endm

/****************************************************************************
 * Name: exception_entry_hook
 *
 * Description:
 *   Perform chip-specific exception entry operations.
 *
 * Entry Conditions:
 *   level  - Interrupt level
 *   reg_sp - Stack pointer
 *   tmp    - Temporary register
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_HAVE_GENERAL_EXCEPTION_HOOKS
    .macro exception_entry_hook level reg_sp tmp
    .ifeq (\level - 1) * (\level - 3)
    clear_wcl_write_buffer \tmp

  /* Save World information from interruptee when handling User Exceptions
   * (Level 1) and Software-triggered interrupts (Level 3).
   */

    get_prev_world \level \tmp
    s32i    \tmp, \reg_sp, (4 * REG_INT_CTX) /* Save World into context */
    .endif
    .endm
#endif

/****************************************************************************
 * Name: exception_exit_hook
 *
 * Description:
 *   Perform chip-specific exception exit operations.
 *
 * Entry Conditions:
 *   level  - Interrupt level
 *   reg_sp - Stack pointer
 *   tmp1   - Temporary register 1
 *   tmp2   - Temporary register 2
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_HAVE_GENERAL_EXCEPTION_HOOKS
    .macro exception_exit_hook level reg_sp tmp1 tmp2
  /* Configure the World Controller for the new execution context before
   * returning from User Exceptions (Level 1) and Software-triggered
   * interrupts (Level 3).
   */

    .ifeq (\level - 1) * (\level - 3)
    movi    \tmp1, 0x0
    .ifeq (\level - 1)
    movi    \tmp2, WCL_CORE_0_STATUSTABLE1_REG
    .endif
    .ifeq (\level - 3)
    movi    \tmp2, WCL_CORE_0_STATUSTABLE2_REG
    .endif
    s32i    \tmp1, \tmp2, 0       /* Clear the table entry */

    l32i    \tmp1, \reg_sp, (4 * REG_INT_CTX)
    beqz    \tmp1, 1f
    set_next_world \reg_sp \tmp1 \tmp2

1:
    memw
    .endif
    .endm
#endif

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
