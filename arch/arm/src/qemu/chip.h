/****************************************************************************
 * arch/arm/src/qemu/chip.h
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

#ifndef __ARCH_ARM_SRC_QEMU_CHIP_H
#define __ARCH_ARM_SRC_QEMU_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHIP_MPCORE_VBASE     0x8000000
#define MPCORE_ICD_OFFSET     0x0000
#define MPCORE_ICC_OFFSET     0x10000

#define PGTABLE_BASE_PADDR    (CONFIG_RAM_START + CONFIG_RAM_SIZE - ALL_PGTABLE_SIZE)
#define PGTABLE_BASE_VADDR    (CONFIG_RAM_START + CONFIG_RAM_SIZE - ALL_PGTABLE_SIZE)

#define NUTTX_TEXT_VADDR      (CONFIG_FLASH_VSTART & 0xfff00000)
#define NUTTX_TEXT_PADDR      (CONFIG_FLASH_VSTART & 0xfff00000)
#define NUTTX_TEXT_PEND       ((CONFIG_FLASH_END + 0x000fffff) & 0xfff00000)
#define NUTTX_TEXT_SIZE       (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

#define NUTTX_RAM_VADDR       (CONFIG_RAM_VSTART & 0xfff00000)
#define NUTTX_RAM_PADDR       (CONFIG_RAM_START & 0xfff00000)
#define NUTTX_RAM_PEND        ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#define NUTTX_RAM_SIZE        (NUTTX_RAM_PEND - NUTTX_RAM_PADDR)

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

/****************************************************************************
 * Name: cpuindex
 *
 * Description:
 *   Return an index idenifying the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
  .macro  cpuindex, index
  mrc  p15, 0, \index, c0, c0, 5  /* Read the MPIDR */
  and  \index, \index, #3         /* Bits 0-1=CPU ID */
  .endm
#endif

/****************************************************************************
 * Name: setirqstack
 *
 * Description:
 *   Set the current stack pointer to the  -"top" of the IRQ interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
  .macro  setirqstack, tmp1, tmp2
  mrc  p15, 0, \tmp1, c0, c0, 5  /* tmp1=MPIDR */
  and  \tmp1, \tmp1, #3          /* Bits 0-1=CPU ID */
  ldr  \tmp2, =g_irqstack_top    /* tmp2=Array of IRQ stack pointers */
  lsls \tmp1, \tmp1, #2          /* tmp1=Array byte offset */
  add  \tmp2, \tmp2, \tmp1       /* tmp2=Offset address into array */
  ldr  sp, [\tmp2, #0]           /* sp=Address in stack allocation */
  .endm
#endif

/****************************************************************************
 * Name: setfiqstack
 *
 * Description:
 *   Set the current stack pointer to the  -"top" of the FIQ interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
  .macro  setfiqstack, tmp1, tmp2
  mrc  p15, 0, \tmp1, c0, c0, 5  /* tmp1=MPIDR */
  and  \tmp1, \tmp1, #3          /* Bits 0-1=CPU ID */
  ldr  \tmp2, =g_fiqstack_top    /* tmp2=Array of FIQ stack pointers */
  lsls \tmp1, \tmp1, #2          /* tmp1=Array byte offset */
  add  \tmp2, \tmp2, \tmp1       /* tmp2=Offset address into array */
  ldr  sp, [\tmp2, #0]           /* sp=Address in stack allocation */
  .endm
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_QEMU_CHIP_H */
