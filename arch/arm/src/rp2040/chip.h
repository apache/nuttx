/****************************************************************************
 * arch/arm/src/rp2040/chip.h
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

#ifndef __ARCH_ARM_SRC_RP2040_CHIP_H
#define __ARCH_ARM_SRC_RP2040_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#endif

/* Include the chip capabilities file */

#include <arch/rp2040/chip.h>

/* Define the number of interrupt vectors that need to be supported for
 * this chip
 */

#define ARMV6M_PERIPHERAL_INTERRUPTS 32

/* Include the memory map file.  Other chip hardware files should then
 * include this file for the proper setup.
 */

#include "hardware/rp2040_memorymap.h"

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 3
#  include "hardware/rp2040_sio.h"
#endif

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the correct interrupt stack
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 3
  .macro  setintstack, tmp1, tmp2
  ldr  \tmp1, =RP2040_SIO_CPUID
  ldr  \tmp1, [\tmp1, #0]
  lsl  \tmp1, \tmp1, #2
  ldr  \tmp2, =g_cpu_intstack_top
  add  \tmp2, \tmp2, \tmp1
  ldr  \tmp2, [\tmp2, #0]
  mov  sp, \tmp2 /* sp = g_cpu_intstack_top[cpuid] */
  .endm
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 7 */

#endif /* __ASSEMBLY__  */
#endif /* __ARCH_ARM_SRC_RP2040_CHIP_H */
