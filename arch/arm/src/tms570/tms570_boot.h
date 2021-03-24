/****************************************************************************
 * arch/arm/src/tms570/tms570_boot.h
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

#ifndef __ARCH_ARM_SRC_TMS570_TMS570_BOOT_H
#define __ARCH_ARM_SRC_TMS570_TMS570_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"

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
 * Name: tms570_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.
 *   Performs low level initialization including setup of the console SCI.
 *   This SCI done early so that the serial console is available for
 *   debugging very early in the boot sequence.
 *
 ****************************************************************************/

void tms570_lowsetup(void);

/****************************************************************************
 * Name: tms570_boardinitialize
 *
 * Description:
 *   All TMS570 architectures must provide the following entry point.  This
 *   function is called near the beginning of _start.  This function is
 *   called after clocking has been configured but before caches have been
 *   enabled and before any devices have been initialized.
 *   .data/.bss memory may or may not have been initialized
 *   (see the "special precautions" below).
 *
 *   This function must perform low level initialization including
 *
 *   - Initialization of board-specific memory resources (e.g., SDRAM)
 *   - Configuration of board specific resources (GIOs, LEDs, etc).
 *   - Setup of the console SCI.  This SCI done early so that the serial
 *     console is available for debugging very early in the boot sequence.
 *
 *   Special precautions must be taken if .data/.bss lie in SRAM.  in that
 *   case, the boot logic cannot initialize .data or .bss.
 *   The function must then:
 *
 *   - Take precautions to assume that logic does not access any global data
 *     that might lie in SDRAM.
 *   - Call the function arm_data_initialize() as soon as SDRAM has been
 *     properly configured for use.
 *
 ****************************************************************************/

void tms570_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TMS570_TMS570_BOOT_H */
