/****************************************************************************
 * arch/arm/src/imx6/imx_boot.h
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_BOOT_H
#define __ARCH_ARM_SRC_IMX6_IMX_BOOT_H

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
 * Name: imx_cpu_disable
 *
 * Description:
 *   Called from CPU0 to make sure that all other CPUs are in the disabled
 *   state.  This is a formality because the other CPUs are actually running
 *   then we have probably already crashed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
void imx_cpu_disable(void);
#else
#  define imx_cpu_disable()
#endif

/****************************************************************************
 * Name: imx_cpu_enable
 *
 * Description:
 *   Called from CPU0 to enable all other CPUs.  The enabled CPUs will start
 *   execution at __cpuN_start and, after very low-level CPU initialization
 *   has been performed, will branch to arm_cpu_boot()
 *   (see arch/arm/src/armv7-a/smp.h)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
void imx_cpu_enable(void);
#else
#  define imx_cpu_enable()
#endif

/****************************************************************************
 * Name: imx_memory_initialize
 *
 * Description:
 *   All i.MX6 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization before memory has
 *   been configured.  This board-specific function is responsible for
 *   configuring any on-board memories.
 *
 *   Logic in imx_memory_initialize must be careful to avoid using any
 *   global variables because those will be uninitialized at the time this
 *   function is called.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx_memory_initialize(void);

/****************************************************************************
 * Name: imx_board_initialize
 *
 * Description:
 *   All i.MX6 architectures must provide the following entry point.  This
 *   entry point is called in the initialization phase -- after
 *   imx_memory_initialize and after all memory has been configured and
 *   mapped but before any devices have been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMX6_IMX_BOOT_H */
