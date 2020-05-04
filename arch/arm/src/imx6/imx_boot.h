/****************************************************************************
 * arch/arm/src/imx/imx_boot.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
