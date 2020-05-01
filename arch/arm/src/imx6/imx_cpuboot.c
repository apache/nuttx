/****************************************************************************
 * arch/arm/src/imx6/imx_cpuboot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/imx_src.h"
#include "sctlr.h"
#include "smp.h"
#include "scu.h"
#include "fpu.h"
#include "gic.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE void (*cpu_start_t)(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if 0 /* Not used */
static const uint32_t g_cpu_reset[CONFIG_SMP_NCPUS] =
{
    SRC_SCR_CORE0_RST
#if CONFIG_SMP_NCPUS > 1
  , SRC_SCR_CORE1_RST
#endif
#if CONFIG_SMP_NCPUS > 2
  , SRC_SCR_CORE2_RST
#endif
#if CONFIG_SMP_NCPUS > 3
  , SRC_SCR_CORE3_RST
#endif
};
#endif

static const uint32_t g_cpu_ctrl[CONFIG_SMP_NCPUS] =
{
    0
#if CONFIG_SMP_NCPUS > 1
  , SRC_SCR_CORE1_ENABLE
#endif
#if CONFIG_SMP_NCPUS > 2
  , SRC_SCR_CORE2_ENABLE
#endif
#if CONFIG_SMP_NCPUS > 3
  , SRC_SCR_CORE3_ENABLE
#endif
};

static const uintptr_t g_cpu_gpr[CONFIG_SMP_NCPUS] =
{
    IMX_SRC_GPR1
#if CONFIG_SMP_NCPUS > 1
  , IMX_SRC_GPR3
#endif
#if CONFIG_SMP_NCPUS > 2
  , IMX_SRC_GPR5
#endif
#if CONFIG_SMP_NCPUS > 3
  , IMX_SRC_GPR7
#endif
};

static const cpu_start_t g_cpu_boot[CONFIG_SMP_NCPUS] =
{
  0,
#if CONFIG_SMP_NCPUS > 1
  __cpu1_start,
#endif
#if CONFIG_SMP_NCPUS > 2
  __cpu2_start,
#endif
#if CONFIG_SMP_NCPUS > 3
  __cpu3_start
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Symbols defined via the linker script */

extern uint32_t _vector_start; /* Beginning of vector block */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_cpu_reset
 *
 * Description:
 *   CPUn software reset
 *
 ****************************************************************************/

#if 0 /* Not used */
static void imx_cpu_reset(int cpu)
{
  uint32_t regval;

  regval  = getreg32(IMX_SRC_SCR);
  regval |= g_cpu_reset[cpu];
  putreg32(regval, IMX_SRC_SCR);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

void imx_cpu_disable(void)
{
  uint32_t regval;
  uint32_t cpumask;

  cpumask  = 0;
#if CONFIG_SMP_NCPUS > 1
  cpumask |= SRC_SCR_CORE1_ENABLE;
#endif
#if CONFIG_SMP_NCPUS > 2
  cpumask |= SRC_SCR_CORE2_ENABLE;
#endif
#if CONFIG_SMP_NCPUS > 3
  cpumask |= SRC_SCR_CORE3_ENABLE;
#endif

  regval  = getreg32(IMX_SRC_SCR);
  regval &= ~cpumask;
  putreg32(regval, IMX_SRC_SCR);
}

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

void imx_cpu_enable(void)
{
  cpu_start_t bootaddr;
  uintptr_t regaddr;
  uint32_t regval;
  int cpu;

  for (cpu = 1; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* Set the start up address */

      regaddr  = g_cpu_gpr[cpu];
      bootaddr = g_cpu_boot[cpu];
      putreg32((uint32_t)bootaddr, regaddr);

      /* Then enable the CPU */

      regval   = getreg32(IMX_SRC_SCR);
      regval  |= g_cpu_ctrl[cpu];
      putreg32(regval, IMX_SRC_SCR);
    }
}

/****************************************************************************
 * Name: arm_cpu_boot
 *
 * Description:
 *   Continues the C-level initialization started by the assembly language
 *   __cpu[n]_start function.  At a minimum, this function needs to initialize
 *   interrupt handling and, perhaps, wait on WFI for arm_cpu_start() to
 *   issue an SGI.
 *
 *   This function must be provided by the each ARMv7-A MCU and implement
 *   MCU-specific initialization logic.
 *
 * Input Parameters:
 *   cpu - The CPU index.  This is the same value that would be obtained by
 *      calling up_cpu_index();
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void arm_cpu_boot(int cpu)
{
  /* Enable SMP cache coherency for the CPU */

  arm_enable_smp(cpu);

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
#endif

  /* Initialize the Generic Interrupt Controller (GIC) for CPUn (n != 0) */

  arm_gic_initialize();

#ifdef CONFIG_ARCH_LOWVECTORS
  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are two ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *  The second method is used by this logic.
   */

  /* Set the VBAR register to the address of the vector table */

  DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
  cp15_wrvbar((uint32_t)&_vector_start);
#endif /* CONFIG_ARCH_LOWVECTORS */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_enable();
#endif

  /* The next thing that we expect to happen is for logic running on CPU0
   * to call up_cpu_start() which generate an SGI and a context switch to
   * the configured NuttX IDLE task.
   */

  for (; ; )
    {
      asm("WFI");
    }
}
#endif /* CONFIG_SMP */
