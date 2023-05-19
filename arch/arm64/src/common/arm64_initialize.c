/****************************************************************************
 * arch/arm64/src/common/arm64_initialize.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <arch/limits.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/loop.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/power/pm.h>
#include <arch/chip/chip.h>

#include "arm64_arch.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif
#include "arm64_internal.h"
#include "chip.h"

/****************************************************************************
 * Public data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

/* For the case of configurations with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint64_t *g_current_regs[CONFIG_SMP_NCPUS];

#ifdef CONFIG_ARCH_FPU
static struct notifier_block g_fpu_panic_block;
#endif

#ifdef CONFIG_SMP
INIT_STACK_ARRAY_DEFINE(g_cpu_idlestackalloc, CONFIG_SMP_NCPUS,
                          SMP_STACK_SIZE);
INIT_STACK_ARRAY_DEFINE(g_interrupt_stacks, CONFIG_SMP_NCPUS,
                          INTSTACK_SIZE);
#else
/* idle thread stack for primary core */

INIT_STACK_DEFINE(g_idle_stack, CONFIG_IDLETHREAD_STACKSIZE);
INIT_STACK_DEFINE(g_interrupt_stack, INTSTACK_SIZE);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_intstack_alloc
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
uintptr_t arm64_intstack_alloc(void)
{
  return (uintptr_t)(g_interrupt_stacks[up_cpu_index()]);
}

/****************************************************************************
 * Name: arm64_intstack_top
 *
 * Description:
 *   Return a pointer to the top the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

uintptr_t arm64_intstack_top(void)
{
  return (uintptr_t)(g_interrupt_stacks[up_cpu_index()] + INTSTACK_SIZE);
}

#endif

/****************************************************************************
 * Name: up_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
static void up_color_intstack(void)
{
#ifdef CONFIG_SMP
  void *ptr = (void *)g_interrupt_stacks[up_cpu_index()];
#else
  void *ptr = (void *)g_interrupt_stack;
#endif
  arm64_stack_color(ptr, INTSTACK_SIZE);
}
#else
#  define up_color_intstack()
#endif

/****************************************************************************
 * Name: arm64_panic_disable_fpu
 *
 * Description:
 *   This is called when panic to close fpu.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
int arm64_panic_disable_fpu(struct notifier_block *this,
                            unsigned long action, void *data)
{
  arm64_fpu_disable();
  return 0;
}
#endif

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
  /* Initialize global variables */

  up_color_intstack();

  /* Add any extra memory fragments to the memory manager */

  arm64_addregion();

#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  arm64_pminitialize();
#endif

#ifdef CONFIG_ARCH_DMA

  arm64_dma_initialize();

#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  arm64_serialinit();
#endif

#ifndef CONFIG_NETDEV_LATEINIT
  /* Initialize the network */

  arm64_netinitialize();
#endif

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
  /* Initialize USB -- device and/or host */

  arm64_usbinitialize();
#endif

#ifdef CONFIG_ARCH_FPU
  g_fpu_panic_block.notifier_call = arm64_panic_disable_fpu;
  g_fpu_panic_block.priority = INT_MAX;
  panic_notifier_chain_register(&g_fpu_panic_block);

#ifdef CONFIG_FS_PROCFS_REGISTER
  arm64_fpu_procfs_register();
#endif

#endif
}
