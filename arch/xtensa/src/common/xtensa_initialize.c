/****************************************************************************
 * arch/xtensa/src/common/xtensa_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "xtensa.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
static inline void xtensa_color_intstack(void)
{
#ifdef CONFIG_SMP
  uint32_t *ptr = (uint32_t *)xtensa_intstack_alloc();
#else
  uint32_t *ptr = (uint32_t *)&g_intstackalloc;
#endif
  ssize_t size;

  for (size = INTSTACK_SIZE * CONFIG_SMP_NCPUS;
       size > 0; size -= sizeof(uint32_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define xtensa_color_intstack()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  xtensa_color_intstack();

  /* Add any extra memory fragments to the memory manager */

  xtensa_add_region();

#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  xtensa_pminitialize();
#endif

  /* Initialize the internal heap */

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
  xtensa_imm_initialize();
#endif

#ifdef CONFIG_ARCH_DMA
  /* Initialize the DMA subsystem if the weak function xtensa_dma_initialize
   * has been brought into the build
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (xtensa_dma_initialize)
#endif
    {
      xtensa_dma_initialize();
    }
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  xtensa_serialinit();
#endif

#ifndef CONFIG_NETDEV_LATEINIT
  /* Initialize the network */

  up_netinitialize();
#endif

  /* Initialize USB -- device and/or host */

  up_usbinitialize();

  board_autoled_on(LED_IRQSENABLED);
}
