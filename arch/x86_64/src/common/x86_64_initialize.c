/****************************************************************************
 * arch/x86_64/src/common/x86_64_initialize.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifdef CONFIG_DEV_SIMPLE_ADDRENV
#  include <nuttx/drivers/addrenv.h>
#endif

#include <arch/acpi.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEV_SIMPLE_ADDRENV

static const struct simple_addrenv_s g_addrenv[] =
{
  /* Map 1:1 with 0x100000000 offset for RAM */

  {
    .va   = X86_64_LOAD_OFFSET,
    .pa   = 0,
    .size = CONFIG_RAM_SIZE
  },

  /* Map the rest of memory as 1:1 */

  {
    .va   = 0,
    .pa   = 0,
    .size = 0
  }
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_init
 *
 * Description:
 *   Initialize addrenv.
 *
 ****************************************************************************/

static void x86_64_addrenv_init(void)
{
#ifdef CONFIG_DEV_SIMPLE_ADDRENV
  simple_addrenv_initialize(g_addrenv);
#endif
}

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
  /* Add any extra memory fragments to the memory manager */

  x86_64_addregion();

  /* Initialzie addrenv */

  x86_64_addrenv_init();

#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  up_pminitialize();
#endif

#ifdef CONFIG_ARCH_DMA
  /* Initialize the DMA subsystem if the weak function x86_64_dma_initialize
   * has been brought into the build
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (x86_64_dma_initialize)
#endif
    {
      x86_64_dma_initialize();
    }
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  x86_64_serialinit();
#endif

  /* Initialize the network */

#ifndef CONFIG_NETDEV_LATEINIT
  x86_64_netinitialize();
#endif

  /* Initialize the PCI bus */

#ifdef CONFIG_PCI
  x86_64_pci_init();
#endif

  /* Initialize USB -- device and/or host */

  x86_64_usbinitialize();

#ifdef CONFIG_ARCH_X86_64_ACPI_DUMP
  /* Dump ACPI tables */

  acpi_dump();
#endif

  board_autoled_on(LED_IRQSENABLED);
}
