/****************************************************************************
 * arch/arm/src/goldfish/goldfish_boot.c
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

#include "arm_internal.h"

#include "goldfish_irq.h"
#include "goldfish_memorymap.h"

#ifdef CONFIG_DEVICE_TREE
#  include <nuttx/fdt.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 ****************************************************************************/

void arm_boot(void)
{
  /* Perf init */

  up_perf_init(0);

  /* Set the page table for section */

  goldfish_setupmappings();

  arm_fpuconfig();

#ifdef CONFIG_DEVICE_TREE
  fdt_register((const char *)0x40000000);
#endif

#if defined(CONFIG_ARCH_HAVE_PSCI)
  arm_psci_init("smc");
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm_earlyserialinit();
#endif
}
