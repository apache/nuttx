/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_boot.c
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

#include <nuttx/config.h>

#include "arm_internal.h"

#ifdef CONFIG_ARM_PSCI
#  include "arm_cpu_psci.h"
#endif

#include "rtl8730e_irq.h"
#include "rtl8730e_memorymap.h"
#include "rtl8730e_userspace.h"
#include "smp.h"
#include "gic.h"
#include "scu.h"

#ifdef CONFIG_DEVICE_TREE
#  include <nuttx/fdt.h>
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION
#  include <sched/sched.h>
#  include <nuttx/sched_note.h>
#endif

#ifdef CONFIG_ARCH_ARMV7R
#  include <nuttx/init.h>
#endif

#include <nuttx/syslog/syslog_rpmsg.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_RPMSG
static char g_syslog_rpmsg_buf[4096];
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
#ifdef CONFIG_ARCH_PERF_EVENTS
  /* Perf init */

  up_perf_init(0);
#endif

#ifdef CONFIG_ARCH_ARMV7A
  /* Set the page table for section */

  rtl8730e_setupmappings();
#endif

#ifdef CONFIG_SMP
  /* Enable SMP cache coherency for CPU0 */

  arm_enable_smp(0);
#endif

  arm_fpuconfig();

#ifdef CONFIG_ARM_PSCI
  /* The CA32 runs in the normal world under the ATF SP_MIN secure monitor,
   * so PSCI calls (used for SMP CPU_ON in a later milestone) go through the
   * "smc" conduit rather than qemu's "hvc".
   */

  arm_psci_init("smc");
#endif

#ifdef CONFIG_DEVICE_TREE
  fdt_register((const char *)0x40000000);
#endif

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm_earlyserialinit();
#endif

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(g_syslog_rpmsg_buf, sizeof(g_syslog_rpmsg_buf));
#endif

#ifdef CONFIG_BUILD_PROTECTED
  rtl8730e_userspace();
#endif

#ifdef CONFIG_ARCH_ARMV7R
  /* dont return per armv7-r/arm_head.S design */

  nx_start();
#endif
}

#if defined(CONFIG_ARM_PSCI) && defined(CONFIG_SMP)
int up_cpu_start(int cpu)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Copy cpu0 page table to target cpu. */

  memcpy((uint32_t *)(PGTABLE_BASE_VADDR + PGTABLE_SIZE * cpu),
          (uint32_t *)PGTABLE_BASE_VADDR, PGTABLE_SIZE);
  UP_DSB();
#endif

  return psci_cpu_on(cpu, (uintptr_t)__start);
}
#endif
