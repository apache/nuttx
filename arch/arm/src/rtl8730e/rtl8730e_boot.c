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
#include <nuttx/arch.h>

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

/* RTL8730E HSYS / CA32 register addresses for Core1 power sequencing.
 * Core1 is powered off by default; ATF SP_MIN waits for Core1 to poll its
 * mailbox but the core never wakes unless the HSYS power rails are enabled
 * first.  These constants mirror the SDK smp.c / ameba_hsys.h definitions
 * without requiring vendor headers.
 */

#define RTL8730E_HSYS_BASE      0x41000000u
#define RTL8730E_HSYS_HP_PWC    0x000u
#define RTL8730E_HSYS_HP_ISO    0x004u
#define RTL8730E_CA32_RST_CTRL  0x41000204u

#define HSYS_PSW_HP_AP_CORE(x)      (((x) & 0x3u) << 4)
#define HSYS_PSW_HP_AP_CORE_2ND(x)  (((x) & 0x3u) << 6)
#define HSYS_ISO_HP_AP_CORE(x)      (((x) & 0x3u) << 4)
#define HSYS_GET_ISO_HP_AP_CORE(x)  (((x) >> 4) & 0x3u)
#define CA32_NCOREPORESET(x)        (((x) & 0x3u) << 0)
#define CA32_NCORERESET(x)          (((x) & 0x3u) << 4)

static void rtl8730e_core1_power_on(void)
{
  volatile uint32_t *pwc = (volatile uint32_t *)(RTL8730E_HSYS_BASE +
                                                 RTL8730E_HSYS_HP_PWC);
  volatile uint32_t *iso = (volatile uint32_t *)(RTL8730E_HSYS_BASE +
                                                 RTL8730E_HSYS_HP_ISO);
  volatile uint32_t *rst = (volatile uint32_t *)RTL8730E_CA32_RST_CTRL;
  uint32_t val;

  /* Assert reset on core1 */

  *rst &= ~(CA32_NCOREPORESET(0x2u) | CA32_NCORERESET(0x2u));

  /* Assert isolation on core1 */

  val  = *iso;
  val |= HSYS_ISO_HP_AP_CORE(0x2u);
  *iso = val;
  up_udelay(50);

  /* First-stage power-on (mask 0x3 keeps core0 rails stable) */

  val  = *pwc;
  val |= HSYS_PSW_HP_AP_CORE(0x3u);
  *pwc = val;
  up_udelay(50);

  /* Second-stage power-on */

  val  = *pwc;
  val |= HSYS_PSW_HP_AP_CORE_2ND(0x3u);
  *pwc = val;
  up_udelay(500);

  /* Release isolation */

  val  = *iso;
  val &= ~HSYS_ISO_HP_AP_CORE(0x3u);
  *iso = val;
  up_udelay(50);

  /* Release reset */

  *rst |= (CA32_NCOREPORESET(0x2u) | CA32_NCORERESET(0x2u));
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

  if (cpu == 1)
    {
      rtl8730e_core1_power_on();
      up_udelay(40);
    }

  return psci_cpu_on(cpu, (uintptr_t)__start);
}
#endif
