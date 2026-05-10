/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_cpustart.c
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

#include <assert.h>
#include <nuttx/debug.h>

#include "arm64_smp.h"
#include "arm64_internal.h"

#include "hardware/bcm2711_memmap.h"
#include "bcm2711_cpustart.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_cpustart
 *
 * Description:
 *   Populate CPU core spin tables with the correct start addresses and
 *   notify cores.
 *
 ****************************************************************************/

void bcm2711_cpustart(void)
{
  /* According to the bcm2711.dtsi file in the Linux source tree [1], the
   * CPUs on the BCM2711 use a spin-table enable method and poll the
   * following addresses:
   *
   * CPU0: 0x000000d8 (BCM_MBOX_CLR06)
   * CPU1: 0x000000e0 (BCM_MBOX_CLR08)
   * CPU2: 0x000000e8 (BCM_MBOX_CLR10)
   * CPU3: 0x000000f0 (BCM_MBOX_CLR12)
   *
   * Some kernel docs about booting [2] have a handy explanation of how this
   * works:
   *
   * "polling their cpu-release-addr location, which must be contained in the
   * reserved region ... when a read of the location pointed to by the
   * cpu-release-addr returns a non-zero value, the CPU must jump to this
   * value" [2]
   *
   * In our case, we want these CPUs to load the NuttX kernel defined by
   * `_start`. We don't need to worry about CPU0, that one always starts.
   * These are 64-bit words (hence skipping every second register).
   *
   * [1] https://github.com/raspberrypi/linux/blob/rpi-6.12.y/
   *     arch/arm/boot/dts/broadcom/bcm2711.dtsi
   *
   * [2] https://www.kernel.org/doc/Documentation/arm64/booting.txt
   */

  for (uint8_t cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      putreg64((uint64_t)__start, BCM_SPINTBL_CPU(cpu));
      sinfo("Starting CPU %u at %p\n", cpu, __start);
    }

  UP_DSB();
  UP_SEV();
}
