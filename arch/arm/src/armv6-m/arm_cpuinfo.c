/****************************************************************************
 * arch/arm/src/armv6-m/arm_cpuinfo.c
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
#include <nuttx/fs/procfs.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hwcap.h"
#include "nvic.h"

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_CPUINFO)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: up_show_cpuinfo
 ****************************************************************************/

ssize_t up_show_cpuinfo(FAR char *buf, size_t buf_size, off_t file_off)
{
  int i;
  uint32_t cpuid;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      procfs_sprintf(buf, buf_size, &file_off, "processor\t: %d\n", i);
      procfs_sprintf(buf, buf_size, &file_off, "BogoMIPS\t: %u.%02u\n",
                     (CONFIG_BOARD_LOOPSPERMSEC / 1000),
                     (CONFIG_BOARD_LOOPSPERMSEC / 10) % 100);
      procfs_sprintf(buf, buf_size, &file_off, "cpu MHz\t\t: %lu.%02lu\n",
                     up_perf_getfreq() / 1000000,
                     (up_perf_getfreq() / 10000) % 100);

      /* CPU Features */

      procfs_sprintf(buf, buf_size, &file_off, "Features\t:");
      procfs_sprintf(buf, buf_size, &file_off, " %s %s %s %s",
                     HWCAP_HALF, HWCAP_FAST_MULT, HWCAP_THUMB, HWCAP_EDSP);

#ifdef CONFIG_SCHED_THREAD_LOCAL
      procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_TLS);
#endif

      /* Cpuid info */

      cpuid = getreg32(ARMV6M_SYSCON_CPUID);
      procfs_sprintf(buf, buf_size, &file_off,
                     "\nmodel name\t: %s rev %" PRIx32 " (%s)\n"
                     "CPU architecture: %s\n",
                     "ARMv6-compatible processor", cpuid & 15, "v6", "6TEJ");

      procfs_sprintf(buf, buf_size, &file_off,
                     "CPU implementer\t: 0x%02" PRIx32 "\n"
                     "CPU variant\t: 0x%" PRIx32 "\n"
                     "CPU part\t: 0x%03" PRIx32 "\n"
                     "CPU revision\t: %" PRIu32"\n\n",
                     cpuid >> 24,
                     (cpuid >> 20) & 0xf,
                     (cpuid >> 4) & 0xfff,
                     cpuid & 0xf);
    }

  return -file_off;
}
#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_CPUINFO */
