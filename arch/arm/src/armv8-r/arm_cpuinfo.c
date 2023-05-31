/****************************************************************************
 * arch/arm/src/armv8-r/arm_cpuinfo.c
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
#include <nuttx/config.h>
#include <nuttx/fs/procfs.h>

#include "arm_internal.h"
#include "hwcap.h"
#include "cp15.h"

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_CPUINFO)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * name: hwcap_extract_field
 ****************************************************************************/

static int hwcap_extract_field(uint32_t features, int field)
{
  int feature;

  feature = (features >> field) & 0xf;
  if (feature > 7)
    {
      feature -= 16;
    }

  return feature;
}

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
      procfs_sprintf(buf, buf_size, &file_off, " %s %s %s",
                     HWCAP_HALF, HWCAP_FAST_MULT, HWCAP_EDSP);

      /* If the CPU supports LDREX/STREX and LDREXB/STREXB,
       * avoid advertising SWP; it may not be atomic with
       * multiprocessing cores.
       */

      if (hwcap_extract_field(CP15_GET(ID_ISAR3), 12) <= 1 &&
          hwcap_extract_field(CP15_GET(ID_ISAR4), 20) < 3)
        {
          procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_SWP);
        }

#ifdef CONFIG_ARM_THUMB
      if (hwcap_extract_field(CP15_GET(ID_ISAR0), 24) >= 1)
        {
          procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_IDIVT);
        }

      procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_THUMB);
#endif
      if (hwcap_extract_field(CP15_GET(ID_ISAR0), 24) == 2)
        {
          procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_IDIVA);
        }

#ifdef CONFIG_SCHED_THREAD_LOCAL
      procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_TLS);
#endif

      /* LPAE implies atomic ldrd/strd instructions */

      if (hwcap_extract_field(CP15_GET(ID_ISAR2), 0) == 1)
        {
          procfs_sprintf(buf, buf_size, &file_off, " %s", HWCAP_LPAE);
        }

      /* VFP Features */

#if defined(CONFIG_ARCH_FPU)
      procfs_sprintf(buf, buf_size, &file_off, " %s %s %s %s",
                     HWCAP_VFPV3, HWCAP_FPSP, HWCAP_FPDP, HWCAP_VFPV3D16);
#endif

      /* Cpuid info */

      cpuid = CP15_GET(MIDR);
      procfs_sprintf(buf, buf_size, &file_off,
                     "\nmodel name\t: %s rev %" PRIx32 " (%s)\n"
                     "CPU architecture: %s\n",
                     "ARMv8 Processor", cpuid & 15, "v8", "8");

      procfs_sprintf(buf, buf_size, &file_off,
                     "CPU implementer\t: 0x%02" PRIx32 "\n"
                     "CPU variant\t: 0x%" PRIx32 "\n"
                     "CPU part\t: 0x%03" PRIx32 "\n"
                     "CPU revision\t: %" PRIu32 "\n\n",
                     cpuid >> 24,
                     (cpuid >> 20) & 0xf,
                     (cpuid >> 4) & 0xfff,
                     cpuid & 0xf);
    }

  return -file_off;
}
#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_CPUINFO */
