/****************************************************************************
 * arch/risc-v/src/common/riscv_cpuinfo.c
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

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      procfs_sprintf(buf, buf_size, &file_off, "processor\t: %d\n", i);
      procfs_sprintf(buf, buf_size, &file_off, "hart\t\t: %d\n", i);

      /* ISA */

      procfs_sprintf(buf, buf_size, &file_off, "isa\t\t: %si",
                     CONFIG_ARCH_FAMILY);
#ifdef CONFIG_ARCH_RV_ISA_M
      procfs_sprintf(buf, buf_size, &file_off, "%s", "m");
#endif
#ifdef CONFIG_ARCH_RV_ISA_A
      procfs_sprintf(buf, buf_size, &file_off, "%s", "a");
#endif
#ifdef CONFIG_ARCH_FPU
      procfs_sprintf(buf, buf_size, &file_off, "%s", "f");
#endif
#ifdef CONFIG_ARCH_DPFPU
      procfs_sprintf(buf, buf_size, &file_off, "%s", "d");
#endif
#ifdef CONFIG_ARCH_RV_ISA_C
      procfs_sprintf(buf, buf_size, &file_off, "%s", "c");
#endif

      /* MMU type */

      procfs_sprintf(buf, buf_size, &file_off, "\nmmu\t\t: ");
#ifdef ARCH_HAVE_MMU
# ifdef ARCH_MMU_TYPE_SV39
      procfs_sprintf(buf, buf_size, &file_off, "%s\n", "sv39");
# endif
#else
      procfs_sprintf(buf, buf_size, &file_off, "%s\n", "none");
#endif
    }

  return -file_off;
}
#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_CPUINFO */
