/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpuinfo.c
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

#include <arch/xtensa/xtensa_specregs.h>
#include <sys/types.h>

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_CPUINFO)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_getconfig0
 *
 * Description:
 *   Get the value of the CONFIGID0 register.
 *
 ****************************************************************************/

static inline uint32_t xtensa_getconfig0(void)
{
  uint32_t reg;

    __asm__ __volatile__
  (
    "rsr %0, CONFIGID0"  : "=r"(reg)
  );

  return reg;
}

/****************************************************************************
 * Name: xtensa_getconfig1
 *
 * Description:
 *   Get the value of the CONFIGID1 register.
 *
 ****************************************************************************/

static inline uint32_t xtensa_getconfig1(void)
{
  uint32_t reg;

    __asm__ __volatile__
  (
    "rsr %0, CONFIGID1"  : "=r"(reg)
  );

  return reg;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: up_show_cpuinfo
 ****************************************************************************/

ssize_t up_show_cpuinfo(FAR char *buf, size_t buf_size, off_t file_off)
{
  procfs_sprintf(buf, buf_size, &file_off,
                 "CPU count\t: %u\n"
                 "vendor_id\t: Tensilica\n"
                 "model\t\t: Xtensa " XCHAL_HW_VERSION_NAME "\n",
                 CONFIG_SMP_NCPUS);

  procfs_sprintf(buf, buf_size, &file_off,
                 "core ID\t\t: " XCHAL_CORE_ID "\n"
                 "build ID\t: 0x%x\n"
                 "config ID\t: %08" PRIx32 ":%08" PRIx32 "\n",
                 XCHAL_BUILD_UNIQUE_ID,
                 xtensa_getconfig0(), xtensa_getconfig1());

  procfs_sprintf(buf, buf_size, &file_off,
                 "byte order\t: %s\n"
                 "cpu MHz\t\t: %lu.%02lu\n"
                 "bogomips\t: %u.%02u\n",
                 XCHAL_HAVE_BE ? "big" : "little",
                 up_perf_getfreq() / 1000000,
                 (up_perf_getfreq() / 10000) % 100,
                 (CONFIG_BOARD_LOOPSPERMSEC / 1000),
                 (CONFIG_BOARD_LOOPSPERMSEC / 10) % 100);

  /* Features */

  procfs_sprintf(buf, buf_size, &file_off, "flags\t\t: "
#if XCHAL_HAVE_NMI
                 "nmi "
#endif
#if XCHAL_HAVE_DEBUG
                 "debug "
# if XCHAL_HAVE_OCD
                 "ocd "
# endif
#endif
#if XCHAL_HAVE_DENSITY
                 "density "
#endif
#if XCHAL_HAVE_BOOLEANS
                 "boolean "
#endif
#if XCHAL_HAVE_LOOPS
                 "loop "
#endif
#if XCHAL_HAVE_NSA
                 "nsa "
#endif
#if XCHAL_HAVE_MINMAX
                 "minmax "
#endif
#if XCHAL_HAVE_SEXT
                 "sext "
#endif
#if XCHAL_HAVE_CLAMPS
                 "clamps "
#endif
#if XCHAL_HAVE_MAC16
                 "mac16 "
#endif
#if XCHAL_HAVE_MUL16
                 "mul16 "
#endif
#if XCHAL_HAVE_MUL32
                 "mul32 "
#endif
#if XCHAL_HAVE_MUL32_HIGH
                 "mul32h "
#endif
#if XCHAL_HAVE_FP
                 "fpu "
#endif
#if XCHAL_HAVE_S32C1I
                 "s32c1i "
#endif
#ifdef XCHAL_HAVE_EXCLUSIVE
# if XCHAL_HAVE_EXCLUSIVE
                 "exclusive "
# endif
#endif
                 "\n");

  /* Registers */

  procfs_sprintf(buf, buf_size, &file_off,
                 "physical aregs\t: %d\n"
                 "misc regs\t: %d\n"
                 "ibreak\t\t: %d\n"
                 "dbreak\t\t: %d\n",
                 XCHAL_NUM_AREGS,
                 XCHAL_NUM_MISC_REGS,
                 XCHAL_NUM_IBREAK,
                 XCHAL_NUM_DBREAK);

  /* Interrupt */

  procfs_sprintf(buf, buf_size, &file_off,
                 "num ints\t: %d\n"
                 "ext ints\t: %d\n"
                 "int levels\t: %d\n"
                 "timers\t\t: %d\n"
                 "debug level\t: %d\n",
                 XCHAL_NUM_INTERRUPTS,
                 XCHAL_NUM_EXTINTERRUPTS,
                 XCHAL_NUM_INTLEVELS,
                 XCHAL_NUM_TIMERS,
                 XCHAL_DEBUGLEVEL);

  /* Cache */

#ifdef CONFIG_XTENSA_ICACHE
  procfs_sprintf(buf, buf_size, &file_off,
                 "icache line size: %d\n"
                 "icache ways\t: %d\n"
                 "icache size\t: %d\n"
                 "icache flags\t: "
#if XCHAL_ICACHE_LINE_LOCKABLE
                 "lock "
#endif
                 "\n",
                 XCHAL_ICACHE_LINESIZE,
                 XCHAL_ICACHE_WAYS,
                 XCHAL_ICACHE_SIZE);
#endif

#ifdef CONFIG_XTENSA_DCACHE
  procfs_sprintf(buf, buf_size, &file_off,
                 "dcache line size: %d\n"
                 "dcache ways\t: %d\n"
                 "dcache size\t: %d\n"
                 "dcache flags\t: "
#if XCHAL_DCACHE_IS_WRITEBACK
                 "writeback "
#endif
#if XCHAL_DCACHE_LINE_LOCKABLE
                 "lock "
#endif
                 "\n",
                 XCHAL_DCACHE_LINESIZE,
                 XCHAL_DCACHE_WAYS,
                 XCHAL_DCACHE_SIZE);
#endif

  return -file_off;
}
#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_CPUINFO */
