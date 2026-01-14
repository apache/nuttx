/****************************************************************************
 * arch/risc-v/src/common/supervisor/riscv_sbi.c
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>

#include "riscv_sbi.h"
#include "riscv_internal.h"

#ifdef CONFIG_NUTTSBI
#include "sbi_mcall.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline sbiret_t sbi_ecall(unsigned int extid, unsigned int fid,
                                 uintreg_t parm0, uintreg_t parm1,
                                 uintreg_t parm2, uintreg_t parm3,
                                 uintreg_t parm4, uintreg_t parm5)
{
  register long r0 asm("a0") = (long)(parm0);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);
  register long r6 asm("a6") = (long)(fid);
  register long r7 asm("a7") = (long)(extid);
  sbiret_t ret;

  asm volatile
    (
     "ecall"
     : "+r"(r0), "+r"(r1)
     : "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6), "r"(r7)
     : "memory"
     );

  ret.error = r0;
  ret.value = (uintreg_t)r1;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sbi_err_map_errno(intreg_t err)
{
  switch (err)
  {
    case SBI_SUCCESS:
      return 0;
    case SBI_ERR_DENIED:
      return -EPERM;
    case SBI_ERR_INVALID_PARAM:
      return -EINVAL;
    case SBI_ERR_INVALID_ADDRESS:
      return -EFAULT;
    case SBI_ERR_NOT_SUPPORTED:
    case SBI_ERR_FAILED:
    default:
      return -ENOTSUP;
  };
}

void riscv_sbi_set_timer(uint64_t stime_value)
{
#ifdef CONFIG_ARCH_RV64
  sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER, stime_value, 0, 0, 0, 0,
            0);
#else
  sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER, stime_value,
            stime_value >> 32, 0, 0, 0, 0);
#endif
}

uint64_t riscv_sbi_get_time(void)
{
#ifdef CONFIG_NUTTSBI
  sbiret_t ret = sbi_ecall(SBI_EXT_FIRMWARE, SBI_EXT_FIRMWARE_GET_MTIME,
                           0, 0, 0, 0, 0, 0);

#  ifdef CONFIG_ARCH_RV64
  return ret.error;
#  else
  return (((uint64_t)ret.value << 32) | ret.error);
#  endif
#elif defined(CONFIG_ARCH_RV64)
  return READ_CSR(CSR_TIME);
#else
  uint32_t hi;
  uint32_t lo;

  do
    {
      hi = READ_CSR(CSR_TIMEH);
      lo = READ_CSR(CSR_TIME);
    }
  while (hi != READ_CSR(CSR_TIMEH));

  return (((uint64_t) hi) << 32) | lo;
#endif
}

void riscv_sbi_send_ipi(uintreg_t hmask, uintreg_t hbase)
{
  sbi_ecall(SBI_EXT_IPI, SBI_EXT_IPI_SEND_IPI,
            hmask, hbase, 0, 0, 0, 0);
}

#ifndef CONFIG_NUTTSBI
int riscv_sbi_boot_secondary(uintreg_t hartid, uintreg_t addr,
                                  uintreg_t a1)
{
  sbiret_t ret = sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_START,
                           hartid, addr, a1, 0, 0, 0);

  if (ret.error < 0)
    {
      return sbi_err_map_errno(ret.error);
    }

  return 0;
}

int riscv_sbi_system_reset(uint32_t type, uint32_t reason)
{
  sbiret_t ret = sbi_ecall(SBI_EXT_SRST, SBI_EXT_SRST_SYS_RESET,
                           type, reason, 0, 0, 0, 0);

  if (ret.error < 0)
    {
      return sbi_err_map_errno(ret.error);
    }

  return 0;
}
#endif /* CONFIG_NUTTSBI */
