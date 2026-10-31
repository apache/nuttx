/****************************************************************************
 * arch/arm64/src/common/arm64_fatal.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <arch/irq.h>
#include <nuttx/debug.h>
#include <assert.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/tls.h>
#include <nuttx/board.h>
#include <arch/chip/chip.h>
#include <nuttx/syslog/syslog.h>
#include "sched/sched.h"
#include "irq/irq.h"

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "arm64_mmu.h"
#include "arm64_arch_timer.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ISS bits of a data abort: 10, FAR is not valid; 6, a write caused it */

#define ESR_ELX_FNV             (1 << 10)
#define ESR_ELX_WNR             (1 << 6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fatal_handle_info
{
  fatal_handle_func_t handle_fn;
  const char *name;
};

/****************************************************************************
 * Private Functions Declarations
 ****************************************************************************/

/* Default callback handler for debug and fatal event
 * Can be override by other handler
 */

static int default_debug_handler(uint64_t *regs, uint64_t far, uint64_t esr);
static int default_fatal_handler(uint64_t *regs, uint64_t far, uint64_t esr);

/****************************************************************************
 * External Definitions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
/* The code of memcpy() and memset() (libs/libc/machine/arm64).  They are
 * not there if the C versions are in use.
 */

extern const char arm64_memcpy_start[] weak_data;
extern const char arm64_memcpy_end[] weak_data;
extern const char arm64_memset_start[] weak_data;
extern const char arm64_memset_end[] weak_data;
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_esr_class_str[] =
{
  [ESR_ELX_EC_UNKNOWN]     = "Unknown/Uncategorized",
  [ESR_ELX_EC_WFX]         = "WFI/WFE",
  [ESR_ELX_EC_CP15_32]     = "CP15 MCR/MRC",
  [ESR_ELX_EC_CP15_64]     = "CP15 MCRR/MRRC",
  [ESR_ELX_EC_CP14_MR]     = "CP14 MCR/MRC",
  [ESR_ELX_EC_CP14_LS]     = "CP14 LDC/STC",
  [ESR_ELX_EC_FP_ASIMD]    = "ASIMD",
  [ESR_ELX_EC_CP10_ID]     = "CP10 MRC/VMRS",
  [ESR_ELX_EC_PAC]         = "PAC",
  [ESR_ELX_EC_CP14_64]     = "CP14 MCRR/MRRC",
  [ESR_ELX_EC_BTI]         = "BTI",
  [ESR_ELX_EC_ILL]         = "PSTATE.IL",
  [ESR_ELX_EC_SVC32]       = "SVC (AArch32)",
  [ESR_ELX_EC_HVC32]       = "HVC (AArch32)",
  [ESR_ELX_EC_SMC32]       = "SMC (AArch32)",
  [ESR_ELX_EC_SVC64]       = "SVC (AArch64)",
  [ESR_ELX_EC_HVC64]       = "HVC (AArch64)",
  [ESR_ELX_EC_SMC64]       = "SMC (AArch64)",
  [ESR_ELX_EC_SYS64]       = "MSR/MRS (AArch64)",
  [ESR_ELX_EC_SVE]         = "SVE",
  [ESR_ELX_EC_ERET]        = "ERET/ERETAA/ERETAB",
  [ESR_ELX_EC_FPAC]        = "FPAC",
  [ESR_ELX_EC_SME]         = "SME",
  [ESR_ELX_EC_IMP_DEF]     = "EL3 IMP DEF",
  [ESR_ELX_EC_IABT_LOW]    = "IABT (lower EL)",
  [ESR_ELX_EC_IABT_CUR]    = "IABT (current EL)",
  [ESR_ELX_EC_PC_ALIGN]    = "PC Alignment",
  [ESR_ELX_EC_DABT_LOW]    = "DABT (lower EL)",
  [ESR_ELX_EC_DABT_CUR]    = "DABT (current EL)",
  [ESR_ELX_EC_SP_ALIGN]    = "SP Alignment",
  [ESR_ELX_EC_MOPS]        = "MOPS",
  [ESR_ELX_EC_FP_EXC32]    = "FP (AArch32)",
  [ESR_ELX_EC_FP_EXC64]    = "FP (AArch64)",
  [ESR_ELX_EC_SERROR]      = "SError",
  [ESR_ELX_EC_BREAKPT_LOW] = "Breakpoint (lower EL)",
  [ESR_ELX_EC_BREAKPT_CUR] = "Breakpoint (current EL)",
  [ESR_ELX_EC_SOFTSTP_LOW] = "Software Step (lower EL)",
  [ESR_ELX_EC_SOFTSTP_CUR] = "Software Step (current EL)",
  [ESR_ELX_EC_WATCHPT_LOW] = "Watchpoint (lower EL)",
  [ESR_ELX_EC_WATCHPT_CUR] = "Watchpoint (current EL)",
  [ESR_ELX_EC_BKPT32]      = "BKPT (AArch32)",
  [ESR_ELX_EC_VECTOR32]    = "Vector catch (AArch32)",
  [ESR_ELX_EC_BRK64]       = "BRK (AArch64)",
};

static const char *g_esr_desc_str[] =
{
  [ESR_ELX_EC_UNKNOWN]   = "Unknown/Uncategorized",
  [ESR_ELX_EC_WFX]       = "Trapped WFI or WFE instruction execution",
  [ESR_ELX_EC_CP15_32]   = "Trapped MCR or MRC access with"
                           "(coproc==0b1111) "
                           "that is not reported using EC 0b000000",
  [ESR_ELX_EC_CP15_64]   = "Trapped MCRR or MRRC access with"
                           "(coproc==0b1111) "
                           "that is not reported using EC 0b000000",
  [ESR_ELX_EC_CP14_MR]  = "Trapped MCR or MRC access with (coproc==0b1110)",
  [ESR_ELX_EC_CP14_LS]  = "Trapped LDC or STC access",
  [ESR_ELX_EC_FP_ASIMD] = "Trapped access to SVE, Advanced SIMD, or "
                          "floating-point functionality",
  [ESR_ELX_EC_CP10_ID] = "CP10 MRC/VMRS",
  [ESR_ELX_EC_PAC]     = "PAC",
  [ESR_ELX_EC_CP14_64] = "Trapped MRRC access with (coproc==0b1110)",
  [ESR_ELX_EC_BTI]     = "Branch Target Exception",
  [ESR_ELX_EC_ILL]     = "Illegal Execution state",
  [ESR_ELX_EC_SVC32]   = "SVC instruction execution in AArch32 state",
  [ESR_ELX_EC_HVC32]   = "HVC (AArch32)",
  [ESR_ELX_EC_SMC32]   = "SMC (AArch32)",
  [ESR_ELX_EC_SVC64]   = "SVC (AArch64)",
  [ESR_ELX_EC_HVC64]   = "HVC (AArch64)",
  [ESR_ELX_EC_SMC64]   = "SMC (AArch64)",
  [ESR_ELX_EC_SYS64]   = "Trapped MSR, MRS or System instruction "
                         "execution in AArch64 state, that is not "
                         "reported using EC 0b000000, 0b000001 or 0b000111",
  [ESR_ELX_EC_SVE]  = "Trapped access to SVE functionality",
  [ESR_ELX_EC_ERET] = "ERET/ERETAA/ERETAB",
  [ESR_ELX_EC_FPAC] = "Exception from a Pointer Authentication "
                      "instruction authentication failure",
  [ESR_ELX_EC_SME] = "SME",
  [ESR_ELX_EC_IMP_DEF]  = "EL3 IMP DEF",
  [ESR_ELX_EC_IABT_LOW] = "Instruction Abort from a lower Exception level, "
                          "that might be using AArch32 or AArch64",
  [ESR_ELX_EC_IABT_CUR] = "Instruction Abort taken without a change "
                          "in Exception level",
  [ESR_ELX_EC_PC_ALIGN] = "PC alignment fault exception.",
  [ESR_ELX_EC_DABT_LOW] = "Data Abort from a lower Exception level, "
                          "that might be using AArch32 or AArch64",
  [ESR_ELX_EC_DABT_CUR] = "Data Abort taken without a change in "
                          "Exception level",
  [ESR_ELX_EC_SP_ALIGN] = "SP alignment fault exception",
  [ESR_ELX_EC_MOPS]     = "MOPS",
  [ESR_ELX_EC_FP_EXC32] = "Trapped floating-point exception taken from "
                          "AArch32 state",
  [ESR_ELX_EC_FP_EXC64] = "Trapped floating-point exception taken from "
                          "AArch64 state",
  [ESR_ELX_EC_SERROR]   = "SError interrupt",
  [ESR_ELX_EC_BREAKPT_LOW] = "Breakpoint exception from a lower "
                             "Exception level, "
                             "that might be using AArch32 or AArch64",
  [ESR_ELX_EC_BREAKPT_CUR] = "Breakpoint exception taken without a change "
                             "in Exception level",
  [ESR_ELX_EC_SOFTSTP_LOW] = "Software Step exception from a lower "
                             "Exception level,"
                             "that might be using AArch32 or AArch64",
  [ESR_ELX_EC_SOFTSTP_CUR] = "Software Step exception taken without a "
                             "change in Exception level",
  [ESR_ELX_EC_WATCHPT_LOW] = "Watchpoint exception from a lower "
                             "Exception level, "
                             "that might be using AArch32 or AArch64",
  [ESR_ELX_EC_WATCHPT_CUR] = "Watchpoint exception taken without "
                             "a change in Exception level.",
  [ESR_ELX_EC_BKPT32]   = "BKPT instruction execution in AArch32 state",
  [ESR_ELX_EC_VECTOR32] = "Vector catch (AArch32)",
  [ESR_ELX_EC_BRK64]    = "BRK instruction execution in AArch64 state.",
};

static struct fatal_handle_info g_fatal_handler[] =
{
  { default_fatal_handler, "ttbr address size fault" },
  { default_fatal_handler, "level 1 address size fault" },
  { default_fatal_handler, "level 2 address size fault" },
  { default_fatal_handler, "level 3 address size fault" },
  { default_fatal_handler, "level 0 translation fault" },
  { default_fatal_handler, "level 1 translation fault" },
  { default_fatal_handler, "level 2 translation fault" },
  { default_fatal_handler, "level 3 translation fault" },
  { default_fatal_handler, "unknown 8"                 },
  { default_fatal_handler, "level 1 access flag fault" },
  { default_fatal_handler, "level 2 access flag fault" },
  { default_fatal_handler, "level 3 access flag fault" },
  { default_fatal_handler, "unknown 12"                },
  { default_fatal_handler, "level 1 permission fault"  },
  { default_fatal_handler, "level 2 permission fault"  },
  { default_fatal_handler, "level 3 permission fault"  },
  { default_fatal_handler, "synchronous external abort" },
  { default_fatal_handler, "synchronous tag check fault" },
  { default_fatal_handler, "unknown 18" },
  { default_fatal_handler, "unknown 19" },
  { default_fatal_handler, "level 0 (translation table walk)" },
  { default_fatal_handler, "level 1 (translation table walk)" },
  { default_fatal_handler, "level 2 (translation table walk)" },
  { default_fatal_handler, "level 3 (translation table walk)" },
  { default_fatal_handler, "synchronous parity or ECC error" },
  { default_fatal_handler, "unknown 25" },
  { default_fatal_handler, "unknown 26" },
  { default_fatal_handler, "unknown 27" },
  { default_fatal_handler, "level 0 synchronous parity "
                      "error (translation table walk)" },
  { default_fatal_handler, "level 1 synchronous parity "
                      "error (translation table walk)" },
  { default_fatal_handler, "level 2 synchronous parity "
                      "error (translation table walk)" },
  { default_fatal_handler, "level 3 synchronous parity "
                      "error (translation table walk)" },
  { default_fatal_handler, "unknown 32"   },
  { default_fatal_handler, "alignment fault" },
  { default_fatal_handler, "unknown 34" },
  { default_fatal_handler, "unknown 35" },
  { default_fatal_handler, "unknown 36" },
  { default_fatal_handler, "unknown 37" },
  { default_fatal_handler, "unknown 38" },
  { default_fatal_handler, "unknown 39" },
  { default_fatal_handler, "unknown 40" },
  { default_fatal_handler, "unknown 41" },
  { default_fatal_handler, "unknown 42" },
  { default_fatal_handler, "unknown 43" },
  { default_fatal_handler, "unknown 44" },
  { default_fatal_handler, "unknown 45" },
  { default_fatal_handler, "unknown 46" },
  { default_fatal_handler, "unknown 47" },
  { default_fatal_handler, "TLB conflict abort" },
  { default_fatal_handler, "Unsupported atomic hardware update fault" },
  { default_fatal_handler, "unknown 50" },
  { default_fatal_handler, "unknown 51" },
  { default_fatal_handler, "implementation fault (lockdown abort)" },
  { default_fatal_handler, "implementation fault (unsupported exclusive)" },
  { default_fatal_handler, "unknown 54" },
  { default_fatal_handler, "unknown 55" },
  { default_fatal_handler, "unknown 56" },
  { default_fatal_handler, "unknown 57" },
  { default_fatal_handler, "unknown 58" },
  { default_fatal_handler, "unknown 59" },
  { default_fatal_handler, "unknown 60" },
  { default_fatal_handler, "section domain fault" },
  { default_fatal_handler, "page domain fault" },
  { default_fatal_handler, "unknown 63" },
};

static struct fatal_handle_info g_debug_handler[] =
{
  { default_debug_handler, "hardware breakpoint"  },
  { default_debug_handler, "hardware single-step" },
  { default_debug_handler, "hardware watchpoint"  },
  { default_debug_handler, "unknown 3"            },
  { default_debug_handler, "aarch32 BKPT"         },
  { default_debug_handler, "aarch32 vector catch" },
  { default_debug_handler, "aarch64 BRK"          },
  { default_debug_handler, "unknown 7"            },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *esr_get_class_string(uint64_t esr)
{
  uint32_t ec = ESR_ELX_EC(esr);

  return g_esr_class_str[ec];
}

static const char *esr_get_desc_string(uint64_t esr)
{
  uint32_t ec = ESR_ELX_EC(esr);

  return g_esr_desc_str[ec];
}

static void print_ec_cause(uint64_t esr)
{
  const char *class_string = esr_get_class_string(esr);
  const char *desc_string = esr_get_desc_string(esr);

  if (class_string && desc_string)
    {
      serr("%s\n", class_string);
      serr("%s\n", desc_string);
    }
  else
    {
      serr("UNRECOGNIZED EC\n");
    }
}

static int default_fatal_handler(uint64_t *regs, uint64_t far, uint64_t esr)
{
  struct fatal_handle_info *inf = g_fatal_handler + (esr & ESR_ELX_FSC);

  /* Data Fault Status Code. */

  serr("(IFSC/DFSC) for Data/Instruction aborts: %s\n", inf->name);

  return -EINVAL; /* "fault" */
}

static int default_debug_handler(uint64_t *regs, uint64_t far, uint64_t esr)
{
  struct fatal_handle_info *inf = g_debug_handler + DBG_ESR_EVT(esr);

  serr("Default Debug Handler: %s\n", inf->name);
  return -1; /* "fault" */
}

static int arm64_el1_abort(uint64_t *regs, uint64_t esr)
{
  uint64_t                  far = read_sysreg(far_el1);
  struct fatal_handle_info *inf = g_fatal_handler + (esr & ESR_ELX_FSC);

  return inf->handle_fn(regs, far, esr);
}

static int arm64_el1_pc(uint64_t *regs, uint64_t esr)
{
  uint64_t far = read_sysreg(far_el1);

  serr("SP/PC alignment exception at 0x%" PRIx64 "\n", far);
  return -EINVAL; /* "fault" */
}

static int arm64_el1_bti(uint64_t *regs, uint64_t esr)
{
  uint64_t far = read_sysreg(far_el1);

  serr("BTI exception at 0x%" PRIx64 "\n", far);
  return -EINVAL; /* "fault" */
}

static int arm64_el1_undef(uint64_t *regs, uint64_t esr)
{
  uint64_t elr = regs[REG_ELR];
  uint32_t insn;
  int      i;

  serr("Undefined instruction at 0x%" PRIx64 "\n", elr);

  /* Only dump the surrounding words for an exception taken at EL1, where
   * ELR is kernel code that was just fetched.  From EL0, ELR is a user
   * address and the words around it can be in a page that is not mapped.
   * A fault here is a nested exception, which trips the DEBUGASSERT in
   * arm64_fatal_handler() before the user fault is reported.
   */

  if ((regs[REG_SPSR] & SPSR_MODE_MASK) == SPSR_MODE_EL0T)
    {
      return -1;
    }

  serr("dump:\n");
  for (i = -8; i <= 8; i += 4)
    {
      memcpy(&insn, (FAR void *)(elr + i), sizeof(insn));
      serr("0x%" PRIx64 " : 0x%" PRIx32 "\n", elr + i, insn);
    }

  return -1;
}

static int arm64_el1_fpac(uint64_t *regs, uint64_t esr)
{
  uint64_t far = read_sysreg(far_el1);

  /* Unexpected FPAC exception in the kernel. */

  serr("Unexpected FPAC exception at 0x%" PRIx64 "\n", far);
  return -EINVAL;
}

static int arm64_el1_dbg(uint64_t *regs, uint64_t esr)
{
  uint64_t                  far = read_sysreg(far_el1);
  struct fatal_handle_info *inf = g_debug_handler + DBG_ESR_EVT(esr);

  return inf->handle_fn(regs, far, esr);
}

static int arm64_el1_exception_handler(uint64_t esr,
                                       uint64_t *regs)
{
  uint32_t  ec = ESR_ELX_EC(esr);
  int       ret;

  switch (ec)
    {
      /* Data/Instruction Abort at EL1 */

      case ESR_ELX_EC_DABT_CUR:
      case ESR_ELX_EC_IABT_CUR:
        {
          ret = arm64_el1_abort(regs, esr);
          break;
        }

      /* PC alignment fault exception. */

      case ESR_ELX_EC_PC_ALIGN:
        {
          ret = arm64_el1_pc(regs, esr);
          break;
        }

      /* Trapped MSR, MRS or System instruction execution
       * in AArch64 state
       */

      case ESR_ELX_EC_SYS64:
      case ESR_ELX_EC_UNKNOWN:
        {
          ret = arm64_el1_undef(regs, esr);
          break;
        }

      /* Branch Target Exception */

      case ESR_ELX_EC_BTI:
        {
          ret = arm64_el1_bti(regs, esr);
          break;
        }

      case ESR_ELX_EC_BREAKPT_CUR:

      /* Breakpoint exception taken in current Exception level */

      case ESR_ELX_EC_SOFTSTP_CUR:

      /* Software Step exception taken in current Exception level */

      case ESR_ELX_EC_WATCHPT_CUR:

      /* Watchpoint exception taken in current Exception level */

      case ESR_ELX_EC_BRK64:
        {
          /* BRK instruction execution in AArch64 state */

          ret = arm64_el1_dbg(regs, esr);
          break;
        }

      case ESR_ELX_EC_FPAC:
        {
          /* Exception from a Pointer Authentication
           * instruction authentication failure
           */

          ret = arm64_el1_fpac(regs, esr);
          break;
        }

      default:
        {
          serr("64-bit el1h sync, esr = 0x%x", ec);
          ret = -EINVAL;
        }
    }

  return ret;
}

static void arm64_get_exception_info(uint64_t el, uint64_t *esr,
                                     uint64_t *far, uint64_t *elr,
                                     const char **el_str)
{
  switch (el)
    {
      case MODE_EL1:
        {
          if (el_str != NULL)
            {
              *el_str = "MODE_EL1";
            }

          if (esr != NULL)
            {
              *esr = read_sysreg(esr_el1);
            }

          if (far != NULL)
            {
              *far = read_sysreg(far_el1);
            }

          if (elr != NULL)
            {
              *elr = read_sysreg(elr_el1);
            }
          break;
        }

      case MODE_EL2:
        {
          if (el_str != NULL)
            {
              *el_str = "MODE_EL2";
            }

          if (esr != NULL)
            {
              *esr = read_sysreg(esr_el2);
            }

          if (far != NULL)
            {
              *far = read_sysreg(far_el2);
            }

          if (elr != NULL)
            {
              *elr = read_sysreg(elr_el2);
            }
          break;
        }

#ifdef CONFIG_ARCH_HAVE_EL3
      case MODE_EL3:
        {
          if (el_str != NULL)
            {
              *el_str = "MODE_EL3";
            }

          if (esr != NULL)
            {
              *esr = read_sysreg(esr_el3);
            }

          if (far != NULL)
            {
              *far = read_sysreg(far_el3);
            }

          if (elr != NULL)
            {
              *elr = read_sysreg(elr_el3);
            }
          break;
        }
#endif

      default:
        {
          if (el_str != NULL)
            {
              *el_str = "Unknown";
            }

          if (esr != NULL)
            {
              *esr = 0;
            }

          if (far != NULL)
            {
              *far = 0;
            }

          if (elr != NULL)
            {
              *elr = 0;
            }
          break;
        }
    }
}

static int arm64_exception_handler(uint64_t *regs)
{
  uint64_t    el;
  uint64_t    esr;
  uint64_t    elr;
  uint64_t    far;
  const char *el_str;
  int         ret = -EINVAL;

  el = arm64_current_el();
  arm64_get_exception_info(el, &esr, &far, &elr, &el_str);

  if (el == MODE_EL1)
    {
      ret = arm64_el1_exception_handler(esr, regs);
    }

  if (ret != 0)
    {
      serr("CurrentEL: %s\n", el_str);
      serr("ESR_ELn: 0x%" PRIx64 "\n", esr);
      serr("FAR_ELn: 0x%" PRIx64 "\n", far);
      serr("ELR_ELn: 0x%" PRIx64 "\n", elr);

      print_ec_cause(esr);
    }

  return ret;
}

#ifdef CONFIG_BUILD_KERNEL
/****************************************************************************
 * Name: arm64_syscall_user_fault
 *
 * Description:
 *   Return true if a kernel data abort is on a user address that a system
 *   call touched for its caller.  Then the bad pointer came from the
 *   caller, and only the caller must end.
 *
 ****************************************************************************/

static bool arm64_syscall_user_fault(struct tcb_s *tcb, uint64_t esr,
                                     uint64_t far)
{
  uint64_t fsc = esr & ESR_ELX_FSC_TYPE;

  return ESR_ELX_EC(esr) == ESR_ELX_EC_DABT_CUR &&
         (esr & ESR_ELX_FNV) == 0 &&
         (fsc == ESR_ELX_FSC_FAULT || fsc == ESR_ELX_FSC_ACCESS ||
          fsc == ESR_ELX_FSC_PERM) &&
         (tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL &&
         (tcb->flags & (TCB_FLAG_SYSCALL | TCB_FLAG_EXIT_PROCESSING)) ==
         TCB_FLAG_SYSCALL &&
         tcb->addrenv_curr == tcb->addrenv_own &&
         up_addrenv_user_vaddr(far);
}

/****************************************************************************
 * Name: arm64_syscall_resume
 *
 * Description:
 *   Return true if the fault is in memcpy() or memset().  Both are leaf
 *   functions: x30 holds the return address and x0 the destination, so the
 *   handler can return from them, and the system call can finish and
 *   release its locks, references and memory.  If memcpy() faulted on a
 *   read of the user source, clear the kernel destination first, so that
 *   old kernel data does not go on in place of the user data.
 *
 ****************************************************************************/

static bool arm64_syscall_resume(uint64_t *regs, uint64_t esr)
{
  uint64_t elr = regs[REG_ELR];
  uint64_t dst = regs[REG_X0];
  uint64_t end = regs[REG_X5];   /* dstend in arch_memcpy.S */

  if (arm64_memcpy_start != NULL &&
      elr >= (uintptr_t)arm64_memcpy_start &&
      elr < (uintptr_t)arm64_memcpy_end)
    {
      if ((esr & ESR_ELX_WNR) == 0 && end > dst &&
          !up_addrenv_user_vaddr(dst) && !up_addrenv_user_vaddr(end - 1))
        {
          memset((void *)dst, 0, end - dst);
        }

      return true;
    }

  return arm64_memset_start != NULL &&
         elr >= (uintptr_t)arm64_memset_start &&
         elr < (uintptr_t)arm64_memset_end;
}

/****************************************************************************
 * Name: arm64_kstack_frame
 *
 * Description:
 *   Move the exception frame to the top of the kernel stack, so that
 *   _exit() does not run below the frames of the system call that faulted.
 *
 ****************************************************************************/

static uint64_t *arm64_kstack_frame(struct tcb_s *tcb, uint64_t *regs)
{
#ifdef CONFIG_ARCH_KERNEL_STACK
  uintptr_t base = (uintptr_t)tcb->xcp.kstack;
  uintptr_t top  = base + ARCH_KERNEL_STACKSIZE - XCPTCONTEXT_SIZE;

  if (base != 0 && (uintptr_t)regs >= base && (uintptr_t)regs <= top)
    {
      memmove((void *)top, regs, XCPTCONTEXT_SIZE);
      regs = (uint64_t *)top;
    }
#endif

  return regs;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint64_t *arm64_fatal_handler(uint64_t *regs)
{
  struct tcb_s *tcb = this_task();
  bool inirq = up_interrupt_context();
  bool resume = false;
  bool report = true;
  bool user = false;
  uint64_t esr;
  uint64_t far;
  int ret;

  /* Nested exception are not supported */

  DEBUGASSERT(!inirq);

  tcb->xcp.regs = (uint64_t *)regs;

  /* Set irq flag */

  write_sysreg((uintptr_t)tcb | 1, tpidr_el1);

  ret = arm64_exception_handler(regs);

  if (ret != 0)
    {
      arm64_get_exception_info(arm64_current_el(), &esr, &far, NULL, NULL);

      if (((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL) &&
          ((tcb->flags & TCB_FLAG_SYSCALL) == 0) &&
          ((regs[REG_SPSR] & SPSR_MODE_MASK) == SPSR_MODE_EL0T))
        {
          user = true;
        }
#ifdef CONFIG_BUILD_KERNEL
      else if (!inirq && arm64_syscall_user_fault(tcb, esr, far))
        {
          /* A bad pointer from the caller: end the caller, not the system.
           * In memcpy() or memset(), return from it and end the caller when
           * the system call returns (see dispatch_syscall()).  Elsewhere
           * end the caller now: the system call does not finish, so what
           * it holds stays held.
           */

          resume = arm64_syscall_resume(regs, esr);
          report = !resume || tcb->xcp.ufault == 0;
          if (report)
            {
              _alert("Bad user address 0x%" PRIx64 " in a system call\n",
                     far);
            }

          if (resume)
            {
              tcb->xcp.ufault = far;
            }
          else
            {
              tcb->flags &= ~TCB_FLAG_SYSCALL;
              regs = arm64_kstack_frame(tcb, regs);
              tcb->xcp.regs = regs;
            }

          user = true;
        }
#endif

      if (user)
        {
          struct tcb_s *ptcb;
          const char *reason;
          const char *desc;

          reason = esr_get_class_string(esr);
          if (reason == NULL)
            {
              reason = "Unknown/Uncategorized";
            }

          desc = esr_get_desc_string(esr);
          if (desc == NULL)
            {
              desc = "";
            }

          /* This is not a panic:  the offending task is terminated and the
           * system continues.  Say so in the same words as risc-v, and leave
           * the register dump to the paths that are fatal.
           */

          if (report)
            {
              ptcb = nxsched_get_tcb(tcb->group->tg_pid);

              _alert("Segmentation fault in %s (PID %d: %s)\n",
                     ptcb != NULL ? get_task_name(ptcb) : "<gone>",
                     tcb->pid, get_task_name(tcb));
              _alert("Reason: %s - %s\n", reason, desc);
            }

          if (resume)
            {
              /* Return from memcpy() or memset() to its caller */

              regs[REG_ELR] = regs[REG_X30];
            }
          else
            {
              tcb->flags |= TCB_FLAG_FORCED_CANCEL;

              regs[REG_ELR] = (uint64_t)_exit;
              regs[REG_X0] = SIGSEGV;
              regs[REG_SPSR] &= ~SPSR_MODE_MASK;
              regs[REG_SPSR] |= SPSR_MODE_EL1H;
            }
        }
      else
        {
          /* The fatal is not handled, print error and hung */

          PANIC_WITH_REGS("panic", regs);
        }
    }

  /* Clear irq flag */

  write_sysreg((uintptr_t)tcb & ~1ul, tpidr_el1);

  return regs;
}

void arm64_register_debug_hook(int nr, fatal_handle_func_t fn)
{
  DEBUGVERIFY(nr > 0 && nr <= nitems(g_debug_handler));

  /* Override the default handler */

  g_debug_handler[nr].handle_fn = fn;
}
