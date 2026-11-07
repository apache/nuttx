/****************************************************************************
 * arch/rh850/src/common/rh850_exception.c
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

#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include "rh850_internal.h"
#include "rh850_sysreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The high 16 bits of EIIC/FEIC hold a detail code */

#define RH850_CAUSE(code)   ((code) & 0xffff)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *rh850_cause_str(uint32_t cause)
{
  uint32_t c = RH850_CAUSE(cause);

  if (c >= 0x10 && c <= 0x1f)
    {
      return "SYSERR";
    }

  if (c >= 0x30 && c <= 0x3f)
    {
      return "FETRAP";
    }

  if (c >= 0x40 && c <= 0x4f)
    {
      return "TRAP0";
    }

  if (c >= 0x50 && c <= 0x5f)
    {
      return "TRAP1";
    }

  if (c >= 0x80 && c <= 0x82)
    {
      return "UCPOP";
    }

  if (c >= 0xf0 && c <= 0xff)
    {
      return "FEINT";
    }

  switch (c)
    {
      case 0x60:
        return "RIE";

      case 0x71:
        return "FPE";

      case 0x72:
        return "FPI";

      case 0x90:
        return "MIP";

      case 0x91:
      case 0x95:
        return "MDP";

      case 0xa0:
        return "PIE";

      case 0xc0:
        return "MAE";

      case 0xe0:
        return "FENMI";

      default:
        return "unknown";
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_exception
 *
 * Description:
 *   EI and FE level exceptions and TRAP 0 codes other than the system call:
 *   report the cause and panic.
 *
 ****************************************************************************/

void rh850_exception(uint32_t cause, uint32_t level, uintptr_t *regs)
{
  bool fe = level == RH850_EXC_LEVEL_FE;
  uint32_t c = RH850_CAUSE(cause);

  _alert("EXCEPTION: %s level %s, %s %08" PRIx32 ", PC %08" PRIxPTR
         ", SP %08" PRIxPTR "\n",
         fe ? "FE" : "EI", rh850_cause_str(cause), fe ? "FEIC" : "EIIC",
         cause, regs[REG_EIPC], regs[REG_SP]);

  _alert("Interrupt context %d, EIINT nesting %d\n",
         up_interrupt_context(), g_rh850_irqnest);

  if (c == 0xc0 || c == 0x91 || c == 0x95)
    {
      _alert("MEA %08" PRIx32 " MEI %08" PRIx32 "\n",
             rh850_getsr(RH850_SR_MEA), rh850_getsr(RH850_SR_MEI));
    }

  _alert("PANIC!!! Exception = %" PRIx32 "\n", c);
  up_irq_save();
  up_set_interrupt_context(true);
  PANIC_WITH_REGS("panic", regs);
}
