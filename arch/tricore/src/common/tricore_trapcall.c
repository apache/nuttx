/****************************************************************************
 * arch/tricore/src/common/tricore_trapcall.c
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

#include <stddef.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/arch.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum tricore_trap_class
{
  TRICORE_CLASS_MMU,
  TRICORE_CLASS_IP,
  TRICORE_CLASS_IE,
  TRICORE_CLASS_CTX,
  TRICORE_CLASS_BUS,
  TRICORE_CLASS_ASSERT,
  TRICORE_CLASS_SYSCALL,
  TRICORE_CLASS_NMI
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trap description tables indexed by trap class and TIN.
 * For classes where TIN numbering starts at 1, index 0 is reserved.
 */

static const char * const g_trap_class0[] =
{
  "Virtual Address Fill",
  "Virtual Address Protection",
};

static const char * const g_trap_class1[] =
{
  "Reserved",
  "Privileged Instruction",
  "Memory Protection Read",
  "Memory Protection Write",
  "Memory Protection Execute",
  "Memory Protection Peripheral Access",
  "Memory Protection Null Address",
  "Global Register Write Protection",
};

static const char * const g_trap_class2[] =
{
  "Reserved",
  "Illegal Opcode",
  "Unimplemented Opcode",
  "Invalid Operand specification",
  "Data Address Alignment",
  "Invalid Local Memory Address",
};

static const char * const g_trap_class3[] =
{
  "Reserved",
  "Free Context List Depletion (FCX = LCX)",
  "Call Depth Overflow",
  "Call Depth Underflow",
  "Free Context List Underflow (FCX = 0)",
  "Call Stack Underflow (PCX = 0)",
  "Context Type (PCXI.UL wrong)",
  "Nesting Error: RFE with non-zero call depth",
};

static const char * const g_trap_class4[] =
{
  "Reserved",
  "Program Fetch Synchronous Error",
  "Data Access Synchronous Error",
  "Data Access Asynchronous Error",
  "Coprocessor Trap Asynchronous Error",
  "Program Memory Integrity Error",
  "Data Memory Integrity Error",
  "Temporal Asynchronous Error",
};

static const char * const g_trap_class5[] =
{
  "Reserved",
  "Arithmetic Overflow",
  "Sticky Arithmetic Overflow",
};

static const char * const g_trap_class6[] =
{
  "System Call",
};

static const char * const g_trap_class7[] =
{
  "Non-Maskable Interrupt",
};

static const char * const * const g_trap_class_str[] =
{
  g_trap_class0, g_trap_class1, g_trap_class2, g_trap_class3,
  g_trap_class4, g_trap_class5, g_trap_class6, g_trap_class7,
};

static const uint8_t g_trap_class_tin_count[] =
{
  sizeof(g_trap_class0) / sizeof(g_trap_class0[0]),
  sizeof(g_trap_class1) / sizeof(g_trap_class1[0]),
  sizeof(g_trap_class2) / sizeof(g_trap_class2[0]),
  sizeof(g_trap_class3) / sizeof(g_trap_class3[0]),
  sizeof(g_trap_class4) / sizeof(g_trap_class4[0]),
  sizeof(g_trap_class5) / sizeof(g_trap_class5[0]),
  sizeof(g_trap_class6) / sizeof(g_trap_class6[0]),
  sizeof(g_trap_class7) / sizeof(g_trap_class7[0]),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *tricore_trap_cause_str(uint32_t tclass, uint32_t tin)
{
  if (tclass >= sizeof(g_trap_class_str) / sizeof(g_trap_class_str[0]))
    {
      return "Unknown Trap Class";
    }

  if (tin >= g_trap_class_tin_count[tclass])
    {
      return "Unknown TIN";
    }

  return g_trap_class_str[tclass][tin];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_trap_handler
 *
 * Description:
 *   This is the common Trap exception handler. It is invoked by the BTV
 *   trap stubs in tricore_trap.S after svlcx has saved the lower context.
 *   The current PCXI therefore points at the just-saved lower CSA whose
 *   PCXI field chains back to the upper CSA pushed by hardware on trap
 *   entry.
 *
 ****************************************************************************/

void tricore_trap_handler(uint32_t tclass, uint32_t tin)
{
  uintptr_t *regs;
  uintptr_t pcxi;

  if (tclass == TRICORE_CLASS_SYSCALL)
    {
      tricore_svcall(NULL);
      return;
    }

  TRICORE_MFCR(TRICORE_CPU_PCXI, pcxi);
  regs = tricore_csa2addr(pcxi);

  lowsyslog("TriCore Trap: Class %" PRIu32 " TIN %" PRIu32 " (%s)\n",
            tclass, tin, tricore_trap_cause_str(tclass, tin));
  up_dump_register(regs);
}
