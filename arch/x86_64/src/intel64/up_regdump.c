/****************************************************************************
 * arch/x86/src/intel64/up_regdump.c
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

#include <debug.h>
#include <nuttx/irq.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

void print_mem(void *sp, size_t size)
{
  char buf[9];
  int i;
  int j;

  _alert("Memory Dump (%d bytes):\n", size);

  for (i = 0; i < size / 8; i++)
    {
      for (j = 0; j < 8; j++)
        {
          buf[j] = *((uint8_t *)(sp + i * 8 + j));
          if ((buf[j] > 126) || (buf[j] < 32))
            {
              buf[j] = '.';
            }
        }

    buf[8] = '\0';
    _alert(" %016llx\t%02x %02x %02x %02x %02x %02x %02x %02x\t%s\n",
            (sp + i * 8),
            *((uint8_t *)(sp + i * 8 + 0)),
            *((uint8_t *)(sp + i * 8 + 1)),
            *((uint8_t *)(sp + i * 8 + 2)),
            *((uint8_t *)(sp + i * 8 + 3)),
            *((uint8_t *)(sp + i * 8 + 4)),
            *((uint8_t *)(sp + i * 8 + 5)),
            *((uint8_t *)(sp + i * 8 + 6)),
            *((uint8_t *)(sp + i * 8 + 7)),
            buf);
    }
}

void backtrace(uint64_t rbp)
{
  int i;

  _alert("Frame Dump (64 bytes):\n");

  for (i = 0; i < 16; i++)
    {
      if ((rbp < 0x200000) || (rbp > 0xffffffff))
        {
          break;
        }

      _alert("  %016llx\t%016llx\n",
             *((uint64_t *)(rbp)), *((uint64_t *)(rbp + 1 * 8)));

      if ((rbp) && *((uint64_t *)(rbp + 1 * 8)))
        {
          rbp = *(uint64_t *)rbp;
        }
      else
        {
          break;
        }
    }
}

void up_registerdump(uint64_t *regs)
{
  uint64_t mxcsr;
  uint64_t cr2;

  asm volatile ("stmxcsr %0"::"m"(mxcsr):"memory");
  asm volatile ("mov %%cr2, %%rax; mov %%rax, %0"::"m"(cr2):"memory", "rax");
  _alert("----------------CUT HERE-----------------\n");
  _alert("Gerneral Informations:\n");
  _alert("CPL: %d, RPL: %d\n", regs[REG_CS] & 0x3, regs[REG_DS] & 0x3);
  _alert("RIP: %016llx, RSP: %016llx\n", regs[REG_RIP], regs[REG_RSP]);
  _alert("RBP: %016llx, RFLAGS: %016llx\n",
         regs[REG_RBP], regs[REG_RFLAGS]);
  _alert("MSR_STAR: %016llx, MSR_LSTAR: %016llx\n",
         read_msr(0xc0000081), read_msr(0xc0000082));
  _alert("MXCSR: %016llx, MSR_FS_BASE: %016llx\n",
         mxcsr, read_msr(MSR_FS_BASE));
  _alert("CR2: %016llx\n", cr2);
  _alert("Selector Dump:\n");
  _alert("CS: %016llx, DS: %016llx, SS: %016llx\n",
         regs[REG_CS], regs[REG_DS], regs[REG_SS]);
  _alert("ES: %016llx, FS: %016llx, GS: %016llx\n",
         regs[REG_ES], regs[REG_FS], regs[REG_GS]);
  _alert("Register Dump:\n");
  _alert("RAX: %016llx, RBX: %016llx\n", regs[REG_RAX], regs[REG_RBX]);
  _alert("RCX: %016llx, RDX: %016llx\n", regs[REG_RCX], regs[REG_RDX]);
  _alert("RDI: %016llx, RSI: %016llx\n", regs[REG_RDI], regs[REG_RSI]);
  _alert(" R8: %016llx,  R9: %016llx\n", regs[REG_R8] , regs[REG_R9]);
  _alert("R10: %016llx, R11: %016llx\n", regs[REG_R10], regs[REG_R11]);
  _alert("R12: %016llx, R13: %016llx\n", regs[REG_R12], regs[REG_R13]);
  _alert("R14: %016llx, R15: %016llx\n", regs[REG_R14], regs[REG_R15]);
  _alert("Dumping Stack (+-64 bytes):\n");

  if (regs[REG_RSP] > 0 && regs[REG_RSP] < 0x1000000)
    {
      print_mem((void *)regs[REG_RSP] - 512,
          128 * 0x200000 - regs[REG_RSP] + 512);
    }
  else
    {
      print_mem((void *)regs[REG_RSP] - 512, 1024);
    }

#ifdef CONFIG_DEBUG_NOOPT
  backtrace(regs[REG_RBP]);
#endif
  _alert("-----------------------------------------\n");
}
