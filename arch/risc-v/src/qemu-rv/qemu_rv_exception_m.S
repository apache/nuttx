/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_exception_m.S
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

#include <arch/arch.h>
#include <arch/irq.h>
#include <arch/mode.h>

#include <sys/types.h>

#include "chip.h"

#include "riscv_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide a default section for the exeception handler. */

#ifndef EXCEPTION_SECTION
#  define EXCEPTION_SECTION .text
#endif

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

.section .text
.balign  8
.global  __trap_vec_m

/****************************************************************************
 * Name: __trap_vec_m
 *
 * Description:
 *   All M-mode exceptions and interrupts will be handled from here. If
 *   kernel is in S-mode delegated exceptions and interrupts are handled.
 *
 ****************************************************************************/

__trap_vec_m:
  j    exception_m

/****************************************************************************
 * Name: exception_m
 *
 * Description:
 *   Handles interrupts for m-mode
 *
 ****************************************************************************/

.section EXCEPTION_SECTION
.global exception_m
.align  8

exception_m:

  /* Swap mscratch with sp */
  /* NOTE: mscratch has been set in up_mtimer_initialize() */

  csrrw     sp, mscratch, sp

  /* Save the context */

  save_ctx  sp

  /* Handle the mtimer interrupt */
  /* NOTE: we assume exception/interrupt only happens for mtimer */

  jal       ra, qemu_rv_mtimer_interrupt

  /* Restore the context */

  load_ctx  sp

  /* Swap mscratch with sp */

  csrrw     sp, mscratch, sp

  /* Return from exception */

  mret
