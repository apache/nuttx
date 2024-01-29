/****************************************************************************
 * arch/risc-v/src/k230/k230_hart.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI)

/****************************************************************************
 * Name: hart_has_vec_ext()
 * Description: returns Non-zero if CPU has vector extension
 ****************************************************************************/

int hart_has_vec_ext(void)
{
  #define MISA_VECTOR_BIT   ('V'-'A')
  #define MISA_VECOTR_MASK  ( 1 << MISA_VECTOR_BIT )

  return (READ_CSR(CSR_MISA) & MISA_VECOTR_MASK);
}

/****************************************************************************
 * Name: k230_hart_init()
 * Description: K230 M-mode HART initialization
 ****************************************************************************/

void k230_hart_init(void)
{
  if (hart_has_vec_ext())
    {
      WRITE_CSR(CSR_MHCR,  0x11ff);
      WRITE_CSR(CSR_MCOR,  0x70013);
      WRITE_CSR(CSR_MSMPR, 0x1);
      WRITE_CSR(CSR_MCCR2, 0xe0410009);
      WRITE_CSR(CSR_MHINT, 0x16e30c);
    }

  /* turn off MAEE to have standard PTE format */

  WRITE_CSR(CSR_MXSTATUS, 0x438000);
}
#endif /* !defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_NUTTSBI) */

#ifdef CONFIG_NUTTSBI
/****************************************************************************
 * Name: sbi_late_initialize runs in M-mode
 ****************************************************************************/

void sbi_late_initialize(void)
{
  /* delegate K230 plic enable to S-mode */

  *((volatile uint32_t *)K230_PLIC_CTRL) = 1;
  k230_hart_init();
}
#endif
