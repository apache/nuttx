/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_RV32M1_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_RV32M1_RV32M1_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum clk_e
{
  CLK_SIRC     = 0x000, /* Slow IRC Clock */
  CLK_SIRCDIV1,
  CLK_SIRCDIV2,
  CLK_SIRCDIV3,

  CLK_FIRC     = 0x100, /* Fast IRC Clock */
  CLK_FIRCDIV1,
  CLK_FIRCDIV2,
  CLK_FIRCDIV3,

  CLK_LPFLL    = 0x200, /* Low Power FLL Cock */
  CLK_LPFLLDIV1,
  CLK_LPFLLDIV2,
  CLK_LPFLLDIV3,

  CLK_SOSC     = 0x300,  /* System Oscillator Clock */
  CLK_SOSCDIV1,
  CLK_SOSCDIV2,
  CLK_SOSCDIV3,

  CLK_LPOC     = 0x400,  /* LPO Clock */
  CLK_ROSC     = 0x500,  /* RTC Clock */

  CLK_CORE     = 0x600,
  CLK_PLAT,
  CLK_SYS,

  CLK_BUS      = 0x700,
  CLK_EXT      = 0x800,
  CLK_SLOW     = 0x900,
};

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN void rv32m1_clockconfig(void);
EXTERN unsigned rv32m1_clockfreq(enum clk_e clk);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_RV32M1_RV32M1_CLOCKCONFIG_H */
