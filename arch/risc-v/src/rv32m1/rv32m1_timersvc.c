/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_timersvc.c
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

#include "riscv_internal.h"
#include "rv32m1.h"
#include "hardware/rv32m1_tstmr.h"
#include "rv32m1_timersvc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Data Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_timersvc_up
 ****************************************************************************/

bool rv32m1_timersvc_up(void)
{
#ifdef CONFIG_RV32M1_TSTMR
  /* TSTMR starts up when System Reset and TSTMR is Enabled by Option Byte.
   * Query the Option Byte to check whether TSTMR is enabled.
   */

  return (getreg8(RV32M1_FTFE_BASE + 0x10) & 0x01) != 0;
#else
  return false;
#endif
}

/****************************************************************************
 * Name: rv32m1_timersvc_freq
 ****************************************************************************/

uint32_t rv32m1_timersvc_freq(void)
{
  /* TSTMR runs off 1MHz */

  return  1000000u;
}

/****************************************************************************
 * Name: rv32m1_timersvc_period
 ****************************************************************************/

uint32_t rv32m1_timersvc_period(void)
{
  return 0xffffffffu;
}

/****************************************************************************
 * Name: rv32m1_timersvc_value
 ****************************************************************************/

uint32_t rv32m1_timersvc_value(void)
{
  /* Read High and Low Registers completely for the Right Result */

  uint64_t value = *(volatile uint64_t *)(RV32M1_TSTMR_BASE);

  /* It is ok to return the ONLY low valud caused the it is accumulated
   * outside.
   */

  return (uint32_t)value;
}
