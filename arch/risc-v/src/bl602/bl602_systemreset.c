/****************************************************************************
 * arch/risc-v/src/bl602/bl602_systemreset.c
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
#include "hardware/bl602_glb.h"
#include "hardware/bl602_hbn.h"
#include "bl602_romapi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We choose to use ROM driver here.
 *
 * Because BL602 will reset the XIP Flash controller when performing
 * reset, this part of the code cannot be placed on the XIP Flash.
 */

#define bl602_romapi_reset_system ((void (*)(void))BL602_ROMAPI_RST_SYSTEM)
#define bl602_romapi_reset_cpu_sw ((void (*)(void))BL602_ROMAPI_RST_CPU_SW)
#define bl602_romapi_reset_por ((void (*)(void))BL602_ROMAPI_RST_POR)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  /* When perform reset before, MUST disable interrupt */

  asm volatile("csrci mstatus, 8");

  bl602_romapi_reset_system();

  /* Wait for the reset */

  for (; ; );
}

/****************************************************************************
 * Name: bl602_cpu_reset
 *
 * Description:
 *   Reset only the CPU
 *
 ****************************************************************************/

void bl602_cpu_reset(void)
{
  /* When perform reset before, MUST disable interrupt */

  asm volatile("csrci mstatus, 8");

  bl602_romapi_reset_cpu_sw();
}

/****************************************************************************
 * Name: bl602_por_reset
 *
 * Description:
 *   Trigger Power-on-Reset
 *
 ****************************************************************************/

void bl602_por_reset(void)
{
  /* When perform reset before, MUST disable interrupt */

  asm volatile("csrci mstatus, 8");

  bl602_romapi_reset_por();
}
