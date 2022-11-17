/****************************************************************************
 * arch/ceva/src/xm6/xm6_psu.c
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

#include <nuttx/irq.h>

#include "ceva_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* disable psu function temporily */

#define CEVAXM6_PSU_ENABLE 0

#if CEVAXM6_PSU_ENABLE

/* Core auto restore to DPS here after wakeup */

#define up_cpu_pmod(pmod_inst)                                        \
  __asm__ __volatile__                                                \
  (                                                                   \
    "nop #0x04\nnop\n"                                                \
    "movp %0.ui, moda.ui\n"       /* Enable the interrupt */          \
    pmod_inst                     /* Enter the low power mode */      \
    "nop #0x04\nnop #0x04\nnop\n" /* Clear the pipe of instruction */ \
    "movp %1.ui, moda.ui"         /* restore the interrupt */         \
     : : "r"(REG_MODA_ENABLE), "r"(REG_MODA_DISABLE)                  \
  )
#else
#define up_cpu_pmod(pmod_inst)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ceva_cpu_doze(void)
{
  up_cpu_pmod("psu {lightsleep}\n");
}

void ceva_cpu_idle(void)
{
  up_cpu_pmod("psu {standby}\n");
}
