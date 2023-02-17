/****************************************************************************
 * arch/ceva/src/xc5/xc5_psu.c
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

/* disable psu function temporarily */

#define CONFIG_XC5_PSU_ENABLE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_XC5_PSU_ENABLE
static void up_cpu_pmod(uint32_t psvm)
{
  /* Core auto restore to DPS here after wakeup */

  __asm__ __volatile__
  (
    "mov #0x2,    mod2\n"
    "mov #0x3f80, modp\n"      /* Enable the interrupt */
    "mov %0,   r0\n"           /* Enter the low power mode */
    "mov #0x250,  r1\n"
    "out {dw,cpm} r0, (r1)\n"  /* output to cpm register psmv */
    "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
    "mov #0x0080, modp"        /* restore the interrupt */
    : : "r"(psvm)
  );
}
#else
static void up_cpu_pmod(uint32_t psvm)
{
}
#endif /* CONFIG_XC5_PSU_ENABLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_psu_lp(int value)
{
  up_cpu_pmod(value);
}
