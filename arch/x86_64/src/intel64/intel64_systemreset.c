/****************************************************************************
 * arch/x86_64/src/intel64/intel64_systemreset.c
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

#include <nuttx/arch.h>
#include <arch/io.h>

#include <stdint.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal, intel64 reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  uint8_t regval = (X86_RST_CNT_CPU_RST |
                    X86_RST_CNT_SYS_RST |
                    X86_RST_CNT_FULL_RST);

  /* Write to Reset Control Register */

  outb(regval, X86_RST_CNT_REG);

  while (1)
    {
      asm volatile("hlt");
    }
}
