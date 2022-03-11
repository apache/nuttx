/****************************************************************************
 * boards/arm/stm32/shenzhou/src/stm32_chipid.c
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

#include <stdio.h>

#include <arch/board/board.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const char *stm32_getchipid(void)
{
  static char cpuid[12];
  int i;

  for (i = 0; i < 12; i++)
    {
      cpuid[i] = getreg8(0x1ffff7e8 + i);
    }

  return cpuid;
}

const char *stm32_getchipid_string(void)
{
  static char cpuid[27];
  int c;
  int i;

  for (i = 0, c = 0; i < 12; i++)
    {
      sprintf(&cpuid[c], "%02X", getreg8(0x1ffff7e8 + 11 - i));
      c += 2;
      if (i % 4 == 3)
        {
          cpuid[c++] = '-';
        }
    }

  cpuid[26] = '\0';
  return cpuid;
}
