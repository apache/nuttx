/****************************************************************************
 * arch/x86_64/src/intel64/intel64_systempoweroff.c
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
#include <arch/acpi.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systempoweroff
 *
 * Description:
 *   Internal, intel64 poweroff logic.
 *
 ****************************************************************************/

void up_systempoweroff(void)
{
  uint32_t pm1a_cnt = 0;
  uint32_t pm1b_cnt = 0;
  uint32_t regvala  = 0;
  uint32_t regvalb  = 0;

  acpi_poweroff_param_get(&pm1a_cnt, &pm1b_cnt, &regvala, &regvalb);

  /* Write to Poweroff Control Register */

  outw(regvala | 0x2000, pm1a_cnt);
  if (pm1b_cnt != 0)
    {
      outw(regvalb | 0x2000, pm1b_cnt);
    }

  while (1);
}
