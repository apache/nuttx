/****************************************************************************
 * arch/arm/src/pp502x/pp502x_cache.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pp502x_cache_init
 *
 * Description:
 *   Turn the CPU cache on: mask and operation word cleared, init mode, one
 *   status word per line cleared, CPU priority bit, the mask, the init
 *   operation, enable and run, and one read of every line of the first
 *   8 KiB of logical SDRAM, which is cached; the physical alias is not.
 *
 ****************************************************************************/

void pp502x_cache_init(void)
{
  uintptr_t addr;

  putreg32(0, PP502X_CACHE_MASK);
  putreg32(0, PP502X_CACHE_OPERATION);
  putreg32(PP502X_CACHE_CTL_INIT, PP502X_CACHE_CTL);

  for (addr = PP502X_CACHE_STATUS_CPU;
       addr < PP502X_CACHE_STATUS_CPU + PP502X_CACHE_SIZE;
       addr += PP502X_CACHE_LINESIZE)
    {
      putreg32(0, addr);
    }

  putreg32(getreg32(PP502X_CACHE_PRIORITY) | PP502X_CACHE_PRIORITY_CPU,
           PP502X_CACHE_PRIORITY);
  putreg32(PP502X_CACHE_MASK_CACHED, PP502X_CACHE_MASK);
  putreg32(PP502X_CACHE_OP_INIT, PP502X_CACHE_OPERATION);
  putreg32(PP502X_CACHE_CTL_ENABLE | PP502X_CACHE_CTL_RUN |
           PP502X_CACHE_CTL_INIT, PP502X_CACHE_CTL);

  for (addr = PP502X_SDRAM_LOGICAL;
       addr < PP502X_SDRAM_LOGICAL + PP502X_CACHE_SIZE;
       addr += PP502X_CACHE_LINESIZE)
    {
      (void)getreg8(addr);
    }
}
