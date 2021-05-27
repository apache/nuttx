/****************************************************************************
 * arch/misoc/src/minerva/minerva_flushcache.c
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

#include "chip.h"
#include "minerva.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: minerva_flush_dcache
 *
 * Description:
 *   Flush the data cache of the cpu
 *
 ****************************************************************************/

void minerva_flush_dcache(void)
{
#warning "FIX D CACHE FLUSH"
  asm volatile ("nop\n" "nop\n" "nop\n" "nop\n");
}

/****************************************************************************
 * Name: minerva_flush_icache
 *
 * Description:
 *   Flush the instruction cache of the cpu
 *
 ****************************************************************************/

void minerva_flush_icache(void)
{
  asm volatile ("fence.i\n" "nop\n" "nop\n" "nop\n" "nop\n");
}
