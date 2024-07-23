/****************************************************************************
 * arch/tricore/src/common/tricore_registerdump.c
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
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void *regs)
{
  uintptr_t *csa = regs;

  while (((uintptr_t)csa & PCXI_UL) == 0)
    {
      csa = tricore_csa2addr((uintptr_t)csa);
      csa = (uintptr_t *)csa[0];
    }

  csa = tricore_csa2addr((uintptr_t)csa);

  return csa[REG_SP];
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
}
