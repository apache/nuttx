/****************************************************************************
 * arch/risc-v/src/k210/k210_fpioa.c
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

#include <assert.h>
#include <debug.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "k210_memorymap.h"
#include "k210_fpioa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void k210_fpioa_config(uint32_t io, uint32_t ioflags)
{
  uint32_t *fpioa = (uint32_t *)K210_FPIOA_BASE;
  DEBUGASSERT(io < K210_IO_NUMBER);
  putreg32(ioflags, &fpioa[io]);
}
