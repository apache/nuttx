/****************************************************************************
 * arch/risc-v/src/common/riscv_cpuindex.c
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
#include <nuttx/arch.h>

#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

int up_cpu_index(void)
{
  int mhartid;

  asm volatile ("csrr %0, mhartid": "=r" (mhartid));
  return mhartid;
}
