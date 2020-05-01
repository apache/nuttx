/****************************************************************************
 * arch/arm/src/armv8-m/arm_memfault.c
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

#include <arch/irq.h>

#include "arm_arch.h"
#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MEMFAULT
# define mferr(format, ...)  _alert(format, ##__VA_ARGS__)
# define mfinfo(format, ...) _alert(format, ##__VA_ARGS__)
#else
# define mferr(x...)
# define mfinfo(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_memfault
 *
 * Description:
 *   This is Memory Management Fault exception handler.  Normally we get
 *   here when the Cortex M3 MPU is enabled and an MPU fault is detected.
 *   However, I understand that there are other error conditions that can
 *   also generate memory management faults.
 *
 ****************************************************************************/

int arm_memfault(int irq, FAR void *context, FAR void *arg)
{
  /* Dump some memory management fault info */

  up_irq_save();
  _alert("PANIC!!! Memory Management Fault:\n");
  mfinfo("  IRQ: %d context: %p\n", irq, context);
  _alert("  CFAULTS: %08x MMFAR: %08x\n",
        getreg32(NVIC_CFAULTS), getreg32(NVIC_MEMMANAGE_ADDR));
  mfinfo("  BASEPRI: %08x PRIMASK: %08x IPSR: %08x CONTROL: %08x\n",
         getbasepri(), getprimask(), getipsr(), getcontrol());

  PANIC();
  return OK; /* Won't get here */
}
