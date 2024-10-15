/****************************************************************************
 * arch/arm/src/armv8-m/arm_dumpnvic.c
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
#include <nuttx/coredump.h>
#include <sys/types.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "nvic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARM_COREDUMP_REGION

/****************************************************************************
 * Function:  arm_coredump_add_region
 *
 * Description:
 *   Dump all NVIC registers during a core dump.
 *
 ****************************************************************************/

void arm_coredump_add_region(void)
{
  coredump_add_memory_region((uint32_t *)ARMV8M_NVIC_BASE,
                             NVIC_CID3 + 4 - ARMV8M_NVIC_BASE,
                             PF_REGISTER);
}

#endif /* CONFIG_ARM_COREDUMP_REGION */
