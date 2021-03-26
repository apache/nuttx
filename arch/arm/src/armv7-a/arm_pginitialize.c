/****************************************************************************
 * arch/arm/src/armv7-a/arm_pginitialize.c
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

#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/page.h>

#include "arm_internal.h"

#ifdef CONFIG_PAGING

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pginitialize()
 *
 * Description:
 *  Initialize the MMU for on-demand paging support..
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.  This function will crash if any errors are detected during MMU
 *   initialization
 *
 * Assumptions:
 *   - Called early in the platform initialization sequence so that no
 *     special concurrency protection is required.
 *
 ****************************************************************************/

void arm_pginitialize(void)
{
  /* None needed at present.  This file is just retained in case the need
   * arises in the future.  Nothing calls arm_pginitialize() now.  If needed,
   * if should be called early in arm_boot.c to assure that all paging is
   * ready.
   */
}

#endif /* CONFIG_PAGING */
