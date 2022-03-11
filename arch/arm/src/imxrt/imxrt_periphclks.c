/****************************************************************************
 * arch/arm/src/imxrt/imxrt_periphclks.c
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

#include "arm_internal.h"
#include "imxrt_periphclks.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_periphclk_configure
 *
 * Description:
 *   Configure a peripheral clock by modifying the appropriate field in the
 *   appropriate CCRGR register.
 *
 * Input Parameters:
 *   regaddr - The CCMD CCGR register to be modified
 *   index   - The index of the field to be modified
 *   value   - The new value of the field
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void imxrt_periphclk_configure(uintptr_t regaddr, unsigned int index,
                               unsigned int value)
{
  modifyreg32(regaddr, CCM_CCGRX_CG_MASK(index), CCM_CCGRX_CG(index, value));
}
