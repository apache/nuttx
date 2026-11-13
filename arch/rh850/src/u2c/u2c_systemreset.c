/****************************************************************************
 * arch/rh850/src/u2c/u2c_systemreset.c
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

#include <nuttx/arch.h>

#include "rh850_internal.h"
#include "hardware/u2c_sysctrl.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Software system reset (System Reset 2) through SWSRESA, which needs the
 *   RESKCPROT0 key.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  up_irq_save();
  putreg32(U2C_KCPROT_ENABLE, U2C_RESKCPROT0);
  putreg32(U2C_SWSRESA_TRIGGER, U2C_SWSRESA);

  for (; ; )
    {
    }
}
