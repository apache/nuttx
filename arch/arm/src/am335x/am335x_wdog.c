/****************************************************************************
 * arch/arm/src/am335x/am335x_wdog.c
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

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/am335x_wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_wdog_disable
 *
 * Description:
 *   Called at the very beginning of _start.  Disables all watchdogs
 *
 ****************************************************************************/

void am335x_wdog_disable_all(void)
{
  putreg32(WDT_SPR_STOP_FEED_A, AM335X_WDT_SPR);
  while ((getreg32(AM335X_WDT_WPS) & WDT_WPS_W_PEND_WSPR) != 0)
    {
    }

  putreg32(WDT_SPR_STOP_FEED_B, AM335X_WDT_SPR);
  while ((getreg32(AM335X_WDT_WPS) & WDT_WPS_W_PEND_WSPR) != 0)
    {
    }
}
