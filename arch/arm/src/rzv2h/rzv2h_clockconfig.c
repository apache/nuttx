/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_clockconfig.c
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

#include "rzv2h_clockconfig.h"
#include <bsp_api.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_clockconfig
 ****************************************************************************/

/* Confirmed on hardware: bsp_clock_init() alone is enough for xSPI
 * DirectTransfer (reads/WREN/status/program). The CM33 PD_XSPI_ON
 * reset/clock/TZC/master-access sequence is not needed since the
 * memory-mapped write window is never used -- and driving it from a
 * CR8-only boot has been observed to break DirectTransfer instead
 * (asynchronous external abort). Leave as-is until proven otherwise.
 */

void rzv2h_clockconfig(void)
{
  bsp_clock_init();
}
