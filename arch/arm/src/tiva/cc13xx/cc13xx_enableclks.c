/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_enableclks.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "hardware/tiva_prcm.h"
#include "tiva_enableclks.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cc13xx_periph_enableclks
 *
 * Description:
 *   Enable clocking in the selected modes for this peripheral.
 *
 ****************************************************************************/

void cc13xx_periph_enableclk(uint32_t peripheral, uint32_t modeset)
{
  DEBUGASSERT(modeset != 0);

  if ((modeset & CC13XX_RUNMODE_CLOCK) != 0)
    {
      prcm_periph_runenable(peripheral);
    }

  if ((modeset & CC13XX_SLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_sleepenable(peripheral);
    }

  if ((modeset & CC13XX_DEEPSLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_deepsleepenable(peripheral);
    }

  prcm_load_set();
  while (!prcm_load_get())
    {
    }
}

/****************************************************************************
 * Name:  cc13xx_periph_disableclk
 *
 * Description:
 *   Disable clocking in the selected modes for this peripheral.
 *
 ****************************************************************************/

void cc13xx_periph_disableclk(uint32_t peripheral, uint32_t modeset)
{
  DEBUGASSERT(modeset != 0);

  if ((modeset & CC13XX_RUNMODE_CLOCK) != 0)
    {
      prcm_periph_rundisable(peripheral);
    }

  if ((modeset & CC13XX_SLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_sleepdisable(peripheral);
    }

  if ((modeset & CC13XX_DEEPSLEEPMODE_CLOCK) != 0)
    {
      prcm_periph_deepsleepdisable(peripheral);
    }

  prcm_load_set();
}
