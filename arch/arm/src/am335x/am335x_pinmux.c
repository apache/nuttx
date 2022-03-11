/****************************************************************************
 * arch/arm/src/am335x/am335x_pinmux.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include "arm_internal.h"
#include "am335x_pinmux.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_pinmux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int am335x_pinmux_configure(uintptr_t padctl, pinmux_pinset_t muxset)
{
  uint32_t regval = 0;
  uint32_t value;

  /* Select mux mode */

  value = (muxset & PINMUX_MODE_MASK) >> PINMUX_MODE_SHIFT;
  regval |= PADCTL_MUXMODE(value);

  /* Select pull up/down type */

  if ((muxset & PINMUX_PULL_TYPE_UP) != 0)
    {
      regval |= PADCTL_PULLUP_EN;
    }

  /* Select pull up/down enable */

  if ((muxset & PINMUX_PULL_UP_DISABLE) != 0)
    {
      regval |= PADCTL_PULLUDDIS;
    }

  /* Select receive enable */

  if ((muxset & PINMUX_RX_ENABLE) != 0)
    {
      regval |= PADCTL_RXACTIVE;
    }

  /* Select slow/fast slew rate */

  if ((muxset & PINMUX_SLEW_SLOW) != 0)
    {
      regval |= PADCTL_SLEWCTRL;
    }

  /* Write the result to the specified Pad Control register */

  putreg32(regval, padctl);
  return OK;
}
