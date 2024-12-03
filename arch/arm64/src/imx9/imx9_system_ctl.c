/****************************************************************************
 * arch/arm64/src/imx9/imx9_system_ctl.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/chip/chip.h>
#include "imx9_system_ctl.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imx9_mix_powerup(void)
{
  uint32_t val = 0;

  /* Authen ctrl, enable NS access to slice registers */

  modifyreg32(IMX9_SRC_MEDIA_SLICE_BASE +
              SRC_SLICE_AUTHEN_CTRL_OFFSET, 0, BIT(9));
  modifyreg32(IMX9_SRC_ML_SLICE_BASE +
              SRC_SLICE_AUTHEN_CTRL_OFFSET, 0, BIT(9));

  /* Enable s400 handsake */

  modifyreg32(IMX9_BLK_CTRL_S_AONMIX2_BASE +
              AON_MIX_LP_HANDSAKE, 0, BIT(13));

  /* Counter mode */

  modifyreg32(IMX9_SRC_MEDIA_SLICE_BASE +
              SRC_SLICE_PSW_ACK_CTRL0_OFFSET, BIT(28), BIT(29));
  modifyreg32(IMX9_SRC_ML_SLICE_BASE +
              SRC_SLICE_PSW_ACK_CTRL0_OFFSET, BIT(28), BIT(29));

  /* release media and ml from reset */

  modifyreg32(IMX9_SRC_GENERAL_REG_BASE +
              SRC_CTRL_OFFSET, 0, (BIT(4) | BIT(5)));

  /* Enable mem in Low power auto sequence */

  modifyreg32(IMX9_SRC_MEDIA_MEM_BASE + MEM_CTRL_OFFSET, 0, BIT(2));
  modifyreg32(IMX9_SRC_ML_MEM_BASE + MEM_CTRL_OFFSET, 0, BIT(2));

  /* Mediamix powerdown */

  modifyreg32(IMX9_SRC_MEDIA_SLICE_BASE + SRC_SLICE_SW_CTRL_OFFSET,
              0, BIT(31));

  val = getreg32(IMX9_SRC_MEDIA_SLICE_BASE + SRC_SLICE_FUNC_STAT_OFFSET);
  if (val & 1)
    {
      while (!(val & BIT(12)))
        {
          val = getreg32(IMX9_SRC_MEDIA_SLICE_BASE +
                         SRC_SLICE_FUNC_STAT_OFFSET);
        }

      up_udelay(1);
    }
  else
    {
      while (!(val & BIT(0)))
        {
          val = getreg32(IMX9_SRC_MEDIA_SLICE_BASE +
                         SRC_SLICE_FUNC_STAT_OFFSET);
        }
    }

  /* Power on */

  modifyreg32(IMX9_SRC_MEDIA_SLICE_BASE +
              SRC_SLICE_SW_CTRL_OFFSET, BIT(31), 0);
  while (!(val & BIT(4)))
    {
      val = getreg32(IMX9_SRC_MEDIA_SLICE_BASE + SRC_SLICE_FUNC_STAT_OFFSET);
    }

  /* ML powerdown */

  modifyreg32(IMX9_SRC_ML_SLICE_BASE + SRC_SLICE_SW_CTRL_OFFSET, 0, BIT(31));

  val = getreg32(IMX9_SRC_ML_SLICE_BASE + SRC_SLICE_FUNC_STAT_OFFSET);
  if (val & 1)
    {
      while (!(val & BIT(12)))
        {
          val = getreg32(IMX9_SRC_ML_SLICE_BASE +
                         SRC_SLICE_FUNC_STAT_OFFSET);
        }

      up_udelay(1);
    }
  else
    {
      while (!(val & BIT(0)))
        {
          val = getreg32(IMX9_SRC_ML_SLICE_BASE +
                         SRC_SLICE_FUNC_STAT_OFFSET);
        }
    }

  /* Power on */

  modifyreg32(IMX9_SRC_ML_SLICE_BASE + SRC_SLICE_SW_CTRL_OFFSET, BIT(31), 0);
  while (!(val & BIT(4)))
    {
      val = getreg32(IMX9_SRC_ML_SLICE_BASE + SRC_SLICE_FUNC_STAT_OFFSET);
    }

  /* Disable isolation usb, dsi, csi */

  putreg32(0, IMX9_SRC_GENERAL_REG_BASE + SRC_SP_ISO_CTRL_OFFSET);
}

