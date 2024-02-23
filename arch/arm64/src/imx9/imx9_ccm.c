/****************************************************************************
 * arch/arm64/src/imx9/imx9_ccm.c
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
#include <stdbool.h>

#include <sys/param.h>
#include <sys/types.h>

#include "barriers.h"

#include "arm64_internal.h"
#include "imx9_ccm.h"

#include "hardware/imx9_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define mb()       \
  do               \
    {              \
      ARM64_DSB(); \
      ARM64_ISB(); \
    }              \
  while (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_ccm_configure_root_clock
 *
 * Description:
 *   Change root clock source and divider. Leaves the clock running state
 *   unaltered.
 *
 * Input Parameters:
 *   root - The root clock index.
 *   src  - The root clock MUX source.
 *   div  - The root clock divider.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_configure_root_clock(int root, int src, uint32_t div)
{
  uint32_t value;
  int i;

  if (root >= CCM_CR_COUNT || div == 0 || div > 255)
    {
      return -EINVAL;
    }

  /* Find the corresponding MUX register value for root and source */

  for (i = 0; i < ROOT_MUX_MAX; i++)
    {
      if (g_ccm_root_mux[root][i] == src)
        {
          break;
        }
    }

  if (i == ROOT_MUX_MAX)
    {
      return -EINVAL;
    }

  /* Set the new value */

  value = CCM_CR_CTRL_MUX_SRCSEL(i) | CCM_CR_CTRL_DIV(div);
  putreg32(value, IMX9_CCM_CR_CTRL(root));
  mb();

  /* Wait for the clock state change */

  while (getreg32(IMX9_CCM_CR_STAT0(root)) & CCM_CR_STAT0_CHANGING);

  return OK;
}

/****************************************************************************
 * Name: imx9_ccm_root_clock_on
 *
 * Description:
 *   Enable / disable root clock.
 *
 * Input Parameters:
 *   root    - The root clock index.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_root_clock_on(int root, bool enabled)
{
  if (root >= CCM_CR_COUNT)
    {
      return -EINVAL;
    }

  if (enabled)
    {
      putreg32(CCM_CR_CTRL_OFF, IMX9_CCM_CR_CTRL_CLR(root));
    }
  else
    {
      putreg32(CCM_CR_CTRL_OFF, IMX9_CCM_CR_CTRL_SET(root));
    }

  mb();

  /* Wait for the clock state change */

  while (getreg32(IMX9_CCM_CR_STAT0(root)) & CCM_CR_STAT0_CHANGING);

  return OK;
}

/****************************************************************************
 * Name: imx9_ccm_gate_on
 *
 * Description:
 *   Enable / disable clock.
 *
 * Input Parameters:
 *   gate    - The clock gate index.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_gate_on(int gate, bool enabled)
{
  uint32_t value;

  if (gate >= CCM_LPCG_COUNT)
    {
      return -EINVAL;
    }

  /* Make sure direct mode is on, which is what we support */

  value = getreg32(IMX9_CCM_LPCG_AUTH(gate));
  if (value & CCM_LPCG_AUTH_CPULPM)
    {
      value &= ~CCM_LPCG_AUTH_CPULPM;
      putreg32(value, IMX9_CCM_LPCG_AUTH(gate));
      mb();
    }

  value = enabled ? 1 : 0;
  putreg32(value, IMX9_CCM_LPCG_DIR(gate));
  mb();

  /* Wait for the clock state change */

  while ((getreg32(IMX9_CCM_LPCG_STAT0(gate)) & CCM_LPCG_STAT0_ON) != value);

  return OK;
}
