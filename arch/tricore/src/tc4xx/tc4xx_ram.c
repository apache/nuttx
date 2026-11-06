/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_ram.c
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

#include <sys/param.h>

#include "tricore_internal.h"
#include "hardware/tc4xx_vmt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SRAMs that the boot firmware initializes after a power-on reset */

#define TC4XX_UCB0_RAMINIT          0xae408814
#define TC4XX_RAMIN_MASK            (0x3 << 0)  /* 0: warm and cold PORST */
#define TC4XX_RAMINSEL_CPU0         (1 << 16)   /* Set: not the CPU0 SRAMs */
#define TC4XX_LMUINSEL_CPU0         (1 << 24)   /* Set: not the CPU0 DLMU */

/* SSH (MBIST controller) selectors of the DSPR and the DLMU of CPU0 */

#define TC4XX_SSH_CPU0_DSPR         0
#define TC4XX_SSH_CPU0_DLMU         4

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SSH selectors of the LMU0-9 and MCAN0-4 RAMs, which the boot firmware
 * never initializes
 */

static const uint8_t g_tc4xx_sram[] =
{
  0x17, 0x18, 0x20, 0x21, 0x19, 0x1a, 0x1b, 0x22, 0x23, 0x24, /* LMU0-9 */
  0xa2, 0xa3, 0xa4, 0xa5, 0xa6                                /* MCAN0-4 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_sram_clear
 *
 * Description:
 *   Fill a whole SRAM with ECC correct zeros through its SSH, with
 *   MCONTROL.SRAM_CLR, as IfxVmt_clearSram does.
 *
 ****************************************************************************/

static void tc4xx_sram_clear(uint32_t sel)
{
  uintptr_t vmt = TC4XX_VMT_BASE + (sel >> 5) * TC4XX_VMT_SIZE;
  uintptr_t mc = vmt + TC4XX_VMT_MC(sel % 32);
  uint32_t mask = 1 << (sel % 32);

  if ((getreg32(vmt + TC4XX_VMT_CLC) & TC4XX_VMT_CLC_DISS) != 0)
    {
      putreg32(0, vmt + TC4XX_VMT_CLC);
    }

  /* Enable the MBIST shell and start the clear */

  putreg32(getreg32(mc + TC4XX_VMT_MC_ALMSRCS) & ~TC4XX_ALMSRCS_ALARMS,
           mc + TC4XX_VMT_MC_ALMSRCS);
  putreg32(getreg32(vmt + TC4XX_VMT_MEMTEST) | mask,
           vmt + TC4XX_VMT_MEMTEST);
  while ((getreg32(vmt + TC4XX_VMT_MEMSTAT) & mask) != 0);

  putreg32((1 << 15) | (1 << 0), mc + TC4XX_VMT_MC_MCONTROL);
  putreg32(getreg32(mc + TC4XX_VMT_MC_MCONTROL) & ~(1 << 0),
           mc + TC4XX_VMT_MC_MCONTROL);
  while ((getreg32(vmt + TC4XX_VMT_MEMDONE) & mask) == 0);

  /* Read address 0 once, then disable the MBIST shell again */

  putreg32(1 << 3, mc + TC4XX_VMT_MC_MCONTROL);
  putreg32((1 << 12) | (1 << 0), mc + TC4XX_VMT_MC_CONFIG0);
  putreg32(0, mc + TC4XX_VMT_MC_CONFIG1);
  putreg32(0, mc + TC4XX_VMT_MC_RANGE);
  putreg32((1 << 3) | (1 << 0), mc + TC4XX_VMT_MC_MCONTROL);
  putreg32(1 << 3, mc + TC4XX_VMT_MC_MCONTROL);
  while ((getreg32(vmt + TC4XX_VMT_MEMDONE) & mask) == 0);

  putreg32(getreg32(vmt + TC4XX_VMT_MEMTEST) & ~mask,
           vmt + TC4XX_VMT_MEMTEST);
  while ((getreg32(vmt + TC4XX_VMT_MEMSTAT) & mask) != 0);
  putreg32(getreg32(mc + TC4XX_VMT_MC_ALMSRCS) | TC4XX_ALMSRCS_ALARMS,
           mc + TC4XX_VMT_MC_ALMSRCS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_ram_init
 *
 * Description:
 *   Initialize the RAMs that the boot firmware leaves uninitialized, so
 *   that they can be read before they are written: the DSPR and the DLMU of
 *   CPU0 unless UCB0_RAMINIT has the boot firmware initialize them after
 *   every power-on reset, and the LMU and MCAN RAMs.  tricore_head calls it
 *   on the boot stack and CSAs in the PSPR, before the CSA list in the DSPR
 *   and before .data and .bss.
 *
 ****************************************************************************/

void tricore_ram_init(void)
{
  uint32_t raminit = getreg32(TC4XX_UCB0_RAMINIT);
  uint32_t partial = raminit & TC4XX_RAMIN_MASK;
  int i;

  if (partial != 0 || (raminit & TC4XX_RAMINSEL_CPU0) != 0)
    {
      tc4xx_sram_clear(TC4XX_SSH_CPU0_DSPR);
    }

  if (partial != 0 || (raminit & TC4XX_LMUINSEL_CPU0) != 0)
    {
      tc4xx_sram_clear(TC4XX_SSH_CPU0_DLMU);
    }

  for (i = 0; i < nitems(g_tc4xx_sram); i++)
    {
      tc4xx_sram_clear(g_tc4xx_sram[i]);
    }
}
