/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_clockconfig.c
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

#include "arm_internal.h"
#include "mx8mp_ccm.h"
#include "mx8mp_clockconfig.h"
#include "hardware/mx8mp_gpc.h"
#include <arch/board/board.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_clockconfig
 *
 * Description:
 *   Called to initialize the i.MXRT.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void mx8mp_clockconfig(void)
{
  /* Ensure the clock of the peripherals used by M core not be affected by
   * A core which is running in a different domain.
   */

  mx8mp_ccm_gate_clock(CCM_IOMUX_CLK_GATE,  CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_IPMUX1_CLK_GATE, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_IPMUX2_CLK_GATE, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_IPMUX3_CLK_GATE, CLK_ALWAYS_NEEDED);

  mx8mp_ccm_gate_pll(SYSTEM_PLL1_CLK, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_pll(SYSTEM_PLL2_CLK, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_pll(SYSTEM_PLL3_CLK, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_pll(AUDIO_PLL1_CLK,  CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_pll(AUDIO_PLL2_CLK,  CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_pll(VIDEO_PLL_CLK,   CLK_ALWAYS_NEEDED);

  /* Set Cortex-M7 clock source to System PLL1 with no division (800MHz) */

  mx8mp_ccm_configure_clock(ARM_M7_CLK_ROOT, SYSTEM_PLL1_CLK, 1, 1);

  /* Add dependency on OCRAM and RDC module to the M7 */

  mx8mp_ccm_gate_clock(CCM_OCRAM_CLK_GATE,  CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_RDC_CLK_GATE,    CLK_ALWAYS_NEEDED);

  /* Enable Audio clock to power on the audiomix domain */

  mx8mp_ccm_gate_clock(CCM_AUDIO_CLK_GATE, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_enable_clock(AUDIO_AXI_CLK_ROOT);

  /* Make sure the M7 core could work normally when A53 core
   * enters the low power status.
   */

  mx8mp_ccm_gate_clock(CCM_OCRAM_CLK_GATE,      CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_RDC_CLK_GATE,        CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_SIM_M_CLK_GATE,      CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_SIM_MAIN_CLK_GATE,   CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_SIM_S_CLK_GATE,      CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_SIM_WAKEUP_CLK_GATE, CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_DEBUG_CLK_GATE,      CLK_ALWAYS_NEEDED);
  mx8mp_ccm_gate_clock(CCM_SEC_DEBUG_CLK_GATE,  CLK_ALWAYS_NEEDED);

  /* Make sure that main buses are enabled (TODO to be tuned or adjust
   * by configuration)
   */

  mx8mp_ccm_enable_clock(IPG_CLK_ROOT);
  mx8mp_ccm_enable_clock(AHB_CLK_ROOT);
  mx8mp_ccm_enable_clock(MAIN_AXI_CLK_ROOT);
  mx8mp_ccm_enable_clock(DRAM_ALT_CLK_ROOT);

  /* Power up the audiomix domain by M7 core. */

  /* Map the audiomix domain to M7 */

  modreg32(AUDIOMIX_DOMAIN,
           AUDIOMIX_DOMAIN,
           GPC_PGC_CPU_M7_MAPPING);

  /* Software request to trigger power up the domain */

  modreg32(AUDIOMIX_SW_PUP_REQ,
           AUDIOMIX_SW_PUP_REQ,
           GPC_PU_PGC_SW_PUP_REQ);

  /* Waiting the AUDIOMIX_SW_PUP_REQ bit to self-cleared after power up */

  while (getreg32(GPC_PU_PGC_SW_PUP_REQ) & AUDIOMIX_SW_PUP_REQ);

  /* Do the handshake to make sure the NOC bus ready after
   * power up the AUDIOMIX domain.
   */

  modreg32(GPC_AUDIOMIX_NOC_PWRDNREQN,
           GPC_AUDIOMIX_NOC_PWRDNREQN,
           GPC_PU_PWRHSK);
  while (!(getreg32(GPC_PU_PWRHSK) & GPC_AUDIOMIX_PWRDNACKN));

  /* Init Audio PLL and buses */

  /* AUDIO PLL1 to 393.216 MHz (multiple of 48kHz audio sampling rate) */

  mx8mp_ccm_configure_pll(CCM_ANALOG_AUDIO_PLL1, 262, 2, 3, 9437);

  /* AUDIO PLL2 to 361.2672 MHz (multiple of 44.1kHz audio sampling rate) */

  mx8mp_ccm_configure_pll(CCM_ANALOG_AUDIO_PLL2, 361, 3, 3, 17511);

  /* Audio AXI on PLL1 @ 800MHz */

  mx8mp_ccm_configure_clock(AUDIO_AXI_CLK_ROOT, SYSTEM_PLL1_CLK, 1, 1);

  /* Audio AHB on PLL1 @ 400MHz */

  mx8mp_ccm_configure_clock(AUDIO_AHB_CLK_ROOT, SYSTEM_PLL1_CLK, 1, 2);
}
