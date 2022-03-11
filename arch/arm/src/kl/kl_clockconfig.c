/****************************************************************************
 * arch/arm/src/kl/kl_clockconfig.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "kl_gpio.h"
#include "hardware/kl_mcg.h"
#include "hardware/kl_sim.h"
#include "hardware/kl_osc.h"
#include "hardware/kl_fmc.h"
#include "hardware/kl_llwu.h"
#include "hardware/kl_pinmux.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_portclocks
 *
 * Description:
 *   Enable all of the port clocks
 *
 ****************************************************************************/

static inline void kl_portclocks(void)
{
  uint32_t regval;

  /* Enable all of the port clocks */

  regval = getreg32(KL_SIM_SCGC5);
  regval |= (SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC |
             SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);
  putreg32(regval, KL_SIM_SCGC5);
}

/****************************************************************************
 * Name: kl_pllconfig
 *
 * Description:
 *   Initialize the PLL using the settings in board.h.  This assumes that
 *   the MCG is in default FLL Engaged Internal (FEI mode) out of reset.
 *
 ****************************************************************************/

void kl_pllconfig(void)
{
  uint32_t regval32;
  uint8_t regval8;

  /* Enable clock gate to Port A module to enable pin routing (PORTA=1) */

  regval32  = getreg32(KL_SIM_SCGC5);
  regval32 |= SIM_SCGC5_PORTA;
  putreg32(regval32, KL_SIM_SCGC5);

  /* Divide-by-2 for clock 1 and clock 4. OUTDIV1 and OUTDIV4 determined by
   * settings in the board.h header file.
   */

  regval32 = (SIM_CLKDIV1_OUTDIV1(BOARD_OUTDIV1) |
              SIM_CLKDIV1_OUTDIV4(BOARD_OUTDIV4));
  putreg32(regval32, KL_SIM_CLKDIV1);

  /* System oscillator drives 32 kHz clock for various peripherals
   * (OSC32KSEL=0)
   */

  regval32  = getreg32(KL_SIM_SOPT1);
  regval32 &= ~(SIM_SOPT1_OSC32KSEL_MASK);
  putreg32(regval32, KL_SIM_SOPT1);

  /* Select PLL as a clock source for various peripherals (PLLFLLSEL=1)
   * Clock source for TPM counter clock is MCGFLLCLK or MCGPLLCLK/2
   */

  regval32  = getreg32(KL_SIM_SOPT2);
  regval32 |= SIM_SOPT2_PLLFLLSEL;
  putreg32(regval32, KL_SIM_SOPT2);

  regval32  = (regval32 & ~(SIM_SOPT2_TPMSRC_OCSERCLK)) |
               SIM_SOPT2_TPMSRC_MCGCLK;
  putreg32(regval32, KL_SIM_SOPT2);

  /* PORTA_PCR18: ISF=0, MUX=0 */

  /* PORTA_PCR19: ISF=0, MUX=0 */

  regval32  = getreg32(KL_PORTA_PCR18);
  regval32 &= ~(PORT_PCR_ISF | PORT_PCR_MUX_ALT7);
  putreg32(regval32, KL_PORTA_PCR18);

  regval32  = getreg32(KL_PORTA_PCR19);
  regval32 &= ~(PORT_PCR_ISF | PORT_PCR_MUX_ALT7);
  putreg32(regval32, KL_PORTA_PCR19);

  /* Switch to FBE Mode */

  /* OSC0_CR: ERCLKEN=0,
   * ??=0, EREFSTEN=0, ??=0, SC2P=0, SC4P=0, SC8P=0, SC16P=0
   */

  putreg8(0, KL_OSC_CR);

  /* MCG_C2: LOCRE0=0, ??=0, RANGE0=2, HGO0=0, EREFS0=1, LP=0, IRCS=0 */

  regval8 = (MCG_C2_RANGE_VHIGH | MCG_C2_EREFS);
  putreg8(regval8, KL_MCG_C2);

  /* MCG_C1: CLKS=2, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0 */

  regval8 = (MCG_C1_CLKS_EXTREF | MCG_C1_FRDIV_R0DIV8);
  putreg8(regval8, KL_MCG_C1);

  /* MCG_C4: DMX32=0, DRST_DRS=0 */

  regval8  = getreg8(KL_MCG_C4);
  regval8 &= ~(MCG_C4_DMX32 | MCG_C4_DRST_DRS_MASK);
  putreg8(regval8, KL_MCG_C4);

  /* MCG_C5: ??=0, PLLCLKEN0=0, PLLSTEN0=0, PRDIV0 determined by board
   * settings in the board.h header file.
   */

  regval8 = MCG_C5_PRDIV(BOARD_PRDIV0);
  putreg8(regval8, KL_MCG_C5);

  /* MCG_C6: LOLIE0=0, PLLS=0, CME0=0, VDIV0 determined by board
   * settings in the board.h header file.
   */

  putreg8(MCG_C6_VDIV(BOARD_VDIV0), KL_MCG_C6);

  /* Check that the source of the FLL reference clock is the external
   * reference clock.
   */

  while ((getreg8(KL_MCG_S) & MCG_S_IREFST) != 0)
    ;

  /* Wait until external reference */

  while ((getreg8(KL_MCG_S) & MCG_S_CLKST_MASK) != 8)
    ;

  /* Switch to PBE mode.
   * Select PLL as MCG source (PLLS=1)
   */

  putreg8(MCG_C6_PLLS, KL_MCG_C6);

  /* Wait until PLL locked */

  while ((getreg8(KL_MCG_S) & MCG_S_LOCK) == 0)
    ;

  /* Switch to PEE mode
   * Select PLL output (CLKS=0)
   * FLL external reference divider (FRDIV=3)
   * External reference clock for FLL (IREFS=0)
   */

  putreg8(MCG_C1_FRDIV_R0DIV8, KL_MCG_C1);

  /* Wait until PLL output */

  while ((getreg8(KL_MCG_S) & MCG_S_CLKST_MASK) != 0x0c);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_clockconfig
 *
 * Description:
 *   Called to initialize the Kinetis chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void kl_clockconfig(void)
{
  /* Enable all of the port clocks */

  kl_portclocks();

  /* Configure the PLL based on settings in the board.h file */

  kl_pllconfig();

  /* For debugging, we will normally want to enable the trace clock and/or
   * the FlexBus clock.
   */

  /* kl_traceconfig(); */

  /* kl_fbconfig(); */
}
