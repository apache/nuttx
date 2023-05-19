/****************************************************************************
 * arch/arm/src/kinetis/kinetis_clockconfig.c
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
#include "kinetis.h"
#include "hardware/kinetis_mcg.h"
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_fmc.h"
#include "hardware/kinetis_pmc.h"
#include "hardware/kinetis_llwu.h"
#include "hardware/kinetis_pinmux.h"
#include "hardware/kinetis_osc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_RAMFUNCS
# error "CONFIG_ARCH_RAMFUNCS must be defined for this logic"
#endif

/* A board may provide an override for BOARD_FRDIV */

#if !defined(BOARD_FRDIV)
#  define BOARD_FRDIV               MCG_C1_FRDIV_DIV256
#endif

/* A board may provide BOARD_MCG_C2 with all the MCG_C2 setting
 * or use individual setting with 0 defaults
 */

#if !defined(BOARD_MCG_C2)

/* A board may provide an override for BOARD_EXTCLOCK_MCG_C2 */

#  if defined(BOARD_EXTCLOCK)
#    if defined(BOARD_EXTCLOCK_MCG_C2)
#      define EXTCLOCK_MCG_C2   BOARD_EXTCLOCK_MCG_C2
#    else
#      define EXTCLOCK_MCG_C2   0
#    endif
#  endif

/* A board may provide BOARD_EXTAL_LP to not choose MCG_C2_HGO */

#  if defined(BOARD_EXTAL_LP)
#    define BOARD_MGC_C2_HGO        0  /* Do not use MCG_C2_HGO */
#  else
#    if !defined(KINETIS_MCG_HAS_C2_HGO)
#      error BOARD_EXTAL_LP is not defined and MCG_C2_HGO is not supported on this SoC!
#    else
#      define BOARD_MGC_C2_HGO      MCG_C2_HGO
#    endif
#  endif

/* A board must provide BOARD_MCG_C2_FCFTRIM when SoC has the setting */

#  if defined(KINETIS_MCG_HAS_C2_FCFTRIM) && !defined(BOARD_MCG_C2_FCFTRIM)
#    error MCG_C2_FCFTRIM is supported on this SoC and BOARD_MCG_C2_FCFTRIM is not defined!
#  endif

#  if !defined(KINETIS_MCG_HAS_C2_FCFTRIM) && defined(BOARD_MCG_C2_FCFTRIM)
#    error BOARD_MCG_C2_FCFTRIM is defined but MCG_C2_FCFTRIM is not supported on this SoC!
#  endif

/* Provide the 0 default */

#  if !defined(KINETIS_MCG_HAS_C2_FCFTRIM) && !defined(BOARD_MCG_C2_FCFTRIM)
#    define BOARD_MCG_C2_FCFTRIM 0
#  endif

/* A board must provide BOARD_MCG_C2_LOCRE0 when SoC has the setting */

#  if defined(KINETIS_MCG_HAS_C2_LOCRE0) && !defined(BOARD_MCG_C2_LOCRE0)
#    error MCG_C2_LOCRE0 is supported on this SoC and BOARD_MCG_C2_LOCRE0 is not defined!
#  endif

#  if !defined(KINETIS_MCG_HAS_C2_LOCRE0) && defined(BOARD_MCG_C2_LOCRE0)
#    error BOARD_MCG_C2_LOCRE0 is defined but MCG_C2_LOCRE0 is not supported on this SoC!
#  endif

/* Provide the 0 default */

#  if !defined(KINETIS_MCG_HAS_C2_LOCRE0) && !defined(BOARD_MCG_C2_LOCRE0)
#    define BOARD_MCG_C2_LOCRE0 0
#  endif
#endif /* !defined(BOARD_MCG_C2) */

/* Do some sanity checking */

#if BOARD_PRDIV > KINETIS_MCG_C5_PRDIV_MAX || \
    BOARD_PRDIV < KINETIS_MCG_C5_PRDIV_BASE
#  error BOARD_PRDIV must satisfy KINETIS_MCG_C5_PRDIV_BASE >= \
         BOARD_VDIV <= KINETIS_MCG_C5_PRDIV_MAX
#endif

#if BOARD_VDIV > KINETIS_MCG_C6_VDIV_MAX || \
    BOARD_VDIV < KINETIS_MCG_C6_VDIV_BASE
#  error BOARD_VDIV must satisfy KINETIS_MCG_C6_VDIV_BASE >= \
         BOARD_VDIV <= KINETIS_MCG_C6_VDIV_MAX
#endif

#if BOARD_PLLIN_FREQ < KINETIS_MCG_PLL_REF_MIN || \
    BOARD_PLLIN_FREQ > KINETIS_MCG_PLL_REF_MAX
#  error BOARD_PLLIN_FREQ must satisfy KINETIS_MCG_PLL_REF_MIN >= \
         BOARD_PLLIN_FREQ <= KINETIS_MCG_PLL_REF_MAX
#endif

#if ((BOARD_FRDIV & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT) > KINETIS_MCG_C1_FRDIV_MAX
#  error BOARD_FRDIV choice is not supported on this SoC
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void __ramfunc__
kinesis_setdividers(uint32_t div1, uint32_t div2,
                    uint32_t div3, uint32_t div4);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinesis_portclocks
 *
 * Description:
 *   Enable all of the port clocks
 *
 ****************************************************************************/

static inline void kinesis_portclocks(void)
{
  uint32_t regval;

  /* Enable all of the port clocks */

  regval = getreg32(KINETIS_SIM_SCGC5);
  regval |= (SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC |
             SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);
  putreg32(regval, KINETIS_SIM_SCGC5);
}

/****************************************************************************
 * Name: kinetis_pllconfig
 *
 * Description:
 *   Initialize the PLL using the settings in board.h.  This assumes that
 *   the MCG is in default FLL Engaged Internal (FEI mode) out of reset.
 *
 ****************************************************************************/

void kinetis_pllconfig(void)
{
#if defined(SIM_SCGC4_LLWU) || defined(BOARD_SOPT2_PLLFLLSEL) || \
    defined(BOARD_SIM_CLKDIV3_FREQ)
  uint32_t regval32;
#endif
  uint8_t regval8;

#if defined(BOARD_OSC_CR)
  /* Use complete BOARD_OSC_CR settings */

  putreg8(BOARD_OSC_CR, KINETIS_OSC_CR);
#endif

#if defined(BOARD_OSC_DIV)
  /* Use complete BOARD_OSC_DIV settings */

  putreg8(BOARD_OSC_DIV, KINETIS_OSC_DIV);
#endif

#if defined(BOARD_MCG_C2)

  /* Use complete BOARD_MCG_C2 settings */

  putreg8(BOARD_MCG_C2, KINETIS_MCG_C2);
#else

  /* Transition to FLL Bypassed External (FBE) mode */

#  if defined(BOARD_EXTCLOCK)
  /*   IRCS  = 0 (Internal Reference Clock Select)
   *   LP    = 0 (Low Power Select)
   *   EREFS = 0 (External Reference Select)
   *   HGO   = 0 (High Gain Oscillator Select)
   *   RANGE = 0 (Oscillator of 32 kHz to 40 kHz)
   */

  putreg8(EXTCLOCK_MCG_C2, KINETIS_MCG_C2);
#  else
  /* Enable external oscillator:
   *
   *   IRCS    = 0 (Internal Reference Clock Select)
   *   LP      = 0 (Low Power Select)
   *   EREFS   = 1 (External Reference Select)
   *   HGO     = 1 (High Gain Oscillator Select)
   *   RANGE   = 2 (Oscillator of 8 MHz to 32 MHz)
   *   FCFTRIM = 0 if not supported or value provided by board
   *   LOCRE0  = 0 if not supported or value provided by board
   */

  putreg8(BOARD_MCG_C2_LOCRE0 | BOARD_MCG_C2_FCFTRIM | BOARD_MGC_C2_HGO |
          MCG_C2_RANGE_VHIGH | MCG_C2_EREFS, KINETIS_MCG_C2);
#  endif
#endif /* defined(BOARD_MCG_C2) */
#if defined(SIM_SCGC4_LLWU)
  /* Released latched state of oscillator and GPIO */

  regval32 = getreg32(KINETIS_SIM_SCGC4);
  regval32 |= SIM_SCGC4_LLWU;
  putreg32(regval32, KINETIS_SIM_SCGC4);
#endif

#if defined(LLWU_CS_ACKISO)
  regval8 = getreg8(KINETIS_LLWU_CS);
  regval8 |= LLWU_CS_ACKISO;
  putreg8(regval8, KINETIS_LLWU_CS);
#endif

#if defined(PMC_REGSC_ACKISO)
  regval8 = getreg8(KINETIS_PMC_REGSC);
  regval8 |= PMC_REGSC_ACKISO;
  putreg8(regval8, KINETIS_PMC_REGSC);
#endif

  /* Select external oscillator and Reference Divider and clear IREFS to
   * start the external oscillator.
   *
   *   IREFSTEN = 0 (Internal Reference Stop Enable)
   *   IRCLKEN  = 0 (Internal Reference Clock Enable)
   *   IREFS    = 0 (Internal Reference Select)
   *   FRDIV    = BOARD_FRDIV (FLL External Reference Divider)
   *   CLKS     = 2 (Clock Source Select, External reference clock)
   */

  putreg8(BOARD_FRDIV | MCG_C1_CLKS_EXTREF, KINETIS_MCG_C1);

#ifndef BOARD_EXTCLOCK
  /* If we aren't using an oscillator input we don't need to wait for the
   * oscillator to initialize
   */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_OSCINIT) == 0);
#endif

  /* Wait for Reference clock Status bit to clear */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_IREFST) != 0);

  /* Wait for clock status bits to show that the clock source is the
   * external reference clock.
   */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_CLKST_MASK) != MCG_S_CLKST_EXTREF);

  /* We are now in  FLL Bypassed External (FBE) mode. Configure PLL
   * reference clock divider:
   *
   *   PLLCLKEN = 0
   *   PLLSTEN  = 0
   *   PRDIV    = Determined by PLL reference clock frequency
   *
   * Either the external clock or crystal frequency is used to select the
   * PRDIV value. Only reference clock frequencies are supported that will
   * produce a KINETIS_MCG_PLL_REF_MIN >= PLLIN <= KINETIS_MCG_PLL_REF_MAX
   * reference clock to the PLL.
   */

  putreg8(MCG_C5_PRDIV(BOARD_PRDIV), KINETIS_MCG_C5);

  /* Ensure that MCG_C6 is at the reset default of 0: LOLIE disabled, PLL
   * disabled, clk monitor disabled, PLL VCO divider cleared.
   */

  putreg8(0, KINETIS_MCG_C6);

  /* Set system options dividers based on settings from the board.h file.
   *
   * MCG         = PLL
   * Core        = MCG / BOARD_OUTDIV1
   * bus         = MCG / BOARD_OUTDIV2
   * FlexBus     = MCG / BOARD_OUTDIV3
   * Flash clock = MCG / BOARD_OUTDIV4
   */

  kinesis_setdividers(BOARD_OUTDIV1, BOARD_OUTDIV2,
                      BOARD_OUTDIV3, BOARD_OUTDIV4);

  /* Set the VCO divider, VDIV, is defined in the board.h file.  VDIV
   * selects the amount to divide the VCO output of the PLL. The VDIV bits
   * establish the multiplication factor applied to the reference clock
   * frequency.  Also set
   *
   * LOLIE       = 0 (Loss of Lock Interrupt Enable)
   * PLLS        = 1 (PLL Select)
   * CME         = 0 (Clock Monitor Enable)
   */

  putreg8(MCG_C6_PLLS | MCG_C6_VDIV(BOARD_VDIV), KINETIS_MCG_C6);

  /* Wait for the PLL status bit to set */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_PLLST) == 0);

  /* Wait for the PLL LOCK bit to set */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_LOCK0) == 0);

  /* We are now running in PLL Bypassed External (PBE) mode.  Transition to
   * PLL Engaged External (PEE) mode by setting CLKS to 0
   */

  regval8 = getreg8(KINETIS_MCG_C1);
  regval8 &= ~MCG_C1_CLKS_MASK;
  putreg8(regval8, KINETIS_MCG_C1);

  /* Wait for clock status bits to update */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_CLKST_MASK) != MCG_S_CLKST_PLL);

  /* We are now running in PLL Engaged External (PEE) mode. */

  /* Do we have BOARD_SOPT2_PLLFLLSEL */

#if defined(BOARD_SOPT2_PLLFLLSEL)
  /* Set up the SOPT2[PLLFLLSEL] */

  regval32 = getreg32(KINETIS_SIM_SOPT2);
  regval32 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
  regval32 |= BOARD_SOPT2_PLLFLLSEL;
  putreg32(regval32, KINETIS_SIM_SOPT2);
#endif

#if defined(BOARD_SIM_CLKDIV2_FREQ)
  /* Set up the SIM_CLKDIV2[USBFRAC, USBDIV] */

  regval32 = getreg32(KINETIS_SIM_CLKDIV2);
  regval32 &= ~(SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK);
  regval32 |= (SIM_CLKDIV2_USBFRAC(BOARD_SIM_CLKDIV2_USBFRAC) |
               SIM_CLKDIV2_USBDIV(BOARD_SIM_CLKDIV2_USBDIV));
  putreg32(regval32, KINETIS_SIM_CLKDIV2);
#endif

#if defined(BOARD_SIM_CLKDIV3_FREQ)
  /* Set up the SIM_CLKDIV3 [PLLFLLFRAC, PLLFLLDIV] */

  regval32 = getreg32(KINETIS_SIM_CLKDIV3);
  regval32 &= ~(SIM_CLKDIV3_PLLFLLFRAC_MASK | SIM_CLKDIV3_PLLFLLDIV_MASK);
  regval32 |= (SIM_CLKDIV3_PLLFLLFRAC(BOARD_SIM_CLKDIV3_PLLFLLFRAC) |
               SIM_CLKDIV3_PLLFLLDIV(BOARD_SIM_CLKDIV3_PLLFLLDIV));
  putreg32(regval32, KINETIS_SIM_CLKDIV3);
#endif
}

/****************************************************************************
 * Name: kinetis_traceconfig
 *
 * Description:
 *   Enable trace clocks.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_TRACE
static inline void kinetis_traceconfig(void)
{
  uint32_t regval;

  /* Set the trace clock to the core clock frequency in the SIM SOPT2
   * register
   */

  regval  = getreg32(KINETIS_SIM_SOPT2);
  regval |= SIM_SOPT2_TRACECLKSEL;
  putreg32(regval, KINETIS_SIM_SOPT2);

  /* Enable the TRACE_CLKOUT pin function on the configured pin */

  kinetis_gpioconfig(GPIO_TRACE_CLKOUT);
}
#else
#  define kinetis_traceconfig()
#endif

/****************************************************************************
 * Name: kinetis_fbconfig
 *
 * Description:
 *   Enable FlexBus clocking.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_FLEXBUS
static inline void kinetis_fbconfig(void)
{
  uint32_t regval;

  /* Enable the clock to the FlexBus module */

  regval  = getreg32(KINETIS_SIM_SCGC7);
  regval |= SIM_SCGC7_FLEXBUS;
  putreg32(regval, KINETIS_SIM_SCGC7);

  /* Enable the FB_CLKOUT function on PTC3 (alt5 function) */

  kinetis_gpioconfig(GPIO_FB_CLKOUT);
}
#else
#  define kinetis_fbconfig()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_clockconfig
 *
 * Description:
 *   Called to initialize the Kinetis chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void kinetis_clockconfig(void)
{
  /* Enable all of the port clocks */

  kinesis_portclocks();

  /* Configure the PLL based on settings in the board.h file */

  kinetis_pllconfig();

  /* For debugging, we will normally want to enable the trace clock and/or
   * the FlexBus clock.
   */

  kinetis_traceconfig();
  kinetis_fbconfig();
}

/****************************************************************************
 * Name: kinesis_setdividers
 *
 * Description:
 *  "This routine must be placed in RAM. It is a workaround for errata e2448.
 *   Flash prefetch must be disabled when the flash clock divider is changed.
 *   This cannot be performed while executing out of flash.  There must be a
 *   short delay after the clock dividers are changed before prefetch can be
 *   re-enabled."
 *
 * NOTE: This must have global scope only to prevent optimization logic from
 *   inlining the function.
 *
 ****************************************************************************/

void __ramfunc__
kinesis_setdividers(uint32_t div1, uint32_t div2,
                    uint32_t div3, uint32_t div4)
{
  uint32_t regval;
  volatile int i;

  /* Save the current value of the Flash Access Protection Register */

  regval = getreg32(KINETIS_FMC_PFAPR);

  /* Set M0PFD through M7PFD to 1 to disable prefetch */

  putreg32(FMC_PFAPR_M7PFD | FMC_PFAPR_M6PFD | FMC_PFAPR_M5PFD |
           FMC_PFAPR_M4PFD | FMC_PFAPR_M3PFD | FMC_PFAPR_M2PFD |
           FMC_PFAPR_M1PFD | FMC_PFAPR_M0PFD,
           KINETIS_FMC_PFAPR);

  /* Set clock dividers to desired value */

  putreg32(SIM_CLKDIV1_OUTDIV1(div1) | SIM_CLKDIV1_OUTDIV2(div2) |
           SIM_CLKDIV1_OUTDIV3(div3) | SIM_CLKDIV1_OUTDIV4(div4),
           KINETIS_SIM_CLKDIV1);

  /* Wait for dividers to change */

  for (i = 0 ; i < div4 ; i++);

  /* Re-store the saved value of FMC_PFAPR */

  putreg32(regval, KINETIS_FMC_PFAPR);
}
