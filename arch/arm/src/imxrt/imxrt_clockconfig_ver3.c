/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig_ver3.c
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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/imxrt_dcdc.h"
#include "hardware/rt118x/imxrt118x_anadig.h"
#include "hardware/rt118x/imxrt118x_ccm.h"
#include "imxrt_clockconfig_ver3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fixed clock source frequencies programmed by the Cortex-M33 boot ROM
 * before the M7 is released.  NuttX does not reprogram these PLLs, so
 * their frequencies are treated as static constants; if a future use
 * case needs to reconfigure the PLLs, extend imxrt_get_clock() to read
 * the ANADIG registers instead.
 */

#define IMXRT_OSC_24M_HZ         (24000000u)
#define IMXRT_OSC_RC_24M_HZ      (24000000u)
#define IMXRT_OSC_RC_400M_HZ     (400000000u)
#define IMXRT_ARM_PLL_HZ         (798000000u)
#define IMXRT_SYS_PLL1_HZ        (1000000000u)
#define IMXRT_SYS_PLL2_HZ        (528000000u)
#define IMXRT_SYS_PLL3_HZ        (480000000u)

/* Standard PFD fractions used by the boot ROM (IMXRT1180RM Ch. 20.5 default
 * configuration).  Output = (PLL * 18) / frac.
 */

#define IMXRT_SYS_PLL2_PFD0_HZ   (352000000u)  /* frac = 27 */
#define IMXRT_SYS_PLL2_PFD1_HZ   (594000000u)  /* frac = 16 */
#define IMXRT_SYS_PLL2_PFD2_HZ   (396000000u)  /* frac = 24 */
#define IMXRT_SYS_PLL2_PFD3_HZ   (297000000u)  /* frac = 32 */
#define IMXRT_SYS_PLL3_PFD0_HZ   (664615384u)  /* frac = 13 */
#define IMXRT_SYS_PLL3_PFD1_HZ   (508235294u)  /* frac = 17 */
#define IMXRT_SYS_PLL3_PFD2_HZ   (392727272u)  /* frac = 22 */
#define IMXRT_SYS_PLL3_PFD3_HZ   (392727272u)  /* frac = 22 */

/* ARM_PLL loop divider for 798 MHz with post_div = 2:
 *
 *   Fout = 24 MHz * 133 / (2 * 2) = 798 MHz
 */

#define ARM_PLL_DIV_SELECT       (133)

/* Analog settling delays, expressed as NOP spins because no timer exists
 * this early.  The M33 still runs from OSC_RC_400M/2 (200 MHz), so one
 * iteration is roughly 5 ns; both values are generously rounded up.
 */

#define PLL_LDO_SETTLE_SPINS     (1000)     /* SDK uses 1 us */
#define ARM_PLL_SETTLE_SPINS     (25000)    /* SDK uses 30 us */
#define DCDC_SETTLE_SPINS        (100000)   /* ~123 us of 32 kHz clock */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_MIMXRT1189CVM8C_CM33

/****************************************************************************
 * Name: imxrt_spin
 *
 * Description:
 *   Crude busy delay for the analog settling times required during clock
 *   bring-up.  The OS timer is not running this early and the core clock
 *   is still OSC_RC_400M/2, so this is expressed in loop iterations rather
 *   than microseconds.
 *
 ****************************************************************************/

static void imxrt_spin(int spins)
{
  int i;

  for (i = 0; i < spins; i++)
    {
      __asm__ __volatile__("nop");
    }
}

/****************************************************************************
 * Name: imxrt_set_core_voltage
 *
 * Description:
 *   Program the VDD1P0 run-mode target of the on-chip DCDC.
 *
 *   Both cores share a single VDD1P0 rail.  The DCDC arbitrates TRG_SW_0
 *   and TRG_SW_1 and applies the higher of the two targets
 *   (IMXRT1180RM Rev. 10 sec. 22.7.1.8), so both must be programmed for
 *   the new target to take effect.
 *
 * Input Parameters:
 *   target - VDD1P0CTRL_TRG encoding, 25 mV per step from 0.6 V.
 *
 ****************************************************************************/

static void imxrt_set_core_voltage(uint32_t target)
{
  uint32_t regval;

  regval  = getreg32(IMXRT_DCDC_TRG_SW_0);
  regval &= ~DCDC_TRG_SW_VDD1P0CTRL_TRG_MASK;
  regval |= DCDC_TRG_SW_VDD1P0CTRL_TRG(target);
  putreg32(regval, IMXRT_DCDC_TRG_SW_0);

  regval  = getreg32(IMXRT_DCDC_TRG_SW_1);
  regval &= ~DCDC_TRG_SW_VDD1P0CTRL_TRG_MASK;
  regval |= DCDC_TRG_SW_VDD1P0CTRL_TRG(target);
  putreg32(regval, IMXRT_DCDC_TRG_SW_1);

  /* Wait for the regulator to reach the new target.  CURRENT_TRG
   * [DCDC_UPDATING] must not be used for this: RM sec. 22.7.1.5 states it
   * can stay asserted for seconds and is intended for debug only.  The
   * supply status is REG0[STS_DC_OK], which is synchronised to the 32 kHz
   * clock and settles within ~4 of its cycles (~123 us), so allow the
   * request to reach the analog before sampling it.
   */

  imxrt_spin(DCDC_SETTLE_SPINS);

  while ((getreg32(IMXRT_DCDC_REG0) & DCDC_REG0_STS_DC_OK) == 0);
}

/****************************************************************************
 * Name: imxrt_enable_pll_ldo
 *
 * Description:
 *   Enable the PHY LDO that supplies the PLLs.  It must be up and stable
 *   before ARM_PLL is powered on.
 *
 ****************************************************************************/

static void imxrt_enable_pll_ldo(void)
{
  uint32_t regval;

  regval = PHY_LDO_CTRL0_LINREG_OUTPUT_TRG(0x10) | PHY_LDO_CTRL0_LINREG_EN;

  if (getreg32(IMXRT_PHY_LDO_CTRL0_RW) != regval)
    {
      /* Bring the regulator up with its current limiter armed ... */

      putreg32(regval | PHY_LDO_CTRL0_LINREG_ILIMIT_EN,
               IMXRT_PHY_LDO_CTRL0_RW);
      imxrt_spin(PLL_LDO_SETTLE_SPINS);

      /* ... then drop the limiter once stable, which lowers ARM PLL jitter
       * at cold temperature.
       */

      putreg32(regval, IMXRT_PHY_LDO_CTRL0_RW);
    }
}

/****************************************************************************
 * Name: imxrt_init_arm_pll
 *
 * Description:
 *   Bring up ARM_PLL at 798 MHz.
 *
 *     Fout = 24 MHz * DIV_SELECT / (2 * post_div)
 *          = 24 MHz * 133 / (2 * 2)
 *          = 798 MHz
 *
 *   This is the maximum supported ARM_PLL output.  It clocks the M7
 *   directly and gives the M33 266 MHz through its integer divider.
 *
 *   The RM's ARM_PLL_CTRL description refers to a "PLL Enable Sequence"
 *   topic for the HOLD_RING_OFF handling, but no such section exists in
 *   Rev. 10 and no hold time is given.  NXP's own CLOCK_InitArmPll() does
 *   not use HOLD_RING_OFF on this PLL at all, so follow that: configure
 *   the dividers while gated, power up, allow the analog to settle, then
 *   wait for lock.
 *
 ****************************************************************************/

static void imxrt_init_arm_pll(void)
{
  uint32_t regval;

  imxrt_enable_pll_ldo();

  regval = ANADIG_PLL_ARM_DIV_SELECT(ARM_PLL_DIV_SELECT) |
           ANADIG_PLL_ARM_POST_DIV_SEL(ANADIG_PLL_ARM_POST_DIV_2);

  /* Power the PLL down before touching the dividers, keeping the output
   * gated so that nothing sees an intermediate frequency.
   */

  putreg32(regval | ANADIG_PLL_ARM_GATE, IMXRT_ANADIG_PLL_ARM_CTRL);

  /* Apply the configuration and power up, still gated. */

  putreg32(regval | ANADIG_PLL_ARM_GATE | ANADIG_PLL_ARM_POWERUP,
           IMXRT_ANADIG_PLL_ARM_CTRL);

  UP_DSB();
  UP_ISB();

  imxrt_spin(ARM_PLL_SETTLE_SPINS);

  while ((getreg32(IMXRT_ANADIG_PLL_ARM_CTRL) &
          ANADIG_PLL_ARM_STABLE) == 0);

  /* Locked: enable the clock and ungate the output. */

  putreg32(regval | ANADIG_PLL_ARM_POWERUP | ANADIG_PLL_ARM_ENABLE_CLK,
           IMXRT_ANADIG_PLL_ARM_CTRL);
}

#endif /* CONFIG_ARCH_CHIP_MIMXRT1189CVM8C_CM33 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ccm_configure_root_clock
 ****************************************************************************/

int imxrt_ccm_configure_root_clock(int root, int src, uint32_t div)
{
  uint32_t auth;
  uint32_t regval;
  uint32_t newval;
  int i;

  if (root >= CCM_CR_COUNT || div == 0 || div > 256)
    {
      return -EINVAL;
    }

  auth = getreg32(IMXRT_CCM_CR_AUTH(root));

  if ((auth & CCM_CR_AUTH_DOMAIN(BOARD_CCM_DOMAIN_ID)) == 0)
    {
      return -EINVAL;
    }

  /* Find the mux index that selects the requested source. */

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

  regval = getreg32(IMXRT_CCM_CR_CTRL(root));
  newval = regval & ~(CCM_CR_CTRL_MUX_MASK | CCM_CR_CTRL_DIV_MASK);
  newval |= CCM_CR_CTRL_MUX_SRCSEL(i) | CCM_CR_CTRL_DIV(div);

  /* Do not reconfigure a root that already has the requested settings. */

  if (newval == regval)
    {
      return OK;
    }

  putreg32(newval, IMXRT_CCM_CR_CTRL(root));

  while (getreg32(IMXRT_CCM_CR_STAT0(root)) & CCM_CR_STAT0_CHANGING);

  return OK;
}

/****************************************************************************
 * Name: imxrt_ccm_gate_on
 ****************************************************************************/

int imxrt_ccm_gate_on(int gate, bool enabled)
{
  uint32_t value;

  if (gate >= CCM_LPCG_COUNT)
    {
      return -EINVAL;
    }

  value = enabled ? CCM_LPCG_DIR_ON : 0u;
  putreg32(value, IMXRT_CCM_LPCG_DIR(gate));

  while ((getreg32(IMXRT_CCM_LPCG_STAT0(gate)) & CCM_LPCG_STAT0_ON) !=
         value);

  return OK;
}

/****************************************************************************
 * Name: imxrt_periphclk_configure
 *
 * Description:
 *   Thin wrapper that lets legacy callers pass CCM_LPCG_DIR_ON /
 *   CCM_LPCG_DIR_OFF as the second argument.
 *
 ****************************************************************************/

void imxrt_periphclk_configure(unsigned int index, unsigned int value)
{
  imxrt_ccm_gate_on((int)index, (value & CCM_LPCG_DIR_ON) != 0);
}

/****************************************************************************
 * Name: imxrt_get_clock
 ****************************************************************************/

int imxrt_get_clock(int clkname, uint32_t *frequency)
{
  switch (clkname)
    {
      case OSC_RC_24M:
        *frequency = IMXRT_OSC_RC_24M_HZ;
        break;

      case OSC_RC_400M:
        *frequency = IMXRT_OSC_RC_400M_HZ;
        break;

      case OSC_24M:
        *frequency = IMXRT_OSC_24M_HZ;
        break;

      case ARM_PLL_OUT:
        *frequency = IMXRT_ARM_PLL_HZ;
        break;

      case SYS_PLL1_OUT:
        *frequency = IMXRT_SYS_PLL1_HZ;
        break;

      case SYS_PLL1_DIV2:
        *frequency = IMXRT_SYS_PLL1_HZ / 2;
        break;

      case SYS_PLL1_DIV5:
        *frequency = IMXRT_SYS_PLL1_HZ / 5;
        break;

      case SYS_PLL2_OUT:
        *frequency = IMXRT_SYS_PLL2_HZ;
        break;

      case SYS_PLL2_PFD0:
        *frequency = IMXRT_SYS_PLL2_PFD0_HZ;
        break;

      case SYS_PLL2_PFD1:
        *frequency = IMXRT_SYS_PLL2_PFD1_HZ;
        break;

      case SYS_PLL2_PFD2:
        *frequency = IMXRT_SYS_PLL2_PFD2_HZ;
        break;

      case SYS_PLL2_PFD3:
        *frequency = IMXRT_SYS_PLL2_PFD3_HZ;
        break;

      case SYS_PLL3_OUT:
        *frequency = IMXRT_SYS_PLL3_HZ;
        break;

      case SYS_PLL3_DIV2:
        *frequency = IMXRT_SYS_PLL3_HZ / 2;
        break;

      case SYS_PLL3_PFD0:
        *frequency = IMXRT_SYS_PLL3_PFD0_HZ;
        break;

      case SYS_PLL3_PFD1:
        *frequency = IMXRT_SYS_PLL3_PFD1_HZ;
        break;

      case SYS_PLL3_PFD2:
        *frequency = IMXRT_SYS_PLL3_PFD2_HZ;
        break;

      case SYS_PLL3_PFD3:
        *frequency = IMXRT_SYS_PLL3_PFD3_HZ;
        break;

      case AUDIO_PLL_OUT:
        *frequency = 0;
        break;

      default:
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_get_rootclock
 ****************************************************************************/

int imxrt_get_rootclock(uint32_t clkroot, uint32_t *frequency)
{
  uint32_t reg;
  uint32_t mux;
  uint32_t div;
  int clkname;

  if (clkroot >= CCM_CR_COUNT)
    {
      return -ENODEV;
    }

  reg = getreg32(IMXRT_CCM_CR_CTRL(clkroot));

  if ((reg & CCM_CR_CTRL_OFF) != 0)
    {
      *frequency = 0;
      return OK;
    }

  mux     = (reg & CCM_CR_CTRL_MUX_MASK) >> CCM_CR_CTRL_MUX_SHIFT;
  clkname = g_ccm_root_mux[clkroot][mux];
  imxrt_get_clock(clkname, frequency);
  div     = ((reg & CCM_CR_CTRL_DIV_MASK) >> CCM_CR_CTRL_DIV_SHIFT) + 1;
  *frequency = *frequency / div;

  return OK;
}

/****************************************************************************
 * Name: imxrt_clockconfig
 *
 * Description:
 *   Called to initialize the i.MX RT.  This does whatever setup is needed to
 *   put the SoC in a usable state.  The Cortex-M33 boot ROM has already
 *   started essential PLLs and set the FlexSPI1 clock, so only the roots
 *   required by NuttX are configured here.  CCM_CR_FLEXSPI1 and its LPCG
 *   MUST NOT be touched while executing XIP from FlexSPI1.
 *
 ****************************************************************************/

void imxrt_clockconfig(void)
{
#ifdef CONFIG_ARCH_CHIP_MIMXRT1189CVM8C_CM33
  /* Raise VDD1P0 to the HSRUN (OverDrive) level.  Both cores share this
   * rail and the M33 is the boot core, so it does this on behalf of both.
   *
   * Forward body bias is deliberately not enabled here.  Per RM sec.
   * 19.3.3 and 24.3.2.3.3.2 FBB is implemented in the Cortex-M7 platform
   * only; the well-bias power switch it turns on belongs to the M7 domain.
   * It is enabled by the bootloader as part of releasing the M7.
   */

  imxrt_set_core_voltage(DCDC_1P0_TARGET_1P125V);

  /* Configure ARM_PLL and the M33 root here.  The M7 root and M7-specific
   * power setup are owned by the bootloader.  The M7 must not touch
   * ARM_PLL or the M33 root when it later executes this function.
   */

  imxrt_init_arm_pll();

  /* M33 core clock: ARM_PLL (798 MHz) / 3 = 266 MHz. */

  imxrt_ccm_configure_root_clock(CCM_CR_M33, ARM_PLL_OUT, 3);

  /* Root1 belongs exclusively to the M33 domain.  LOCK_LIST makes this
   * ownership immutable until the next system reset.
   */

  putreg32(CCM_CR_AUTH_DOMAIN(BOARD_CCM_DOMAIN_ID) |
           CCM_CR_AUTH_LOCK_LIST,
           IMXRT_CCM_CR_AUTH(CCM_CR_M33));
#endif

  /* AON bus (LPUART1/2 pclk): SYS_PLL2 (528 MHz) / 4 = 132 MHz */

  imxrt_ccm_configure_root_clock(CCM_CR_BUS_AON, SYS_PLL2_OUT, 4);

  /* M7 SysTick reference: 24 MHz / 240 = 100 kHz */

  imxrt_ccm_configure_root_clock(CCM_CR_M7_SYSTICK, OSC_24M, 240);

  /* LPUART1/2 functional clock: SYS_PLL3_DIV2 (240 MHz) / 10 = 24 MHz */

  imxrt_ccm_configure_root_clock(CCM_CR_LPUART0102, SYS_PLL3_DIV2, 10);

  /* Turn on the LPCGs used by the minimal port */

  imxrt_ccm_gate_on(CCM_LPCG_IOMUXC1, true);
  imxrt_ccm_gate_on(CCM_LPCG_IOMUXC2, true);
  imxrt_ccm_gate_on(CCM_LPCG_LPUART1, true);
}
