/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_clockconfig.c
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

#include <stdbool.h>
#include <stdint.h>

#include "tricore_internal.h"
#include "tc3xx_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCU clock control unit (CCU) registers */

#define TC3XX_SCU_OSCCON           0xf0036010
#define TC3XX_SCU_SYSPLLSTAT       0xf0036014
#define TC3XX_SCU_SYSPLLCON0       0xf0036018
#define TC3XX_SCU_SYSPLLCON1       0xf003601c
#define TC3XX_SCU_PERPLLSTAT       0xf0036024
#define TC3XX_SCU_PERPLLCON0       0xf0036028
#define TC3XX_SCU_PERPLLCON1       0xf003602c
#define TC3XX_SCU_CCUCON0          0xf0036030
#define TC3XX_SCU_CCUCON1          0xf0036034
#define TC3XX_SCU_CCUCON2          0xf0036040
#define TC3XX_SCU_CCUCON5          0xf003604c
#define TC3XX_SCU_CCUCON6          0xf0036080  /* CCUCON6 to CCUCON11 */

#define TC3XX_OSCCON_PLLLV         (1 << 1)
#define TC3XX_OSCCON_MODE_MASK     (0x3 << 5)
#define TC3XX_OSCCON_PLLHV         (1 << 8)
#define TC3XX_OSCCON_OSCVAL_SHIFT  16
#define TC3XX_OSCCON_OSCVAL_MASK   (0x1f << TC3XX_OSCCON_OSCVAL_SHIFT)

#define TC3XX_PLLCON0_DIVBY        (1 << 0)    /* PERPLLCON0 only */
#define TC3XX_PLLCON0_NDIV_SHIFT   9
#define TC3XX_PLLCON0_NDIV_MASK    (0x7f << TC3XX_PLLCON0_NDIV_SHIFT)
#define TC3XX_PLLCON0_PLLPWD       (1 << 16)
#define TC3XX_PLLCON0_RESLD        (1 << 18)
#define TC3XX_PLLCON0_PDIV_SHIFT   24
#define TC3XX_PLLCON0_PDIV_MASK    (0x7 << TC3XX_PLLCON0_PDIV_SHIFT)
#define TC3XX_PLLCON0_INSEL_SHIFT  30          /* SYSPLLCON0 only */
#define TC3XX_PLLCON0_INSEL_MASK   (0x3 << TC3XX_PLLCON0_INSEL_SHIFT)

#define TC3XX_PLLCON1_K2DIV_MASK   (0x7 << 0)
#define TC3XX_PLLCON1_K3DIV_SHIFT  8           /* PERPLLCON1 only */
#define TC3XX_PLLCON1_K3DIV_MASK   (0x7 << TC3XX_PLLCON1_K3DIV_SHIFT)

#define TC3XX_PLLSTAT_PWDSTAT      (1 << 1)
#define TC3XX_PLLSTAT_LOCK         (1 << 2)
#define TC3XX_PLLSTAT_K3RDY        (1 << 4)    /* PERPLLSTAT only */
#define TC3XX_PLLSTAT_K2RDY        (1 << 5)

#define TC3XX_CCUCON_UP            (1 << 30)
#define TC3XX_CCUCON_LCK           (1 << 31)
#define TC3XX_CCUCON0_CLKSEL_MASK  (0x3 << 28)
#define TC3XX_CCUCON0_CLKSEL_PLL   (0x1 << 28)
#define TC3XX_CCUCON1_CLKSEL_MASK  ((0x3 << 4) | (0x3 << 20) | (0x3 << 28))
#define TC3XX_CCUCON2_CLKSEL_MASK  (0x3 << 12)

/* SMU alarms of the clock monitors, masked while the PLLs change */

#define TC3XX_SMU_CMD              0xf0036820
#define TC3XX_SMU_KEYS             0xf0036834
#define TC3XX_SMU_AGCF8(n)         (0xf0036960 + (n) * 4)
#define TC3XX_SMU_AG8              0xf00369e0

#define TC3XX_SMU_KEYS_CFG         0xbc
#define TC3XX_SMU_CMD_ALARM_CLR    0x5
#define TC3XX_SMU_CCU_ALARMS       0x1d

/* STM0 timer bits [31:0], for the waits of the PLL ramp up */

#define TC3XX_STM0_TIM0            0xf0001010

/* Loop counts before a wait gives up */

#define TC3XX_CCUCON_LCK_TIMEOUT   0x1000
#define TC3XX_PLL_LOCK_TIMEOUT     0x3000
#define TC3XX_PLL_PWDSTAT_TIMEOUT  0x3000
#define TC3XX_PLL_KRDY_TIMEOUT     0x6000
#define TC3XX_OSC_TIMEOUT          0x493e0

/* Clock setup: 20 MHz crystal, system PLL 300 MHz, peripheral PLL 320 and
 * 200 MHz, as the iLLD IfxScuCcu default configuration for TC39xB.
 */

#define TC3XX_XTAL_MHZ             20
#define TC3XX_PLL_INSEL_OSC0       1

#define TC3XX_SYSPLL_PDIV          (1 - 1)
#define TC3XX_SYSPLL_NDIV          (30 - 1)
#define TC3XX_SYSPLL_K2_INITIAL    (6 - 1)

#define TC3XX_PERPLL_PDIV          (1 - 1)
#define TC3XX_PERPLL_NDIV          (32 - 1)
#define TC3XX_PERPLL_K2            (2 - 1)
#define TC3XX_PERPLL_K3            (2 - 1)

/* Wait of each PLL ramp up step, 100 us at the 100 MHz STM clock */

#define TC3XX_PLL_STEP_WAIT        10000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tc3xx_ccucon_s
{
  uint32_t value;
  uint32_t mask;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* System PLL K2 divider steps from 100 MHz up to 300 MHz */

static const uint8_t g_tc3xx_k2_steps[] =
{
  4 - 1, 3 - 1, 2 - 1
};

/* Clock distribution: CCUCON0, 1, 2, 5, then 6 to 11 */

static const struct tc3xx_ccucon_s g_tc3xx_ccucon[] =
{
  { 0x07230113, 0x0fff0fff },
  { 0x21110212, 0x3f3f0fbf },
  { 0x00001201, 0x00003f0f },
  { 0x00000132, 0x00000fff },
  { 0x00000000, 0x0000003f },
  { 0x00000000, 0x0000003f },
  { 0x00000000, 0x0000003f },
  { 0x00000000, 0x0000003f },
  { 0x00000000, 0x0000003f },
  { 0x00000000, 0x0000003f },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc3xx_wait
 *
 * Description:
 *   Wait until all bits of mask in the register equal the bits in value.
 *   Return 1 if the wait gave up after count loops, otherwise 0.
 *
 ****************************************************************************/

static int tc3xx_wait(uintptr_t reg, uint32_t mask, uint32_t value,
                      uint32_t count)
{
  while ((getreg32(reg) & mask) != value)
    {
      if (--count == 0)
        {
          return 1;
        }
    }

  return 0;
}

static void tc3xx_modify(uintptr_t reg, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(reg) & ~clear) | set, reg);
}

/****************************************************************************
 * Name: tc3xx_ccucon_write
 *
 * Description:
 *   Write a CCUCON register that has a LCK bit, waiting until it is
 *   unlocked before and after the write.
 *
 ****************************************************************************/

static int tc3xx_ccucon_write(uintptr_t reg, uint32_t value)
{
  int ret;

  ret  = tc3xx_wait(reg, TC3XX_CCUCON_LCK, 0, TC3XX_CCUCON_LCK_TIMEOUT);
  putreg32(value, reg);
  ret |= tc3xx_wait(reg, TC3XX_CCUCON_LCK, 0, TC3XX_CCUCON_LCK_TIMEOUT);

  return ret;
}

static uint32_t tc3xx_ccucon_value(uintptr_t reg,
                                   const struct tc3xx_ccucon_s *cfg)
{
  return (getreg32(reg) & ~cfg->mask) | (cfg->value & cfg->mask);
}

static void tc3xx_smu_ccu_alarms(bool enable)
{
  int i;

  putreg32(TC3XX_SMU_KEYS_CFG, TC3XX_SMU_KEYS);
  if (enable)
    {
      putreg32(TC3XX_SMU_CMD_ALARM_CLR, TC3XX_SMU_CMD);
      putreg32(TC3XX_SMU_CCU_ALARMS, TC3XX_SMU_AG8);
    }
  else
    {
      for (i = 0; i < 3; i++)
        {
          tc3xx_modify(TC3XX_SMU_AGCF8(i), TC3XX_SMU_CCU_ALARMS, 0);
        }
    }

  putreg32(0, TC3XX_SMU_KEYS);
}

/****************************************************************************
 * Name: tc3xx_pll_init
 *
 * Description:
 *   Run the clocks from the backup clock, set up the oscillator and both
 *   PLLs, wait for the lock and switch back to the PLLs.
 *
 ****************************************************************************/

static int tc3xx_pll_init(void)
{
  uint32_t kready = TC3XX_PLLSTAT_K2RDY | TC3XX_PLLSTAT_K3RDY;
  uint32_t count;
  int ret;

  tc3xx_wdt_safety_endinit(false);

  ret  = tc3xx_wait(TC3XX_SCU_CCUCON0, TC3XX_CCUCON_LCK, 0,
                    TC3XX_CCUCON_LCK_TIMEOUT);
  tc3xx_modify(TC3XX_SCU_CCUCON0, TC3XX_CCUCON0_CLKSEL_MASK,
               TC3XX_CCUCON_UP);
  ret |= tc3xx_wait(TC3XX_SCU_CCUCON0, TC3XX_CCUCON_LCK, 0,
                    TC3XX_CCUCON_LCK_TIMEOUT);

  tc3xx_smu_ccu_alarms(false);

  /* Power down both PLLs, so that the new settings cause no glitches */

  tc3xx_modify(TC3XX_SCU_SYSPLLCON0, TC3XX_PLLCON0_PLLPWD, 0);
  tc3xx_modify(TC3XX_SCU_PERPLLCON0, TC3XX_PLLCON0_PLLPWD, 0);
  ret |= tc3xx_wait(TC3XX_SCU_SYSPLLSTAT, TC3XX_PLLSTAT_PWDSTAT,
                    TC3XX_PLLSTAT_PWDSTAT, TC3XX_PLL_PWDSTAT_TIMEOUT);
  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, TC3XX_PLLSTAT_PWDSTAT,
                    TC3XX_PLLSTAT_PWDSTAT, TC3XX_PLL_PWDSTAT_TIMEOUT);

  /* External crystal */

  tc3xx_modify(TC3XX_SCU_OSCCON,
               TC3XX_OSCCON_MODE_MASK | TC3XX_OSCCON_OSCVAL_MASK,
               (TC3XX_XTAL_MHZ - 15) << TC3XX_OSCCON_OSCVAL_SHIFT);

  tc3xx_modify(TC3XX_SCU_SYSPLLCON0,
               TC3XX_PLLCON0_PDIV_MASK | TC3XX_PLLCON0_NDIV_MASK |
               TC3XX_PLLCON0_INSEL_MASK,
               (TC3XX_SYSPLL_PDIV << TC3XX_PLLCON0_PDIV_SHIFT) |
               (TC3XX_SYSPLL_NDIV << TC3XX_PLLCON0_NDIV_SHIFT) |
               (TC3XX_PLL_INSEL_OSC0 << TC3XX_PLLCON0_INSEL_SHIFT));

  /* K3DIV must be 0 or 1 before PERPLLCON0.DIVBY changes */

  tc3xx_modify(TC3XX_SCU_PERPLLCON1, TC3XX_PLLCON1_K3DIV_MASK,
               1 << TC3XX_PLLCON1_K3DIV_SHIFT);
  tc3xx_modify(TC3XX_SCU_PERPLLCON0,
               TC3XX_PLLCON0_DIVBY | TC3XX_PLLCON0_PDIV_MASK |
               TC3XX_PLLCON0_NDIV_MASK,
               (TC3XX_PERPLL_PDIV << TC3XX_PLLCON0_PDIV_SHIFT) |
               (TC3XX_PERPLL_NDIV << TC3XX_PLLCON0_NDIV_SHIFT));

  /* Power up both PLLs and set the initial K dividers */

  tc3xx_modify(TC3XX_SCU_SYSPLLCON0, 0, TC3XX_PLLCON0_PLLPWD);
  tc3xx_modify(TC3XX_SCU_PERPLLCON0, 0, TC3XX_PLLCON0_PLLPWD);
  ret |= tc3xx_wait(TC3XX_SCU_SYSPLLSTAT, TC3XX_PLLSTAT_PWDSTAT, 0,
                    TC3XX_PLL_PWDSTAT_TIMEOUT);
  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, TC3XX_PLLSTAT_PWDSTAT, 0,
                    TC3XX_PLL_PWDSTAT_TIMEOUT);

  ret |= tc3xx_wait(TC3XX_SCU_SYSPLLSTAT, TC3XX_PLLSTAT_K2RDY,
                    TC3XX_PLLSTAT_K2RDY, TC3XX_PLL_KRDY_TIMEOUT);
  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, kready, kready,
                    TC3XX_PLL_KRDY_TIMEOUT);

  tc3xx_modify(TC3XX_SCU_SYSPLLCON1, TC3XX_PLLCON1_K2DIV_MASK,
               TC3XX_SYSPLL_K2_INITIAL);
  tc3xx_modify(TC3XX_SCU_PERPLLCON1,
               TC3XX_PLLCON1_K2DIV_MASK | TC3XX_PLLCON1_K3DIV_MASK,
               TC3XX_PERPLL_K2 | (1 << TC3XX_PLLCON1_K3DIV_SHIFT));

  ret |= tc3xx_wait(TC3XX_SCU_SYSPLLSTAT, TC3XX_PLLSTAT_K2RDY,
                    TC3XX_PLLSTAT_K2RDY, TC3XX_PLL_KRDY_TIMEOUT);
  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, kready, kready,
                    TC3XX_PLL_KRDY_TIMEOUT);

  /* Wait for the oscillator, then for the lock of both PLLs */

  count = TC3XX_OSC_TIMEOUT;
  while ((getreg32(TC3XX_SCU_OSCCON) &
          (TC3XX_OSCCON_PLLLV | TC3XX_OSCCON_PLLHV)) == 0)
    {
      if (--count == 0)
        {
          ret = 1;
          break;
        }
    }

  tc3xx_modify(TC3XX_SCU_SYSPLLCON0, 0, TC3XX_PLLCON0_RESLD);
  tc3xx_modify(TC3XX_SCU_PERPLLCON0, 0, TC3XX_PLLCON0_RESLD);
  ret |= tc3xx_wait(TC3XX_SCU_SYSPLLSTAT, TC3XX_PLLSTAT_LOCK,
                    TC3XX_PLLSTAT_LOCK, TC3XX_PLL_LOCK_TIMEOUT);
  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, TC3XX_PLLSTAT_LOCK,
                    TC3XX_PLLSTAT_LOCK, TC3XX_PLL_LOCK_TIMEOUT);

  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, TC3XX_PLLSTAT_K3RDY,
                    TC3XX_PLLSTAT_K3RDY, TC3XX_PLL_KRDY_TIMEOUT);
  tc3xx_modify(TC3XX_SCU_PERPLLCON1, TC3XX_PLLCON1_K3DIV_MASK,
               TC3XX_PERPLL_K3 << TC3XX_PLLCON1_K3DIV_SHIFT);
  ret |= tc3xx_wait(TC3XX_SCU_PERPLLSTAT, TC3XX_PLLSTAT_K3RDY,
                    TC3XX_PLLSTAT_K3RDY, TC3XX_PLL_KRDY_TIMEOUT);

  tc3xx_smu_ccu_alarms(true);

  ret |= tc3xx_ccucon_write(TC3XX_SCU_CCUCON0,
                            (getreg32(TC3XX_SCU_CCUCON0) &
                             ~TC3XX_CCUCON0_CLKSEL_MASK) |
                            TC3XX_CCUCON0_CLKSEL_PLL | TC3XX_CCUCON_UP);

  tc3xx_wdt_safety_endinit(true);
  return ret;
}

/****************************************************************************
 * Name: tc3xx_clock_distribute
 *
 * Description:
 *   Set the clock dividers.  A clock select of CCUCON1 or CCUCON2 that is
 *   not 0 is first cleared together with the new dividers.
 *
 ****************************************************************************/

static int tc3xx_clock_distribute(void)
{
  const struct tc3xx_ccucon_s *cfg = g_tc3xx_ccucon;
  int ret;
  int i;

  tc3xx_wdt_safety_endinit(false);

  ret = tc3xx_ccucon_write(TC3XX_SCU_CCUCON0,
                           tc3xx_ccucon_value(TC3XX_SCU_CCUCON0, &cfg[0]));

  if ((getreg32(TC3XX_SCU_CCUCON1) & TC3XX_CCUCON1_CLKSEL_MASK) != 0)
    {
      ret |= tc3xx_ccucon_write(TC3XX_SCU_CCUCON1,
                                tc3xx_ccucon_value(TC3XX_SCU_CCUCON1,
                                                   &cfg[1]) &
                                ~TC3XX_CCUCON1_CLKSEL_MASK);
    }

  ret |= tc3xx_ccucon_write(TC3XX_SCU_CCUCON1,
                            tc3xx_ccucon_value(TC3XX_SCU_CCUCON1, &cfg[1]));

  if ((getreg32(TC3XX_SCU_CCUCON2) & TC3XX_CCUCON2_CLKSEL_MASK) != 0)
    {
      ret |= tc3xx_ccucon_write(TC3XX_SCU_CCUCON2,
                                tc3xx_ccucon_value(TC3XX_SCU_CCUCON2,
                                                   &cfg[2]) &
                                ~TC3XX_CCUCON2_CLKSEL_MASK);
    }

  ret |= tc3xx_ccucon_write(TC3XX_SCU_CCUCON2,
                            tc3xx_ccucon_value(TC3XX_SCU_CCUCON2, &cfg[2]));

  ret |= tc3xx_ccucon_write(TC3XX_SCU_CCUCON5,
                            tc3xx_ccucon_value(TC3XX_SCU_CCUCON5, &cfg[3]) |
                            TC3XX_CCUCON_UP);

  for (i = 0; i < 6; i++)
    {
      uintptr_t reg = TC3XX_SCU_CCUCON6 + i * 4;

      putreg32(tc3xx_ccucon_value(reg, &cfg[4 + i]), reg);
    }

  tc3xx_wdt_safety_endinit(true);
  return ret;
}

/****************************************************************************
 * Name: tc3xx_pll_ramp_up
 *
 * Description:
 *   Raise the system PLL to its final frequency in steps of the K2
 *   divider, waiting after each step.
 *
 ****************************************************************************/

static int tc3xx_pll_ramp_up(void)
{
  uint32_t start;
  int ret = 0;
  int i;

  for (i = 0; i < sizeof(g_tc3xx_k2_steps); i++)
    {
      tc3xx_wdt_safety_endinit(false);
      ret |= tc3xx_wait(TC3XX_SCU_SYSPLLSTAT, TC3XX_PLLSTAT_K2RDY,
                        TC3XX_PLLSTAT_K2RDY, TC3XX_PLL_KRDY_TIMEOUT);
      tc3xx_modify(TC3XX_SCU_SYSPLLCON1, TC3XX_PLLCON1_K2DIV_MASK,
                   g_tc3xx_k2_steps[i]);
      tc3xx_wdt_safety_endinit(true);

      start = getreg32(TC3XX_STM0_TIM0);
      while (getreg32(TC3XX_STM0_TIM0) - start < TC3XX_PLL_STEP_WAIT);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_clockconfig
 *
 * Description:
 *   Set up the oscillator, the PLLs and the clock distribution.  Called by
 *   the startup code before the C runtime is initialized, so it may only
 *   use constant data.
 *
 * Returned Value:
 *   0 on success, 1 if a wait for the hardware gave up.
 *
 ****************************************************************************/

int tricore_clockconfig(void)
{
  int ret;

  ret = tc3xx_pll_init();
  if (ret == 0)
    {
      ret = tc3xx_clock_distribute();
    }

  if (ret == 0)
    {
      ret = tc3xx_pll_ramp_up();
    }

  return ret;
}
