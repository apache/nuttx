/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_clockconfig.c
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
#include "tc4xx_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock module registers */

#define TC4XX_CLOCK_OSCCON         0xf0064100
#define TC4XX_CLOCK_RAMPCON0       0xf0064200
#define TC4XX_CLOCK_RAMPSTAT       0xf006420c
#define TC4XX_CLOCK_SYSPLLCON0     0xf0064300
#define TC4XX_CLOCK_SYSPLLCON1     0xf0064304
#define TC4XX_CLOCK_SYSPLLSTAT     0xf006430c
#define TC4XX_CLOCK_PERPLLCON0     0xf0064380
#define TC4XX_CLOCK_PERPLLCON1     0xf0064384
#define TC4XX_CLOCK_PERPLLSTAT     0xf006438c
#define TC4XX_CLOCK_CCUCON         0xf0064400
#define TC4XX_CLOCK_CCUSTAT        0xf0064404
#define TC4XX_CLOCK_SYSCCUCON0     0xf0064410
#define TC4XX_CLOCK_SYSCCUCON1     0xf0064414
#define TC4XX_CLOCK_PERCCUCON0     0xf0064420
#define TC4XX_CLOCK_PERCCUCON1     0xf0064424
#define TC4XX_CLOCK_OSCMON1        0xf0064508
#define TC4XX_CLOCK_OSCSTAT        0xf006450c
#define TC4XX_CLOCK_RAMPMON        0xf0064510

#define TC4XX_OSCCON_MODE_SHIFT    8
#define TC4XX_OSCCON_MODE_MASK     (0x7 << TC4XX_OSCCON_MODE_SHIFT)
#define TC4XX_OSCCON_INSEL_SHIFT   24
#define TC4XX_OSCCON_INSEL_MASK    (0x3 << TC4XX_OSCCON_INSEL_SHIFT)

#define TC4XX_OSCMON1_OSCVAL_MASK  (0x7f << 0)
#define TC4XX_OSCMON1_SMONEN       (1 << 8)
#define TC4XX_OSCMON1_PMONEN       (1 << 24)
#define TC4XX_OSCSTAT_WDS_MASK     ((0x3 << 0) | (0x3 << 16))

#define TC4XX_RAMPCON0_UFL_MASK    (0x3ff << 0)
#define TC4XX_RAMPCON0_CMD_SHIFT   16
#define TC4XX_RAMPCON0_CMD_MASK    (0x7 << TC4XX_RAMPCON0_CMD_SHIFT)
#define TC4XX_RAMPCON0_PWR         (1 << 19)
#define TC4XX_RAMPSTAT_FSTAT_SHIFT 16
#define TC4XX_RAMPSTAT_FSTAT_MASK  (0x3 << TC4XX_RAMPSTAT_FSTAT_SHIFT)
#define TC4XX_RAMPSTAT_ACTIVE      (1 << 23)
#define TC4XX_RAMPSTAT_FLLLOCK     (1 << 24)
#define TC4XX_RAMPMON_FLLLDEN      (1 << 28)

#define TC4XX_PLLCON0_PLLPWR       (1 << 0)
#define TC4XX_PLLCON0_NDIV_SHIFT   8
#define TC4XX_PLLCON0_NDIV_MASK    (0x7f << TC4XX_PLLCON0_NDIV_SHIFT)
#define TC4XX_PLLCON0_PDIV_SHIFT   16
#define TC4XX_PLLCON0_PDIV_MASK    (0x7 << TC4XX_PLLCON0_PDIV_SHIFT)
#define TC4XX_PLLSTAT_PWRSTAT      (1 << 0)
#define TC4XX_PLLSTAT_PLLLOCK      (1 << 1)

#define TC4XX_PLLCON1_K2DIV_MASK   (0xf << 0)
#define TC4XX_PLLCON1_K2PRE_SHIFT  4
#define TC4XX_PLLCON1_K2PRE_MASK   (0x3 << TC4XX_PLLCON1_K2PRE_SHIFT)
#define TC4XX_PLLCON1_K3DIV_SHIFT  8
#define TC4XX_PLLCON1_K3DIV_MASK   (0x7 << TC4XX_PLLCON1_K3DIV_SHIFT)
#define TC4XX_PLLCON1_K3PRE_SHIFT  12
#define TC4XX_PLLCON1_K3PRE_MASK   (0xf << TC4XX_PLLCON1_K3PRE_SHIFT)
#define TC4XX_PLLCON1_K4DIV_SHIFT  16          /* PERPLLCON1 only */
#define TC4XX_PLLCON1_K4DIV_MASK   (0x7 << TC4XX_PLLCON1_K4DIV_SHIFT)
#define TC4XX_PLLCON1_K4PRE_SHIFT  20          /* PERPLLCON1 only */
#define TC4XX_PLLCON1_K4PRE_MASK   (0x3 << TC4XX_PLLCON1_K4PRE_SHIFT)

#define TC4XX_CCUSTAT_LCK          (1 << 31)
#define TC4XX_CCU_CLKSELS_MASK     (0x3 << 0)  /* CCUCON and CCUSTAT */
#define TC4XX_CCU_CLKSELS_PLL      0
#define TC4XX_CCU_CLKSELS_BACK     1
#define TC4XX_CCU_CLKSELS_RAMP     2
#define TC4XX_CCU_CLKSELP          (1 << 16)   /* Set: backup clock */
#define TC4XX_CCUCON_UP            (1 << 30)   /* SYSCCUCON0 and 1 */

/* SMU alarms 272 and 273: system and peripheral PLL crystal clock out of
 * range, in alarm group 8 of SMU_SAFE0
 */

#define TC4XX_SMU_SAFE0_CMD        0xf0029000
#define TC4XX_SMU_SAFE0_KEYS       0xf002901c
#define TC4XX_SMU_SAFE0_AGSF8_CON(n) (0xf00291a0 + (n) * 4)
#define TC4XX_SMU_SAFE0_AGSF8_STS  0xf00291b8
#define TC4XX_SMU_KEYS_CFG         0xbc
#define TC4XX_SMU_CMD_ALARM_CLR    5
#define TC4XX_SMU_PLL_ALARMS       ((1 << 16) | (1 << 17))

/* Flash wait states for the final clocks */

#define TC4XX_DMU_HOST_DFWAIT      0xf8040100
#define TC4XX_DMU_HOST_PFWAIT      0xf804011c
#define TC4XX_DMU_RFLASH_MASK      0x1f
#define TC4XX_DFLASH_WAIT          5
#define TC4XX_PFLASH_WAIT          8

/* CPU0 STM timer, 100 MHz while the clocks are set up */

#define TC4XX_STM0_ABS             0xf8800020
#define TC4XX_STM_TICKS_PER_US     100

/* Loop counts before a wait gives up */

#define TC4XX_CCU_LCK_TIMEOUT      0x1000
#define TC4XX_PLL_LCK_TIMEOUT      0x2710
#define TC4XX_PLL_LOCK_TIMEOUT     0xf000
#define TC4XX_RAMP_TIMEOUT         0x1fffff

/* Clock setup: 25 MHz crystal, system PLL 500 MHz, peripheral PLL 160 and
 * 200 MHz, as the iLLD IfxClock default configuration for TC4Dx.
 */

#define TC4XX_XTAL_MHZ             25
#define TC4XX_PLL_INSEL_OSC0       1
#define TC4XX_OSCCON_MASK          (0xff020)   /* Bits 5 and 12 to 19 */
#define TC4XX_OSCCON_VALUE         (1 << 5)
#define TC4XX_RAMP_UFL             400

#define TC4XX_SYSPLL_PDIV          (1 - 1)
#define TC4XX_SYSPLL_NDIV          (40 - 1)
#define TC4XX_SYSPLL_K2            (2 - 1)
#define TC4XX_SYSPLL_K2PRE         (1 - 1)
#define TC4XX_SYSPLL_K3            (2 - 1)
#define TC4XX_SYSPLL_K3PRE         (2 - 1)
#define TC4XX_SYSPLLCON1_INITIAL   0x0000a111

#define TC4XX_PERPLL_PDIV          (1 - 1)
#define TC4XX_PERPLL_NDIV          (32 - 1)
#define TC4XX_PERPLL_K2            (5 - 1)
#define TC4XX_PERPLL_K2PRE         0
#define TC4XX_PERPLL_K3            (2 - 1)
#define TC4XX_PERPLL_K3PRE         10
#define TC4XX_PERPLL_K4            (2 - 1)
#define TC4XX_PERPLL_K4PRE         1
#define TC4XX_PERPLLCON1_PREDIV    0x0010a010
#define TC4XX_PERPLLCON1_DIV       0x00010101

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tc4xx_ccucon_s
{
  uintptr_t reg;
  uint32_t  value;
  uint32_t  mask;
  uint32_t  set;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Clock distribution, written before the PLLs are set up */

static const struct tc4xx_ccucon_s g_tc4xx_ccucon[] =
{
  { TC4XX_CLOCK_SYSCCUCON0, 0x21051225, 0x2f0fffff, TC4XX_CCUCON_UP },
  { TC4XX_CLOCK_SYSCCUCON1, 0x02222002, 0x0ffff00f, TC4XX_CCUCON_UP },
  { TC4XX_CLOCK_PERCCUCON0, 0x12212111, 0xff3f3f3f, 0 },
  { TC4XX_CLOCK_PERCCUCON1, 0x1f115111, 0x1ff1ffff, 0 },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_wait
 *
 * Description:
 *   Wait until all bits of mask in the register equal the bits in value.
 *   Return 1 if the wait gave up after count loops, otherwise 0.
 *
 ****************************************************************************/

static int tc4xx_wait(uintptr_t reg, uint32_t mask, uint32_t value,
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

static int tc4xx_wait_unlocked(uint32_t count)
{
  return tc4xx_wait(TC4XX_CLOCK_CCUSTAT, TC4XX_CCUSTAT_LCK, 0, count);
}

static void tc4xx_modify(uintptr_t reg, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(reg) & ~clear) | set, reg);
}

/****************************************************************************
 * Name: tc4xx_delay
 *
 * Description:
 *   Wait for us microseconds on the STM, servicing the watchdogs.
 *
 ****************************************************************************/

static void tc4xx_delay(uint32_t us)
{
  uint32_t start = getreg32(TC4XX_STM0_ABS);

  while (getreg32(TC4XX_STM0_ABS) - start < us * TC4XX_STM_TICKS_PER_US)
    {
      tc4xx_wdt_service();
    }
}

static void tc4xx_smu_pll_alarms(bool enable)
{
  int i;

  if (enable)
    {
      putreg32(TC4XX_SMU_CMD_ALARM_CLR, TC4XX_SMU_SAFE0_CMD);
      putreg32(1 << 16, TC4XX_SMU_SAFE0_AGSF8_STS);
      putreg32(TC4XX_SMU_CMD_ALARM_CLR, TC4XX_SMU_SAFE0_CMD);
      putreg32(1 << 17, TC4XX_SMU_SAFE0_AGSF8_STS);
      return;
    }

  /* Internal alarm action disabled */

  putreg32(TC4XX_SMU_KEYS_CFG, TC4XX_SMU_SAFE0_KEYS);
  for (i = 0; i < 4; i++)
    {
      tc4xx_modify(TC4XX_SMU_SAFE0_AGSF8_CON(i), TC4XX_SMU_PLL_ALARMS, 0);
    }

  putreg32(0, TC4XX_SMU_SAFE0_KEYS);
}

/****************************************************************************
 * Name: tc4xx_clock_distribute
 ****************************************************************************/

static int tc4xx_clock_distribute(void)
{
  const struct tc4xx_ccucon_s *cfg;
  int ret = 0;
  int i;

  tc4xx_modify(TC4XX_DMU_HOST_DFWAIT, TC4XX_DMU_RFLASH_MASK,
               TC4XX_DFLASH_WAIT);
  tc4xx_modify(TC4XX_DMU_HOST_PFWAIT, TC4XX_DMU_RFLASH_MASK,
               TC4XX_PFLASH_WAIT);

  for (i = 0; i < sizeof(g_tc4xx_ccucon) / sizeof(g_tc4xx_ccucon[0]); i++)
    {
      uint32_t regval;

      cfg    = &g_tc4xx_ccucon[i];
      regval = (getreg32(cfg->reg) & ~cfg->mask) |
               (cfg->value & cfg->mask) | cfg->set;
      ret   |= tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
      putreg32(regval, cfg->reg);
    }

  return ret;
}

/****************************************************************************
 * Name: tc4xx_osc_init
 *
 * Description:
 *   Power up the crystal and the ramp oscillator, select the crystal as
 *   PLL input and wait for it to be stable.
 *
 ****************************************************************************/

static int tc4xx_osc_init(void)
{
  uint32_t mode;
  uint32_t osccon;
  uint32_t count;
  int ret;

  ret  = tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);

  tc4xx_smu_pll_alarms(false);

  mode = (getreg32(TC4XX_CLOCK_OSCCON) & TC4XX_OSCCON_MODE_MASK) >>
         TC4XX_OSCCON_MODE_SHIFT;
  if (mode == 7)
    {
      tc4xx_modify(TC4XX_CLOCK_OSCCON, TC4XX_OSCCON_MODE_MASK,
                   3 << TC4XX_OSCCON_MODE_SHIFT);
      tc4xx_modify(TC4XX_CLOCK_OSCCON, TC4XX_OSCCON_MODE_MASK, 0);
    }
  else if (mode != 2 && mode != 0)
    {
      osccon = getreg32(TC4XX_CLOCK_OSCCON) & ~TC4XX_OSCCON_MODE_MASK;
      tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
      putreg32(osccon, TC4XX_CLOCK_OSCCON);
    }

  if ((getreg32(TC4XX_CLOCK_RAMPSTAT) & TC4XX_RAMPSTAT_ACTIVE) == 0)
    {
      tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
      tc4xx_modify(TC4XX_CLOCK_RAMPCON0, 0, TC4XX_RAMPCON0_PWR);

      count = TC4XX_CCU_LCK_TIMEOUT;
      while ((getreg32(TC4XX_CLOCK_RAMPSTAT) & TC4XX_RAMPSTAT_ACTIVE) == 0 &&
             --count != 0);
    }

  tc4xx_modify(TC4XX_CLOCK_RAMPMON, 0, TC4XX_RAMPMON_FLLLDEN);

  osccon = getreg32(TC4XX_CLOCK_OSCCON) & ~TC4XX_OSCCON_MASK;
  tc4xx_modify(TC4XX_CLOCK_OSCMON1, TC4XX_OSCMON1_OSCVAL_MASK,
               (TC4XX_XTAL_MHZ - 15) | TC4XX_OSCMON1_SMONEN |
               TC4XX_OSCMON1_PMONEN);

  ret |= tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
  putreg32(osccon | TC4XX_OSCCON_VALUE, TC4XX_CLOCK_OSCCON);
  ret |= tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
  tc4xx_modify(TC4XX_CLOCK_OSCCON, TC4XX_OSCCON_INSEL_MASK,
               TC4XX_PLL_INSEL_OSC0 << TC4XX_OSCCON_INSEL_SHIFT);

  /* The crystal may need up to 3 ms to give a stable clock */

  tc4xx_delay(3000);
  if ((getreg32(TC4XX_CLOCK_OSCSTAT) & TC4XX_OSCSTAT_WDS_MASK) != 0)
    {
      ret = 1;
    }

  return ret;
}

/****************************************************************************
 * Name: tc4xx_pll_init
 *
 * Description:
 *   Set up one PLL: power it down, set the initial K dividers, power it up
 *   with P and N, wait for the lock and set the final K dividers.
 *
 * Returned Value:
 *   0 on success, 1 if a wait gave up, -1 if the PLL cannot be powered
 *   down because the clock runs from it.
 *
 ****************************************************************************/

static int tc4xx_pll_init(uintptr_t con0, uintptr_t con1, uintptr_t stat,
                          uint32_t pdiv, uint32_t ndiv, uint32_t prediv,
                          uint32_t premask, uint32_t div, uint32_t divmask,
                          bool sys)
{
  uint32_t regval;
  int ret = 0;

  if ((getreg32(stat) & TC4XX_PLLSTAT_PWRSTAT) != 0)
    {
      regval = getreg32(TC4XX_CLOCK_CCUSTAT);
      if (sys ? ((regval & TC4XX_CCU_CLKSELS_MASK) ==
                 TC4XX_CCU_CLKSELS_PLL ||
                 (regval & TC4XX_CCU_CLKSELS_MASK) == 3) :
                (regval & TC4XX_CCU_CLKSELP) == 0)
        {
          return -1;
        }

      tc4xx_modify(con0, TC4XX_PLLCON0_PLLPWR, 0);
    }

  ret |= tc4xx_wait_unlocked(TC4XX_PLL_LCK_TIMEOUT);
  if (sys)
    {
      putreg32(TC4XX_SYSPLLCON1_INITIAL, con1);
    }
  else
    {
      putreg32(TC4XX_PERPLLCON1_PREDIV, con1);
      ret |= tc4xx_wait_unlocked(TC4XX_PLL_LCK_TIMEOUT);
      tc4xx_modify(con1, 0, TC4XX_PERPLLCON1_DIV);
      ret |= tc4xx_wait_unlocked(TC4XX_PLL_LCK_TIMEOUT);
    }

  /* The peripheral PLL K pre-dividers are set from the value written
   * above, as IfxClock does
   */

  regval = getreg32(con1);

  tc4xx_modify(con0, TC4XX_PLLCON0_PDIV_MASK | TC4XX_PLLCON0_NDIV_MASK,
               (pdiv << TC4XX_PLLCON0_PDIV_SHIFT) |
               (ndiv << TC4XX_PLLCON0_NDIV_SHIFT) | TC4XX_PLLCON0_PLLPWR);
  ret |= tc4xx_wait_unlocked(TC4XX_PLL_LCK_TIMEOUT);
  ret |= tc4xx_wait(stat, TC4XX_PLLSTAT_PWRSTAT, TC4XX_PLLSTAT_PWRSTAT,
                    TC4XX_PLL_LCK_TIMEOUT);

  /* Wait 1 ms to avoid jitter */

  tc4xx_delay(1000);
  ret |= tc4xx_wait(stat, TC4XX_PLLSTAT_PLLLOCK, TC4XX_PLLSTAT_PLLLOCK,
                    TC4XX_PLL_LCK_TIMEOUT);

  if (sys)
    {
      regval = getreg32(con1);
    }

  putreg32((regval & ~premask) | prediv, con1);
  tc4xx_delay(10);
  ret |= tc4xx_wait_unlocked(TC4XX_PLL_LCK_TIMEOUT);

  tc4xx_modify(con1, divmask, div);
  ret |= tc4xx_wait_unlocked(TC4XX_PLL_LCK_TIMEOUT);

  return ret;
}

/****************************************************************************
 * Name: tc4xx_clock_switch
 *
 * Description:
 *   Run the system clock from the ramp oscillator, ramp it up to the PLL
 *   frequency and switch both clocks to the PLLs.
 *
 ****************************************************************************/

static int tc4xx_clock_switch(void)
{
  uint32_t regval;
  int ret;

  tc4xx_smu_pll_alarms(true);

  regval  = getreg32(TC4XX_CLOCK_CCUCON);
  regval &= ~TC4XX_CCU_CLKSELS_MASK;
  regval |= TC4XX_CCU_CLKSELS_RAMP | TC4XX_CCU_CLKSELP;
  ret     = tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
  putreg32(regval, TC4XX_CLOCK_CCUCON);

  ret |= tc4xx_wait(TC4XX_CLOCK_RAMPSTAT, TC4XX_RAMPSTAT_FLLLOCK,
                    TC4XX_RAMPSTAT_FLLLOCK, TC4XX_CCU_LCK_TIMEOUT);

  regval  = getreg32(TC4XX_CLOCK_RAMPCON0);
  regval &= ~(TC4XX_RAMPCON0_UFL_MASK | TC4XX_RAMPCON0_CMD_MASK);
  regval |= TC4XX_RAMP_UFL | (1 << TC4XX_RAMPCON0_CMD_SHIFT);
  ret    |= tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
  putreg32(regval, TC4XX_CLOCK_RAMPCON0);

  /* Ramp frequency at the top */

  ret |= tc4xx_wait(TC4XX_CLOCK_RAMPSTAT, TC4XX_RAMPSTAT_FSTAT_MASK,
                    2 << TC4XX_RAMPSTAT_FSTAT_SHIFT, TC4XX_RAMP_TIMEOUT);
  ret |= tc4xx_wait(TC4XX_CLOCK_SYSPLLSTAT, TC4XX_PLLSTAT_PLLLOCK,
                    TC4XX_PLLSTAT_PLLLOCK, TC4XX_PLL_LOCK_TIMEOUT);
  ret |= tc4xx_wait(TC4XX_CLOCK_PERPLLSTAT, TC4XX_PLLSTAT_PLLLOCK,
                    TC4XX_PLLSTAT_PLLLOCK, TC4XX_PLL_LOCK_TIMEOUT);

  regval  = getreg32(TC4XX_CLOCK_CCUCON);
  regval &= ~(TC4XX_CCU_CLKSELS_MASK | TC4XX_CCU_CLKSELP);
  regval |= TC4XX_CCU_CLKSELS_PLL;
  ret    |= tc4xx_wait_unlocked(TC4XX_CCU_LCK_TIMEOUT);
  putreg32(regval, TC4XX_CLOCK_CCUCON);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_clockconfig
 *
 * Description:
 *   Set up the flash wait states, the clock distribution, the oscillators
 *   and the PLLs.  Called by the startup code before the C runtime is
 *   initialized, so it may only use constant data.
 *
 * Returned Value:
 *   0 on success, 1 if a wait for the hardware gave up.
 *
 ****************************************************************************/

int tricore_clockconfig(void)
{
  int ret;
  int pll;

  ret = tc4xx_clock_distribute();
  if (ret != 0)
    {
      return ret;
    }

  ret |= tc4xx_osc_init();

  pll = tc4xx_pll_init(TC4XX_CLOCK_SYSPLLCON0, TC4XX_CLOCK_SYSPLLCON1,
                       TC4XX_CLOCK_SYSPLLSTAT, TC4XX_SYSPLL_PDIV,
                       TC4XX_SYSPLL_NDIV,
                       (TC4XX_SYSPLL_K2PRE << TC4XX_PLLCON1_K2PRE_SHIFT) |
                       (TC4XX_SYSPLL_K3PRE << TC4XX_PLLCON1_K3PRE_SHIFT),
                       TC4XX_PLLCON1_K2PRE_MASK | TC4XX_PLLCON1_K3PRE_MASK,
                       TC4XX_SYSPLL_K2 |
                       (TC4XX_SYSPLL_K3 << TC4XX_PLLCON1_K3DIV_SHIFT),
                       TC4XX_PLLCON1_K2DIV_MASK | TC4XX_PLLCON1_K3DIV_MASK,
                       true);
  if (pll < 0)
    {
      return 1;
    }

  ret |= pll;
  pll  = tc4xx_pll_init(TC4XX_CLOCK_PERPLLCON0, TC4XX_CLOCK_PERPLLCON1,
                        TC4XX_CLOCK_PERPLLSTAT, TC4XX_PERPLL_PDIV,
                        TC4XX_PERPLL_NDIV,
                        (TC4XX_PERPLL_K2PRE << TC4XX_PLLCON1_K2PRE_SHIFT) |
                        (TC4XX_PERPLL_K3PRE << TC4XX_PLLCON1_K3PRE_SHIFT) |
                        (TC4XX_PERPLL_K4PRE << TC4XX_PLLCON1_K4PRE_SHIFT),
                        TC4XX_PLLCON1_K2PRE_MASK | TC4XX_PLLCON1_K3PRE_MASK |
                        TC4XX_PLLCON1_K4PRE_MASK,
                        TC4XX_PERPLL_K2 |
                        (TC4XX_PERPLL_K3 << TC4XX_PLLCON1_K3DIV_SHIFT) |
                        (TC4XX_PERPLL_K4 << TC4XX_PLLCON1_K4DIV_SHIFT),
                        TC4XX_PLLCON1_K2DIV_MASK | TC4XX_PLLCON1_K3DIV_MASK |
                        TC4XX_PLLCON1_K4DIV_MASK, false);
  if (pll < 0)
    {
      return 1;
    }

  ret |= pll;
  ret |= tc4xx_clock_switch();

  return ret;
}
