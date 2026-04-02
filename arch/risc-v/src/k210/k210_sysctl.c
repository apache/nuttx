/****************************************************************************
 * arch/risc-v/src/k210/k210_sysctl.c
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
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "hardware/k210_sysctl.h"
#include "hardware/k210_memorymap.h"
#include "k210_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_PLL_INPUT_FREQ    26000000UL

#define K210_PLL_REF_MIN       13671900UL
#define K210_PLL_REF_MAX       1750000000ULL
#define K210_PLL_VCO_MIN       350000000ULL
#define K210_PLL_VCO_MAX       1750000000ULL

#define K210_PLL_NR_MIN        1
#define K210_PLL_NR_MAX        16
#define K210_PLL_NF_MIN        1
#define K210_PLL_NF_MAX        64
#define K210_PLL_OD_MIN        1
#define K210_PLL_OD_MAX        16

#define K210_PLL_LOCK_TIMEOUT_US 50000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct k210_pll_params_s
{
  uint8_t clkr;
  uint8_t clkf;
  uint8_t clkod;
  uint8_t bwadj;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_sysctl_pll_get_freq
 *
 * Description:
 *   Calculate PLL output frequency from register fields.
 *
 *   Formula: FOUT = FIN * (CLKF + 1) / (CLKR + 1) / (CLKOD + 1)
 *   Where FIN = 26MHz (external crystal oscillator)
 *
 * Input Parameters:
 *   pll_offset - PLL register offset (K210_SYSCTL_PLL0/PLL1/PLL2)
 *
 * Returned Value:
 *   PLL output frequency in Hz
 *
 ****************************************************************************/

static uint32_t k210_sysctl_pll_get_freq(uint32_t pll_base)
{
  uint32_t regval = getreg32(pll_base);
  uint32_t clkr = (regval >> PLL_CLKR_SHIFT) & 0x0f;
  uint32_t clkf = (regval >> PLL_CLKF_SHIFT) & 0x3f;
  uint32_t clkod = (regval >> PLL_CLKOD_SHIFT) & 0x0f;

  return (K210_PLL_INPUT_FREQ * (clkf + 1)) / (clkr + 1) / (clkod + 1);
}

/****************************************************************************
 * Name: k210_clock_to_bit
 *
 * Description:
 *   Map a clock ID to its corresponding bit position in CLK_EN_PERI
 *   register.
 *
 * Input Parameters:
 *   clkid - Clock ID to map
 *   bit   - Pointer to store the bit position
 *
 * Returned Value:
 *   OK on success, -EINVAL for invalid clock ID
 *
 ****************************************************************************/

static int k210_clock_to_bit(k210_clockid_t clkid, uint32_t *bit)
{
  switch (clkid)
    {
      case K210_CLOCK_ROM:
        *bit = CLK_EN_PERI_ROM_SHIFT;
        break;
      case K210_CLOCK_DMA:
        *bit = CLK_EN_PERI_DMA_SHIFT;
        break;
      case K210_CLOCK_AI:
        *bit = CLK_EN_PERI_AI_SHIFT;
        break;
      case K210_CLOCK_DVP:
        *bit = CLK_EN_PERI_DVP_SHIFT;
        break;
      case K210_CLOCK_FFT:
        *bit = CLK_EN_PERI_FFT_SHIFT;
        break;
      case K210_CLOCK_GPIO:
        *bit = CLK_EN_PERI_GPIO_SHIFT;
        break;
      case K210_CLOCK_SPI0:
        *bit = CLK_EN_PERI_SPI0_SHIFT;
        break;
      case K210_CLOCK_SPI1:
        *bit = CLK_EN_PERI_SPI1_SHIFT;
        break;
      case K210_CLOCK_SPI2:
        *bit = CLK_EN_PERI_SPI2_SHIFT;
        break;
      case K210_CLOCK_SPI3:
        *bit = CLK_EN_PERI_SPI3_SHIFT;
        break;
      case K210_CLOCK_I2S0:
        *bit = CLK_EN_PERI_I2S0_SHIFT;
        break;
      case K210_CLOCK_I2S1:
        *bit = CLK_EN_PERI_I2S1_SHIFT;
        break;
      case K210_CLOCK_I2S2:
        *bit = CLK_EN_PERI_I2S2_SHIFT;
        break;
      case K210_CLOCK_I2C0:
        *bit = CLK_EN_PERI_I2C0_SHIFT;
        break;
      case K210_CLOCK_I2C1:
        *bit = CLK_EN_PERI_I2C1_SHIFT;
        break;
      case K210_CLOCK_I2C2:
        *bit = CLK_EN_PERI_I2C2_SHIFT;
        break;
      case K210_CLOCK_UART1:
        *bit = CLK_EN_PERI_UART1_SHIFT;
        break;
      case K210_CLOCK_UART2:
        *bit = CLK_EN_PERI_UART2_SHIFT;
        break;
      case K210_CLOCK_UART3:
        *bit = CLK_EN_PERI_UART3_SHIFT;
        break;
      case K210_CLOCK_AES:
        *bit = CLK_EN_PERI_AES_SHIFT;
        break;
      case K210_CLOCK_FPIOA:
        *bit = CLK_EN_PERI_FPIOA_SHIFT;
        break;
      case K210_CLOCK_TIMER0:
        *bit = CLK_EN_PERI_TIMER0_SHIFT;
        break;
      case K210_CLOCK_TIMER1:
        *bit = CLK_EN_PERI_TIMER1_SHIFT;
        break;
      case K210_CLOCK_TIMER2:
        *bit = CLK_EN_PERI_TIMER2_SHIFT;
        break;
      case K210_CLOCK_WDT0:
        *bit = CLK_EN_PERI_WDT0_SHIFT;
        break;
      case K210_CLOCK_WDT1:
        *bit = CLK_EN_PERI_WDT1_SHIFT;
        break;
      case K210_CLOCK_SHA:
        *bit = CLK_EN_PERI_SHA_SHIFT;
        break;
      case K210_CLOCK_OTP:
        *bit = CLK_EN_PERI_OTP_SHIFT;
        break;
      case K210_CLOCK_RTC:
        *bit = CLK_EN_PERI_RTC_SHIFT;
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: k210_reset_to_bit
 *
 * Description:
 *   Map a reset ID to its corresponding bit position in PERI_RESET register.
 *
 * Input Parameters:
 *   rstidx - Reset ID to map
 *   bit    - Pointer to store the bit position
 *
 * Returned Value:
 *   OK on success, -EINVAL for invalid reset ID
 *
 ****************************************************************************/

static int k210_reset_to_bit(k210_rstidx_t rstidx, uint32_t *bit)
{
  switch (rstidx)
    {
      case K210_RESET_ROM:
        *bit = PERI_RESET_ROM_SHIFT;
        break;
      case K210_RESET_DMA:
        *bit = PERI_RESET_DMA_SHIFT;
        break;
      case K210_RESET_AI:
        *bit = PERI_RESET_AI_SHIFT;
        break;
      case K210_RESET_DVP:
        *bit = PERI_RESET_DVP_SHIFT;
        break;
      case K210_RESET_FFT:
        *bit = PERI_RESET_FFT_SHIFT;
        break;
      case K210_RESET_GPIO:
        *bit = PERI_RESET_GPIO_SHIFT;
        break;
      case K210_RESET_SPI0:
        *bit = PERI_RESET_SPI0_SHIFT;
        break;
      case K210_RESET_SPI1:
        *bit = PERI_RESET_SPI1_SHIFT;
        break;
      case K210_RESET_SPI2:
        *bit = PERI_RESET_SPI2_SHIFT;
        break;
      case K210_RESET_SPI3:
        *bit = PERI_RESET_SPI3_SHIFT;
        break;
      case K210_RESET_I2S0:
        *bit = PERI_RESET_I2S0_SHIFT;
        break;
      case K210_RESET_I2S1:
        *bit = PERI_RESET_I2S1_SHIFT;
        break;
      case K210_RESET_I2S2:
        *bit = PERI_RESET_I2S2_SHIFT;
        break;
      case K210_RESET_I2C0:
        *bit = PERI_RESET_I2C0_SHIFT;
        break;
      case K210_RESET_I2C1:
        *bit = PERI_RESET_I2C1_SHIFT;
        break;
      case K210_RESET_I2C2:
        *bit = PERI_RESET_I2C2_SHIFT;
        break;
      case K210_RESET_UART1:
        *bit = PERI_RESET_UART1_SHIFT;
        break;
      case K210_RESET_UART2:
        *bit = PERI_RESET_UART2_SHIFT;
        break;
      case K210_RESET_UART3:
        *bit = PERI_RESET_UART3_SHIFT;
        break;
      case K210_RESET_AES:
        *bit = PERI_RESET_AES_SHIFT;
        break;
      case K210_RESET_FPIOA:
        *bit = PERI_RESET_FPIOA_SHIFT;
        break;
      case K210_RESET_TIMER0:
        *bit = PERI_RESET_TIMER0_SHIFT;
        break;
      case K210_RESET_TIMER1:
        *bit = PERI_RESET_TIMER1_SHIFT;
        break;
      case K210_RESET_TIMER2:
        *bit = PERI_RESET_TIMER2_SHIFT;
        break;
      case K210_RESET_WDT0:
        *bit = PERI_RESET_WDT0_SHIFT;
        break;
      case K210_RESET_WDT1:
        *bit = PERI_RESET_WDT1_SHIFT;
        break;
      case K210_RESET_SHA:
        *bit = PERI_RESET_SHA_SHIFT;
        break;
      case K210_RESET_RTC:
        *bit = PERI_RESET_RTC_SHIFT;
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: k210_clock_to_apb
 *
 * Description:
 *   Map a clock ID to its APB bus number.
 *
 *   APB0: GPIO, UART1-3, I2S0-2, I2C0-2, FPIOA, TIMER0-2, SHA
 *   APB1: AES, WDT0-1, OTP, RTC
 *   APB2: SPI0, SPI1
 *   Others: ROM, DMA, AI, DVP, FFT, SPI2, SPI3 do not require APB bus clock
 *
 * Input Parameters:
 *   clkid - Clock ID to map
 *
 * Returned Value:
 *   0 for APB0, 1 for APB1, 2 for APB2, -1 if not on APB bus
 *
 ****************************************************************************/

static int k210_clock_to_apb(k210_clockid_t clkid)
{
  switch (clkid)
    {
      case K210_CLOCK_GPIO:
      case K210_CLOCK_UART1:
      case K210_CLOCK_UART2:
      case K210_CLOCK_UART3:
      case K210_CLOCK_I2S0:
      case K210_CLOCK_I2S1:
      case K210_CLOCK_I2S2:
      case K210_CLOCK_I2C0:
      case K210_CLOCK_I2C1:
      case K210_CLOCK_I2C2:
      case K210_CLOCK_FPIOA:
      case K210_CLOCK_TIMER0:
      case K210_CLOCK_TIMER1:
      case K210_CLOCK_TIMER2:
      case K210_CLOCK_SHA:
        return 0;

      case K210_CLOCK_AES:
      case K210_CLOCK_WDT0:
      case K210_CLOCK_WDT1:
      case K210_CLOCK_OTP:
      case K210_CLOCK_RTC:
        return 1;

      case K210_CLOCK_SPI0:
      case K210_CLOCK_SPI1:
        return 2;

      default:
        return -1;
    }
}

/****************************************************************************
 * Name: k210_clock_is_central
 *
 * Description:
 *   Check if a clock ID is a central clock (CPU, SRAM, APB).
 *
 *   Central clocks are controlled via CLK_EN_CENT register, while
 *   peripheral clocks use CLK_EN_PERI register.
 *
 * Input Parameters:
 *   clkid - Clock ID to check
 *
 * Returned Value:
 *   true if central clock, false otherwise
 *
 ****************************************************************************/

static bool k210_clock_is_central(k210_clockid_t clkid)
{
  switch (clkid)
    {
      case K210_CLOCK_CPU:
      case K210_CLOCK_SRAM0:
      case K210_CLOCK_SRAM1:
      case K210_CLOCK_APB0:
      case K210_CLOCK_APB1:
      case K210_CLOCK_APB2:
        return true;
      default:
        return false;
    }
}

/****************************************************************************
 * Name: k210_clock_to_central_bit
 *
 * Description:
 *   Map a central clock ID to its corresponding bit mask in CLK_EN_CENT.
 *
 * Input Parameters:
 *   clkid - Central clock ID to map
 *   mask  - Pointer to store the bit mask
 *
 * Returned Value:
 *   OK on success, -EINVAL for invalid clock ID
 *
 ****************************************************************************/

static int k210_clock_to_central_bit(k210_clockid_t clkid, uint32_t *mask)
{
  switch (clkid)
    {
      case K210_CLOCK_CPU:
        *mask = CLK_EN_CENT_CPU_MASK;
        break;
      case K210_CLOCK_SRAM0:
        *mask = CLK_EN_CENT_SRAM0_MASK;
        break;
      case K210_CLOCK_SRAM1:
        *mask = CLK_EN_CENT_SRAM1_MASK;
        break;
      case K210_CLOCK_APB0:
        *mask = CLK_EN_CENT_APB0_MASK;
        break;
      case K210_CLOCK_APB1:
        *mask = CLK_EN_CENT_APB1_MASK;
        break;
      case K210_CLOCK_APB2:
        *mask = CLK_EN_CENT_APB2_MASK;
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: k210_sysctl_enable_apb_clock
 *
 * Description:
 *   Enable the APB bus clock for a peripheral.
 *
 *   K210 peripheral clock enable requires a two-stage process:
 *   1. First enable the corresponding APB bus clock (CLK_EN_CENT)
 *   2. Then enable the peripheral clock (CLK_EN_PERI)
 *
 *   This separation prevents accidental APB clock disable when CPU
 *   manipulates peripheral clock bits.
 *
 * Input Parameters:
 *   clkid - Clock ID of the peripheral
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int k210_sysctl_enable_apb_clock(k210_clockid_t clkid)
{
  int apb = k210_clock_to_apb(clkid);
  uint32_t regval;

  if (apb < 0)
    {
      return OK;
    }

  regval = getreg32(K210_SYSCTL_CLK_EN_CENT);

  switch (apb)
    {
      case 0:
        regval |= CLK_EN_CENT_APB0_MASK;
        break;
      case 1:
        regval |= CLK_EN_CENT_APB1_MASK;
        break;
      case 2:
        regval |= CLK_EN_CENT_APB2_MASK;
        break;
    }

  putreg32(regval, K210_SYSCTL_CLK_EN_CENT);
  return OK;
}

/****************************************************************************
 * Name: k210_sysctl_pll_wait_lock
 ****************************************************************************/

static void k210_sysctl_pll_clear_slip(uint32_t pll_base)
{
  uint32_t regval;

  regval = getreg32(K210_SYSCTL_PLL_LOCK);

  if (pll_base == K210_SYSCTL_PLL0)
    {
      regval |= PLL_LOCK_PLL0_SLIP_CLR;
    }
  else if (pll_base == K210_SYSCTL_PLL1)
    {
      regval |= PLL_LOCK_PLL1_SLIP_CLR;
    }
  else if (pll_base == K210_SYSCTL_PLL2)
    {
      regval |= PLL_LOCK_PLL2_SLIP_CLR;
    }

  putreg32(regval, K210_SYSCTL_PLL_LOCK);
}

static bool k210_sysctl_pll_wait_lock(uint32_t pll_base)
{
  uint32_t timeout = K210_PLL_LOCK_TIMEOUT_US;

  while (timeout-- > 0)
    {
      if (k210_sysctl_pll_is_locked(pll_base))
        {
          return true;
        }

      k210_sysctl_pll_clear_slip(pll_base);
      up_udelay(1);
    }

  return false;
}

/****************************************************************************
 * Name: k210_sysctl_pll_calc_params
 ****************************************************************************/

static bool k210_sysctl_pll_calc_params(uint32_t fin, uint32_t fout,
                                        struct k210_pll_params_s *params)
{
  uint32_t nr;
  uint32_t nf;
  uint32_t od;
  uint64_t best_err = UINT64_MAX;
  uint64_t best_vco = 0;
  uint32_t best_nr = 0;
  bool found = false;

  if (fout == 0 || params == NULL)
    {
      return false;
    }

  for (nr = K210_PLL_NR_MIN; nr <= K210_PLL_NR_MAX; nr++)
    {
      uint64_t ref_hz = fin / nr;

      if (ref_hz < K210_PLL_REF_MIN || ref_hz > K210_PLL_REF_MAX)
        {
          continue;
        }

      for (nf = K210_PLL_NF_MIN; nf <= K210_PLL_NF_MAX; nf++)
        {
          uint64_t vco_hz = ((uint64_t)fin * nf) / nr;

          if (vco_hz < K210_PLL_VCO_MIN || vco_hz > K210_PLL_VCO_MAX)
            {
              continue;
            }

          for (od = K210_PLL_OD_MIN; od <= K210_PLL_OD_MAX; od++)
            {
              uint64_t out_hz = vco_hz / od;
              uint64_t err = out_hz > fout ? out_hz - fout : fout - out_hz;

              if (!found ||
                  err < best_err ||
                  (err == best_err && vco_hz > best_vco) ||
                  (err == best_err && vco_hz == best_vco && nr < best_nr))
                {
                  found = true;
                  best_err = err;
                  best_vco = vco_hz;
                  best_nr = nr;

                  params->clkr = nr - 1;
                  params->clkf = nf - 1;
                  params->clkod = od - 1;
                  params->bwadj = nf - 1;

                  if (err == 0)
                    {
                      return true;
                    }
                }
            }
        }
    }

  return found;
}

/****************************************************************************
 * Name: k210_sysctl_pll_set_freq
 ****************************************************************************/

static uint32_t k210_sysctl_pll_set_freq(uint32_t pll_base, uint32_t freq)
{
  struct k210_pll_params_s params =
  {
    0
  };

  uint32_t regval;
  uint32_t clksel0;

  if (freq == 0 || pll_base != K210_SYSCTL_PLL0)
    {
      return 0;
    }

  if (!k210_sysctl_pll_calc_params(K210_PLL_INPUT_FREQ, freq, &params))
    {
      return 0;
    }

  /* 1. Switch CPU clock to IN0 before touching PLL0 */

  clksel0 = getreg32(K210_SYSCTL_CLKSEL0);
  clksel0 &= ~CLKSEL0_ACLK_SEL_MASK;
  putreg32(clksel0, K210_SYSCTL_CLKSEL0);

  /* 2. Disable PLL output */

  regval = getreg32(pll_base);
  regval &= ~PLL_OUT_EN_MASK;
  putreg32(regval, pll_base);

  /* 3. Power down PLL */

  regval &= ~PLL_PWRD_MASK;
  putreg32(regval, pll_base);

  /* 4. Program PLL parameters */

  regval &= ~(PLL_CLKR_MASK | PLL_CLKF_MASK | PLL_CLKOD_MASK |
              PLL_BWADJ_MASK | PLL_BYPASS_MASK);
  regval |= ((uint32_t)params.clkr << PLL_CLKR_SHIFT);
  regval |= ((uint32_t)params.clkf << PLL_CLKF_SHIFT);
  regval |= ((uint32_t)params.clkod << PLL_CLKOD_SHIFT);
  regval |= ((uint32_t)params.bwadj << PLL_BWADJ_SHIFT);
  putreg32(regval, pll_base);

  /* 5. Power on PLL */

  regval |= PLL_PWRD_MASK;
  putreg32(regval, pll_base);
  up_udelay(1);

  /* 6. Reset PLL and release reset */

  regval &= ~PLL_RESET_MASK;
  putreg32(regval, pll_base);
  regval |= PLL_RESET_MASK;
  putreg32(regval, pll_base);
  up_udelay(1);
  regval &= ~PLL_RESET_MASK;
  putreg32(regval, pll_base);

  /* 7. Wait lock with timeout */

  if (!k210_sysctl_pll_wait_lock(pll_base))
    {
      return 0;
    }

  /* 8. Enable PLL output */

  regval = getreg32(pll_base);
  regval |= PLL_OUT_EN_MASK;
  putreg32(regval, pll_base);

  /* 9. Switch CPU clock back to PLL0 */

  clksel0 = getreg32(K210_SYSCTL_CLKSEL0);
  clksel0 |= CLKSEL0_ACLK_SEL_MASK;
  putreg32(clksel0, K210_SYSCTL_CLKSEL0);

  return k210_sysctl_pll_get_freq(pll_base);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_sysctl_pll_is_locked
 *
 * Description:
 *   Check if a PLL is locked by reading the PLL_LOCK register.
 *
 *   PLL0 requires both overflow and slip bits to be set (value == 3).
 *   PLL1 and PLL2 only check the lock bit.
 *
 * Input Parameters:
 *   pll_offset - PLL register offset (K210_SYSCTL_PLL0/PLL1/PLL2)
 *
 * Returned Value:
 *   true if PLL is locked, false otherwise
 *
 ****************************************************************************/

bool k210_sysctl_pll_is_locked(uint32_t pll_offset)
{
  uint32_t regval = getreg32(K210_SYSCTL_PLL_LOCK);

  if (pll_offset == K210_SYSCTL_PLL0)
    {
      return (regval & (1 << PLL_LOCK_PLL0_SHIFT)) != 0;
    }
  else if (pll_offset == K210_SYSCTL_PLL1)
    {
      return (regval & PLL_LOCK_PLL1_MASK) == PLL_LOCK_PLL1_LOCKED;
    }
  else if (pll_offset == K210_SYSCTL_PLL2)
    {
      return (regval & PLL_LOCK_PLL2_MASK) == PLL_LOCK_PLL2_LOCKED;
    }

  return false;
}

/****************************************************************************
 * Name: k210_sysctl_init
 ****************************************************************************/

void k210_sysctl_init(void)
{
  uint32_t clksel0;

  /* Default to ACLK divider /2 */

  clksel0 = getreg32(K210_SYSCTL_CLKSEL0);
  clksel0 &= ~CLKSEL0_ACLK_DIV_MASK;
  putreg32(clksel0, K210_SYSCTL_CLKSEL0);

  k210_sysctl_cpu_set_freq(CONFIG_K210_CPU_FREQ);
}

/****************************************************************************
 * Name: k210_sysctl_cpu_get_freq
 ****************************************************************************/

uint32_t k210_sysctl_cpu_get_freq(void)
{
  return k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
}

/****************************************************************************
 * Name: k210_sysctl_cpu_set_freq
 ****************************************************************************/

uint32_t k210_sysctl_cpu_set_freq(uint32_t freq)
{
  uint32_t actual_pll;
  uint32_t actual;
  uint32_t clksel0;
  uint32_t div_sel;
  uint32_t target_pll;
  uint64_t target64;

  if (freq == 0)
    {
      return 0;
    }

  clksel0 = getreg32(K210_SYSCTL_CLKSEL0);
  div_sel = (clksel0 & CLKSEL0_ACLK_DIV_MASK) >> CLKSEL0_ACLK_DIV_SHIFT;
  target64 = (uint64_t)freq * (uint64_t)(2u << div_sel);
  if (target64 > UINT32_MAX)
    {
      return 0;
    }

  target_pll = (uint32_t)target64;
  actual_pll = k210_sysctl_pll_set_freq(K210_SYSCTL_PLL0, target_pll);

  if (actual_pll == 0)
    {
      return 0;
    }

  actual = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
  return actual;
}

/****************************************************************************
 * Name: k210_sysctl_clock_enable
 *
 * Description:
 *   Enable a peripheral or central clock.
 *
 *   For peripheral clocks, this follows a two-stage process:
 *   1. Enable the corresponding APB bus clock (CLK_EN_CENT)
 *   2. Enable the peripheral clock (CLK_EN_PERI)
 *
 *   For central clocks (CPU, SRAM, APB), only CLK_EN_CENT is used.
 *
 * Input Parameters:
 *   clkid - Clock ID to enable
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int k210_sysctl_clock_enable(k210_clockid_t clkid)
{
  uint32_t regval;
  uint32_t bit;
  int ret;

  if (clkid >= K210_CLOCK_MAX)
    {
      return -EINVAL;
    }

  if (k210_clock_is_central(clkid))
    {
      ret = k210_clock_to_central_bit(clkid, &bit);
      if (ret < 0)
        {
          return ret;
        }

      regval = getreg32(K210_SYSCTL_CLK_EN_CENT);
      regval |= bit;
      putreg32(regval, K210_SYSCTL_CLK_EN_CENT);
    }
  else
    {
      ret = k210_sysctl_enable_apb_clock(clkid);
      if (ret < 0)
        {
          return ret;
        }

      ret = k210_clock_to_bit(clkid, &bit);
      if (ret < 0)
        {
          return ret;
        }

      regval = getreg32(K210_SYSCTL_CLK_EN_PERI);
      regval |= (1 << bit);
      putreg32(regval, K210_SYSCTL_CLK_EN_PERI);
    }

  return OK;
}

/****************************************************************************
 * Name: k210_sysctl_clock_disable
 *
 * Description:
 *   Disable a peripheral or central clock.
 *
 *   For peripheral clocks, only the peripheral clock is disabled.
 *   The APB bus clock is kept enabled to protect other peripherals
 *   on the same bus.
 *
 *   For central clocks (CPU, SRAM, APB), CLK_EN_CENT is used.
 *   WARNING: Disabling central clocks can cause system instability.
 *
 * Input Parameters:
 *   clkid - Clock ID to disable
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int k210_sysctl_clock_disable(k210_clockid_t clkid)
{
  uint32_t regval;
  uint32_t bit;
  int ret;

  if (clkid >= K210_CLOCK_MAX)
    {
      return -EINVAL;
    }

  if (k210_clock_is_central(clkid))
    {
      ret = k210_clock_to_central_bit(clkid, &bit);
      if (ret < 0)
        {
          return ret;
        }

      regval = getreg32(K210_SYSCTL_CLK_EN_CENT);
      regval &= ~bit;
      putreg32(regval, K210_SYSCTL_CLK_EN_CENT);
    }
  else
    {
      ret = k210_clock_to_bit(clkid, &bit);
      if (ret < 0)
        {
          return ret;
        }

      regval = getreg32(K210_SYSCTL_CLK_EN_PERI);
      regval &= ~(1 << bit);
      putreg32(regval, K210_SYSCTL_CLK_EN_PERI);
    }

  return OK;
}

/****************************************************************************
 * Name: k210_sysctl_clock_get_freq
 *
 * Description:
 *   Get the frequency of a clock.
 *
 *   Clock frequency relationships:
 *   - PLL0/1/2: Calculated from PLL registers
 *   - CPU (ACLK): PLL0 / (2^aclk_div) when aclk_sel=1, else IN0
 *   - APB0: ACLK / (apb0_div + 1)
 *   - APB1: ACLK / (apb1_div + 1)
 *   - APB2: ACLK / (apb2_div + 1)
 *   - Peripheral clocks: Equal to corresponding APB bus frequency
 *
 * Input Parameters:
 *   clkid - Clock ID to query
 *
 * Returned Value:
 *   Clock frequency in Hz, 0 for unknown clocks
 *
 ****************************************************************************/

uint32_t k210_sysctl_clock_get_freq(k210_clockid_t clkid)
{
  uint32_t regval;
  uint32_t div;
  uint32_t source;

  switch (clkid)
    {
      case K210_CLOCK_PLL0:
        return k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);

      case K210_CLOCK_PLL1:
        return k210_sysctl_pll_get_freq(K210_SYSCTL_PLL1);

      case K210_CLOCK_PLL2:
        return k210_sysctl_pll_get_freq(K210_SYSCTL_PLL2);

      case K210_CLOCK_CPU:
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        if (regval & CLKSEL0_ACLK_SEL_MASK)
          {
            source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
            div = (regval >> CLKSEL0_ACLK_DIV_SHIFT) & 0x03;
            return source / (2u << div);
          }
        else
          {
            return K210_PLL_INPUT_FREQ;
          }

      case K210_CLOCK_SRAM0:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLK_TH0);
        div = (regval >> CLK_TH0_SRAM0_SHIFT) & 0x0f;
        return source / (div + 1);

      case K210_CLOCK_SRAM1:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLK_TH0);
        div = (regval >> CLK_TH0_SRAM1_SHIFT) & 0x0f;
        return source / (div + 1);

      case K210_CLOCK_ROM:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLK_TH0);
        div = (regval >> CLK_TH0_ROM_SHIFT) & 0x0f;
        return source / (div + 1);

      case K210_CLOCK_DVP:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLK_TH0);
        div = (regval >> CLK_TH0_DVP_SHIFT) & 0x0f;
        return source / (div + 1);

      case K210_CLOCK_APB0:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        div = (regval >> CLKSEL0_APB0_DIV_SHIFT) & 0x07;
        return source / (div + 1);

      case K210_CLOCK_APB1:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        div = (regval >> CLKSEL0_APB1_DIV_SHIFT) & 0x07;
        return source / (div + 1);

      case K210_CLOCK_APB2:
        source = k210_sysctl_clock_get_freq(K210_CLOCK_CPU);
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        div = (regval >> CLKSEL0_APB2_DIV_SHIFT) & 0x07;
        return source / (div + 1);

      case K210_CLOCK_AI:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL1);
        regval = getreg32(K210_SYSCTL_CLK_TH0);
        div = (regval >> CLK_TH0_AI_SHIFT) & 0x0f;
        return source / (div + 1);

      case K210_CLOCK_DMA:
      case K210_CLOCK_FFT:
        return k210_sysctl_clock_get_freq(K210_CLOCK_CPU);

      case K210_CLOCK_SPI0:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
        regval = getreg32(K210_SYSCTL_CLK_TH1);
        div = (regval >> CLK_TH1_SPI0_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_SPI1:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
        regval = getreg32(K210_SYSCTL_CLK_TH1);
        div = (regval >> CLK_TH1_SPI1_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_SPI2:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
        regval = getreg32(K210_SYSCTL_CLK_TH1);
        div = (regval >> CLK_TH1_SPI2_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_SPI3:
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        if ((regval >> 12) & 0x01)
          {
            source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
          }
        else
          {
            source = K210_PLL_INPUT_FREQ;
          }

        regval = getreg32(K210_SYSCTL_CLK_TH1);
        div = (regval >> CLK_TH1_SPI3_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_I2S0:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL2);
        regval = getreg32(K210_SYSCTL_CLK_TH3);
        div = (regval >> CLK_TH3_I2S0_SHIFT) & 0xffff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_I2S1:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL2);
        regval = getreg32(K210_SYSCTL_CLK_TH3);
        div = (regval >> CLK_TH3_I2S1_SHIFT) & 0xffff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_I2S2:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL2);
        regval = getreg32(K210_SYSCTL_CLK_TH4);
        div = (regval >> CLK_TH4_I2S2_SHIFT) & 0xffff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_I2C0:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
        regval = getreg32(K210_SYSCTL_CLK_TH5);
        div = (regval >> CLK_TH5_I2C0_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_I2C1:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
        regval = getreg32(K210_SYSCTL_CLK_TH5);
        div = (regval >> CLK_TH5_I2C1_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_I2C2:
        source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
        regval = getreg32(K210_SYSCTL_CLK_TH5);
        div = (regval >> CLK_TH5_I2C2_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_TIMER0:
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        if ((regval >> 13) & 0x01)
          {
            source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
          }
        else
          {
            source = K210_PLL_INPUT_FREQ;
          }

        regval = getreg32(K210_SYSCTL_CLK_TH2);
        div = (regval >> CLK_TH2_TIMER0_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_TIMER1:
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        if ((regval >> 14) & 0x01)
          {
            source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
          }
        else
          {
            source = K210_PLL_INPUT_FREQ;
          }

        regval = getreg32(K210_SYSCTL_CLK_TH2);
        div = (regval >> CLK_TH2_TIMER1_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_TIMER2:
        regval = getreg32(K210_SYSCTL_CLKSEL0);
        if ((regval >> 15) & 0x01)
          {
            source = k210_sysctl_pll_get_freq(K210_SYSCTL_PLL0);
          }
        else
          {
            source = K210_PLL_INPUT_FREQ;
          }

        regval = getreg32(K210_SYSCTL_CLK_TH2);
        div = (regval >> CLK_TH2_TIMER2_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_WDT0:
        source = K210_PLL_INPUT_FREQ;
        regval = getreg32(K210_SYSCTL_CLK_TH6);
        div = (regval >> CLK_TH6_WDT0_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_WDT1:
        source = K210_PLL_INPUT_FREQ;
        regval = getreg32(K210_SYSCTL_CLK_TH6);
        div = (regval >> CLK_TH6_WDT1_SHIFT) & 0xff;
        return source / ((div + 1) * 2);

      case K210_CLOCK_GPIO:
      case K210_CLOCK_UART1:
      case K210_CLOCK_UART2:
      case K210_CLOCK_UART3:
      case K210_CLOCK_FPIOA:
      case K210_CLOCK_SHA:
        return k210_sysctl_clock_get_freq(K210_CLOCK_APB0);

      case K210_CLOCK_AES:
      case K210_CLOCK_OTP:
        return k210_sysctl_clock_get_freq(K210_CLOCK_APB1);

      case K210_CLOCK_RTC:
        return K210_PLL_INPUT_FREQ;

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: k210_sysctl_reset
 *
 * Description:
 *   Reset a peripheral with proper timing sequence.
 *
 *   Reset sequence:
 *   1. Assert reset (set bit in PERI_RESET register)
 *   2. Wait approximately 10us for reset to take effect
 *   3. Deassert reset (clear bit in PERI_RESET register)
 *
 * Input Parameters:
 *   rstidx - Reset ID of the peripheral to reset
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int k210_sysctl_reset(k210_rstidx_t rstidx)
{
  uint32_t bit;
  uint32_t regval;
  int ret;

  ret = k210_reset_to_bit(rstidx, &bit);
  if (ret < 0)
    {
      return ret;
    }

  regval = getreg32(K210_SYSCTL_PERI_RESET);

  regval |= (1 << bit);
  putreg32(regval, K210_SYSCTL_PERI_RESET);

  up_udelay(10);

  regval &= ~(1 << bit);
  putreg32(regval, K210_SYSCTL_PERI_RESET);

  return OK;
}

/****************************************************************************
 * Name: k210_sysctl_init_peripheral
 *
 * Description:
 *   Initialize a peripheral by enabling its clock and releasing from reset.
 *
 *   This is a convenience function that combines clock enable and reset
 *   operations for simplified peripheral initialization.
 *
 *   Typical usage:
 *     k210_sysctl_init_peripheral(K210_CLOCK_UART1, K210_RESET_UART1);
 *     k210_sysctl_init_peripheral(K210_CLOCK_SPI0, K210_RESET_SPI0);
 *
 * Input Parameters:
 *   clkid  - Clock ID of the peripheral
 *   rstidx - Reset ID of the peripheral
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int k210_sysctl_init_peripheral(k210_clockid_t clkid, k210_rstidx_t rstidx)
{
  int ret;

  ret = k210_sysctl_clock_enable(clkid);
  if (ret < 0)
    {
      return ret;
    }

  ret = k210_sysctl_reset(rstidx);
  if (ret < 0)
    {
      k210_sysctl_clock_disable(clkid);
      return ret;
    }

  return OK;
}
