/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cgc.c
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

#include "rx65n_macrodriver.h"
#include "rx65n_cgc.h"
#include "arch/board/board.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r_cgc_create
 *
 * Description:
 * Clock generator Initialization
 ****************************************************************************/

void r_cgc_create(void)
{
#ifdef CONFIG_RX65N_RTC
  volatile uint8_t i;
#endif
#if ((24 * RX_CLK_1MHz) == RX_RESONATOR)
  /* Set main clock control registers */

  SYSTEM.MOFCR.BYTE    = _00_CGC_MAINOSC_RESONATOR |
                         _00_CGC_MAINOSC_UNDER24M;
  SYSTEM.MOSCWTCR.BYTE = _5C_CGC_MOSCWTCR_VALUE;

  /* Set main clock operation */

  SYSTEM.MOSCCR.BIT.MOSTP = 0u;

  /* Wait for main clock oscillator wait counter overflow */

  while (1u != SYSTEM.OSCOVFSR.BIT.MOOVF)
    {
      /* Do nothing */
    }

      /* Set system clock */

  SYSTEM.SCKCR.LONG = _00000002_CGC_PCLKD_DIV_4 | _00000020_CGC_PCLKC_DIV_4 |
                      _00000200_CGC_PCLKB_DIV_4 | _00001000_CGC_PCLKA_DIV_2 |
                      _00010000_CGC_BCLK_DIV_2  | _01000000_CGC_ICLK_DIV_2 |
                      _20000000_CGC_FCLK_DIV_4;

  /* Set PLL circuit */

  SYSTEM.PLLCR.WORD   = _0000_CGC_PLL_FREQ_DIV_1 |
                        _0000_CGC_PLL_SOURCE_MAIN |
                        _1300_CGC_PLL_FREQ_MUL_10_0;
  SYSTEM.PLLCR2.BIT.PLLEN = 0u;

  /* Wait for PLL wait counter overflow */

  while (1u != SYSTEM.OSCOVFSR.BIT.PLOVF)
    {
      /* Do nothing */
    }

      /* Stop sub-clock */

  RTC.RCR3.BIT.RTCEN = 0u;

  /* Wait for the register modification to complete */

  while (0u != RTC.RCR3.BIT.RTCEN)
    {
      /* Do nothing */
    }

  /* Stop sub-clock */

  SYSTEM.SOSCCR.BIT.SOSTP = 1u;

  /* Wait for the register modification to complete */

  while (1u != SYSTEM.SOSCCR.BIT.SOSTP)
    {
          /* Do nothing */
    }

  /* Wait for sub-clock oscillation stopping */

  while (0u != SYSTEM.OSCOVFSR.BIT.SOOVF)
    {
      /* Do nothing */
    }

  /* Set UCLK */

  SYSTEM.SCKCR2.WORD = _0040_CGC_UCLK_DIV_5 | _0001_SCKCR2_BIT0;

  /* Set ROM wait cycle */

  SYSTEM.ROMWT.BYTE = _02_CGC_ROMWT_CYCLE_2;

  /* Set SDCLK */

  SYSTEM.SCKCR.BIT.PSTOP0 = 1U;

  /* Set clock source */

  SYSTEM.SCKCR3.WORD = _0400_CGC_CLOCKSOURCE_PLL;

  /* Set LOCO */

  SYSTEM.LOCOCR.BIT.LCSTP = 1U;
#ifdef CONFIG_RX65N_RTC
  RTC.RCR4.BYTE = _00_RTC_SOURCE_SELECT_SUB;
  for (i = 0; i < 4; i++)
    {
      __asm("nop");
    }

  if (0 != RTC.RCR4.BIT.RCKSEL)
    {
      __asm("nop");
    }

  RTC.RCR3.BIT.RTCEN = 0;
  for (i = 0; i < 4; i++)
    {
      __asm("nop");
    }

  if (0 != RTC.RCR3.BIT.RTCEN)
    {
      __asm("nop");
    }

  SYSTEM.SOSCCR.BYTE = 0x01;
  if (0x01 != SYSTEM.SOSCCR.BYTE)
    {
      __asm("nop");
    }

  while (0 != SYSTEM.OSCOVFSR.BIT.SOOVF);
  RTC.RCR3.BIT.RTCDV = 1U;

  /* Wait for the register modification to complete */

  while (1U != RTC.RCR3.BIT.RTCDV)
    {
      /* Do nothing */
    }

  /* Set sub-clock oscillation wait time */

  SYSTEM.SOSCWTCR.BYTE = _25_CGC_SOSCWTCR_VALUE;

  /* Set sub-clock */

  SYSTEM.SOSCCR.BIT.SOSTP = 0U;

  /* Wait for the register modification to complete */

  while (0U != SYSTEM.SOSCCR.BIT.SOSTP)
    {
      /* Do nothing */
    }

  /* Wait for sub-clock to be stable */

  while (1U != SYSTEM.OSCOVFSR.BIT.SOOVF)
    {
      /* Do nothing */
    }
#endif
#elif   ((12 * RX_CLK_1MHz) == RX_RESONATOR)
  SYSTEM.MOFCR.BIT.MOFXIN = 0;
  SYSTEM.MOFCR.BIT.MOSEL  = 0;

  if (1 == SYSTEM.HOCOCR.BIT.HCSTP)
    {
      SYSTEM.HOCOPCR.BYTE = 0x01;
    }

  else
    {
      while (0 == SYSTEM.OSCOVFSR.BIT.HCOVF);
    }

  SYSTEM.MOFCR.BIT.MODRV2 = 2;
  SYSTEM.MOSCWTCR.BYTE = 0x53;
  SYSTEM.MOSCCR.BYTE = 0x00;

  if (0x00 ==  SYSTEM.MOSCCR.BYTE)
    {
      __asm("nop");
    }

  while (0 == SYSTEM.OSCOVFSR.BIT.MOOVF);
  if (0 == SYSTEM.RSTSR1.BIT.CWSF)
    {
#ifdef CONFIG_RX65N_RTC
  RTC.RCR4.BYTE = _00_RTC_SOURCE_SELECT_SUB;
  for (i = 0; i < 4; i++)
    {
      __asm("nop");
    }

  if (0 != RTC.RCR4.BIT.RCKSEL)
    {
      __asm("nop");
    }

  RTC.RCR3.BIT.RTCEN = 0;
  for (i = 0; i < 4; i++)
    {
      __asm("nop");
    }

  if (0 != RTC.RCR3.BIT.RTCEN)
    {
       __asm("nop");
    }

  SYSTEM.SOSCCR.BYTE = 0x01;
  if (0x01 != SYSTEM.SOSCCR.BYTE)
    {
      __asm("nop");
    }

  while (0 != SYSTEM.OSCOVFSR.BIT.SOOVF);
  RTC.RCR3.BIT.RTCDV = 1U;

  /* Wait for the register modification to complete */

  while (1U != RTC.RCR3.BIT.RTCDV)
    {
      /* Do nothing */
    }

  /* Set sub-clock oscillation wait time */

  SYSTEM.SOSCWTCR.BYTE = _25_CGC_SOSCWTCR_VALUE;

  /* Set sub-clock */

  SYSTEM.SOSCCR.BIT.SOSTP = 0U;

  /* Wait for the register modification to complete */

  while (0U != SYSTEM.SOSCCR.BIT.SOSTP)
    {
      /* Do nothing */
    }

  /* Wait for sub-clock to be stable */

  while (1U != SYSTEM.OSCOVFSR.BIT.SOOVF)
    {
      /* Do nothing */
    }
#endif
    }
  else
    {
      SYSTEM.SOSCCR.BYTE = 0x01;
      if (0x01 != SYSTEM.SOSCCR.BYTE)
        {
                 __asm("nop");
        }

      while (0 != SYSTEM.OSCOVFSR.BIT.SOOVF);
    }

  SYSTEM.PLLCR.WORD   = _0000_CGC_PLL_FREQ_DIV_1 |
                        _0000_CGC_PLL_SOURCE_MAIN |
                        _2700_CGC_PLL_FREQ_MUL_20_0;
  SYSTEM.PLLCR2.BYTE =  0x00;
  while (0 == SYSTEM.OSCOVFSR.BIT.PLOVF);
  SYSTEM.ROMWT.BYTE = 0x02;
  if (0x02 == SYSTEM.ROMWT.BYTE)
    {
      __asm("nop");
    }

  SYSTEM.SCKCR.LONG = _00000002_CGC_PCLKD_DIV_4 | _00000020_CGC_PCLKC_DIV_4 |
                       _00000200_CGC_PCLKB_DIV_4 |
                       _00001000_CGC_PCLKA_DIV_2 |
                       _00010000_CGC_BCLK_DIV_2  |
                       _00C00000_CGC_PSTOP0_PSTOP1 |
                       _01000000_CGC_ICLK_DIV_2 |
                       _20000000_CGC_FCLK_DIV_4;

#if defined(CONFIG_USBHOST) || defined(CONFIG_USBDEV)
  SYSTEM.SCKCR2.WORD = _0040_CGC_UCLK_DIV_5 | _0001_SCKCR2_BIT0;
#else
  SYSTEM.SCKCR2.WORD = _0010_CGC_UCLK_DIV_1 | _0001_SCKCR2_BIT0;
#endif
  SYSTEM.SCKCR3.WORD = _0400_CGC_CLOCKSOURCE_PLL;   /* BSP_CFG_CLOCK_SOURCE */

  SYSTEM.LOCOCR.BYTE = 0x01;
#else
#  error "RX_RESONATOR is not defined in board.h"
#endif
}
