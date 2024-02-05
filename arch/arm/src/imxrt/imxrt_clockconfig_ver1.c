/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig_ver1.c
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
#include <arch/board/board.h>
#include "hardware/imxrt_ccm.h"
#include "hardware/imxrt_dcdc.h"
#include "imxrt_clockconfig_ver1.h"
#include "imxrt_lcd.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/imxrt_iomuxc.h"

#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_PLL_MIN_FREQ 650000000
#define OSC24_FREQ         24000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lcd_clockconfig
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LCD
static void imxrt_lcd_clockconfig(void)
{
  uint32_t reg;
  uint32_t reg2;

  int post;
  int pre;

  uint32_t numerator;
  uint32_t denominator;
  uint32_t post_divider;
  uint32_t pre_divider;
  uint32_t loop_divider;
  uint32_t target_freq;
  uint32_t freq_error;

  target_freq = (CONFIG_IMXRT_LCD_HWIDTH +
                 CONFIG_IMXRT_LCD_HPULSE +
                 CONFIG_IMXRT_LCD_HFRONTPORCH +
                 CONFIG_IMXRT_LCD_HBACKPORCH) *
                (CONFIG_IMXRT_LCD_VHEIGHT +
                 CONFIG_IMXRT_LCD_VPULSE +
                 CONFIG_IMXRT_LCD_VFRONTPORCH +
                 CONFIG_IMXRT_LCD_VBACKPORCH) *
                 CONFIG_IMXRT_LCD_REFRESH_FREQ;

  for (post_divider = 1; post_divider < 16; post_divider <<= 1)
    {
      if (IMXRT_LCD_VIDEO_PLL_FREQ * post_divider >= VIDEO_PLL_MIN_FREQ)
        {
          break;
        }
    }

  loop_divider = (IMXRT_LCD_VIDEO_PLL_FREQ * post_divider) / OSC24_FREQ;
  numerator    = (IMXRT_LCD_VIDEO_PLL_FREQ * post_divider) -
                 (loop_divider * OSC24_FREQ);
  denominator  = OSC24_FREQ;

  /* Bypass PLL first */

  modifyreg32(IMXRT_CCM_ANALOG_PLL_VIDEO,
      CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK,
      CCM_ANALOG_PLL_VIDEO_BYPASS |
      CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_REF_24M);

  putreg32(CCM_ANALOG_PLL_VIDEO_NUM_A(numerator),
           IMXRT_CCM_ANALOG_PLL_VIDEO_NUM);
  putreg32(CCM_ANALOG_PLL_VIDEO_DENOM_B(denominator),
           IMXRT_CCM_ANALOG_PLL_VIDEO_DENOM);

  /* Set post divider:
   *
   * ------------------------------------------------------------------------
   * | config->postDivider | PLL_VIDEO[POST_DIV_SELECT]  | MISC2[VIDEO_DIV] |
   * ------------------------------------------------------------------------
   * |         1           |            2                |        0         |
   * ------------------------------------------------------------------------
   * |         2           |            1                |        0         |
   * ------------------------------------------------------------------------
   * |         4           |            2                |        3         |
   * ------------------------------------------------------------------------
   * |         8           |            1                |        3         |
   * ------------------------------------------------------------------------
   * |         16          |            0                |        3         |
   * ------------------------------------------------------------------------
   */

  reg   = getreg32(IMXRT_CCM_ANALOG_PLL_VIDEO);
  reg  &= ~(CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK |
            CCM_ANALOG_PLL_VIDEO_POWERDOWN);
  reg  |= CCM_ANALOG_PLL_VIDEO_ENABLE |
          CCM_ANALOG_PLL_VIDEO_DIV_SELECT(loop_divider);

  reg2  = getreg32(IMXRT_CCM_ANALOG_MISC2);
  reg2 &= ~CCM_ANALOG_MISC2_VIDEO_DIV_MASK;

  switch (post_divider)
    {
    case 16:
      reg  |= CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_DIV4;
      reg2 |= CCM_ANALOG_MISC2_VIDEO_DIV(3);
      break;

    case 8:
      reg  |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV2;
      reg2 |= CCM_ANALOG_MISC2_VIDEO_DIV(3);
      break;

    case 4:
      reg  |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV1;
      reg2 |= CCM_ANALOG_MISC2_VIDEO_DIV(3);
      break;

    case 2:
      reg  |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV2;
      reg2 |= 0;
      break;

    default:
      reg  |= CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_DIV1;
      reg2 |= 0;
      break;
    }

  putreg32(reg, IMXRT_CCM_ANALOG_PLL_VIDEO);

  putreg32(reg2, IMXRT_CCM_ANALOG_MISC2);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_VIDEO) &
      CCM_ANALOG_PLL_VIDEO_LOCK) == 0)
    {
    }

  /* Disable Bypass */

  modifyreg32(IMXRT_CCM_ANALOG_PLL_VIDEO,
      CCM_ANALOG_PLL_VIDEO_BYPASS,
      0);

  freq_error   = IMXRT_LCD_VIDEO_PLL_FREQ;
  pre_divider  = 0;
  post_divider = 0;

  for (post = 0; post < 8; post++)
    {
      for (pre = 0; pre < 8; pre++)
        {
          int32_t temp_error;
          temp_error = labs((post + 1) * (pre + 1) * target_freq -
              IMXRT_LCD_VIDEO_PLL_FREQ);
          if (temp_error < freq_error)
            {
              pre_divider = pre;
              post_divider = post;
              freq_error = temp_error;
            }
        }
    }

  /* Select PLL5 as LCD Clock and set Pre divider. */

  modifyreg32(IMXRT_CCM_CSCDR2,
              CCM_CSCDR2_LCDIF_PRE_CLK_SEL_MASK |
              CCM_CSCDR2_LCDIF_PRED_MASK,
              CCM_CSCDR2_LCDIF_PRE_CLK_SEL_PLL5 |
              CCM_CSCDR2_LCDIF_PRED(pre_divider));

  /* Set Post divider. */

  modifyreg32(IMXRT_CCM_CBCMR, CCM_CBCMR_LCDIF_PODF_MASK,
      CCM_CBCMR_LCDIF_PODF(post_divider));
}

#endif

/****************************************************************************
 * Name: imxrt_pllsetup
 ****************************************************************************/

static void imxrt_pllsetup(void)
{
#ifdef CONFIG_ARCH_FAMILY_IMXRT102x
  uint32_t pll2reg;
#endif
  uint32_t pll3reg;
  uint32_t reg;

#if (defined(CONFIG_ARCH_FAMILY_IMXRT105x) || \
     defined(CONFIG_ARCH_FAMILY_IMXRT106x))

  /* Init Arm PLL1 */

  reg = CCM_ANALOG_PLL_ARM_DIV_SELECT(IMXRT_ARM_PLL_DIV_SELECT) |
        CCM_ANALOG_PLL_ARM_ENABLE;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_ARM);
  while ((getreg32(IMXRT_CCM_ANALOG_PLL_ARM) & CCM_ANALOG_PLL_ARM_LOCK) == 0)
    {
    }

  /* Init Sys PLL2 */

  reg = CCM_ANALOG_PLL_SYS_DIV_SELECT(IMXRT_SYS_PLL_SELECT) |
        CCM_ANALOG_PLL_SYS_ENABLE;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_SYS);
  while ((getreg32(IMXRT_CCM_ANALOG_PLL_SYS) & CCM_ANALOG_PLL_SYS_LOCK) == 0)
    {
    }

  /* Init USB PLL3 */

  /* capture it's original value */

  pll3reg = getreg32(IMXRT_CCM_ANALOG_PFD_480);
  putreg32(pll3reg                         |
           CCM_ANALOG_PFD_480_PFD0_CLKGATE |
           CCM_ANALOG_PFD_480_PFD1_CLKGATE |
           CCM_ANALOG_PFD_480_PFD2_CLKGATE |
           CCM_ANALOG_PFD_480_PFD3_CLKGATE,
           IMXRT_CCM_ANALOG_PFD_480);

  reg = IMXRT_USB1_PLL_DIV_SELECT       |
        CCM_ANALOG_PLL_USB1_ENABLE      |
        CCM_ANALOG_PLL_USB1_EN_USB_CLKS |
        CCM_ANALOG_PLL_USB1_POWER;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_USB1);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_USB1) &
          CCM_ANALOG_PLL_USB1_LOCK) == 0)
    {
    }

  putreg32(pll3reg, IMXRT_CCM_ANALOG_PFD_480);

#ifdef CONFIG_IMXRT_LCD
  /* Init Video PLL5 */

  imxrt_lcd_clockconfig();
#endif

#if defined(CONFIG_IMXRT_ENET)
  /* Init ENET PLL6 */
#   if defined(CONFIG_IMXRT_ENET1)
  reg    = CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_50MHZ |
           CCM_ANALOG_PLL_ENET_ENABLE                 |
#     if defined(IMXRT_MAC_PROVIDES_TXC)
           CCM_ANALOG_PLL_ENET_ENET1_25M_REF_EN;
#     else
           0;
#     endif
#   endif
#   if defined(CONFIG_IMXRT_ENET2)
  reg    = CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT_50MHZ |
           CCM_ANALOG_PLL_ENET_ENABLE                 |
#     if defined(IMXRT_MAC_PROVIDES_TXC)
           CCM_ANALOG_PLL_ENET_ENET2_25M_REF_EN;
#     else
           0;
#     endif
#   endif

  putreg32(reg, IMXRT_CCM_ANALOG_PLL_ENET);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_ENET) &
          CCM_ANALOG_PLL_ENET_LOCK) == 0)
    {
    }
#endif
#elif defined(CONFIG_ARCH_FAMILY_IMXRT102x)
  /* Init Sys PLL2 */

  /* First reset its fractional dividers */

  pll2reg = getreg32(IMXRT_CCM_ANALOG_PFD_528);
  putreg32(pll2reg |
           CCM_ANALOG_PFD_528_PFD0_CLKGATE |
           CCM_ANALOG_PFD_528_PFD1_CLKGATE |
           CCM_ANALOG_PFD_528_PFD2_CLKGATE |
           CCM_ANALOG_PFD_528_PFD3_CLKGATE,
           IMXRT_CCM_ANALOG_PFD_528);

  reg = CCM_ANALOG_PLL_SYS_DIV_SELECT(IMXRT_SYS_PLL_DIV_SELECT) |
        CCM_ANALOG_PLL_SYS_ENABLE;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_SYS);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_SYS) &
          CCM_ANALOG_PLL_SYS_LOCK) == 0)
    {
    }

  putreg32(pll2reg, IMXRT_CCM_ANALOG_PFD_528);

  /* Init USB PLL3 */

  /* capture it's original value */

  pll3reg = getreg32(IMXRT_CCM_ANALOG_PFD_480);
  putreg32(pll3reg                         |
           CCM_ANALOG_PFD_480_PFD0_CLKGATE |
           CCM_ANALOG_PFD_480_PFD1_CLKGATE |
           CCM_ANALOG_PFD_480_PFD2_CLKGATE |
           CCM_ANALOG_PFD_480_PFD3_CLKGATE,
           IMXRT_CCM_ANALOG_PFD_480);

  reg = CCM_ANALOG_PLL_USB1_DIV_SELECT(IMXRT_USB1_PLL_DIV_SELECT) |
        CCM_ANALOG_PLL_USB1_ENABLE | CCM_ANALOG_PLL_USB1_EN_USB_CLKS |
        CCM_ANALOG_PLL_USB1_POWER;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_USB1);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_USB1) &
          CCM_ANALOG_PLL_USB1_LOCK) == 0)
    {
    }

  putreg32(pll3reg, IMXRT_CCM_ANALOG_PFD_480);

  /* Init Audio PLL4 */

  reg = CCM_ANALOG_PLL_AUDIO_DIV_SELECT(IMXRT_AUDIO_PLL_DIV_SELECT) |
        CCM_ANALOG_PLL_AUDIO_ENABLE;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_AUDIO);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_AUDIO) &
          CCM_ANALOG_PLL_AUDIO_LOCK) == 0)
    {
    }

  /* Init ENET PLL6 */

  reg = CCM_ANALOG_PLL_ENET_ENET1_DIV_SELECT_50MHZ |
        CCM_ANALOG_PLL_ENET_ENABLE                 |
        CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN        |
        CCM_ANALOG_PLL_ENET_ENET_500M_REF_EN;

  putreg32(reg, IMXRT_CCM_ANALOG_PLL_ENET);

  while ((getreg32(IMXRT_CCM_ANALOG_PLL_ENET) &
          CCM_ANALOG_PLL_ENET_LOCK) == 0)
    {
    }

#else
#  error Unrecognised IMXRT family member for clock config
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_clockconfig
 *
 * Description:
 *   Called to initialize the i.MXRT.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imxrt_clockconfig(void)
{
  /* Don't change the current basic clock configuration if we are running
   * from SDRAM.  In this case, some bootloader logic has already configured
   * clocking and SDRAM.  We are pretty much committed to using things the
   * way that the bootloader has left them.
   *
   * Note that although this is safe at boot while nothing is using
   * the clocks additional caution is required if at some later date
   * we want to manipulate the PODFs while the system is running
   * (for power minimisation) because changing those is not glitch free.
   */

#ifndef CONFIG_IMXRT_BOOT_SDRAM
  uint32_t reg;

  /* Set clock mux and dividers */

  /* Set PERIPH_CLK2 MUX to OSC */

  reg  = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
  reg |= CCM_CBCMR_PERIPH_CLK2_SEL_OSC_CLK;
  putreg32(reg, IMXRT_CCM_CBCMR);

  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY) != 0)
    {
    }

  /* Set PERIPH_CLK MUX to PERIPH_CLK2 */

  reg  = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_PERIPH_CLK_SEL_MASK;
  reg |= CCM_CBCDR_PERIPH_CLK_SEL(CCM_CBCDR_PERIPH_CLK_SEL_PERIPH_CLK2);
  putreg32(reg, IMXRT_CCM_CBCDR);
  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) != 0)
    {
    }

  /* Set Soc VDD and wait for it to stablise */

  reg  = getreg32(IMXRT_DCDC_REG3);
  reg &= ~(DCDC_REG3_TRG_MASK);
  reg |= DCDC_REG3_TRG(IMXRT_VDD_SOC);
  putreg32(reg, IMXRT_DCDC_REG3);
  while ((getreg32(IMXRT_DCDC_REG0) & DCDC_REG0_STS_DC_OK) == 0)
    {
    }

  /* OK, now nothing is depending on us, configure the PLLs */

  imxrt_pllsetup();

  /* Set Dividers */

  reg  = getreg32(IMXRT_CCM_CACRR);
  reg &= ~CCM_CACRR_ARM_PODF_MASK;
  reg |= CCM_CACRR_ARM_PODF(CCM_PODF_FROM_DIVISOR(IMXRT_ARM_PODF_DIVIDER));
  putreg32(reg, IMXRT_CCM_CACRR);
  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_ARM_PODF_BUSY) != 0)
    {
    }

  reg  = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_AHB_PODF_MASK;
  reg |= CCM_CBCDR_AHB_PODF(CCM_PODF_FROM_DIVISOR(IMXRT_AHB_PODF_DIVIDER));
  putreg32(reg, IMXRT_CCM_CBCDR);
  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_AHB_PODF_BUSY) != 0)
    {
    }

  /* Adjust IPG and PERCLK PODFs. Consumers of these clocks will need to
   * be gated if there are any (there aren't at boot).
   */

  reg  = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_IPG_PODF_MASK;
  reg |= CCM_CBCDR_IPG_PODF(CCM_PODF_FROM_DIVISOR(IMXRT_IPG_PODF_DIVIDER));
  putreg32(reg, IMXRT_CCM_CBCDR);

  reg  = getreg32(IMXRT_CCM_CSCMR1);
  reg &= ~CCM_CSCMR1_PERCLK_PODF_MASK;
  reg |= CCM_CSCMR1_PERCLK_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_PERCLK_PODF_DIVIDER));
  putreg32(reg, IMXRT_CCM_CSCMR1);

#ifndef CONFIG_IMXRT_SEMC_INIT_DONE
  /* Configure SEMC Clock only if not already done by DCD SDR */

  reg  = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_SEMC_PODF_MASK;
  reg |= CCM_CBCDR_SEMC_PODF(CCM_PODF_FROM_DIVISOR(IMXRT_SEMC_PODF_DIVIDER));
  putreg32(reg, IMXRT_CCM_CBCDR);

  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_SEMC_PODF_BUSY) != 0)
    {
    }
#endif

  /* Set PRE_PERIPH_CLK to Board Selection */

  reg  = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK;
  reg |= CCM_CBCMR_PRE_PERIPH_CLK_SEL(IMXRT_PRE_PERIPH_CLK_SEL);
  putreg32(reg, IMXRT_CCM_CBCMR);

  /* Set PERIPH_CLK MUX to Board Selection */

  reg  = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_PERIPH_CLK_SEL_MASK;
  reg |= CCM_CBCDR_PERIPH_CLK_SEL(IMXRT_PERIPH_CLK_SEL);
  putreg32(reg, IMXRT_CCM_CBCDR);

  /* Wait handshake */

  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) != 0)
    {
    }

  /* Set PERCLK_CLK_SEL to Board Selection */

  reg  = getreg32(IMXRT_CCM_CSCMR1);
  reg &= ~CCM_CSCMR1_PERCLK_CLK_SEL_MASK;
  reg |= CCM_CSCMR1_PERCLK_CLK_SEL(IMXRT_PERCLK_CLK_SEL);
  putreg32(reg, IMXRT_CCM_CSCMR1);

  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) != 0)
    {
    }

  /* Setup perhiperals. At this point these are not activated so don't
   * need to worry too much about switching off the clock feeds.
   */

  /* Set UART source to PLL3 80M */

  reg  = getreg32(IMXRT_CCM_CSCDR1);
  reg &= ~CCM_CSCDR1_UART_CLK_SEL;
  reg |= CCM_CSCDR1_UART_CLK_SEL_PLL3_80;
  putreg32(reg, IMXRT_CCM_CSCDR1);

  /* Set UART divider to 1 */

  reg  = getreg32(IMXRT_CCM_CSCDR1);
  reg &= ~CCM_CSCDR1_UART_CLK_PODF_MASK;
  reg |= CCM_CSCDR1_UART_CLK_PODF(CCM_PODF_FROM_DIVISOR(1));
  putreg32(reg, IMXRT_CCM_CSCDR1);

#ifdef CONFIG_IMXRT_FLEXIO1
#ifdef CONFIG_ARCH_FAMILY_IMXRT102x
  /* Set FlEXIO1 source */

  reg = getreg32(IMXRT_CCM_CSCMR2);
  reg &= ~CCM_CSCMR2_FLEXIO1_CLK_SEL_MASK;
  reg |= CCM_CSCMR2_FLEXIO1_CLK_SEL(CONFIG_FLEXIO1_CLK);
  putreg32(reg, IMXRT_CCM_CSCMR2);

  /* Set FlEXIO1 divider */

  reg = getreg32(IMXRT_CCM_CS1CDR);
  reg &= ~(CCM_CS1CDR_FLEXIO1_CLK_PODF_MASK | \
            CCM_CS1CDR_FLEXIO1_CLK_PRED_MASK);
  reg |= CCM_CS1CDR_FLEXIO1_CLK_PODF
            (CCM_PODF_FROM_DIVISOR(CONFIG_FLEXIO1_PODF_DIVIDER));
  reg |= CCM_CS1CDR_FLEXIO1_CLK_PRED
            (CCM_PRED_FROM_DIVISOR(CONFIG_FLEXIO1_PRED_DIVIDER));
  putreg32(reg, IMXRT_CCM_CS1CDR);

#elif (defined(CONFIG_ARCH_FAMILY_IMXRT105x) || \
       defined(CONFIG_ARCH_FAMILY_IMXRT106x))

  /* Set FlEXIO1 source & divider */

  reg = getreg32(IMXRT_CCM_CDCDR);
  reg &= ~(CCM_CDCDR_FLEXIO1_CLK_SEL_MASK |
           CCM_CDCDR_FLEXIO1_CLK_PODF_MASK |
           CCM_CDCDR_FLEXIO1_CLK_PRED_MASK);
  reg |= CCM_CDCDR_FLEXIO1_CLK_SEL(CONFIG_FLEXIO1_CLK);
  reg |= CCM_CDCDR_FLEXIO1_CLK_PODF
            (CCM_PODF_FROM_DIVISOR(CONFIG_FLEXIO1_PODF_DIVIDER));
  reg |= CCM_CDCDR_FLEXIO1_CLK_PRED
            (CCM_PRED_FROM_DIVISOR(CONFIG_FLEXIO1_PRED_DIVIDER));
  putreg32(reg, IMXRT_CCM_CDCDR);

#endif /* CONFIG_ARCH_FAMILY_IMXRT102x */
#endif /* CONFIG_IMXRT_FLEXIO1 */

#if (defined(CONFIG_IMXRT_FLEXIO2) || defined(CONFIG_IMXRT_FLEXIO3))
  /* Set FlEXIO2 source */

  reg = getreg32(IMXRT_CCM_CSCMR2);
  reg &= ~CCM_CSCMR2_FLEXIO2_CLK_SEL_MASK;
  reg |= CCM_CSCMR2_FLEXIO2_CLK_SEL(CONFIG_FLEXIO2_CLK);
  putreg32(reg, IMXRT_CCM_CSCMR2);

  /* Set FlEXIO2 divider */

  reg = getreg32(IMXRT_CCM_CS1CDR);
  reg &= ~(CCM_CS1CDR_FLEXIO2_CLK_PODF_MASK | \
            CCM_CS1CDR_FLEXIO2_CLK_PRED_MASK);
  reg |= CCM_CS1CDR_FLEXIO2_CLK_PODF
            (CCM_PODF_FROM_DIVISOR(CONFIG_FLEXIO2_PODF_DIVIDER));
  reg |= CCM_CS1CDR_FLEXIO2_CLK_PRED
            (CCM_PRED_FROM_DIVISOR(CONFIG_FLEXIO2_PRED_DIVIDER));
  putreg32(reg, IMXRT_CCM_CS1CDR);

#endif /* CONFIG_IMXRT_FLEXIO2 */

#ifdef CONFIG_IMXRT_LPI2C
  /* Set LPI2C source to PLL3 60M */

  reg  = getreg32(IMXRT_CCM_CSCDR2);
  reg &= ~CCM_CSCDR2_LPI2C_CLK_SEL;
  reg |= IMXRT_LPI2C_CLK_SELECT;
  putreg32(reg, IMXRT_CCM_CSCDR2);

  /* Set LPI2C divider to 5  for 12 MHz */

  reg  = getreg32(IMXRT_CCM_CSCDR2);
  reg &= ~CCM_CSCDR2_LPI2C_CLK_PODF_MASK;
  reg |= CCM_CSCDR2_LPI2C_CLK_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_LSI2C_PODF_DIVIDER)
         );
  putreg32(reg, IMXRT_CCM_CSCDR2);

#endif

#ifdef CONFIG_IMXRT_FLEXCAN
  /* Set FlexCAN clock source to PLL3 80M */

  reg = getreg32(IMXRT_CCM_CSCMR2);
  reg &= ~CCM_CSCMR2_CAN_CLK_SEL_MASK;
  reg |= IMXRT_CAN_CLK_SELECT;
  putreg32(reg, IMXRT_CCM_CSCMR2);

  /* Set FlexCAN dividet to 1 for 80 MHz */

  reg  = getreg32(IMXRT_CCM_CSCMR2);
  reg &= ~CCM_CSCMR2_CAN_CLK_PODF_MASK;
  reg |= CCM_CSCMR2_CAN_CLK_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_CAN_PODF_DIVIDER)
         );
  putreg32(reg, IMXRT_CCM_CSCMR2);

#endif

#ifdef CONFIG_IMXRT_LPSPI
  /* Set LPSPI clock source to PLL3 PFD0 */

  reg  = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_LPSPI_CLK_SEL_MASK;
  reg |= IMXRT_LPSPI_CLK_SELECT;
  putreg32(reg, IMXRT_CCM_CBCMR);

  /* Set LPSPI divider to IMXRT_LSPI_PODF_DIVIDER */

  reg  = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_LPSPI_PODF_MASK;
  reg |= CCM_CBCMR_LPSPI_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_LSPI_PODF_DIVIDER)
         );
  putreg32(reg, IMXRT_CCM_CBCMR);
#endif

#ifdef IMXRT_TRACE_PODF_DIVIDER
  /* Set TRACE clock source and speed */

  reg  = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_TRACE_CLK_SEL_MASK;
  reg |= IMXRT_TRACE_CLK_SELECT;
  putreg32(reg, IMXRT_CCM_CBCMR);

  reg  = getreg32(IMXRT_CCM_CSCDR1);
  reg &= ~CCM_CSCDR1_TRACE_PODF_MASK;
  reg |= CCM_CSCDR1_TRACE_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_TRACE_PODF_DIVIDER));
  putreg32(reg, IMXRT_CCM_CSCDR1);
#endif

#ifdef CONFIG_IMXRT_USDHC
  /* Optionally set USDHC1 & 2 to generate clocks
   * from IMXRT_USDHC1_CLK_SELECT
   */

  reg  = getreg32(IMXRT_CCM_CSCMR1);
  reg &= ~(CCM_CSCMR1_USDHC1_CLK_SEL | CCM_CSCMR1_USDHC2_CLK_SEL);
#if defined(IMXRT_USDHC1_CLK_SELECT)
  reg |= IMXRT_USDHC1_CLK_SELECT;
#endif
#if defined(IMXRT_USDHC2_CLK_SELECT)
  reg |= IMXRT_USDHC2_CLK_SELECT;
#endif
  putreg32(reg, IMXRT_CCM_CSCMR1);

  /* Now divide down clocks by IMXRT_USDHC[1|2]_PODF_DIVIDER */

  reg  = getreg32(IMXRT_CCM_CSCDR1);
  reg &= ~(CCM_CSCDR1_USDHC1_PODF_MASK | CCM_CSCDR1_USDHC2_PODF_MASK);
#if defined(IMXRT_USDHC1_PODF_DIVIDER)
  reg |= CCM_CSCDR1_USDHC1_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_USDHC1_PODF_DIVIDER));
#endif
#if defined(IMXRT_USDHC2_PODF_DIVIDER)
  reg |= CCM_CSCDR1_USDHC2_PODF(
           CCM_PODF_FROM_DIVISOR(IMXRT_USDHC2_PODF_DIVIDER));
#endif
  putreg32(reg, IMXRT_CCM_CSCDR1);
#endif

  /* Ensure platform memory clocks remain enabled in WFI */

  reg  = getreg32(IMXRT_CCM_CGPR);
  reg |= CCM_CGPR_INT_MEM_CLK_LPM;
  putreg32(reg, IMXRT_CCM_CGPR);

  /* Remain in run mode */

  modifyreg32(IMXRT_CCM_CLPCR,
              CCM_CLPCR_LPM_MASK,
              CCM_CLPCR_LPM_RUN);
#endif
}
