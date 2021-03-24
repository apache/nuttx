/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_clkexten.c
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

#include <arch/board/board.h>

#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_enableexten
 *
 * Description:
 *   Enable external enabling for the specified possible clocks.
 *
 ****************************************************************************/

void lpc31_enableexten(enum lpc31_clockid_e clkid)
{
  uint32_t regaddr;
  uint32_t regval;

  switch (clkid)
    {
      case CLKID_DMACLKGATED:      /*  9 DMA_CLK_GATED */
      case CLKID_EVENTROUTERPCLK:  /* 31 EVENT_ROUTER_PCLK */
      case CLKID_ADCPCLK:          /* 32 ADC_PCLK */
      case CLKID_IOCONFPCLK:       /* 35 IOCONF_PCLK */
      case CLKID_CGUPCLK:          /* 36 CGU_PCLK */
      case CLKID_SYSCREGPCLK:      /* 37 SYSCREG_PCLK */
      case CLKID_OTPPCLK:          /* 38 OTP_PCLK (Reserved on LPC313X) */
      case CLKID_PWMPCLKREGS:      /* 46 PWM_PCLK_REGS */
      case CLKID_PCMAPBPCLK:       /* 52 PCM_APB_PCLK */
      case CLKID_SPIPCLKGATED:     /* 57 SPI_PCLK_GATED */
      case CLKID_SPICLKGATED:      /* 90 SPI_CLK_GATED */
      case CLKID_PCMCLKIP:         /* 71 PCM_CLK_IP */
        regaddr = LPC31_CGU_PCR(clkid);
        regval  = getreg32(regaddr);
        regval |= CGU_PCR_EXTENEN;
        putreg32(regval, regaddr);
        break;

      /* Otherwise, force disable for the clocks.
       * NOTE that a larger set will be disabled than will be enabled.
       */

      default:
        lpc31_disableexten(clkid);
        break;
    }
}

/****************************************************************************
 * Name: lpc31_disableexten
 *
 * Description:
 *   Disable external enabling for the specified possible clocks.
 *
 ****************************************************************************/

void lpc31_disableexten(enum lpc31_clockid_e clkid)
{
  uint32_t regaddr;
  uint32_t regval;

  switch (clkid)
    {
      case CLKID_DMACLKGATED:      /*  9 DMA_CLK_GATED */
      case CLKID_EVENTROUTERPCLK:  /* 31 EVENT_ROUTER_PCLK */
      case CLKID_ADCPCLK:          /* 32 ADC_PCLK */
      case CLKID_WDOGPCLK:         /* 34 WDOG_PCLK */
      case CLKID_IOCONFPCLK:       /* 35 IOCONF_PCLK */
      case CLKID_CGUPCLK:          /* 36 CGU_PCLK */
      case CLKID_SYSCREGPCLK:      /* 37 SYSCREG_PCLK */
      case CLKID_OTPPCLK:          /* 38 OTP_PCLK (Reserved on LPC313X) */
      case CLKID_PWMPCLKREGS:      /* 46 PWM_PCLK_REGS */
      case CLKID_I2C0PCLK:         /* 48 I2C0_PCLK */
      case CLKID_I2C1PCLK:         /* 49 I2C1_PCLK */
      case CLKID_PCMAPBPCLK:       /* 52 PCM_APB_PCLK */
      case CLKID_UARTAPBCLK:       /* 53 UART_APB_CLK */
      case CLKID_SPIPCLKGATED:     /* 57 SPI_PCLK_GATED */
      case CLKID_SPICLKGATED:      /* 90 SPI_CLK_GATED */
      case CLKID_PCMCLKIP:         /* 71 PCM_CLK_IP */
      case CLKID_LCDPCLK:          /* 54 LCD_PCLK */
        regaddr = LPC31_CGU_PCR(clkid);
        regval  = getreg32(regaddr);
        regval &= ~CGU_PCR_EXTENEN;
        putreg32(regval, regaddr);
        break;

      default:
        break;
    }
}
