/****************************************************************************
 * boards/arm/s32k1xx/s32k146evb/src/s32k1xx_periphclocks.c
 *
 *   Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2018 NXP
 *   All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "s32k14x/s32k14x_clocknames.h"
#include "s32k1xx_periphclocks.h"

#include "s32k146evb.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each S32K1XX board must provide the following initialized structure.
 * This is needed to establish the initial peripheral clocking.
 */

const struct peripheral_clock_config_s g_peripheral_clockconfig0[] =
{
  {
    .clkname = FLEXCAN0_CLK,
#ifdef CONFIG_S32K1XX_FLEXCAN0
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = FLEXCAN1_CLK,
#ifdef CONFIG_S32K1XX_FLEXCAN1
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = FLEXCAN2_CLK,
#ifdef CONFIG_S32K1XX_FLEXCAN2
    .clkgate = true,
#else
    .clkgate = false,
#endif
  },
  {
    .clkname = LPI2C0_CLK,
#ifdef CONFIG_S32K1XX_LPI2C0
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = LPSPI0_CLK,
#ifdef CONFIG_S32K1XX_LPSPI0
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = LPSPI1_CLK,
#ifdef CONFIG_S32K1XX_LPSPI1
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = LPSPI2_CLK,
#ifdef CONFIG_S32K1XX_LPSPI2
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = LPUART0_CLK,
#ifdef CONFIG_S32K1XX_LPUART0
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = LPUART1_CLK,
#ifdef CONFIG_S32K1XX_LPUART1
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = LPUART2_CLK,
#ifdef CONFIG_S32K1XX_LPUART2
    .clkgate = true,
#else
    .clkgate = false,
#endif
    .clksrc  = CLK_SRC_SIRC_DIV2,
  },
  {
    .clkname = PORTA_CLK,
    .clkgate = true,
  },
  {
    .clkname = PORTB_CLK,
    .clkgate = true,
  },
  {
    .clkname = PORTC_CLK,
    .clkgate = true,
  },
  {
    .clkname = PORTD_CLK,
    .clkgate = true,
  },
  {
    .clkname = PORTE_CLK,
    .clkgate = true,
  },
};

size_t const num_of_peripheral_clocks_0 =
    sizeof(g_peripheral_clockconfig0) /
    sizeof(g_peripheral_clockconfig0[0]);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
