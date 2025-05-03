/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/zynq_pll.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include "arm64_internal.h"
#include "chip.h"
#include "hardware/zynq_memorymap.h"

#include "hardware/zynq_pll.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Register definitions of clock source
 ****************************************************************************/

#define MPSOC_CLK_MAXDIV        0x3f
#define CLK_CTRL_DIV1_SHIFT     16
#define CLK_CTRL_DIV1_MASK      (MPSOC_CLK_MAXDIV << CLK_CTRL_DIV1_SHIFT)
#define CLK_CTRL_DIV0_SHIFT     8
#define CLK_CTRL_DIV0_MASK      (MPSOC_CLK_MAXDIV << CLK_CTRL_DIV0_SHIFT)
#define CLK_CTRL_SRCSEL_SHIFT   0
#define CLK_CTRL_SRCSEL_MASK    (0x3 << CLK_CTRL_SRCSEL_SHIFT)

#define PLLCTRL_BYPASS_MASK     (0x1 << 3)
#define PLLCTRL_RESET_SHIFT     0
#define PLLCTRL_RESET_MASK      (0x01 << PLLCTRL_RESET_SHIFT)
#define PLLCTRL_FBDIV_SHIFT     8
#define PLLCTRL_FBDIV_MASK      (0x7f << PLLCTRL_FBDIV_SHIFT)
#define PLLCTRL_DIV2_SHIFT      16
#define PLLCTRL_DIV2_MASK       (0x01 << PLLCTRL_DIV2_SHIFT)
#define PLLCTRL_POST_SRC_SHFT   24
#define PLLCTRL_POST_SRC_MASK   (0x7 << PLLCTRL_POST_SRC_SHFT)

#define SRCSEL_CPU_APLL             0
#define SRCSEL_CPU_DPLL             2
#define SRCSEL_CPU_VPLL             3
#define SRCSEL_PER_IOPLL            0
#define SRCSEL_PER_RPLL             2
#define SRCSEL_PER_DPLL_CLK_TO_LPD  3
#define SRCSEL_DDR_DPLL             0
#define SRCSEL_DDR_VPLL             1

#define POST_SRC_PS_REF_CLK         0
#define POST_SRC_VIDEO_REF_CLK      4
#define POST_SRC_ALT_REF_CLK        5
#define POST_SRC_AUX_REF_CLK        6
#define POST_SRC_GT_REF_CLK         7

/****************************************************************************
 * Full power domain clocks
 ****************************************************************************/

#define CRF_APB_APLL_CTRL          (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x20)
#define CRF_APB_DPLL_CTRL          (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x2C)
#define CRF_APB_VPLL_CTRL          (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x38)
#define CRF_APB_PLL_STATUS         (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x44)
#define CRF_APB_APLL_TO_LPD_CTRL   (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x48)
#define CRF_APB_DPLL_TO_LPD_CTRL   (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x4C)
#define CRF_APB_VPLL_TO_LPD_CTRL   (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x50)

/****************************************************************************
 * Peripheral clocks
 ****************************************************************************/

#define CRF_APB_ACPU_CTRL          (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x60)
#define CRF_APB_DBG_TRACE_CTRL     (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x64)
#define CRF_APB_DBG_FPD_CTRL       (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x68)
#define CRF_APB_DP_VIDEO_REF_CTRL  (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x70)
#define CRF_APB_DP_AUDIO_REF_CTRL  (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x74)
#define CRF_APB_DP_STC_REF_CTRL    (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x7C)
#define CRF_APB_DDR_CTRL           (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x80)
#define CRF_APB_GPU_REF_CTRL       (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0x84)
#define CRF_APB_SATA_REF_CTRL      (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xA0)
#define CRF_APB_PCIE_REF_CTRL      (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xB4)
#define CRF_APB_GDMA_REF_CTRL      (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xB8)
#define CRF_APB_DPDMA_REF_CTRL     (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xBC)
#define CRF_APB_TOPSW_MAIN_CTRL    (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xC0)
#define CRF_APB_TOPSW_LSBUS_CTRL   (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xC4)
#define CRF_APB_DBG_TSTMP_CTRL     (ZYNQ_MPSOC_CRF_APB_CLKC_ADDR + 0xF8)

/****************************************************************************
 * Low power domain clocks
 ****************************************************************************/

#define CRL_APB_IOPLL_CTRL         (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x20)
#define CRL_APB_RPLL_CTRL          (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x30)
#define CRL_APB_PLL_STATUS         (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x40)
#define CRL_APB_IOPLL_TO_FPD_CTRL  (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x44)
#define CRL_APB_RPLL_TO_FPD_CTRL   (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x48)

/****************************************************************************
 * Peripheral clocks
 ****************************************************************************/

#define CRL_APB_USB3_DUAL_REF_CTRL (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x4C)
#define CRL_APB_GEM0_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x50)
#define CRL_APB_GEM1_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x54)
#define CRL_APB_GEM2_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x58)
#define CRL_APB_GEM3_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x5C)
#define CRL_APB_USB0_BUS_REF_CTRL  (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x60)
#define CRL_APB_USB1_BUS_REF_CTRL  (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x64)
#define CRL_APB_QSPI_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x68)
#define CRL_APB_SDIO0_REF_CTRL     (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x6C)
#define CRL_APB_SDIO1_REF_CTRL     (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x70)
#define CRL_APB_UART0_REF_CTRL     (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x74)
#define CRL_APB_UART1_REF_CTRL     (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x78)
#define CRL_APB_SPI0_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x7C)
#define CRL_APB_SPI1_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x80)
#define CRL_APB_CAN0_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x84)
#define CRL_APB_CAN1_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x88)
#define CRL_APB_CPU_R5_CTRL        (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x90)
#define CRL_APB_IOU_SWITCH_CTRL    (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x9C)
#define CRL_APB_CSU_PLL_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xA0)
#define CRL_APB_PCAP_CTRL          (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xA4)
#define CRL_APB_LPD_SWITCH_CTRL    (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xA8)
#define CRL_APB_LPD_LSBUS_CTRL     (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xAC)
#define CRL_APB_DBG_LPD_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xB0)
#define CRL_APB_NAND_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xB4)
#define CRL_APB_ADMA_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xB8)
#define CRL_APB_PL0_REF_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xC0)
#define CRL_APB_PL1_REF_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xC4)
#define CRL_APB_PL2_REF_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xC8)
#define CRL_APB_PL3_REF_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xCC)
#define CRL_APB_PL0_THR_CNT        (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xD4)
#define CRL_APB_PL1_THR_CNT        (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xDC)
#define CRL_APB_PL2_THR_CNT        (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xE4)
#define CRL_APB_PL3_THR_CNT        (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0xFC)
#define CRL_APB_GEM_TSU_REF_CTRL   (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x100)
#define CRL_APB_DLL_REF_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x104)
#define CRL_APB_AMS_REF_CTRL       (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x108)
#define CRL_APB_I2C0_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x120)
#define CRL_APB_I2C1_REF_CTRL      (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x124)
#define CRL_APB_TIMESTAMP_REF_CTRL (ZYNQ_MPSOC_CRL_APB_CLKC_ADDR + 0x128)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: mpsoc_clk_register_get
 *
 * Description:
 *  Get register of the give closk source.
 *
 * Input Parameters:
 *   clk_pll - Clock source
 *
 * Returned Value:
 *   Ctrl register address of input clock source.
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint32_t  mpsoc_clk_register_get(enum mpsoc_clk clk_pll)
{
  switch (clk_pll)
    {
      case iopll:
        return  CRL_APB_IOPLL_CTRL;
      case rpll:
        return  CRL_APB_RPLL_CTRL;
      case apll:
        return  CRF_APB_APLL_CTRL;
      case dpll:
        return  CRF_APB_DPLL_CTRL;
      case vpll:
        return  CRF_APB_VPLL_CTRL;
      case acpu:
        return  CRF_APB_ACPU_CTRL;
      case ddr_ref:
        return  CRF_APB_DDR_CTRL;
      case qspi_ref:
        return  CRL_APB_QSPI_REF_CTRL;
      case gem0_ref:
        return  CRL_APB_GEM0_REF_CTRL;
      case gem1_ref:
        return  CRL_APB_GEM1_REF_CTRL;
      case gem2_ref:
        return  CRL_APB_GEM2_REF_CTRL;
      case gem3_ref:
        return  CRL_APB_GEM3_REF_CTRL;
      case uart0_ref:
        return  CRL_APB_UART0_REF_CTRL;
      case uart1_ref:
        return  CRL_APB_UART1_REF_CTRL;
      case sdio0_ref:
        return  CRL_APB_SDIO0_REF_CTRL;
      case sdio1_ref:
        return  CRL_APB_SDIO1_REF_CTRL;
      case spi0_ref:
        return  CRL_APB_SPI0_REF_CTRL;
      case spi1_ref:
        return  CRL_APB_SPI1_REF_CTRL;
      case nand_ref:
        return  CRL_APB_NAND_REF_CTRL;
      case i2c0_ref:
        return  CRL_APB_I2C0_REF_CTRL;
      case i2c1_ref:
        return  CRL_APB_I2C1_REF_CTRL;
      case can0_ref:
        return  CRL_APB_CAN0_REF_CTRL;
      case can1_ref:
        return  CRL_APB_CAN1_REF_CTRL;
      case sata_ref:
        return  CRF_APB_SATA_REF_CTRL;
      default:
        _err("Invalid clk id%d\n", clk_pll);
    }

    return 0;
}

/****************************************************************************
 * Function: mpsoc_clk_pll_get
 *
 * Description:
 *  Get clock source post frequency
 *
 * Input Parameters:
 *   clk_ctrl - Ctrl register value of clock source.
 *
 * Returned Value:
 *   Clock source post frequency.
 *
 ****************************************************************************/

static uintptr_t  mpsoc_clk_pll_get(uintptr_t  clk_ctrl)
{
  uint32_t  src_sel;

  src_sel = (clk_ctrl & PLLCTRL_POST_SRC_MASK) >>
             PLLCTRL_POST_SRC_SHFT; /* get pass-through clock source */

  switch (src_sel)
    {
    case  POST_SRC_VIDEO_REF_CLK:   /* Clock src is Video Ref Clk */
        return  CLK_CCF_VIDEO_CLK;

    case  POST_SRC_ALT_REF_CLK:     /* Clock src is Alt Ref Clk   */
        return  CLK_CCF_ALT_REF_CLK;

    case  POST_SRC_AUX_REF_CLK:     /*  Clock src is Aux Ref Clk  */
        return  CLK_CCF_AUX_REF_CLK;

    case  POST_SRC_GT_REF_CLK:      /* Clock src is GT Ref Clk    */
        return  CLK_CCF_GT_CRX_REF_CLK;

    default:                        /* Clock src is PS Ref Clk    */
        return  CLK_CCF_PSS_REF_CLK;
    }
}

/****************************************************************************
 * Function: mpsoc_clk_pll_rate_get
 *
 * Description:
 *  Get clock source pll output frequency
 *
 * Input Parameters:
 *   clk_pll - clock source pll.
 *
 * Returned Value:
 *   Pll output frequency.
 *
 ****************************************************************************/

static uintptr_t  mpsoc_clk_pll_rate_get(enum mpsoc_clk  clk_pll)
{
  uint32_t  clkctrl;
  uint32_t  reset;
  uint32_t  mul;
  uintptr_t freq;

  clkctrl = getreg32(mpsoc_clk_register_get(clk_pll));             /* Get clock source config */
  freq    = mpsoc_clk_pll_get(clkctrl);                            /* Get clock source post freq */

  reset   = (clkctrl & PLLCTRL_RESET_MASK) >> PLLCTRL_RESET_SHIFT; /* Get PLL reset status */
  if (reset && !(clkctrl & PLLCTRL_BYPASS_MASK))                   /* PLL is reset state */
    {
      return  0;                                                   /* return immediately */
    }

  mul   = (clkctrl & PLLCTRL_FBDIV_MASK) >> PLLCTRL_FBDIV_SHIFT;   /*  Get PLL clock Feedback */
  freq *= mul;

  if (clkctrl & PLLCTRL_DIV2_MASK) /*  Is Div2 enabled? */
    {
      freq /= 2;
    }

  return  freq;
}

/****************************************************************************
 * Function: mpsoc_cpu_clk_pll_get
 *
 * Description:
 *  Get clock source pll of ARM CPU
 *
 * Input Parameters:
 *   clkctrl - Ctrl register value of cpu clock source.
 *
 * Returned Value:
 *   Clock source pll of ARM CPU.
 *
 ****************************************************************************/

static enum mpsoc_clk mpsoc_cpu_clk_pll_get(uint32_t  clkctrl)
{
  /* Get clock source pll of CPU */

  uint32_t  src_sel = (clkctrl & CLK_CTRL_SRCSEL_MASK) >>
                       CLK_CTRL_SRCSEL_SHIFT;

  switch (src_sel)
  {
    case SRCSEL_CPU_DPLL: /* clock source is DPLL */
      return  dpll;

    case SRCSEL_CPU_VPLL: /* clock source is VPLL */
      return  vpll;

    case SRCSEL_CPU_APLL: /* clock source is APLL */
    default:
      return  apll;
  }
}

/****************************************************************************
 * Function: mpsoc_ddr_clk_pll_get
 *
 * Description:
 *  Get clock source pll of DDR
 *
 * Input Parameters:
 *   clkctrl - Ctrl register value of ddr clock source.
 *
 * Returned Value:
 *   Clock source pll of DDR.
 *
 ****************************************************************************/

static enum mpsoc_clk mpsoc_ddr_clk_pll_get(uint32_t  clkctrl)
{
  /*  Get clock source pll of DDR */

  uint32_t  src_sel = (clkctrl & CLK_CTRL_SRCSEL_MASK) >>
                       CLK_CTRL_SRCSEL_SHIFT;

  switch (src_sel)
  {
    case SRCSEL_DDR_VPLL: /* clock source is VPLL */
      return  vpll;

    case SRCSEL_DDR_DPLL: /* clock source is DPLL */
    default:
      return  dpll;
  }
}

/****************************************************************************
 * Function: mpsoc_peripheral_clk_pll_set
 *
 * Description:
 *  Get clock source pll of peripheral
 *
 * Input Parameters:
 *   clkctrl - Ctrl register value of peripheral clock source.
 *
 * Returned Value:
 *   Clock source pll of peripheral.
 *
 ****************************************************************************/

static enum mpsoc_clk mpsoc_peripheral_clk_pll_set(uint32_t  clkctrl)
{
  /*  Get clock source pll of peripheral */

  uint32_t src_sel = (clkctrl & CLK_CTRL_SRCSEL_MASK) >>
                      CLK_CTRL_SRCSEL_SHIFT;

  switch (src_sel)
  {
    case  SRCSEL_PER_RPLL:
      return rpll;

    case  SRCSEL_PER_DPLL_CLK_TO_LPD:
      return dpll;

    case  SRCSEL_PER_IOPLL:
    default:
      return iopll;
  }
}

/****************************************************************************
 * Function: mpsoc_cpu_clk_get
 *
 * Description:
 *  Get ARM CPU running frequency
 *
 * Input Parameters:
 *   none.
 *
 * Returned Value:
 *   ARM CPU running frequency.
 *
 ****************************************************************************/

static uint32_t mpsoc_cpu_clk_get(void)
{
  uint32_t       div;
  uint32_t       clkctrl;
  enum mpsoc_clk pllsrc;
  uint32_t          pll_rate;

  clkctrl = getreg32(CRF_APB_ACPU_CTRL);

  /* Get CPU clock divisor value */

  div      = (clkctrl & CLK_CTRL_DIV0_MASK) >> CLK_CTRL_DIV0_SHIFT;

  pllsrc   = mpsoc_cpu_clk_pll_get(clkctrl);  /* Get CPU pll src */
  pll_rate = mpsoc_clk_pll_rate_get(pllsrc);  /*  Get CPU clock div freq */

  return div_round_closest(pll_rate, div);
}

/****************************************************************************
 * Function: mpsoc_ddr_clk_get
 *
 * Description:
 *  Get DDR running frequency
 *
 * Input Parameters:
 *   none.
 *
 * Returned Value:
 *   DDR running frequency.
 *
 ****************************************************************************/

static uint32_t mpsoc_ddr_clk_get(void)
{
  uint32_t          div;
  uint32_t          clkctrl;
  enum mpsoc_clk    pllsrc;
  uint32_t          pll_rate;

  clkctrl = getreg32(CRF_APB_DDR_CTRL);

  /* Get DDR clock division */

  div     = (clkctrl & CLK_CTRL_DIV0_MASK) >> CLK_CTRL_DIV0_SHIFT;

  pllsrc  = mpsoc_ddr_clk_pll_get(clkctrl);  /* Get DDR clock src */
  pll_rate = mpsoc_clk_pll_rate_get(pllsrc); /* Get DDR running freq */

  return div_round_closest(pll_rate, div);
}

/****************************************************************************
 * Function: mpsoc_peripheral_clk_get
 *
 * Description:
 *  Get peripheral running frequency
 *
 * Input Parameters:
 *   peripheral clock ID.
 *
 * Returned Value:
 *   peripheral running frequency.
 *
 ****************************************************************************/

static uint32_t mpsoc_peripheral_clk_get(enum mpsoc_clk clk)
{
  enum mpsoc_clk  pll;
  uint32_t     clkctrl;
  uint32_t     div0;
  uint32_t     div1;
  uintptr_t    pll_rate;

  clkctrl = getreg32(mpsoc_clk_register_get(clk));

  div0 = (clkctrl & CLK_CTRL_DIV0_MASK) >> CLK_CTRL_DIV0_SHIFT;
  if (!div0)
    {
      div0 = 1; /* if divisor0 is 0, set it as 1 */
    }

  div1 = (clkctrl & CLK_CTRL_DIV1_MASK) >> CLK_CTRL_DIV1_SHIFT;
  if (!div1)
    {
      div1 = 1; /* if divisor1 is 0, set it as 1 */
    }

  pll = mpsoc_peripheral_clk_pll_set(clkctrl);
  pll_rate = mpsoc_clk_pll_rate_get(pll);

  return  div_round_closest(div_round_closest(pll_rate, div0), div1);
}

/****************************************************************************
 * Function: mpsoc_peripheral_clk_two_divs_calc
 *
 * Description:
 *  Get peripheral running frequency
 *
 * Input Parameters:
 *   rate - clock freq
 *   ulPllRate - pll freq
 *   div0 - divisor0
 *   div1 - divisor1
 *
 * Returned Value:
 *   best running frequency.
 *
 ****************************************************************************/

static uintptr_t mpsoc_peripheral_clk_two_divs_calc(uintptr_t rate,
                                                    uintptr_t pll_rate,
                                                    uint32_t  *div0,
                                                    uint32_t  *div1)
{
  uint32_t   id0;
  uint32_t   id1;
  uintptr_t  new_rate;
  uintptr_t  best_rate = 0;
  intptr_t   new_err;
  intptr_t   best_err = (intptr_t)(~0UL >> 1);

  for (id0 = 1; id0 <= MPSOC_CLK_MAXDIV; id0++)
    {
      for (id1 = 1; id1 <= MPSOC_CLK_MAXDIV >> 1; id1++)
        {
          new_rate = div_round_closest(div_round_closest(pll_rate, id0),
                                       id1);
          new_err  = abs(new_rate - rate);

          if (new_err < best_err)
            {
                *div0     = id0;
                *div1     = id1;
                best_err  = new_err;
                best_rate = new_rate;
            }
        }
    }

  return best_rate;
}

/****************************************************************************
 * Function: mpsoc_peripheral_clk_set
 *
 * Description:
 *  Set peripheral running frequency
 *
 * Input Parameters:
 *   clk - peripheral clock ID
 *   rate - clock freq
 *
 * Returned Value:
 *   New peripheral running frequency.
 *
 ****************************************************************************/

static uint32_t mpsoc_peripheral_clk_set(enum mpsoc_clk clk, uintptr_t rate)
{
  uint32_t       clkctrl;
  uint32_t       new_rate;
  uint32_t       div0 = 0;
  uint32_t       div1 = 0;
  uint32_t       mask;
  uint32_t       regval;
  uint32_t       regadd;
  enum mpsoc_clk pll;
  uintptr_t      pll_rate;

  regadd  = mpsoc_clk_register_get(clk);
  clkctrl = getreg32(regadd);

  pll = mpsoc_peripheral_clk_pll_set(clkctrl);
  pll_rate = mpsoc_clk_pll_rate_get(pll);

  clkctrl &= ~CLK_CTRL_DIV0_MASK;
  clkctrl &= ~CLK_CTRL_DIV1_MASK;

  new_rate = mpsoc_peripheral_clk_two_divs_calc(rate, pll_rate,
                                                &div0, &div1);
  clkctrl |= div1 << CLK_CTRL_DIV1_SHIFT;
  clkctrl |= div0 << CLK_CTRL_DIV0_SHIFT;

  mask = (MPSOC_CLK_MAXDIV << CLK_CTRL_DIV0_SHIFT) |
           (MPSOC_CLK_MAXDIV << CLK_CTRL_DIV1_SHIFT);

  regval  = getreg32(regadd);
  regval &= ~mask;
  regval |= clkctrl;

  putreg32(regval, regadd);

  return  new_rate;
}

/****************************************************************************
 * Function: mpsoc_ttc_clk_get
 *
 * Description:
 *  Get ttc controller running frequency
 *
 * Input Parameters:
 *   clk - peripheral clock ID
 *
 * Returned Value:
 *   PLL frequency.
 *
 ****************************************************************************/

static uint32_t mpsoc_ttc_clk_get(enum mpsoc_clk clk)
{
  uint32_t  clk_sel;
  uint32_t  clkctrl = 0;
  uintptr_t freq    = 0;

  /* Get IOU_TTC_APB_CLK */

  clk_sel = getreg32(ZYNQ_MPSOC_IOU_SLCR_ADDR + 0x380);
  clk_sel = (clk_sel >> (clk - ttc0_ref)) & 0x3;

  switch (clk_sel)
    {
      case 0:
        clkctrl = getreg32(CRL_APB_LPD_LSBUS_CTRL); /* Get LPD LSBUS clock src config */
        break;

      case 0x1:
        freq = mpsoc_clk_pll_get(POST_SRC_PS_REF_CLK);
        break;

      case 0x2:
        clkctrl = getreg32(CRL_APB_CPU_R5_CTRL);   /* Get RPU clock src config */
        break;

      case 0x3:
        break;
      default:
        break;
    }

  if (!clkctrl)
    {
      return  (freq);
    }

  if ((clkctrl & 0x3) == 0x00)
    {
      return  mpsoc_clk_pll_rate_get(rpll);
    }
  else if ((clkctrl & 0x3) == 0x2)
    {
      return  mpsoc_clk_pll_rate_get(iopll);
    }
  else
    {
      return  mpsoc_clk_pll_rate_get(dpll);
    }

  return  freq;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: mpsoc_clk_rate_get
 *
 * Description:
 *  Get controller running frequency
 *
 * Input Parameters:
 *   clk - peripheral clock ID
 *
 * Returned Value:
 *   Running frequency.
 *
 ****************************************************************************/

uintptr_t mpsoc_clk_rate_get(enum mpsoc_clk clk)
{
  switch (clk)
  {
    case iopll ... vpll:
      return mpsoc_clk_pll_rate_get(clk);

    case acpu:
      return mpsoc_cpu_clk_get();

    case ddr_ref:
      return mpsoc_ddr_clk_get();

    case gem0_ref ... gem3_ref:
    case qspi_ref ... can1_ref:
      return mpsoc_peripheral_clk_get(clk);

    case ttc0_ref ... ttc3_ref:
      return mpsoc_ttc_clk_get(clk);

    default:
      return -ENXIO;
  }
}

/****************************************************************************
 * Function: mpsoc_clk_rate_set
 *
 * Description:
 *  Set running frequency
 *
 * Input Parameters:
 *   clk - peripheral clock ID
 *   rate - clock freq
 *
 * Returned Value:
 *   peripheral running frequency.
 *
 ****************************************************************************/

uintptr_t mpsoc_clk_rate_set(enum mpsoc_clk clk, uintptr_t rate)
{
  switch (clk)
    {
      case gem0_ref ... gem3_ref:
      case qspi_ref ... can1_ref:
        return mpsoc_peripheral_clk_set(clk, rate);

      default:
        return -ENXIO;
    }
}
