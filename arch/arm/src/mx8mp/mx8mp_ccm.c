/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_ccm.c
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

#include "mx8mp_ccm.h"
#include "hardware/mx8mp_rdc.h"

#include <debug.h>
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t CLK_ROOT_MAP[CLOCK_ROOT_MAP_SIZE][8] =
{
  ARM_A53_CLK_MUX,
  ARM_M7_CLK_MUX,
  ML_CLK_MUX,
  GPU3D_CORE_CLK_MUX,
  GPU3D_SHADER_CLK_MUX,
  GPU2D_CLK_MUX,
  AUDIO_AXI_CLK_MUX,
  HSIO_AXI_CLK_MUX,
  MEDIA_ISP_CLK_MUX,
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  MAIN_AXI_CLK_MUX,
  ENET_AXI_CLK_MUX,
  NAND_USDHC_BUS_CLK_MUX,
  VPU_BUS_CLK_MUX,
  MEDIA_AXI_CLK_MUX,
  MEDIA_APB_CLK_MUX,
  HDMI_APB_CLK_MUX,
  HDMI_AXI_CLK_MUX,
  GPU_AXI_CLK_MUX,
  GPU_AHB_CLK_MUX,
  NOC_CLK_MUX,
  NOC_IO_CLK_MUX,
  ML_AXI_CLK_MUX,
  ML_AHB_CLK_MUX,
  {},
  {},
  AHB_CLK_MUX,
  IPG_CLK_MUX,
  AUDIO_AHB_CLK_MUX,
  {},
  {},
  {},
  MEDIA_DISP2_CLK_MUX,
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  DRAM_SEL_CFG_MUX,
  ARM_A53_CLK_ROOT_SEL_MUX,
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  {},
  DRAM_ALT_CLK_MUX,
  DRAM_APB_CLK_MUX,
  VPU_G1_CLK_MUX,
  VPU_G2_CLK_MUX,
  CAN1_CLK_MUX,
  CAN2_CLK_MUX,
  MEMREPAIR_CLK_MUX,
  PCIE_PHY_CLK_MUX,
  PCIE_AUX_CLK_MUX,
  I2C5_CLK_MUX,
  I2C6_CLK_MUX,
  SAI1_CLK_MUX,
  SAI2_CLK_MUX,
  SAI3_CLK_MUX,
  {},
  SAI5_CLK_MUX,
  SAI6_CLK_MUX,
  ENET_QOS_CLK_MUX,
  ENET_QOS_TIMER_CLK_MUX,
  ENET_REF_CLK_MUX,
  ENET_TIMER_CLK_MUX,
  ENET_PHY_REF_CLK_MUX,
  NAND_CLK_MUX,
  QSPI_CLK_MUX,
  USDHC1_CLK_MUX,
  USDHC2_CLK_MUX,
  I2C1_CLK_MUX,
  I2C2_CLK_MUX,
  I2C3_CLK_MUX,
  I2C4_CLK_MUX,
  UART1_CLK_MUX,
  UART2_CLK_MUX,
  UART3_CLK_MUX,
  UART4_CLK_MUX,
  {},
  {},
  GIC_CLK_MUX,
  ECSPI1_CLK_MUX,
  ECSPI2_CLK_MUX,
  PWM1_CLK_MUX,
  PWM2_CLK_MUX,
  PWM3_CLK_MUX,
  PWM4_CLK_MUX,
  GPT1_CLK_MUX,
  GPT2_CLK_MUX,
  GPT3_CLK_MUX,
  GPT4_CLK_MUX,
  GPT5_CLK_MUX,
  GPT6_CLK_MUX,
  TRACE_CLK_MUX,
  WDOG_CLK_MUX,
  WRCLK_CLK_MUX,
  IPP_DO_CLKO1_MUX,
  IPP_DO_CLKO2_MUX,
  HDMI_FDCC_TST_CLK_MUX,
  HDMI_27M_CLK_MUX,
  HDMI_REF_266M_CLK_MUX,
  USDHC3_CLK_MUX,
  MEDIA_CAM1_PIX_CLK_MUX,
  MEDIA_MIPI_PHY1_REF_CLK_MUX,
  MEDIA_DISP1_PIX_CLK_MUX,
  MEDIA_CAM2_PIX_CLK_MUX,
  MEDIA_LDB_CLK_MUX,
  {},
  {},
  {},
  MEDIA_MIPI_TEST_BYTE_CLK_MUX,
  ECSPI3_CLK_MUX,
  PDM_CLK_MUX,
  VPU_VC8000E_CLK_MUX,
  SAI7_CLK_MUX,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t root_frequency(uint32_t freq_in,
                               uint32_t main_div,
                               uint32_t pre_div,
                               uint32_t scaler,
                               uint32_t dsm)
{
  /* Reference manual chapter 5.1.5.4.4 SSCG and Fractional PLLs
   * Fout = ((m + k /65536) * Fin) / (p * 2^s)
   * with m -> main divider, p -> pre-divider, s -> post-scaler and k -> DSM
   * => ((m * 65536 + k) * Fin) / (65536 * p * (1 << s))
   */

  return  (uint32_t)(((main_div * 65536llu + dsm) * freq_in) /
                      (65536llu * pre_div * (1 << scaler)));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t mx8mp_ccm_get_root_clock(int clk_root_src)
{
  uint32_t pll;
  uint32_t dsm;
  uint32_t final_div;
  uint32_t fdiv0;
  uint32_t main_div;
  uint32_t pre_div;
  uint32_t post_div;
  uint32_t root_freq;

  dsm = 0;
  final_div = 1;

  switch (clk_root_src)
    {
      case OSC_24M_REF_CLK:
        return 24000000;

      case OSC_32K_REF_CLK:
        return 32000;

      case ARM_PLL_CLK:
        {
          pll = CCM_ANALOG_ARM_PLL;
        }
        break;

      case DRAM_PLL1_CLK:
        {
          pll = CCM_ANALOG_DRAM_PLL;
          dsm = (getreg32(pll + CCM_ANALOG_FDIV1) & CCM_FDIV1_DSM_MASK)
                  >> CCM_FDIV1_DSM_SHIFT;
        }
        break;

      case VPU_PLL_CLK:
        {
          pll = CCM_ANALOG_VPU_PLL;
        }
        break;

      case GPU_PLL_CLK:
        {
          pll = CCM_ANALOG_GPU_PLL;
        }
        break;

      case SYSTEM_PLL1_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
        }
        break;

      case SYSTEM_PLL1_DIV2_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 2;
        }
        break;

      case SYSTEM_PLL1_DIV3_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 3;
        }
        break;

      case SYSTEM_PLL1_DIV4_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 4;
        }
        break;

      case SYSTEM_PLL1_DIV5_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 5;
        }
        break;

      case SYSTEM_PLL1_DIV6_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 6;
        }
        break;

      case SYSTEM_PLL1_DIV8_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 8;
        }
        break;

      case SYSTEM_PLL1_DIV10_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 10;
        }
        break;

      case SYSTEM_PLL1_DIV20_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL1;
          final_div = 20;
        }
        break;

      case SYSTEM_PLL2_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
        }
        break;

      case SYSTEM_PLL2_DIV2_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 2;
        }
        break;

      case SYSTEM_PLL2_DIV3_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 3;
        }
        break;

      case SYSTEM_PLL2_DIV4_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 4;
        }
        break;

      case SYSTEM_PLL2_DIV5_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 5;
        }
        break;

      case SYSTEM_PLL2_DIV6_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 6;
        }
        break;

      case SYSTEM_PLL2_DIV8_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 8;
        }
        break;

      case SYSTEM_PLL2_DIV10_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 10;
        }
        break;

      case SYSTEM_PLL2_DIV20_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL2;
          final_div = 20;
          break;
        }

      case SYSTEM_PLL3_CLK:
        {
          pll = CCM_ANALOG_SYSTEM_PLL3;
        }
        break;

      case AUDIO_PLL1_CLK:
        {
          pll = CCM_ANALOG_AUDIO_PLL1;
          dsm = (getreg32(pll + CCM_ANALOG_FDIV1) & CCM_FDIV1_DSM_MASK)
                  >> CCM_FDIV1_DSM_SHIFT;
        }
        break;

      case AUDIO_PLL2_CLK:
        {
          pll = CCM_ANALOG_AUDIO_PLL2;
          dsm = (getreg32(pll + CCM_ANALOG_FDIV1) & CCM_FDIV1_DSM_MASK)
                  >> CCM_FDIV1_DSM_SHIFT;
        }
        break;

      case VIDEO_PLL_CLK:
        {
          pll = CCM_ANALOG_VIDEO_PLL1;
          dsm = (getreg32(pll + CCM_ANALOG_FDIV1) & CCM_FDIV1_DSM_MASK)
                  >> CCM_FDIV1_DSM_SHIFT;
        }
        break;

      default:
        return 0;
    }

  fdiv0 = getreg32(pll + CCM_ANALOG_FDIV0);
  main_div = (fdiv0 & CCM_FDIV0_MAIN_DIV_MASK) >> CCM_FDIV0_MAIN_DIV_SHIFT;
  pre_div  = (fdiv0 & CCM_FDIV0_PRE_DIV_MASK)  >> CCM_FDIV0_PRE_DIV_SHIFT;
  post_div = (fdiv0 & CCM_FDIV0_POST_DIV_MASK) >> CCM_FDIV0_POST_DIV_SHIFT;
  root_freq =  root_frequency(24000000,
                              main_div,
                              pre_div,
                              post_div,
                              dsm);
  return root_freq / final_div;
}

uint32_t mx8mp_ccm_get_clock(int clk_index)
{
  uint32_t reg;
  uint32_t is_enable;
  uint32_t pre_podf;
  uint32_t post_podf;
  uint32_t mux;
  uint32_t src_clk;

  reg = getreg32(CCM_CLK_ROOT_BASE + 128 * clk_index);
  is_enable = reg & CCM_CLK_ROOT_ENABLE;
  pre_podf  = (reg & CCM_CLK_ROOT_PRE_PODF_MASK)
                >> CCM_CLK_ROOT_PRE_PODF_SHIFT;
  post_podf = (reg & CCM_CLK_ROOT_POST_PODF_MASK)
                >> CCM_CLK_ROOT_POST_PODF_SHIFT;
  mux       = (reg & CCM_CLK_ROOT_MUX_MASK)
                >> CCM_CLK_ROOT_MUX_SHIFT;

  if (!is_enable)
    {
      return 0;
    }

  src_clk = mx8mp_ccm_get_root_clock(CLK_ROOT_MAP[clk_index][mux]);
  return src_clk / (pre_podf + 1) / (post_podf + 1);
}

int mx8mp_ccm_configure_clock(int clk_index,
                              int clk_root_src,
                              uint32_t pre_div,
                              uint32_t post_div)
{
  uint32_t reg;
  uint32_t value;
  int mux = 8;
  int i;

  for (i = 0; i < 8; ++i)
    {
      if (CLK_ROOT_MAP[clk_index][i] == clk_root_src)
        {
          mux = i;
          break;
        }
    }

  if (mux == 8)
    {
      /* cannot find a mux for the desired clock root:
       * skip the configuration
       */

      return -1;
    }

  reg = CCM_CLK_ROOT_BASE + 128 * clk_index;
  value = getreg32(reg) & CCM_CLK_ROOT_ENABLE;
  value |= (((pre_div  - 1) << CCM_CLK_ROOT_PRE_PODF_SHIFT)
              & CCM_CLK_ROOT_PRE_PODF_MASK);
  value |= (((post_div - 1) << CCM_CLK_ROOT_POST_PODF_SHIFT)
              & CCM_CLK_ROOT_POST_PODF_MASK);
  value |= ((mux << CCM_CLK_ROOT_MUX_SHIFT)
              & CCM_CLK_ROOT_MUX_MASK);
  putreg32(value, reg);

  return 0;
}

void mx8mp_ccm_gate_clock(int gate_index, uint32_t value)
{
  uint32_t reg = (CCM_CCGR_BASE + gate_index * 16);
  putreg32(value, reg);
}

void mx8mp_ccm_enable_clock(int clk_index)
{
  uint32_t reg = CCM_CLK_ROOT_BASE + 128 * clk_index;
  modreg32(CCM_CLK_ROOT_ENABLE, CCM_CLK_ROOT_ENABLE, reg);
}

void mx8mp_ccm_gate_pll(int pll_index, uint32_t value)
{
  uint32_t reg = (CCM_PLL_BASE + pll_index * 16);
  putreg32(value, reg);
}

void mx8mp_ccm_configure_pll(int pll_index,
                             uint32_t main_div,
                             uint32_t pre_div,
                             uint32_t post_div,
                             uint32_t dsm)
{
  uint32_t reg_genctrl;
  uint32_t reg_fdiv0;
  uint32_t reg_fdiv1;
  uint32_t genctrl;
  uint32_t fdiv0;

  /* Associate pll_index (pll gating) to
   * pll configuration register in CCM_ANALOG
   */

  switch (pll_index)
  {
    case ARM_PLL_CLK:
      {
        reg_genctrl = CCM_ANALOG_ARM_PLL;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case GPU_PLL_CLK:
      {
        reg_genctrl = CCM_ANALOG_GPU_PLL;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case VPU_PLL_CLK:
      {
        reg_genctrl = CCM_ANALOG_VPU_PLL;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case DRAM_PLL1_CLK:
      {
        reg_genctrl = CCM_ANALOG_DRAM_PLL;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case SYSTEM_PLL1_CLK:
      {
        reg_genctrl = CCM_ANALOG_SYSTEM_PLL1;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case SYSTEM_PLL2_CLK:
      {
        reg_genctrl = CCM_ANALOG_SYSTEM_PLL2;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case SYSTEM_PLL3_CLK:
      {
        reg_genctrl = CCM_ANALOG_SYSTEM_PLL3;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = 0;
      }
      break;

    case AUDIO_PLL1_CLK:
      {
        reg_genctrl = CCM_ANALOG_AUDIO_PLL1;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = reg_genctrl + CCM_ANALOG_FDIV1;
      }
      break;

    case AUDIO_PLL2_CLK:
      {
        reg_genctrl = CCM_ANALOG_ARM_PLL;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = reg_genctrl + CCM_ANALOG_FDIV1;
      }
      break;

    case VIDEO_PLL_CLK:
      {
        reg_genctrl = CCM_ANALOG_ARM_PLL;
        reg_fdiv0   = reg_genctrl + CCM_ANALOG_FDIV0;
        reg_fdiv1   = reg_genctrl + CCM_ANALOG_FDIV1;
      }
      break;

    default:  /* unknown PLL - skip configuration */
      return;
  }

  /* Disable PLL, no bypass, clock ref on 24MHz */

  genctrl = getreg32(reg_genctrl);
  genctrl &= ~(CCM_PLL_RST | CCM_PLL_BYPASS | CCM_PLL_REF_CLK_SEL_MASK);
  putreg32(genctrl, reg_genctrl);

  /* Apply dividers */

  fdiv0 = 0;
  fdiv0 |= ((main_div << CCM_FDIV0_MAIN_DIV_SHIFT)
              & CCM_FDIV0_MAIN_DIV_MASK);
  fdiv0 |= ((pre_div  << CCM_FDIV0_PRE_DIV_SHIFT)
              & CCM_FDIV0_PRE_DIV_MASK);
  fdiv0 |= ((post_div << CCM_FDIV0_POST_DIV_SHIFT)
              & CCM_FDIV0_POST_DIV_MASK);
  putreg32(fdiv0, reg_fdiv0);

  if (reg_fdiv1 != 0)
  {
      uint32_t fdiv1 = 0;
      fdiv1 |= ((dsm << CCM_FDIV1_DSM_SHIFT)
                  & CCM_FDIV1_DSM_MASK);
      putreg32(fdiv1, reg_fdiv1);
  }

  /* Enable PLL */

  genctrl = getreg32(reg_genctrl);
  genctrl |= (CCM_PLL_CLKE | CCM_PLL_RST);
  putreg32(genctrl, reg_genctrl);

  /* Wait for PLL to stabilize */

  while (!(getreg32(reg_genctrl) & CCM_PLL_LOCK));
}
