/****************************************************************************
 * arch/arm64/src/a64/a64_de.c
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

/* Reference:
 *
 * "Rendering PinePhone's Display (DE and TCON0)"
 * https://lupyuen.github.io/articles/de
 *
 * "NuttX RTOS for PinePhone: Render Graphics in Zig"
 * https://lupyuen.github.io/articles/de2
 *
 * "A64 Page" refers to Allwinner A64 User Manual
 * https://lupyuen.github.io/images/Allwinner_A64_User_Manual_V1.1.pdf
 *
 * "A31 Page" refers to Allwinner A31 User Manual
 * https://lupyuen.github.io/images/A31_User_Manual_v1.3_20150510.pdf
 *
 * "DE Page" refers to Allwinner Display Engine 2.0 Specification
 * https://lupyuen.github.io/images/Allwinner_DE2.0_Spec_V1.0.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include "hardware/a64_memorymap.h"
#include "arm64_arch.h"
#include "a64_de.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout for Display Engine PLL in milliseconds */

#define PLL_TIMEOUT_MS          5

/* A64 Display Engine Base Addresses ****************************************/

/* Mixer 0 (DE Page 24) */

#define A64_MIXER0_ADDR         (A64_DE_ADDR + 0x100000)
#define A64_MIXER0_LEN          0x6000

/* Global Registers (DE Page 90) */

#define A64_GLB_ADDR            (A64_MIXER0_ADDR + 0x0000)

/* Blender (DE Page 90) */

#define A64_BLD_ADDR            (A64_MIXER0_ADDR + 0x1000)

/* UI Overlay 1 (DE Page 102) */

#define A64_OVL_UI_CH1_ADDR     (A64_MIXER0_ADDR + 0x3000)

/* Video Scaler (DE Page 90) */

#define A64_VIDEO_SCALER_ADDR   (A64_MIXER0_ADDR + 0x020000)

/* UI Scaler 1 (DE Page 90) */

#define A64_UI_SCALER1_ADDR     (A64_MIXER0_ADDR + 0x040000)

/* UI Scaler 2 (DE Page 90) */

#define A64_UI_SCALER2_ADDR     (A64_MIXER0_ADDR + 0x050000)

/* Fresh and Contrast Enhancement (DE Page 61) */

#define A64_FCE_ADDR            (A64_MIXER0_ADDR + 0x0a0000)

/* Black and White Stretch (DE Page 42) */

#define A64_BWS_ADDR            (A64_MIXER0_ADDR + 0x0a2000)

/* Luminance Transient Improvement (DE Page 71) */

#define A64_LTI_ADDR            (A64_MIXER0_ADDR + 0x0a4000)

/* Luma Peaking (DE Page 80) */

#define A64_PEAKING_ADDR        (A64_MIXER0_ADDR + 0x0a6000)

/* Adaptive Saturation Enhancement (DE Page 40) */

#define A64_ASE_ADDR            (A64_MIXER0_ADDR + 0x0a8000)

/* Fancy Color Curvature Change (DE Page 56) */

#define A64_FCC_ADDR            (A64_MIXER0_ADDR + 0x0aa000)

/* UI Overlays 1, 2 and 3 (DE Page 102) */

#define A64_OVL_UI_ADDR(ch)     (A64_OVL_UI_CH1_ADDR + ((ch) - 1) * 0x1000)

/* UI Scalers 1, 2 and 3 (DE Page 90) */

#define A64_UI_SCALER_ADDR(ch)  (A64_UI_SCALER1_ADDR + ((ch) - 1) * 0x10000)

/* Dynamic Range Controller (DE Page 48) */

#define A64_DRC_ADDR            (A64_DE_ADDR + 0x1b0000)

/* A64 System Control Registers and Bit Definitions *************************/

/* SRAM Control Register 1 (A31 Page 191) */

#define SRAM_CTRL_REG1          (A64_SYSCTL_ADDR + 0x4)

/* A64 CCU Registers and Bit Definitions ************************************/

/* PLL Display Engine Control Register (A64 Page 96) */

#define PLL_DE_CTRL_REG  (A64_CCU_ADDR + 0x0048)
#define PLL_PRE_DIV_M(n) ((n) << 0)
#define PLL_FACTOR_N(n)  ((n) << 8)
#define PLL_MODE_SEL     (1 << 24)
#define PLL_LOCK         (1 << 28)
#define PLL_ENABLE       (1 << 31)

/* Bus Clock Gating Register 1 (A64 Page 102) */

#define BUS_CLK_GATING_REG1 (A64_CCU_ADDR + 0x0064)
#define DE_GATING           (1 << 12)

/* Display Engine Clock Register (A64 Page 117) */

#define DE_CLK_REG       (A64_CCU_ADDR + 0x0104)
#define CLK_SRC_SEL(n)   ((n) << 24)
#define CLK_SRC_SEL_MASK (0b111 << 24)
#define SCLK_GATING      (1 << 31)
#define SCLK_GATING_MASK (0b1 << 31)

/* Bus Software Reset Register 1 (A64 Page 140) */

#define BUS_SOFT_RST_REG1 (A64_CCU_ADDR + 0x02c4)
#define DE_RST            (1 << 12)

/* A64 Display Engine Registers and Bit Definitions *************************/

/* DE SCLK Gating Register (DE Page 25) */

#define SCLK_GATE        (A64_DE_ADDR + 0x000)
#define CORE0_SCLK_GATE  (1 << 0)

/* DE AHB Reset register (DE Page 25) */

#define AHB_RESET        (A64_DE_ADDR + 0x008)
#define CORE0_HCLK_RESET (1 << 0)

/* DE HCLK Gating Register (DE Page 25) */

#define HCLK_GATE        (A64_DE_ADDR + 0x004)
#define CORE0_HCLK_GATE  (1 << 0)

/* DE2TCON MUX Register (DE Page 26) */

#define DE2TCON_MUX      (A64_DE_ADDR + 0x010)
#define DE2TCON_MUX_MASK (1 << 0)

/* Video Scaler Control Register (DE Page 130) */

#define VS_CTRL_REG   (A64_VIDEO_SCALER_ADDR + 0x00)

/* 0x113 0000 is undocumented. Might be a mix-up with UI_SCALER3. */

#define UNDOC_1130000 (A64_DE_ADDR + 0x130000)

/* UI Scaler 1 Control Register (DE Page 66) */

#define UIS_CTRL_REG1 (A64_UI_SCALER1_ADDR + 0x00)

/* UI Scaler 2 Control Register (DE Page 66) */

#define UIS_CTRL_REG2 (A64_UI_SCALER2_ADDR + 0x00)

/* Fresh and Contrast Enhancement Global Control Register (DE Page 61) */

#define GCTRL_REG_FCE (A64_FCE_ADDR + 0x00)

/* Black and White Stretch Global Control Register (DE Page 42) */

#define GCTRL_REG_BWS (A64_BWS_ADDR + 0x00)

/* Luminance Transient Improvement Global Control Register (DE Page 72) */

#define LTI_CTL       (A64_LTI_ADDR + 0x00)

/* Luma Peaking Module Control Register (DE Page 80) */

#define LP_CTRL_REG   (A64_PEAKING_ADDR + 0x00)

/* Adaptive Saturation Enhancement Global Control Register (DE Page 40) */

#define ASE_CTL_REG   (A64_ASE_ADDR + 0x00)

/* Fancy Color Curvature Change Control Register (DE Page 56) */

#define FCC_CTL_REG   (A64_FCC_ADDR + 0x00)

/* Dynamic Range Controller Module General Control Register (DE Page 49) */

#define GNECTL_REG    (A64_DRC_ADDR + 0x00)

/* Mixer 0 Global Control Register (DE Page 92) */

#define GLB_CTL  (A64_MIXER0_ADDR + 0x00)
#define EN_MIXER (1 << 0)

/* Global Double Buffer Control (DE Page 93) */

#define GLB_DBUFFER       (A64_GLB_ADDR + 0x008)
#define DOUBLE_BUFFER_RDY (1 << 0)

/* Global Size (DE Page 93) */

#define GLB_SIZE (A64_GLB_ADDR + 0x00c)

/* Blender Fill Color Control (DE Page 106) */

#define BLD_FILL_COLOR_CTL (A64_BLD_ADDR + 0x000)
#define P0_FCEN(n)         ((n) << 0)
#define P0_EN(n)           ((n) << 8)
#define P1_EN(n)           ((n) << 9)
#define P2_EN(n)           ((n) << 10)

/* Note: DE Page 91 shows incorrect offset N*0x14 for
 * BLD_FILL_COLOR, BLD_CH_ISIZE and BLD_CH_OFFSET.
 * Correct offset is N*0x10 (DE Page 108)
 * (N = Pipe Number, from 0 to 2 for Channels 1 to 3)
 */

/* Blender Fill Color (DE Page 107) */

#define BLD_FILL_COLOR(p) (A64_BLD_ADDR + 0x004 + (p) * 0x10)
#define FILL_BLUE(n)      ((n) << 0)
#define FILL_GREEN(n)     ((n) << 8)
#define FILL_RED(n)       ((n) << 16)
#define FILL_ALPHA(n)     ((n) << 24)

/* Blender Input Memory Size (DE Page 108) */

#define BLD_CH_ISIZE(p)   (A64_BLD_ADDR + 0x008 + (p) * 0x10)

/* Blender Input Memory Offset (DE Page 108) */

#define BLD_CH_OFFSET(p)  (A64_BLD_ADDR + 0x00c + (p) * 0x10)

/* Blender Routing Control (DE Page 108) */

#define BLD_CH_RTCTL (A64_BLD_ADDR + 0x080)
#define P0_RTCTL(n)  ((n) << 0)
#define P1_RTCTL(n)  ((n) << 4)
#define P2_RTCTL(n)  ((n) << 8)

/* Blender Pre-Multiply Control (DE Page 109) */

#define BLD_PREMUL_CTL   (A64_BLD_ADDR + 0x084)
#define P0_ALPHA_MODE(n) ((n) << 0)
#define P1_ALPHA_MODE(n) ((n) << 1)
#define P2_ALPHA_MODE(n) ((n) << 2)
#define P3_ALPHA_MODE(n) ((n) << 3)

/* Blender Background Color (DE Page 109) */

#define BLD_BK_COLOR (A64_BLD_ADDR + 0x088)
#define BK_BLUE(n)   ((n) << 0)
#define BK_GREEN(n)  ((n) << 8)
#define BK_RED(n)    ((n) << 16)
#define BK_RESERVED  (0xff << 24)

/* Blender Output Size Setting (DE Page 110) */

#define BLD_SIZE (A64_BLD_ADDR + 0x08c)

/* Blender Control (DE Page 110) */

#define BLD_CTL(p)   (A64_BLD_ADDR + 0x090 + (p) * 4)
#define BLEND_PFS(n) ((n) << 0)
#define BLEND_PFD(n) ((n) << 8)
#define BLEND_AFS(n) ((n) << 16)
#define BLEND_AFD(n) ((n) << 24)

/* UI Overlay Attribute Control (DE Page 102) */

#define OVL_UI_ATTR_CTL(ch) (A64_OVL_UI_ADDR(ch) + 0x00)
#define LAY_EN              (1 << 0)
#define LAY_ALPHA_MODE(n)   ((n) << 1)
#define LAY_FBFMT(n)        ((n) << 8)
#define LAY_GLBALPHA(n)     ((n) << 24)

/* UI Overlay Memory Block Size (DE Page 104) */

#define OVL_UI_MBSIZE(ch)   (A64_OVL_UI_ADDR(ch) + 0x04)

/* UI Overlay Memory Block Coordinate (DE Page 104) */

#define OVL_UI_COOR(ch)     (A64_OVL_UI_ADDR(ch) + 0x08)

/* UI Overlay Memory Pitch (DE Page 104) */

#define OVL_UI_PITCH(ch)    (A64_OVL_UI_ADDR(ch) + 0x0c)

/* UI Overlay Top Field Memory Block Low Address (DE Page 104) */

#define OVL_UI_TOP_LADD(ch) (A64_OVL_UI_ADDR(ch) + 0x10)

/* UI Overlay Overlay Window Size (DE Page 106) */

#define OVL_UI_SIZE(ch)     (A64_OVL_UI_ADDR(ch) + 0x88)

/* UI Scaler Control Register (DE Page 66) */

#define UIS_CTRL_REG(ch)    (A64_UI_SCALER_ADDR(ch) + 0x00)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_wait_pll
 *
 * Description:
 *   Wait for Display Engine PLL to be stable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

static int a64_wait_pll(void)
{
  int i;

  for (i = 0; i < PLL_TIMEOUT_MS; i++)
    {
      /* Poll on LOCK (Bit 28) of PLL Display Engine Control Register */

      if ((getreg32(PLL_DE_CTRL_REG) & PLL_LOCK) != 0)
        {
          /* If LOCK is 1, then Display Engine PLL is stable */

          return OK;
        }

      /* Sleep 1 millisecond and try again */

      up_mdelay(1);
    }

  gerr("PLL Timeout");
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_de_init
 *
 * Description:
 *   Initialize the Display Engine on the SoC.  Mixer 0 will be configured
 *   to stream pixel data to Timing Controller TCON0.  Should be called
 *   before any Display Engine operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

int a64_de_init(void)
{
  int i;
  int ret;
  uint32_t pll;
  uint32_t clk;
  uint32_t clk_mask;

  /* Set High Speed SRAM to DMA Mode ****************************************/

  ginfo("Set High Speed SRAM to DMA Mode\n");

  /* SRAM Control Register 1 (A31 Page 191)
   * Set BIST_DMA_CTRL_SEL (Bit 31) to 0 (DMA)
   */

  putreg32(0x0, SRAM_CTRL_REG1);

  /* Set Display Engine PLL to 297 MHz **************************************/

  ginfo("Set Display Engine PLL to 297 MHz\n");

  /* PLL Display Engine Control Register (A64 Page 96)
   * Set PLL_ENABLE (Bit 31) to 1 (Enable PLL)
   * Set PLL_MODE_SEL (Bit 24) to 1 (Integer Mode)
   * Set PLL_FACTOR_N (Bits 8 to 14) to 23 (N = 24)
   * Set PLL_PRE_DIV_M (Bits 0 to 3) to 1 (M = 2)
   * Actual PLL Output = 24 MHz * N / M = 288 MHz
   * (Slighltly below 297 MHz due to truncation)
   */

  pll = PLL_ENABLE | PLL_MODE_SEL | PLL_FACTOR_N(23) | PLL_PRE_DIV_M(1);
  putreg32(pll, PLL_DE_CTRL_REG);

  /* Wait for Display Engine PLL to be stable *******************************/

  ginfo("Wait for Display Engine PLL to be stable\n");
  ret = a64_wait_pll();
  if (ret < 0)
    {
      return ret;
    }

  /* Set Special Clock to Display Engine PLL ********************************/

  ginfo("Set Special Clock to Display Engine PLL\n");

  /* Display Engine Clock Register (A64 Page 117)
   * Set SCLK_GATING (Bit 31) to 1
   *   (Enable Special Clock)
   * Set CLK_SRC_SEL (Bits 24 to 26) to 1
   *   (Clock Source is Display Engine PLL)
   */

  clk = SCLK_GATING | CLK_SRC_SEL(1);
  clk_mask = SCLK_GATING_MASK | CLK_SRC_SEL_MASK;
  modreg32(clk, clk_mask, DE_CLK_REG);

  /* Enable AHB for Display Engine: De-Assert Display Engine ****************/

  ginfo("Enable AHB for Display Engine: De-Assert Display Engine\n");

  /* Bus Software Reset Register 1 (A64 Page 140)
   * Set DE_RST (Bit 12) to 1 (De-Assert Display Engine)
   */

  modreg32(DE_RST, DE_RST, BUS_SOFT_RST_REG1);

  /* Enable AHB for Display Engine: Pass Display Engine *********************/

  ginfo("Enable AHB for Display Engine: Pass Display Engine\n");

  /* Bus Clock Gating Register 1 (A64 Page 102)
   * Set DE_GATING (Bit 12) to 1 (Pass Display Engine)
   */

  modreg32(DE_GATING, DE_GATING, BUS_CLK_GATING_REG1);

  /* Enable Clock for MIXER0: SCLK Clock Pass *******************************/

  ginfo("Enable Clock for MIXER0: SCLK Clock Pass\n");

  /* DE SCLK Gating Register (DE Page 25)
   * Set CORE0_SCLK_GATE (Bit 0) to 1 (Clock Pass)
   */

  modreg32(CORE0_SCLK_GATE, CORE0_SCLK_GATE, SCLK_GATE);

  /* Enable Clock for MIXER0: HCLK Clock Reset Off **************************/

  ginfo("Enable Clock for MIXER0: HCLK Clock Reset Off\n");

  /* DE AHB Reset register (DE Page 25)
   * Set CORE0_HCLK_RESET (Bit 0) to 1 (Reset Off)
   */

  modreg32(CORE0_HCLK_RESET, CORE0_HCLK_RESET, AHB_RESET);

  /* Enable Clock for MIXER0: HCLK Clock Pass *******************************/

  ginfo("Enable Clock for MIXER0: HCLK Clock Pass\n");

  /* DE HCLK Gating Register (DE Page 25)
   * Set CORE0_HCLK_GATE (Bit 0) to 1 (Clock Pass)
   */

  modreg32(CORE0_HCLK_GATE, CORE0_HCLK_GATE, HCLK_GATE);

  /* Route MIXER0 to TCON0 **************************************************/

  ginfo("Route MIXER0 to TCON0\n");

  /* DE2TCON MUX Register (DE Page 26)
   * Set DE2TCON_MUX (Bit 0) to 0
   * (Route MIXER0 to TCON0; Route MIXER1 to TCON1)
   */

  modreg32(0, DE2TCON_MUX_MASK, DE2TCON_MUX);

  /* Clear MIXER0 Registers: GLB, BLD, OVL_V, OVL_UI ************************/

  ginfo("Clear MIXER0 Registers: GLB, BLD, OVL_V, OVL_UI\n");

  for (i = 0; i < A64_MIXER0_LEN; i += 4)
    {
      putreg32(0, A64_MIXER0_ADDR + i);
    }

  /* Disable MIXER0 VSU *****************************************************/

  ginfo("Disable MIXER0 VSU\n");

  /* Video Scaler Control Register (DE Page 130)
   * Set EN (Bit 0) to 0 (Disable Video Scaler)
   */

  putreg32(0, VS_CTRL_REG);

  /* Disable MIXER0 Undocumented ********************************************/

  ginfo("Disable MIXER0 Undocumented\n");

  /* 0x113 0000 is undocumented. Might be a mix-up with UI_SCALER3. */

  putreg32(0, UNDOC_1130000);

  /* Disable MIXER0 UI_SCALER1 **********************************************/

  ginfo("Disable MIXER0 UI_SCALER1\n");

  /* UI Scaler 1 Control Register (DE Page 66)
   * Set EN (Bit 0) to 0 (Disable UI Scaler)
   */

  putreg32(0, UIS_CTRL_REG1);

  /* Disable MIXER0 UI_SCALER2 **********************************************/

  ginfo("Disable MIXER0 UI_SCALER2\n");

  /* UI Scaler 2 Control Register (DE Page 66)
   * Set EN (Bit 0) to 0 (Disable UI Scaler)
   */

  putreg32(0, UIS_CTRL_REG2);

  /* Note: Missing UI_SCALER3(CH3) at MIXER0 Offset 0x06 0000 (DE Page 90).
   * Might be a mix-up with 0x113 0000 above.
   */

  /* Disable MIXER0 FCE *****************************************************/

  ginfo("Disable MIXER0 FCE\n");

  /* Fresh and Contrast Enhancement Global Control Register (DE Page 61)
   * Set EN (Bit 0) to 0 (Disable FCE)
   */

  putreg32(0, GCTRL_REG_FCE);

  /* Disable MIXER0 BWS *****************************************************/

  ginfo("Disable MIXER0 BWS\n");

  /* Black and White Stretch Global Control Register (DE Page 42)
   * Set EN (Bit 0) to 0 (Disable BWS)
   */

  putreg32(0, GCTRL_REG_BWS);

  /* Disable MIXER0 LTI *****************************************************/

  ginfo("Disable MIXER0 LTI\n");

  /* Luminance Transient Improvement Global Control Register (DE Page 72)
   * Set LTI_EN (Bit 0) to 0 (Close LTI)
   */

  putreg32(0, LTI_CTL);

  /* Disable MIXER0 PEAKING *************************************************/

  ginfo("Disable MIXER0 PEAKING\n");

  /* Luma Peaking Module Control Register (DE Page 80)
   * Set EN (Bit 0) to 0 (Disable PEAKING)
   */

  putreg32(0, LP_CTRL_REG);

  /* Disable MIXER0 ASE *****************************************************/

  ginfo("Disable MIXER0 ASE\n");

  /* Adaptive Saturation Enhancement Global Control Register (DE Page 40)
   * Set ASE_EN (Bit 0) to 0 (Disable ASE)
   */

  putreg32(0, ASE_CTL_REG);

  /* Disable MIXER0 FCC *****************************************************/

  ginfo("Disable MIXER0 FCC\n");

  /* Fancy Color Curvature Change Control Register (DE Page 56)
   * Set Enable (Bit 0) to 0 (Disable FCC)
   */

  putreg32(0, FCC_CTL_REG);

  /* Disable MIXER0 DRC *****************************************************/

  ginfo("Disable MIXER0 DRC\n");

  /* Dynamic Range Controller Module General Control Register (DE Page 49)
   * Set BIST_EN (Bit 0) to 0 (Disable BIST)
   */

  putreg32(0, GNECTL_REG);

  /* Enable MIXER0 **********************************************************/

  ginfo("Enable MIXER0\n");

  /* Mixer 0 Global Control Register (DE Page 92)
   * Set EN (Bit 0) to 1 (Enable Mixer)
   */

  putreg32(EN_MIXER, GLB_CTL);

  return OK;
}

/****************************************************************************
 * Name: a64_de_blender_init
 *
 * Description:
 *   Initialize the UI Blender for Display Engine.  Should be called after
 *   a64_de_init() and before a64_de_ui_channel_init().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_de_blender_init(void)
{
  uint32_t color;
  uint32_t premultiply;

  /* Set Blender Background *************************************************/

  ginfo("Set Blender Background\n");

  /* Blender Background Color (DE Page 109)
   * Set to Black Background Color
   * Set RESERVED (Bits 24 to 31) to 0xFF (Undocumented)
   * Set RED (Bits 16 to 23) to 0
   * Set GREEN (Bits 8 to 15) to 0
   * Set BLUE (Bits 0 to 7) to 0
   */

  color = BK_RESERVED | BK_RED(0) | BK_GREEN(0) | BK_BLUE(0);
  putreg32(color, BLD_BK_COLOR);

  /* Set Blender Pre-Multiply ***********************************************/

  ginfo("Set Blender Pre-Multiply\n");

  /* Blender Pre-Multiply Control (DE Page 109)
   * Set to No Pre-Multiply for Alpha, Pipes 0 to 3
   * Set P3_ALPHA_MODE (Bit 3) to 0 (Pipe 3: No Pre-Multiply)
   * Set P2_ALPHA_MODE (Bit 2) to 0 (Pipe 2: No Pre-Multiply)
   * Set P1_ALPHA_MODE (Bit 1) to 0 (Pipe 1: No Pre-Multiply)
   * Set P0_ALPHA_MODE (Bit 0) to 0 (Pipe 0: No Pre-Multiply)
   */

  premultiply = P3_ALPHA_MODE(0) |
                P2_ALPHA_MODE(0) |
                P1_ALPHA_MODE(0) |
                P0_ALPHA_MODE(0);
  putreg32(premultiply, BLD_PREMUL_CTL);

  return OK;
}

/****************************************************************************
 * Name: a64_de_ui_channel_init
 *
 * Description:
 *   Initialize a UI Channel for Display Engine.  Display Engine will
 *   stream the pixel data from the Frame Buffer Memory (over DMA) to the
 *   UI Blender.  There are 3 UI Channels: Base UI Channel (Channel 1) and
 *   2 Overlay UI Channels (Channels 2 and 3).  Should be called after
 *   a64_de_blender_init() and before a64_de_enable().
 *
 * Input Parameters:
 *   channel - UI Channel Number: 1, 2 or 3
 *   fbmem   - Start of Frame Buffer Memory (address should be 32-bit),
 *             or NULL if this UI Channel should be disabled
 *   fblen   - Length of Frame Buffer Memory in bytes
 *   xres    - Horizontal resolution in pixel columns
 *   yres    - Vertical resolution in pixel rows
 *   xoffset - Horizontal offset in pixel columns
 *   yoffset - Vertical offset in pixel rows
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_de_ui_channel_init(uint8_t channel,
                           void *fbmem,
                           size_t fblen,
                           uint16_t xres,
                           uint16_t yres,
                           uint16_t xoffset,
                           uint16_t yoffset)
{
  uint8_t pipe;
  uint32_t lay_glbalpha;
  uint32_t lay_fbfmt;
  uint32_t attr;
  uint32_t height_width;
  uint32_t color;
  uint32_t offset;
  uint32_t blend;

  DEBUGASSERT(channel >= 1 && channel <= 3);
  DEBUGASSERT(fblen == xres * yres * 4);

  /* If UI Channel should be disabled... */

  if (fbmem == NULL)
    {
      /* Disable Overlay and Pipe *******************************************/

      ginfo("Channel %d: Disable Overlay and Pipe\n", channel);

      /* UI Overlay Attribute Control (DE Page 102)
       * Set LAY_EN (Bit 0) to 0 (Disable Layer)
       */

      putreg32(0, OVL_UI_ATTR_CTL(channel));

      /* Disable Scaler *****************************************************/

      ginfo("Channel %d: Disable Scaler\n", channel);

      /* UI Scaler Control Register (DE Page 66)
       * Set EN (Bit 0) to 0 (Disable UI Scaler)
       */

      putreg32(0, UIS_CTRL_REG(channel));

      /* Skip to next UI Channel */

      return OK;
    }

  /* Set Overlay (Assume Layer = 0) *****************************************/

  ginfo("Channel %d: Set Overlay (%d x %d)\n", channel, xres, yres);

  /* UI Overlay Attribute Control (DE Page 102)
   * Set LAY_GLBALPHA (Bits 24 to 31) to 0xFF or 0x7F
   *   (Global Alpha Value is Opaque or Semi-Transparent)
   * Set LAY_FBFMT (Bits 8 to 12) to 4 or 0
   *   (Input Data Format is XRGB 8888 or ARGB 8888)
   * Set LAY_ALPHA_MODE (Bits 1 to 2) to 2
   *   (Global Alpha is mixed with Pixel Alpha)
   *   (Input Alpha Value = Global Alpha Value * Pixel’s Alpha Value)
   * Set LAY_EN (Bit 0) to 1 (Enable Layer)
   */

  lay_glbalpha = (channel == 1) ? 0xff :  /* Channel 1: Opaque */
                 (channel == 2) ? 0xff :  /* Channel 2: Opaque */
                 (channel == 3) ? 0x7f :  /* Channel 3: Semi-Transparent */
                 0xff;                    /* Never comes here */

  lay_fbfmt = (channel == 1) ? 4 :  /* Channel 1: XRGB 8888 */
              (channel == 2) ? 0 :  /* Channel 2: ARGB 8888 */
              (channel == 3) ? 0 :  /* Channel 3: ARGB 8888 */
              0;                    /* Never comes here */

  attr = LAY_GLBALPHA(lay_glbalpha) |
         LAY_FBFMT(lay_fbfmt) |
         LAY_ALPHA_MODE(2) |
         LAY_EN;

  putreg32(attr, OVL_UI_ATTR_CTL(channel));

  /* UI Overlay Top Field Memory Block Low Address (DE Page 104)
   * Set to Frame Buffer Address (32 bits only)
   */

  DEBUGASSERT((((uint64_t)fbmem) & 0xffffffff) == (uint64_t)fbmem);
  putreg32((uint64_t)fbmem, OVL_UI_TOP_LADD(channel));

  /* UI Overlay Memory Pitch (DE Page 104)
   * Set to (width * 4), number of bytes per row
   */

  putreg32(xres * 4, OVL_UI_PITCH(channel));

  /* UI Overlay Memory Block Size (DE Page 104)
   * Set to (height-1) << 16 + (width-1)
   */

  height_width = ((yres - 1) << 16) | (xres - 1);
  putreg32(height_width, OVL_UI_MBSIZE(channel));

  /* UI Overlay Overlay Window Size (DE Page 106)
   * Set to (height-1) << 16 + (width-1)
   */

  putreg32(height_width, OVL_UI_SIZE(channel));

  /* UI Overlay Memory Block Coordinate (DE Page 104)
   * Set to 0 (Overlay at X=0, Y=0)
   */

  putreg32(0, OVL_UI_COOR(channel));

  /* For Channel 1: Set Blender Output */

  if (channel == 1)
    {
      /* Set Blender Output *************************************************/

      ginfo("Channel %d: Set Blender Output\n", channel);

      /* Blender Output Size Setting (DE Page 110)
       * Set to (height-1) << 16 + (width-1)
       */

      putreg32(height_width, BLD_SIZE);

      /* Global Size (DE Page 93)
       * Set to (height-1) << 16 + (width-1)
       */

      putreg32(height_width, GLB_SIZE);
    }

  /* Set Blender Input Pipe *************************************************/

  pipe = channel - 1;  /* Pipe Number is 0 to 2 for Channels 1 to 3 */
  ginfo("Channel %d: Set Blender Input Pipe %d (%d x %d)\n",
        channel, pipe, xres, yres);

  /* Blender Input Memory Size (DE Page 108)
   * Set to (height-1) << 16 + (width-1)
   */

  putreg32(height_width, BLD_CH_ISIZE(pipe));

  /* Blender Fill Color (DE Page 107)
   * Set to Opaque Black
   * Set ALPHA (Bits 24 to 31) to 0xFF (Opaque)
   * Set RED (Bits 16 to 23) to 0
   * Set GREEN (Bits 8 to 15) to 0
   * Set BLUE (Bits 0 to 7) to 0
   */

  color = FILL_ALPHA(0xff) | FILL_RED(0) | FILL_GREEN(0) | FILL_BLUE(0);
  putreg32(color, BLD_FILL_COLOR(pipe));

  /* Blender Input Memory Offset (DE Page 108)
   * Set to y_offset << 16 + x_offset
   */

  offset = ((yoffset) << 16) | xoffset;
  putreg32(offset, BLD_CH_OFFSET(pipe));

  /* Blender Control (DE Page 110)
   * Set BLEND_AFD (Bits 24 to 27) to 3
   *   (Coefficient for destination alpha data Q[d] is 1-A[s])
   * Set BLEND_AFS (Bits 16 to 19) to 1
   *   (Coefficient for source alpha data Q[s] is 1)
   * Set BLEND_PFD (Bits 8 to 11) to 3
   *   (Coefficient for destination pixel data F[d] is 1-A[s])
   * Set BLEND_PFS (Bits 0 to 3) to 1
   *   (Coefficient for source pixel data F[s] is 1)
   */

  blend = BLEND_AFD(3) | BLEND_AFS(1) | BLEND_PFD(3) | BLEND_PFS(1);
  putreg32(blend, BLD_CTL(pipe));

  /* Disable Scaler *********************************************************/

  ginfo("Channel %d: Disable Scaler\n", channel);

  /* UI Scaler Control Register (DE Page 66)
   * Set EN (Bit 0) to 0 (Disable UI Scaler)
   */

  putreg32(0, UIS_CTRL_REG(channel));

  return OK;
}

/****************************************************************************
 * Name: a64_de_enable
 *
 * Description:
 *   Set the UI Blender Route, enable the Blender Pipes and enable the
 *   Display Engine.  Should be called after all 3 UI Channels have been
 *   initialized.
 *
 * Input Parameters:
 *   channels - Number of UI Channels to enable: 1 or 3
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_de_enable(uint8_t channels)
{
  uint32_t p2_rtctl;
  uint32_t p1_rtctl;
  uint32_t route;
  uint32_t p2_en;
  uint32_t p1_en;
  uint32_t fill;

  DEBUGASSERT(channels == 1 || channels == 3);

  /* Set Blender Route ******************************************************/

  ginfo("Set Blender Route\n");

  /* Blender Routing Control (DE Page 108)
   * If Rendering 1 UI Channel:
   *   Set P0_RTCTL (Bits 0 to 3) to 1 (Pipe 0 from Channel 1)
   * If Rendering 3 UI Channels:
   *   Set P2_RTCTL (Bits 8 to 11) to 3 (Pipe 2 from Channel 3)
   *   Set P1_RTCTL (Bits 4 to 7) to 2 (Pipe 1 from Channel 2)
   *   Set P0_RTCTL (Bits 0 to 3) to 1 (Pipe 0 from Channel 1)
   */

  p2_rtctl = (channels == 1) ? 0 :  /* Unused Pipe 2 */
             (channels == 3) ? 3 :  /* Select Pipe 2 from UI Channel 3 */
             0;                     /* Never comes here */

  p1_rtctl = (channels == 1) ? 0 :  /* Unused Pipe 1 */
             (channels == 3) ? 2 :  /* Select Pipe 1 from UI Channel 2 */
             0;                     /* Never comes here */

  route = P2_RTCTL(p2_rtctl) | P1_RTCTL(p1_rtctl) | P0_RTCTL(1);
  putreg32(route, BLD_CH_RTCTL);

  /* Enable Blender Pipes ***************************************************/

  ginfo("Enable Blender Pipes\n");

  /* Blender Fill Color Control (DE Page 106)
   * If Rendering 1 UI Channel:
   *   Set P0_EN (Bit 8) to 1 (Enable Pipe 0)
   *   Set P0_FCEN (Bit 0) to 1 (Enable Pipe 0 Fill Color)
   * If Rendering 3 UI Channels:
   *   Set P2_EN (Bit 10) to 1 (Enable Pipe 2)
   *   Set P1_EN (Bit 9) to 1 (Enable Pipe 1)
   *   Set P0_EN (Bit 8) to 1 (Enable Pipe 0)
   *   Set P0_FCEN (Bit 0) to 1 (Enable Pipe 0 Fill Color)
   */

  p2_en = (channels == 1) ? 0 :  /* 1 UI Channel: Disable Pipe 2 */
          (channels == 3) ? 1 :  /* 3 UI Channels: Enable Pipe 2 */
          0;                     /* Never comes here */

  p1_en = (channels == 1) ? 0 :  /* 1 UI Channel: Disable Pipe 1 */
          (channels == 3) ? 1 :  /* 3 UI Channels: Enable Pipe 1 */
          0;                     /* Never comes here */

  fill = P2_EN(p2_en) | P1_EN(p1_en) | P0_EN(1) | P0_FCEN(1);
  putreg32(fill, BLD_FILL_COLOR_CTL);

  /* Apply Settings *********************************************************/

  ginfo("Apply Settings\n");

  /* Global Double Buffer Control (DE Page 93)
   * Set DOUBLE_BUFFER_RDY (Bit 0) to 1
   * (Register Value is ready for update)
   */

  putreg32(DOUBLE_BUFFER_RDY, GLB_DBUFFER);

  return OK;
}
