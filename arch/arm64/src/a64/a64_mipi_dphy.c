/****************************************************************************
 * arch/arm64/src/a64/a64_mipi_dphy.c
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
 * "Understanding PinePhone's Display (MIPI DSI)"
 * https://lupyuen.github.io/articles/dsi
 *
 * "NuttX RTOS for PinePhone: Display Driver in Zig"
 * https://lupyuen.github.io/articles/dsi2
 *
 * "A64 Page" refers to Allwinner A64 User Manual
 * https://lupyuen.github.io/images/Allwinner_A64_User_Manual_V1.1.pdf
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
#include "arm64_arch.h"
#include "a64_mipi_dphy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A64 CCU Registers and Bit Definitions ************************************/

/* MIPI_DSI Clock Register */

#define MIPI_DSI_CLK_REG    (A64_CCU_ADDR + 0x168)
#define DPHY_CLK_DIV_M(n)   ((n) << 0)
#define DSI_DPHY_SRC_SEL(n) ((n) << 8)
#define DSI_DPHY_GATING     (1 << 15)

/* A64 MIPI D-PHY Registers (Undocumented) **********************************/

#define DPHY_TX_CTL_REG   (A64_DPHY_ADDR + 0x04)
#define DPHY_TX_TIME0_REG (A64_DPHY_ADDR + 0x10)
#define DPHY_TX_TIME1_REG (A64_DPHY_ADDR + 0x14)
#define DPHY_TX_TIME2_REG (A64_DPHY_ADDR + 0x18)
#define DPHY_TX_TIME3_REG (A64_DPHY_ADDR + 0x1c)
#define DPHY_TX_TIME4_REG (A64_DPHY_ADDR + 0x20)
#define DPHY_GCTL_REG     (A64_DPHY_ADDR + 0x00)
#define DPHY_ANA0_REG     (A64_DPHY_ADDR + 0x4c)
#define DPHY_ANA1_REG     (A64_DPHY_ADDR + 0x50)
#define DPHY_ANA4_REG     (A64_DPHY_ADDR + 0x5c)
#define DPHY_ANA2_REG     (A64_DPHY_ADDR + 0x54)
#define DPHY_ANA3_REG     (A64_DPHY_ADDR + 0x58)

/* A64 MIPI D-PHY Values (Undocumented) */

#define ANA1_VTTMODE      0x80000000
#define ANA2_ENABLECKCPU  0x10
#define ANA2_ENABLEP2SCPU 0xf000000
#define ANA3_ENABLEVTTC   0xf8000000
#define ANA3_ENABLEDIV    0x4000000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_mipi_dphy_enable
 *
 * Description:
 *   Enable MIPI Display Physical Layer (D-PHY).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_mipi_dphy_enable(void)
{
  uint32_t mipi_dsi_clk;

  /* Set DSI Clock to 150 MHz (600 MHz / 4) *********************************/

  ginfo("Set DSI Clock to 150 MHz\n");

  /* MIPI_DSI Clock Register
   * Set DSI_DPHY_GATING (Bit 15) to 1
   *   (DSI DPHY Clock is On)
   * Set DSI_DPHY_SRC_SEL (Bits 8 to 9) to 0b10
   *   (DSI DPHY Clock Source is PLL_PERIPH0(1X))
   * Set DPHY_CLK_DIV_M (Bits 0 to 3) to 3
   *   (DSI DPHY Clock divide ratio - 1)
   */

  mipi_dsi_clk = DSI_DPHY_GATING |
                 DSI_DPHY_SRC_SEL(0b10) |
                 DPHY_CLK_DIV_M(3);
  putreg32(mipi_dsi_clk, MIPI_DSI_CLK_REG);

  /* Power on DPHY Tx (Undocumented) ****************************************/

  ginfo("Power on DPHY Tx\n");
  putreg32(0x10000000, DPHY_TX_CTL_REG);
  putreg32(0xa06000e,  DPHY_TX_TIME0_REG);
  putreg32(0xa033207,  DPHY_TX_TIME1_REG);
  putreg32(0x1e,       DPHY_TX_TIME2_REG);
  putreg32(0x0,        DPHY_TX_TIME3_REG);
  putreg32(0x303,      DPHY_TX_TIME4_REG);

  /* Enable DPHY (Undocumented) *********************************************/

  ginfo("Enable DPHY\n");
  putreg32(0x31,       DPHY_GCTL_REG);
  putreg32(0x9f007f00, DPHY_ANA0_REG);
  putreg32(0x17000000, DPHY_ANA1_REG);
  putreg32(0x1f01555,  DPHY_ANA4_REG);
  putreg32(0x2,        DPHY_ANA2_REG);
  up_mdelay(1);  /* Wait at least 5 microseconds */

  /* Enable LDOR, LDOC, LDOD (Undocumented) *********************************/

  ginfo("Enable LDOR, LDOC, LDOD\n");
  putreg32(0x3040000, DPHY_ANA3_REG);
  up_mdelay(1);  /* Wait at least 1 microsecond */

  modreg32(ANA3_ENABLEVTTC,   ANA3_ENABLEVTTC,   DPHY_ANA3_REG);
  up_mdelay(1);  /* Wait at least 1 microsecond */

  modreg32(ANA3_ENABLEDIV,    ANA3_ENABLEDIV,    DPHY_ANA3_REG);
  up_mdelay(1);  /* Wait at least 1 microsecond */

  modreg32(ANA2_ENABLECKCPU,  ANA2_ENABLECKCPU,  DPHY_ANA2_REG);
  up_mdelay(1);  /* Wait at least 1 microsecond */

  modreg32(ANA1_VTTMODE,      ANA1_VTTMODE,      DPHY_ANA1_REG);
  modreg32(ANA2_ENABLEP2SCPU, ANA2_ENABLEP2SCPU, DPHY_ANA2_REG);

  return OK;
}
