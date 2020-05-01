/****************************************************************************
 * arch/arm/src/imx6/imx_irq.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include "arm_arch.h"
#include "imx_iomuxc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This table is indexed by the Pad Mux register index and provides the index
 * to the corresponding Pad Control register.
 *
 * REVISIT:  This could be greatly simplified:  The Pad Control registers
 * map 1-to-1 with the Pad Mux registers except for two regions where
 * there are no corresponding Pad Mux registers.  The entire table could be
 * replaced to two range checks and the appropriate offset added to the Pad
 * Mux Register index.
 */

static const uint8_t g_mux2ctl_map[IMX_PADMUX_NREGISTERS] =
{
  /* The first mappings are simple 1-to-1 mappings.  This may be a little wasteful */

  IMX_PADCTL_SD2_DATA1_INDEX,       /* IMX_PADMUX_SD2_DATA1_INDEX */
  IMX_PADCTL_SD2_DATA2_INDEX,       /* IMX_PADMUX_SD2_DATA2_INDEX */
  IMX_PADCTL_SD2_DATA0_INDEX,       /* IMX_PADMUX_SD2_DATA0_INDEX */
  IMX_PADCTL_RGMII_TXC_INDEX,       /* IMX_PADMUX_RGMII_TXC_INDEX */
  IMX_PADCTL_RGMII_TD0_INDEX,       /* IMX_PADMUX_RGMII_TD0_INDEX */
  IMX_PADCTL_RGMII_TD1_INDEX,       /* IMX_PADMUX_RGMII_TD1_INDEX */
  IMX_PADCTL_RGMII_TD2_INDEX,       /* IMX_PADMUX_RGMII_TD2_INDEX */
  IMX_PADCTL_RGMII_TD3_INDEX,       /* IMX_PADMUX_RGMII_TD3_INDEX */
  IMX_PADCTL_RGMII_RX_CTL_INDEX,    /* IMX_PADMUX_RGMII_RX_CTL_INDEX */
  IMX_PADCTL_RGMII_RD0_INDEX,       /* IMX_PADMUX_RGMII_RD0_INDEX */
  IMX_PADCTL_RGMII_TX_CTL_INDEX,    /* IMX_PADMUX_RGMII_TX_CTL_INDEX */
  IMX_PADCTL_RGMII_RD1_INDEX,       /* IMX_PADMUX_RGMII_RD1_INDEX */
  IMX_PADCTL_RGMII_RD2_INDEX,       /* IMX_PADMUX_RGMII_RD2_INDEX */
  IMX_PADCTL_RGMII_RD3_INDEX,       /* IMX_PADMUX_RGMII_RD3_INDEX */
  IMX_PADCTL_RGMII_RXC_INDEX,       /* IMX_PADMUX_RGMII_RXC_INDEX */
  IMX_PADCTL_EIM_ADDR25_INDEX,      /* IMX_PADMUX_EIM_ADDR25_INDEX */
  IMX_PADCTL_EIM_EB2_INDEX,         /* IMX_PADMUX_EIM_EB2_INDEX */
  IMX_PADCTL_EIM_DATA16_INDEX,      /* IMX_PADMUX_EIM_DATA16_INDEX */
  IMX_PADCTL_EIM_DATA17_INDEX,      /* IMX_PADMUX_EIM_DATA17_INDEX */
  IMX_PADCTL_EIM_DATA18_INDEX,      /* IMX_PADMUX_EIM_DATA18_INDEX */
  IMX_PADCTL_EIM_DATA19_INDEX,      /* IMX_PADMUX_EIM_DATA19_INDEX */
  IMX_PADCTL_EIM_DATA20_INDEX,      /* IMX_PADMUX_EIM_DATA20_INDEX */
  IMX_PADCTL_EIM_DATA21_INDEX,      /* IMX_PADMUX_EIM_DATA21_INDEX */
  IMX_PADCTL_EIM_DATA22_INDEX,      /* IMX_PADMUX_EIM_DATA22_INDEX */
  IMX_PADCTL_EIM_DATA23_INDEX,      /* IMX_PADMUX_EIM_DATA23_INDEX */
  IMX_PADCTL_EIM_EB3_INDEX,         /* IMX_PADMUX_EIM_EB3_INDEX */
  IMX_PADCTL_EIM_DATA24_INDEX,      /* IMX_PADMUX_EIM_DATA24_INDEX */
  IMX_PADCTL_EIM_DATA25_INDEX,      /* IMX_PADMUX_EIM_DATA25_INDEX */
  IMX_PADCTL_EIM_DATA26_INDEX,      /* IMX_PADMUX_EIM_DATA26_INDEX */
  IMX_PADCTL_EIM_DATA27_INDEX,      /* IMX_PADMUX_EIM_DATA27_INDEX */
  IMX_PADCTL_EIM_DATA28_INDEX,      /* IMX_PADMUX_EIM_DATA28_INDEX */
  IMX_PADCTL_EIM_DATA29_INDEX,      /* IMX_PADMUX_EIM_DATA29_INDEX */
  IMX_PADCTL_EIM_DATA30_INDEX,      /* IMX_PADMUX_EIM_DATA30_INDEX */
  IMX_PADCTL_EIM_DATA31_INDEX,      /* IMX_PADMUX_EIM_DATA31_INDEX */
  IMX_PADCTL_EIM_ADDR24_INDEX,      /* IMX_PADMUX_EIM_ADDR24_INDEX */
  IMX_PADCTL_EIM_ADDR23_INDEX,      /* IMX_PADMUX_EIM_ADDR23_INDEX */
  IMX_PADCTL_EIM_ADDR22_INDEX,      /* IMX_PADMUX_EIM_ADDR22_INDEX */
  IMX_PADCTL_EIM_ADDR21_INDEX,      /* IMX_PADMUX_EIM_ADDR21_INDEX */
  IMX_PADCTL_EIM_ADDR20_INDEX,      /* IMX_PADMUX_EIM_ADDR20_INDEX */
  IMX_PADCTL_EIM_ADDR19_INDEX,      /* IMX_PADMUX_EIM_ADDR19_INDEX */
  IMX_PADCTL_EIM_ADDR18_INDEX,      /* IMX_PADMUX_EIM_ADDR18_INDEX */
  IMX_PADCTL_EIM_ADDR17_INDEX,      /* IMX_PADMUX_EIM_ADDR17_INDEX */
  IMX_PADCTL_EIM_ADDR16_INDEX,      /* IMX_PADMUX_EIM_ADDR16_INDEX */
  IMX_PADCTL_EIM_CS0_INDEX,         /* IMX_PADMUX_EIM_CS0_INDEX */
  IMX_PADCTL_EIM_CS1_INDEX,         /* IMX_PADMUX_EIM_CS1_INDEX */
  IMX_PADCTL_EIM_OE_INDEX,          /* IMX_PADMUX_EIM_OE_INDEX */
  IMX_PADCTL_EIM_RW_INDEX,          /* IMX_PADMUX_EIM_RW_INDEX */
  IMX_PADCTL_EIM_LBA_INDEX,         /* IMX_PADMUX_EIM_LBA_INDEX */
  IMX_PADCTL_EIM_EB0_INDEX,         /* IMX_PADMUX_EIM_EB0_INDEX */
  IMX_PADCTL_EIM_EB1_INDEX,         /* IMX_PADMUX_EIM_EB1_INDEX */
  IMX_PADCTL_EIM_AD00_INDEX,        /* IMX_PADMUX_EIM_AD00_INDEX */
  IMX_PADCTL_EIM_AD01_INDEX,        /* IMX_PADMUX_EIM_AD01_INDEX */
  IMX_PADCTL_EIM_AD02_INDEX,        /* IMX_PADMUX_EIM_AD02_INDEX */
  IMX_PADCTL_EIM_AD03_INDEX,        /* IMX_PADMUX_EIM_AD03_INDEX */
  IMX_PADCTL_EIM_AD04_INDEX,        /* IMX_PADMUX_EIM_AD04_INDEX */
  IMX_PADCTL_EIM_AD05_INDEX,        /* IMX_PADMUX_EIM_AD05_INDEX */
  IMX_PADCTL_EIM_AD06_INDEX,        /* IMX_PADMUX_EIM_AD06_INDEX */
  IMX_PADCTL_EIM_AD07_INDEX,        /* IMX_PADMUX_EIM_AD07_INDEX */
  IMX_PADCTL_EIM_AD08_INDEX,        /* IMX_PADMUX_EIM_AD08_INDEX */
  IMX_PADCTL_EIM_AD09_INDEX,        /* IMX_PADMUX_EIM_AD09_INDEX */
  IMX_PADCTL_EIM_AD10_INDEX,        /* IMX_PADMUX_EIM_AD10_INDEX */
  IMX_PADCTL_EIM_AD11_INDEX,        /* IMX_PADMUX_EIM_AD11_INDEX */
  IMX_PADCTL_EIM_AD12_INDEX,        /* IMX_PADMUX_EIM_AD12_INDEX */
  IMX_PADCTL_EIM_AD13_INDEX,        /* IMX_PADMUX_EIM_AD13_INDEX */
  IMX_PADCTL_EIM_AD14_INDEX,        /* IMX_PADMUX_EIM_AD14_INDEX */
  IMX_PADCTL_EIM_AD15_INDEX,        /* IMX_PADMUX_EIM_AD15_INDEX */
  IMX_PADCTL_EIM_WAIT_INDEX,        /* IMX_PADMUX_EIM_WAIT_INDEX */
  IMX_PADCTL_EIM_BCLK_INDEX,        /* IMX_PADMUX_EIM_BCLK_INDEX */
  IMX_PADCTL_DI0_DISP_CLK_INDEX,    /* IMX_PADMUX_DI0_DISP_CLK_INDEX */
  IMX_PADCTL_DI0_PIN15_INDEX,       /* IMX_PADMUX_DI0_PIN15_INDEX */
  IMX_PADCTL_DI0_PIN02_INDEX,       /* IMX_PADMUX_DI0_PIN02_INDEX */
  IMX_PADCTL_DI0_PIN03_INDEX,       /* IMX_PADMUX_DI0_PIN03_INDEX */
  IMX_PADCTL_DI0_PIN04_INDEX,       /* IMX_PADMUX_DI0_PIN04_INDEX */
  IMX_PADCTL_DISP0_DATA00_INDEX,    /* IMX_PADMUX_DISP0_DATA00_INDEX */
  IMX_PADCTL_DISP0_DATA01_INDEX,    /* IMX_PADMUX_DISP0_DATA01_INDEX */
  IMX_PADCTL_DISP0_DATA02_INDEX,    /* IMX_PADMUX_DISP0_DATA02_INDEX */
  IMX_PADCTL_DISP0_DATA03_INDEX,    /* IMX_PADMUX_DISP0_DATA03_INDEX */
  IMX_PADCTL_DISP0_DATA04_INDEX,    /* IMX_PADMUX_DISP0_DATA04_INDEX */
  IMX_PADCTL_DISP0_DATA05_INDEX,    /* IMX_PADMUX_DISP0_DATA05_INDEX */
  IMX_PADCTL_DISP0_DATA06_INDEX,    /* IMX_PADMUX_DISP0_DATA06_INDEX */
  IMX_PADCTL_DISP0_DATA07_INDEX,    /* IMX_PADMUX_DISP0_DATA07_INDEX */
  IMX_PADCTL_DISP0_DATA08_INDEX,    /* IMX_PADMUX_DISP0_DATA08_INDEX */
  IMX_PADCTL_DISP0_DATA09_INDEX,    /* IMX_PADMUX_DISP0_DATA09_INDEX */
  IMX_PADCTL_DISP0_DATA10_INDEX,    /* IMX_PADMUX_DISP0_DATA10_INDEX */
  IMX_PADCTL_DISP0_DATA11_INDEX,    /* IMX_PADMUX_DISP0_DATA11_INDEX */
  IMX_PADCTL_DISP0_DATA12_INDEX,    /* IMX_PADMUX_DISP0_DATA12_INDEX */
  IMX_PADCTL_DISP0_DATA13_INDEX,    /* IMX_PADMUX_DISP0_DATA13_INDEX */
  IMX_PADCTL_DISP0_DATA14_INDEX,    /* IMX_PADMUX_DISP0_DATA14_INDEX */
  IMX_PADCTL_DISP0_DATA15_INDEX,    /* IMX_PADMUX_DISP0_DATA15_INDEX */


  IMX_PADCTL_DISP0_DATA16_INDEX,    /* IMX_PADMUX_DISP0_DATA16_INDEX */
  IMX_PADCTL_DISP0_DATA17_INDEX,    /* IMX_PADMUX_DISP0_DATA17_INDEX */
  IMX_PADCTL_DISP0_DATA18_INDEX,    /* IMX_PADMUX_DISP0_DATA18_INDEX */
  IMX_PADCTL_DISP0_DATA19_INDEX,    /* IMX_PADMUX_DISP0_DATA19_INDEX */
  IMX_PADCTL_DISP0_DATA20_INDEX,    /* IMX_PADMUX_DISP0_DATA20_INDEX */
  IMX_PADCTL_DISP0_DATA21_INDEX,    /* IMX_PADMUX_DISP0_DATA21_INDEX */
  IMX_PADCTL_DISP0_DATA22_INDEX,    /* IMX_PADMUX_DISP0_DATA22_INDEX */
  IMX_PADCTL_DISP0_DATA23_INDEX,    /* IMX_PADMUX_DISP0_DATA23_INDEX */
  IMX_PADCTL_ENET_MDIO_INDEX,       /* IMX_PADMUX_ENET_MDIO_INDEX */
  IMX_PADCTL_ENET_REF_CLK_INDEX,    /* IMX_PADMUX_ENET_REF_CLK_INDEX */
  IMX_PADCTL_ENET_RX_ER_INDEX,      /* IMX_PADMUX_ENET_RX_ER_INDEX */
  IMX_PADCTL_ENET_CRS_DV_INDEX,     /* IMX_PADMUX_ENET_CRS_DV_INDEX */
  IMX_PADCTL_ENET_RX_DATA1_INDEX,   /* IMX_PADMUX_ENET_RX_DATA1_INDEX */
  IMX_PADCTL_ENET_RX_DATA0_INDEX,   /* IMX_PADMUX_ENET_RX_DATA0_INDEX */
  IMX_PADCTL_ENET_TX_EN_INDEX,      /* IMX_PADMUX_ENET_TX_EN_INDEX */
  IMX_PADCTL_ENET_TX_DATA1_INDEX,   /* IMX_PADMUX_ENET_TX_DATA1_INDEX */
  IMX_PADCTL_ENET_TX_DATA0_INDEX,   /* IMX_PADMUX_ENET_TX_DATA0_INDEX */
  IMX_PADCTL_ENET_MDC_INDEX,        /* IMX_PADMUX_ENET_MDC_INDEX */

  /* There is then a group of Pad Control registers with no Pad Mux register counterpart */

                                    /* IMX_PADCTL_DRAM_SDQS5_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM5_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM4_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS4_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS3_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM3_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS2_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM2_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR00_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR01_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR02_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR03_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR04_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR05_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR06_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR07_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR08_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR09_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR10_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR11_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR12_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR13_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR14_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ADDR15_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_CAS_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_CS0_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_CS1_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_RAS_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_RESET_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDBA0_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDBA1_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDCLK0_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDBA2_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDCKE0_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDCLK1_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDCKE1_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ODT0_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_ODT1_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDWE_B_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS0_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM0_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS1_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM1_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS6_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM6_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_SDQS7_P_INDEX - No counterpart */
                                    /* IMX_PADCTL_DRAM_DQM7_INDEX - No counterpart */

  /* The mapping is again 1-to-1 with an offset for the above registers that
   * have no Pad Mux register counterpart.
   */

  IMX_PADCTL_KEY_COL0_INDEX,        /* IMX_PADMUX_KEY_COL0_INDEX */
  IMX_PADCTL_KEY_ROW0_INDEX,        /* IMX_PADMUX_KEY_ROW0_INDEX */
  IMX_PADCTL_KEY_COL1_INDEX,        /* IMX_PADMUX_KEY_COL1_INDEX */
  IMX_PADCTL_KEY_ROW1_INDEX,        /* IMX_PADMUX_KEY_ROW1_INDEX */
  IMX_PADCTL_KEY_COL2_INDEX,        /* IMX_PADMUX_KEY_COL2_INDEX */
  IMX_PADCTL_KEY_ROW2_INDEX,        /* IMX_PADMUX_KEY_ROW2_INDEX */
  IMX_PADCTL_KEY_COL3_INDEX,        /* IMX_PADMUX_KEY_COL3_INDEX */
  IMX_PADCTL_KEY_ROW3_INDEX,        /* IMX_PADMUX_KEY_ROW3_INDEX */
  IMX_PADCTL_KEY_COL4_INDEX,        /* IMX_PADMUX_KEY_COL4_INDEX */
  IMX_PADCTL_KEY_ROW4_INDEX,        /* IMX_PADMUX_KEY_ROW4_INDEX */
  IMX_PADCTL_GPIO00_INDEX,          /* IMX_PADMUX_GPIO00_INDEX */
  IMX_PADCTL_GPIO01_INDEX,          /* IMX_PADMUX_GPIO01_INDEX */
  IMX_PADCTL_GPIO09_INDEX,          /* IMX_PADMUX_GPIO09_INDEX */
  IMX_PADCTL_GPIO03_INDEX,          /* IMX_PADMUX_GPIO03_INDEX */
  IMX_PADCTL_GPIO06_INDEX,          /* IMX_PADMUX_GPIO06_INDEX */
  IMX_PADCTL_GPIO02_INDEX,          /* IMX_PADMUX_GPIO02_INDEX */
  IMX_PADCTL_GPIO04_INDEX,          /* IMX_PADMUX_GPIO04_INDEX */
  IMX_PADCTL_GPIO05_INDEX,          /* IMX_PADMUX_GPIO05_INDEX */
  IMX_PADCTL_GPIO07_INDEX,          /* IMX_PADMUX_GPIO07_INDEX */
  IMX_PADCTL_GPIO08_INDEX,          /* IMX_PADMUX_GPIO08_INDEX */
  IMX_PADCTL_GPIO16_INDEX,          /* IMX_PADMUX_GPIO16_INDEX */
  IMX_PADCTL_GPIO17_INDEX,          /* IMX_PADMUX_GPIO17_INDEX */
  IMX_PADCTL_GPIO18_INDEX,          /* IMX_PADMUX_GPIO18_INDEX */
  IMX_PADCTL_GPIO19_INDEX,          /* IMX_PADMUX_GPIO19_INDEX */
  IMX_PADCTL_CSI0_PIXCLK_INDEX,     /* IMX_PADMUX_CSI0_PIXCLK_INDEX */
  IMX_PADCTL_CSI0_HSYNC_INDEX,      /* IMX_PADMUX_CSI0_HSYNC_INDEX */
  IMX_PADCTL_CSI0_DATA_EN_INDEX,    /* IMX_PADMUX_CSI0_DATA_EN_INDEX */
  IMX_PADCTL_CSI0_VSYNC_INDEX,      /* IMX_PADMUX_CSI0_VSYNC_INDEX */
  IMX_PADCTL_CSI0_DATA04_INDEX,     /* IMX_PADMUX_CSI0_DATA04_INDEX */
  IMX_PADCTL_CSI0_DATA05_INDEX,     /* IMX_PADMUX_CSI0_DATA05_INDEX */
  IMX_PADCTL_CSI0_DATA06_INDEX,     /* IMX_PADMUX_CSI0_DATA06_INDEX */
  IMX_PADCTL_CSI0_DATA07_INDEX,     /* IMX_PADMUX_CSI0_DATA07_INDEX */
  IMX_PADCTL_CSI0_DATA08_INDEX,     /* IMX_PADMUX_CSI0_DATA08_INDEX */
  IMX_PADCTL_CSI0_DATA09_INDEX,     /* IMX_PADMUX_CSI0_DATA09_INDEX */
  IMX_PADCTL_CSI0_DATA10_INDEX,     /* IMX_PADMUX_CSI0_DATA10_INDEX */
  IMX_PADCTL_CSI0_DATA11_INDEX,     /* IMX_PADMUX_CSI0_DATA11_INDEX */
  IMX_PADCTL_CSI0_DATA12_INDEX,     /* IMX_PADMUX_CSI0_DATA12_INDEX */
  IMX_PADCTL_CSI0_DATA13_INDEX,     /* IMX_PADMUX_CSI0_DATA13_INDEX */
  IMX_PADCTL_CSI0_DATA14_INDEX,     /* IMX_PADMUX_CSI0_DATA14_INDEX */
  IMX_PADCTL_CSI0_DATA15_INDEX,     /* IMX_PADMUX_CSI0_DATA15_INDEX */
  IMX_PADCTL_CSI0_DATA16_INDEX,     /* IMX_PADMUX_CSI0_DATA16_INDEX */
  IMX_PADCTL_CSI0_DATA17_INDEX,     /* IMX_PADMUX_CSI0_DATA17_INDEX */
  IMX_PADCTL_CSI0_DATA18_INDEX,     /* IMX_PADMUX_CSI0_DATA18_INDEX */
  IMX_PADCTL_CSI0_DATA19_INDEX,     /* IMX_PADMUX_CSI0_DATA19_INDEX */

  /* There is a second group of Pad Control registers with no Pad Mux register counterpart */

                                    /* IMX_PADCTL_JTAG_TMS_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_MOD_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TRSTB_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TDI_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TCK_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TDO_INDEX - No counterpart */

  /* The mapping is again 1-to-1 with an offset for the above registers that
   * have no Pad Mux register counterpart.
   */

  IMX_PADCTL_SD3_DATA7_INDEX,       /* IMX_PADMUX_SD3_DATA7_INDEX */
  IMX_PADCTL_SD3_DATA6_INDEX,       /* IMX_PADMUX_SD3_DATA6_INDEX */
  IMX_PADCTL_SD3_DATA5_INDEX,       /* IMX_PADMUX_SD3_DATA5_INDEX */
  IMX_PADCTL_SD3_DATA4_INDEX,       /* IMX_PADMUX_SD3_DATA4_INDEX */
  IMX_PADCTL_SD3_CMD_INDEX,         /* IMX_PADMUX_SD3_CMD_INDEX */
  IMX_PADCTL_SD3_CLK_INDEX,         /* IMX_PADMUX_SD3_CLK_INDEX */
  IMX_PADCTL_SD3_DATA0_INDEX,       /* IMX_PADMUX_SD3_DATA0_INDEX */
  IMX_PADCTL_SD3_DATA1_INDEX,       /* IMX_PADMUX_SD3_DATA1_INDEX */
  IMX_PADCTL_SD3_DATA2_INDEX,       /* IMX_PADMUX_SD3_DATA2_INDEX */
  IMX_PADCTL_SD3_DATA3_INDEX,       /* IMX_PADMUX_SD3_DATA3_INDEX */
  IMX_PADCTL_SD3_RESET_INDEX,       /* IMX_PADMUX_SD3_RESET_INDEX */
  IMX_PADCTL_NAND_CLE_INDEX,        /* IMX_PADMUX_NAND_CLE_INDEX */
  IMX_PADCTL_NAND_ALE_INDEX,        /* IMX_PADMUX_NAND_ALE_INDEX */
  IMX_PADCTL_NAND_WP_INDEX,         /* IMX_PADMUX_NAND_WP_INDEX */
  IMX_PADCTL_NAND_READY_INDEX,      /* IMX_PADMUX_NAND_READY_INDEX */
  IMX_PADCTL_NAND_CS0_INDEX,        /* IMX_PADMUX_NAND_CS0_INDEX */
  IMX_PADCTL_NAND_CS1_INDEX,        /* IMX_PADMUX_NAND_CS1_INDEX */
  IMX_PADCTL_NAND_CS2_INDEX,        /* IMX_PADMUX_NAND_CS2_INDEX */
  IMX_PADCTL_NAND_CS3_INDEX,        /* IMX_PADMUX_NAND_CS3_INDEX */
  IMX_PADCTL_SD4_CMD_INDEX,         /* IMX_PADMUX_SD4_CMD_INDEX */
  IMX_PADCTL_SD4_CLK_INDEX,         /* IMX_PADMUX_SD4_CLK_INDEX */
  IMX_PADCTL_NAND_DATA00_INDEX,     /* IMX_PADMUX_NAND_DATA00_INDEX */
  IMX_PADCTL_NAND_DATA01_INDEX,     /* IMX_PADMUX_NAND_DATA01_INDEX */
  IMX_PADCTL_NAND_DATA02_INDEX,     /* IMX_PADMUX_NAND_DATA02_INDEX */
  IMX_PADCTL_NAND_DATA03_INDEX,     /* IMX_PADMUX_NAND_DATA03_INDEX */
  IMX_PADCTL_NAND_DATA04_INDEX,     /* IMX_PADMUX_NAND_DATA04_INDEX */
  IMX_PADCTL_NAND_DATA05_INDEX,     /* IMX_PADMUX_NAND_DATA05_INDEX */
  IMX_PADCTL_NAND_DATA06_INDEX,     /* IMX_PADMUX_NAND_DATA06_INDEX */
  IMX_PADCTL_NAND_DATA07_INDEX,     /* IMX_PADMUX_NAND_DATA07_INDEX */
  IMX_PADCTL_SD4_DATA0_INDEX,       /* IMX_PADMUX_SD4_DATA0_INDEX */
  IMX_PADCTL_SD4_DATA1_INDEX,       /* IMX_PADMUX_SD4_DATA1_INDEX */
  IMX_PADCTL_SD4_DATA2_INDEX,       /* IMX_PADMUX_SD4_DATA2_INDEX */
  IMX_PADCTL_SD4_DATA3_INDEX,       /* IMX_PADMUX_SD4_DATA3_INDEX */
  IMX_PADCTL_SD4_DATA4_INDEX,       /* IMX_PADMUX_SD4_DATA4_INDEX */
  IMX_PADCTL_SD4_DATA5_INDEX,       /* IMX_PADMUX_SD4_DATA5_INDEX */
  IMX_PADCTL_SD4_DATA6_INDEX,       /* IMX_PADMUX_SD4_DATA6_INDEX */
  IMX_PADCTL_SD4_DATA7_INDEX,       /* IMX_PADMUX_SD4_DATA7_INDEX */
  IMX_PADCTL_SD1_DATA1_INDEX,       /* IMX_PADMUX_SD1_DATA1_INDEX */
  IMX_PADCTL_SD1_DATA0_INDEX,       /* IMX_PADMUX_SD1_DATA0_INDEX */
  IMX_PADCTL_SD1_DATA3_INDEX,       /* IMX_PADMUX_SD1_DATA3_INDEX */
  IMX_PADCTL_SD1_CMD_INDEX,         /* IMX_PADMUX_SD1_CMD_INDEX */
  IMX_PADCTL_SD1_DATA2_INDEX,       /* IMX_PADMUX_SD1_DATA2_INDEX */
  IMX_PADCTL_SD1_CLK_INDEX,         /* IMX_PADMUX_SD1_CLK_INDEX */
  IMX_PADCTL_SD2_CLK_INDEX,         /* IMX_PADMUX_SD2_CLK_INDEX */
  IMX_PADCTL_SD2_CMD_INDEX,         /* IMX_PADMUX_SD2_CMD_INDEX */
  IMX_PADCTL_SD2_DATA3_INDEX,       /* IMX_PADMUX_SD2_DATA3_INDEX */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_padmux_map
 *
 * Description:
 *   This function map a Pad Mux register index to the corresponding Pad
 *   Control register index.
 *
 ****************************************************************************/

unsigned int imx_padmux_map(unsigned int padmux)
{
  DEBUGASSERT(padmux < IMX_PADMUX_NREGISTERS);
  return (unsigned int)g_mux2ctl_map[padmux];
}

/****************************************************************************
 * Name: imx_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int imx_iomux_configure(uintptr_t padctl, iomux_pinset_t ioset)
{
  uint32_t regval = 0;
  uint32_t value;

  /* Select CMOS input or Schmitt Trigger input */

  if ((ioset & IOMUX_SCHMITT_TRIGGER) != 0)
    {
      regval |= PADCTL_SRE;
    }

  /* Select drive strength */

  value = (ioset & IOMUX_DRIVE_MASK) >> IOMUX_DRIVE_SHIFT;
  regval |= PADCTL_DSE(value);

  /* Select spped */

  value = (ioset & IOMUX_SPEED_MASK) >> IOMUX_SPEED_SHIFT;
  regval |= PADCTL_SPEED(value);

  /* Select CMOS output or Open Drain outpout */

  if ((ioset & IOMUX_OPENDRAIN) != 0)
    {
      regval |= PADCTL_ODE;
    }

  /* Handle pull/keep selection */

  switch (ioset & _IOMUX_PULLTYPE_MASK)
    {
      default:
      case _IOMUX_PULL_NONE:
        break;

      case _IOMUX_PULL_KEEP:
        {
          regval |= PADCTL_PKE;
        }
        break;

      case _IOMUX_PULL_ENABLE:
        {
          regval |= (PADCTL_PKE | PADCTL_PUE);

          value   = (ioset & _IOMUX_PULLDESC_MASK) >> _IOMUX_PULLDESC_SHIFT;
          regval |= PADCTL_PUS(value);
        }
        break;
    }

  /* Select slow/fast slew rate */

  if ((ioset & IOMUX_SLEW_FAST) != 0)
    {
      regval |= PADCTL_HYS;
    }

  /* Write the result to the specified Pad Control register */

  putreg32(regval, padctl);
  return OK;
}
