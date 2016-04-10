/************************************************************************************
 * arch/arm/src/imx6/imx_ccm.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_CCM_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_CCM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <chip/imx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CCM Register Offsets *************************************************************/

#define IMX_CCM_CCR_OFFSET         0x0000  /* CCM Control Register */
#define IMX_CCM_CCDR_OFFSET        0x0004  /* CCM Control Divider Register */
#define IMX_CCM_CSR_OFFSET         0x0008  /* CCM Status Register */
#define IMX_CCM_CCSR_OFFSET        0x000c  /* CCM Clock Switcher Register */
#define IMX_CCM_CACRR_OFFSET       0x0010  /* CCM Arm Clock Root Register */
#define IMX_CCM_CBCDR_OFFSET       0x0014  /* CCM Bus Clock Divider Register */
#define IMX_CCM_CBCMR_OFFSET       0x0018  /* CCM Bus Clock Multiplexer Register */
#define IMX_CCM_CSCMR1_OFFSET      0x001c  /* CCM Serial Clock Multiplexer Register 1 */
#define IMX_CCM_CSCMR2_OFFSET      0x0020  /* CCM Serial Clock Multiplexer Register 2 */
#define IMX_CCM_CSCDR1_OFFSET      0x0024  /* CCM Serial Clock Divider Register 1 */
#define IMX_CCM_CS1CDR_OFFSET      0x0028  /* CCM SSI1 Clock Divider Register */
#define IMX_CCM_CS2CDR_OFFSET      0x002c  /* CCM SSI2 Clock Divider Register */
#define IMX_CCM_CDCDR_OFFSET       0x0030  /* CCM D1 Clock Divider Register */
#define IMX_CCM_CHSCCDR_OFFSET     0x0034  /* CCM HSC Clock Divider Register */
#define IMX_CCM_CSCDR2_OFFSET      0x0038  /* CCM Serial Clock Divider Register 2 */
#define IMX_CCM_CSCDR3_OFFSET      0x003c  /* CCM Serial Clock Divider Register 3 */
#define IMX_CCM_CWDR_OFFSET        0x0044  /* CCM Wakeup Detector Register */
#define IMX_CCM_CDHIPR_OFFSET      0x0048  /* CCM Divider Handshake In-Process Register */
#define IMX_CCM_CLPCR_OFFSET       0x0054  /* CCM Low Power Control Register */
#define IMX_CCM_CISR_OFFSET        0x0058  /* CCM Interrupt Status Register */
#define IMX_CCM_CIMR_OFFSET        0x005c  /* CCM Interrupt Mask Register */
#define IMX_CCM_CCOSR_OFFSET       0x0060  /* CCM Clock Output Source Register */
#define IMX_CCM_CGPR_OFFSET        0x0064  /* CCM General Purpose Register */
#define IMX_CCM_CCGR0_OFFSET       0x0068  /* CCM Clock Gating Register 0 */
#define IMX_CCM_CCGR1_OFFSET       0x006c  /* CCM Clock Gating Register 1 */
#define IMX_CCM_CCGR2_OFFSET       0x0070  /* CCM Clock Gating Register 2 */
#define IMX_CCM_CCGR3_OFFSET       0x0074  /* CCM Clock Gating Register 3 */
#define IMX_CCM_CCGR4_OFFSET       0x0078  /* CCM Clock Gating Register 4 */
#define IMX_CCM_CCGR5_OFFSET       0x007c  /* CCM Clock Gating Register 5 */
#define IMX_CCM_CCGR6_OFFSET       0x0080  /* CCM Clock Gating Register 6 */
#define IMX_CCM_CMEOR_OFFSET       0x0088  /* CCM Module Enable Overide Register */

/* CCM Register Addresses ***********************************************************/

#define IMX_CCM_CCR                (IMX_CCM_VBASE+IMX_CCM_CCR_OFFSET)
#define IMX_CCM_CCDR               (IMX_CCM_VBASE+IMX_CCM_CCDR_OFFSET)
#define IMX_CCM_CSR                (IMX_CCM_VBASE+IMX_CCM_CSR_OFFSET)
#define IMX_CCM_CCSR               (IMX_CCM_VBASE+IMX_CCM_CCSR_OFFSET)
#define IMX_CCM_CACRR              (IMX_CCM_VBASE+IMX_CCM_CACRR_OFFSET)
#define IMX_CCM_CBCDR              (IMX_CCM_VBASE+IMX_CCM_CBCDR_OFFSET)
#define IMX_CCM_CBCMR              (IMX_CCM_VBASE+IMX_CCM_CBCMR_OFFSET)
#define IMX_CCM_CSCMR1             (IMX_CCM_VBASE+IMX_CCM_CSCMR1_OFFSET)
#define IMX_CCM_CSCMR2             (IMX_CCM_VBASE+IMX_CCM_CSCMR2_OFFSET)
#define IMX_CCM_CSCDR1             (IMX_CCM_VBASE+IMX_CCM_CSCDR1_OFFSET)
#define IMX_CCM_CS1CDR             (IMX_CCM_VBASE+IMX_CCM_CS1CDR_OFFSET)
#define IMX_CCM_CS2CDR             (IMX_CCM_VBASE+IMX_CCM_CS2CDR_OFFSET)
#define IMX_CCM_CDCDR              (IMX_CCM_VBASE+IMX_CCM_CDCDR_OFFSET)
#define IMX_CCM_CHSCCDR            (IMX_CCM_VBASE+IMX_CCM_CHSCCDR_OFFSET)
#define IMX_CCM_CSCDR2             (IMX_CCM_VBASE+IMX_CCM_CSCDR2_OFFSET)
#define IMX_CCM_CSCDR3             (IMX_CCM_VBASE+IMX_CCM_CSCDR3_OFFSET)
#define IMX_CCM_CWDR               (IMX_CCM_VBASE+IMX_CCM_CWDR_OFFSET)
#define IMX_CCM_CDHIPR             (IMX_CCM_VBASE+IMX_CCM_CDHIPR_OFFSET)
#define IMX_CCM_CLPCR              (IMX_CCM_VBASE+IMX_CCM_CLPCR_OFFSET)
#define IMX_CCM_CISR               (IMX_CCM_VBASE+IMX_CCM_CISR_OFFSET)
#define IMX_CCM_CIMR               (IMX_CCM_VBASE+IMX_CCM_CIMR_OFFSET)
#define IMX_CCM_CCOSR              (IMX_CCM_VBASE+IMX_CCM_CCOSR_OFFSET)
#define IMX_CCM_CGPR               (IMX_CCM_VBASE+IMX_CCM_CGPR_OFFSET)
#define IMX_CCM_CCGR0              (IMX_CCM_VBASE+IMX_CCM_CCGR0_OFFSET)
#define IMX_CCM_CCGR1              (IMX_CCM_VBASE+IMX_CCM_CCGR1_OFFSET)
#define IMX_CCM_CCGR2              (IMX_CCM_VBASE+IMX_CCM_CCGR2_OFFSET)
#define IMX_CCM_CCGR3              (IMX_CCM_VBASE+IMX_CCM_CCGR3_OFFSET)
#define IMX_CCM_CCGR4              (IMX_CCM_VBASE+IMX_CCM_CCGR4_OFFSET)
#define IMX_CCM_CCGR5              (IMX_CCM_VBASE+IMX_CCM_CCGR5_OFFSET)
#define IMX_CCM_CCGR6              (IMX_CCM_VBASE+IMX_CCM_CCGR6_OFFSET)
#define IMX_CCM_CMEOR              (IMX_CCM_VBASE+IMX_CCM_CMEOR_OFFSET)

/* CCM Register Bit Definitions *****************************************************/

/* CCM Control Register */

#define CCM_CCR_OSCNT_SHIFT            (0)       /* Bits 0-7: Oscillator ready counter value */
#define CCM_CCR_OSCNT_MASK             (0xff << CCM_CCR_OSCNT_SHIFT)
#  define CCM_CCR_OSCNT(n)             ((uint32_t)(n) << CCM_CCR_OSCNT_SHIFT)
#define CCM_CCR_COSC_EN                (1 << 12) /* Bit 12: On chip oscillator enable bit */
#define CCM_CCR_WB_COUNT_SHIFT         (16)      /* Bits 16-18: Well Bias counter */
#define CCM_CCR_WB_COUNT_MASK          (7 << CCM_CCR_WB_COUNT_SHIFT)
#  define CCM_CCR_WB_COUNT(n)          ((uint32_t)(n) << CCM_CCR_WB_COUNT_SHIFT)
#define CCM_CCR_REG_BYPASS_COUNT_SHIFT (21)      /* Bits 21-26: Counter for analog_reg_bypass signal assertion */
#define CCM_CCR_REG_BYPASS_COUNT_MASK  (0x3f << CCM_CCR_REG_BYPASS_COUNT_SHIFT)
#  define CCM_CCR_REG_BYPASS_COUNT(n)  ((uint32_t)(n) << CCM_CCR_REG_BYPASS_COUNT_SHIFT)
#define CCM_CCR_RBC_EN                 (1 << 27) /* Bit 27: Enable for REG_BYPASS_COUNTER */

/* CCM Control Divider Register */

#define CCM_CCDR_MMDC_CH1_MASK         (1 << 16) /* Bit 16: Mask handshake with mmdc_ch1 */
#define CCM_CCDR_MMDC_CH0_MASK         (1 << 17) /* Bit 17: Mask handshake with mmdc_ch0 */

/* CCM Status Register */

#define CCM_CSR_REF_EN_B               (1 << 0)  /* Bit 0:  Value of CCM_REF_EN_B output */
#define CCM_CSR_COSC_READY             (1 << 5)  /* Bit 5:  Status of on-board oscillator */

/* CCM Clock Switcher Register */

#define CCM_CCSR_PLL3_SW_CLK_SEL       (1 << 0)  /* Bit 0: Selects source to generate pll3_sw_clk */
#define CCM_CCSR_PLL1_SW_CLK_SEL       (1 << 2)  /* Bit 2:  Selects source to generate pll1_sw_clk */
#define CCM_CCSR_STEP_SEL              (1 << 8)  /* Bit 8:  Step frequency when shifting ARM frequency */
#define CCM_CCSR_PLL2_PFD2_DIS         (1 << 9)  /* Bit 9:  Mask of PLL2 PFD2 auto-disable */
#define CCM_CCSR_PLL2_PFD0_DIS         (1 << 10) /* Bit 10: Mask of PLL2 PFD0 auto-disable */
#define CCM_CCSR_PLL2_PFD1_594M_DIS    (1 << 11) /* Bit 11: Mask of PLL2 PFD1 auto-disable */
#define CCM_CCSR_PLL3_PFD2_DIS         (1 << 12) /* Bit 12: Mask of PLL3 PFD2 auto-disable */
#define CCM_CCSR_PLL3_PFD3_DIS         (1 << 13) /* Bit 13: Mask of PLL3 PFD3 auto-disable */
#define CCM_CCSR_PLL3_PFD0_DIS         (1 << 14) /* Bit 14: Mask of PLL3 PFD0 auto-disable */
#define CCM_CCSR_PLL3_PFD1_DIS         (1 << 15) /* Bit 15: Mask of PLL3 PFD1 auto-disable */

/* CCM Arm Clock Root Register */

#define CCM_CACRR_ARM_PODF_SHIFT       (0)       /* Bits 0-2: Divider for ARM clock root */
#define CCM_CACRR_ARM_PODF_MASK        (7 << CCM_CACRR_ARM_PODF_SHIFT)
#  define CCM_CACRR_ARM_PODF(n)        ((uint32_t)(n) << CCM_CACRR_ARM_PODF_SHIFT)

/* CCM Bus Clock Divider Register */

#define CCM_CBCDR_PERIPH2_CLK2_PODF_SHIFT  (0)       /* Bits 0-2: Divider for periph2_clk2 podf */
#define CCM_CBCDR_PERIPH2_CLK2_PODF_MASK   (7 << CCM_CBCDR_PERIPH2_CLK2_PODF_SHIFT)
#  define CCM_CBCDR_PERIPH2_CLK2_PODF(n)   ((uint32_t)(n) << CCM_CBCDR_PERIPH2_CLK2_PODF_SHIFT)
#define CCM_CBCDR_MMDC_CH1_AXI_PODF_SHIFT  (3)       /* Bits 3-5: Divider for mmdc_ch1_axi podf */
#define CCM_CBCDR_MMDC_CH1_AXI_PODF_MASK   (7 << CCM_CBCDR_MMDC_CH1_AXI_PODF_SHIFT)
#  define CCM_CBCDR_MMDC_CH1_AXI_PODF(n)   ((uint32_t)(n) << CCM_CBCDR_MMDC_CH1_AXI_PODF_SHIFT)
#define CCM_CBCDR_AXI_SEL                  (1 << 6)  /* Bit 6:  AXI clock source select */
#define CCM_CBCDR_AXI_ALT_SEL              (1 << 7)  /* Bit 7:  AXI alternative clock select */
#define CCM_CBCDR_IPG_PODF_SHIFT           (8)       /* Bits 8-9: Divider for ipg podf */
#define CCM_CBCDR_IPG_PODF_MASK            (3 << CCM_CBCDR_IPG_PODF_SHIFT)
#  define CCM_CBCDR_IPG_PODF(n)            ((uint32_t)(n) << CCM_CBCDR_IPG_PODF_SHIFT)
#define CCM_CBCDR_AHB_PODF_SHIFT           (10)      /* Bits 10-12: Divider for AHB PODF */
#define CCM_CBCDR_AHB_PODF_MASK            (7 << CCM_CBCDR_AHB_PODF_SHIFT)
#  define CCM_CBCDR_AHB_PODF(n)            ((uint32_t)(n) << CCM_CBCDR_AHB_PODF_SHIFT)
#define CCM_CBCDR_AXI_PODF_SHIFT           (16)      /* Bits 16-18: Divider for AXI PODF */
#define CCM_CBCDR_AXI_PODF_MASK            (7 << CCM_CBCDR_AXI_PODF_SHIFT)
#  define CCM_CBCDR_AXI_PODF(n)            ((uint32_t)(n) << CCM_CBCDR_AXI_PODF_SHIFT)
#define CCM_CBCDR_MMDC_CH0_AXI_PODF_SHIFT  (19)      /* Bits 19-21: Divider for mmdc_ch0_axi podf */
#define CCM_CBCDR_MMDC_CH0_AXI_PODF_MASK   (7 << CCM_CBCDR_MMDC_CH0_AXI_PODF_SHIFT)
#  define CCM_CBCDR_MMDC_CH0_AXI_PODF(n)   ((uint32_t)(n) << CCM_CBCDR_MMDC_CH0_AXI_PODF_SHIFT)
#define CCM_CBCDR_PERIPH_CLK_SEL           (1 << 25) /* Bit 25: Selector for peripheral main clock */
#define CCM_CBCDR_PERIPH2_CLK_SEL          (1 << 26) /* Bit 26: Selector for peripheral2 main clock */
#define CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT   (27)      /* Bits 27-29: Divider for periph2 clock podf */
#define CCM_CBCDR_PERIPH_CLK2_PODF_MASK    (7 << CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT)
#  define CCM_CBCDR_PERIPH_CLK2_PODF(n)    ((uint32_t)(n) << CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT)

/* CCM Bus Clock Multiplexer Register */

#define CCM_CBCMR_GPU2D_AXI_CLK_SEL                   (1 << 0)  /* Bit 0:  Selector for gpu2d_axi clock multiplexer */
#define CCM_CBCMR_GPU3D_AXI_CLK_SEL                   (1 << 1)  /* Bit 1:   Selector for gpu3d_axi clock multiplexer */
#define CCM_CBCMR_GPU3D_CORE_CLK_SEL_SHIFT            (4)       /* Bits 4-5: Selector for gpu3d_core clock multiplexer */
#define CCM_CBCMR_GPU3D_CORE_CLK_SEL_MASK             (3 << CCM_CBCMR_GPU3D_CORE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_GPU3D_CORE_CLK_SEL_MMDC_CH0       (0 << CCM_CBCMR_GPU3D_CORE_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 */
#  define CCM_CBCMR_GPU3D_CORE_CLK_SEL_PLL3_SWCLK     (1 << CCM_CBCMR_GPU3D_CORE_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CBCMR_GPU3D_CORE_CLK_SEL_PLL2_PFD1      (2 << CCM_CBCMR_GPU3D_CORE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD1 */
#  define CCM_CBCMR_GPU3D_CORE_CLK_SEL_PLL2_PFD2      (3 << CCM_CBCMR_GPU3D_CORE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#define CCM_CBCMR_GPU3D_SHADER_CLK_SEL_SHIFT          (8)       /* Bits 8-9: Selector for gpu3d_shader clock multiplexer */
#define CCM_CBCMR_GPU3D_SHADER_CLK_SEL_MASK           (3 << CCM_CBCMR_GPU3D_SHADER_CLK_SEL_SHIFT)
#  define CCM_CBCMR_GPU3D_SHADER_CLK_SEL_MMDC_CH0     (0 << CCM_CBCMR_GPU3D_SHADER_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clk */
#  define CCM_CBCMR_GPU3D_SHADER_CLK_SEL_PLL3_SWCLK   (1 << CCM_CBCMR_GPU3D_SHADER_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CBCMR_GPU3D_SHADER_CLK_SEL_PLL2_PFD1    (2 << CCM_CBCMR_GPU3D_SHADER_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD1 */
#  define CCM_CBCMR_GPU3D_SHADER_CLK_SEL_PLL3_PFD0    (3 << CCM_CBCMR_GPU3D_SHADER_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD0 */
#define CCM_CBCMR_PCIE_AXI_CLK_SEL                    (1 << 10) /* Bit 10: Selector for pcie_axi clock multiplexer */
#define CCM_CBCMR_VDOAXI_CLK_SEL                      (1 << 11) /* Bit 11: Selector for vdoaxi clock multiplexer */
#define CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT               (12)      /* Bits 12-13: Selector for peripheral clk2 clock multiplexer */
#define CCM_CBCMR_PERIPH_CLK2_SEL_MASK                (3 << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)
#  define CCM_CBCMR_PERIPH_CLK2_SEL_PLL3_SWCLK        (0 << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CBCMR_PERIPH_CLK2_SEL_OSC_CLK           (1 << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT) /* Derive clock from osc_clk (pll1_ref_clk) */
#  define CCM_CBCMR_PERIPH_CLK2_SEL_PLL2_BYPASS       (2 << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT) /* Derive clock from pll2_bypass_clk */
#define CCM_CBCMR_VPU_AXI_CLK_SEL_SHIFT               (14)      /* Bits 14-15: Selector for VPU axi clock multiplexer */
#define CCM_CBCMR_VPU_AXI_CLK_SEL_MASK                (3 << CCM_CBCMR_VPU_AXI_CLK_SEL_SHIFT)
#  define CCM_CBCMR_VPU_AXI_CLK_SEL_AXI               (0 << CCM_CBCMR_VPU_AXI_CLK_SEL_SHIFT) /* Derive clock from AXI */
#  define CCM_CBCMR_VPU_AXI_CLK_SEL_PLL2_PFD2         (1 << CCM_CBCMR_VPU_AXI_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CBCMR_VPU_AXI_CLK_SEL_PLL2_PFD0         (2 << CCM_CBCMR_VPU_AXI_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#define CCM_CBCMR_GPU2D_CORE_CLK_SEL_SHIFT            (16)      /* Bits 16-17: Selector for open vg (GPU2D Core) clock multiplexer */
#define CCM_CBCMR_GPU2D_CORE_CLK_SEL_MASK             (3 << CCM_CBCMR_GPU2D_CORE_CLK_SEL_SHIFT)
#  define CCM_CBCMR_GPU2D_CORE_CLK_SEL_AXI            (0 << CCM_CBCMR_GPU2D_CORE_CLK_SEL_SHIFT) /* Derive clock from AXI */
#  define CCM_CBCMR_GPU2D_CORE_CLK_SEL_PLL3_SWCLK     (1 << CCM_CBCMR_GPU2D_CORE_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CBCMR_GPU2D_CORE_CLK_SEL_PLL2_PFD0      (2 << CCM_CBCMR_GPU2D_CORE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CBCMR_GPU2D_CORE_CLK_SEL_PLL2_PFD2      (3 << CCM_CBCMR_GPU2D_CORE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT            (18)      /* Bits 18-19: Selector for pre_periph clock multiplexer */
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK             (3 << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT)
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL2           (0 << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT) /* Derive clock from PLL2 */
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL2_PFD2      (1 << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL2_PFD0      (2 << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CBCMR_PRE_PERIPH_CLK_SEL_DIV_PLL2_PFD2  (3 << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT) /* Derive clock from divided (/2) PLL2 PFD2 */
#define CCM_CBCMR_PERIPH2_CLK2_SEL                    (1 << 20) /* Bit 20: Selector for periph2_clk2 clock multiplexer */
#define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_SHIFT           (21)      /* Bits 21-22: Selector for pre_periph2 clock multiplexer */
#define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_MASK            (3 << CCM_CBCMR_PRE_PERIPH2_CLK_SEL_SHIFT)
#  define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_PLL2          (0 << CCM_CBCMR_PRE_PERIPH2_CLK_SEL_SHIFT) /* Derive clock from PLL2 */
#  define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_PLL2_PFD2     (1 << CCM_CBCMR_PRE_PERIPH2_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_PLL2_PFD0     (2 << CCM_CBCMR_PRE_PERIPH2_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_DIV_PLL2_PFD2 (3 << CCM_CBCMR_PRE_PERIPH2_CLK_SEL_SHIFT) /* Derive clock from divided (/2) PLL2 PFD2 */
#define CCM_CBCMR_GPU2D_CORE_CLK_PODF_SHIFT           (23)      /* Bits 23-25: Divider for gpu2d_core clock */
#define CCM_CBCMR_GPU2D_CORE_CLK_PODF_MASK            (7 << CCM_CBCMR_GPU2D_CORE_CLK_PODF_SHIFT)
#  define CCM_CBCMR_GPU2D_CORE_CLK_PODF(n)            ((uint32_t)(n) << CCM_CBCMR_GPU2D_CORE_CLK_PODF_SHIFT)
#define CCM_CBCMR_GPU3D_CORE_PODF_SHIFT               (26)      /* Bits 26-28: Divider for gpu3d_core clock */
#define CCM_CBCMR_GPU3D_CORE_PODF_MASK                (7 << CCM_CBCMR_GPU3D_CORE_PODF_SHIFT)
#  define CCM_CBCMR_GPU3D_CORE_PODF(n)                ((uint32_t)(n) << CCM_CBCMR_GPU3D_CORE_PODF_SHIFT)
#define CCM_CBCMR_GPU3D_SHADER_PODF_SHIFT             (29)      /* Bits 29-31: Divider for gpu3d_shader clock */
#define CCM_CBCMR_GPU3D_SHADER_PODF_MASK              (7 << CCM_CBCMR_GPU3D_SHADER_PODF_SHIFT)
#  define CCM_CBCMR_GPU3D_SHADER_PODF(n)              ((uint32_t)(n) << CCM_CBCMR_GPU3D_SHADER_PODF_SHIFT)

/* CCM Serial Clock Multiplexer Register 1 */

#define CCM_CSCMR1_PERCLK_PODF_SHIFT         (0)       /* Bits 0-5: Divider for perclk podf */
#define CCM_CSCMR1_PERCLK_PODF_MASK          (0x3f << CCM_CSCMR1_PERCLK_PODF_SHIFT)
#  define CCM_CSCMR1_PERCLK_PODF(n)          ((uint32_t)(n) << CCM_CSCMR1_PERCLK_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCMR1_SSI1_CLK_SEL_SHIFT        (10)       /* Bits 10-11: Selector for ssi1 clock multiplexer */
#define CCM_CSCMR1_SSI1_CLK_SEL_MASK         (3 << CCM_CSCMR1_SSI1_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SSI1_CLK_SEL_PLL3_PFD2  (0 << CCM_CSCMR1_SSI1_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD2 */
#  define CCM_CSCMR1_SSI1_CLK_SEL_PLL3_PFD3  (1 << CCM_CSCMR1_SSI1_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD3 */
#  define CCM_CSCMR1_SSI1_CLK_SEL_PLL4       (2 << CCM_CSCMR1_SSI1_CLK_SEL_SHIFT) /* Derive clock from PLL4 */
#define CCM_CSCMR1_SSI2_CLK_SEL_SHIFT        (12)       /* Bits 12-13: Selector for ssi2 clock multiplexer */
#define CCM_CSCMR1_SSI2_CLK_SEL_MASK         (3 << CCM_CSCMR1_SSI2_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SSI2_CLK_SEL_PLL3_PFD2  (0 << CCM_CSCMR1_SSI2_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD2 */
#  define CCM_CSCMR1_SSI2_CLK_SEL_PLL3_PFD3  (1 << CCM_CSCMR1_SSI2_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD3 */
#  define CCM_CSCMR1_SSI2_CLK_SEL_PLL4       (2 << CCM_CSCMR1_SSI2_CLK_SEL_SHIFT) /* Derive clock from PLL4 */
#define CCM_CSCMR1_SSI3_CLK_SEL_SHIFT        (14)       /* Bits 14-15: Selector for ssi3 clock multiplexer */
#define CCM_CSCMR1_SSI3_CLK_SEL_MASK         (3 << CCM_CSCMR1_SSI3_CLK_SEL_SHIFT)
#  define CCM_CSCMR1_SSI3_CLK_SEL_PLL3_PFD2  (0 << CCM_CSCMR1_SSI3_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD2 */
#  define CCM_CSCMR1_SSI3_CLK_SEL_PLL3_PFD3  (1 << CCM_CSCMR1_SSI3_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD3 */
#  define CCM_CSCMR1_SSI3_CLK_SEL_PLL4       (2 << CCM_CSCMR1_SSI3_CLK_SEL_SHIFT) /* Derive clock from PLL4 */
#define CCM_CSCMR1_USDHC1_CLK_SEL            (1 << 16)  /* Bit 16:  Selector for usdhc1 clock multiplexer */
#define CCM_CSCMR1_USDHC2_CLK_SEL            (1 << 17)  /* Bit 17:  Selector for usdhc2 clock multiplexer */
#define CCM_CSCMR1_USDHC3_CLK_SEL            (1 << 18)  /* Bit 18:  Selector for usdhc3 clock multiplexer */
#define CCM_CSCMR1_USDHC4_CLK_SEL            (1 << 19)  /* Bit 19:  Selector for usdhc4 clock multiplexer */
#define CCM_CSCMR1_ACLK_PODF_SHIFT           (20)       /* Bits 20-22: Divider for aclk clock root */
#define CCM_CSCMR1_ACLK_PODF_MASK            (7 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV1         (6 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV1         (0 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV2         (7 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV2         (1 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV3         (4 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV3         (2 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV4         (5 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV4         (3 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV5         (2 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV5         (4 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV6         (3 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV6         (5 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV7         (0 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV7         (6 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_WDIV8         (1 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_PODF_RDIV8         (7 << CCM_CSCMR1_ACLK_PODF_SHIFT)
#define CCM_CSCMR1_ACLK_EIM_SLOW_PODF_SHIFT  (23)       /* Bits 23-25: Divider for aclk_eim_slow clock root */
#define CCM_CSCMR1_ACLK_EIM_SLOW_PODF_MASK   (7 << CCM_CSCMR1_ACLK_EIM_SLOW_PODF_SHIFT)
#  define CCM_CSCMR1_ACLK_EIM_SLOW_PODF(n)   ((uint32_t)(n) << CCM_CSCMR1_ACLK_EIM_SLOW_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCMR1_ACLK_SEL_SHIFT            (27)       /* Bits 27-28: Selector for aclk root clock multiplexer */
#define CCM_CSCMR1_ACLK_SEL_MASK             (3 << CCM_CSCMR1_ACLK_SEL_SHIFT)
#  define CCM_CSCMR1_ACLK_SEL_PLL2_PFD2      (0 << CCM_CSCMR1_ACLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CSCMR1_ACLK_SEL_PLL3_SWCLK     (1 << CCM_CSCMR1_ACLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CSCMR1_ACLK_SEL_AXI            (2 << CCM_CSCMR1_ACLK_SEL_SHIFT) /* Derive clock from AXI */
#  define CCM_CSCMR1_ACLK_SEL_PLL2_PFD0      (3 << CCM_CSCMR1_ACLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#define CCM_CSCMR1_ACLK_EIM_SLOW_SEL_SHIFT   (29)       /* Bits 29-30: Selector for aclk_eim_slow root clock multiplexer */
#define CCM_CSCMR1_ACLK_EIM_SLOW_SEL_MASK    (3 << CCM_CSCMR1_ACLK_EIM_SLOW_SEL_SHIFT)
#  define CCM_CSCMR1_ACLK_EIM_SLOW_SEL_AXI        (0 << CCM_CSCMR1_ACLK_EIM_SLOW_SEL_SHIFT) /* Derive clock from AXI */
#  define CCM_CSCMR1_ACLK_EIM_SLOW_SEL_PLL3_SWCLK (1 << CCM_CSCMR1_ACLK_EIM_SLOW_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CSCMR1_ACLK_EIM_SLOW_SEL_PLL2_PFD   (2 << CCM_CSCMR1_ACLK_EIM_SLOW_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CSCMR1_ACLK_EIM_SLOW_SEL_PLL2_PFD0  (3 << CCM_CSCMR1_ACLK_EIM_SLOW_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */

/* CCM Serial Clock Multiplexer Register 2 */

#define CCM_CSCMR2_CAN_CLK_PODF_SHIFT        (2)       /* Bits 2-7: Divider for can clock podf */
#define CCM_CSCMR2_CAN_CLK_PODF_MASK         (0x3f << CCM_CSCMR2_CAN_CLK_PODF_SHIFT)
#  define CCM_CSCMR2_CAN_CLK_PODF(n)         ((uint32_t)(n) << CCM_CSCMR2_CAN_CLK_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCMR2_LDB_DI0_IPU_DIV           (1 << 10)  /* Bit 10:  Control for divider of ldb clock for IPU di0 */
#define CCM_CSCMR2_LDB_DI1_IPU_DIV           (1 << 11)  /* Bit 11:  Control for divider of ldb clock for IPU di1 */
#define CCM_CSCMR2_ESAI_CLK_SEL_SHIFT        (19)       /* Bits 19-20: Selector for esai clock multiplexer */
#define CCM_CSCMR2_ESAI_CLK_SEL_MASK         (3 << CCM_CSCMR2_ESAI_CLK_SEL_SHIFT)
#  define CCM_CSCMR2_ESAI_CLK_SEL_DIV_PLL4   (0 << CCM_CSCMR2_ESAI_CLK_SEL_SHIFT) /* Derive clock from PLL4 divided clock */
#  define CCM_CSCMR2_ESAI_CLK_SEL_PLL3_PFD2  (1 << CCM_CSCMR2_ESAI_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD2 clock */
#  define CCM_CSCMR2_ESAI_CLK_SEL_PLL3_PFD3  (2 << CCM_CSCMR2_ESAI_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD3 clock */
#  define CCM_CSCMR2_ESAI_CLK_SEL_PLL3_SWCLK (3 << CCM_CSCMR2_ESAI_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */

/* CCM Serial Clock Divider Register 1 */

#define CCM_CSCDR1_UART_CLK_PODF_SHIFT    (0)        /* Bits 0-5: Divider for uart clock podf */
#define CCM_CSCDR1_UART_CLK_PODF_MASK     (0x3f << CCM_CSCDR1_UART_CLK_PODF_SHIFT)
#  define CCM_CSCDR1_UART_CLK_PODF(n)     ((uint32_t)(n) << CCM_CSCDR1_UART_CLK_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR1_USDHC1_PODF_SHIFT      (11)       /* Bits 11-13: Divider for usdhc1 clock podf */
#define CCM_CSCDR1_USDHC1_PODF_MASK       (7 << CCM_CSCDR1_USDHC1_PODF_SHIFT)
#  define CCM_CSCDR1_USDHC1_PODF(n)       ((uint32_t)(n) << CCM_CSCDR1_USDHC1_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR1_USDHC2_PODF_SHIFT      (16)       /* Bits 16-18: Divider for usdhc2 clock */
#define CCM_CSCDR1_USDHC2_PODF_MASK       (7 << CCM_CSCDR1_USDHC2_PODF_SHIFT)
#  define CCM_CSCDR1_USDHC2_PODF(n)       ((uint32_t)(n) << CCM_CSCDR1_USDHC2_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR1_USDHC3_PODF_SHIFT      (19)       /* Bits 19-21: Divider for usdhc3 clock podf */
#define CCM_CSCDR1_USDHC3_PODF_MASK       (7 << CCM_CSCDR1_USDHC3_PODF_SHIFT)
#  define CCM_CSCDR1_USDHC3_PODF(n)       ((uint32_t)(n) << CCM_CSCDR1_USDHC3_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR1_USDHC4_PODF_SHIFT      (22)       /* Bits 22-24: Divider for usdhc4 clock pred */
#define CCM_CSCDR1_USDHC4_PODF_MASK       (7 << CCM_CSCDR1_USDHC4_PODF_SHIFT)
#  define CCM_CSCDR1_USDHC4_PODF(n)       ((uint32_t)(n) << CCM_CSCDR1_USDHC4_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR1_VPU_AXI_PODF_SHIFT     (25)       /* Bits 25-27: Divider for vpu axi clock podf */
#define CCM_CSCDR1_VPU_AXI_PODF_MASK      (7 << CCM_CSCDR1_VPU_AXI_PODF_SHIFT)
#  define CCM_CSCDR1_VPU_AXI_PODF(n)      ((uint32_t)(n) << CCM_CSCDR1_VPU_AXI_PODF_SHIFT) /* n=(divisor-1) */

/* CCM SSI1 Clock Divider Register */

#define CCM_CS1CDR_SSI1_CLK_PODF_SHIFT    (0)        /* Bits 0-5: Divider for ssi1 clock podf */
#define CCM_CS1CDR_SSI1_CLK_PODF_MASK     (0x3f << CCM_CS1CDR_SSI1_CLK_PODF_SHIFT)
#  define CCM_CS1CDR_SSI1_CLK_PODF(n)     ((uint32_t)(n) << CCM_CS1CDR_SSI1_CLK_PODF_SHIFT)
#define CCM_CS1CDR_SSI1_CLK_PRED_SHIFT    (6)        /* Bits 6-8: Divider for ssi1 clock pred */
#define CCM_CS1CDR_SSI1_CLK_PRED_MASK     (7 << CCM_CS1CDR_SSI1_CLK_PRED_SHIFT)
#  define CCM_CS1CDR_SSI1_CLK_PRED(n)     ((uint32_t)(n) << CCM_CS1CDR_SSI1_CLK_PRED_SHIFT) /* n=(divisor-1) */
#define CCM_CS1CDR_ESAI_CLK_PRED_SHIFT    (9)        /* Bits 9-11: Divider for esai clock pred */
#define CCM_CS1CDR_ESAI_CLK_PRED_MASK     (7 << CCM_CS1CDR_ESAI_CLK_PRED_SHIFT)
#  define CCM_CS1CDR_ESAI_CLK_PRED(n)     ((uint32_t)(n) << CCM_CS1CDR_ESAI_CLK_PRED_SHIFT) /* n=(divisor-1) */
#define CCM_CS1CDR_SSI3_CLK_PODF_SHIFT    (16)       /* Bits 16-21: Divider for ssi3 clock podf */
#define CCM_CS1CDR_SSI3_CLK_PODF_MASK     (0x3f << CCM_CS1CDR_SSI3_CLK_PODF_SHIFT)
#  define CCM_CS1CDR_SSI3_CLK_PODF(n)     ((uint32_t)(n) << CCM_CS1CDR_SSI3_CLK_PODF_SHIFT)
#define CCM_CS1CDR_SSI3_CLK_PRED_SHIFT    (22)       /* Bits 22-24: Divider for ssi3 clock pred */
#define CCM_CS1CDR_SSI3_CLK_PRED_MASK     (7 << CCM_CS1CDR_SSI3_CLK_PRED_SHIFT)
#  define CCM_CS1CDR_SSI3_CLK_PRED(n)     ((uint32_t)(n) << CCM_CS1CDR_SSI3_CLK_PRED_SHIFT) /* n=(divisor-1) */
#define CCM_CS1CDR_ESAI_CLK_PODF_SHIFT    (25)       /* Bits 25-27: Divider for esai clock podf */
#define CCM_CS1CDR_ESAI_CLK_PODF_MASK     (7 << CCM_CS1CDR_ESAI_CLK_PODF_SHIFT)
#  define CCM_CS1CDR_ESAI_CLK_PODF(n)     ((uint32_t)(n) << CCM_CS1CDR_ESAI_CLK_PODF_SHIFT) /* n=(divisor-1) */

/* CCM SSI2 Clock Divider Register */

#define CCM_CS2CDR_SSI2_CLK_PODF_SHIFT          (0)        /* Bits 0-5: Divider for ssi2 clock podf */
#define CCM_CS2CDR_SSI2_CLK_PODF_MASK           (0x3f << CCM_CS2CDR_SSI2_CLK_PODF_SHIFT)
#  define CCM_CS2CDR_SSI2_CLK_PODF(n)           ((uint32_t)(n) << CCM_CS2CDR_SSI2_CLK_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CS2CDR_SSI2_CLK_PRED_SHIFT          (6)        /* Bits 6-8: Divider for ssi2 clock pred */
#define CCM_CS2CDR_SSI2_CLK_PRED_MASK           (7 << CCM_CS2CDR_SSI2_CLK_PRED_SHIFT)
#  define CCM_CS2CDR_SSI2_CLK_PRED(n)           ((uint32_t)(n) << CCM_CS2CDR_SSI2_CLK_PRED_SHIFT) /* n=(divisor-1) */
#define CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT        (9)        /* Bits 9-11: Selector for ldb_di1 clock multiplexer */
#define CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK         (7 << CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT)
#  define CCM_CS2CDR_LDB_DI0_CLK_SEL_PLL5       (0 << CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT) /* Derive from PLL5 clock */
#  define CCM_CS2CDR_LDB_DI0_CLK_SEL_PLL2_PFD0  (1 << CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CS2CDR_LDB_DI0_CLK_SEL_PLL2_PFD2  (2 << CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CS2CDR_LDB_DI0_CLK_SEL_MMDC_CH1   (3 << CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch1 clock */
#  define CCM_CS2CDR_LDB_DI0_CLK_SEL_PLL3_SWCLK (4 << CCM_CS2CDR_LDB_DI0_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#define CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT        (12)       /* Bits 12-14: Selector for ldb_di1 clock multiplexer */
#define CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK         (7 << CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT)
#  define CCM_CS2CDR_LDB_DI1_CLK_SEL_PLL5       (0 << CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT) /* Derive PLL5 clock */
#  define CCM_CS2CDR_LDB_DI1_CLK_SEL_PLL2_PFD0  (1 << CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CS2CDR_LDB_DI1_CLK_SEL_PLL2_PFD2  (2 << CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CS2CDR_LDB_DI1_CLK_SEL_MMDC_CH1   (3 << CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch1 clock */
#  define CCM_CS2CDR_LDB_DI1_CLK_SEL_PLL3_SWCLK (4 << CCM_CS2CDR_LDB_DI1_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#define CCM_CS2CDR_ENFC_CLK_SEL_SHIFT           (16)       /* Bits 16-17: Selector for enfc clock multiplexer */
#define CCM_CS2CDR_ENFC_CLK_SEL_MASK            (3 << CCM_CS2CDR_ENFC_CLK_SEL_SHIFT)
#  define CCM_CS2CDR_ENFC_CLK_SEL_PLL2_PFD0     (0 << CCM_CS2CDR_ENFC_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CS2CDR_ENFC_CLK_SEL_PLL2          (1 << CCM_CS2CDR_ENFC_CLK_SEL_SHIFT) /* Derive clock from PLL2 */
#  define CCM_CS2CDR_ENFC_CLK_SEL_PLL3_SWCLK    (2 << CCM_CS2CDR_ENFC_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CS2CDR_ENFC_CLK_SEL_PLL2_PFD2     (3 << CCM_CS2CDR_ENFC_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#define CCM_CS2CDR_ENFC_CLK_PRED_SHIFT          (18)       /* Bits 18-20: Divider for enfc clock pred divider */
#define CCM_CS2CDR_ENFC_CLK_PRED_MASK           (7 << CCM_CS2CDR_ENFC_CLK_PRED_SHIFT)
#  define CCM_CS2CDR_ENFC_CLK_PRED(n)           ((uint32_t)(n) << CCM_CS2CDR_ENFC_CLK_PRED_SHIFT) /* n=(divisor-1) */
#define CCM_CS2CDR_ENFC_CLK_PODF_SHIFT          (21)       /* Bits 21-26: Divider for enfc clock divider */
#define CCM_CS2CDR_ENFC_CLK_PODF_MASK           (0x3f << CCM_CS2CDR_ENFC_CLK_PODF_SHIFT)
#  define CCM_CS2CDR_ENFC_CLK_PODF(n)           ((uint32_t)(n) << CCM_CS2CDR_ENFC_CLK_PODF_SHIFT)

/* CCM D1 Clock Divider Register */

#define CCM_CDCDR_SPDIF1_CLK_SEL_SHIFT        (7)       /* Bits 7-8: Selector for spdif1 clock multiplexer */
#define CCM_CDCDR_SPDIF1_CLK_SEL_MASK         (3 << CCM_CDCDR_SPDIF1_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF1_CLK_SEL_DIV_PLL4   (0 << CCM_CDCDR_SPDIF1_CLK_SEL_SHIFT) /* Derive clock from PLL4 divided clock */
#  define CCM_CDCDR_SPDIF1_CLK_SEL_PLL3_PFD2  (1 << CCM_CDCDR_SPDIF1_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD2 */
#  define CCM_CDCDR_SPDIF1_CLK_SEL_PLL3_PFD3  (2 << CCM_CDCDR_SPDIF1_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD3 */
#  define CCM_CDCDR_SPDIF1_CLK_SEL_PLL3_SWCLK (3 << CCM_CDCDR_SPDIF1_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#define CCM_CDCDR_SPDIF1_CLK_PODF_SHIFT       (9)        /* Bits 9-11: Divider for spdif1 clock podf */
#define CCM_CDCDR_SPDIF1_CLK_PODF_MASK        (7 << CCM_CDCDR_SPDIF1_CLK_PODF_SHIFT)
#  define CCM_CDCDR_SPDIF1_CLK_PODF(n)        ((uint32_t)(n) << CCM_CDCDR_SPDIF1_CLK_PODF_SHIFT) /* n=0,7 (divisor-1) */
#define CCM_CDCDR_SPDIF1_CLK_PRED_SHIFT       (12)       /* Bits 12-14: Divider for spdif1 clock pred */
#define CCM_CDCDR_SPDIF1_CLK_PRED_MASK        (7 << CCM_CDCDR_SPDIF1_CLK_PRED_SHIFT)
#  define CCM_CDCDR_SPDIF1_CLK_PRED(n)        ((uint32_t)(n) << CCM_CDCDR_SPDIF1_CLK_PRED_SHIFT) /* n=0,1,2,7 (divisor-1) */
#define CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT        (20)       /* Bits 20-21: Selector for spdif0 clock multiplexer */
#define CCM_CDCDR_SPDIF0_CLK_SEL_MASK         (3 << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_SEL_DIV_PLL4   (0 << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT) /* Derive clock from PLL4 divided clock */
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL3_PFD2  (1 << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD2 */
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL3_PFD3  (2 << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD3 */
#  define CCM_CDCDR_SPDIF0_CLK_SEL_PLL3_SWCLK (3 << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#define CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT       (22)       /* Bits 22-24: Divider for spdif0 clock podf */
#define CCM_CDCDR_SPDIF0_CLK_PODF_MASK        (7 << CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_PODF(n)        ((uint32_t)(n) << CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT) /* n=0,7 (divisor-1) */
#define CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT       (25)       /* Bits 25-27: Divider for spdif0 clock pred */
#define CCM_CDCDR_SPDIF0_CLK_PRED_MASK        (7 << CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT)
#  define CCM_CDCDR_SPDIF0_CLK_PRED(n)        ((uint32_t)(n) << CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT) /* n=0,1,2,7 (divisor-1) */
#define CCM_CDCDR_HSI_TX_CLK_SEL              (1 << 28)  /* Bit 28:  Selector for hsi_tx clock multiplexer */
#define CCM_CDCDR_HSI_TX_PODF_SHIFT           (29)       /* Bits 29-31: Divider for hsi_tx clock podf */
#define CCM_CDCDR_HSI_TX_PODF_MASK            (7 << CCM_CDCDR_HSI_TX_PODF_SHIFT)
#  define CCM_CDCDR_HSI_TX_PODF(n)            ((uint32_t)(n) << CCM_CDCDR_HSI_TX_PODF_SHIFT) /* n=(divisor-1) */

/* CCM HSC Clock Divider Register */

#define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT            (0)        /* Bits 0-2: Selector for ipu1 di0 root clock multiplexer */
#define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK             (7 << CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT)
#  define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_DIV_IPU1       (0 << CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT) /* Derive clock from divided pre-muxed ipu1 di0 clock */
#  define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_IPP_DI0_CLK    (1 << CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT) /* Derive clock from ipp_di0_clk */
#  define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_IPP_DI1_CLK    (2 << CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT) /* Derive clock from ipp_di1_clk */
#  define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_LDB_DI0_CLK    (3 << CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT) /* Derive clock from ldb_di0_clk */
#  define CCM_CHSCCDR_IPU1_DI0_CLK_SEL_LDB_DI1_CLK    (4 << CCM_CHSCCDR_IPU1_DI0_CLK_SEL_SHIFT) /* Derive clock from ldb_di1_clk */
#define CCM_CHSCCDR_IPU1_DI0_PODF_SHIFT               (3)       /* Bits 3-5: Divider for ipu1_di0 clock divider */
#define CCM_CHSCCDR_IPU1_DI0_PODF_MASK                (7 << CCM_CHSCCDR_IPU1_DI0_PODF_SHIFT)
#  define CCM_CHSCCDR_IPU1_DI0_PODF(n)                ((uint32_t)(n) << CCM_CHSCCDR_IPU1_DI0_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT        (6)        /* Bits 6-8: Selector for ipu1 di0 root clock pre-multiplexer */
#define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK         (7 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT)
#  define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MMDC_CH0   (0 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clock */
#  define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_PLL3_SWCLK (1 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_PLL5       (2 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from pll5 */
#  define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_PLL2_PFD0  (3 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_PLL2_PFD2  (4 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_PLL3_PFD1  (5 << CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD1 */
#define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT            (9)        /* Bits 9-11: Selector for ipu1 di1 root clock multiplexer */
#define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_MASK             (7 << CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT)
#  define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_DIV_IPU1       (0 << CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT) /* Derive clock from divided pre-muxed ipu1 di1 clock */
#  define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_IPP_DI0_CLK    (1 << CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT) /* Derive clock from ipp_di0_clk */
#  define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_IPP_DI1_CLK    (2 << CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT) /* Derive clock from ipp_di1_clk */
#  define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_LDB_DI0_CLK    (3 << CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT) /* Derive clock from ldb_di0_clk */
#  define CCM_CHSCCDR_IPU1_DI1_CLK_SEL_LDB_DI1_CLK    (4 << CCM_CHSCCDR_IPU1_DI1_CLK_SEL_SHIFT) /* Derive clock from ldb_di1_clk */
#define CCM_CHSCCDR_IPU1_DI1_PODF_SHIFT               (12)       /* Bits 12-14: Divider for ipu1_di clock divider */
#define CCM_CHSCCDR_IPU1_DI1_PODF_MASK                (7 << CCM_CHSCCDR_IPU1_DI1_PODF_SHIFT)
#  define CCM_CHSCCDR_IPU1_DI1_PODF(n)                ((uint32_t)(n) << CCM_CHSCCDR_IPU1_DI1_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT        (15)       /* Bits 15-17: Selector for ipu1 di1 root clock pre-multiplexer */
#define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_MASK         (7 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT)
#  define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_MMDC_CH0   (0 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clock */
#  define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_PLL3_SWCLK (1 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_PLL5       (2 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from pll5 */
#  define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_PLL2_PFD0  (3 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_PLL2_PFD2  (4 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_PLL3_PFD1  (5 << CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD1 */

/* CCM Serial Clock Divider Register 2 */

#define CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT            (0)       /* Bits 0-2: Selector for ipu2 di0 root clock multiplexer */
#define CCM_CSCDR2_IPU2_DI0_CLK_SEL_MASK             (7 << CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_IPU2_DI0_CLK_SEL_DIV_IPU1       (0 << CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT) /* Derive clock from divided pre-muxed ipu1 di0 clock */
#  define CCM_CSCDR2_IPU2_DI0_CLK_SEL_IPP_DI0_CLK    (1 << CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT) /* Derive clock from ipp_di0_clk */
#  define CCM_CSCDR2_IPU2_DI0_CLK_SEL_IPP_DI1_CLK    (2 << CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT) /* Derive clock from ipp_di1_clk */
#  define CCM_CSCDR2_IPU2_DI0_CLK_SEL_LDB_DI0_CLK    (3 << CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT) /* Derive clock from ldb_di0_clk */
#  define CCM_CSCDR2_IPU2_DI0_CLK_SEL_LDB_DI1_CLK    (4 << CCM_CSCDR2_IPU2_DI0_CLK_SEL_SHIFT) /* Derive clock from ldb_di1_clk */
#define CCM_CSCDR2_IPU2_DI0_PODF_SHIFT               (3)        /* Bits 3-5: Divider for ipu2_di0 clock divider */
#define CCM_CSCDR2_IPU2_DI0_PODF_MASK                (7 << CCM_CSCDR2_IPU2_DI0_PODF_SHIFT)
#  define CCM_CSCDR2_IPU2_DI0_PODF(n)                ((uint32_t)(n) << CCM_CSCDR2_IPU2_DI0_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT        (6)        /* Bits 6-8: Selector for ipu2 di0 root clock pre-multiplexer */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_MASK         (7 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT)
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_MMDC_CH0     (0 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clock */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_PLL3_SWCLK   (1 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_PLL5         (2 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from pll5 */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_PLL2_PFD     (3 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_PLL2_PFD2    (4 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#define CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_PLL3_PFD1    (5 << CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD1 */
#define CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT            (9)        /* Bits 9-11: Selector for ipu1 di2 root clock multiplexer */
#define CCM_CSCDR2_IPU2_DI1_CLK_SEL_MASK             (7 << CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_IPU2_DI1_CLK_SEL_DIV_IPU1       (0 << CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT) /* Derive clock from divided pre-muxed ipu1 di1 clock */
#  define CCM_CSCDR2_IPU2_DI1_CLK_SEL_IPP_DI0_CLK    (1 << CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT) /* Derive clock from ipp_di0_clk */
#  define CCM_CSCDR2_IPU2_DI1_CLK_SEL_IPP_DI1_CLK    (2 << CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT) /* Derive clock from ipp_di1_clk */
#  define CCM_CSCDR2_IPU2_DI1_CLK_SEL_LDB_DI0_CLK    (3 << CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT) /* Derive clock from ldb_di0_clk */
#  define CCM_CSCDR2_IPU2_DI1_CLK_SEL_LDB_DI1_CLK    (4 << CCM_CSCDR2_IPU2_DI1_CLK_SEL_SHIFT) /* Derive clock from ldb_di1_clk */
#define CCM_CSCDR2_IPU2_DI1_PODF_SHIFT               (12)       /* Bits 12-14: Divider for ipu2_di1 clock divider */
#define CCM_CSCDR2_IPU2_DI1_PODF_MASK                (7 << CCM_CSCDR2_IPU2_DI1_PODF_SHIFT)
#  define CCM_CSCDR2_IPU2_DI1_PODF(n)                ((uint32_t)(n) << CCM_CSCDR2_IPU2_DI1_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT        (15)       /* Bits 15-17: Selector for ipu2 di1 root clock pre-multiplexer */
#define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_MASK         (7 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT)
#  define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_MMDC_CH0   (0 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clock */
#  define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_PLL3_SWCLK (1 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from pll3_sw_clk */
#  define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_PLL5       (2 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL5 */
#  define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_PLL2_PFD0  (3 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD0 */
#  define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_PLL2_PFD2  (4 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_PLL3_PFD1  (5 << CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD1 */
#define CCM_CSCDR2_ECSPI_CLK_PODF_SHIFT              (19)       /* Bits 19-24: Divider for ecspi clock podf */
#define CCM_CSCDR2_ECSPI_CLK_PODF_MASK               (0x3f << CCM_CSCDR2_ECSPI_CLK_PODF_SHIFT)
#  define CCM_CSCDR2_ECSPI_CLK_PODF(n)               ((uint32_t)(n) << CCM_CSCDR2_ECSPI_CLK_PODF_SHIFT) /* n=(divisor-1) */

/* CCM Serial Clock Divider Register 3 */

#define CCM_CSCDR3_IPU1_HSP_CLK_SEL_SHIFT        (9)       /* Bits 9-10: Selector for ipu1_hsp clock multiplexer */
#define CCM_CSCDR3_IPU1_HSP_CLK_SEL_MASK         (3 << CCM_CSCDR3_IPU1_HSP_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_IPU1_HSP_CLK_SEL_MMDC_CH0   (0 << CCM_CSCDR3_IPU1_HSP_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clock */
#  define CCM_CSCDR3_IPU1_HSP_CLK_SEL_PLL2_PFD2  (1 << CCM_CSCDR3_IPU1_HSP_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CSCDR3_IPU1_HSP_CLK_SEL_PLL3_120M  (2 << CCM_CSCDR3_IPU1_HSP_CLK_SEL_SHIFT) /* Derive clock from pll3_120M */
#  define CCM_CSCDR3_IPU1_HSP_CLK_SEL_PLL3_PFD1  (3 << CCM_CSCDR3_IPU1_HSP_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD1 */
#define CCM_CSCDR3_IPU1_HSP_PODF_SHIFT           (11)       /* Bits 11-13: Divider for ipu1_hsp clock */
#define CCM_CSCDR3_IPU1_HSP_PODF_MASK            (7 << CCM_CSCDR3_IPU1_HSP_PODF_SHIFT)
#  define CCM_CSCDR3_IPU1_HSP_PODF(n)            ((uint32_t)(n) << CCM_CSCDR3_IPU1_HSP_PODF_SHIFT) /* n=(divisor-1) */
#define CCM_CSCDR3_IPU2_HSP_CLK_SEL_SHIFT        (14)       /* Bits 14-15: Selector for ipu2_hsp clock multiplexer */
#define CCM_CSCDR3_IPU2_HSP_CLK_SEL_MASK         (3 << CCM_CSCDR3_IPU2_HSP_CLK_SEL_SHIFT)
#  define CCM_CSCDR3_IPU2_HSP_CLK_SEL_MMDC_CH0   (0 << CCM_CSCDR3_IPU2_HSP_CLK_SEL_SHIFT) /* Derive clock from mmdc_ch0 clock */
#  define CCM_CSCDR3_IPU2_HSP_CLK_SEL_PLL2_PFD2  (1 << CCM_CSCDR3_IPU2_HSP_CLK_SEL_SHIFT) /* Derive clock from PLL2 PFD2 */
#  define CCM_CSCDR3_IPU2_HSP_CLK_SEL_PLL3_120M  (2 << CCM_CSCDR3_IPU2_HSP_CLK_SEL_SHIFT) /* Derive clock from pll3_120M */
#  define CCM_CSCDR3_IPU2_HSP_CLK_SEL_PLL3_PFD1  (3 << CCM_CSCDR3_IPU2_HSP_CLK_SEL_SHIFT) /* Derive clock from PLL3 PFD1 */
#define CCM_CSCDR3_IPU2_HSP_PODF_SHIFT           (16)       /* Bits 16-18: Divider for ipu2_hsp clock */
#define CCM_CSCDR3_IPU2_HSP_PODF_MASK            (7 << CCM_CSCDR3_IPU2_HSP_PODF_SHIFT)
#  define CCM_CSCDR3_IPU2_HSP_PODF(n)            ((uint32_t)(n) << CCM_CSCDR3_IPU2_HSP_PODF_SHIFT) /* n=(divisor-1) */

/* CCM Wakeup Detector Register -- Reserved, has no defined fields */

/* CCM Divider Handshake In-Process Register */

#define CCM_CDHIPR_AXI_PODF_BUSY         (1 << 0)  /* Bit 0:  Busy indicator for axi_podf */
#define CCM_CDHIPR_AHB_PODF_BUSY         (1 << 1)  /* Bit 1:  Busy indicator for ahb_podf */
#define CCM_CDHIPR_MMDC_CH1_PODF_BUSY    (1 << 2)  /* Bit 2:  Busy indicator for mmdc_ch1_axi_podf */
#define CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY  (1 << 3)  /* Bit 3:  Busy indicator for periph2_clk_sel mux control */
#define CCM_CDHIPR_MMDC_CH0_PODF_BUSY    (1 << 4)  /* Bit 4:  Busy indicator for mmdc_ch0_axi_podf */
#define CCM_CDHIPR_PERIPH_CLK_SEL_BUSY   (1 << 5)  /* Bit 5:  Busy indicator for periph_clk_sel mux control */
#define CCM_CDHIPR_ARM_PODF_BUSY         (1 << 16) /* Bit 16: Busy indicator for arm_podf */

/* CCM Low Power Control Register */

#define CCM_CLPCR_LPM_SHIFT              (0)       /* Bits 0-1: Low power mode on next dsm_request signal */
#define CCM_CLPCR_LPM_MASK               (3 << CCM_CLPCR_LPM_SHIFT)
#  define CCM_CLPCR_LPM_RUNMODE          (0 << CCM_CLPCR_LPM_SHIFT) /* Remain in run mode */
#  define CCM_CLPCR_LPM_WAITMODE         (1 << CCM_CLPCR_LPM_SHIFT) /* Transfer to wait mode */
#  define CCM_CLPCR_LPM_STOPMODE         (2 << CCM_CLPCR_LPM_SHIFT) /* Transfer to stop mode */
#define CCM_CLPCR_ARM_CLK_DIS_ON_LPM     (1 << 5)  /* Bit 5:  ARM clocks disabled on wait mode */
#define CCM_CLPCR_SBYOS                  (1 << 6)  /* Bit 6:  Standby clock oscillator bit */
#define CCM_CLPCR_DIS_REF_OSC            (1 << 7)  /* Bit 7:  Control closing of external reference oscillator clock */
#define CCM_CLPCR_VSTBY                  (1 << 8)  /* Bit 8:  Voltage standby request bit */
#define CCM_CLPCR_STBY_COUNT_SHIFT       (9)       /* Bits 9-10: Standby counter definition */
#define CCM_CLPCR_STBY_COUNT_MASK        (3 << CCM_CLPCR_STBY_COUNT_SHIFT)
#  define CCM_CLPCR_STBY_COUNT_1         (0 << CCM_CLPCR_STBY_COUNT_SHIFT) /* Wait (1*pmic_delay_scaler)+1 ckil clocks */
#  define CCM_CLPCR_STBY_COUNT_3         (1 << CCM_CLPCR_STBY_COUNT_SHIFT) /* Wait (3*pmic_delay_scaler)+1 ckil clocks */
#  define CCM_CLPCR_STBY_COUNT_7         (2 << CCM_CLPCR_STBY_COUNT_SHIFT) /* Wait (7*pmic_delay_scaler)+1 ckil clocks */
#  define CCM_CLPCR_STBY_COUNT_15        (3 << CCM_CLPCR_STBY_COUNT_SHIFT) /* Wait (15*pmic_delay_scaler)+1 ckil clocks */
#define CCM_CLPCR_COSC_PWRDOWN           (1 << 11) /* Bit 11: Control powering down of on chip oscillator */
#define CCM_CLPCR_WB_PER_AT_LPM          (1 << 16) /* Bit 16: Enable periphery charge pump for well biasing at low power mode */
#define CCM_CLPCR_BYPASS_MMDC_CH0_LPM_HS (1 << 19) /* Bit 19: Bypass handshake with mmdc_ch0 on next entrance to low power mode */
#define CCM_CLPCR_BYPASS_MMDC_CH1_LPM_HS (1 << 21) /* Bit 21: Bypass handshake with mmdc_ch1 on next entrance to low power mode */
#define CCM_CLPCR_MASK_CORE0_WFI         (1 << 22) /* Bit 22: Mask WFI of core0 for entering low power mode */
#define CCM_CLPCR_MASK_CORE1_WFI         (1 << 23) /* Bit 23: Mask WFI of core1 for entering low power mode */
#define CCM_CLPCR_MASK_CORE2_WFI         (1 << 24) /* Bit 24: Mask WFI of core2 for entering low power mode */
#define CCM_CLPCR_MASK_CORE3_WFI         (1 << 25) /* Bit 25: Mask WFI of core3 for entering low power mode */
#define CCM_CLPCR_MASK_SCU_IDLE          (1 << 26) /* Bit 26: Mask SCU IDLE for entering low power mode */
#define CCM_CLPCR_MASK_L2CC_IDLE         (1 << 27) /* Bit 27: Mask L2CC IDLE for entering low power mode */

/* CCM Interrupt Status Register and CCM Interrupt Mask Register */

#define CCM_CINT_LRF_PLL                       (1 << 0)  /* Bit 0:  Lock of all enabled and not bypaseed PLLs interrupt */
#define CCM_CINT_COSC_READY                    (1 << 6)  /* Bit 6:  On board oscillator ready interrupt */
#define CCM_CINT_AXI_PODF_LOADED               (1 << 17) /* Bit 17: Frequency change of axi_podf interrupt */
#define CCM_CINT_MASK_MMDC_CH0_AXI_PODF_LOADED (1 << 18) /* Bit 18: Frequency change of mmdc_ch0_axi_podf interrupt */
#define CCM_CINT_PERIPH2_CLK_SEL_LOADED        (1 << 19) /* Bit 19: Frequency change of periph2_clk_sel interrupt */
#define CCM_CINT_AHB_PODF_LOADED               (1 << 20) /* Bit 20: Frequency change of ahb_podf interrupt */
#define CCM_CINT_MMDC_CH1_PODF_LOADED          (1 << 21) /* Bit 21: Frequency change of mmdc_ch0_podf_ loaded interrupt */
#define CCM_CINT_PERIPH_CLK_SEL_LOADED         (1 << 22) /* Bit 22: Update of periph_clk_sel interrupt */
#define CCM_CINT_MMDC_CH0_PODF_LOADED          (1 << 23) /* Bit 23: Update of mmdc_ch0_axi_podf interrupt */
#define CCM_CINT_ARM_PODF_LOADED               (1 << 26) /* Bit 26: Frequency change of arm_podf interrupt */

/* CCM Clock Output Source Register */

#define CCM_CCOSR_CLKO1_SEL_SHIFT                (0)       /* Bits 0-3: Selection of the clock to be generated on CCM_CLKO1 */
#define CCM_CCOSR_CLKO1_SEL_MASK                 (15 << CCM_CCOSR_CLKO1_SEL_SHIFT)
#  define CCM_CCOSR_CLKO1_SEL_PLL3_SW_CLK        (0 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* pll3_sw_clk (/2) */
#  define CCM_CCOSR_CLKO1_SEL_PLL2_MAIN_CLK      (1 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* pll2_main_clk (/2) */
#  define CCM_CCOSR_CLKO1_SEL_PLL1_MAIN_CLK      (2 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* pll1_main_clk (/2) */
#  define CCM_CCOSR_CLKO1_SEL_PLL5_MAIN_CLK      (3 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* pll5_main_clk (/2) */
#  define CCM_CCOSR_CLKO1_SEL_VIDEO_27M_CLK_ROOT (4 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* video_27M_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_AXI_CLK_ROOT       (5 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* axi_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_ENFC_CLK_ROOT      (6 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* enfc_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_IPU1_DI0_CLK_ROOT  (7 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ipu1_di0_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_IPU1_DI1_CLK_ROOT  (8 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ipu1_di1_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_IPU2_DI0_CLK_ROOT  (9 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ipu2_di0_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_IPU2_DI1_CLK_ROOT  (10 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ipu2_di1_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_AHB_CLK_ROOT       (11 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ahb_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_IPG_CLK_ROOT       (12 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ipg_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_PERCLK_ROOT        (13 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* perclk_root */
#  define CCM_CCOSR_CLKO1_SEL_CKIL_SYNC_CLK_ROOT (14 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* ckil_sync_clk_root */
#  define CCM_CCOSR_CLKO1_SEL_PLL4_MAIN_CLK      (15 << CCM_CCOSR_CLKO1_SEL_SHIFT) /* pll4_main_clk */
#define CCM_CCOSR_CLKO1_DIV_SHIFT                (4)       /* Bits 4-6: Setting the divider of CCM_CLKO1 */
#define CCM_CCOSR_CLKO1_DIV_MASK                 (7 << CCM_CCOSR_CLKO1_DIV_SHIFT)
#  define CCM_CCOSR_CLKO1_DIV(n)                 ((uint32_t)(n) << CCM_CCOSR_CLKO1_DIV_SHIFT) /* n=(divisor-1) */
#define CCM_CCOSR_CLKO1_EN                       (1 << 7)  /* Bit 7:  Enable of CCM_CLKO1 clock */
#define CCM_CCOSR_CLK_OUT_SEL                    (1 << 8)  /* Bit 8:  CCM_CLKO1 output to reflect CCM_CLKO1 or CCM_CLKO2 clocks */
#define CCM_CCOSR_CLKO2_SEL_SHIFT                (16)       /* Bits 16-20: Selection of the clock to be generated on CCM_CLKO2 */
#define CCM_CCOSR_CLKO2_SEL_MASK                 (0x1f << CCM_CCOSR_CLKO2_SEL_SHIFT)
#  define CCM_CCOSR_CLKO2_SEL_MMDC_CH0_CLK_ROOT       (0 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* mmdc_ch0_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_MMDC_CH1_CLK_ROOT       (1 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* mmdc_ch1_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_USDHC4_CLK_ROOT         (2 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* usdhc4_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_USDHC1_CLK_ROOT         (3 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* usdhc1_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_GPU2D_AXI_CLK_ROOT      (4 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* gpu2d_axi_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_WRCK_CLK_ROOT           (5 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* wrck_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_ECSPI_CLK_ROOT          (6 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ecspi_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_GPU3D_AXI_CLK_ROOT      (7 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* gpu3d_axi_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_USDHC3_CLK_ROOT         (8 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* usdhc3_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_125M_CLK_ROOT           (9 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* 125M_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_ARM_CLK_ROOT            (10 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* arm_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_IPU1_HSP_CLK_ROOT       (11 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ipu1_hsp_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_IPU2_HSP_CLK_ROOT       (12 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ipu2_hsp_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_VDO_AXI_CLK_ROOT        (13 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* vdo_axi_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_OSC_CLK                 (14 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* osc_clk */
#  define CCM_CCOSR_CLKO2_SEL_GPU2D_CORE_CLK_ROOT     (15 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* gpu2d_core_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_GPU3D_CORE_CLK_ROOT     (16 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* gpu3d_core_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_USDHC2_CLK_ROOT         (17 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* usdhc2_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_SSI1_CLK_ROOT           (18 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ssi1_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_SSI2_CLK_ROOT           (19 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ssi2_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_SSI3_CLK_ROOT           (20 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ssi3_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_GPU3D_SHADER_CLK_ROOT   (21 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* gpu3d_shader_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_VPU_AXI_CLK_ROOT        (22 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* vpu_axi_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_CAN_CLK_ROOT            (23 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* can_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_LDB_DI0_SERIAL_CLK_ROOT (24 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ldb_di0_serial_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_LDB_DI1_SERIAL_CLK_ROOT (25 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* ldb_di1_serial_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_ESAI_CLK_ROOT           (26 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* esai_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_ACLK_EIM_SLOW_CLK_ROOT  (27 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* aclk_eim_slow_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_UART_CLK_ROOT           (28 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* uart_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_SPDIF0_CLK_ROOT         (29 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* spdif0_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_SPDIF1_CLK_ROOT         (30 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* spdif1_clk_root */
#  define CCM_CCOSR_CLKO2_SEL_HSI_TX_CLK_ROOT         (31 << CCM_CCOSR_CLKO2_SEL_SHIFT) /* hsi_tx_clk_root */
#define CCM_CCOSR_CLKO2_DIV_SHIFT                (21)       /* Bits 21-23: Setting the divider of CCM_CLKO2 */
#define CCM_CCOSR_CLKO2_DIV_MASK                 (7 << CCM_CCOSR_CLKO2_DIV_SHIFT)
#  define CCM_CCOSR_CLKO2_DIV(n)                 ((uint32_t)(n) << CCM_CCOSR_CLKO2_DIV_SHIFT) /* n=(divisor-1) */
#define CCM_CCOSR_CLKO2_EN                       (1 << 24)  /* Bit 24:  Enable of CCM_CLKO2 clock */

/* CCM General Purpose Register */

#define CCM_CGPR_PMIC_DELAY_SCALER      (1 << 0)  /* Bit 0:  Defines clock dividion of clock for stby_count */
#define CCM_CGPR_MMDC_EXT_CLK_DIS       (1 << 2)  /* Bit 2:  Disable external clock driver of MMDC during STOP mode */
#define CCM_CGPR_EFUSE_PROG_SUPPLY_GATE (1 << 4)  /* Bit 4:  Defines the value of the output signal cgpr_dout[4] */
#define CCM_CGPR_FPL                    (1 << 16) /* Bit 16: Fast PLL enable */
#define CCM_CGPR_INT_MEM_CLK_LPM        (1 << 17) /* Bit 17: Control for the Deep Sleep signal to the ARM Platform memories */

/* CCM Clock Gating Register 0-6 common definitions */

#define CCM_CCGR_OFF                    0         /* Clock is off during all modes */
#define CCM_CCGR_RUNMODE                1         /* Clock is on in run mode, but off in WAIT and STOP modes */
#define CCM_CCGR_ALLMODES               3         /* Clock is on during all modes, except STOP mode */

/* CCM Clock Gating Register 0 */

#define CCM_CCGR0_CG0_SHIFT             (0)       /* Bits 0-1: aips_tz1 clocks (aips_tz1_clk_enable) */
#define CCM_CCGR0_CG0_MASK              (3 << CCM_CCGR0_CG0_SHIFT)
#  define CCM_CCGR0_CG0(n)              ((uint32_t)(n) << CCM_CCGR0_CG0_SHIFT)
#define CCM_CCGR0_CG1_SHIFT             (2)       /* Bits 2-3: aips_tz2 clocks (aips_tz2_clk_enable) */
#define CCM_CCGR0_CG1_MASK              (3 << CCM_CCGR0_CG1_SHIFT)
#  define CCM_CCGR0_CG1(n)              ((uint32_t)(n) << CCM_CCGR0_CG1_SHIFT)
#define CCM_CCGR0_CG2_SHIFT             (4)       /* Bits 4-5: apbhdma hclk clock (apbhdma_hclk_enable) */
#define CCM_CCGR0_CG2_MASK              (3 << CCM_CCGR0_CG2_SHIFT)
#  define CCM_CCGR0_CG2(n)              ((uint32_t)(n) << CCM_CCGR0_CG2_SHIFT)
#define CCM_CCGR0_CG3_SHIFT             (6)       /* Bits 6-7: asrc clock (asrc_clk_enable) */
#define CCM_CCGR0_CG3_MASK              (3 << CCM_CCGR0_CG3_SHIFT)
#  define CCM_CCGR0_CG3(n)              ((uint32_t)(n) << CCM_CCGR0_CG3_SHIFT)
#define CCM_CCGR0_CG4_SHIFT             (8)       /* Bits 8-9: caam_secure_mem clock (caam_secure_mem_clk_enable) */
#define CCM_CCGR0_CG4_MASK              (3 << CCM_CCGR0_CG4_SHIFT)
#  define CCM_CCGR0_CG4(n)              ((uint32_t)(n) << CCM_CCGR0_CG4_SHIFT)
#define CCM_CCGR0_CG5_SHIFT             (10)      /* Bits 10-11: caam_wrapper_aclk clock (caam_wrapper_aclk_enable) */
#define CCM_CCGR0_CG5_MASK              (3 << CCM_CCGR0_CG5_SHIFT)
#  define CCM_CCGR0_CG5(n)              ((uint32_t)(n) << CCM_CCGR0_CG5_SHIFT)
#define CCM_CCGR0_CG6_SHIFT             (12)      /* Bits 12-13: caam_wrapper_ipg clock (caam_wrapper_ipg_enable) */
#define CCM_CCGR0_CG6_MASK              (3 << CCM_CCGR0_CG6_SHIFT)
#  define CCM_CCGR0_CG6(n)              ((uint32_t)(n) << CCM_CCGR0_CG6_SHIFT)
#define CCM_CCGR0_CG7_SHIFT             (14)      /* Bits 14-15: can1 clock (can1_clk_enable) */
#define CCM_CCGR0_CG7_MASK              (3 << CCM_CCGR0_CG7_SHIFT)
#  define CCM_CCGR0_CG7(n)              ((uint32_t)(n) << CCM_CCGR0_CG7_SHIFT)
#define CCM_CCGR0_CG8_SHIFT             (16)      /* Bits 16-17: can1_serial clock (can1_serial_clk_enable) */
#define CCM_CCGR0_CG8_MASK              (3 << CCM_CCGR0_CG8_SHIFT)
#  define CCM_CCGR0_CG8(n)              ((uint32_t)(n) << CCM_CCGR0_CG8_SHIFT)
#define CCM_CCGR0_CG9_SHIFT             (18)      /* Bits 18-19: can2 clock (can2_clk_enable) */
#define CCM_CCGR0_CG9_MASK              (3 << CCM_CCGR0_CG9_SHIFT)
#  define CCM_CCGR0_CG9(n)              ((uint32_t)(n) << CCM_CCGR0_CG9_SHIFT)
#define CCM_CCGR0_CG10_SHIFT            (20)      /* Bits 20-21: can2_serial clock (can2_serial_clk_enable) */
#define CCM_CCGR0_CG10_MASK             (3 << CCM_CCGR0_CG10_SHIFT)
#  define CCM_CCGR0_CG10(n)             ((uint32_t)(n) << CCM_CCGR0_CG10_SHIFT)
#define CCM_CCGR0_CG11_SHIFT            (22)      /* Bits 22-23: CPU debug clocks (arm_dbg_clk_enable) */
#define CCM_CCGR0_CG11_MASK             (3 << CCM_CCGR0_CG11_SHIFT)
#  define CCM_CCGR0_CG11(n)             ((uint32_t)(n) << CCM_CCGR0_CG11_SHIFT)
#define CCM_CCGR0_CG12_SHIFT            (24)      /* Bits 24-25: dcic1 clocks (dcic1_clk_enable) */
#define CCM_CCGR0_CG12_MASK             (3 << CCM_CCGR0_CG12_SHIFT)
#  define CCM_CCGR0_CG12(n)             ((uint32_t)(n) << CCM_CCGR0_CG12_SHIFT)
#define CCM_CCGR0_CG13_SHIFT            (26)      /* Bits 26-27: dcic2 clocks (dcic2_clk_enable) */
#define CCM_CCGR0_CG13_MASK             (3 << CCM_CCGR0_CG13_SHIFT)
#  define CCM_CCGR0_CG13(n)             ((uint32_t)(n) << CCM_CCGR0_CG13_SHIFT)
#define CCM_CCGR0_CG14_SHIFT            (28)      /* Bits 28-29: dtcp clocks (dtcp_clk_enable) */
#define CCM_CCGR0_CG14_MASK             (3 << CCM_CCGR0_CG14_SHIFT)
#  define CCM_CCGR0_CG14(n)             ((uint32_t)(n) << CCM_CCGR0_CG14_SHIFT)
#define CCM_CCGR0_CG15_SHIFT            (30)      /* Bits 30-31: Reserved */
#define CCM_CCGR0_CG15_MASK             (3 << CCM_CCGR0_CG15_SHIFT)
#  define CCM_CCGR0_CG15(n)             ((uint32_t)(n) << CCM_CCGR0_CG15_SHIFT)

/* CCM Clock Gating Register 1 */

#define CCM_CCGR1_CG0_SHIFT             (0)       /* Bits 0-1: ecspi1 clocks (ecspi1_clk_enable) */
#define CCM_CCGR1_CG0_MASK              (3 << CCM_CCGR1_CG0_SHIFT)
#  define CCM_CCGR1_CG0(n)              ((uint32_t)(n) << CCM_CCGR1_CG0_SHIFT)
#define CCM_CCGR1_CG1_SHIFT             (2)       /* Bits 2-3: ecspi2 clocks (ecspi2_clk_enable) */
#define CCM_CCGR1_CG1_MASK              (3 << CCM_CCGR1_CG1_SHIFT)
#  define CCM_CCGR1_CG1(n)              ((uint32_t)(n) << CCM_CCGR1_CG1_SHIFT)
#define CCM_CCGR1_CG2_SHIFT             (4)       /* Bits 4-5: ecspi3 clocks (ecspi3_clk_enable) */
#define CCM_CCGR1_CG2_MASK              (3 << CCM_CCGR1_CG2_SHIFT)
#  define CCM_CCGR1_CG2(n)              ((uint32_t)(n) << CCM_CCGR1_CG2_SHIFT)
#define CCM_CCGR1_CG3_SHIFT             (6)       /* Bits 6-7: ecspi4 clocks (ecspi4_clk_enable) */
#define CCM_CCGR1_CG3_MASK              (3 << CCM_CCGR1_CG3_SHIFT)
#  define CCM_CCGR1_CG3(n)              ((uint32_t)(n) << CCM_CCGR1_CG3_SHIFT)
#define CCM_CCGR1_CG4_SHIFT             (8)       /* Bits 8-9: ecspi5 clocks (ecspi5_clk_enable) */
#define CCM_CCGR1_CG4_MASK              (3 << CCM_CCGR1_CG4_SHIFT)
#  define CCM_CCGR1_CG4(n)              ((uint32_t)(n) << CCM_CCGR1_CG4_SHIFT)
#define CCM_CCGR1_CG5_SHIFT             (10)      /* Bits 10-11: enet clock (enet_clk_enable) */
#define CCM_CCGR1_CG5_MASK              (3 << CCM_CCGR1_CG5_SHIFT)
#  define CCM_CCGR1_CG5(n)              ((uint32_t)(n) << CCM_CCGR1_CG5_SHIFT)
#define CCM_CCGR1_CG6_SHIFT             (12)      /* Bits 12-13: epit1 clocks (epit1_clk_enable) */
#define CCM_CCGR1_CG6_MASK              (3 << CCM_CCGR1_CG6_SHIFT)
#  define CCM_CCGR1_CG6(n)              ((uint32_t)(n) << CCM_CCGR1_CG6_SHIFT)
#define CCM_CCGR1_CG7_SHIFT             (14)      /* Bits 14-15: epit2 clocks (epit2_clk_enable) */
#define CCM_CCGR1_CG7_MASK              (3 << CCM_CCGR1_CG7_SHIFT)
#  define CCM_CCGR1_CG7(n)              ((uint32_t)(n) << CCM_CCGR1_CG7_SHIFT)
#define CCM_CCGR1_CG8_SHIFT             (16)      /* Bits 16-17: esai clocks (esai_clk_enable) */
#define CCM_CCGR1_CG8_MASK              (3 << CCM_CCGR1_CG8_SHIFT)
#  define CCM_CCGR1_CG8(n)              ((uint32_t)(n) << CCM_CCGR1_CG8_SHIFT)
#define CCM_CCGR1_CG9_SHIFT             (18)      /* Bits 18-19: Reserved */
#define CCM_CCGR1_CG9_MASK              (3 << CCM_CCGR1_CG9_SHIFT)
#  define CCM_CCGR1_CG9(n)              ((uint32_t)(n) << CCM_CCGR1_CG9_SHIFT)
#define CCM_CCGR1_CG10_SHIFT            (20)      /* Bits 20-21: gpt bus clock (gpt_clk_enable) */
#define CCM_CCGR1_CG10_MASK             (3 << CCM_CCGR1_CG10_SHIFT)
#  define CCM_CCGR1_CG10(n)             ((uint32_t)(n) << CCM_CCGR1_CG10_SHIFT)
#define CCM_CCGR1_CG11_SHIFT            (22)      /* Bits 22-23: gpt serial clock (gpt_serial_clk_enable) */
#define CCM_CCGR1_CG11_MASK             (3 << CCM_CCGR1_CG11_SHIFT)
#  define CCM_CCGR1_CG11(n)             ((uint32_t)(n) << CCM_CCGR1_CG11_SHIFT)
#define CCM_CCGR1_CG12_SHIFT            (24)      /* Bits 24-25: gpu2d clock (gpu2d_clk_enable) */
#define CCM_CCGR1_CG12_MASK             (3 << CCM_CCGR1_CG12_SHIFT)
#  define CCM_CCGR1_CG12(n)             ((uint32_t)(n) << CCM_CCGR1_CG12_SHIFT)
#define CCM_CCGR1_CG13_SHIFT            (26)      /* Bits 26-27: gpu3d clock (gpu3d_clk_enable) */
#define CCM_CCGR1_CG13_MASK             (3 << CCM_CCGR1_CG13_SHIFT)
#  define CCM_CCGR1_CG13(n)             ((uint32_t)(n) << CCM_CCGR1_CG13_SHIFT)
#define CCM_CCGR1_CG14_SHIFT            (28)      /* Bits 28-29: Reserved */
#define CCM_CCGR1_CG14_MASK             (3 << CCM_CCGR1_CG14_SHIFT)
#  define CCM_CCGR1_CG14(n)             ((uint32_t)(n) << CCM_CCGR1_CG14_SHIFT)
#define CCM_CCGR1_CG15_SHIFT            (30)      /* Bits 30-31: Reserved */
#define CCM_CCGR1_CG15_MASK             (3 << CCM_CCGR1_CG15_SHIFT)
#  define CCM_CCGR1_CG15(n)             ((uint32_t)(n) << CCM_CCGR1_CG15_SHIFT)

/* CCM Clock Gating Register 2 */

#define CCM_CCGR2_CG0_SHIFT             (0)       /* Bits 0-1: hdmi_tx_iahbclk, hdmi_tx_ihclk clock (hdmi_tx_enable) */
#define CCM_CCGR2_CG0_MASK              (3 << CCM_CCGR2_CG0_SHIFT)
#  define CCM_CCGR2_CG0(n)              ((uint32_t)(n) << CCM_CCGR2_CG0_SHIFT)
#define CCM_CCGR2_CG1_SHIFT             (2)       /* Bits 2-3: Reserved */
#define CCM_CCGR2_CG1_MASK              (3 << CCM_CCGR2_CG1_SHIFT)
#  define CCM_CCGR2_CG1(n)              ((uint32_t)(n) << CCM_CCGR2_CG1_SHIFT)
#define CCM_CCGR2_CG2_SHIFT             (4)       /* Bits 4-5: hdmi_tx_isfrclk clock (hdmi_tx_isfrclk_enable) */
#define CCM_CCGR2_CG2_MASK              (3 << CCM_CCGR2_CG2_SHIFT)
#  define CCM_CCGR2_CG2(n)              ((uint32_t)(n) << CCM_CCGR2_CG2_SHIFT)
#define CCM_CCGR2_CG3_SHIFT             (6)       /* Bits 6-7: i2c1_serial clock (i2c1_serial_clk_enable) */
#define CCM_CCGR2_CG3_MASK              (3 << CCM_CCGR2_CG3_SHIFT)
#  define CCM_CCGR2_CG3(n)              ((uint32_t)(n) << CCM_CCGR2_CG3_SHIFT)
#define CCM_CCGR2_CG4_SHIFT             (8)       /* Bits 8-9: i2c2_serial clock (i2c2_serial_clk_enable) */
#define CCM_CCGR2_CG4_MASK              (3 << CCM_CCGR2_CG4_SHIFT)
#  define CCM_CCGR2_CG4(n)              ((uint32_t)(n) << CCM_CCGR2_CG4_SHIFT)
#define CCM_CCGR2_CG5_SHIFT             (10)      /* Bits 10-11: i2c3_serial clock (i2c3_serial_clk_enable) */
#define CCM_CCGR2_CG5_MASK              (3 << CCM_CCGR2_CG5_SHIFT)
#  define CCM_CCGR2_CG5(n)              ((uint32_t)(n) << CCM_CCGR2_CG5_SHIFT)
#define CCM_CCGR2_CG6_SHIFT             (12)      /* Bits 12-13: OCOTP_CTRL clock (iim_clk_enable) */
#define CCM_CCGR2_CG6_MASK              (3 << CCM_CCGR2_CG6_SHIFT)
#  define CCM_CCGR2_CG6(n)              ((uint32_t)(n) << CCM_CCGR2_CG6_SHIFT)
#define CCM_CCGR2_CG7_SHIFT             (14)      /* Bits 14-15: iomux_ipt_clk_io clock (iomux_ipt_clk_io_enable) */
#define CCM_CCGR2_CG7_MASK              (3 << CCM_CCGR2_CG7_SHIFT)
#  define CCM_CCGR2_CG7(n)              ((uint32_t)(n) << CCM_CCGR2_CG7_SHIFT)
#define CCM_CCGR2_CG8_SHIFT             (16)      /* Bits 16-17: ipmux1 clock (ipmux1_clk_enable) */
#define CCM_CCGR2_CG8_MASK              (3 << CCM_CCGR2_CG8_SHIFT)
#  define CCM_CCGR2_CG8(n)              ((uint32_t)(n) << CCM_CCGR2_CG8_SHIFT)
#define CCM_CCGR2_CG9_SHIFT             (18)      /* Bits 18-19: ipmux2 clock (ipmux2_clk_enable) */
#define CCM_CCGR2_CG9_MASK              (3 << CCM_CCGR2_CG9_SHIFT)
#  define CCM_CCGR2_CG9(n)              ((uint32_t)(n) << CCM_CCGR2_CG9_SHIFT)
#define CCM_CCGR2_CG10_SHIFT            (20)      /* Bits 20-21: ipmux3 clock (ipmux3_clk_enable) */
#define CCM_CCGR2_CG10_MASK             (3 << CCM_CCGR2_CG10_SHIFT)
#  define CCM_CCGR2_CG10(n)             ((uint32_t)(n) << CCM_CCGR2_CG10_SHIFT)
#define CCM_CCGR2_CG11_SHIFT            (22)      /* Bits 22-23: ipsync_ip2apb_tzasc1_ipg clocks (ipsync_ip2apb_tzasc1_ipg_master_clk_enable) */
#define CCM_CCGR2_CG11_MASK             (3 << CCM_CCGR2_CG11_SHIFT)
#  define CCM_CCGR2_CG11(n)             ((uint32_t)(n) << CCM_CCGR2_CG11_SHIFT)
#define CCM_CCGR2_CG12_SHIFT            (24)      /* Bits 24-25: ipsync_ip2apb_tzasc2_ipg clocks (ipsync_ip2apb_tzasc2_ipg_master_clk_enable) */
#define CCM_CCGR2_CG12_MASK             (3 << CCM_CCGR2_CG12_SHIFT)
#  define CCM_CCGR2_CG12(n)             ((uint32_t)(n) << CCM_CCGR2_CG12_SHIFT)
#define CCM_CCGR2_CG13_SHIFT            (26)      /* Bits 26-27: ipsync_vdoa_ipg clocks (ipsync_vdoa_ipg_master_clk_enable)) */
#define CCM_CCGR2_CG13_MASK             (3 << CCM_CCGR2_CG13_SHIFT)
#  define CCM_CCGR2_CG13(n)             ((uint32_t)(n) << CCM_CCGR2_CG13_SHIFT)
#define CCM_CCGR2_CG14_SHIFT            (28)      /* Bits 28-29: Reserved */
#define CCM_CCGR2_CG14_MASK             (3 << CCM_CCGR2_CG14_SHIFT)
#  define CCM_CCGR2_CG14(n)             ((uint32_t)(n) << CCM_CCGR2_CG14_SHIFT)
#define CCM_CCGR2_CG15_SHIFT            (30)      /* Bits 30-31: Reserved */
#define CCM_CCGR2_CG15_MASK             (3 << CCM_CCGR2_CG15_SHIFT)
#  define CCM_CCGR2_CG15(n)             ((uint32_t)(n) << CCM_CCGR2_CG15_SHIFT)

/* CCM Clock Gating Register 3 */

#define CCM_CCGR3_CG0_SHIFT             (0)       /* Bits 0-1: ipu1_ipu clock (ipu1_ipu_clk_enable) */
#define CCM_CCGR3_CG0_MASK              (3 << CCM_CCGR3_CG0_SHIFT)
#  define CCM_CCGR3_CG0(n)              ((uint32_t)(n) << CCM_CCGR3_CG0_SHIFT)
#define CCM_CCGR3_CG1_SHIFT             (2)       /* Bits 2-3: ipu1_di0 clock and pre-clock (ipu1_ipu_di0_clk_enable) */
#define CCM_CCGR3_CG1_MASK              (3 << CCM_CCGR3_CG1_SHIFT)
#  define CCM_CCGR3_CG1(n)              ((uint32_t)(n) << CCM_CCGR3_CG1_SHIFT)
#define CCM_CCGR3_CG2_SHIFT             (4)       /* Bits 4-5: ipu1_di1 clock and pre-clock (ipu1_ipu_di1_clk_enable) */
#define CCM_CCGR3_CG2_MASK              (3 << CCM_CCGR3_CG2_SHIFT)
#  define CCM_CCGR3_CG2(n)              ((uint32_t)(n) << CCM_CCGR3_CG2_SHIFT)
#define CCM_CCGR3_CG3_SHIFT             (6)       /* Bits 6-7: ipu2_ipu clock (ipu2_ipu_clk_enable) */
#define CCM_CCGR3_CG3_MASK              (3 << CCM_CCGR3_CG3_SHIFT)
#  define CCM_CCGR3_CG3(n)              ((uint32_t)(n) << CCM_CCGR3_CG3_SHIFT)
#define CCM_CCGR3_CG4_SHIFT             (8)       /* Bits 8-9: ipu2_di0 clock and pre-clock (ipu2_ipu_di0_clk_enable) */
#define CCM_CCGR3_CG4_MASK              (3 << CCM_CCGR3_CG4_SHIFT)
#  define CCM_CCGR3_CG4(n)              ((uint32_t)(n) << CCM_CCGR3_CG4_SHIFT)
#define CCM_CCGR3_CG5_SHIFT             (10)      /* Bits 10-11: ipu2_di1 clock and pre-clock (ipu2_ipu_di1_clk_enable) */
#define CCM_CCGR3_CG5_MASK              (3 << CCM_CCGR3_CG5_SHIFT)
#  define CCM_CCGR3_CG5(n)              ((uint32_t)(n) << CCM_CCGR3_CG5_SHIFT)
#define CCM_CCGR3_CG6_SHIFT             (12)      /* Bits 12-13: ldb_di0 clock (ldb_di0_clk_enable) */
#define CCM_CCGR3_CG6_MASK              (3 << CCM_CCGR3_CG6_SHIFT)
#  define CCM_CCGR3_CG6(n)              ((uint32_t)(n) << CCM_CCGR3_CG6_SHIFT)
#define CCM_CCGR3_CG7_SHIFT             (14)      /* Bits 14-15: ldb_di1 clock (ldb_di1_clk_enable) */
#define CCM_CCGR3_CG7_MASK              (3 << CCM_CCGR3_CG7_SHIFT)
#  define CCM_CCGR3_CG7(n)              ((uint32_t)(n) << CCM_CCGR3_CG7_SHIFT)
#define CCM_CCGR3_CG8_SHIFT             (16)      /* Bits 16-17: mipi_core_cfg clock (mipi_core_cfg_clk_enable) */
#define CCM_CCGR3_CG8_MASK              (3 << CCM_CCGR3_CG8_SHIFT)
#  define CCM_CCGR3_CG8(n)              ((uint32_t)(n) << CCM_CCGR3_CG8_SHIFT)
#define CCM_CCGR3_CG9_SHIFT             (18)      /* Bits 18-19: mlb clock (mlb_clk_enable) */
#define CCM_CCGR3_CG9_MASK              (3 << CCM_CCGR3_CG9_SHIFT)
#  define CCM_CCGR3_CG9(n)              ((uint32_t)(n) << CCM_CCGR3_CG9_SHIFT)
#define CCM_CCGR3_CG10_SHIFT            (20)      /* Bits 20-21: mmdc_core_aclk_fast_core_p0 clock (mmdc_core_aclk_fast_core_p0_enable) */
#define CCM_CCGR3_CG10_MASK             (3 << CCM_CCGR3_CG10_SHIFT)
#  define CCM_CCGR3_CG10(n)             ((uint32_t)(n) << CCM_CCGR3_CG10_SHIFT)
#define CCM_CCGR3_CG11_SHIFT            (22)      /* Bits 22-23: Reserved */
#define CCM_CCGR3_CG11_MASK             (3 << CCM_CCGR3_CG11_SHIFT)
#  define CCM_CCGR3_CG11(n)             ((uint32_t)(n) << CCM_CCGR3_CG11_SHIFT)
#define CCM_CCGR3_CG12_SHIFT            (24)      /* Bits 24-25: mmdc_core_ipg_clk_p0 clock (mmdc_core_ipg_clk_p0_enable) */
#define CCM_CCGR3_CG12_MASK             (3 << CCM_CCGR3_CG12_SHIFT)
#  define CCM_CCGR3_CG12(n)             ((uint32_t)(n) << CCM_CCGR3_CG12_SHIFT)
#define CCM_CCGR3_CG13_SHIFT            (26)      /* Bits 26-27: Reserved */
#define CCM_CCGR3_CG13_MASK             (3 << CCM_CCGR3_CG13_SHIFT)
#  define CCM_CCGR3_CG13(n)             ((uint32_t)(n) << CCM_CCGR3_CG13_SHIFT)
#define CCM_CCGR3_CG14_SHIFT            (28)      /* Bits 28-29: ocram clock (ocram_clk_enable) */
#define CCM_CCGR3_CG14_MASK             (3 << CCM_CCGR3_CG14_SHIFT)
#  define CCM_CCGR3_CG14(n)             ((uint32_t)(n) << CCM_CCGR3_CG14_SHIFT)
#define CCM_CCGR3_CG15_SHIFT            (30)      /* Bits 30-31: openvgaxiclk clock (openvgaxiclk_clk_root_enable) */
#define CCM_CCGR3_CG15_MASK             (3 << CCM_CCGR3_CG15_SHIFT)
#  define CCM_CCGR3_CG15(n)             ((uint32_t)(n) << CCM_CCGR3_CG15_SHIFT)

/* CCM Clock Gating Register 4 */

#define CCM_CCGR4_CG0_SHIFT             (0)       /* Bits 0-1: pcie clock (pcie_root_enable) */
#define CCM_CCGR4_CG0_MASK              (3 << CCM_CCGR4_CG0_SHIFT)
#  define CCM_CCGR4_CG0(n)              ((uint32_t)(n) << CCM_CCGR4_CG0_SHIFT)
#define CCM_CCGR4_CG1_SHIFT             (2)       /* Bits 2-3: Reserved */
#define CCM_CCGR4_CG1_MASK              (3 << CCM_CCGR4_CG1_SHIFT)
#  define CCM_CCGR4_CG1(n)              ((uint32_t)(n) << CCM_CCGR4_CG1_SHIFT)
#define CCM_CCGR4_CG2_SHIFT             (4)       /* Bits 4-5: Reserved */
#define CCM_CCGR4_CG2_MASK              (3 << CCM_CCGR4_CG2_SHIFT)
#  define CCM_CCGR4_CG2(n)              ((uint32_t)(n) << CCM_CCGR4_CG2_SHIFT)
#define CCM_CCGR4_CG3_SHIFT             (6)       /* Bits 6-7: ? */
#define CCM_CCGR4_CG3_MASK              (3 << CCM_CCGR4_CG3_SHIFT)
#  define CCM_CCGR4_CG3(n)              ((uint32_t)(n) << CCM_CCGR4_CG3_SHIFT)
#define CCM_CCGR4_CG4_SHIFT             (8)       /* Bits 8-9: pl301_mx6qfast1_s133 clock (pl301_mx6qfast1_s133clk_enable) */
#define CCM_CCGR4_CG4_MASK              (3 << CCM_CCGR4_CG4_SHIFT)
#  define CCM_CCGR4_CG4(n)              ((uint32_t)(n) << CCM_CCGR4_CG4_SHIFT)
#define CCM_CCGR4_CG5_SHIFT             (10)      /* Bits 10-11: Reserved */
#define CCM_CCGR4_CG5_MASK              (3 << CCM_CCGR4_CG5_SHIFT)
#  define CCM_CCGR4_CG5(n)              ((uint32_t)(n) << CCM_CCGR4_CG5_SHIFT)
#define CCM_CCGR4_CG6_SHIFT             (12)      /* Bits 12-13: pl301_mx6qper1_bch clocks (pl301_mx6qper1_bchclk_enable) */
#define CCM_CCGR4_CG6_MASK              (3 << CCM_CCGR4_CG6_SHIFT)
#  define CCM_CCGR4_CG6(n)              ((uint32_t)(n) << CCM_CCGR4_CG6_SHIFT)
#define CCM_CCGR4_CG7_SHIFT             (14)      /* Bits 14-15: pl301_mx6qper2_mainclk_enable (pl301_mx6qper2_mainclk_enable) */
#define CCM_CCGR4_CG7_MASK              (3 << CCM_CCGR4_CG7_SHIFT)
#  define CCM_CCGR4_CG7(n)              ((uint32_t)(n) << CCM_CCGR4_CG7_SHIFT)
#define CCM_CCGR4_CG8_SHIFT             (16)      /* Bits 16-17: pwm1 clocks (pwm1_clk_enable) */
#define CCM_CCGR4_CG8_MASK              (3 << CCM_CCGR4_CG8_SHIFT)
#  define CCM_CCGR4_CG8(n)              ((uint32_t)(n) << CCM_CCGR4_CG8_SHIFT)
#define CCM_CCGR4_CG9_SHIFT             (18)      /* Bits 18-19: pwm2 clocks (pwm2_clk_enable) */
#define CCM_CCGR4_CG9_MASK              (3 << CCM_CCGR4_CG9_SHIFT)
#  define CCM_CCGR4_CG9(n)              ((uint32_t)(n) << CCM_CCGR4_CG9_SHIFT)
#define CCM_CCGR4_CG10_SHIFT            (20)      /* Bits 20-21: pwm3 clocks (pwm3_clk_enable) */
#define CCM_CCGR4_CG10_MASK             (3 << CCM_CCGR4_CG10_SHIFT)
#  define CCM_CCGR4_CG10(n)             ((uint32_t)(n) << CCM_CCGR4_CG10_SHIFT)
#define CCM_CCGR4_CG11_SHIFT            (22)      /* Bits 22-23: pwm4 clocks (pwm4_clk_enable) */
#define CCM_CCGR4_CG11_MASK             (3 << CCM_CCGR4_CG11_SHIFT)
#  define CCM_CCGR4_CG11(n)             ((uint32_t)(n) << CCM_CCGR4_CG11_SHIFT)
#define CCM_CCGR4_CG12_SHIFT            (24)      /* Bits 24-25: rawnand_u_bch_input_apb clock (rawnand_u_bch_input_apb_clk_enable) */
#define CCM_CCGR4_CG12_MASK             (3 << CCM_CCGR4_CG12_SHIFT)
#  define CCM_CCGR4_CG12(n)             ((uint32_t)(n) << CCM_CCGR4_CG12_SHIFT)
#define CCM_CCGR4_CG13_SHIFT            (26)      /* Bits 26-27: rawnand_u_gpmi_bch_input_bch clock (rawnand_u_gpmi_bch_input_bch_clk_enable)) */
#define CCM_CCGR4_CG13_MASK             (3 << CCM_CCGR4_CG13_SHIFT)
#  define CCM_CCGR4_CG13(n)             ((uint32_t)(n) << CCM_CCGR4_CG13_SHIFT)
#define CCM_CCGR4_CG14_SHIFT            (28)      /* Bits 28-29: rawnand_u_gpmi_bch_input_gpmi_io clock (rawnand_u_gpmi_bch_input_gpmi_io_clk_enable) */
#define CCM_CCGR4_CG14_MASK             (3 << CCM_CCGR4_CG14_SHIFT)
#  define CCM_CCGR4_CG14(n)             ((uint32_t)(n) << CCM_CCGR4_CG14_SHIFT)
#define CCM_CCGR4_CG15_SHIFT            (30)      /* Bits 30-31: rawnand_u_gpmi_input_apb clock (rawnand_u_gpmi_input_apb_clk_enable) */
#define CCM_CCGR4_CG15_MASK             (3 << CCM_CCGR4_CG15_SHIFT)
#  define CCM_CCGR4_CG15(n)             ((uint32_t)(n) << CCM_CCGR4_CG15_SHIFT)

/* CCM Clock Gating Register 5 */

#define CCM_CCGR5_CG0_SHIFT             (0)       /* Bits 0-1: rom clock (rom_clk_enable) */
#define CCM_CCGR5_CG0_MASK              (3 << CCM_CCGR5_CG0_SHIFT)
#  define CCM_CCGR5_CG0(n)              ((uint32_t)(n) << CCM_CCGR5_CG0_SHIFT)
#define CCM_CCGR5_CG1_SHIFT             (2)       /* Bits 2-3: Reserved */
#define CCM_CCGR5_CG1_MASK              (3 << CCM_CCGR5_CG1_SHIFT)
#  define CCM_CCGR5_CG1(n)              ((uint32_t)(n) << CCM_CCGR5_CG1_SHIFT)
#define CCM_CCGR5_CG2_SHIFT             (4)       /* Bits 4-5: sata clock (sata_clk_enable) */
#define CCM_CCGR5_CG2_MASK              (3 << CCM_CCGR5_CG2_SHIFT)
#  define CCM_CCGR5_CG2(n)              ((uint32_t)(n) << CCM_CCGR5_CG2_SHIFT)
#define CCM_CCGR5_CG3_SHIFT             (6)       /* Bits 6-7: sdma clock (sdma_clk_enable) */
#define CCM_CCGR5_CG3_MASK              (3 << CCM_CCGR5_CG3_SHIFT)
#  define CCM_CCGR5_CG3(n)              ((uint32_t)(n) << CCM_CCGR5_CG3_SHIFT)
#define CCM_CCGR5_CG4_SHIFT             (8)       /* Bits 8-9: Reserved */
#define CCM_CCGR5_CG4_MASK              (3 << CCM_CCGR5_CG4_SHIFT)
#  define CCM_CCGR5_CG4(n)              ((uint32_t)(n) << CCM_CCGR5_CG4_SHIFT)
#define CCM_CCGR5_CG5_SHIFT             (10)      /* Bits 10-11: Reserved */
#define CCM_CCGR5_CG5_MASK              (3 << CCM_CCGR5_CG5_SHIFT)
#  define CCM_CCGR5_CG5(n)              ((uint32_t)(n) << CCM_CCGR5_CG5_SHIFT)
#define CCM_CCGR5_CG6_SHIFT             (12)      /* Bits 12-13: spba clock (spba_clk_enable) */
#define CCM_CCGR5_CG6_MASK              (3 << CCM_CCGR5_CG6_SHIFT)
#  define CCM_CCGR5_CG6(n)              ((uint32_t)(n) << CCM_CCGR5_CG6_SHIFT)
#define CCM_CCGR5_CG7_SHIFT             (14)      /* Bits 14-15: spdif clock (spdif_clk_enable) */
#define CCM_CCGR5_CG7_MASK              (3 << CCM_CCGR5_CG7_SHIFT)
#  define CCM_CCGR5_CG7(n)              ((uint32_t)(n) << CCM_CCGR5_CG7_SHIFT)
#define CCM_CCGR5_CG8_SHIFT             (16)      /* Bits 16-17: Reserved */
#define CCM_CCGR5_CG8_MASK              (3 << CCM_CCGR5_CG8_SHIFT)
#  define CCM_CCGR5_CG8(n)              ((uint32_t)(n) << CCM_CCGR5_CG8_SHIFT)
#define CCM_CCGR5_CG9_SHIFT             (18)      /* Bits 18-19: ssi1 clocks (ssi1_clk_enable) */
#define CCM_CCGR5_CG9_MASK              (3 << CCM_CCGR5_CG9_SHIFT)
#  define CCM_CCGR5_CG9(n)              ((uint32_t)(n) << CCM_CCGR5_CG9_SHIFT)
#define CCM_CCGR5_CG10_SHIFT            (20)      /* Bits 20-21: ssi2 clocks (ssi2_clk_enable) */
#define CCM_CCGR5_CG10_MASK             (3 << CCM_CCGR5_CG10_SHIFT)
#  define CCM_CCGR5_CG10(n)             ((uint32_t)(n) << CCM_CCGR5_CG10_SHIFT)
#define CCM_CCGR5_CG11_SHIFT            (22)      /* Bits 22-23: ssi3 clocks (ssi3_clk_enable) */
#define CCM_CCGR5_CG11_MASK             (3 << CCM_CCGR5_CG11_SHIFT)
#  define CCM_CCGR5_CG11(n)             ((uint32_t)(n) << CCM_CCGR5_CG11_SHIFT)
#define CCM_CCGR5_CG12_SHIFT            (24)      /* Bits 24-25: uart clock (uart_clk_enable) */
#define CCM_CCGR5_CG12_MASK             (3 << CCM_CCGR5_CG12_SHIFT)
#  define CCM_CCGR5_CG12(n)             ((uint32_t)(n) << CCM_CCGR5_CG12_SHIFT)
#define CCM_CCGR5_CG13_SHIFT            (26)      /* Bits 26-27: uart_serial clock (uart_serial_clk_enable)) */
#define CCM_CCGR5_CG13_MASK             (3 << CCM_CCGR5_CG13_SHIFT)
#  define CCM_CCGR5_CG13(n)             ((uint32_t)(n) << CCM_CCGR5_CG13_SHIFT)
#define CCM_CCGR5_CG14_SHIFT            (28)      /* Bits 28-29: Reserved */
#define CCM_CCGR5_CG14_MASK             (3 << CCM_CCGR5_CG14_SHIFT)
#  define CCM_CCGR5_CG14(n)             ((uint32_t)(n) << CCM_CCGR5_CG14_SHIFT)
#define CCM_CCGR5_CG15_SHIFT            (30)      /* Bits 30-31: Reserved */
#define CCM_CCGR5_CG15_MASK             (3 << CCM_CCGR5_CG15_SHIFT)
#  define CCM_CCGR5_CG15(n)             ((uint32_t)(n) << CCM_CCGR5_CG15_SHIFT)

/* CCM Clock Gating Register 6 */

#define CCM_CCGR6_CG0_SHIFT             (0)       /* Bits 0-1: usboh3 clock (usboh3_clk_enable) */
#define CCM_CCGR6_CG0_MASK              (3 << CCM_CCGR6_CG0_SHIFT)
#  define CCM_CCGR6_CG0(n)              ((uint32_t)(n) << CCM_CCGR6_CG0_SHIFT)
#define CCM_CCGR6_CG1_SHIFT             (2)       /* Bits 2-3: usdhc1 clocks (usdhc1_clk_enable) */
#define CCM_CCGR6_CG1_MASK              (3 << CCM_CCGR6_CG1_SHIFT)
#  define CCM_CCGR6_CG1(n)              ((uint32_t)(n) << CCM_CCGR6_CG1_SHIFT)
#define CCM_CCGR6_CG2_SHIFT             (4)       /* Bits 4-5: usdhc2 clocks (usdhc2_clk_enable) */
#define CCM_CCGR6_CG2_MASK              (3 << CCM_CCGR6_CG2_SHIFT)
#  define CCM_CCGR6_CG2(n)              ((uint32_t)(n) << CCM_CCGR6_CG2_SHIFT)
#define CCM_CCGR6_CG3_SHIFT             (6)       /* Bits 6-7: usdhc3 clocks (usdhc3_clk_enable) */
#define CCM_CCGR6_CG3_MASK              (3 << CCM_CCGR6_CG3_SHIFT)
#  define CCM_CCGR6_CG3(n)              ((uint32_t)(n) << CCM_CCGR6_CG3_SHIFT)
#define CCM_CCGR6_CG4_SHIFT             (8)       /* Bits 8-9: usdhc4 clocks (usdhc4_clk_enable) */
#define CCM_CCGR6_CG4_MASK              (3 << CCM_CCGR6_CG4_SHIFT)
#  define CCM_CCGR6_CG4(n)              ((uint32_t)(n) << CCM_CCGR6_CG4_SHIFT)
#define CCM_CCGR6_CG5_SHIFT             (10)      /* Bits 10-11: eim_slow clocks (eim_slow_clk_enable) */
#define CCM_CCGR6_CG5_MASK              (3 << CCM_CCGR6_CG5_SHIFT)
#  define CCM_CCGR6_CG5(n)              ((uint32_t)(n) << CCM_CCGR6_CG5_SHIFT)
#define CCM_CCGR6_CG6_SHIFT             (12)      /* Bits 12-13: vdoaxiclk root clock (vdoaxiclk_clk_enable) */
#define CCM_CCGR6_CG6_MASK              (3 << CCM_CCGR6_CG6_SHIFT)
#  define CCM_CCGR6_CG6(n)              ((uint32_t)(n) << CCM_CCGR6_CG6_SHIFT)
#define CCM_CCGR6_CG7_SHIFT             (14)      /* Bits 14-15: vpu clocks (vpu_clk_enable) */
#define CCM_CCGR6_CG7_MASK              (3 << CCM_CCGR6_CG7_SHIFT)
#  define CCM_CCGR6_CG7(n)              ((uint32_t)(n) << CCM_CCGR6_CG7_SHIFT)
#define CCM_CCGR6_CG8_SHIFT             (16)      /* Bits 16-17: Reserved */
#define CCM_CCGR6_CG8_MASK              (3 << CCM_CCGR6_CG8_SHIFT)
#  define CCM_CCGR6_CG8(n)              ((uint32_t)(n) << CCM_CCGR6_CG8_SHIFT)
#define CCM_CCGR6_CG9_SHIFT             (18)      /* Bits 18-19: Reserved */
#define CCM_CCGR6_CG9_MASK              (3 << CCM_CCGR6_CG9_SHIFT)
#  define CCM_CCGR6_CG9(n)              ((uint32_t)(n) << CCM_CCGR6_CG9_SHIFT)
#define CCM_CCGR6_CG10_SHIFT            (20)      /* Bits 20-21: Reserved */
#define CCM_CCGR6_CG10_MASK             (3 << CCM_CCGR6_CG10_SHIFT)
#  define CCM_CCGR6_CG10(n)             ((uint32_t)(n) << CCM_CCGR6_CG10_SHIFT)
#define CCM_CCGR6_CG11_SHIFT            (22)      /* Bits 22-23: Reserved */
#define CCM_CCGR6_CG11_MASK             (3 << CCM_CCGR6_CG11_SHIFT)
#  define CCM_CCGR6_CG11(n)             ((uint32_t)(n) << CCM_CCGR6_CG11_SHIFT)
#define CCM_CCGR6_CG12_SHIFT            (24)      /* Bits 24-25: Reserved */
#define CCM_CCGR6_CG12_MASK             (3 << CCM_CCGR6_CG12_SHIFT)
#  define CCM_CCGR6_CG12(n)             ((uint32_t)(n) << CCM_CCGR6_CG12_SHIFT)
#define CCM_CCGR6_CG13_SHIFT            (26)      /* Bits 26-27: Reserved) */
#define CCM_CCGR6_CG13_MASK             (3 << CCM_CCGR6_CG13_SHIFT)
#  define CCM_CCGR6_CG13(n)             ((uint32_t)(n) << CCM_CCGR6_CG13_SHIFT)
#define CCM_CCGR6_CG14_SHIFT            (28)      /* Bits 28-29: Reserved */
#define CCM_CCGR6_CG14_MASK             (3 << CCM_CCGR6_CG14_SHIFT)
#  define CCM_CCGR6_CG14(n)             ((uint32_t)(n) << CCM_CCGR6_CG14_SHIFT)
#define CCM_CCGR6_CG15_SHIFT            (30)      /* Bits 30-31: Reserved */
#define CCM_CCGR6_CG15_MASK             (3 << CCM_CCGR6_CG15_SHIFT)
#  define CCM_CCGR6_CG15(n)             ((uint32_t)(n) << CCM_CCGR6_CG15_SHIFT)

/* CCM Module Enable Overide Register */

#define CCM_CMEOR_MOD_EN_OV_VDOA        (1 << 4)  /* Bit 4:  Overide clock enable signal from vdoa */
#define CCM_CMEOR_MOD_EN_OV_GPT         (1 << 5)  /* Bit 5:  Overide clock enable signal from GPT */
#define CCM_CMEOR_MOD_EN_OV_EPIT        (1 << 6)  /* Bit 6:  Overide clock enable signal from EPIT */
#define CCM_CMEOR_MOD_EN_USDHC          (1 << 7)  /* Bit 7:  Overide clock enable signal from USDHC */
#define CCM_CMEOR_MOD_EN_OV_DAP         (1 << 8)  /* Bit 8:  Overide clock enable signal from DAP */
#define CCM_CMEOR_MOD_EN_OV_VPU         (1 << 9)  /* Bit 9:  Overide clock enable signal from VPU */
#define CCM_CMEOR_MOD_EN_OV_GPU2D       (1 << 10) /* Bit 10: Overide clock enable signal from GPU2D */
#define CCM_CMEOR_MOD_EN_OV_GPU3D       (1 << 11) /* Bit 11: Overide clock enable signal from GPU3D */
#define CCM_CMEOR_MOD_EN_OV_CAN2_CPI    (1 << 28) /* Bit 28: Overide clock enable signal from CAN2 */
#define CCM_CMEOR_MOD_EN_OV_CAN1_CPI    (1 << 30) /* Bit 30: Overide clock enable signal from CAN1 */

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_CCM_H */
