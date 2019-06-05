/************************************************************************************
 * arch/arm/src/sama5/hardware/sam_isi.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_ISI_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_ISI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* ISI Register Offsets *************************************************************/

#define SAM_ISI_CFG1_OFFSET        0x0000 /* ISI Configuration 1 Register */
#define SAM_ISI_CFG2_OFFSET        0x0004 /* ISI Configuration 2 Register */
#define SAM_ISI_PSIZE_OFFSET       0x0008 /* ISI Preview Size Register */
#define SAM_ISI_PDECF_OFFSET       0x000c /* ISI Preview Decimation Factor Register */
#define SAM_ISI_Y2R_SET0_OFFSET    0x0010 /* ISI CSC YCrCb To RGB Set 0 Register */
#define SAM_ISI_Y2R_SET1_OFFSET    0x0014 /* ISI CSC YCrCb To RGB Set 1 Register */
#define SAM_ISI_R2Y_SET0_OFFSET    0x0018 /* ISI CSC RGB To YCrCb Set 0 Register */
#define SAM_ISI_R2Y_SET1_OFFSET    0x001c /* ISI CSC RGB To YCrCb Set 1 Register */
#define SAM_ISI_R2Y_SET2_OFFSET    0x0020 /* ISI CSC RGB To YCrCb Set 2 Register */
#define SAM_ISI_CR_OFFSET          0x0024 /* ISI Control Register */
#define SAM_ISI_SR_OFFSET          0x0028 /* ISI Status Register */
#define SAM_ISI_IER_OFFSET         0x002c /* ISI Interrupt Enable Register */
#define SAM_ISI_IDR_OFFSET         0x0030 /* ISI Interrupt Disable Register */
#define SAM_ISI_IMR_OFFSET         0x0034 /* ISI Interrupt Mask Register */
#define SAM_ISI_DMA_CHER_OFFSET    0x0038 /* DMA Channel Enable Register */
#define SAM_ISI_DMA_CHDR_OFFSET    0x003c /* DMA Channel Disable Register */
#define SAM_ISI_DMA_CHSR_OFFSET    0x0040 /* DMA Channel Status Register */
#define SAM_ISI_DMA_PADDR_OFFSET   0x0044 /* DMA Preview Base Address Register */
#define SAM_ISI_DMA_PCTRL_OFFSET   0x0048 /* DMA Preview Control Register */
#define SAM_ISI_DMA_PDSCR_OFFSET   0x004c /* DMA Preview Descriptor Address Register */
#define SAM_ISI_DMA_CADDR_OFFSET   0x0050 /* DMA Codec Base Address Register */
#define SAM_ISI_DMA_CCTRL_OFFSET   0x0054 /* DMA Codec Control Register */
#define SAM_ISI_DMA_CDSCR_OFFSET   0x0058 /* DMA Codec Descriptor Address Register */
                                          /* 0x005c-0x00e0 Reserved */
#define SAM_ISI_WPMR_OFFSET        0x00e4 /* Write Protection Mode Register */
#define SAM_ISI_WPSR_OFFSET        000xe8 /* Write Protection Status Register */
                                          /* 0x00ec-0x00fc Reserved */

/* ISI Register Addresses ***********************************************************/

#define SAM_ISI_CFG1               (SAM_ISI_VBASE+SAM_ISI_CFG1_OFFSET)
#define SAM_ISI_CFG2               (SAM_ISI_VBASE+SAM_ISI_CFG2_OFFSET)
#define SAM_ISI_PSIZE              (SAM_ISI_VBASE+SAM_ISI_PSIZE_OFFSET)
#define SAM_ISI_PDECF              (SAM_ISI_VBASE+SAM_ISI_PDECF_OFFSET)
#define SAM_ISI_Y2R_SET0           (SAM_ISI_VBASE+SAM_ISI_Y2R_SET0_OFFSET)
#define SAM_ISI_Y2R_SET1           (SAM_ISI_VBASE+SAM_ISI_Y2R_SET1_OFFSET)
#define SAM_ISI_R2Y_SET0           (SAM_ISI_VBASE+SAM_ISI_R2Y_SET0_OFFSET)
#define SAM_ISI_R2Y_SET1           (SAM_ISI_VBASE+SAM_ISI_R2Y_SET1_OFFSET)
#define SAM_ISI_R2Y_SET2           (SAM_ISI_VBASE+SAM_ISI_R2Y_SET2_OFFSET)
#define SAM_ISI_CR                 (SAM_ISI_VBASE+SAM_ISI_CR_OFFSET)
#define SAM_ISI_SR                 (SAM_ISI_VBASE+SAM_ISI_SR_OFFSET)
#define SAM_ISI_IER                (SAM_ISI_VBASE+SAM_ISI_IER_OFFSET)
#define SAM_ISI_IDR                (SAM_ISI_VBASE+SAM_ISI_IDR_OFFSET)
#define SAM_ISI_IMR                (SAM_ISI_VBASE+SAM_ISI_IMR_OFFSET)
#define SAM_ISI_DMA_CHER           (SAM_ISI_VBASE+SAM_ISI_DMA_CHER_OFFSET)
#define SAM_ISI_DMA_CHDR           (SAM_ISI_VBASE+SAM_ISI_DMA_CHDR_OFFSET)
#define SAM_ISI_DMA_CHSR           (SAM_ISI_VBASE+SAM_ISI_DMA_CHSR_OFFSET)
#define SAM_ISI_DMA_PADDR          (SAM_ISI_VBASE+SAM_ISI_DMA_PADDR_OFFSET)
#define SAM_ISI_DMA_PCTRL          (SAM_ISI_VBASE+SAM_ISI_DMA_PCTRL_OFFSET)
#define SAM_ISI_DMA_PDSCR          (SAM_ISI_VBASE+SAM_ISI_DMA_PDSCR_OFFSET)
#define SAM_ISI_DMA_CADDR          (SAM_ISI_VBASE+SAM_ISI_DMA_CADDR_OFFSET)
#define SAM_ISI_DMA_CCTRL          (SAM_ISI_VBASE+SAM_ISI_DMA_CCTRL_OFFSET)
#define SAM_ISI_DMA_CDSCR          (SAM_ISI_VBASE+SAM_ISI_DMA_CDSCR_OFFSET)
#define SAM_ISI_WPMR               (SAM_ISI_VBASE+SAM_ISI_WPMR_OFFSET)
#define SAM_ISI_WPSR               (SAM_ISI_VBASE+SAM_ISI_WPSR_OFFSET)

/* ISI Register Bit Definitions *****************************************************/

/* ISI Configuration 1 Register */

#define ISI_CFG1_HSYNC_POL         (1 << 2)  /* Bit 2:  Horizontal Synchronization Polarity */
#define ISI_CFG1_VSYNC_POL         (1 << 3)  /* Bit 3:  Vertical Synchronization Polarity */
#define ISI_CFG1_PIXCLK_POL        (1 << 4)  /* Bit 4:  Pixel Clock Polarity */
#define ISI_CFG1_EMB_SYNC          (1 << 6)  /* Bit 6:  Embedded Synchronization */
#define ISI_CFG1_CRC_SYNC          (1 << 7)  /* Bit 7:  Embedded Synchronization Correction */
#define ISI_CFG1_FRATE_SHIFT       (8)       /* Bits 8-10: Frame Rate [0..7] */
#define ISI_CFG1_FRATE_MASK        (7 << ISI_CFG1_FRATE_SHIFT)
#  define ISI_CFG1_FRATE(n)        ((uint32_t)(n) << ISI_CFG1_FRATE_SHIFT)
#define ISI_CFG1_DISCR             (1 << 11) /* Bit 11: Disable Codec Request */
#define ISI_CFG1_FULL              (1 << 12) /* Bit 12: Full Mode is Allowed */
#define ISI_CFG1_THMASK_SHIFT      (13)      /* Bits 13-14: Threshold Mask */
#define ISI_CFG1_THMASK_MASK       (3 << ISI_CFG1_THMASK_SHIFT)
#  define ISI_CFG1_THMASK_BEATS4   (0 << ISI_CFG1_THMASK_SHIFT) /* Only 4 beats AHB burst allowed */
#  define ISI_CFG1_THMASK_BEATS8   (1 << ISI_CFG1_THMASK_SHIFT) /* Only 4 and 8 beats AHB burst allowed */
#  define ISI_CFG1_THMASK_BEATS16  (2 << ISI_CFG1_THMASK_SHIFT) /* 4, 8 and 16 beats AHB burst allowed */
#define ISI_CFG1_SLD_SHIFT         (16)      /* Bits 16-23: Start of Line Delay */
#define ISI_CFG1_SLD_MASK          (0xff << ISI_CFG1_SLD_SHIFT)
#  define ISI_CFG1_SLD(n)          ((uint32_t)(n) << ISI_CFG1_SLD_SHIFT)
#define ISI_CFG1_SFD_SHIFT         (24)      /* Bits 24-31: Start of Frame Delay */
#define ISI_CFG1_SFD_MASK          (0xff << ISI_CFG1_SLD_SHIFT)
#  define ISI_CFG1_SFD(n)          ((uint32_t)(n) << ISI_CFG1_SLD_SHIFT)

/* ISI Configuration 2 Register */

#define ISI_CFG2_IMVSIZE_SHIFT     (0)       /* Bits 0-10: Vertical Size of the Image Sensor [0..2047] */
#define ISI_CFG2_IMVSIZE_MASK      (0x7ff << ISI_CFG2_IMVSIZE_SHIFT)
#  define ISI_CFG2_IMVSIZE(n)      ((uint32_t)(n) << ISI_CFG2_IMVSIZE_SHIFT)
#define ISI_CFG2_GS_MODE           (1 << 11) /* Bit 11: 1 (vs 2) pixels per word */
#  define ISI_CFG2_GS_16BPP        (0)       /* Bit 11: 0=2 pixels per word */
#  define ISI_CFG2_GS_32BPP        (1 << 11) /* Bit 11: 1=1 pixels per word */
#define ISI_CFG2_RGB_MODE          (1 << 12) /* Bit 12: RGB Input Mode */
#  define ISI_CFG2_RGB888          (0)       /* Bit 12: 0=RGB 8:8:8 24 bits */
#  define ISI_CFG2_RGB565          (1 << 12) /* Bit 12: 1=RGB 5:6:5 16 bits */
#define ISI_CFG2_GRAYSCALE         (1 << 13) /* Bit 13: Enable grayscale encoding */
#define ISI_CFG2_RGBSWAP           (1 << 14) /* Bit 14: RGB Swap */
#define ISI_CFG2_COLSPACE          (1 << 15) /* Bit 15: Color Space for the Image Data */
#define ISI_CFG2_IMHSIZE_SHIFT     (16)      /* Bits 16-26: Horizontal Size of the Image Sensor */
#define ISI_CFG2_IMHSIZE_MASK      (0x7ff << ISI_CFG2_IMHSIZE_SHIFT)
#  define ISI_CFG2_IMHSIZE(n)      ((uint32_t)(n) << ISI_CFG2_IMHSIZE_SHIFT)
#define ISI_CFG2_YCCSWAP_SHIFT     (28)      /* Bits 28-29: Defines the YCC Image Data */
#define ISI_CFG2_YCCSWAP_MASK      (3 << ISI_CFG2_YCCSWAP_SHIFT)
#  define ISI_CFG2_YCCSWAP_DEFAULT (0 << ISI_CFG2_YCCSWAP_SHIFT) /* Cb(i) Y(i) Cr(i) Y(i+1) */
#  define ISI_CFG2_YCCSWAP_MODE1   (1 << ISI_CFG2_YCCSWAP_SHIFT) /* Cr(i) Y(i) Cb(i) Y(i+1) */
#  define ISI_CFG2_YCCSWAP_MODE2   (2 << ISI_CFG2_YCCSWAP_SHIFT) /* Y(i) Cb(i) Y(i+1) Cr(i) */
#  define ISI_CFG2_YCCSWAP_MODE3   (3 << ISI_CFG2_YCCSWAP_SHIFT) /* Y(i) Cr(i) Y(i+1) Cb(i) */
#define ISI_CFG2_RGBCFG_SHIFT      (30)      /* Bits 30-31: Defines RGB Pattern when RGB_MODE is set to 1 */
#define ISI_CFG2_RGBCFG_MASK       (3 << ISI_CFG2_RGBCFG_SHIFT)
#  define ISI_CFG2_RGBCFG_DEFAULT  (0 << ISI_CFG2_RGBCFG_SHIFT) /* R/G(MSB) G(LSB)/B R/G(MSB) G(LSB)/B */
#  define ISI_CFG2_RGBCFG_MODE1    (1 << ISI_CFG2_RGBCFG_SHIFT) /* B/G(MSB) G(LSB)/R B/G(MSB) G(LSB)/R */
#  define ISI_CFG2_RGBCFG_MODE2    (2 << ISI_CFG2_RGBCFG_SHIFT) /* G(LSB)/R B/G(MSB) G(LSB)/R B/G(MSB) */
#  define ISI_CFG2_RGBCFG_MODE3    (3 << ISI_CFG2_RGBCFG_SHIFT) /* G(LSB)/B R/G(MSB) G(LSB)/B R/G(MSB) */

/* ISI Preview Size Register */

#define ISI_PSIZE_PVSIZE_SHIFT     (0)       /* Bits 0-9: Vertical Size for the Preview Path */
#define ISI_PSIZE_PVSIZE_MASK      (0x3ff << ISI_PSIZE_PVSIZE_SHIFT)
#  define ISI_PSIZE_PVSIZE(n)      ((uint32_t)(n) << ISI_PSIZE_PVSIZE_SHIFT)
#define ISI_PSIZE_PHSIZE_SHIFT     (16)      /* Bits 16-25: Horizontal Size for the Preview Path */
#define ISI_PSIZE_PHSIZE_MASK      (0x3ff << ISI_PSIZE_PHSIZE_SHIFT)
#  define ISI_PSIZE_PHSIZE(n)      ((uint32_t)(n) << ISI_PSIZE_PHSIZE_SHIFT)

/* ISI Preview Decimation Factor Register */

#define ISI_PDECF_MASK             (0xff) /* Bits 0-7: Decimation Factor */

/* ISI CSC YCrCb To RGB Set 0 Register */

#define ISI_Y2R_SET0_C0_SHIFT      (0)      /* Bits 0-7: Color Space Conversion Matrix Coefficient C0 */
#define ISI_Y2R_SET0_C0_MASK       (0xff << ISI_Y2R_SET0_C0_SHIFT)
#  define ISI_Y2R_SET0_C0(n)       ((uint32_t)(n) << ISI_Y2R_SET0_C0_SHIFT)
#define ISI_Y2R_SET0_C1_SHIFT      (8)      /* Bits 8-15: Color Space Conversion Matrix Coefficient C1 */
#define ISI_Y2R_SET0_C1_MASK       (0xff << ISI_Y2R_SET0_C1_SHIFT)
#  define ISI_Y2R_SET0_C1(n)       ((uint32_t)(n) << ISI_Y2R_SET0_C1_SHIFT)
#define ISI_Y2R_SET0_C2_SHIFT      (16)      /* Bits 16-23: Color Space Conversion Matrix Coefficient C2 */
#define ISI_Y2R_SET0_C2_MASK       (0xff << ISI_Y2R_SET0_C2_SHIFT)
#  define ISI_Y2R_SET0_C2(n)       ((uint32_t)(n) << ISI_Y2R_SET0_C2_SHIFT)
#define ISI_Y2R_SET0_C3_SHIFT      (24)      /* Bits 24-31: Color Space Conversion Matrix Coefficient C3 */
#define ISI_Y2R_SET0_C3_MASK       (0xff << ISI_Y2R_SET0_C3_SHIFT)
#  define ISI_Y2R_SET0_C3(n)       ((uint32_t)(n) << ISI_Y2R_SET0_C3_SHIFT)

/* ISI CSC YCrCb To RGB Set 1 Register */

#define ISI_Y2R_SET1_C4_SHIFT      (0)       /* Bits 0-8: Color Space Conversion Matrix Coefficient C4 */
#define ISI_Y2R_SET1_C4_MASK       (0x1ff << ISI_Y2R_SET1_C4_SHIFT)
#  define ISI_Y2R_SET1_C4(n)       ((uint32_t)(n) << ISI_Y2R_SET1_C4_SHIFT)
#define ISI_Y2R_SET1_YOFF          (1 << 12) /* Bit 12: Color Space Conversion Luminance Default Offset */
#define ISI_Y2R_SET1_CROFF         (1 << 13) /* Bit 13: Color Space Conversion Red Chrominance Default Offset */
#define ISI_Y2R_SET1_CBOFF         (1 << 14) /* Bit 14: Color Space Conversion Blue Chrominance Default Offset */

/* ISI CSC RGB To YCrCb Set 0 Register */

#define ISI_R2Y_SET0_C0_SHIFT      (0)       /* Bits 0-6: Color Space Conversion Matrix Coefficient C0 */
#define ISI_R2Y_SET0_C0_MASK       (0x7f << ISI_R2Y_SET0_C0_SHIFT)
#  define ISI_R2Y_SET0_C0(n)       ((uint32_t)(n) << ISI_R2Y_SET0_C0_SHIFT)
#define ISI_R2Y_SET0_C1_SHIFT      (8)       /* Bits 8-14: Color Space Conversion Matrix Coefficient C1 */
#define ISI_R2Y_SET0_C1_MASK       (0x7f << ISI_R2Y_SET0_C1_SHIFT)
#  define ISI_R2Y_SET0_C1(n)       ((uint32_t)(n) << ISI_R2Y_SET0_C1_SHIFT)
#define ISI_R2Y_SET0_C2_SHIFT      (16)      /* Bits 16-22: Color Space Conversion Matrix Coefficient C2 */
#define ISI_R2Y_SET0_C2_MASK       (0x7f << ISI_R2Y_SET0_C2_SHIFT)
#  define ISI_R2Y_SET0_C2(n)       ((uint32_t)(n) << ISI_R2Y_SET0_C2_SHIFT)
#define ISI_R2Y_SET0_ROFF          (1 << 24) /* Bit 24: Color Space Conversion Red Component Offset */

/* ISI CSC RGB To YCrCb Set 1 Register */

#define ISI_R2Y_SET1_C3_SHIFT      (0)       /* Bits 0-6: Color Space Conversion Matrix Coefficient C3 */
#define ISI_R2Y_SET1_C3_MASK       (0x7f << ISI_R2Y_SET1_C3_SHIFT)
#  define ISI_R2Y_SET1_C3(n)       ((uint32_t)(n) << ISI_R2Y_SET1_C3_SHIFT)
#define ISI_R2Y_SET1_C4_SHIFT      (8)       /* Bits 8-14: Color Space Conversion Matrix Coefficient C4 */
#define ISI_R2Y_SET1_C4_MASK       (0x7f << ISI_R2Y_SET1_C4_SHIFT)
#  define ISI_R2Y_SET1_C4(n)       ((uint32_t)(n) << ISI_R2Y_SET1_C4_SHIFT)
#define ISI_R2Y_SET1_C5_SHIFT      (16)      /* Bits 16-22: Color Space Conversion Matrix Coefficient C5 */
#define ISI_R2Y_SET1_C5_MASK       (0x7f << ISI_R2Y_SET1_C5_SHIFT)
#  define ISI_R2Y_SET1_C5(n)       ((uint32_t)(n) << ISI_R2Y_SET1_C5_SHIFT)
#define ISI_R2Y_SET1_GOFF          (1 << 24) /* Bit 24: Color Space Conversion Green Component Offset */

/* ISI CSC RGB To YCrCb Set 2 Register */

#define ISI_R2Y_SET2_C6_SHIFT      (0)       /* Bits 0-6: Color Space Conversion Matrix Coefficient C6 */
#define ISI_R2Y_SET2_C6_MASK       (0x7f << ISI_R2Y_SET2_C6_SHIFT)
#  define ISI_R2Y_SET2_C6(n)       ((uint32_t)(n) << ISI_R2Y_SET2_C6_SHIFT)
#define ISI_R2Y_SET2_C7_SHIFT      (8)       /* Bits 8-14: Color Space Conversion Matrix Coefficient C7 */
#define ISI_R2Y_SET2_C7_MASK       (0x7f << ISI_R2Y_SET2_C7_SHIFT)
#  define ISI_R2Y_SET2_C7(n)       ((uint32_t)(n) << ISI_R2Y_SET2_C7_SHIFT)
#define ISI_R2Y_SET2_C8_SHIFT      (16)      /* Bits 16-22: Color Space Conversion Matrix Coefficient C8 */
#define ISI_R2Y_SET2_C8_MASK       (0x7f << ISI_R2Y_SET2_C8_SHIFT)
#  define ISI_R2Y_SET2_C8(n)       ((uint32_t)(n) << ISI_R2Y_SET2_C8_SHIFT)
#define ISI_R2Y_SET2_BOFF          (1 << 24) /* Bit 24:  Color Space Conversion Blue Component Offset */

/* ISI Control Register */

#define ISI_CR_ISIEN               (1 << 0) /* Bit 0:  ISI Module Enable Request */
#define ISI_CR_ISIDIS              (1 << 1) /* Bit 1:  ISI Module Disable Request */
#define ISI_CR_ISISRST             (1 << 2) /* Bit 2:  ISI Software Reset Request */
#define ISI_CR_ISICDC              (1 << 8) /* Bit 8:  ISI Codec Request */

/* ISI Status Register, ISI Interrupt Enable Register, ISI Interrupt Disable
 * Register, and ISI Interrupt Mask Register
 */

#define ISI_SR_ENABLE              (1 << 0)  /* Bit 0:  Module is enabled (status only) */
#define ISI_INT_DISDONE            (1 << 1)  /* Bit 1:  Module Disable Request has Terminated */
#define ISI_INT_SRST               (1 << 2)  /* Bit 2:  Module Software Reset Request has Terminated */
#define ISI_SR_CDCPND              (1 << 8)  /* Bit 8:  Pending Codec Request (status only) */
#define ISI_INT_VSYNC              (1 << 10) /* Bit 10: Vertical Synchronization */
#define ISI_INT_PXFRDONE           (1 << 16) /* Bit 16: Preview DMA Transfer has Terminated */
#define ISI_INT_CXFRDONE           (1 << 17) /* Bit 17: Codec DMA Transfer has Terminated */
#define ISI_SR_SIP                 (1 << 19) /* Bit 19: Synchronization in Progress (status only) */
#define ISI_INT_POVR               (1 << 24) /* Bit 24: Preview Datapath Overflow */
#define ISI_INT_COVR               (1 << 25) /* Bit 25: Codec Datapath Overflow */
#define ISI_INT_CRCERR             (1 << 26) /* Bit 26: CRC Synchronization Error */
#define ISI_INT_FROVR              (1 << 27) /* Bit 27: Frame Rate Overrun */

/* DMA Channel Enable Register, DMA Channel Disable Register, and DMA Channel Status
 * Register
 */

#define ISI_DMA_PCH                (1 << 0) /* Bit 0:  Preview Channel */
#define ISI_DMA_CCH                (1 << 1) /* Bit 1:  CODEC Channel */

/* DMA Preview Base Address Register */

#define ISI_DMA_PADDR_MASK         (0xfffffffc) /* Bits 2-31: Preview Image Base Address */

/* DMA Preview Control Register */

#define ISI_DMA_PCTRL_PFETCH       (1 << 0)  /* Bit 0:  Descriptor Fetch Control Field */
#define ISI_DMA_PCTRL_PWB          (1 << 1)  /* Bit 1:  Descriptor Writeback Control Field */
#define ISI_DMA_PCTRL_PIEN         (1 << 2)  /* Bit 2:  Transfer Done Flag Control */
#define ISI_DMA_PCTRL_PDONE        (1 << 3)  /* Bit 3:  (This field is only updated in the memory) */

/* DMA Preview Descriptor Address Register */

#define ISI_DMA_PDSCR_MASK         (0xfffffffc) /* Bits 2-31: Preview Descriptor Base Address */

/* DMA Codec Base Address Register */

#define ISI_DMA_CADDR_MASK         (0xfffffffc) /* Bits 2-31: Codec Image Base Address */

/* DMA Codec Control Register */

#define ISI_DMA_CCTRL_CFETCH       (1 << 0)  /* Bit 0:  Descriptor Fetch Control Field */
#define ISI_DMA_CCTRL_CWB          (1 << 1)  /* Bit 1:  Descriptor Writeback Control Field */
#define ISI_DMA_CCTRL_CIEN         (1 << 2)  /* Bit 2:  Transfer Done flag control */
#define ISI_DMA_CCTRL_CDONE        (1 << 3)  /* Bit 3:  (This field is only updated in the memory) */

/* DMA Codec Descriptor Address Register */

#define ISI_DMA_CDSR_MASK          (0xfffffffc) /* Bits 2-31: Codec Descriptor Base Address */

/* Write Protection Mode Register */

#define ISI_WPMR_WPEN              (1 << 0)  /* Bit 0:  Write Protection Enable */
#define ISI_WPMR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protection KEY Password */
#define ISI_WPMR_WPKEY_MASK        (000xffffff << ISI_WPMR_WPKEY_SHIFT)
#  define ISI_WPMR_WPKEY           (0x00495349 << ISI_WPMR_WPKEY_SHIFT) /* (ASCII code for "ISI") */

/* Write Protection Status Register */

#define ISI_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protection Violation Status */
#define ISI_WPSR_WPVSRC_SHIFT      (8)       /* Bits 8-23: Write Protection Violation Source */
#define ISI_WPSR_WPVSRC_MASK       (0xffff << ISI_WPSR_WPVSRC_SHIFT)
#  define ISI_WPSR_WPVSRC_NONE     (0 << ISI_WPSR_WPVSRC_SHIFT) /* No Write Protection Violation */
#  define ISI_WPSR_WPVSRC_CFG1     (1 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_CFG1 */
#  define ISI_WPSR_WPVSRC_CFG2     (2 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_CFG2 */
#  define ISI_WPSR_WPVSRC_PSIZE    (3 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_PSIZE */
#  define ISI_WPSR_WPVSRC_PDECF    (4 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_PDECF */
#  define ISI_WPSR_WPVSRC_Y2R_SET0 (5 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_Y2R_SET0 */
#  define ISI_WPSR_WPVSRC_Y2R_SET1 (6 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_Y2R_SET1 */
#  define ISI_WPSR_WPVSRC_R2Y_SET0 (7 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_R2Y_SET0 */
#  define ISI_WPSR_WPVSRC_R2Y_SET1 (8 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_R2Y_SET1 */
#  define ISI_WPSR_WPVSRC_R2Y_SET2 (9 << ISI_WPSR_WPVSRC_SHIFT) /* Write access in ISI_R2Y_SET2 */

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* "The destination frame buffers are defined by a series of Frame Buffer Descriptors
 *  (FBD). Each FBD controls the transfer of one entire frame and then optionally
 *  loads a further FBD to switch the DMA operation at another frame buffer address.
 *
 * "The FBD is defined by a series of three words. The first one defines the current
 *  frame buffer address (named DMA_X_ADDR register), the second defines control
 *  information (named DMA_X_CTRL register) and the third defines the next descriptor
 *  address (named DMA_X_DSCR). DMA transfer mode with linked list support is
 *  available for both codec and preview datapath."
 */

struct isi_dscr_s
{
  uint32_t addr;  /* Current framebuffer address */
  uint32_t ctrl;  /* Control information */
  uint32_t dscr;  /* Next descriptor address */
};

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_ISI_H */
