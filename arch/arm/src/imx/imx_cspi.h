/************************************************************************************
 * arch/arm/src/imx/imx_cspi.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_IMX_CSPI_H
#define __ARCH_ARM_IMX_CSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* CSPI Register Offsets ************************************************************/

#define CSPI_SPIRXD_OFFSET         0x0000
#define CSPI_SPITXD_OFFSET         0x0004
#define CSPI_SPICONT1_OFFSET       0x0008
#define CSPI_INTCS_OFFSET          0x000c
#define CSPI_SPITEST_OFFSET        0x0010
#define CSPI_SPISPCR_OFFSET        0x0014
#define CSPI_SPIDMA_OFFSET         0x0018
#define CSPI_SPIRESET_OFFSET       0x001c

/* CSPI Register Addresses **********************************************************/

/* CSPI1 */

#define IMX_CSPI1_SPIRXD           (IMX_CSPI1_VBASE + CSPI_SPIRXD_OFFSET)
#define IMX_CSPI1_SPITXD           (IMX_CSPI1_VBASE + CSPI_SPITXD_OFFSET)
#define IMX_CSPI1_SPICONT1         (IMX_CSPI1_VBASE + CSPI_SPICONT1_OFFSET)
#define IMX_CSPI1_INTCS            (IMX_CSPI1_VBASE + CSPI_INTCS_OFFSET)
#define IMX_CSPI1_SPITEST          (IMX_CSPI1_VBASE + CSPI_SPITEST_OFFSET)
#define IMX_CSPI1_SPISPCR          (IMX_CSPI1_VBASE + CSPI_SPISPCR_OFFSET)
#define IMX_CSPI1_SPIDMA           (IMX_CSPI1_VBASE + CSPI_SPIDMA_OFFSET)
#define IMX_CSPI1_SPIRESET         (IMX_CSPI1_VBASE + CSPI_SPIRESET_OFFSET)

/* CSPI1 */

#define IMX_CSPI2_SPIRXD           (IMX_CSPI2_VBASE + CSPI_SPIRXD_OFFSET)
#define IMX_CSPI2_SPITXD           (IMX_CSPI2_VBASE + CSPI_SPITXD_OFFSET)
#define IMX_CSPI2_SPICONT1         (IMX_CSPI2_VBASE + CSPI_SPICONT1_OFFSET)
#define IMX_CSPI2_INTCS            (IMX_CSPI2_VBASE + CSPI_INTCS_OFFSET)
#define IMX_CSPI2_SPITEST          (IMX_CSPI2_VBASE + CSPI_SPITEST_OFFSET)
#define IMX_CSPI2_SPISPCR          (IMX_CSPI2_VBASE + CSPI_SPISPCR_OFFSET)
#define IMX_CSPI2_SPIDMA           (IMX_CSPI2_VBASE + CSPI_SPIDMA_OFFSET)
#define IMX_CSPI2_SPIRESET         (IMX_CSPI2_VBASE + CSPI_SPIRESET_OFFSET)

/* CSPI Register Bit Definitions ****************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif /* __cplusplus */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* The external functions, imx_spiselect and imx_spistatus must be provided by
 * board-specific logic.  The are implementations of the select and status methods
 * SPI interface defined by struct spi_ops_s (see include/nuttx/spi.h).  All other
 * methods (including up_spiinitialize()) are provided by common logic.  To use this
 * common SPI logic on your board:
 *
 *   1. Provide imx_spiselect() and imx_spistatus() functions in your board-specific 
 *      logic.  This function will perform chip selection and status operations using
 *      GPIOs in the way your board is configured.
 *   2. Add a call to up_spiinitialize() in your low level initialization logic
 *   3. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 */

EXTERN void imx_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected);
EXTERN ubyte imx_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_IMX_CSPI_H */
