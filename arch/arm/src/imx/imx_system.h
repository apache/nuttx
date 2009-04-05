/************************************************************************************
 * arch/arm/src/imx/imx_system.h
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

#ifndef __ARCH_ARM_IMX_SYSTEM_H
#define __ARCH_ARM_IMX_SYSTEM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* AIPI Register Offsets ************************************************************/

#define AIPI_PSR0_OFFSET        0x0000 /* Peripheral Size Register 0 */
#define AIPI_PSR1_OFFSET        0x0004 /* Peripheral Size Register 1 */
#define AIPI_PAR_OFFSET         0x0008 /* Peripheral Access Register */

/* AIPI Register Addresses **********************************************************/

#define IMX_AIPI1_PSR0          (IMX_AIPI1_VBASE + AIPI_PSR0_OFFSET)
#define IMX_AIPI1_PSR1          (IMX_AIPI1_VBASE + AIPI_PSR1_OFFSET)
#define IMX_AIPI1_PAR           (IMX_AIPI1_VBASE + AIPI_PAR_OFFSET)

#define IMX_AIPI2_PSR0          (IMX_AIP2_VBASE + AIPI_PSR0_OFFSET)
#define IMX_AIPI2_PSR1          (IMX_AIP2_VBASE + AIPI_PSR1_OFFSET)
#define IMX_AIPI2_PAR           (IMX_AIP2_VBASE + 0xAIPI_PAR_OFFSET)

/* AIPI Register Bit Definitions ****************************************************/

/* PLL Register Offsets *************************************************************/

#define PLL_CSCR_OFFSET         0x0000 /* Clock Source Control Register */
#define PLL_MCTL0_OFFSET        0x0004 /* MCU PLL Control Register 0 */
#define PLL_MCTL1_OFFSET        0x0008 /* MCU PLL Control Register 1 */
#define PLL_SPCTL0_OFFSET       0x000c /* System PLL Control Register 0 */
#define PLL_SPCTL1_OFFSET       0x0010 /* System PLL Control Register 1 */
#define PLL_PCDR_OFFSET         0x0020 /* Peripherial Clock Divider Register */

/* PLL Register Addresses ***********************************************************/

#define IMX_PLL_CSCR            (IMX_PLL_VBASE + PLL_CSCR_OFFSET)
#define IMX_PLL_MCTL0           (IMX_PLL_VBASE + PLL_MCTL0_OFFSET)
#define IMX_PLL_MCTL1           (IMX_PLL_VBASE + PLL_MCTL1_OFFSET)
#define IMX_PLL_SPCTL0          (IMX_PLL_VBASE + PLL_SPCTL0_OFFSET)
#define IMX_PLL_SPCTL1          (IMX_PLL_VBASE + PLL_SPCTL1_OFFSET)
#define IMX_PLL_PCDR            (IMX_PLL_VBASE + PLL_PCDR_OFFSET)

/* PLL Register Bit Definitions *****************************************************/

/* SC Register Offsets **************************************************************/

#define SC_RSR_OFFSET           0x0000 /* Reset Source Register */
#define SC_SIDR_OFFSET          0x0004 /* Silicon ID Register */
#define SC_FMCR_OFFSET          0x0008 /* Function Muxing Control Register */
#define SC_GPCR_OFFSET          0x000c /* Global Peripherial Control Regiser */

/* SC Register Addresses ************************************************************/

#define IMX_SC_RSR              (IMX_SC_VBASE + SC_RSR_OFFSET)
#define IMX_SC_SIDR             (IMX_SC_VBASE + SC_SIDR_OFFSET)
#define IMX_SC_FMCR             (IMX_SC_VBASE + SC_FMCR_OFFSET)
#define IMX_SC_GPCR             (IMX_SC_VBASE + SC_GPCR_OFFSET)

/* SC Register Bit Definitions ******************************************************/

/* SDRAMC Register Offsets **********************************************************/

#define SDRAMC_SDCTL0_OFFSET    0x0000                
#define SDRAMC_SDCTL1_OFFSET    0x0004              

/* SDRAMC Register Addresses ********************************************************/

#define IMX_SDRAMC_SDCTL0       (IMX_SDRAMC_VBASE + SDRAMC_SDCTL0_OFFSET)                
#define IMX_SDRAMC_SDCTL1       (IMX_SDRAMC_VBASE + SDRAMC_SDCTL1_OFFSET))             

/* SDRAMC Register Bit Definitions **************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_SYSTEM_H */
