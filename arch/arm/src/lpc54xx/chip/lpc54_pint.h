/****************************************************************************************************
 * arch/arm/src/lpc54xx/chip/lpc54_pint.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_PINT_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_PINT_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/

#define LPC54_PINT_ISEL_OFFSET    0x0000 /* Pin interrupt mode */
#define LPC54_PINT_IENR_OFFSET    0x0004 /* Pin interrupt level or rising edge interrupt enable */
#define LPC54_PINT_SIENR_OFFSET   0x0008 /* Pin interrupt level or rising edge interrupt enable set */
#define LPC54_PINT_CIENR_OFFSET   0x000c /* Pin interrupt level or rising edge interrupt enable clear */
#define LPC54_PINT_IENF_OFFSET    0x0010 /* Pin interrupt active level or falling edge interrupt enable */
#define LPC54_PINT_SIENF_OFFSET   0x0014 /* Pin interrupt active level or falling edge interrupt set */
#define LPC54_PINT_CIENF_OFFSET   0x0018 /* Pin interrupt active level or falling edge interrupt clear */
#define LPC54_PINT_RISE_OFFSET    0x001c /* Pin interrupt rising edge */
#define LPC54_PINT_FALL_OFFSET    0x0020 /* Pin interrupt falling edge */
#define LPC54_PINT_IST_OFFSET     0x0024 /* Pin interrupt status */
#define LPC54_PINT_PMCTRL_OFFSET  0x0028 /* Pattern match interrupt control */
#define LPC54_PINT_PMSRC_OFFSET   0x002c /* Pattern match interrupt bit-slice source */
#define LPC54_PINT_PMCFG_OFFSET   0x0030 /* Pattern match interrupt bit slice configuration */

/* Register addresses *******************************************************************************/

#define LPC54_PINT_ISEL           (LPC54_PINT_BASE + LPC54_PINT_ISEL_OFFSET)
#define LPC54_PINT_IENR           (LPC54_PINT_BASE + LPC54_PINT_IENR_OFFSET)
#define LPC54_PINT_SIENR          (LPC54_PINT_BASE + LPC54_PINT_SIENR_OFFSET)
#define LPC54_PINT_CIENR          (LPC54_PINT_BASE + LPC54_PINT_CIENR_OFFSET)
#define LPC54_PINT_IENF           (LPC54_PINT_BASE + LPC54_PINT_IENF_OFFSET)
#define LPC54_PINT_SIENF          (LPC54_PINT_BASE + LPC54_PINT_SIENF_OFFSET)
#define LPC54_PINT_CIENF          (LPC54_PINT_BASE + LPC54_PINT_CIENF_OFFSET)
#define LPC54_PINT_RISE           (LPC54_PINT_BASE + LPC54_PINT_RISE_OFFSET)
#define LPC54_PINT_FALL           (LPC54_PINT_BASE + LPC54_PINT_FALL_OFFSET)
#define LPC54_PINT_IST            (LPC54_PINT_BASE + LPC54_PINT_IST_OFFSET)
#define LPC54_PINT_PMCTRL         (LPC54_PINT_BASE + LPC54_PINT_PMCTRL_OFFSET)
#define LPC54_PINT_PMSRC          (LPC54_PINT_BASE + LPC54_PINT_PMSRC_OFFSET)
#define LPC54_PINT_PMCFG          (LPC54_PINT_BASE + LPC54_PINT_PMCFG_OFFSET)

/* Register bit definitions *************************************************************************/

/* Pin interrupt mode */
#define PINT_ISEL_
/* Pin interrupt level or rising edge interrupt enable */
#define PINT_IENR_
/* Pin interrupt level or rising edge interrupt enable set */
#define PINT_SIENR_
/* Pin interrupt level or rising edge interrupt enable clear */
#define PINT_CIENR_
/* Pin interrupt active level or falling edge interrupt enable */
#define PINT_IENF_
/* Pin interrupt active level or falling edge interrupt set */
#define PINT_SIENF_
/* Pin interrupt active level or falling edge interrupt clear */
#define PINT_CIENF_
/* Pin interrupt rising edge */
#define PINT_RISE_
/* Pin interrupt falling edge */
#define PINT_FALL_
/* Pin interrupt status */
#define PINT_IST_
/* Pattern match interrupt control */
#define PINT_PMCTRL_
/* Pattern match interrupt bit-slice source */
#define PINT_PMSRC_
/* Pattern match interrupt bit slice configuration */
#define PINT_PMCFG_

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_PINT_H */
