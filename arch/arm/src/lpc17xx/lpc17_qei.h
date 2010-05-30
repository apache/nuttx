/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_qei.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_QEI_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_QEI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Control registers */

#define LPC17_QEI_CON_OFFSET     0x0000 /* Control register */
#define LPC17_QEI_STAT_OFFSET    0x0004 /* Encoder status register */
#define LPC17_QEI_CONF_OFFSET    0x0008 /* Configuration register */

/* Position, index, and timer registers */

#define LPC17_QEI_POS_OFFSET     0x000c /* Position register */
#define LPC17_QEI_MAXPOS_OFFSET  0x0010 /* Maximum position register */
#define LPC17_QEI_CMPOS0_OFFSET  0x0014 /* Position compare register */
#define LPC17_QEI_CMPOS1_OFFSET  0x0018 /* Position compare register */
#define LPC17_QEI_CMPOS2_OFFSET  0x001c /* Position compare register */
#define LPC17_QEI_INXCNT_OFFSET  0x0020 /* Index count register */
#define LPC17_QEI_INXCMP_OFFSET  0x0024 /* Index compare register */
#define LPC17_QEI_LOAD_OFFSET    0x0028 /* Velocity timer reload register */
#define LPC17_QEI_TIME_OFFSET    0x002c /* Velocity timer register */
#define LPC17_QEI_VEL_OFFSET     0x0030 /* Velocity counter register */
#define LPC17_QEI_CAP_OFFSET     0x0034 /* Velocity capture register */
#define LPC17_QEI_VELCOMP_OFFSET 0x0038 /* Velocity compare register */
#define LPC17_QEI_FILTER_OFFSET  0x003c /* Digital filter register */

/* Interrupt registers */

#define LPC17_QEI_IEC_OFFSET     0x0fd8 /* Interrupt enable clear register */
#define LPC17_QEI_IES_OFFSET     0x0fdc /* Interrupt enable set register */
#define LPC17_QEI_INTSTAT_OFFSET 0x0fe0 /* Interrupt status register */
#define LPC17_QEI_IE_OFFSET      0x0fe4 /* Interrupt enable register */
#define LPC17_QEI_CLR_OFFSET     0x0fe8 /* Interrupt status clear register */
#define LPC17_QEI_SET_OFFSET     0x0fec /* Interrupt status set register */

/* Register addresses ***************************************************************/
/* Control registers */

#define LPC17_QEI_CON            (LPC17_QEI_BASE+LPC17_QEI_CON_OFFSET)
#define LPC17_QEI_STAT           (LPC17_QEI_BASE+LPC17_QEI_STAT_OFFSET)
#define LPC17_QEI_CONF           (LPC17_QEI_BASE+LPC17_QEI_CONF_OFFSET)

/* Position, index, and timer registers */

#define LPC17_QEI_POS            (LPC17_QEI_BASE+LPC17_QEI_POS_OFFSET)
#define LPC17_QEI_MAXPOS         (LPC17_QEI_BASE+LPC17_QEI_MAXPOS_OFFSET)
#define LPC17_QEI_CMPOS0         (LPC17_QEI_BASE+LPC17_QEI_CMPOS0_OFFSET)
#define LPC17_QEI_CMPOS1         (LPC17_QEI_BASE+LPC17_QEI_CMPOS1_OFFSET)
#define LPC17_QEI_CMPOS2         (LPC17_QEI_BASE+LPC17_QEI_CMPOS2_OFFSET)
#define LPC17_QEI_INXCNT         (LPC17_QEI_BASE+LPC17_QEI_INXCNT_OFFSET)
#define LPC17_QEI_INXCMP         (LPC17_QEI_BASE+LPC17_QEI_INXCMP_OFFSET)
#define LPC17_QEI_LOAD           (LPC17_QEI_BASE+LPC17_QEI_LOAD_OFFSET)
#define LPC17_QEI_TIME           (LPC17_QEI_BASE+LPC17_QEI_TIME_OFFSET)
#define LPC17_QEI_VEL            (LPC17_QEI_BASE+LPC17_QEI_VEL_OFFSET)
#define LPC17_QEI_CAP            (LPC17_QEI_BASE+LPC17_QEI_CAP_OFFSET)
#define LPC17_QEI_VELCOMP        (LPC17_QEI_BASE+LPC17_QEI_VELCOMP_OFFSET)
#define LPC17_QEI_FILTER         (LPC17_QEI_BASE+LPC17_QEI_FILTER_OFFSET)

/* Interrupt registers */

#define LPC17_QEI_IEC            (LPC17_QEI_BASE+LPC17_QEI_IEC_OFFSET)
#define LPC17_QEI_IES            (LPC17_QEI_BASE+LPC17_QEI_IES_OFFSET)
#define LPC17_QEI_INTSTAT        (LPC17_QEI_BASE+LPC17_QEI_INTSTAT_OFFSET)
#define LPC17_QEI_IE             (LPC17_QEI_BASE+LPC17_QEI_IE_OFFSET)
#define LPC17_QEI_CLR            (LPC17_QEI_BASE+LPC17_QEI_CLR_OFFSET)
#define LPC17_QEI_SET            (LPC17_QEI_BASE+LPC17_QEI_SET_OFFSET)

/* Register bit definitions *********************************************************/
/* Control registers */

/* Control register */

#define QEI_CON_

/* Encoder status register */

#define QEI_STAT_

/* Configuration register */

#define QEI_CONF_

/* Position, index, and timer registers */
/* Position register */

#define QEI_POS_

/* Maximum position register */

#define QEI_MAXPOS_

/* Position compare register */

#define QEI_CMPOS0_

/* Position compare register */

#define QEI_CMPOS1_

/* Position compare register */

#define QEI_CMPOS2_

/* Index count register */

#define QEI_INXCNT_

/* Index compare register */

#define QEI_INXCMP_

/* Velocity timer reload register */

#define QEI_LOAD_

/* Velocity timer register */

#define QEI_TIME_

/* Velocity counter register */

#define QEI_VEL_

/* Velocity capture register */

#define QEI_CAP_

/* Velocity compare register */

#define QEI_VELCOMP_

/* Digital filter register */

#define QEI_FILTER_

/* Interrupt registers */
/* Interrupt enable clear register */

#define QEI_IEC_

/* Interrupt enable set register */

#define QEI_IES_

/* Interrupt status register */

#define QEI_INTSTAT_

/* Interrupt enable register */

#define QEI_IE_

/* Interrupt status clear register */

#define QEI_CLR_

/* Interrupt status set register */

#define QEI_SET_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_QEI_H */
