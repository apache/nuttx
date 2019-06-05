/***********************************************************************************
 * arch/arm/src/sam34/hardware/sam4cm_ipc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ***********************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_IPC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_IPC_H

/***********************************************************************************
 * Included Files
 ***********************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/***********************************************************************************
 * Pre-processor Definitions
 ***********************************************************************************/

/* IPC register offsets ************************************************************/

#define SAM_SLCDC_CR_OFFSET        0x0000 /* Control Register */

#define SAM_IPC_ISCR_OFFSET        0x0000 /* Interrupt Set Command Register */
#define SAM_IPC_ICCR_OFFSET        0x0004 /* Interrupt Clear Command Register */
#define SAM_IPC_IPR_OFFSET         0x0008 /* Interrupt Pending Register */
#define SAM_IPC_IECR_OFFSET        0x000C /* Interrupt Enable Command Register */
#define SAM_IPC_IDCR_OFFSET        0x0010 /* Interrupt Disable Command Register */
#define SAM_IPC_IMR_OFFSET         0x0014 /* Interrupt Mask Register */
#define SAM_IPC_ISR_OFFSET         0x0018 /* Interrupt Status Register */

/* IPC register addresses **********************************************************/

#define SAM_IPC0_ISCR              (SAM_IPC0_BASE + SAM_IPC_ISCR_OFFSET)
#define SAM_IPC0_ICCR              (SAM_IPC0_BASE + SAM_IPC_ICCR_OFFSET)
#define SAM_IPC0_IPR               (SAM_IPC0_BASE + SAM_IPC_IPR_OFFSET)
#define SAM_IPC0_IECR              (SAM_IPC0_BASE + SAM_IPC_IECR_OFFSET)
#define SAM_IPC0_IDCR              (SAM_IPC0_BASE + SAM_IPC_IDCR_OFFSET)
#define SAM_IPC0_IMR               (SAM_IPC0_BASE + SAM_IPC_IMR_OFFSET)
#define SAM_IPC0_ISR               (SAM_IPC0_BASE + SAM_IPC_ISR_OFFSET)

#define SAM_IPC1_ISCR              (SAM_IPC1_BASE + SAM_IPC_ISCR_OFFSET)
#define SAM_IPC1_ICCR              (SAM_IPC1_BASE + SAM_IPC_ICCR_OFFSET)
#define SAM_IPC1_IPR               (SAM_IPC1_BASE + SAM_IPC_IPR_OFFSET)
#define SAM_IPC1_IECR              (SAM_IPC1_BASE + SAM_IPC_IECR_OFFSET)
#define SAM_IPC1_IDCR              (SAM_IPC1_BASE + SAM_IPC_IDCR_OFFSET)
#define SAM_IPC1_IMR               (SAM_IPC1_BASE + SAM_IPC_IMR_OFFSET)
#define SAM_IPC1_ISR               (SAM_IPC1_BASE + SAM_IPC_ISR_OFFSET)

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_IPC_H */
