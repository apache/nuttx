/************************************************************************************
 * arch/arm/src/sama5/sam_can.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_CAN_H
#define __ARCH_ARM_SRC_SAMA5_SAM_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_can.h"

#include <nuttx/can/can.h>

#if defined(CONFIG_CAN) && (defined(CONFIG_SAMA5_CAN0) || defined(CONFIG_SAMA5_CAN1))

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* CAN BAUD */

#if defined(CONFIG_SAMA5_CAN0) && !defined(CONFIG_SAMA5_CAN0_BAUD)
#  error "CONFIG_SAMA5_CAN0_BAUD is not defined"
#endif

#if defined(CONFIG_SAMA5_CAN1) && !defined(CONFIG_SAMA5_CAN1_BAUD)
#  error "CONFIG_SAMA5_CAN1_BAUD is not defined"
#endif

/* There must be at least one but not more than three receive mailboxes */

#ifdef CONFIG_SAMA5_CAN0
#  if !defined(CONFIG_SAMA5_CAN0_NRECVMB) || CONFIG_SAMA5_CAN0_NRECVMB < 1
#    undef  CONFIG_SAMA5_CAN0_NRECVMB
#    define CONFIG_SAMA5_CAN0_NRECVMB 1
#  endif
#  if CONFIG_SAMA5_CAN0_NRECVMB > 3
#    warning Current implementation only supports up to three receive mailboxes
#    undef  CONFIG_SAMA5_CAN0_NRECVMB
#    define CONFIG_SAMA5_CAN0_NRECVMB 3
#  endif
#else
#  undef  CONFIG_SAMA5_CAN0_NRECVMB
#  define CONFIG_SAMA5_CAN0_NRECVMB 0
#endif

#ifdef CONFIG_SAMA5_CAN1
#  if !defined(CONFIG_SAMA5_CAN1_NRECVMB) || CONFIG_SAMA5_CAN1_NRECVMB < 1
#    undef  CONFIG_SAMA5_CAN1_NRECVMB
#    define CONFIG_SAMA5_CAN1_NRECVMB 1
#  endif
#  if CONFIG_SAMA5_CAN1_NRECVMB > 3
#    warning Current implementation only supports up to three receive mailboxes
#    undef  CONFIG_SAMA5_CAN1_NRECVMB
#    define CONFIG_SAMA5_CAN1_NRECVMB 3
#  endif
#else
#  undef  CONFIG_SAMA5_CAN1_NRECVMB
#  define CONFIG_SAMA5_CAN1_NRECVMB 0
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: sama5_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number: 0=CAN0, 1=CAN1
 *
 * Returned Value:
 *   Valid CAN device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
FAR struct can_dev_s *sam_caninitialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_SAMA5_CAN0 || CONFIG_SAMA5_CAN1) */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_CAN_H */
