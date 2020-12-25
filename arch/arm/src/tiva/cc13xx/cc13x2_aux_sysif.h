/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x2_aux_sysif.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's aux_sysif.h file which has a fully compatible BSD
 * license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13X2_AUX_SYSIF_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13X2_AUX_SYSIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "hardware/tiva_aux_sysif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AUX_SYSIF_OPMODE_TARGET_PDLP AUX_SYSIF_OPMODEREQ_REQ_PDLP
#define AUX_SYSIF_OPMODE_TARGET_PDA  AUX_SYSIF_OPMODEREQ_REQ_PDA
#define AUX_SYSIF_OPMODE_TARGET_LP   AUX_SYSIF_OPMODEREQ_REQ_LP
#define AUX_SYSIF_OPMODE_TARGET_A    AUX_SYSIF_OPMODEREQ_REQ_A

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aux_sysif_opmode
 *
 * Description:
 *
 *   This function controls the change of the AUX operational mode.
 *   The function controls the change of the current operational mode to the
 *   operational mode target by adhering to rules specified by HW.
 *
 * Input Parameters:
 *   - opmode:  AUX operational mode.  One of
 *              AUX_SYSIF_OPMODE_TARGET_PDLP: Power down operational mode
 *                                            with wakeup to low power mode)
 *              AUX_SYSIF_OPMODE_TARGET_PDA:  Power down operational mode
 *                                            with wakeup to active mode
 *              AUX_SYSIF_OPMODE_TARGET_LP:   Low power operational mode)
 *              AUX_SYSIF_OPMODE_TARGET_A:    Active operational mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aux_sysif_opmode(uint32_t opmode);

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13X2_AUX_SYSIF_H */
