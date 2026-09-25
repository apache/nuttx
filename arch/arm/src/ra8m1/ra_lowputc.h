/****************************************************************************
 * arch/arm/src/ra8m1/ra_lowputc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RA8M1_RA_LOWPUTC_H
#define __ARCH_ARM_SRC_RA8M1_RA_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.
 *   Performs low level initialization including setup of the console UART.
 *   This UART done early so that the serial console is available for
 *   debugging very early in the boot sequence.
 *
 ****************************************************************************/

void ra_lowsetup(void);

/****************************************************************************
 * Name: ra_sci_baud_ccr2
 *
 * Description:
 *   Find the SCI_B baud rate generator settings closest to the requested
 *   baud rate for the given SCICLK (the baud rate generator input clock,
 *   TCLK, when CCR3.BPEN is 0), following RA8M1 User's Manual Table 31.7
 *   for asynchronous mode.
 *
 * Input Parameters:
 *   sciclk_hz - SCICLK frequency, in Hz.
 *   baud      - Requested baud rate.
 *
 * Returned Value:
 *   The CCR2 bits to set: BGDM/ABCS/ABCSE, BRR and CKS.  All other CCR2
 *   bits are 0.
 *
 ****************************************************************************/

uint32_t ra_sci_baud_ccr2(uint32_t sciclk_hz, uint32_t baud);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8M1_RA_LOWPUTC_H */
