/****************************************************************************************
 * arch/arm/include/tms570/tms570ls04x03x_irq.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_TMS570_TMS570LS04X03X_IRQ_H
#define __ARCH_ARM_INCLUDE_TMS570_TMS570LS04X03X_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/
/* The interrupt vector table only has 96 entries, one phantom vector and 95 interrupt
 * channels. Channel 95 does not have a dedicated vector and shall not be used.
 */

#define TMS570_IRQ_NCHANNELS   95  /* The "phantom" vector is followed by 95 real
                                    * interrupt channels */

/* Default request to channel assignments.  Undefined requests are reserved */

#define TMS570_REQ_ESMHIGH     0 /* ESM High level interrupt (NMI) */
#define TMS570_REQ_RTICMP0     2 /* RTI compare interrupt 0 */
#define TMS570_REQ_RTICMP1     3 /* RTI compare interrupt 1 */
#define TMS570_REQ_RTICMP2     4 /* RTI compare interrupt 2 */
#define TMS570_REQ_RTICMP3     5 /* RTI compare interrupt 3 */
#define TMS570_REQ_RTIOVF0     6 /* RTI overflow interrupt 0 */
#define TMS570_REQ_RTIOVF1     7 /* RTI overflow interrupt 1 */
#define TMS570_REQ_GIOA        9 /* GIO interrupt A */
#define TMS570_REQ_N2HET_0     10 /* N2HET level 0 interrupt */
#define TMS570_REQ_HTU_0       11 /* HTU level 0 interrupt */
#define TMS570_REQ_MIBSPI1_0   12 /* MIBSPI1 level 0 interrupt */
#define TMS570_REQ_SCI_0       13 /* SCI/LIN level 0 interrupt */
#define TMS570_REQ_MIBADCEV    14 /* MIBADC event group interrupt */
#define TMS570_REQ_MIBADSW1    15 /* MIBADC sw group 1 interrupt */
#define TMS570_REQ_DCAN1_0     16 /* DCAN1 level 0 interrupt */
#define TMS570_REQ_SPI2_0      17 /* SPI2 level 0 interrupt */
#define TMS570_REQ_ESMLO       20 /* ESM Low level interrupt */
#define TMS570_REQ_SYSTEM      21 /* Software interrupt (SSI) */
#define TMS570_REQ_CPU         22 /* PMU interrupt */
#define TMS570_REQ_GIOB        23 /* GIO interrupt B */
#define TMS570_REQ_N2HET_1     24 /* N2HET level 1 interrupt */
#define TMS570_REQ_HTU_1       25 /* HTU level 1 interrupt */
#define TMS570_REQ_MIBSPI1_1   26 /* MIBSPI1 level 1 interrupt */
#define TMS570_REQ_SCI_1       27 /* SCI/LIN level 1 interrupt */
#define TMS570_REQ_MIBADCSW2   28 /* MIBADC sw group 2 interrupt */
#define TMS570_REQ_DCAN1_1     29 /* DCAN1 level 1 interrupt */
#define TMS570_REQ_SPI2_1      30 /* SPI2 level 1 interrupt */
#define TMS570_REQ_MIBADCMC    31 /* MIBADC magnitude compare interrupt */
#define TMS570_REQ_DCAN2_0     35 /* DCAN2 level 0 interrupt */
#define TMS570_REQ_SPI3_0      37 /* SPI3 level 0 interrupt */
#define TMS570_REQ_SPI3_1      38 /* SPI3 level 1 interrupt */
#define TMS570_REQ_DCAN2_1     42 /* DCAN2 level 1 interrupt */
#define TMS570_REQ_FMC         61 /* FSM_DONE interrupt */
#define TMS570_REQ_HWAGH       80 /* WA_INT_REQ_H */
#define TMS570_REQ_DCC         82 /* DCC done interrupt */
#define TMS570_REQ_EQEP        84 /* eQEP Interrupt */
#define TMS570_REQ_PBIST       85 /* PBIST Done Interrupt */
#define TMS570_REQ_HWAGL       88 /* HWA_INT_REQ_L */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_TMS570_TMS570LS04X03X_IRQ_H */

