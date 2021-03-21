/****************************************************************************
 * arch/arm/include/tms570/tms570ls04x03x_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_TMS570_TMS570LS04X03X_IRQ_H
#define __ARCH_ARM_INCLUDE_TMS570_TMS570LS04X03X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* The interrupt vector table only has 96 entries, one phantom vector and 95
 * interrupt channels.
 * Channel 95 does not have a dedicated vector and shall not be used.
 */

#define TMS570_IRQ_NCHANNELS   95  /* The "phantom" vector is followed by 95 real
                                    * interrupt channels */

/* Default request to channel assignments.  Undefined requests are reserved */

#define TMS570_REQ_ESMHIGH     0  /* ESM High level interrupt (NMI) */
#define TMS570_REQ_RTICMP0     2  /* RTI compare interrupt 0 */
#define TMS570_REQ_RTICMP1     3  /* RTI compare interrupt 1 */
#define TMS570_REQ_RTICMP2     4  /* RTI compare interrupt 2 */
#define TMS570_REQ_RTICMP3     5  /* RTI compare interrupt 3 */
#define TMS570_REQ_RTIOVF0     6  /* RTI overflow interrupt 0 */
#define TMS570_REQ_RTIOVF1     7  /* RTI overflow interrupt 1 */
#define TMS570_REQ_GIO_0       9  /* GIO level 0 interrupt */
#define TMS570_REQ_N2HET_0     10 /* N2HET level 0 interrupt */
#define TMS570_REQ_HTU_0       11 /* HTU level 0 interrupt */
#define TMS570_REQ_MIBSPI1_0   12 /* MIBSPI1 level 0 interrupt */
#define TMS570_REQ_SCI1_0      13 /* SCI1/LIN1 level 0 interrupt */
#define TMS570_REQ_MIBADCEV    14 /* MIBADC event group interrupt */
#define TMS570_REQ_MIBADSW1    15 /* MIBADC sw group 1 interrupt */
#define TMS570_REQ_DCAN1_0     16 /* DCAN1 level 0 interrupt */
#define TMS570_REQ_SPI2_0      17 /* SPI2 level 0 interrupt */
#define TMS570_REQ_ESMLO       20 /* ESM Low level interrupt */
#define TMS570_REQ_SYSTEM      21 /* Software interrupt (SSI) */
#define TMS570_REQ_CPU         22 /* PMU interrupt */
#define TMS570_REQ_GIO_1       23 /* GIO level 1 interrupt */
#define TMS570_REQ_N2HET_1     24 /* N2HET level 1 interrupt */
#define TMS570_REQ_HTU_1       25 /* HTU level 1 interrupt */
#define TMS570_REQ_MIBSPI1_1   26 /* MIBSPI1 level 1 interrupt */
#define TMS570_REQ_SCI1_1      27 /* SCI1/LIN1 level 1 interrupt */
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

#ifdef CONFIG_TMS570_GIO_IRQ
#  define TMS570_IRQ_GIOA0     (TMS570_IRQ_NCHANNELS+1)
#  define TMS570_IRQ_GIOA1     (TMS570_IRQ_NCHANNELS+2)
#  define TMS570_IRQ_GIOA2     (TMS570_IRQ_NCHANNELS+3)
#  define TMS570_IRQ_GIOA3     (TMS570_IRQ_NCHANNELS+4)
#  define TMS570_IRQ_GIOA4     (TMS570_IRQ_NCHANNELS+5)
#  define TMS570_IRQ_GIOA5     (TMS570_IRQ_NCHANNELS+6)
#  define TMS570_IRQ_GIOA6     (TMS570_IRQ_NCHANNELS+7)
#  define TMS570_IRQ_GIOA7     (TMS570_IRQ_NCHANNELS+8)

#  define TMS570_IRQ_GIOB0     (TMS570_IRQ_NCHANNELS+9)
#  define TMS570_IRQ_GIOB1     (TMS570_IRQ_NCHANNELS+10)
#  define TMS570_IRQ_GIOB2     (TMS570_IRQ_NCHANNELS+11)
#  define TMS570_IRQ_GIOB3     (TMS570_IRQ_NCHANNELS+12)
#  define TMS570_IRQ_GIOB4     (TMS570_IRQ_NCHANNELS+13)
#  define TMS570_IRQ_GIOB5     (TMS570_IRQ_NCHANNELS+14)
#  define TMS570_IRQ_GIOB6     (TMS570_IRQ_NCHANNELS+15)
#  define TMS570_IRQ_GIOB7     (TMS570_IRQ_NCHANNELS+16)

#  define TMS570_IRQ_GIOC0     (TMS570_IRQ_NCHANNELS+17)
#  define TMS570_IRQ_GIOC1     (TMS570_IRQ_NCHANNELS+18)
#  define TMS570_IRQ_GIOC2     (TMS570_IRQ_NCHANNELS+19)
#  define TMS570_IRQ_GIOC3     (TMS570_IRQ_NCHANNELS+20)
#  define TMS570_IRQ_GIOC4     (TMS570_IRQ_NCHANNELS+21)
#  define TMS570_IRQ_GIOC5     (TMS570_IRQ_NCHANNELS+22)
#  define TMS570_IRQ_GIOC6     (TMS570_IRQ_NCHANNELS+23)
#  define TMS570_IRQ_GIOC7     (TMS570_IRQ_NCHANNELS+24)

#  define TMS570_IRQ_GIOD0     (TMS570_IRQ_NCHANNELS+25)
#  define TMS570_IRQ_GIOD1     (TMS570_IRQ_NCHANNELS+26)
#  define TMS570_IRQ_GIOD2     (TMS570_IRQ_NCHANNELS+27)
#  define TMS570_IRQ_GIOD3     (TMS570_IRQ_NCHANNELS+28)
#  define TMS570_IRQ_GIOD4     (TMS570_IRQ_NCHANNELS+29)
#  define TMS570_IRQ_GIOD5     (TMS570_IRQ_NCHANNELS+30)
#  define TMS570_IRQ_GIOD6     (TMS570_IRQ_NCHANNELS+31)
#  define TMS570_IRQ_GIOD7     (TMS570_IRQ_NCHANNELS+32)

#  define TMS570_NGIO_IRQS     32
#else
#  define TMS570_NGIO_IRQS     0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_TMS570_TMS570LS04X03X_IRQ_H */
