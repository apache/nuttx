/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_ic.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_IC_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_IC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx_memorymap.h"

#if CHIP_NIC > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MX_IC_CON_OFFSET      0x0000 /* Input Capture X Control Register */
#define PIC32MX_IC_CONCLR_OFFSET   0x0004 /* Input Capture X Control Set Register */
#define PIC32MX_IC_CONSET_OFFSET   0x0008 /* Input Capture X Control Clear Register */
#define PIC32MX_IC_CONINV_OFFSET   0x000c /* Input Capture X Control Invert Register */
#define PIC32MX_IC_BUF_OFFSET      0x0010 /* Input Capture X Buffer Register */

/* Register Addresses *******************************************************/

#define PIC32MX_IC_CON(n)          (PIC32MX_IC_K1BASE(n)+PIC32MX_IC_CON_OFFSET)
#define PIC32MX_IC_CONCLR(n)       (PIC32MX_IC_K1BASE(n)+PIC32MX_IC_CONCLR_OFFSET)
#define PIC32MX_IC_CONSET(n)       (PIC32MX_IC_K1BASE(n)+PIC32MX_IC_CONSET_OFFSET)
#define PIC32MX_IC_CONINV(n)       (PIC32MX_IC_K1BASE(n)+PIC32MX_IC_CONINV_OFFSET)
#define PIC32MX_IC_BUF(n)          (PIC32MX_IC_K1BASE(n)+PIC32MX_IC_BUF_OFFSET)

#define PIC32MX_IC1_CON            (PIC32MX_IC1_K1BASE+PIC32MX_IC_CON_OFFSET)
#define PIC32MX_IC1_CONCLR         (PIC32MX_IC1_K1BASE+PIC32MX_IC_CONCLR_OFFSET)
#define PIC32MX_IC1_CONSET         (PIC32MX_IC1_K1BASE+PIC32MX_IC_CONSET_OFFSET)
#define PIC32MX_IC1_CONINV         (PIC32MX_IC1_K1BASE+PIC32MX_IC_CONINV_OFFSET)
#define PIC32MX_IC1_BUF            (PIC32MX_IC1_K1BASE+PIC32MX_IC_BUF_OFFSET)

#if CHIP_NIC > 1
#  define PIC32MX_IC2_CON          (PIC32MX_IC2_K1BASE+PIC32MX_IC_CON_OFFSET)
#  define PIC32MX_IC2_CONCLR       (PIC32MX_IC2_K1BASE+PIC32MX_IC_CONCLR_OFFSET)
#  define PIC32MX_IC2_CONSET       (PIC32MX_IC2_K1BASE+PIC32MX_IC_CONSET_OFFSET)
#  define PIC32MX_IC2_CONINV       (PIC32MX_IC2_K1BASE+PIC32MX_IC_CONINV_OFFSET)
#  define PIC32MX_IC2_BUF          (PIC32MX_IC2_K1BASE+PIC32MX_IC_BUF_OFFSET)
#endif

#if CHIP_NIC > 2
#  define PIC32MX_IC3_CON          (PIC32MX_IC3_K1BASE+PIC32MX_IC_CON_OFFSET)
#  define PIC32MX_IC3_CONCLR       (PIC32MX_IC3_K1BASE+PIC32MX_IC_CONCLR_OFFSET)
#  define PIC32MX_IC3_CONSET       (PIC32MX_IC3_K1BASE+PIC32MX_IC_CONSET_OFFSET)
#  define PIC32MX_IC3_CONINV       (PIC32MX_IC3_K1BASE+PIC32MX_IC_CONINV_OFFSET)
#  define PIC32MX_IC3_BUF          (PIC32MX_IC3_K1BASE+PIC32MX_IC_BUF_OFFSET)
#endif

#if CHIP_NIC > 3
#  define PIC32MX_IC4_CON          (PIC32MX_IC4_K1BASE+PIC32MX_IC_CON_OFFSET)
#  define PIC32MX_IC4_CONCLR       (PIC32MX_IC4_K1BASE+PIC32MX_IC_CONCLR_OFFSET)
#  define PIC32MX_IC4_CONSET       (PIC32MX_IC4_K1BASE+PIC32MX_IC_CONSET_OFFSET)
#  define PIC32MX_IC4_CONINV       (PIC32MX_IC4_K1BASE+PIC32MX_IC_CONINV_OFFSET)
#  define PIC32MX_IC4_BUF          (PIC32MX_IC4_K1BASE+PIC32MX_IC_BUF_OFFSET)
#endif

#if CHIP_NIC > 4
#  define PIC32MX_IC5_CON          (PIC32MX_IC5_K1BASE+PIC32MX_IC_CON_OFFSET)
#  define PIC32MX_IC5_CONCLR       (PIC32MX_IC5_K1BASE+PIC32MX_IC_CONCLR_OFFSET)
#  define PIC32MX_IC5_CONSET       (PIC32MX_IC5_K1BASE+PIC32MX_IC_CONSET_OFFSET)
#  define PIC32MX_IC5_CONINV       (PIC32MX_IC5_K1BASE+PIC32MX_IC_CONINV_OFFSET)
#  define PIC32MX_IC5_BUF          (PIC32MX_IC5_K1BASE+PIC32MX_IC_BUF_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************/

/* Input Capture X Control Register */

#define IC_CON_ICM_SHIFT           (0)     /* Bits 0-2: Input Capture Mode Select */
#define IC_CON_ICM_MASK            (7 << IC_CON_ICM_SHIFT)
#  define IC_CON_ICM_DISABLE       (0 << IC_CON_ICM_SHIFT) /* Capture disable mode */
#  define IC_CON_ICM_EDGE          (1 << IC_CON_ICM_SHIFT) /* Edge detect mode */
#  define IC_CON_ICM_FALLING       (2 << IC_CON_ICM_SHIFT) /* Every falling edge */
#  define IC_CON_ICM_RISING        (3 << IC_CON_ICM_SHIFT) /* Every rising edge */
#  define IC_CON_ICM_4th           (4 << IC_CON_ICM_SHIFT) /* Every fourth rising edge */
#  define IC_CON_ICM_16th          (5 << IC_CON_ICM_SHIFT) /* Every sixteenth rising edge */
#  define IC_CON_ICM_TRIGGER       (6 << IC_CON_ICM_SHIFT) /* Specified edge first and every edge thereafter */
#  define IC_CON_ICM_INTERRUPT     (7 << IC_CON_ICM_SHIFT) /* Interrupt-only mode */

#define IC_CON_ICBNE               (1 << 3)  /* Bit 3:  Input Capture Buffer Not Empty Status */
#define IC_CON_ICOV                (1 << 4)  /* Bit 4:  Input Capture */
#define IC_CON_ICI_SHIFT           (5)       /* Bits 5-6: Interrupt Control */
#define IC_CON_ICI_MASK            (3 << IC_CON_ICI_SHIFT)
#  define IC_CON_ICI_EVERY         (0 << IC_CON_ICI_SHIFT) /* Interrupt every capture event */
#  define IC_CON_ICI_2ND           (1 << IC_CON_ICI_SHIFT) /* Interrupt every 2nd capture event */
#  define IC_CON_ICI_3RD           (2 << IC_CON_ICI_SHIFT) /* Interrupt every 3rd capture event */
#  define IC_CON_ICI_4TH           (3 << IC_CON_ICI_SHIFT) /* Interrupt every 4th capture event */

#define IC_CON_ICTMR               (1 << 7)  /* Bit 7:  Timer Select */
#define IC_CON_C32                 (1 << 8)  /* Bit 8:  32-bit Capture Select */
#define IC_CON_FEDGE               (1 << 9)  /* Bit 9:  First Capture Edge Select */
#define IC_CON_SIDL                (1 << 13) /* Bit 13: Stop in Idle Control */
#define IC_CON_FRZ                 (1 << 14) /* Bit 14: Freeze in Debug Mode Control */
#define IC_CON_ON                  (1 << 15) /* Bit 15: Input Capture Module Enable */

/* Input Capture X Buffer Register -- 32-bit capture value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CHIP_NIC > 0 */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_IC_H */
