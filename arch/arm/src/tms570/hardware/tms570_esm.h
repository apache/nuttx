/****************************************************************************
 * arch/arm/src/tms570/hardware/tms570_esm.h
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

/* References:
 * TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller,
 * Technical Reference Manual, Texas Instruments,
 * Literature Number: SPNU517A, September 2013
 */

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_ESM_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_ESM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESM_GROUP1                0
#define ESM_GROUP2                1
#define ESM_GROUP3                2

/* Register Offsets *********************************************************/

#define TMS570_ESM_EEPAPR1_OFFSET 0x0000 /* ESM Enable ERROR Pin Action/Response Register 1 */
#define TMS570_ESM_DEPAPR1_OFFSET 0x0004 /* ESM Disable ERROR Pin Action/Response Register 1 */
#define TMS570_ESM_IESR1_OFFSET   0x0008 /* ESM Interrupt Enable Set/Status Register 1 */
#define TMS570_ESM_IECR1_OFFSET   0x000c /* ESM Interrupt Enable Clear/Status Register 1 */
#define TMS570_ESM_ILSR1_OFFSET   0x0010 /* Interrupt Level Set/Status Register 1 */
#define TMS570_ESM_ILCR1_OFFSET   0x0014 /* Interrupt Level Clear/Status Register 1 */

#define TMS570_ESM_SRA_OFFSET(n)  (0x0018 + ((n) << 2)) /* n=0, 1, 2 */

#  define TMS570_ESM_SR1_OFFSET   0x0018 /* ESM Status Register for group 1 */
#  define TMS570_ESM_SR2_OFFSET   0x001c /* ESM Status Register for group 2 */
#  define TMS570_ESM_SR3_OFFSET   0x0020 /* ESM Status Register for group 3 */
#define TMS570_ESM_EPSR_OFFSET    0x0024 /* ESM ERROR Pin Status Register */
#define TMS570_ESM_IOFFHR_OFFSET  0x0028 /* ESM Interrupt Offset High Register */
#define TMS570_ESM_IOFFLR_OFFSET  0x002c /* ESM Interrupt Offset Low Register */
#define TMS570_ESM_LTCR_OFFSET    0x0030 /* ESM Low-Time Counter Register Section */
#define TMS570_ESM_LTCPR_OFFSET   0x0034 /* ESM Low-Time Counter Preload Register */
#define TMS570_ESM_EKR_OFFSET     0x0038 /* ESM Error Key Register */
#define TMS570_ESM_SSR2_OFFSET    0x003c /* ESM Status Shadow Register 2 */
#define TMS570_ESM_IEPSR4_OFFSET  0x0040 /* ESM Influence ERROR Pin Set/Status Register 4 */
#define TMS570_ESM_IEPCR4_OFFSET  0x0044 /* ESM Influence ERROR Pin Clear/Status Register 4 */
#define TMS570_ESM_IESR4_OFFSET   0x0048 /* ESM Interrupt Enable Set/Status Register 4 */
#define TMS570_ESM_IECR4_OFFSET   0x004c /* ESM Interrupt Enable Clear/Status Register 4 */
#define TMS570_ESM_ILSR4_OFFSET   0x0050 /* Interrupt Level Set/Status Register 4 */
#define TMS570_ESM_ILCR4_OFFSET   0x0054 /* Interrupt Level Clear/Status Register 4 */
#define TMS570_ESM_SR4_OFFSET     0x0058 /* ESM Status Register 4 */

#define TMS570_ESM_SRB_OFFSET(n)  (0x0058 + (((n)-4) << 2)) /* n=4, 5, 6 */

#  define TMS570_ESM_SR4_OFFSET   0x0058 /* ESM Status Register for group 4 */
#  define TMS570_ESM_SR5_OFFSET   0x005c /* ESM Status Register for group 5 */
#  define TMS570_ESM_SR6_OFFSET   0x0060 /* ESM Status Register for group 6 */
#define TMS570_ESM_SSR5_OFFSET    0x0064 /* ESM Status Shadow Register 5 */

/* Register Addresses *******************************************************/

#define TMS570_ESM_EEPAPR1        (TMS570_ESM_BASE+TMS570_ESM_EEPAPR1_OFFSET)
#define TMS570_ESM_DEPAPR1        (TMS570_ESM_BASE+TMS570_ESM_DEPAPR1_OFFSET)
#define TMS570_ESM_IESR1          (TMS570_ESM_BASE+TMS570_ESM_IESR1_OFFSET)
#define TMS570_ESM_IECR1          (TMS570_ESM_BASE+TMS570_ESM_IECR1_OFFSET)
#define TMS570_ESM_ILSR1          (TMS570_ESM_BASE+TMS570_ESM_ILSR1_OFFSET)
#define TMS570_ESM_ILCR1          (TMS570_ESM_BASE+TMS570_ESM_ILCR1_OFFSET)
#define TMS570_ESM_SRA(n)         (TMS570_ESM_BASE+TMS570_ESM_SRA_OFFSET(n))
#  define TMS570_ESM_SR1          (TMS570_ESM_BASE+TMS570_ESM_SR1_OFFSET)
#  define TMS570_ESM_SR2          (TMS570_ESM_BASE+TMS570_ESM_SR2_OFFSET)
#  define TMS570_ESM_SR3          (TMS570_ESM_BASE+TMS570_ESM_SR3_OFFSET)
#define TMS570_ESM_EPSR           (TMS570_ESM_BASE+TMS570_ESM_EPSR_OFFSET)
#define TMS570_ESM_IOFFHR         (TMS570_ESM_BASE+TMS570_ESM_IOFFHR_OFFSET)
#define TMS570_ESM_IOFFLR         (TMS570_ESM_BASE+TMS570_ESM_IOFFLR_OFFSET)
#define TMS570_ESM_LTCR           (TMS570_ESM_BASE+TMS570_ESM_LTCR_OFFSET)
#define TMS570_ESM_LTCPR          (TMS570_ESM_BASE+TMS570_ESM_LTCPR_OFFSET)
#define TMS570_ESM_EKR            (TMS570_ESM_BASE+TMS570_ESM_EKR_OFFSET)
#define TMS570_ESM_SSR2           (TMS570_ESM_BASE+TMS570_ESM_SSR2_OFFSET)
#define TMS570_ESM_IEPSR4         (TMS570_ESM_BASE+TMS570_ESM_IEPSR4_OFFSET)
#define TMS570_ESM_IEPCR4         (TMS570_ESM_BASE+TMS570_ESM_IEPCR4_OFFSET)
#define TMS570_ESM_IESR4          (TMS570_ESM_BASE+TMS570_ESM_IESR4_OFFSET)
#define TMS570_ESM_IECR4          (TMS570_ESM_BASE+TMS570_ESM_IECR4_OFFSET)
#define TMS570_ESM_ILSR4          (TMS570_ESM_BASE+TMS570_ESM_ILSR4_OFFSET)
#define TMS570_ESM_ILCR4          (TMS570_ESM_BASE+TMS570_ESM_ILCR4_OFFSET)
#define TMS570_ESM_SR4            (TMS570_ESM_BASE+TMS570_ESM_SR4_OFFSET)
#define TMS570_ESM_SRB(n)         (TMS570_ESM_BASE+TMS570_ESM_SRB_OFFSET(n))
#  define TMS570_ESM_SR4          (TMS570_ESM_BASE+TMS570_ESM_SR4_OFFSET)
#  define TMS570_ESM_SR5          (TMS570_ESM_BASE+TMS570_ESM_SR5_OFFSET)
#  define TMS570_ESM_SR6          (TMS570_ESM_BASE+TMS570_ESM_SR6_OFFSET)
#define TMS570_ESM_SSR5           (TMS570_ESM_BASE+TMS570_ESM_SSR5_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* ESM Enable ERROR Pin Action/Response Register 1 */

#define ESM_EEPAPR1_CHAN(n)       (1 << (n)) /* Bit n: Channel n enable */

/* ESM Disable ERROR Pin Action/Response Register 1 */

#define ESM_DEPAPR1_CHAN(n)       (1 << (n)) /* Bit n: Channel n disable */

/* ESM Interrupt Enable Set/Status Register 1 */

#define ESM_IESR1_INT(n)          (1 << (n)) /* Bit n: Channel n interrupt enable */

/* ESM Interrupt Enable Clear/Status Register 1 */

#define ESM_IECR1_INT(n)          (1 << (n)) /* Bit n: Channel n interrupt clear */

/* Interrupt Level Set/Status Register 1 */

#define ESM_ILSR1_INT(n)          (1 << (n)) /* Bit n: Channel n interrupt level high */

/* Interrupt Level Clear/Status Register 1 */

#define ESM_ILCR1_INT(n)          (1 << (n)) /* Bit n: Channel n interrupt level low (write) */

/* ESM Status Register for groups 1, 2, 3, and 4 */

#define ESM_SR_PENDING(n)         (1 << (n)) /* Bit n: Channel n error interrupt pending (read) */
#define ESM_SR_CLEAR(n)           (1 << (n)) /* Bit n: Channel n error interrupt clear (write) */

/* ESM ERROR Pin Status Register */

#define ESM_EPSR_NOERROR(n)       (1 << (n)) /* Bit n: Channel n no error (read) */

/* ESM Interrupt Offset High Register */

#define ESM_IOFFHR_MASK           0x0000007f /* Bits 0-6: Interrupt offset high level */

/* ESM Interrupt Offset Low Register */

#define ESM_IOFFLR_MASK           0x0000007f /* Bits 0-6: Interrupt offset low level */

/* ESM Low-Time Counter Register Section */

#define ESM_LTCR_MASK             0x0000ffff /* Bits 0-15: ERROR pin low time counter */

/* ESM Low-Time Counter Preload Register */

#define ESM_LTCPR_MASK            0x0000ffff /* Bits 0-15: ERROR pin low time counter pre-load */

/* ESM Error Key Register */

#define ESM_EKR_MASK              0x0000000f /* Bits 0-3: Error key value */
#  define ESM_EKR_NORMAL          0x00000000 /*   Activates normal mode */
#  define ESM_EKR_ERROR           0x00000005 /*   nERROR set high when LTC completes */
#  define ESM_EKR_FORCE           0x0000000a /*   Forces error on nERROR pin */

/* ESM Status Shadow Register 2 */

#define ESM_SSR2_ERROR(n)         (1 << (n)) /* Bit n: Channel n error occurred (read) */
#define ESM_SSR2_CLEAR(n)         (1 << (n)) /* Bit n: Channel n error clear (write) */

/* ESM Influence ERROR Pin Set/Status Register 4 */

#define ESM_IEPSR4_CHAN(n)        (1 << (n)) /* Bit n: Channel n read failure enable */

/* ESM Influence ERROR Pin Clear/Status Register 4 */

#define ESM_IEPCR4_CHAN(n)        (1 << (n)) /* Bit n: Channel n read failure disable */

/* ESM Interrupt Enable Set/Status Register 4 */

#define ESM_IESR4_CHAN(n)         (1 << (n)) /* Bit n: Channel n interrupt enable */

/* ESM Interrupt Enable Clear/Status Register 4 */

#define ESM_IECR4_CHAN(n)         (1 << (n)) /* Bit n: Channel n interrupt disable */

/* Interrupt Level Set/Status Register 4 */

#define ESM_ILSR4_CHAN(n)         (1 << (n)) /* Bit n: Maps channel n high level interrupt */

/* Interrupt Level Clear/Status Register 4 */

#define ESM_ILCR4_CHAN(n)         (1 << (n)) /* Bit n: Maps channel n low level interrupt */

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_ESM_H */
