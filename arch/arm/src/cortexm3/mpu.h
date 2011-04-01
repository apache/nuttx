/************************************************************************************
 * arch/arm/src/cortexm3/mpu.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_COMMON_CORTEXM_MPU_H
#define __ARCH_ARM_SRC_COMMON_CORTEXM_MPU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* MPU Register Addresses */

#define MPU_TYPE                0xe000ed90 /* MPU Type Register */
#define MPU_CTRL                0xe000ed94 /* MPU Control Register */
#define MPU_RNR                 0xe000ed98 /* MPU Region Number Register */
#define MPU_RBAR                0xe000ed9c /* MPU Region Base Address Register */
#define MPU_RASR                0xe000eda0 /* MPU Region Attribute and Size Register */

/* MPU Type Register Bit Definitions */

#define MPU_TYPE_SEPARATE       (1 << 0) /* Bit 0: 0:unified or 1:separate memory maps */
#define MPU_TYPE_DREGION_SHIFT  (8)      /* Bits 8-15: Number MPU data regsion */
#define MPU_TYPE_DREGION_MASK   (0xff << MPU_TYPE_DREGION_SHIFT)
#define MPU_TYPE_IREGION_SHIFT  (16)     /* Bits 16-23: Number MPU instruction regions */
#define MPU_TYPE_IREGION_MASK   (0xff << MPU_TYPE_IREGION_SHIFT)

/* MPU Control Register Bit Definitions */

#define MPU_CTRL_ENABLE         (1 << 0)  /* Bit 0: Enable the MPU */
#define MPU_CTRL_HFNMIENA       (1 << 1)  /* Bit 1: Enable MPU during hard fault, NMI, and FAULTMAS */
#define MPU_CTRL_PRIVDEFENA     (1 << 2)  /* Bit 2: Enable privileged access to default memory map */

/* MPU Region Number Register Bit Definitions */

#define MPU_RNR_MASK            (0xff)     

/* MPU Region Base Address Register Bit Definitions */

#define MPU_RBAR_REGION_SHIFT   (0)       /* Bits 0-3: MPU region */
#define MPU_RBAR_REGION_MASK    (15 << MPU_RBAR_REGION_SHIFT)
#define MPU_RBAR_VALID          (1 << 4)  /* Bit 4: MPU Region Number valid */ 
#define MPU_RBAR_ADDR_MASK      0xffffffe0 /* Bits N-31:  Region base addrese */

/* MPU Region Attributes and Size Register Bit Definitions */

#define MPU_RASR_ENABLE         (1 << 0)  /* Bit 0: Region enable */
#define MPU_RASR_SIZE_SHIFT     (1)       /* Bits 1-5: Size of the MPU protection region */  
#define MPU_RASR_SIZE_MASK      (31 << MPU_RASR_SIZE_SHIFT)
#  define MPU_RASR_SIZE_LOG2(n) ((n-1) << MPU_RASR_SIZE_SHIFT)
#define MPU_RASR_SRD_SHIFT      (8)       /* Bits 8-15: Subregion disable */
#define MPU_RASR_SRD_MASK       (0xff << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_0        (0x01 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_1        (0x02 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_2        (0x04 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_3        (0x08 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_4        (0x10 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_5        (0x20 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_6        (0x40 << MPU_RASR_SRD_SHIFT)
#  define MPU_RASR_SRD_7        (0x80 << MPU_RASR_SRD_SHIFT)
#define MPU_RASR_ATTR_SHIFT     (21)      /* Bits 19-21: TEX Address Permisson */
#define MPU_RASR_ATTR__MASK     (7 << MPU_RASR_ATTR_SHIFT)
#define MPU_RASR_S              (1 << 18) /* Bit 18: Shareable */
#define MPU_RASR_C              (1 << 17) /* Bit 17: C Address Permission */
#define MPU_RASR_B              (1 << 16) /* Bit 16: B Address Permission */
#define MPU_RASR_AP_SHIFT       (24)      /* Bits 24-26: Access permission */
#define MPU_RASR_AP_MASK        (7 << MPU_RASR_AP_SHIFT)
#  define MPU_RASR_AP_NONO      (0 << MPU_RASR_AP_SHIFT) /* P:None U:None */
#  define MPU_RASR_AP_RWNO      (1 << MPU_RASR_AP_SHIFT) /* P:RW   U:None */
#  define MPU_RASR_AP_RWRO      (2 << MPU_RASR_AP_SHIFT) /* P:RW   U:RO   */
#  define MPU_RASR_AP_RWRW      (3 << MPU_RASR_AP_SHIFT) /* P:RW   U:RW   */
#  define MPU_RASR_AP_RONO      (5 << MPU_RASR_AP_SHIFT) /* P:RO   U:None */
#  define MPU_RASR_AP_RORO      (6 << MPU_RASR_AP_SHIFT) /* P:R0   U:RO   */
#define MPU_RASR_XN             (1 << 28) /* Bit 28: Instruction access disable */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_COMMON_CORTEXM_MPU_H */
