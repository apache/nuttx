/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_mpu.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MPU_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K1XX_MPU_NSLAVE_PORTS 5
#define S32K1XX_MPU_NREGIONS     16

/* MPU Register Offsets *****************************************************/

#define S32K1XX_MPU_CESR_OFFSET         0x000                 /* Control/Error Status Register */
#define S32K1XX_MPU_SLAVEPORT_OFFSET(s) (0x0010 + ((s) << 3)) /* Slave port s offset, s=0..4 */
#  define S32K1XX_MPU_EAR_OFFSET        0x0000                /* Slave port error address register */
#  define S32K1XX_MPU_EDR_OFFSET        0x0004                /* Slave port error detail register */
#define S32K1XX_MPU_RGD_OFFSET(r)       (0x0400 + ((r) << 4)) /* Region n descriptor */
#  define S32K1XX_MPU_RGD_WORD0_OFFSET  0x0000                /* Region n descriptor word 0 */
#  define S32K1XX_MPU_RGD_WORD1_OFFSET  0x0004                /* Region n descriptor word 1 */
#  define S32K1XX_MPU_RGD_WORD2_OFFSET  0x0008                /* Region n descriptor word 2 */
#  define S32K1XX_MPU_RGD_WORD3_OFFSET  0x000c                /* Region n descriptor word 3 */
#define S32K1XX_MPU_RGDAAC_OFFSET(r)    (0x0800 + ((r) << 2)) /* Region n descriptor alternate access control */

/* MPU Register Addresses ***************************************************/

#define S32K1XX_MPU_CESR                (S32K1XX_MPU_BASE + S32K1XX_MPU_CESR_OFFSET)
#define S32K1XX_MPU_SLAVEPORT_BASE(s)   (S32K1XX_MPU_BASE + S32K1XX_MPU_SLAVEPORT_OFFSET(s))
#  define S32K1XX_MPU_EAR(s)            (32K1XX_MPU_SLAVEPORT_BASE(s) + S32K1XX_MPU_EAR_OFFSET)
#  define S32K1XX_MPU_EDR(s)            (32K1XX_MPU_SLAVEPORT_BASE(s) + S32K1XX_MPU_EDR_OFFSET)
#define S32K1XX_MPU_RGD_BASE(r)         (S32K1XX_MPU_BASE + S32K1XX_MPU_RGD_OFFSET(r))
#  define S32K1XX_MPU_RGD_WORD0(r)      (S32K1XX_MPU_RGD_BASE(r) + S32K1XX_MPU_RGD_WORD0_OFFSET)
#  define S32K1XX_MPU_RGD_WORD1(r)      (S32K1XX_MPU_RGD_BASE(r) + S32K1XX_MPU_RGD_WORD1_OFFSET)
#  define S32K1XX_MPU_RGD_WORD2(r)      (S32K1XX_MPU_RGD_BASE(r) + S32K1XX_MPU_RGD_WORD2_OFFSET)
#  define S32K1XX_MPU_RGD_WORD3(r)      (S32K1XX_MPU_RGD_BASE(r) + S32K1XX_MPU_RGD_WORD3_OFFSET)
#define S32K1XX_MPU_RGDAAC(r)           (S32K1XX_MPU_BASE + S32K1XX_MPU_RGDAAC_OFFSET(r))

/* MPU Register Bitfield Definitions ****************************************/

/* Control/Error Status Register */

#define MPU_CESR_VLD                    (1 << 0)  /* Bit 0:  Valid */
#define MPU_CESR_NRGD_SHIFT             (8)       /* Bits 8-11: Number Of Region Descriptors */
#define MPU_CESR_NRGD_MASK              (15 << MPU_CESR_NRGD_SHIFT)
#  define MPU_CESR_NRGD_8               (0 << MPU_CESR_NRGD_SHIFT) /* 8 region descriptors */
#  define MPU_CESR_NRGD_12              (1 << MPU_CESR_NRGD_SHIFT) /* 12 region descriptors */
#  define MPU_CESR_NRGD_16              (2 << MPU_CESR_NRGD_SHIFT) /* 16 region descriptors */

#define MPU_CESR_NSP_SHIFT              (12)      /* Bits 12-15: Number Of Slave Ports */
#define MPU_CESR_NSP_MASK               (15 << MPU_CESR_NSP_SHIFT)
#define MPU_CESR_HRL_SHIFT              (16)      /* Bits 16-19: Hardware Revision Level */
#define MPU_CESR_HRL_MASK               (15 << MPU_CESR_HRL_SHIFT)
#define MPU_CESR_SPERR4                 (1 << 27) /* Bit 27: Slave Port 4 Error */
#define MPU_CESR_SPERR3                 (1 << 28) /* Bit 28: Slave Port 3 Error */
#define MPU_CESR_SPERR2                 (1 << 29) /* Bit 29: Slave Port 2 Error */
#define MPU_CESR_SPERR1                 (1 << 30) /* Bit 30: Slave Port 1 Error */
#define MPU_CESR_SPERR0                 (1 << 31) /* Bit 31: Slave Port 0 Error */

/* Slave port error address register (32-bit error address) */

/* Slave port error detail register */

#define MPU_EDR_ERW                     (1 << 0)  /* Bit 0:  Error Read/Write */
#  define MPU_EDR_ERW_READ              (0)       /*         Read */
#  define MPU_EDR_ERW_WRITE             (1 << 0)  /*         Write */
#define MPU_EDR_EATTR_SHIFT             (1)       /* Bits 1-3: Error Attributes */
#define MPU_EDR_EATTR_MASK              (7 << MPU_EDR_EATTR_SHIFT)
#  define MPU_EDR_EATTR_IUSER           (0 << MPU_EDR_EATTR_SHIFT) /* User mode, instruction access */
#  define MPU_EDR_EATTR_DUSER           (1 << MPU_EDR_EATTR_SHIFT) /* User mode, data access */
#  define MPU_EDR_EATTR_ISUPER          (2 << MPU_EDR_EATTR_SHIFT) /* Supervisor mode, instruction access */
#  define MPU_EDR_EATTR_DSUPER          (3 << MPU_EDR_EATTR_SHIFT) /* Supervisor mode, data access */

#define MPU_EDR_EMN_SHIFT               (4)       /* Bits 4-7: Error Master Number */
#define MPU_EDR_EMN_MASK                (15 << MPU_EDR_EMN_SHIFT)
#define MPU_EDR_EPID_SHIFT              (8)       /* Bits 8-15: Error Process Identification */
#define MPU_EDR_EPID_MASK               (0xff << MPU_EDR_EPID_SHIFT)
#define MPU_EDR_EACD_SHIFT              (16)      /* Bits 16-31: Error Access Control Detail */
#define MPU_EDR_EACD_MASK               (0xffff << MPU_EDR_EACD_SHIFT)

/* Region n descriptor word 0 */

#define MPU_RGD_WORD0_SRTADDR_MASK      0xffffffe0  /* Bits 5-31:  Start Address bits 5-31 */

/* Region n descriptor word 1 */

#define MPU_RGD_WORD1_ENDADDR_MASK      0xffffffe0  /* Bits 5-31:  End Address bits 5-31 */

/* Region n descriptor word 2 */

#define MPU_RGD_WORD2_M0UM_SHIFT        (0)       /* Bits 0-2: Bus Master 0 User Mode Access Control */
#define MPU_RGD_WORD2_M0UM_MASK         (7 << MPU_RGD_WORD2_M0UM_SHIFT)
#  define MPU_RGD_WORD2_M0UM_XACCESS    (1 << MPU_RGD_WORD2_M0UM_SHIFT)
#  define MPU_RGD_WORD2_M0UM_WACCESS    (2 << MPU_RGD_WORD2_M0UM_SHIFT)
#  define MPU_RGD_WORD2_M0UM_RACCESS    (4 << MPU_RGD_WORD2_M0UM_SHIFT)
#define MPU_RGD_WORD2_M0SM_SHIFT        (3)       /* Bits 3-4: Bus Master 0 Supervisor Mode Access Control */
#define MPU_RGD_WORD2_M0SM_MASK         (3 << MPU_RGD_WORD2_M0SM_SHIFT)
#  define MPU_RGD_WORD2_M0SM_RWX        (0 << MPU_RGD_WORD2_M0SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGD_WORD2_M0SM_RX         (1 << MPU_RGD_WORD2_M0SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGD_WORD2_M0SM_RW         (2 << MPU_RGD_WORD2_M0SM_SHIFT) /* Read and write allowed */
#  define MPU_RGD_WORD2_M0SM_M0UM       (3 << MPU_RGD_WORD2_M0SM_SHIFT) /* Same as User mode defined in M0UM */

#define MPU_RGD_WORD2_M0PE              (1 << 5)  /* Bit 5:  Bus Master 0 Process Identifier enable */
#define MPU_RGD_WORD2_M1UM_SHIFT        (6)       /* Bits 6-8: Bus Master 1 User Mode Access Control */
#define MPU_RGD_WORD2_M1UM_MASK         (7 << MPU_RGD_WORD2_M1UM_SHIFT)
#  define MPU_RGD_WORD2_M1UM_XACCESS    (1 << MPU_RGD_WORD2_M1UM_SHIFT)
#  define MPU_RGD_WORD2_M1UM_WACCESS    (2 << MPU_RGD_WORD2_M1UM_SHIFT)
#  define MPU_RGD_WORD2_M1UM_RACCESS    (4 << MPU_RGD_WORD2_M1UM_SHIFT)

#define MPU_RGD_WORD2_M1SM_SHIFT        (9)       /* Bits 9-10: Bus Master 1 Supervisor Mode Access Control */
#define MPU_RGD_WORD2_M1SM_MASK         (3 << MPU_RGD_WORD2_M1SM_SHIFT)
#  define MPU_RGD_WORD2_M1SM_RWX        (0 << MPU_RGD_WORD2_M1SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGD_WORD2_M1SM_RX         (1 << MPU_RGD_WORD2_M1SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGD_WORD2_M1SM_RW         (2 << MPU_RGD_WORD2_M1SM_SHIFT) /* Read and write allowed */
#  define MPU_RGD_WORD2_M1SM_M1UM       (3 << MPU_RGD_WORD2_M1SM_SHIFT) /* Same as User mode defined in M1UM */

#define MPU_RGD_WORD2_M1PE              (1 << 11) /* Bit 11: Bus Master 1 Process Identifier enable */
#define MPU_RGD_WORD2_M2UM_SHIFT        (12)      /* Bits 12-14: Bus Master 2 User Mode Access control */
#define MPU_RGD_WORD2_M2UM_MASK         (7 << MPU_RGD_WORD2_M2UM_SHIFT)
#  define MPU_RGD_WORD2_M2UM_XACCESS    (1 << MPU_RGD_WORD2_M2UM_SHIFT)
#  define MPU_RGD_WORD2_M2UM_WACCESS    (2 << MPU_RGD_WORD2_M2UM_SHIFT)
#  define MPU_RGD_WORD2_M2UM_RACCESS    (4 << MPU_RGD_WORD2_M2UM_SHIFT)

#define MPU_RGD_WORD2_M2SM_SHIFT        (15)      /* Bits 15-16: Bus Master 2 Supervisor Mode Access Control */
#define MPU_RGD_WORD2_M2SM_MASK         (3 << MPU_RGD_WORD2_M2SM_SHIFT)
#  define MPU_RGD_WORD2_M2SM_RWX        (0 << MPU_RGD_WORD2_M2SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGD_WORD2_M2SM_RX         (1 << MPU_RGD_WORD2_M2SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGD_WORD2_M2SM_RW         (2 << MPU_RGD_WORD2_M2SM_SHIFT) /* Read and write allowed */
#  define MPU_RGD_WORD2_M2SM_M2UM       (3 << MPU_RGD_WORD2_M2SM_SHIFT) /* Same as User mode defined in M2UM */

#define MPU_RGD_WORD2_M3UM_SHIFT        (18)      /* Bits 18-20: Bus Master 3 User Mode Access Control */
#define MPU_RGD_WORD2_M3UM_MASK         (7 << MPU_RGD_WORD2_M3UM_SHIFT)
#  define MPU_RGD_WORD2_M3UM_XACCESS    (1 << MPU_RGD_WORD2_M3UM_SHIFT)
#  define MPU_RGD_WORD2_M3UM_WACCESS    (2 << MPU_RGD_WORD2_M3UM_SHIFT)
#  define MPU_RGD_WORD2_M3UM_RACCESS    (4 << MPU_RGD_WORD2_M3UM_SHIFT)

#define MPU_RGD_WORD2_M3SM_SHIFT        (21)      /* Bits 21-22: Bus Master 3 Supervisor Mode Access Control */
#define MPU_RGD_WORD2_M3SM_MASK         (3 << MPU_RGD_WORD2_M3SM_SHIFT)
#  define MPU_RGD_WORD2_M3SM_RWX        (0 << MPU_RGD_WORD2_M3SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGD_WORD2_M3SM_RX         (1 << MPU_RGD_WORD2_M3SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGD_WORD2_M3SM_RW         (2 << MPU_RGD_WORD2_M3SM_SHIFT) /* Read and write allowed */
#  define MPU_RGD_WORD2_M3SM_M3UM       (3 << MPU_RGD_WORD2_M3SM_SHIFT) /* Same as User mode defined in M3UM */

#define MPU_RGD_WORD2_M4WE              (1 << 24) /* Bit 24: Bus Master 4 Write Enable */
#define MPU_RGD_WORD2_M4RE              (1 << 25) /* Bit 25: Bus Master 4 Read Enable */
#define MPU_RGD_WORD2_M5RE              (1 << 27) /* Bit 27: Bus Master 5 Read Enable */
#define MPU_RGD_WORD2_M6WE              (1 << 28) /* Bit 28: Bus Master 6 Write Enable */
#define MPU_RGD_WORD2_M6RE              (1 << 29) /* Bit 29: Bus Master 6 Read Enable */
#define MPU_RGD_WORD2_M7WE              (1 << 30) /* Bit 30: Bus Master 7 Write Enable */
#define MPU_RGD_WORD2_M7RE              (1 << 31) /* Bit 31: Bus Master 7 Read Enable */

/* Region n descriptor word 3 */

#define MPU_RGD_WORD3_VLD               (1 << 0)  /* Bit 0:  Valid */
#define MPU_RGD_WORD3_PIDMASK_SHIFT     (16)      /* Bits 16-23: Process Identifier Mask */
#define MPU_RGD_WORD3_PIDMASK_MASK      (0xff << MPU_RGD_WORD3_PIDMASK_SHIFT)
#  define MPU_RGD_WORD3_PIDMASK(n)      ((uint32_t)(n) << MPU_RGD_WORD3_PIDMASK_SHIFT)
#define MPU_RGD_WORD3_PID_SHIFT         (24)      /* Bits 24-31: Process Identifier */
#define MPU_RGD_WORD3_PID_MASK          (0xff << MPU_RGD_WORD3_PID_SHIFT)
#  define MPU_RGD_WORD3_PID(n)          ((uint32_t)(n) << MPU_RGD_WORD3_PID_SHIFT)

/* Region n descriptor alternate access control */
#define MPU_RGDAAC_

#define MPU_RGDAAC_M0UM_SHIFT        (0)       /* Bits 0-2: Bus Master 0 User Mode Access Control */
#define MPU_RGDAAC_M0UM_MASK         (7 << MPU_RGDAAC_M0UM_SHIFT)
#  define MPU_RGDAAC_M0UM_XACCESS    (1 << MPU_RGDAAC_M0UM_SHIFT)
#  define MPU_RGDAAC_M0UM_WACCESS    (2 << MPU_RGDAAC_M0UM_SHIFT)
#  define MPU_RGDAAC_M0UM_RACCESS    (4 << MPU_RGDAAC_M0UM_SHIFT)
#define MPU_RGDAAC_M0SM_SHIFT        (3)       /* Bits 3-4: Bus Master 0 Supervisor Mode Access Control */
#define MPU_RGDAAC_M0SM_MASK         (3 << MPU_RGDAAC_M0SM_SHIFT)
#  define MPU_RGDAAC_M0SM_RWX        (0 << MPU_RGDAAC_M0SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGDAAC_M0SM_RX         (1 << MPU_RGDAAC_M0SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGDAAC_M0SM_RW         (2 << MPU_RGDAAC_M0SM_SHIFT) /* Read and write allowed */
#  define MPU_RGDAAC_M0SM_M0UM       (3 << MPU_RGDAAC_M0SM_SHIFT) /* Same as User mode defined in M0UM */

#define MPU_RGDAAC_M0PE              (1 << 5)  /* Bit 5:  Bus Master 0 Process Identifier enable */
#define MPU_RGDAAC_M1UM_SHIFT        (6)       /* Bits 6-8: Bus Master 1 User Mode Access Control */
#define MPU_RGDAAC_M1UM_MASK         (7 << MPU_RGDAAC_M1UM_SHIFT)
#  define MPU_RGDAAC_M1UM_XACCESS    (1 << MPU_RGDAAC_M1UM_SHIFT)
#  define MPU_RGDAAC_M1UM_WACCESS    (2 << MPU_RGDAAC_M1UM_SHIFT)
#  define MPU_RGDAAC_M1UM_RACCESS    (4 << MPU_RGDAAC_M1UM_SHIFT)
#define MPU_RGDAAC_M1SM_SHIFT        (9)       /* Bits 9-10: Bus Master 1 Supervisor Mode Access Control */
#define MPU_RGDAAC_M1SM_MASK         (3 << MPU_RGDAAC_M1SM_SHIFT)
#  define MPU_RGDAAC_M1SM_RWX        (0 << MPU_RGDAAC_M1SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGDAAC_M1SM_RX         (1 << MPU_RGDAAC_M1SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGDAAC_M1SM_RW         (2 << MPU_RGDAAC_M1SM_SHIFT) /* Read and write allowed */
#  define MPU_RGDAAC_M1SM_M1UM       (3 << MPU_RGDAAC_M1SM_SHIFT) /* Same as User mode defined in M1UM */

#define MPU_RGDAAC_M1PE              (1 << 11) /* Bit 11: Bus Master 1 Process Identifier enable */
#define MPU_RGDAAC_M2UM_SHIFT        (12)      /* Bits 12-14: Bus Master 2 User Mode Access control */
#define MPU_RGDAAC_M2UM_MASK         (7 << MPU_RGDAAC_M2UM_SHIFT)
#  define MPU_RGDAAC_M2UM_XACCESS    (1 << MPU_RGDAAC_M2UM_SHIFT)
#  define MPU_RGDAAC_M2UM_WACCESS    (2 << MPU_RGDAAC_M2UM_SHIFT)
#  define MPU_RGDAAC_M2UM_RACCESS    (4 << MPU_RGDAAC_M2UM_SHIFT)

#define MPU_RGDAAC_M2SM_SHIFT        (15)      /* Bits 15-16: Bus Master 2 Supervisor Mode Access Control */
#define MPU_RGDAAC_M2SM_MASK         (3 << MPU_RGDAAC_M2SM_SHIFT)
#  define MPU_RGDAAC_M2SM_RWX        (0 << MPU_RGDAAC_M2SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGDAAC_M2SM_RX         (1 << MPU_RGDAAC_M2SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGDAAC_M2SM_RW         (2 << MPU_RGDAAC_M2SM_SHIFT) /* Read and write allowed */
#  define MPU_RGDAAC_M2SM_M2UM       (3 << MPU_RGDAAC_M2SM_SHIFT) /* Same as User mode defined in M2UM */

#define MPU_RGDAAC_M3UM_SHIFT        (18)      /* Bits 18-20: Bus Master 3 User Mode Access Control */
#define MPU_RGDAAC_M3UM_MASK         (7 << MPU_RGDAAC_M3UM_SHIFT)
#  define MPU_RGDAAC_M3UM_XACCESS    (1 << MPU_RGDAAC_M3UM_SHIFT)
#  define MPU_RGDAAC_M3UM_WACCESS    (2 << MPU_RGDAAC_M3UM_SHIFT)
#  define MPU_RGDAAC_M3UM_RACCESS    (4 << MPU_RGDAAC_M3UM_SHIFT)

#define MPU_RGDAAC_M3SM_SHIFT        (21)      /* Bits 21-22: Bus Master 3 Supervisor Mode Access Control */
#define MPU_RGDAAC_M3SM_MASK         (3 << MPU_RGDAAC_M3SM_SHIFT)
#  define MPU_RGDAAC_M3SM_RWX        (0 << MPU_RGDAAC_M3SM_SHIFT) /* Read, write and execute allowed */
#  define MPU_RGDAAC_M3SM_RX         (1 << MPU_RGDAAC_M3SM_SHIFT) /* Read and execute allowed */
#  define MPU_RGDAAC_M3SM_RW         (2 << MPU_RGDAAC_M3SM_SHIFT) /* Read and write allowed */
#  define MPU_RGDAAC_M3SM_M3UM       (3 << MPU_RGDAAC_M3SM_SHIFT) /* Same as User mode defined in M3UM */

#define MPU_RGDAAC_M4WE              (1 << 24) /* Bit 24: Bus Master 4 Write Enable */
#define MPU_RGDAAC_M4RE              (1 << 25) /* Bit 25: Bus Master 4 Read Enable */
#define MPU_RGDAAC_M5RE              (1 << 27) /* Bit 27: Bus Master 5 Read Enable */
#define MPU_RGDAAC_M6WE              (1 << 28) /* Bit 28: Bus Master 6 Write Enable */
#define MPU_RGDAAC_M6RE              (1 << 29) /* Bit 29: Bus Master 6 Read Enable */
#define MPU_RGDAAC_M7WE              (1 << 30) /* Bit 30: Bus Master 7 Write Enable */
#define MPU_RGDAAC_M7RE              (1 << 31) /* Bit 31: Bus Master 7 Read Enable */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MPU_H */
