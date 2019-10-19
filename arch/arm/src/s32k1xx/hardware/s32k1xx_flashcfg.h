/********************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_flashcfg.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLASHCFG_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLASHCFG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* FLASHCFG Register Offsets ****************************************************************/

#define S32K1XX_FLASHCFG_BACKDOOR1_OFFSET 0x0000 /* Backdoor Comparison Key 1 */
#define S32K1XX_FLASHCFG_BACKDOOR2_OFFSET 0x0004 /* Backdoor Comparison Key 2 */
#define S32K1XX_FLASHCFG_FPROT_OFFSET     0x0008 /* Program flash protection bytes */
#define S32K1XX_FLASHCFG_FSEC_OFFSET      0x000c /* Flash security byte */
#define S32K1XX_FLASHCFG_FOPT_OFFSET      0x000d /* Flash nonvolatile option byte */
#define S32K1XX_FLASHCFG_FEPROT_OFFSET    0x000e /* EEPROM protection byte */
#define S32K1XX_FLASHCFG_FDPROT_OFFSET    0x000f /* Data flash protection byte */

/* FLASHCFG Register Addresses ***************************************************************/

#define S32K1XX_FLASHCFG_BACKDOOR1        (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_BACKDOOR1_OFFSET)
#define S32K1XX_FLASHCFG_BACKDOOR2        (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_BACKDOOR2_OFFSET)
#define S32K1XX_FLASHCFG_FPROT            (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_FPROT_OFFSET)
#define S32K1XX_FLASHCFG_FSEC             (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_FSEC_OFFSET)
#define S32K1XX_FLASHCFG_FOPT             (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_FOPT_OFFSET)
#define S32K1XX_FLASHCFG_FEPROT           (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_FEPROT_OFFSET)
#define S32K1XX_FLASHCFG_FDPROT           (S32K1XX_FLASHCFG_BASE + S32K1XX_FLASHCFG_FDPROT_OFFSET)

/* FLASHCFG Register Bitfield Definitions ***************************************************/

/* Backdoor Comparison Key 1 (32-bits) */
/* Backdoor Comparison Key 2 (32-bits) */

/* Program flash protection bytes
 *
 * Region 0:  0x000a0000
 * Region 1:  0x000a4000
 * Region 2:  0x000a8000
 * Region 3:  0x000ac000
 * ...
 * Region 32: 0x0011c000
 */

#define FLASHCFG_FPROT_REGION(n)          (1 << (n))

/* Flash security byte */

#define FLASHCFG_FSEC_SEC_SHIFT           (0)      /* Bits 0-1: Flash Security */
#define FLASHCFG_FSEC_SEC_MASK            (3 << FLASHCFG_FSEC_SHIFT)
#  define FLASHCFG_FSEC_SEC_SECURE1       (0 << FLASHCFG_FSEC_SHIFT)
#  define FLASHCFG_FSEC_SEC_SECURE2       (1 << FLASHCFG_FSEC_SHIFT)
#  define FLASHCFG_FSEC_SEC_UNSECURE      (2 << FLASHCFG_FSEC_SHIFT)
#  define FLASHCFG_FSEC_SEC_SECURE3       (0 << FLASHCFG_FSEC_SHIFT)
#define FLASHCFG_FSEC_FSLACC_SHIFT        (2)      /* Bits 2-3: Factory Failure Analysis Access Code */
#define FLASHCFG_FSEC_FSLACC_MASK         (3 << FLASHCFG_FSEC_FSLACC_SHIFT)
#  define FLASHCFG_FSEC_FSLACC_GRANTED1   (0 << FLASHCFG_FSEC_FSLACC_SHIFT)
#  define FLASHCFG_FSEC_FSLACC_DENIED1    (1 << FLASHCFG_FSEC_FSLACC_SHIFT)
#  define FLASHCFG_FSEC_FSLACC_DENIED2    (2 << FLASHCFG_FSEC_FSLACC_SHIFT)
#  define FLASHCFG_FSEC_FSLACC_GRANTED2   (3 << FLASHCFG_FSEC_FSLACC_SHIFT)
#define FLASHCFG_FSEC_MEEN_SHIFT          (4)      /* Bits 4-5:  Mass Erase Enable Bits */
#define FLASHCFG_FSEC_MEEN_MASK           (3 << FLASHCFG_FSEC_MEEN_SHIFT)
#  define FLASHCFG_FSEC_MEEN_ENABLED1     (0 << FLASHCFG_FSEC_MEEN_SHIFT)
#  define FLASHCFG_FSEC_MEEN_ENABLED2     (1 << FLASHCFG_FSEC_MEEN_SHIFT)
#  define FLASHCFG_FSEC_MEEN_DISABLED     (2 << FLASHCFG_FSEC_MEEN_SHIFT)
#  define FLASHCFG_FSEC_MEEN_ENABLED3     (3 << FLASHCFG_FSEC_MEEN_SHIFT)
#define FLASHCFG_FSEC_KEYEN_SHIFT         (6)     /* Bits 6-7:  Backdoor Key Security Enable */
#define FLASHCFG_FSEC_KEYEN_MASK          (3 << FLASHCFG_FSEC_KEYEN_SHIFT)
#  define FLASHCFG_FSEC_KEYEN_DISABLED1   (0 << FLASHCFG_FSEC_KEYEN_SHIFT)
#  define FLASHCFG_FSEC_KEYEN_DISABLED2   (1 << FLASHCFG_FSEC_KEYEN_SHIFT)
#  define FLASHCFG_FSEC_KEYEN_ENABLED     (2 << FLASHCFG_FSEC_KEYEN_SHIFT)
#  define FLASHCFG_FSEC_KEYEN_DISABLED3   (3 << FLASHCFG_FSEC_KEYEN_SHIFT)

/* Flash nonvolatile option byte (8-bits, Refer to the device's Chip Configuration details
 * for the definition and use of these bits.
 */

/* EEPROM protection byte.  Each EPROT bit covers one-eighth of the configured EEPROM data */

#define FLASHCFG_FEPROT(n)               (1 << (n))

/* Data flash protection byte.  Each DPROT bit protects one-eighth of the partitioned data
 * flash memory space.
 */

#define FLASHCFG_FDPROT(n)               (1 << (n))

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLASHCFG_H */
