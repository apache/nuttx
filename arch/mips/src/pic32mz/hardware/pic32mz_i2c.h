/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_i2c.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_I2C_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/pic32mz/chip.h>

#include "hardware/pic32mz_memorymap.h"

#if CHIP_NI2C > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Peripheral Offsets ***************************************************/

#define PIC32MZ_I2Cn_OFFSET(n)     ((n) << 9)
#  define PIC32MZ_I2C1_OFFSET      0x0000
#  define PIC32MZ_I2C2_OFFSET      0x0200
#  define PIC32MZ_I2C3_OFFSET      0x0400
#  define PIC32MZ_I2C4_OFFSET      0x0600
#  define PIC32MZ_I2C5_OFFSET      0x0800

/* I2C Register Offsets *****************************************************/

#define PIC32MZ_I2C_CON_OFFSET     0x0000 /* I2C control register */
#define PIC32MZ_I2C_CONCLR_OFFSET  0x0004 /* I2C control clear register */
#define PIC32MZ_I2C_CONSET_OFFSET  0x0008 /* I2C control set register */
#define PIC32MZ_I2C_CONINV_OFFSET  0x000c /* I2C control invert register */

#define PIC32MZ_I2C_STAT_OFFSET    0x0010 /* I2C status register */
#define PIC32MZ_I2C_STATCLR_OFFSET 0x0014 /* I2C status clear register */
#define PIC32MZ_I2C_STATSET_OFFSET 0x0018 /* I2C status set register */
#define PIC32MZ_I2C_STATINV_OFFSET 0x001c /* I2C status invert register */

#define PIC32MZ_I2C_ADD_OFFSET     0x0020 /* I2C address register */
#define PIC32MZ_I2C_ADDCLR_OFFSET  0x0024 /* I2C address clear register */
#define PIC32MZ_I2C_ADDSET_OFFSET  0x0028 /* I2C address set register */
#define PIC32MZ_I2C_ADDINV_OFFSET  0x002c /* I2C address invert register */

#define PIC32MZ_I2C_MSK_OFFSET     0x0030 /* I2C address mask register */
#define PIC32MZ_I2C_MSKCLR_OFFSET  0x0034 /* I2C address mask clear register */
#define PIC32MZ_I2C_MSKSET_OFFSET  0x0038 /* I2C address mask set register */
#define PIC32MZ_I2C_MSKINV_OFFSET  0x003c /* I2C address mask invert register */

#define PIC32MZ_I2C_BRG_OFFSET     0x0040 /* Baud rate generator reload register */
#define PIC32MZ_I2C_BRGCLR_OFFSET  0x0044 /* Baud rate generator reload set register */
#define PIC32MZ_I2C_BRGSET_OFFSET  0x0048 /* Baud rate generator reload clear register */
#define PIC32MZ_I2C_BRGINV_OFFSET  0x004c /* Baud rate generator reload invert register */

#define PIC32MZ_I2C_TRN_OFFSET     0x0050 /* I2C transmit register */
#define PIC32MZ_I2C_TRNCLR_OFFSET  0x0054 /* I2C transmit clear register */
#define PIC32MZ_I2C_TRNSET_OFFSET  0x0058 /* I2C transmit set register */
#define PIC32MZ_I2C_TRNINV_OFFSET  0x005c /* I2C transmit invert register */

#define PIC32MZ_I2C_RCV_OFFSET     0x0060 /* I2C receive buffer register */

/* I2C Peripheral Addresses *************************************************/

#define PIC32MZ_I2Cn_K1BASE(n)     (PIC32MZ_I2C_K1BASE+PIC32MZ_I2Cn_OFFSET(n))
#  define PIC32MZ_I2C1_K1BASE      (PIC32MZ_I2C_K1BASE+PIC32MZ_I2C1_OFFSET)
#  define PIC32MZ_I2C2_K1BASE      (PIC32MZ_I2C_K1BASE+PIC32MZ_I2C2_OFFSET)
#  define PIC32MZ_I2C3_K1BASE      (PIC32MZ_I2C_K1BASE+PIC32MZ_I2C3_OFFSET)
#  define PIC32MZ_I2C4_K1BASE      (PIC32MZ_I2C_K1BASE+PIC32MZ_I2C4_OFFSET)
#  define PIC32MZ_I2C5_K1BASE      (PIC32MZ_I2C_K1BASE+PIC32MZ_I2C5_OFFSET)

/* I2C Register Addresses ***************************************************/

#  define PIC32MZ_I2C1_CON         (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_CON_OFFSET)
#  define PIC32MZ_I2C1_CONCLR      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_CONCLR_OFFSET)
#  define PIC32MZ_I2C1_CONSET      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_CONSET_OFFSET)
#  define PIC32MZ_I2C1_CONINV      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_CONINV_OFFSET)

#  define PIC32MZ_I2C1_STAT        (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_STAT_OFFSET)
#  define PIC32MZ_I2C1_STATCLR     (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_STATCLR_OFFSET)
#  define PIC32MZ_I2C1_STATSET     (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_STATSET_OFFSET)
#  define PIC32MZ_I2C1_STATINV     (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_STATINV_OFFSET)

#  define PIC32MZ_I2C1_ADD         (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_ADD_OFFSET)
#  define PIC32MZ_I2C1_ADDCLR      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_ADDCLR_OFFSET)
#  define PIC32MZ_I2C1_ADDSET      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_ADDSET_OFFSET)
#  define PIC32MZ_I2C1_ADDINV      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_ADDINV_OFFSET)

#  define PIC32MZ_I2C1_MSK         (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_MSK_OFFSET)
#  define PIC32MZ_I2C1_MSKCLR      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_MSKCLR_OFFSET)
#  define PIC32MZ_I2C1_MSKSET      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_MSKSET_OFFSET)
#  define PIC32MZ_I2C1_MSKINV      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_MSKINV_OFFSET)

#  define PIC32MZ_I2C1_BRG         (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_BRG_OFFSET)
#  define PIC32MZ_I2C1_BRGSET      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_BRGSET_OFFSET)
#  define PIC32MZ_I2C1_BRGCLR      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_BRGCLR_OFFSET)
#  define PIC32MZ_I2C1_BRGINV      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_BRGINV_OFFSET)

#  define PIC32MZ_I2C1_TRN         (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_TRN_OFFSET)
#  define PIC32MZ_I2C1_TRNCLR      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_TRNCLR_OFFSET)
#  define PIC32MZ_I2C1_TRNSET      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_TRNSET_OFFSET)
#  define PIC32MZ_I2C1_TRNINV      (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_TRNINV_OFFSET)

#  define PIC32MZ_I2C1_RCV         (PIC32MZ_I2C1_K1BASE+PIC32MZ_I2C_RCV_OFFSET)
#endif

#if CHIP_NI2C > 1
#  define PIC32MZ_I2C2_CON         (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_CON_OFFSET)
#  define PIC32MZ_I2C2_CONCLR      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_CONCLR_OFFSET)
#  define PIC32MZ_I2C2_CONSET      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_CONSET_OFFSET)
#  define PIC32MZ_I2C2_CONINV      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_CONINV_OFFSET)

#  define PIC32MZ_I2C2_STAT        (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_STAT_OFFSET)
#  define PIC32MZ_I2C2_STATCLR     (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_STATCLR_OFFSET)
#  define PIC32MZ_I2C2_STATSET     (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_STATSET_OFFSET)
#  define PIC32MZ_I2C2_STATINV     (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_STATINV_OFFSET)

#  define PIC32MZ_I2C2_ADD         (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_ADD_OFFSET)
#  define PIC32MZ_I2C2_ADDCLR      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_ADDCLR_OFFSET)
#  define PIC32MZ_I2C2_ADDSET      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_ADDSET_OFFSET)
#  define PIC32MZ_I2C2_ADDINV      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_ADDINV_OFFSET)

#  define PIC32MZ_I2C2_MSK         (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_MSK_OFFSET)
#  define PIC32MZ_I2C2_MSKCLR      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_MSKCLR_OFFSET)
#  define PIC32MZ_I2C2_MSKSET      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_MSKSET_OFFSET)
#  define PIC32MZ_I2C2_MSKINV      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_MSKINV_OFFSET)

#  define PIC32MZ_I2C2_BRG         (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_BRG_OFFSET)
#  define PIC32MZ_I2C2_BRGSET      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_BRGSET_OFFSET)
#  define PIC32MZ_I2C2_BRGCLR      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_BRGCLR_OFFSET)
#  define PIC32MZ_I2C2_BRGINV      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_BRGINV_OFFSET)

#  define PIC32MZ_I2C2_TRN         (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_TRN_OFFSET)
#  define PIC32MZ_I2C2_TRNCLR      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_TRNCLR_OFFSET)
#  define PIC32MZ_I2C2_TRNSET      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_TRNSET_OFFSET)
#  define PIC32MZ_I2C2_TRNINV      (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_TRNINV_OFFSET)

#  define PIC32MZ_I2C2_RCV         (PIC32MZ_I2C2_K1BASE+PIC32MZ_I2C_RCV_OFFSET)
#endif

#if CHIP_NI2C > 2
#  define PIC32MZ_I2C3_CON         (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_CON_OFFSET)
#  define PIC32MZ_I2C3_CONCLR      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_CONCLR_OFFSET)
#  define PIC32MZ_I2C3_CONSET      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_CONSET_OFFSET)
#  define PIC32MZ_I2C3_CONINV      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_CONINV_OFFSET)

#  define PIC32MZ_I2C3_STAT        (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_STAT_OFFSET)
#  define PIC32MZ_I2C3_STATCLR     (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_STATCLR_OFFSET)
#  define PIC32MZ_I2C3_STATSET     (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_STATSET_OFFSET)
#  define PIC32MZ_I2C3_STATINV     (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_STATINV_OFFSET)

#  define PIC32MZ_I2C3_ADD         (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_ADD_OFFSET)
#  define PIC32MZ_I2C3_ADDCLR      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_ADDCLR_OFFSET)
#  define PIC32MZ_I2C3_ADDSET      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_ADDSET_OFFSET)
#  define PIC32MZ_I2C3_ADDINV      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_ADDINV_OFFSET)

#  define PIC32MZ_I2C3_MSK         (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_MSK_OFFSET)
#  define PIC32MZ_I2C3_MSKCLR      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_MSKCLR_OFFSET)
#  define PIC32MZ_I2C3_MSKSET      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_MSKSET_OFFSET)
#  define PIC32MZ_I2C3_MSKINV      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_MSKINV_OFFSET)

#  define PIC32MZ_I2C3_BRG         (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_BRG_OFFSET)
#  define PIC32MZ_I2C3_BRGSET      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_BRGSET_OFFSET)
#  define PIC32MZ_I2C3_BRGCLR      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_BRGCLR_OFFSET)
#  define PIC32MZ_I2C3_BRGINV      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_BRGINV_OFFSET)

#  define PIC32MZ_I2C3_TRN         (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_TRN_OFFSET)
#  define PIC32MZ_I2C3_TRNCLR      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_TRNCLR_OFFSET)
#  define PIC32MZ_I2C3_TRNSET      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_TRNSET_OFFSET)
#  define PIC32MZ_I2C3_TRNINV      (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_TRNINV_OFFSET)

#  define PIC32MZ_I2C3_RCV         (PIC32MZ_I2C3_K1BASE+PIC32MZ_I2C_RCV_OFFSET)
#endif

#if CHIP_NI2C > 3
#  define PIC32MZ_I2C4_CON         (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_CON_OFFSET)
#  define PIC32MZ_I2C4_CONCLR      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_CONCLR_OFFSET)
#  define PIC32MZ_I2C4_CONSET      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_CONSET_OFFSET)
#  define PIC32MZ_I2C4_CONINV      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_CONINV_OFFSET)

#  define PIC32MZ_I2C4_STAT        (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_STAT_OFFSET)
#  define PIC32MZ_I2C4_STATCLR     (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_STATCLR_OFFSET)
#  define PIC32MZ_I2C4_STATSET     (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_STATSET_OFFSET)
#  define PIC32MZ_I2C4_STATINV     (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_STATINV_OFFSET)

#  define PIC32MZ_I2C4_ADD         (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_ADD_OFFSET)
#  define PIC32MZ_I2C4_ADDCLR      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_ADDCLR_OFFSET)
#  define PIC32MZ_I2C4_ADDSET      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_ADDSET_OFFSET)
#  define PIC32MZ_I2C4_ADDINV      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_ADDINV_OFFSET)

#  define PIC32MZ_I2C4_MSK         (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_MSK_OFFSET)
#  define PIC32MZ_I2C4_MSKCLR      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_MSKCLR_OFFSET)
#  define PIC32MZ_I2C4_MSKSET      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_MSKSET_OFFSET)
#  define PIC32MZ_I2C4_MSKINV      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_MSKINV_OFFSET)

#  define PIC32MZ_I2C4_BRG         (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_BRG_OFFSET)
#  define PIC32MZ_I2C4_BRGSET      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_BRGSET_OFFSET)
#  define PIC32MZ_I2C4_BRGCLR      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_BRGCLR_OFFSET)
#  define PIC32MZ_I2C4_BRGINV      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_BRGINV_OFFSET)

#  define PIC32MZ_I2C4_TRN         (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_TRN_OFFSET)
#  define PIC32MZ_I2C4_TRNCLR      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_TRNCLR_OFFSET)
#  define PIC32MZ_I2C4_TRNSET      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_TRNSET_OFFSET)
#  define PIC32MZ_I2C4_TRNINV      (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_TRNINV_OFFSET)

#  define PIC32MZ_I2C4_RCV         (PIC32MZ_I2C4_K1BASE+PIC32MZ_I2C_RCV_OFFSET)
#endif

#if CHIP_NI2C > 4
#  define PIC32MZ_I2C5_CON         (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_CON_OFFSET)
#  define PIC32MZ_I2C5_CONCLR      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_CONCLR_OFFSET)
#  define PIC32MZ_I2C5_CONSET      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_CONSET_OFFSET)
#  define PIC32MZ_I2C5_CONINV      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_CONINV_OFFSET)

#  define PIC32MZ_I2C5_STAT        (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_STAT_OFFSET)
#  define PIC32MZ_I2C5_STATCLR     (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_STATCLR_OFFSET)
#  define PIC32MZ_I2C5_STATSET     (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_STATSET_OFFSET)
#  define PIC32MZ_I2C5_STATINV     (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_STATINV_OFFSET)

#  define PIC32MZ_I2C5_ADD         (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_ADD_OFFSET)
#  define PIC32MZ_I2C5_ADDCLR      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_ADDCLR_OFFSET)
#  define PIC32MZ_I2C5_ADDSET      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_ADDSET_OFFSET)
#  define PIC32MZ_I2C5_ADDINV      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_ADDINV_OFFSET)

#  define PIC32MZ_I2C5_MSK         (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_MSK_OFFSET)
#  define PIC32MZ_I2C5_MSKCLR      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_MSKCLR_OFFSET)
#  define PIC32MZ_I2C5_MSKSET      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_MSKSET_OFFSET)
#  define PIC32MZ_I2C5_MSKINV      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_MSKINV_OFFSET)

#  define PIC32MZ_I2C5_BRG         (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_BRG_OFFSET)
#  define PIC32MZ_I2C5_BRGSET      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_BRGSET_OFFSET)
#  define PIC32MZ_I2C5_BRGCLR      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_BRGCLR_OFFSET)
#  define PIC32MZ_I2C5_BRGINV      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_BRGINV_OFFSET)

#  define PIC32MZ_I2C5_TRN         (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_TRN_OFFSET)
#  define PIC32MZ_I2C5_TRNCLR      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_TRNCLR_OFFSET)
#  define PIC32MZ_I2C5_TRNSET      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_TRNSET_OFFSET)
#  define PIC32MZ_I2C5_TRNINV      (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_TRNINV_OFFSET)

#  define PIC32MZ_I2C5_RCV         (PIC32MZ_I2C5_K1BASE+PIC32MZ_I2C_RCV_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************/

/* I2C control register */

#define I2C_CON_SEN                (1 << 0)  /* Bit 0:  Start condition enable (Master mode) */
#define I2C_CON_RSEN               (1 << 1)  /* Bit 1:  Restart condition enable (Master mode) */
#define I2C_CON_PEN                (1 << 2)  /* Bit 2:  Stop condition enable (Master mode) */
#define I2C_CON_RCEN               (1 << 3)  /* Bit 3:  Receive enable (Master mode) */
#define I2C_CON_ACKEN              (1 << 4)  /* Bit 4:  Acknowledge sequence enable (Master mode) */
#define I2C_CON_ACKDT              (1 << 5)  /* Bit 5:  Acknowledge data (Master mode) */
#define I2C_CON_STREN              (1 << 6)  /* Bit 6:  SCL clock stretch enable (Slave mode) */
#define I2C_CON_GCEN               (1 << 7)  /* Bit 7:  General call enable (Slave mode) */
#define I2C_CON_SMEN               (1 << 8)  /* Bit 8:  SMBus input levels disable */
#define I2C_CON_DISSLW             (1 << 9)  /* Bit 9:  Slew rate control disable */
#define I2C_CON_A10M               (1 << 10) /* Bit 10: 10-bit slave addrewss flag */
#define I2C_CON_STRICT             (1 << 11) /* Bit 11: Strict I2C reserved address rules enable (Slave mode) */
#define I2C_CON_SCLREL             (1 << 12) /* Bit 12: SCL release control */
#define I2C_CON_SIDL               (1 << 13) /* Bit 13: Stop in idle mode */
                                             /* Bit 14: Reserved */
#define I2C_CON_ON                 (1 << 15) /* Bit 15: I2C enable */
#define I2C_CON_DHEN               (1 << 16) /* Bit 16: Data Hold Enable bit (Slave mode) */
#define I2C_CON_AHEN               (1 << 17) /* Bit 17: Address Hold Enable bit (Slave mode) */
#define I2C_CON_SBCDE              (1 << 18) /* Bit 18: Slave Mode Bus Collision Detect Enable (Slave mode) */
#define I2C_CON_SDAHT              (1 << 19) /* Bit 19: SDA Hold Time Selection bit */
#define I2C_CON_BOEN               (1 << 20) /* Bit 20: Buffer Overwrite Enable bit (I2C Slave mode only) */
#define I2C_CON_SCIE               (1 << 21) /* Bit 21: Start Condition Interrupt Enable (Slave mode) */
#define I2C_CON_PCIE               (1 << 22) /* Bit 22: Stop Condition Interrupt Enable (Slave mode) */
                                             /* Bits 23-31: Reserved */
#define I2C_CON_IDLEMASK           (I2C_CON_SEN | I2C_CON_RSEN | I2C_CON_PEN |\
                                    I2C_CON_RCEN | I2C_CON_ACKEN)

/* I2C status register */

#define I2C_STAT_TBF               (1 << 0)  /* Bit 0:  Transmit buffer full */
#define I2C_STAT_RBF               (1 << 1)  /* Bit 1:  Receive buffer full */
#define I2C_STAT_RW                (1 << 2)  /* Bit 2:  Read/write information (Slave mode) */
#define I2C_STAT_S                 (1 << 3)  /* Bit 3:  Start */
#define I2C_STAT_P                 (1 << 4)  /* Bit 4:  Stop */
#define I2C_STAT_DA                (1 << 5)  /* Bit 5:  Data/address (Slave mode) */
#define I2C_STAT_I2COV             (1 << 6)  /* Bit 6:  I2C overflow status */
#define I2C_STAT_IWCOL             (1 << 7)  /* Bit 7:  Write collision detect */
#define I2C_STAT_ADD10             (1 << 8)  /* Bit 8:  10-bit address status */
#define I2C_STAT_GCSTAT            (1 << 9)  /* Bit 9:  General call status */
#define I2C_STAT_BCL               (1 << 10) /* Bit 10: Master bus collision detect */
                                             /* Bits 11-12 */
#define I2C_STAT_ACKTIM            (1 << 13) /* Bit 13: Acknowledge Time Status bit (Slave mode) */
#define I2C_STAT_TRSTAT            (1 << 14) /* Bit 14: Transmit status (Master mode) */
#define I2C_STAT_ACKSTAT           (1 << 15) /* Bit 15: Acknowledge status (Master mode) */

/* I2C address register */

#define I2C_ADD_MASK               0x000003ff /* 10-bit I2C address */

/* I2C address mask register */

#define I2C_MSK_MASK               0x000003ff /* 10-bit I2C address mask */

/* Baud rate generator reload register */

#define I2C_BRG_MASK               0x0000ffff /* 16-bit I2C BRG value */

/* I2C transmit register */

#define I2C_TRN_MASK               0x000000ff /* 8-bit transmit data */

/* I2C receive buffer register */

#define I2C_RCV_MASK               0x000000ff /* 8-bit receive data */

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
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_I2C_H */
