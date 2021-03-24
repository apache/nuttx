/****************************************************************************
 * arch/arm/src/sama5/hardware/_sama5d2x_pio.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D2X_PIO_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D2X_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Misc Helper Definitions **************************************************/

#define PIOA                       (0)
#define PIOB                       (1)
#define PIOC                       (2)
#define PIOD                       (3)
#define PIOE                       (4)

/* PIO register offsets *****************************************************/

#define SAM_PIO_IOGROUP_OFFSET(n)  (0x0000 + ((n) << 6))
#  define SAM_PIO_IOGROUPA_OFFSET  0x0000
#  define SAM_PIO_IOGROUPB_OFFSET  0x0040
#  define SAM_PIO_IOGROUPC_OFFSET  0x0080
#  define SAM_PIO_IOGROUPD_OFFSET  0x00c0

#define SAM_PIO_MSKR_OFFSET        0x0000 /* PIO Mask Register */
#define SAM_PIO_CFGR_OFFSET        0x0004 /* PIO Configuration Register */
#define SAM_PIO_PDSR_OFFSET        0x0008 /* PIO Pin Data Status Register */
#define SAM_PIO_LOCKSR_OFFSET      0x000c /* PIO Lock Status Register */
#define SAM_PIO_SODR_OFFSET        0x0010 /* PIO Set Output Data Register */
#define SAM_PIO_CODR_OFFSET        0x0014 /* PIO Clear Output Data Register */
#define SAM_PIO_ODSR_OFFSET        0x0018 /* PIO Output Data Status Register */
                                          /* 0x001c: Reserved */
#define SAM_PIO_IER_OFFSET         0x0020 /* PIO Interrupt Enable Register */
#define SAM_PIO_IDR_OFFSET         0x0024 /* PIO Interrupt Disable Register */
#define SAM_PIO_IMR_OFFSET         0x0028 /* PIO Interrupt Mask Register */
#define SAM_PIO_ISR_OFFSET         0x002c /* PIO Interrupt Status Register */
                                          /* 0x0030-0x0038: Reserved */
#define SAM_PIO_IOFR_OFFSET        0x003c /* PIO I/O Freeze Register */
                                          /* 0x0040-0x05dc: Reserved */

#define SAM_PIO_WPMR_OFFSET        0x05e0 /* PIO Write Protection Mode Register */
#define SAM_PIO_WPSR_OFFSET        0x05e4 /* PIO Write Protection Status Register */
                                          /* 0x05e8–0x0ffc: Reserved */

#define SAM_SPIO_IOGROUP_OFFSET(n) (0x1000 + ((n) << 6))
#  define SAM_SPIO_IOGROUPA_OFFSET 0x1000
#  define SAM_SPIO_IOGROUPB_OFFSET 0x1040
#  define SAM_SPIO_IOGROUPC_OFFSET 0x1080
#  define SAM_SPIO_IOGROUPD_OFFSET 0x10c0

#define SAM_SPIO_MSKR_OFFSET       0x0000 /* Secure PIO Mask Register */
#define SAM_SPIO_CFGR_OFFSET       0x0004 /* Secure PIO Configuration Register */
#define SAM_SPIO_PDSR_OFFSET       0x0008 /* Secure PIO Pin Data Status Register */
#define SAM_SPIO_LOCKSR_OFFSET     0x000c /* Secure PIO Lock Status Register */
#define SAM_SPIO_SODR_OFFSET       0x0010 /* Secure PIO Set Output Data Register */
#define SAM_SPIO_CODR_OFFSET       0x0014 /* Secure PIO Clear Output Data Register */
#define SAM_SPIO_ODSR_OFFSET       0x0018 /* Secure PIO Output Data Status Register */
                                          /* 0x001c: Reserved */
#define SAM_SPIO_IER_OFFSET        0x0020 /* Secure PIO Interrupt Enable Register */
#define SAM_SPIO_IDR_OFFSET        0x0024 /* Secure PIO Interrupt Disable Register */
#define SAM_SPIO_IMR_OFFSET        0x0028 /* Secure PIO Interrupt Mask Register */
#define SAM_SPIO_ISR_OFFSET        0x002c /* Secure PIO Interrupt Status Register */
#define SAM_SPIO_SIONR_OFFSET      0x0030 /* Secure PIO Set I/O Non-Secure Register */
#define SAM_SPIO_SIOSR_OFFSET      0x0034 /* Secure PIO Set I/O Secure Register */
#define SAM_SPIO_IOSSR_OFFSET      0x0038 /* Secure PIO I/O Security Status Register */
#define SAM_SPIO_IOFR_OFFSET       0x003c /* Secure PIO I/O Freeze Register */
                                          /* 0x1400–0x14fc: Reserved */

#define SAM_SPIO_SCDR_OFFSET       0x1500 /* Secure PIO Slow Clock Divider Debouncing Register */
                                          /* 0x1504-0x15dc: Reserved */
#define SAM_SPIO_WPMR_OFFSET       0x15e0 /* Secure PIO Write Protection Mode Register */
#define SAM_SPIO_WPSR_OFFSET       0x15e4 /* Secure PIO Write Protection Status Register */

/* PIO register addresses ***************************************************/

#define SAM_PIO_IOGROUP_VBASE(n)   (SAM_PIO_VBASE+SAM_PIO_IOGROUP_OFFSET(n))
#  define SAM_PIO_IOGROUPA_VBASE   (SAM_PIO_VBASE+SAM_PIO_IOGROUPA_OFFSET)
#  define SAM_PIO_IOGROUPB_VBASE   (SAM_PIO_VBASE+SAM_PIO_IOGROUPB_OFFSET)
#  define SAM_PIO_IOGROUPC_VBASE   (SAM_PIO_VBASE+SAM_PIO_IOGROUPC_OFFSET)
#  define SAM_PIO_IOGROUPD_VBASE   (SAM_PIO_VBASE+SAM_PIO_IOGROUPD_OFFSET)

#define SAM_PIOA_MSKR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_MSKR_OFFSET)
#define SAM_PIOA_CFGR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_CFGR_OFFSET)
#define SAM_PIOA_PDSR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOA_LOCKSR            (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOA_SODR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOA_CODR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOA_ODSR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOA_IER               (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOA_IDR               (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOA_IMR               (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOA_ISR               (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOA_IOFR              (SAM_PIO_IOGROUPA_VBASE+SAM_PIO_IOFR_OFFSET)

#define SAM_PIOB_MSKR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_MSKR_OFFSET)
#define SAM_PIOB_CFGR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_CFGR_OFFSET)
#define SAM_PIOB_PDSR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOB_LOCKSR            (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOB_SODR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOB_CODR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOB_ODSR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOB_IER               (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOB_IDR               (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOB_IMR               (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOB_ISR               (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOB_IOFR              (SAM_PIO_IOGROUPB_VBASE+SAM_PIO_IOFR_OFFSET)

#define SAM_PIOC_MSKR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_MSKR_OFFSET)
#define SAM_PIOC_CFGR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_CFGR_OFFSET)
#define SAM_PIOC_PDSR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOC_LOCKSR            (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOC_SODR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOC_CODR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOC_ODSR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOC_IER               (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOC_IDR               (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOC_IMR               (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOC_ISR               (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOC_IOFR              (SAM_PIO_IOGROUPC_VBASE+SAM_PIO_IOFR_OFFSET)

#define SAM_PIOD_MSKR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_MSKR_OFFSET)
#define SAM_PIOD_CFGR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_CFGR_OFFSET)
#define SAM_PIOD_PDSR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_PDSR_OFFSET)
#define SAM_PIOD_LOCKSR            (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_LOCKSR_OFFSET)
#define SAM_PIOD_SODR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_SODR_OFFSET)
#define SAM_PIOD_CODR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_CODR_OFFSET)
#define SAM_PIOD_ODSR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_ODSR_OFFSET)
#define SAM_PIOD_IER               (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_IER_OFFSET)
#define SAM_PIOD_IDR               (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_IDR_OFFSET)
#define SAM_PIOD_IMR               (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_IMR_OFFSET)
#define SAM_PIOD_ISR               (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_ISR_OFFSET)
#define SAM_PIOD_IOFR              (SAM_PIO_IOGROUPD_VBASE+SAM_PIO_IOFR_OFFSET)

#define SAM_PIO_WPMR               (SAM_PIO_VBASE+SAM_PIO_WPMR_OFFSET)
#define SAM_PIO_WPSR               (SAM_PIO_VBASE+SAM_PIO_WPSR_OFFSET)

#define SAM_SPIO_IOGROUP_VBASE(n)  (SAM_PIO_VBASE+SAM_SPIO_IOGROUP_OFFSET(n))
#  define SAM_SPIO_IOGROUPA_VBASE  (SAM_PIO_VBASE+SAM_SPIO_IOGROUPA_OFFSET)
#  define SAM_SPIO_IOGROUPB_VBASE  (SAM_PIO_VBASE+SAM_SPIO_IOGROUPB_OFFSET)
#  define SAM_SPIO_IOGROUPC_VBASE  (SAM_PIO_VBASE+SAM_SPIO_IOGROUPC_OFFSET)
#  define SAM_SPIO_IOGROUPD_VBASE  (SAM_PIO_VBASE+SAM_SPIO_IOGROUPD_OFFSET)

#define SAM_SPIOA_MSKR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_MSKR_OFFSET)
#define SAM_SPIOA_CFGR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_CFGR_OFFSET)
#define SAM_SPIOA_PDSR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_PDSR_OFFSET)
#define SAM_SPIOA_LOCKSR           (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_LOCKSR_OFFSET)
#define SAM_SPIOA_SODR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_SODR_OFFSET)
#define SAM_SPIOA_CODR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_CODR_OFFSET)
#define SAM_SPIOA_ODSR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_ODSR_OFFSET)
#define SAM_SPIOA_IER              (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_IER_OFFSET)
#define SAM_SPIOA_IDR              (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_IDR_OFFSET)
#define SAM_SPIOA_IMR              (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_IMR_OFFSET)
#define SAM_SPIOA_ISR              (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_ISR_OFFSET)
#define SAM_SPIOA_SIONR            (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_SIONR_OFFSET)
#define SAM_SPIOA_SIOSR            (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_SIOSR_OFFSET)
#define SAM_SPIOA_IOSSR            (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_IOSSR_OFFSET)
#define SAM_SPIOA_IOFR             (SAM_PIO_IOGROUPA_VBASE+SAM_SPIO_IOFR_OFFSET)

#define SAM_SPIOB_MSKR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_MSKR_OFFSET)
#define SAM_SPIOB_CFGR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_CFGR_OFFSET)
#define SAM_SPIOB_PDSR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_PDSR_OFFSET)
#define SAM_SPIOB_LOCKSR           (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_LOCKSR_OFFSET)
#define SAM_SPIOB_SODR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_SODR_OFFSET)
#define SAM_SPIOB_CODR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_CODR_OFFSET)
#define SAM_SPIOB_ODSR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_ODSR_OFFSET)
#define SAM_SPIOB_IER              (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_IER_OFFSET)
#define SAM_SPIOB_IDR              (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_IDR_OFFSET)
#define SAM_SPIOB_IMR              (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_IMR_OFFSET)
#define SAM_SPIOB_ISR              (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_ISR_OFFSET)
#define SAM_SPIOB_SIONR            (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_SIONR_OFFSET)
#define SAM_SPIOB_SIOSR            (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_SIOSR_OFFSET)
#define SAM_SPIOB_IOSSR            (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_IOSSR_OFFSET)
#define SAM_SPIOB_IOFR             (SAM_PIO_IOGROUPB_VBASE+SAM_SPIO_IOFR_OFFSET)

#define SAM_SPIOC_MSKR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_MSKR_OFFSET)
#define SAM_SPIOC_CFGR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_CFGR_OFFSET)
#define SAM_SPIOC_PDSR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_PDSR_OFFSET)
#define SAM_SPIOC_LOCKSR           (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_LOCKSR_OFFSET)
#define SAM_SPIOC_SODR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_SODR_OFFSET)
#define SAM_SPIOC_CODR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_CODR_OFFSET)
#define SAM_SPIOC_ODSR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_ODSR_OFFSET)
#define SAM_SPIOC_IER              (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_IER_OFFSET)
#define SAM_SPIOC_IDR              (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_IDR_OFFSET)
#define SAM_SPIOC_IMR              (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_IMR_OFFSET)
#define SAM_SPIOC_ISR              (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_ISR_OFFSET)
#define SAM_SPIOC_SIONR            (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_SIONR_OFFSET)
#define SAM_SPIOC_SIOSR            (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_SIOSR_OFFSET)
#define SAM_SPIOC_IOSSR            (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_IOSSR_OFFSET)
#define SAM_SPIOC_IOFR             (SAM_PIO_IOGROUPC_VBASE+SAM_SPIO_IOFR_OFFSET)

#define SAM_SPIOD_MSKR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_MSKR_OFFSET)
#define SAM_SPIOD_CFGR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_CFGR_OFFSET)
#define SAM_SPIOD_PDSR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_PDSR_OFFSET)
#define SAM_SPIOD_LOCKSR           (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_LOCKSR_OFFSET)
#define SAM_SPIOD_SODR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_SODR_OFFSET)
#define SAM_SPIOD_CODR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_CODR_OFFSET)
#define SAM_SPIOD_ODSR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_ODSR_OFFSET)
#define SAM_SPIOD_IER              (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_IER_OFFSET)
#define SAM_SPIOD_IDR              (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_IDR_OFFSET)
#define SAM_SPIOD_IMR              (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_IMR_OFFSET)
#define SAM_SPIOD_ISR              (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_ISR_OFFSET)
#define SAM_SPIOD_SIONR            (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_SIONR_OFFSET)
#define SAM_SPIOD_SIOSR            (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_SIOSR_OFFSET)
#define SAM_SPIOD_IOSSR            (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_IOSSR_OFFSET)
#define SAM_SPIOD_IOFR             (SAM_PIO_IOGROUPD_VBASE+SAM_SPIO_IOFR_OFFSET)

#define SAM_SPIO_SCDR              (SAM_PIO_VBASE+SAM_SPIO_SCDR_OFFSET)
#define SAM_SPIO_WPMR              (SAM_PIO_VBASE+SAM_SPIO_WPMR_OFFSET)
#define SAM_SPIO_WPSR              (SAM_PIO_VBASE+SAM_SPIO_WPSR_OFFSET)

/* PIO register bit definitions *********************************************/

/* Common bit definitions for many IO registers (exceptions follow) */

#define PIO(n)                     (1 << (n)) /* Bit n: PIO n, n=0-31 */

/* PIO Configuration Register and  Secure PIO Configuration Register */

#define PIO_CFGR_FUNC_SHIFT        (0)       /* Bits 0-2: I/O Line Function */
#define PIO_CFGR_FUNC_MASK         (7 << PIO_CFGR_FUNC_SHIFT)
#  define PIO_CFGR_FUNC_GPIO       (0 << PIO_CFGR_FUNC_SHIFT) /* Select PIO mode */
#  define PIO_CFGR_FUNC_PERIPH(n)  ((uint32_t)(n) << PIO_CFGR_FUNC_SHIFT)
#  define PIO_CFGR_FUNC_PERIPHA    (1 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral A */
#  define PIO_CFGR_FUNC_PERIPHB    (2 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral B */
#  define PIO_CFGR_FUNC_PERIPHC    (3 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral C */
#  define PIO_CFGR_FUNC_PERIPHD    (4 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral D */
#  define PIO_CFGR_FUNC_PERIPHE    (5 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral E */
#  define PIO_CFGR_FUNC_PERIPHF    (6 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral F */
#  define PIO_CFGR_FUNC_PERIPHG    (7 << PIO_CFGR_FUNC_SHIFT) /* Select peripheral G */

#define PIO_CFGR_DIR               (1 << 8)  /* Bit 8:  Direction */
#  define PIO_CFGR_DIR_INPUT       (0)       /*         0=Input */
#  define PIO_CFGR_DIR_OUTPUT      (1 << 8)  /*         1=Output */
#define PIO_CFGR_PUEN              (1 << 9)  /* Bit 9:  Pull-Up Enable */
#define PIO_CFGR_PDEN              (1 << 10) /* Bit 10: Pull-Down Enable */
#define PIO_CFGR_IFEN              (1 << 12) /* Bit 12: Input Filter Enable */
#define PIO_CFGR_IFSCEN            (1 << 13) /* Bit 13: Input Filter Slow Clock Enable */
#define PIO_CFGR_OPD               (1 << 14) /* Bit 14: Open-Drain */
#define PIO_CFGR_SCHMITT           (1 << 15) /* Bit 15: Schmitt Trigger */
#define PIO_CFGR_DRVSTR_SHIFT      (16)      /* Bits 16-17: Drive Strength */
#define PIO_CFGR_DRVSTR_MASK       (3 << PIO_CFGR_DRVSTR_SHIFT)
#  define PIO_CFGR_DRVSTR_LOW      (0 << PIO_CFGR_DRVSTR_SHIFT) /* Low drive */
#  define PIO_CFGR_DRVSTR_MED      (2 << PIO_CFGR_DRVSTR_SHIFT) /* Medium drive */
#  define PIO_CFGR_DRVSTR_HIGH     (3 << PIO_CFGR_DRVSTR_SHIFT) /* High drive */

#define PIO_CFGR_EVTSEL_SHIFT      (24)       /* Bits 24-26: Event Selection */
#define PIO_CFGR_EVTSEL_MASK       (7 << PIO_CFGR_EVTSEL_SHIFT)
#  define PIO_CFGR_EVTSEL_FALLING  (0 << PIO_CFGR_EVTSEL_SHIFT) /* Event detection on input falling edge */
#  define PIO_CFGR_EVTSEL_RISING   (1 << PIO_CFGR_EVTSEL_SHIFT) /* Event detection on input rising edge */
#  define PIO_CFGR_EVTSEL_BOTH     (2 << PIO_CFGR_EVTSEL_SHIFT) /* Event detection on input both edge */
#  define PIO_CFGR_EVTSEL_LOW      (3 << PIO_CFGR_EVTSEL_SHIFT) /* Event detection on low level input */
#  define PIO_CFGR_EVTSEL_HIGH     (4 << PIO_CFGR_EVTSEL_SHIFT) /* Event detection on high level input */

#define PIO_CFGR_PCFS              (1 << 29) /* Bit 29: Physical Configuration Freeze Status */
#define PIO_CFGR_ICFS              (1 << 30) /* Bit 30: Interrupt Configuration Freeze Status */

/* PIO I/O Freeze Register and Secure PIO I/O Freeze Register */

#define PIO_IOFR_FPHY              (1 << 0)  /* Bit 0:  Freeze Physical Configuration */
#define PIO_IOFR_FINT              (1 << 1)  /* Bit 1:  Freeze Interrupt Configuration */
#define PIO_IOFR_FRZKEY_SHIFT      (8)       /* Bits 8-31: Freeze Key */
#define PIO_IOFR_FRZKEY_MASK       (0x00ffffff << PIO_IOFR_FRZKEY_SHIFT)
#  define PIO_IOFR_FRZKEY          (0x00494F46 << PIO_IOFR_FRZKEY_SHIFT) /* ""IOF" */

/* PIO Write Protection Mode Register and Secure PIO Write Protection Mode
 * Register
 */

#define PIO_WPMR_WPEN              (1 << 0)  /* Bit 0:  Write Protection Enable */
#define PIO_WPMR_WPITEN            (1 << 1)  /* Bit 1:  Write Protection Interrupt Enable */
#define PIO_WPMR_WPKEY_SHIFT       (8)       /* Bits 8-31: Write Protection Key */
#define PIO_WPMR_WPKEY_MASK        (0x00ffffff << PIO_WPMR_WPKEY_SHIFT)
#  define PIO_WPMR_WPKEY           (0x0050494f << PIO_WPMR_WPKEY_SHIFT) /* "PIO" */

/* PIO Write Protection Status Register and Secure PIO Write Protection
 * Status Register
 */

#define PIO_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protection Violation Status */
#define PIO_WPSR_WPVSRC_SHIFT      (8)       /* Bits 8-23: Write Protection Violation Source */
#define PIO_WPSR_WPVSRC_MASK       (0xffff << PIO_WPSR_WPVSRC_SHIFT)

/* Secure PIO Slow Clock Divider Debouncing Register */

#define SPIO_SCDR_DIV_SHIFT        (0)       /* Bits 0-13: Slow Clock Divider Selection for Debouncing */
#define SPIO_SCDR_DIV_MASK         (0x3fff << SPIO_SCDR_DIV_SHIFT)
#  define SPIO_SCDR_DIV(n)         ((uint32_t)(n) << SPIO_SCDR_DIV_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D2X_PIO_H */
