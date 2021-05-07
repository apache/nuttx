/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_dac.h
 * Definitions for stm32H7 dac
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_DAC_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_DAC1_CR_OFFSET        0x0000  /* control register */
#define STM32_DAC1_SWTRGR_OFFSET    0x0004  /* software trigger register */
#define STM32_DAC1_DHR12R1_OFFSET   0x0008  /* ch1 12-bit right alinged data hold register */
#define STM32_DAC1_DHR12L1_OFFSET   0x000C  /* ch1 12-bit left alinged data hold register */
#define STM32_DAC1_DHR8R1_OFFSET    0x0010  /* ch1 8-bit right alinged data hold register */
#define STM32_DAC1_DHR12R2_OFFSET   0x0014  /* ch2 12-bit right alinged data hold register */
#define STM32_DAC1_DHR12L2_OFFSET   0x0018  /* ch2 12-bit left alinged data hold register */
#define STM32_DAC1_DHR8R2_OFFSET    0x001C  /* ch2 8-bit right alinged data hold register */
#define STM32_DAC1_DHR12RD_OFFSET   0x0020  /* dual 12-bit right alinged data hold register */
#define STM32_DAC1_DHR12LD_OFFSET   0x0024  /* dual 2 12-bit left alinged data hold register */
#define STM32_DAC1_DHR8RD_OFFSET    0x0028  /* dual 2 8-bit right alinged data hold register */
#define STM32_DAC1_DOR1_OFFSET      0x002C  /* ch1 data output register */
#define STM32_DAC1_DOR2_OFFSET      0x0030  /* ch2 data output register */
#define STM32_DAC1_SR_OFFSET        0x0034  /* status register */
#define STM32_DAC1_CCR_OFFSET       0x0038  /* calibration control register */
#define STM32_DAC1_MCR_OFFSET       0x003C  /* mode control register */
#define STM32_DAC1_SHSR1_OFFSET     0x0040  /* ch1 sample and hold sample time register */
#define STM32_DAC1_SHSR2_OFFSET     0x0044  /* ch2 sample and hold sample time register */
#define STM32_DAC1_SHHR_OFFSET      0x0048  /* sample and hold time register */
#define STM32_DAC1_SHRR_OFFSET      0x004C  /* sample and hold refresh time register */

/* Register Addresses *******************************************************/

#define STM32_DAC1_CR               (STM32_DAC1_BASE + STM32_DAC1_CR_OFFSET)
#define STM32_DAC1_SWTRGR           (STM32_DAC1_BASE + STM32_DAC1_SWTRGR_OFFSET)
#define STM32_DAC1_DHR12R1          (STM32_DAC1_BASE + STM32_DAC1_DHR12R1_OFFSET)
#define STM32_DAC1_DHR12L1          (STM32_DAC1_BASE + STM32_DAC1_DHR12L1_OFFSET)
#define STM32_DAC1_DHR8R1           (STM32_DAC1_BASE + STM32_DAC1_DHR8R1_OFFSET)
#define STM32_DAC1_DHR12R2          (STM32_DAC1_BASE + STM32_DAC1_DHR12R2_OFFSET)
#define STM32_DAC1_DHR12L2          (STM32_DAC1_BASE + STM32_DAC1_DHR12L2_OFFSET)
#define STM32_DAC1_DHR8R2           (STM32_DAC1_BASE + STM32_DAC1_DHR8R2_OFFSET)
#define STM32_DAC1_DHR12RD          (STM32_DAC1_BASE + STM32_DAC1_DHR12RD_OFFSET)
#define STM32_DAC1_DHR12LD          (STM32_DAC1_BASE + STM32_DAC1_DHR12LD_OFFSET)
#define STM32_DAC1_DHR8RD           (STM32_DAC1_BASE + STM32_DAC1_DHR8RD_OFFSET)
#define STM32_DAC1_DOR1             (STM32_DAC1_BASE + STM32_DAC1_DOR1_OFFSET)
#define STM32_DAC1_DOR2             (STM32_DAC1_BASE + STM32_DAC1_DOR2_OFFSET)
#define STM32_DAC1_SR               (STM32_DAC1_BASE + STM32_DAC1_SR_OFFSET)
#define STM32_DAC1_CCR              (STM32_DAC1_BASE + STM32_DAC1_CCR_OFFSET)
#define STM32_DAC1_MCR              (STM32_DAC1_BASE + STM32_DAC1_MCR_OFFSET)
#define STM32_DAC1_SHSR1            (STM32_DAC1_BASE + STM32_DAC1_SHSR1_OFFSET)
#define STM32_DAC1_SHSR2            (STM32_DAC1_BASE + STM32_DAC1_SHSR2_OFFSET)
#define STM32_DAC1_SHHR             (STM32_DAC1_BASE + STM32_DAC1_SHHR_OFFSET)
#define STM32_DAC1_SHRR             (STM32_DAC1_BASE + STM32_DAC1_SHRR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* DAC control register */

#define DAC_CR_EN1                  (1 << 0)  /* Bit 0: ch1 enable */
#define DAC_CR_TEN1                 (1 << 1)  /* Bit 1: ch1 trigger enable */
#define DAC_CR_TSEL1_SHIFT          (1 << 2)  /* Bits 5-2: ch1 triger sel */
#define DAC_CR_TSEL1_MASK           (15 << DAC_CR_TSEL1_SHIFT)
#define DAC_CR_WAVE1_SHIFT          (1 << 6)  /* Bits 7-6: ch1 wave enable */
#define DAC_CR_WAVE1_MASK           (3 << DAC_CR_WAVE1_SHIFT)
#define DAC_CR_MAMP1_SHIFT          (1 << 8)  /* Bits 11-8: ch1 mask/amplitude */
#define DAC_CR_MAMP1_MASK           (15 << DAC_CR_MAMP1_SHIFT)
#define DAC_CR_DMAEN1               (1 << 12)  /* Bit 12: ch1 dma enable */
#define DAC_CR_DMAUDRIE1            (1 << 13)  /* Bit 13: ch1 dma underrun interrupt enable */
#define DAC_CR_CEN1                 (1 << 14)  /* Bit 14: ch1 calibration enable */
#define DAC_CR_EN2                  (1 << 16)  /* Bit 16: ch2 enable */
#define DAC_CR_TEN2                 (1 << 17)  /* Bit 17: ch2 trigger enable */
#define DAC_CR_TSEL2_SHIFT          (1 << 18)  /* Bits 28-21: ch2 triger sel */
#define DAC_CR_TSEL2_MASK           (15 << DAC_CR_TSEL2_SHIFT)
#define DAC_CR_WAVE2_SHIFT          (1 << 22)  /* Bits 23-22: ch2 wave enable */
#define DAC_CR_WAVE2_MASK           (3 << DAC_CR_WAVE2_SHIFT)
#define DAC_CR_MAMP2_SHIFT          (1 << 24)  /* Bits 27-24: ch2 mask/amplitude */
#define DAC_CR_MAMP2_MASK           (15 << DAC_CR_MAMP2_SHIFT)
#define DAC_CR_DMAEN2               (1 << 28) /* Bit 28: ch2 dma enable */
#define DAC_CR_DMAUDRIE2            (1 << 29) /* Bit 29: ch2 dma underrun interrupt enable */
#define DAC_CR_CEN2                 (1 << 30) /* Bit 30: ch2 calibration enable */

/* DAC software trigger register */

#define DAC_SWTRGR_SWTRIG1          (1 << 0)  /* Bit 0: ch1 software trigger */
#define DAC_SWTRGR_SWTRIG2          (1 << 1)  /* Bit 1: ch2 software trigger */

/* DAC status register */

#define DAC_SR_DMAUDR1              (1 << 13)  /* Bit 13: ch1 dma underrun flag */
#define DAC_SR_CAL_FLAG1            (1 << 14)  /* Bit 14: ch1 calibration status */
#define DAC_SR_BWST1                (1 << 15)  /* Bit 15: ch1 busy writing */
#define DAC_SR_DMAUDR2              (1 << 29)  /* Bit 29: ch2 dma underrun flag */
#define DAC_SR_CAL_FLAG2            (1 << 30)  /* Bit 30: ch2 calibration status */
#define DAC_SR_BWST2                (1 << 31)  /* Bit 31: ch2 busy writing */

/* DAC mode control register */
#define DAC_MCR_MODE1_SHIFT         (1 << 0)                   /* Bits 2-0: ch1 mode */
#define DAC_MCR_MODE1_MASK          (7 << DAC_MCR_MODE1_SHIFT)
#define DAC_MCR_MODE1_NORM          (4 << DAC_MCR_MODE1_SHIFT) /* ch1 normal mode */
#define DAC_MCR_MODE1_SAMPLE_HOLD   (0 << DAC_MCR_MODE1_SHIFT) /* ch1 sample and hold mode */
#define DAC_MCR_MODE1_DIS_BUFFER    (2 << DAC_MCR_MODE1_SHIFT) /* ch1 buffer disabled */
#define DAC_MCR_MODE1_EXT           (0 << DAC_MCR_MODE1_SHIFT) /* ch1 to external pin */
#define DAC_MCR_MODE1_ON_CHIP       (1 << DAC_MCR_MODE1_SHIFT) /* ch1 to on chip and external */
#define DAC_MCR_MODE2_SHIFT         (1 << 16)                  /* Bits 18-16: ch2 mode */
#define DAC_MCR_MODE2_MASK          (7 << DAC_MCR_MODE2_SHIFT)
#define DAC_MCR_MODE2_NORM          (4 << DAC_MCR_MODE2_SHIFT) /* ch2 normal mode */
#define DAC_MCR_MODE2_SAMPLE_HOLD   (0 << DAC_MCR_MODE2_SHIFT) /* ch2 sample and hold mode */
#define DAC_MCR_MODE2_DIS_BUFFER    (2 << DAC_MCR_MODE2_SHIFT) /* ch2 buffer disabled */
#define DAC_MCR_MODE2_EXT           (0 << DAC_MCR_MODE2_SHIFT) /* ch2 to external pin */
#define DAC_MCR_MODE2_ON_CHIP       (1 << DAC_MCR_MODE2_SHIFT) /* ch2 to on chip and external */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_DAC_H */
