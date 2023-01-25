/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_adc_ver2.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER2_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define IMXRT_LPADC_VERID_OFFSET    0x000000  /* Version ID Register */
#define IMXRT_LPADC_PARAM_OFFSET    0x000004  /* Parameter Register */
#define IMXRT_LPADC_CTRL_OFFSET     0x000010  /* LPADC Control Register */
#define IMXRT_LPADC_STAT_OFFSET     0x000014  /* LPADC Status Register */
#define IMXRT_LPADC_IE_OFFSET       0x000018  /* Interrupt Enable Register */
#define IMXRT_LPADC_DE_OFFSET       0x00001c  /* DMA Enable Register */
#define IMXRT_LPADC_CFG_OFFSET      0x000020  /* LPADC Configuration Register */
#define IMXRT_LPADC_PAUSE_OFFSET    0x000024  /* LPADC Pause Register */
#define IMXRT_LPADC_FCTRL_OFFSET    0x000030  /* LPADC FIFO Control Register */
#define IMXRT_LPADC_SWTRIG_OFFSET   0x000034  /* Software Trigger Register */
#define IMXRT_LPADC_CMDL1_OFFSET    0x000100  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH1_OFFSET    0x000104  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL2_OFFSET    0x000108  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH2_OFFSET    0x00010c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL3_OFFSET    0x000110  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH3_OFFSET    0x000114  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL4_OFFSET    0x000118  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH4_OFFSET    0x00011c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL5_OFFSET    0x000120  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH5_OFFSET    0x000124  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL6_OFFSET    0x000128  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH6_OFFSET    0x00012c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL7_OFFSET    0x000130  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH7_OFFSET    0x000134  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL8_OFFSET    0x000138  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH8_OFFSET    0x00013c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL9_OFFSET    0x000140  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH9_OFFSET    0x000144  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL10_OFFSET   0x000148  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH10_OFFSET   0x00014c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL11_OFFSET   0x000150  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH11_OFFSET   0x000154  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL12_OFFSET   0x000158  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH12_OFFSET   0x00015c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL13_OFFSET   0x000160  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH13_OFFSET   0x000164  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL14_OFFSET   0x000168  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH14_OFFSET   0x00016c  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_CMDL15_OFFSET   0x000170  /* LPADC Command Low Buffer Register */
#define IMXRT_LPADC_CMDH15_OFFSET   0x000174  /* LPADC Command High Buffer Register */
#define IMXRT_LPADC_RESFIFO_OFFSET  0x000300  /* LPADC Data Result FIFO Register */
#define IMXRT_LPADC_TCTRL0_OFFSET   0x0000c0  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL1_OFFSET   0x0000c4  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL2_OFFSET   0x0000c8  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL3_OFFSET   0x0000cc  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL4_OFFSET   0x0000d0  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL5_OFFSET   0x0000d4  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL6_OFFSET   0x0000d8  /* Trigger Control Register */
#define IMXRT_LPADC_TCTRL7_OFFSET   0x0000dc  /* Trigger Control Register */
#define IMXRT_LPADC_CV1_OFFSET      0x000200  /* Compare Value Register */
#define IMXRT_LPADC_CV2_OFFSET      0x000204  /* Compare Value Register */
#define IMXRT_LPADC_CV3_OFFSET      0x000208  /* Compare Value Register */
#define IMXRT_LPADC_CV4_OFFSET      0x00020c  /* Compare Value Register */

/* Register definitions *****************************************************/

#define IMXRT_LPADC1_VERID    (IMXRT_LPADC1_BASE + IMXRT_LPADC_VERID_OFFSET)
#define IMXRT_LPADC1_PARAM    (IMXRT_LPADC1_BASE + IMXRT_LPADC_PARAM_OFFSET)
#define IMXRT_LPADC1_CTRL     (IMXRT_LPADC1_BASE + IMXRT_LPADC_CTRL_OFFSET)
#define IMXRT_LPADC1_STAT     (IMXRT_LPADC1_BASE + IMXRT_LPADC_STAT_OFFSET)
#define IMXRT_LPADC1_IE       (IMXRT_LPADC1_BASE + IMXRT_LPADC_IE_OFFSET)
#define IMXRT_LPADC1_DE       (IMXRT_LPADC1_BASE + IMXRT_LPADC_DE_OFFSET)
#define IMXRT_LPADC1_CFG      (IMXRT_LPADC1_BASE + IMXRT_LPADC_CFG_OFFSET)
#define IMXRT_LPADC1_PAUSE    (IMXRT_LPADC1_BASE + IMXRT_LPADC_PAUSE_OFFSET)
#define IMXRT_LPADC1_FCTRL    (IMXRT_LPADC1_BASE + IMXRT_LPADC_FCTRL_OFFSET)
#define IMXRT_LPADC1_SWTRIG   (IMXRT_LPADC1_BASE + IMXRT_LPADC_SWTRIG_OFFSET)
#define IMXRT_LPADC1_CMDL1    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL1_OFFSET)
#define IMXRT_LPADC1_CMDH1    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH1_OFFSET)
#define IMXRT_LPADC1_CMDL2    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL2_OFFSET)
#define IMXRT_LPADC1_CMDH2    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH2_OFFSET)
#define IMXRT_LPADC1_CMDL3    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL3_OFFSET)
#define IMXRT_LPADC1_CMDH3    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH3_OFFSET)
#define IMXRT_LPADC1_CMDL4    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL4_OFFSET)
#define IMXRT_LPADC1_CMDH4    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH4_OFFSET)
#define IMXRT_LPADC1_CMDL5    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL5_OFFSET)
#define IMXRT_LPADC1_CMDH5    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH5_OFFSET)
#define IMXRT_LPADC1_CMDL6    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL6_OFFSET)
#define IMXRT_LPADC1_CMDH6    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH6_OFFSET)
#define IMXRT_LPADC1_CMDL7    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL7_OFFSET)
#define IMXRT_LPADC1_CMDH7    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH7_OFFSET)
#define IMXRT_LPADC1_CMDL8    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL8_OFFSET)
#define IMXRT_LPADC1_CMDH8    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH8_OFFSET)
#define IMXRT_LPADC1_CMDL9    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL9_OFFSET)
#define IMXRT_LPADC1_CMDH9    (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH9_OFFSET)
#define IMXRT_LPADC1_CMDL10   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL10_OFFSET)
#define IMXRT_LPADC1_CMDH10   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH10_OFFSET)
#define IMXRT_LPADC1_CMDL11   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL11_OFFSET)
#define IMXRT_LPADC1_CMDH11   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH11_OFFSET)
#define IMXRT_LPADC1_CMDL12   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL12_OFFSET)
#define IMXRT_LPADC1_CMDH12   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH12_OFFSET)
#define IMXRT_LPADC1_CMDL13   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL13_OFFSET)
#define IMXRT_LPADC1_CMDH13   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH13_OFFSET)
#define IMXRT_LPADC1_CMDL14   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL14_OFFSET)
#define IMXRT_LPADC1_CMDH14   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH14_OFFSET)
#define IMXRT_LPADC1_CMDL15   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDL15_OFFSET)
#define IMXRT_LPADC1_CMDH15   (IMXRT_LPADC1_BASE + IMXRT_LPADC_CMDH15_OFFSET)
#define IMXRT_LPADC1_RESFIFO  (IMXRT_LPADC1_BASE + IMXRT_LPADC_RESFIFO_OFFSET)
#define IMXRT_LPADC1_TCTRL0   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL0_OFFSET)
#define IMXRT_LPADC1_TCTRL1   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL1_OFFSET)
#define IMXRT_LPADC1_TCTRL2   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL2_OFFSET)
#define IMXRT_LPADC1_TCTRL3   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL3_OFFSET)
#define IMXRT_LPADC1_TCTRL4   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL4_OFFSET)
#define IMXRT_LPADC1_TCTRL5   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL5_OFFSET)
#define IMXRT_LPADC1_TCTRL6   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL6_OFFSET)
#define IMXRT_LPADC1_TCTRL7   (IMXRT_LPADC1_BASE + IMXRT_LPADC_TCTRL7_OFFSET)
#define IMXRT_LPADC1_CV1      (IMXRT_LPADC1_BASE + IMXRT_LPADC_CV1_OFFSET)
#define IMXRT_LPADC1_CV2      (IMXRT_LPADC1_BASE + IMXRT_LPADC_CV2_OFFSET)
#define IMXRT_LPADC1_CV3      (IMXRT_LPADC1_BASE + IMXRT_LPADC_CV3_OFFSET)
#define IMXRT_LPADC1_CV4      (IMXRT_LPADC1_BASE + IMXRT_LPADC_CV4_OFFSET)

#define IMXRT_LPADC2_VERID    (IMXRT_LPADC2_BASE + IMXRT_LPADC_VERID_OFFSET)
#define IMXRT_LPADC2_PARAM    (IMXRT_LPADC2_BASE + IMXRT_LPADC_PARAM_OFFSET)
#define IMXRT_LPADC2_CTRL     (IMXRT_LPADC2_BASE + IMXRT_LPADC_CTRL_OFFSET)
#define IMXRT_LPADC2_STAT     (IMXRT_LPADC2_BASE + IMXRT_LPADC_STAT_OFFSET)
#define IMXRT_LPADC2_IE       (IMXRT_LPADC2_BASE + IMXRT_LPADC_IE_OFFSET)
#define IMXRT_LPADC2_DE       (IMXRT_LPADC2_BASE + IMXRT_LPADC_DE_OFFSET)
#define IMXRT_LPADC2_CFG      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CFG_OFFSET)
#define IMXRT_LPADC2_PAUSE    (IMXRT_LPADC2_BASE + IMXRT_LPADC_PAUSE_OFFSET)
#define IMXRT_LPADC2_FCTRL    (IMXRT_LPADC2_BASE + IMXRT_LPADC_FCTRL_OFFSET)
#define IMXRT_LPADC2_SWTRIG   (IMXRT_LPADC2_BASE + IMXRT_LPADC_SWTRIG_OFFSET)
#define IMXRT_LPADC2_CMDL1    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL1_OFFSET)
#define IMXRT_LPADC2_CMDH1    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH1_OFFSET)
#define IMXRT_LPADC2_CMDL2    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL2_OFFSET)
#define IMXRT_LPADC2_CMDH2    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH2_OFFSET)
#define IMXRT_LPADC2_CMDL3    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL3_OFFSET)
#define IMXRT_LPADC2_CMDH3    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH3_OFFSET)
#define IMXRT_LPADC2_CMDL4    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL4_OFFSET)
#define IMXRT_LPADC2_CMDH4    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH4_OFFSET)
#define IMXRT_LPADC2_CMDL5    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL5_OFFSET)
#define IMXRT_LPADC2_CMDH5    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH5_OFFSET)
#define IMXRT_LPADC2_CMDL6    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL6_OFFSET)
#define IMXRT_LPADC2_CMDH6    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH6_OFFSET)
#define IMXRT_LPADC2_CMDL7    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL7_OFFSET)
#define IMXRT_LPADC2_CMDH7    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH7_OFFSET)
#define IMXRT_LPADC2_CMDL8    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL8_OFFSET)
#define IMXRT_LPADC2_CMDH8    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH8_OFFSET)
#define IMXRT_LPADC2_CMDL9    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL9_OFFSET)
#define IMXRT_LPADC2_CMDH9    (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH9_OFFSET)
#define IMXRT_LPADC2_CMDL10   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL10_OFFSET)
#define IMXRT_LPADC2_CMDH10   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH10_OFFSET)
#define IMXRT_LPADC2_CMDL11   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL11_OFFSET)
#define IMXRT_LPADC2_CMDH11   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH11_OFFSET)
#define IMXRT_LPADC2_CMDL12   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL12_OFFSET)
#define IMXRT_LPADC2_CMDH12   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH12_OFFSET)
#define IMXRT_LPADC2_CMDL13   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL13_OFFSET)
#define IMXRT_LPADC2_CMDH13   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH13_OFFSET)
#define IMXRT_LPADC2_CMDL14   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL14_OFFSET)
#define IMXRT_LPADC2_CMDH14   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH14_OFFSET)
#define IMXRT_LPADC2_CMDL15   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDL15_OFFSET)
#define IMXRT_LPADC2_CMDH15   (IMXRT_LPADC2_BASE + IMXRT_LPADC_CMDH15_OFFSET)
#define IMXRT_LPADC2_RESFIFO  (IMXRT_LPADC2_BASE + IMXRT_LPADC_RESFIFO_OFFSET)
#define IMXRT_LPADC2_TCTRL0   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL0_OFFSET)
#define IMXRT_LPADC2_TCTRL1   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL1_OFFSET)
#define IMXRT_LPADC2_TCTRL2   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL2_OFFSET)
#define IMXRT_LPADC2_TCTRL3   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL3_OFFSET)
#define IMXRT_LPADC2_TCTRL4   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL4_OFFSET)
#define IMXRT_LPADC2_TCTRL5   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL5_OFFSET)
#define IMXRT_LPADC2_TCTRL6   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL6_OFFSET)
#define IMXRT_LPADC2_TCTRL7   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL7_OFFSET)
#define IMXRT_LPADC2_CV1      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV1_OFFSET)
#define IMXRT_LPADC2_CV2      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV2_OFFSET)
#define IMXRT_LPADC2_CV3      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV3_OFFSET)
#define IMXRT_LPADC2_CV4      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV4_OFFSET)
#define IMXRT_LPADC2_TCTRL0   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL0_OFFSET)
#define IMXRT_LPADC2_TCTRL1   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL1_OFFSET)
#define IMXRT_LPADC2_TCTRL2   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL2_OFFSET)
#define IMXRT_LPADC2_TCTRL3   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL3_OFFSET)
#define IMXRT_LPADC2_TCTRL4   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL4_OFFSET)
#define IMXRT_LPADC2_TCTRL5   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL5_OFFSET)
#define IMXRT_LPADC2_TCTRL6   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL6_OFFSET)
#define IMXRT_LPADC2_TCTRL7   (IMXRT_LPADC2_BASE + IMXRT_LPADC_TCTRL7_OFFSET)
#define IMXRT_LPADC2_CV1      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV1_OFFSET)
#define IMXRT_LPADC2_CV2      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV2_OFFSET)
#define IMXRT_LPADC2_CV3      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV3_OFFSET)
#define IMXRT_LPADC2_CV4      (IMXRT_LPADC2_BASE + IMXRT_LPADC_CV4_OFFSET)

/* Register bit definitions *************************************************/

#define IMXRT_LPADC_VERID_RES                   (1 << 0)                                   /* Up to 16-bit differential/15-bit single ended resolution supported. */
#define IMXRT_LPADC_VERID_DIFFEN                (1 << 1)                                   /* Differential operation supported. CMDLa[DIFF] and CMDLa[ABSEL] control fields implemented. */
#define IMXRT_LPADC_VERID_MVI                   (1 << 3)                                   /* Multiple voltage reference inputs supported. */
#define IMXRT_LPADC_VERID_CSW_SHIFT             (4)                                        /* Channel Scale Width */
#define IMXRT_LPADC_VERID_CSW_MASK              (0x07 << IMXRT_LPADC_VERID_CSW_SHIFT)
#define IMXRT_LPADC_VERID_CSW_CSW_0             (0x0 << IMXRT_LPADC_VERID_CSW_SHIFT)       /* Channel scaling not supported. */
#define IMXRT_LPADC_VERID_CSW_CSW_1             (0x1 << IMXRT_LPADC_VERID_CSW_SHIFT)       /* Channel scaling supported. 1-bit CSCALE control field. */
#define IMXRT_LPADC_VERID_CSW_CSW_6             (0x6 << IMXRT_LPADC_VERID_CSW_SHIFT)       /* Channel scaling supported. 6-bit CSCALE control field. */
#define IMXRT_LPADC_VERID_VR1RNGI               (1 << 8)                                   /* Range control required. CFG[VREF1RNG] is implemented. */
#define IMXRT_LPADC_VERID_IADCKI                (1 << 9)                                   /* Internal clock source (and CFG[ADCKEN]) implemented. */
#define IMXRT_LPADC_VERID_CALOFSI               (1 << 10)                                  /* Offset calibration and offset trimming implemented. */
#define IMXRT_LPADC_VERID_MINOR_SHIFT           (16)                                       /* Minor Version Number */
#define IMXRT_LPADC_VERID_MINOR_MASK            (0xff << IMXRT_LPADC_VERID_MINOR_SHIFT)
#define IMXRT_LPADC_VERID_MAJOR_SHIFT           (24)                                       /* Major Version Number */
#define IMXRT_LPADC_VERID_MAJOR_MASK            (0xff << IMXRT_LPADC_VERID_MAJOR_SHIFT)

#define IMXRT_LPADC_PARAM_TRIG_NUM_MASK         (0xff)
#define IMXRT_LPADC_PARAM_TRIG_NUM_TRIG_NUM_8   (0x8)                                      /* 8 hardware triggers implemented */
#define IMXRT_LPADC_PARAM_FIFOSIZE_SHIFT        (8)                                        /* Result FIFO Depth */
#define IMXRT_LPADC_PARAM_FIFOSIZE_MASK         (0xff << IMXRT_LPADC_PARAM_FIFOSIZE_SHIFT)
#define IMXRT_LPADC_PARAM_FIFOSIZE_FIFOSIZE_16  (0x10 << IMXRT_LPADC_PARAM_FIFOSIZE_SHIFT) /* Result FIFO depth = 16 datawords. */
#define IMXRT_LPADC_PARAM_CV_NUM_SHIFT          (16)                                       /* Compare Value Number */
#define IMXRT_LPADC_PARAM_CV_NUM_MASK           (0xff << IMXRT_LPADC_PARAM_CV_NUM_SHIFT)
#define IMXRT_LPADC_PARAM_CV_NUM_CV_NUM_4       (0x4 << IMXRT_LPADC_PARAM_CV_NUM_SHIFT)    /* 4 compare value registers implemented */
#define IMXRT_LPADC_PARAM_CMD_NUM_SHIFT         (24)                                       /* Command Buffer Number */
#define IMXRT_LPADC_PARAM_CMD_NUM_MASK          (0xff << IMXRT_LPADC_PARAM_CMD_NUM_SHIFT)
#define IMXRT_LPADC_PARAM_CMD_NUM_CMD_NUM_15    (0xf << IMXRT_LPADC_PARAM_CMD_NUM_SHIFT)   /* 15 command buffers implemented */

#define IMXRT_LPADC_CTRL_ADCEN                  (1 << 0)                                   /* LPADC is enabled. */
#define IMXRT_LPADC_CTRL_RST                    (1 << 1)                                   /* LPADC logic is reset. */
#define IMXRT_LPADC_CTRL_DOZEN                  (1 << 2)                                   /* LPADC is disabled in Doze mode. */
#define IMXRT_LPADC_CTRL_TRIG_SRC_SHIFT         (3)                                        /* Hardware trigger source selection */
#define IMXRT_LPADC_CTRL_TRIG_SRC_MASK          (0x03 << IMXRT_LPADC_CTRL_TRIG_SRC_SHIFT)
#define IMXRT_LPADC_CTRL_TRIG_SRC_TRIG_SRC_0    (0x0 << IMXRT_LPADC_CTRL_TRIG_SRC_SHIFT)   /* ADC_ETC hw trigger , and HW trigger are enabled */
#define IMXRT_LPADC_CTRL_TRIG_SRC_TRIG_SRC_1    (0x1 << IMXRT_LPADC_CTRL_TRIG_SRC_SHIFT)   /* ADC_ETC hw trigger is enabled */
#define IMXRT_LPADC_CTRL_TRIG_SRC_TRIG_SRC_2    (0x2 << IMXRT_LPADC_CTRL_TRIG_SRC_SHIFT)   /* HW trigger is enabled */
#define IMXRT_LPADC_CTRL_RSTFIFO                (1 << 8)                                   /* FIFO is reset. */

#define IMXRT_LPADC_STAT_RDY                    (1 << 0)                                   /* Result FIFO holding data above watermark level. */
#define IMXRT_LPADC_STAT_FOF                    (1 << 1)                                   /* At least one result FIFO overflow has occurred since the last time the flag was cleared. */
#define IMXRT_LPADC_STAT_ADC_ACTIVE             (1 << 8)                                   /* The LPADC is processing a conversion, running through the power up delay, or servicing a trigger. */
#define IMXRT_LPADC_STAT_TRGACT_SHIFT           (16)                                       /* Trigger Active */
#define IMXRT_LPADC_STAT_TRGACT_MASK            (0x07 << IMXRT_LPADC_STAT_TRGACT_SHIFT)
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_0        (0x0 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) associated with Trigger 0 currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_1        (0x1 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) associated with Trigger 1 currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_2        (0x2 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) associated with Trigger 2 currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_3        (0x3 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) from the associated Trigger number is currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_4        (0x4 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) from the associated Trigger number is currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_5        (0x5 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) from the associated Trigger number is currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_6        (0x6 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) from the associated Trigger number is currently being executed. */
#define IMXRT_LPADC_STAT_TRGACT_TRGACT_7        (0x7 << IMXRT_LPADC_STAT_TRGACT_SHIFT)     /* Command (sequence) from the associated Trigger number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_SHIFT           (24)                                       /* Command Active */
#define IMXRT_LPADC_STAT_CMDACT_MASK            (0x0f << IMXRT_LPADC_STAT_CMDACT_SHIFT)
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_0        (0x0 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* No command is currently in progress. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_1        (0x1 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Command 1 currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_2        (0x2 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Command 2 currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_3        (0x3 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_4        (0x4 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_5        (0x5 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_6        (0x6 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_7        (0x7 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_8        (0x8 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */
#define IMXRT_LPADC_STAT_CMDACT_CMDACT_9        (0x9 << IMXRT_LPADC_STAT_CMDACT_SHIFT)     /* Associated command number is currently being executed. */

#define IMXRT_LPADC_IE_FWMIE                    (1 << 0)                                   /* FIFO watermark interrupts are enabled. */
#define IMXRT_LPADC_IE_FOFIE                    (1 << 1)                                   /* FIFO overflow interrupts are enabled. */

#define IMXRT_LPADC_DE_FWMDE                    (1 << 0)                                   /* DMA request enabled. */

#define IMXRT_LPADC_CFG_TPRICTRL                (1 << 0)                                   /* If a higher priority trigger is received during command processing, the current conversion is completed (including averaging iterations if enabled) and stored to the RESFIFO before the higher priority trigger/command is initiated. Note that compare until true commands can be interrupted prior to resulting in a true conversion. */
#define IMXRT_LPADC_CFG_PWRSEL_SHIFT            (4)                                        /* Power Configuration Select */
#define IMXRT_LPADC_CFG_PWRSEL_MASK             (0x03 << IMXRT_LPADC_CFG_PWRSEL_SHIFT)
#define IMXRT_LPADC_CFG_PWRSEL_PWRSEL_0         (0x0 << IMXRT_LPADC_CFG_PWRSEL_SHIFT)      /* Level 1 (Lowest power setting) */
#define IMXRT_LPADC_CFG_PWRSEL_PWRSEL_1         (0x1 << IMXRT_LPADC_CFG_PWRSEL_SHIFT)      /* Level 2 */
#define IMXRT_LPADC_CFG_PWRSEL_PWRSEL_2         (0x2 << IMXRT_LPADC_CFG_PWRSEL_SHIFT)      /* Level 3 */
#define IMXRT_LPADC_CFG_PWRSEL_PWRSEL_3         (0x3 << IMXRT_LPADC_CFG_PWRSEL_SHIFT)      /* Level 4 (Highest power setting) */
#define IMXRT_LPADC_CFG_REFSEL_SHIFT            (6)                                        /* Voltage Reference Selection */
#define IMXRT_LPADC_CFG_REFSEL_MASK             (0x03 << IMXRT_LPADC_CFG_REFSEL_SHIFT)
#define IMXRT_LPADC_CFG_REFSEL_REFSEL_0         (0x0 << IMXRT_LPADC_CFG_REFSEL_SHIFT)      /* (Default) Option 1 setting. */
#define IMXRT_LPADC_CFG_REFSEL_REFSEL_1         (0x1 << IMXRT_LPADC_CFG_REFSEL_SHIFT)      /* Option 2 setting. */
#define IMXRT_LPADC_CFG_REFSEL_REFSEL_2         (0x2 << IMXRT_LPADC_CFG_REFSEL_SHIFT)      /* Option 3 setting. */
#define IMXRT_LPADC_CFG_PUDLY_SHIFT             (16)                                       /* Power Up Delay */
#define IMXRT_LPADC_CFG_PUDLY_MASK              (0xff << IMXRT_LPADC_CFG_PUDLY_SHIFT)
#define IMXRT_LPADC_CFG_PUDLY(d)                (((d) & 0xff) << IMXRT_LPADC_CFG_PUDLY_SHIFT)
#define IMXRT_LPADC_CFG_PWREN                   (1 << 28)                                  /* LPADC analog circuits are pre-enabled and ready to execute conversions without startup delays (at the cost of higher DC current consumption). When PWREN is set, the power up delay is enforced such that any detected trigger does not begin ADC operation until the power up delay time has passed. */

#define IMXRT_LPADC_PAUSE_PAUSEDLY_MASK         (0x1ff)                                    /* Pause Delay */
#define IMXRT_LPADC_PAUSE_PAUSEEN               (1 << 31)                                  /* Pause operation enabled */

#define IMXRT_LPADC_FCTRL_FCOUNT_MASK           (0x1f)
#define IMXRT_LPADC_FCTRL_FCOUNT_FCOUNT_0       (0x0)                                      /* No data stored in FIFO */
#define IMXRT_LPADC_FCTRL_FCOUNT_FCOUNT_1       (0x1)                                      /* 1 dataword stored in FIFO */
#define IMXRT_LPADC_FCTRL_FCOUNT_FCOUNT_2       (0x2)                                      /* 2 datawords stored in FIFO */
#define IMXRT_LPADC_FCTRL_FCOUNT_FCOUNT_4       (0x4)                                      /* 4 datawords stored in FIFO */
#define IMXRT_LPADC_FCTRL_FCOUNT_FCOUNT_8       (0x8)                                      /* 8 datawords stored in FIFO */
#define IMXRT_LPADC_FCTRL_FCOUNT_FCOUNT_16      (0x10)                                     /* 16 datawords stored in FIFO */
#define IMXRT_LPADC_FCTRL_FWMARK_SHIFT          (16)                                       /* Watermark level selection */
#define IMXRT_LPADC_FCTRL_FWMARK_MASK           (0x0f << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_0       (0x0 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 1st successful conversion - single conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_1       (0x1 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 2nd successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_2       (0x2 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 3rd successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_3       (0x3 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 4th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_4       (0x4 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 5th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_5       (0x5 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 6th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_6       (0x6 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 7th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_7       (0x7 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 8th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_8       (0x8 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 9th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_9       (0x9 << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 10th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_10      (0xa << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 11th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_11      (0xb << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 12th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_12      (0xc << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 13th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_13      (0xd << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 14th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_14      (0xe << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 15th successful conversion */
#define IMXRT_LPADC_FCTRL_FWMARK_FWMARK_15      (0xf << IMXRT_LPADC_FCTRL_FWMARK_SHIFT)    /* Generates STAT[RDY] flag after 16th successful conversion */

#define IMXRT_LPADC_SWTRIG_SWT0                 (1 << 0)                                   /* Trigger 0 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT1                 (1 << 1)                                   /* Trigger 1 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT2                 (1 << 2)                                   /* Trigger 2 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT3                 (1 << 3)                                   /* Trigger 3 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT4                 (1 << 4)                                   /* Trigger 4 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT5                 (1 << 5)                                   /* Trigger 5 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT6                 (1 << 6)                                   /* Trigger 6 event generated. */
#define IMXRT_LPADC_SWTRIG_SWT7                 (1 << 7)                                   /* Trigger 7 event generated. */

#define IMXRT_LPADC_CMDL1_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL1_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL1_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL1_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL1_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH1_CMPEN_MASK            (0x03)
#define IMXRT_LPADC_CMDH1_CMPEN_CMPEN_0         (0x0)                                      /* Compare disabled. */
#define IMXRT_LPADC_CMDH1_CMPEN_CMPEN_2         (0x2)                                      /* Compare enabled. Store on true. */
#define IMXRT_LPADC_CMDH1_CMPEN_CMPEN_3         (0x3)                                      /* Compare enabled. Repeat channel acquisition (sample/convert/compare) until true. */
#define IMXRT_LPADC_CMDH1_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH1_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH1_STS_MASK              (0x07 << IMXRT_LPADC_CMDH1_STS_SHIFT)
#define IMXRT_LPADC_CMDH1_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH1_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH1_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH1_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH1_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH1_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH1_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH1_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH1_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH1_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH1_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH1_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH1_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH1_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH1_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH1_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH1_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL2_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL2_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL2_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL2_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL2_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH2_CMPEN_MASK            (0x03)
#define IMXRT_LPADC_CMDH2_CMPEN_CMPEN_0         (0x0)                                      /* Compare disabled. */
#define IMXRT_LPADC_CMDH2_CMPEN_CMPEN_2         (0x2)                                      /* Compare enabled. Store on true. */
#define IMXRT_LPADC_CMDH2_CMPEN_CMPEN_3         (0x3)                                      /* Compare enabled. Repeat channel acquisition (sample/convert/compare) until true. */
#define IMXRT_LPADC_CMDH2_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH2_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH2_STS_MASK              (0x07 << IMXRT_LPADC_CMDH2_STS_SHIFT)
#define IMXRT_LPADC_CMDH2_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH2_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH2_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH2_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH2_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH2_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH2_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH2_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH2_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH2_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH2_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH2_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH2_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH2_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH2_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH2_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH2_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL3_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL3_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL3_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL3_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL3_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH3_CMPEN_MASK            (0x03)
#define IMXRT_LPADC_CMDH3_CMPEN_CMPEN_0         (0x0)                                      /* Compare disabled. */
#define IMXRT_LPADC_CMDH3_CMPEN_CMPEN_2         (0x2)                                      /* Compare enabled. Store on true. */
#define IMXRT_LPADC_CMDH3_CMPEN_CMPEN_3         (0x3)                                      /* Compare enabled. Repeat channel acquisition (sample/convert/compare) until true. */
#define IMXRT_LPADC_CMDH3_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH3_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH3_STS_MASK              (0x07 << IMXRT_LPADC_CMDH3_STS_SHIFT)
#define IMXRT_LPADC_CMDH3_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH3_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH3_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH3_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH3_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH3_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH3_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH3_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH3_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH3_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH3_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH3_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH3_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH3_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH3_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH3_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH3_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL4_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL4_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL4_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL4_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL4_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH4_CMPEN_MASK            (0x03)
#define IMXRT_LPADC_CMDH4_CMPEN_CMPEN_0         (0x0)                                      /* Compare disabled. */
#define IMXRT_LPADC_CMDH4_CMPEN_CMPEN_2         (0x2)                                      /* Compare enabled. Store on true. */
#define IMXRT_LPADC_CMDH4_CMPEN_CMPEN_3         (0x3)                                      /* Compare enabled. Repeat channel acquisition (sample/convert/compare) until true. */
#define IMXRT_LPADC_CMDH4_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH4_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH4_STS_MASK              (0x07 << IMXRT_LPADC_CMDH4_STS_SHIFT)
#define IMXRT_LPADC_CMDH4_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH4_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH4_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH4_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH4_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH4_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH4_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH4_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH4_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH4_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH4_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH4_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH4_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH4_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH4_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH4_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH4_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL5_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL5_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL5_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL5_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL5_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH5_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH5_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH5_STS_MASK              (0x07 << IMXRT_LPADC_CMDH5_STS_SHIFT)
#define IMXRT_LPADC_CMDH5_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH5_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH5_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH5_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH5_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH5_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH5_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH5_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH5_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH5_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH5_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH5_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH5_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH5_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH5_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH5_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH5_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL6_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL6_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL6_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL6_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL6_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH6_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH6_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH6_STS_MASK              (0x07 << IMXRT_LPADC_CMDH6_STS_SHIFT)
#define IMXRT_LPADC_CMDH6_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH6_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH6_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH6_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH6_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH6_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH6_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH6_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH6_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH6_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH6_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH6_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH6_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH6_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH6_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH6_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH6_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL7_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL7_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL7_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL7_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL7_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH7_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH7_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH7_STS_MASK              (0x07 << IMXRT_LPADC_CMDH7_STS_SHIFT)
#define IMXRT_LPADC_CMDH7_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH7_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH7_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH7_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH7_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH7_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH7_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH7_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH7_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH7_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH7_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH7_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH7_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH7_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH7_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH7_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH7_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL8_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL8_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL8_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL8_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL8_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH8_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH8_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH8_STS_MASK              (0x07 << IMXRT_LPADC_CMDH8_STS_SHIFT)
#define IMXRT_LPADC_CMDH8_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH8_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH8_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH8_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH8_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH8_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH8_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH8_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH8_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH8_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH8_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH8_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH8_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH8_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH8_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH8_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH8_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL9_ADCH_MASK             (0x1f)
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_0           (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_1           (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_2           (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_3           (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_4           (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_5           (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_6           (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_7           (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_8           (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_9           (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_30          (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL9_ADCH_ADCH_31          (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL9_ABSEL                 (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL9_DIFF                  (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL9_CSCALE                (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH9_LWI                   (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH9_STS_SHIFT             (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH9_STS_MASK              (0x07 << IMXRT_LPADC_CMDH9_STS_SHIFT)
#define IMXRT_LPADC_CMDH9_STS_STS_0             (0x0 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH9_STS_STS_1             (0x1 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_STS_STS_2             (0x2 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_STS_STS_3             (0x3 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_STS_STS_4             (0x4 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_STS_STS_5             (0x5 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_STS_STS_6             (0x6 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_STS_STS_7             (0x7 << IMXRT_LPADC_CMDH9_STS_SHIFT)       /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH9_AVGS_SHIFT            (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH9_AVGS_MASK             (0x07 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_0           (0x0 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* Single conversion. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_1           (0x1 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_2           (0x2 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_3           (0x3 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_4           (0x4 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_5           (0x5 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_6           (0x6 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH9_AVGS_AVGS_7           (0x7 << IMXRT_LPADC_CMDH9_AVGS_SHIFT)      /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH9_LOOP_SHIFT            (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH9_LOOP_MASK             (0x0f << IMXRT_LPADC_CMDH9_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_0           (0x0 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_1           (0x1 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_2           (0x2 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_3           (0x3 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_4           (0x4 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_5           (0x5 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_6           (0x6 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_7           (0x7 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_8           (0x8 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_9           (0x9 << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH9_LOOP_LOOP_15          (0xf << IMXRT_LPADC_CMDH9_LOOP_SHIFT)      /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH9_NEXT_SHIFT            (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH9_NEXT_MASK             (0x0f << IMXRT_LPADC_CMDH9_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_0           (0x0 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_1           (0x1 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_2           (0x2 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_3           (0x3 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_4           (0x4 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_5           (0x5 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_6           (0x6 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_7           (0x7 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_8           (0x8 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_9           (0x9 << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH9_NEXT_NEXT_15          (0xf << IMXRT_LPADC_CMDH9_NEXT_SHIFT)      /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL10_ADCH_MASK            (0x1f)
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_0          (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_1          (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_2          (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_3          (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_4          (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_5          (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_6          (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_7          (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_8          (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_9          (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_30         (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL10_ADCH_ADCH_31         (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL10_ABSEL                (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL10_DIFF                 (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL10_CSCALE               (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH10_LWI                  (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH10_STS_SHIFT            (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH10_STS_MASK             (0x07 << IMXRT_LPADC_CMDH10_STS_SHIFT)
#define IMXRT_LPADC_CMDH10_STS_STS_0            (0x0 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH10_STS_STS_1            (0x1 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_STS_STS_2            (0x2 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_STS_STS_3            (0x3 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_STS_STS_4            (0x4 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_STS_STS_5            (0x5 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_STS_STS_6            (0x6 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_STS_STS_7            (0x7 << IMXRT_LPADC_CMDH10_STS_SHIFT)      /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH10_AVGS_SHIFT           (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH10_AVGS_MASK            (0x07 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_0          (0x0 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* Single conversion. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_1          (0x1 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_2          (0x2 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_3          (0x3 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_4          (0x4 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_5          (0x5 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_6          (0x6 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH10_AVGS_AVGS_7          (0x7 << IMXRT_LPADC_CMDH10_AVGS_SHIFT)     /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH10_LOOP_SHIFT           (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH10_LOOP_MASK            (0x0f << IMXRT_LPADC_CMDH10_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_0          (0x0 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_1          (0x1 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_2          (0x2 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_3          (0x3 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_4          (0x4 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_5          (0x5 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_6          (0x6 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_7          (0x7 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_8          (0x8 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_9          (0x9 << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH10_LOOP_LOOP_15         (0xf << IMXRT_LPADC_CMDH10_LOOP_SHIFT)     /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH10_NEXT_SHIFT           (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH10_NEXT_MASK            (0x0f << IMXRT_LPADC_CMDH10_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_0          (0x0 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_1          (0x1 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_2          (0x2 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_3          (0x3 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_4          (0x4 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_5          (0x5 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_6          (0x6 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_7          (0x7 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_8          (0x8 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_9          (0x9 << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH10_NEXT_NEXT_15         (0xf << IMXRT_LPADC_CMDH10_NEXT_SHIFT)     /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL11_ADCH_MASK            (0x1f)
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_0          (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_1          (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_2          (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_3          (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_4          (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_5          (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_6          (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_7          (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_8          (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_9          (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_30         (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL11_ADCH_ADCH_31         (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL11_ABSEL                (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL11_DIFF                 (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL11_CSCALE               (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH11_LWI                  (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH11_STS_SHIFT            (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH11_STS_MASK             (0x07 << IMXRT_LPADC_CMDH11_STS_SHIFT)
#define IMXRT_LPADC_CMDH11_STS_STS_0            (0x0 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH11_STS_STS_1            (0x1 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_STS_STS_2            (0x2 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_STS_STS_3            (0x3 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_STS_STS_4            (0x4 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_STS_STS_5            (0x5 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_STS_STS_6            (0x6 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_STS_STS_7            (0x7 << IMXRT_LPADC_CMDH11_STS_SHIFT)      /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH11_AVGS_SHIFT           (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH11_AVGS_MASK            (0x07 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_0          (0x0 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* Single conversion. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_1          (0x1 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_2          (0x2 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_3          (0x3 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_4          (0x4 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_5          (0x5 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_6          (0x6 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH11_AVGS_AVGS_7          (0x7 << IMXRT_LPADC_CMDH11_AVGS_SHIFT)     /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH11_LOOP_SHIFT           (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH11_LOOP_MASK            (0x0f << IMXRT_LPADC_CMDH11_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_0          (0x0 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_1          (0x1 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_2          (0x2 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_3          (0x3 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_4          (0x4 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_5          (0x5 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_6          (0x6 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_7          (0x7 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_8          (0x8 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_9          (0x9 << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH11_LOOP_LOOP_15         (0xf << IMXRT_LPADC_CMDH11_LOOP_SHIFT)     /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH11_NEXT_SHIFT           (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH11_NEXT_MASK            (0x0f << IMXRT_LPADC_CMDH11_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_0          (0x0 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_1          (0x1 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_2          (0x2 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_3          (0x3 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_4          (0x4 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_5          (0x5 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_6          (0x6 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_7          (0x7 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_8          (0x8 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_9          (0x9 << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH11_NEXT_NEXT_15         (0xf << IMXRT_LPADC_CMDH11_NEXT_SHIFT)     /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL12_ADCH_MASK            (0x1f)
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_0          (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_1          (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_2          (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_3          (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_4          (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_5          (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_6          (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_7          (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_8          (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_9          (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_30         (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL12_ADCH_ADCH_31         (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL12_ABSEL                (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL12_DIFF                 (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL12_CSCALE               (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH12_LWI                  (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH12_STS_SHIFT            (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH12_STS_MASK             (0x07 << IMXRT_LPADC_CMDH12_STS_SHIFT)
#define IMXRT_LPADC_CMDH12_STS_STS_0            (0x0 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH12_STS_STS_1            (0x1 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_STS_STS_2            (0x2 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_STS_STS_3            (0x3 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_STS_STS_4            (0x4 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_STS_STS_5            (0x5 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_STS_STS_6            (0x6 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_STS_STS_7            (0x7 << IMXRT_LPADC_CMDH12_STS_SHIFT)      /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH12_AVGS_SHIFT           (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH12_AVGS_MASK            (0x07 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_0          (0x0 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* Single conversion. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_1          (0x1 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_2          (0x2 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_3          (0x3 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_4          (0x4 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_5          (0x5 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_6          (0x6 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH12_AVGS_AVGS_7          (0x7 << IMXRT_LPADC_CMDH12_AVGS_SHIFT)     /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH12_LOOP_SHIFT           (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH12_LOOP_MASK            (0x0f << IMXRT_LPADC_CMDH12_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_0          (0x0 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_1          (0x1 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_2          (0x2 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_3          (0x3 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_4          (0x4 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_5          (0x5 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_6          (0x6 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_7          (0x7 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_8          (0x8 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_9          (0x9 << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH12_LOOP_LOOP_15         (0xf << IMXRT_LPADC_CMDH12_LOOP_SHIFT)     /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH12_NEXT_SHIFT           (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH12_NEXT_MASK            (0x0f << IMXRT_LPADC_CMDH12_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_0          (0x0 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_1          (0x1 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_2          (0x2 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_3          (0x3 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_4          (0x4 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_5          (0x5 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_6          (0x6 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_7          (0x7 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_8          (0x8 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_9          (0x9 << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH12_NEXT_NEXT_15         (0xf << IMXRT_LPADC_CMDH12_NEXT_SHIFT)     /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL13_ADCH_MASK            (0x1f)
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_0          (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_1          (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_2          (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_3          (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_4          (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_5          (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_6          (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_7          (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_8          (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_9          (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_30         (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL13_ADCH_ADCH_31         (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL13_ABSEL                (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL13_DIFF                 (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL13_CSCALE               (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH13_LWI                  (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH13_STS_SHIFT            (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH13_STS_MASK             (0x07 << IMXRT_LPADC_CMDH13_STS_SHIFT)
#define IMXRT_LPADC_CMDH13_STS_STS_0            (0x0 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH13_STS_STS_1            (0x1 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_STS_STS_2            (0x2 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_STS_STS_3            (0x3 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_STS_STS_4            (0x4 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_STS_STS_5            (0x5 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_STS_STS_6            (0x6 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_STS_STS_7            (0x7 << IMXRT_LPADC_CMDH13_STS_SHIFT)      /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH13_AVGS_SHIFT           (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH13_AVGS_MASK            (0x07 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_0          (0x0 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* Single conversion. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_1          (0x1 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_2          (0x2 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_3          (0x3 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_4          (0x4 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_5          (0x5 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_6          (0x6 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH13_AVGS_AVGS_7          (0x7 << IMXRT_LPADC_CMDH13_AVGS_SHIFT)     /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH13_LOOP_SHIFT           (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH13_LOOP_MASK            (0x0f << IMXRT_LPADC_CMDH13_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_0          (0x0 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_1          (0x1 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_2          (0x2 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_3          (0x3 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_4          (0x4 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_5          (0x5 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_6          (0x6 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_7          (0x7 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_8          (0x8 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_9          (0x9 << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH13_LOOP_LOOP_15         (0xf << IMXRT_LPADC_CMDH13_LOOP_SHIFT)     /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH13_NEXT_SHIFT           (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH13_NEXT_MASK            (0x0f << IMXRT_LPADC_CMDH13_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_0          (0x0 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_1          (0x1 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_2          (0x2 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_3          (0x3 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_4          (0x4 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_5          (0x5 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_6          (0x6 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_7          (0x7 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_8          (0x8 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_9          (0x9 << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH13_NEXT_NEXT_15         (0xf << IMXRT_LPADC_CMDH13_NEXT_SHIFT)     /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL14_ADCH_MASK            (0x1f)
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_0          (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_1          (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_2          (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_3          (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_4          (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_5          (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_6          (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_7          (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_8          (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_9          (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_30         (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL14_ADCH_ADCH_31         (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL14_ABSEL                (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL14_DIFF                 (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL14_CSCALE               (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH14_LWI                  (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH14_STS_SHIFT            (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH14_STS_MASK             (0x07 << IMXRT_LPADC_CMDH14_STS_SHIFT)
#define IMXRT_LPADC_CMDH14_STS_STS_0            (0x0 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH14_STS_STS_1            (0x1 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_STS_STS_2            (0x2 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_STS_STS_3            (0x3 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_STS_STS_4            (0x4 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_STS_STS_5            (0x5 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_STS_STS_6            (0x6 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_STS_STS_7            (0x7 << IMXRT_LPADC_CMDH14_STS_SHIFT)      /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH14_AVGS_SHIFT           (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH14_AVGS_MASK            (0x07 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_0          (0x0 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* Single conversion. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_1          (0x1 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_2          (0x2 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_3          (0x3 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_4          (0x4 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_5          (0x5 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_6          (0x6 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH14_AVGS_AVGS_7          (0x7 << IMXRT_LPADC_CMDH14_AVGS_SHIFT)     /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH14_LOOP_SHIFT           (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH14_LOOP_MASK            (0x0f << IMXRT_LPADC_CMDH14_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_0          (0x0 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_1          (0x1 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_2          (0x2 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_3          (0x3 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_4          (0x4 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_5          (0x5 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_6          (0x6 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_7          (0x7 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_8          (0x8 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_9          (0x9 << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH14_LOOP_LOOP_15         (0xf << IMXRT_LPADC_CMDH14_LOOP_SHIFT)     /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH14_NEXT_SHIFT           (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH14_NEXT_MASK            (0x0f << IMXRT_LPADC_CMDH14_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_0          (0x0 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_1          (0x1 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_2          (0x2 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_3          (0x3 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_4          (0x4 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_5          (0x5 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_6          (0x6 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_7          (0x7 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_8          (0x8 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_9          (0x9 << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH14_NEXT_NEXT_15         (0xf << IMXRT_LPADC_CMDH14_NEXT_SHIFT)     /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_CMDL15_ADCH_MASK            (0x1f)
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_0          (0x0)                                      /* Select CH0A or CH0B or CH0A/CH0B pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_1          (0x1)                                      /* Select CH1A or CH1B or CH1A/CH1B pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_2          (0x2)                                      /* Select CH2A or CH2B or CH2A/CH2B pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_3          (0x3)                                      /* Select CH3A or CH3B or CH3A/CH3B pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_4          (0x4)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_5          (0x5)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_6          (0x6)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_7          (0x7)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_8          (0x8)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_9          (0x9)                                      /* Select corresponding channel CHnA or CHnB or CHnA/CHnB pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_30         (0x1e)                                     /* Select CH30A or CH30B or CH30A/CH30B pair. */
#define IMXRT_LPADC_CMDL15_ADCH_ADCH_31         (0x1f)                                     /* Select CH31A or CH31B or CH31A/CH31B pair. */
#define IMXRT_LPADC_CMDL15_ABSEL                (1 << 5)                                   /* When DIFF=0b0, the associated B-side channel is converted as single-ended. When DIFF=0b1, the ADC result is (CHnB-CHnA). */
#define IMXRT_LPADC_CMDL15_DIFF                 (1 << 6)                                   /* Differential mode. */
#define IMXRT_LPADC_CMDL15_CSCALE               (1 << 13)                                  /* (Default) Full scale (Factor of 1) */

#define IMXRT_LPADC_CMDH15_LWI                  (1 << 7)                                   /* Auto channel increment enabled */
#define IMXRT_LPADC_CMDH15_STS_SHIFT            (8)                                        /* Sample Time Select */
#define IMXRT_LPADC_CMDH15_STS_MASK             (0x07 << IMXRT_LPADC_CMDH15_STS_SHIFT)
#define IMXRT_LPADC_CMDH15_STS_STS_0            (0x0 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* Minimum sample time of 3 ADCK cycles. */
#define IMXRT_LPADC_CMDH15_STS_STS_1            (0x1 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 21 ADCK cycles; 5 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_STS_STS_2            (0x2 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 22 ADCK cycles; 7 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_STS_STS_3            (0x3 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 23 ADCK cycles; 11 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_STS_STS_4            (0x4 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 24 ADCK cycles; 19 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_STS_STS_5            (0x5 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 25 ADCK cycles; 35 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_STS_STS_6            (0x6 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 26 ADCK cycles; 67 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_STS_STS_7            (0x7 << IMXRT_LPADC_CMDH15_STS_SHIFT)      /* 3 + 27 ADCK cycles; 131 ADCK cycles total sample time. */
#define IMXRT_LPADC_CMDH15_AVGS_SHIFT           (12)                                       /* Hardware Average Select */
#define IMXRT_LPADC_CMDH15_AVGS_MASK            (0x07 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_0          (0x0 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* Single conversion. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_1          (0x1 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 2 conversions averaged. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_2          (0x2 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 4 conversions averaged. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_3          (0x3 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 8 conversions averaged. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_4          (0x4 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 16 conversions averaged. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_5          (0x5 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 32 conversions averaged. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_6          (0x6 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 64 conversions averaged. */
#define IMXRT_LPADC_CMDH15_AVGS_AVGS_7          (0x7 << IMXRT_LPADC_CMDH15_AVGS_SHIFT)     /* 128 conversions averaged. */
#define IMXRT_LPADC_CMDH15_LOOP_SHIFT           (16)                                       /* Loop Count Select */
#define IMXRT_LPADC_CMDH15_LOOP_MASK            (0x0f << IMXRT_LPADC_CMDH15_LOOP_SHIFT)
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_0          (0x0 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Looping not enabled. Command executes 1 time. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_1          (0x1 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop 1 time. Command executes 2 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_2          (0x2 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop 2 times. Command executes 3 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_3          (0x3 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_4          (0x4 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_5          (0x5 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_6          (0x6 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_7          (0x7 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_8          (0x8 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_9          (0x9 << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop corresponding number of times. Command executes LOOP+1 times. */
#define IMXRT_LPADC_CMDH15_LOOP_LOOP_15         (0xf << IMXRT_LPADC_CMDH15_LOOP_SHIFT)     /* Loop 15 times. Command executes 16 times. */
#define IMXRT_LPADC_CMDH15_NEXT_SHIFT           (24)                                       /* Next Command Select */
#define IMXRT_LPADC_CMDH15_NEXT_MASK            (0x0f << IMXRT_LPADC_CMDH15_NEXT_SHIFT)
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_0          (0x0 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* No next command defined. Terminate conversions at completion of current command. If lower priority trigger pending, begin command associated with lower priority trigger. */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_1          (0x1 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select CMD1 command buffer register as next command. */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_2          (0x2 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_3          (0x3 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_4          (0x4 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_5          (0x5 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_6          (0x6 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_7          (0x7 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_8          (0x8 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_9          (0x9 << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select corresponding CMD command buffer register as next command */
#define IMXRT_LPADC_CMDH15_NEXT_NEXT_15         (0xf << IMXRT_LPADC_CMDH15_NEXT_SHIFT)     /* Select CMD15 command buffer register as next command. */

#define IMXRT_LPADC_RESFIFO_D_MASK              (0xffff)                                   /* Data result */
#define IMXRT_LPADC_RESFIFO_TSRC_SHIFT          (16)                                       /* Trigger Source */
#define IMXRT_LPADC_RESFIFO_TSRC_MASK           (0x07 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_0         (0x0 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Trigger source 0 initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_1         (0x1 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Trigger source 1 initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_2         (0x2 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Corresponding trigger source initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_3         (0x3 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Corresponding trigger source initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_4         (0x4 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Corresponding trigger source initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_5         (0x5 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Corresponding trigger source initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_6         (0x6 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Corresponding trigger source initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_TSRC_TSRC_7         (0x7 << IMXRT_LPADC_RESFIFO_TSRC_SHIFT)    /* Trigger source 7 initiated this conversion. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT       (20)                                       /* Loop count value */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_MASK        (0x0f << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT)
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_0   (0x0 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from initial conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_1   (0x1 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from second conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_2   (0x2 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_3   (0x3 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_4   (0x4 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_5   (0x5 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_6   (0x6 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_7   (0x7 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_8   (0x8 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_9   (0x9 << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from LOOPCNT+1 conversion in command. */
#define IMXRT_LPADC_RESFIFO_LOOPCNT_LOOPCNT_15  (0xf << IMXRT_LPADC_RESFIFO_LOOPCNT_SHIFT) /* Result is from 16th conversion in command. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT        (24)                                       /* Command Buffer Source */
#define IMXRT_LPADC_RESFIFO_CMDSRC_MASK         (0x0f << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_0     (0x0 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Not a valid value CMDSRC value for a dataword in RESFIFO. 0x0 is only found in initial FIFO state prior to an ADC conversion result dataword being stored to a RESFIFO buffer. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_1     (0x1 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* CMD1 buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_2     (0x2 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_3     (0x3 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_4     (0x4 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_5     (0x5 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_6     (0x6 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_7     (0x7 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_8     (0x8 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_9     (0x9 << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* Corresponding command buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_CMDSRC_CMDSRC_15    (0xf << IMXRT_LPADC_RESFIFO_CMDSRC_SHIFT)  /* CMD15 buffer used as control settings for this conversion. */
#define IMXRT_LPADC_RESFIFO_VALID               (1 << 31)                                  /* FIFO record read from RESFIFO is valid. */

#define IMXRT_LPADC_TCTRL0_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL0_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL0_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL0_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL0_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL0_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL0_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL0_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL0_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL0_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL0_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL0_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL0_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL1_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL1_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL1_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL1_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL1_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL1_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL1_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL1_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL1_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL1_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL1_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL1_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL1_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL2_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL2_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL2_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL2_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL2_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL2_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL2_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL2_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL2_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL2_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL2_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL2_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL2_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL3_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL3_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL3_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL3_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL3_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL3_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL3_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL3_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL3_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL3_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL3_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL3_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL3_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL4_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL4_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL4_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL4_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL4_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL4_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL4_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL4_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL4_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL4_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL4_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL4_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL4_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL5_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL5_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL5_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL5_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL5_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL5_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL5_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL5_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL5_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL5_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL5_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL5_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL5_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL6_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL6_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL6_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL6_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL6_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL6_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL6_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL6_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL6_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL6_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL6_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL6_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL6_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_TCTRL7_HTEN                 (1 << 0)                                   /* Hardware trigger source enabled */
#define IMXRT_LPADC_TCTRL7_CMD_SEL              (1 << 1)                                   /* Software TCDM is bypassed , and hardware TCMD from ADC_ETC module will be used. The trigger command is then defined by ADC hardware trigger command selection field in ADC_ETC->TRIGx_CHAINy_z_n[CSEL]. */
#define IMXRT_LPADC_TCTRL7_TPRI_SHIFT           (8)                                        /* Trigger priority setting */
#define IMXRT_LPADC_TCTRL7_TPRI_MASK            (0x07 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_0          (0x0 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to highest priority, Level 1 */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_1          (0x1 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_2          (0x2 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_3          (0x3 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_4          (0x4 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_5          (0x5 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_6          (0x6 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to corresponding priority level */
#define IMXRT_LPADC_TCTRL7_TPRI_TPRI_7          (0x7 << IMXRT_LPADC_TCTRL7_TPRI_SHIFT)     /* Set to lowest priority, Level 8 */
#define IMXRT_LPADC_TCTRL7_TDLY_SHIFT           (16)                                       /* Trigger delay select */
#define IMXRT_LPADC_TCTRL7_TDLY_MASK            (0x0f << IMXRT_LPADC_TCTRL7_TDLY_SHIFT)
#define IMXRT_LPADC_TCTRL7_TCMD_SHIFT           (24)                                       /* Trigger command select */
#define IMXRT_LPADC_TCTRL7_TCMD_MASK            (0x0f << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_0          (0x0 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Not a valid selection from the command buffer. Trigger event is ignored. */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_1          (0x1 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* CMD1 is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_2          (0x2 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_3          (0x3 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_4          (0x4 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_5          (0x5 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_6          (0x6 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_7          (0x7 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_8          (0x8 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_9          (0x9 << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* Corresponding CMD is executed */
#define IMXRT_LPADC_TCTRL7_TCMD_TCMD_15         (0xf << IMXRT_LPADC_TCTRL7_TCMD_SHIFT)     /* CMD15 is executed */

#define IMXRT_LPADC_CV1_CVL_MASK                (0xffff)                                   /* Compare Value Low */
#define IMXRT_LPADC_CV1_CVH_SHIFT               (16)                                       /* Compare Value High. */
#define IMXRT_LPADC_CV1_CVH_MASK                (0xffff << IMXRT_LPADC_CV1_CVH_SHIFT)

#define IMXRT_LPADC_CV2_CVL_MASK                (0xffff)                                   /* Compare Value Low */
#define IMXRT_LPADC_CV2_CVH_SHIFT               (16)                                       /* Compare Value High. */
#define IMXRT_LPADC_CV2_CVH_MASK                (0xffff << IMXRT_LPADC_CV2_CVH_SHIFT)

#define IMXRT_LPADC_CV3_CVL_MASK                (0xffff)                                   /* Compare Value Low */
#define IMXRT_LPADC_CV3_CVH_SHIFT               (16)                                       /* Compare Value High. */
#define IMXRT_LPADC_CV3_CVH_MASK                (0xffff << IMXRT_LPADC_CV3_CVH_SHIFT)

#define IMXRT_LPADC_CV4_CVL_MASK                (0xffff)                                   /* Compare Value Low */
#define IMXRT_LPADC_CV4_CVH_SHIFT               (16)                                       /* Compare Value High. */
#define IMXRT_LPADC_CV4_CVH_MASK                (0xffff << IMXRT_LPADC_CV4_CVH_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER2_H */
