/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_adc.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_ADC_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_ADC_H

#define SCUADCIF_LPADC_A0		(CXD56_SCU_ADCIF_BASE + 0x200)
#define SCUADCIF_LPADC_A1		(CXD56_SCU_ADCIF_BASE + 0x204)
#define SCUADCIF_LPADC_D0		(CXD56_SCU_ADCIF_BASE + 0x210)
#define SCUADCIF_LPADC_D1		(CXD56_SCU_ADCIF_BASE + 0x214)
#define SCUADCIF_LPADC_D4		(CXD56_SCU_ADCIF_BASE + 0x21c)
#define SCUADCIF_LPADC_D5		(CXD56_SCU_ADCIF_BASE + 0x220)
#define SCUADCIF_LPADC_D6		(CXD56_SCU_ADCIF_BASE + 0x224)
#define SCUADCIF_LPADC_D2		(CXD56_SCU_ADCIF_BASE + 0x218)
#define SCUADCIF_HPADC_AC0		(CXD56_SCU_ADCIF_BASE + 0x240)
#define SCUADCIF_HPADC_AC1		(CXD56_SCU_ADCIF_BASE + 0x244)
#define SCUADCIF_HPADC_DC		(CXD56_SCU_ADCIF_BASE + 0x250)
#define SCUADCIF_HPADC0_A0		(CXD56_SCU_ADCIF_BASE + 0x280)
#define SCUADCIF_HPADC0_A1		(CXD56_SCU_ADCIF_BASE + 0x284)
#define SCUADCIF_HPADC0_A2		(CXD56_SCU_ADCIF_BASE + 0x288)
#define SCUADCIF_HPADC0_A3		(CXD56_SCU_ADCIF_BASE + 0x28c)
#define SCUADCIF_HPADC0_D0		(CXD56_SCU_ADCIF_BASE + 0x290)
#define SCUADCIF_HPADC0_D1		(CXD56_SCU_ADCIF_BASE + 0x294)
#define SCUADCIF_HPADC0_D2		(CXD56_SCU_ADCIF_BASE + 0x298)
#define SCUADCIF_HPADC1_A0		(CXD56_SCU_ADCIF_BASE + 0x2c0)
#define SCUADCIF_HPADC1_A1		(CXD56_SCU_ADCIF_BASE + 0x2c4)
#define SCUADCIF_HPADC1_A2		(CXD56_SCU_ADCIF_BASE + 0x2c8)
#define SCUADCIF_HPADC1_A3		(CXD56_SCU_ADCIF_BASE + 0x2cc)
#define SCUADCIF_HPADC1_D0		(CXD56_SCU_ADCIF_BASE + 0x2d0)
#define SCUADCIF_HPADC1_D1		(CXD56_SCU_ADCIF_BASE + 0x2d4)
#define SCUADCIF_HPADC1_D2		(CXD56_SCU_ADCIF_BASE + 0x2d8)
#define SCUADCIF_LPADC_AT0		(CXD56_SCU_ADCIF_BASE + 0x300)
#define SCUADCIF_LPADC_AT1		(CXD56_SCU_ADCIF_BASE + 0x304)
#define SCUADCIF_HPADC_ACT0		(CXD56_SCU_ADCIF_BASE + 0x340)
#define SCUADCIF_HPADC_ACT1		(CXD56_SCU_ADCIF_BASE + 0x344)
#define SCUADCIF_HPADC0_AT0		(CXD56_SCU_ADCIF_BASE + 0x380)
#define SCUADCIF_HPADC0_AT1		(CXD56_SCU_ADCIF_BASE + 0x384)
#define SCUADCIF_HPADC1_AT0		(CXD56_SCU_ADCIF_BASE + 0x3c0)
#define SCUADCIF_HPADC1_AT1		(CXD56_SCU_ADCIF_BASE + 0x3c4)
#define SCUADCIF_ADCIF_DCT		(CXD56_SCU_ADCIF_BASE + 0x3d0)
#define SCUADCIF_SCU_ADCIF_CKPOWER	(CXD56_SCU_ADCIF_BASE + 0x3d4)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_ADC_H */
