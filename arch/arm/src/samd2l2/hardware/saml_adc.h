/****************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_adc.h
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
 *   "Microchip SAM L21 Family Datasheet", Rev A - 02/2017
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_ADC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC register offsets *****************************************************/

#define SAM_ADC_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_ADC_CTRLB_OFFSET       0x0001 /* Control B Register */
#define SAM_ADC_REFCTL_OFFSET      0x0002 /* Reference Control Register */
#define SAM_ADC_EVCTRL_OFFSET      0x0003 /* Event Control Register */
#define SAM_ADC_INTENCLR_OFFSET    0x0004 /* Interrupt Enable Clear Register */
#define SAM_ADC_INTENSET_OFFSET    0x0005 /* Interrupt Enable Set Register */
#define SAM_ADC_INTFLAG_OFFSET     0x0006 /* Interrupt Flag Status and Clear Register */
#define SAM_ADC_SEQSTATUS_OFFSET   0x0007 /* Sequence Status Register */
#define SAM_ADC_INPUTCTRL_OFFSET   0x0008 /* Input Control Register */
#define SAM_ADC_CTRLC_OFFSET       0x000A /* Control C Register */
#define SAM_ADC_AVGCTRL_OFFSET     0x000C /* Average Control Register */
#define SAM_ADC_SAMPCTRL_OFFSET    0x000D /* Sampling Time Control Register */
#define SAM_ADC_WINLT_OFFSET       0x000E /* Window Monitor Lower Threshold Register */
#define SAM_ADC_WINUT_OFFSET       0x0010 /* Window Monitor Upper Threshold Register */
#define SAM_ADC_GAINCORR_OFFSET    0x0012 /* Gain Correction Register */
#define SAM_ADC_OFFSETCORR_OFFSET  0x0014 /* Offset Correction Register */
#define SAM_ADC_SWTRIG_OFFSET      0x0018 /* Software Trigger Register */
#define SAM_ADC_DBGCTRL_OFFSET     0x001C /* Debug Control Register */
#define SAM_ADC_SYNCBUSY_OFFSET    0x0020 /* Synchronization Busy Register */
#define SAM_ADC_RESULT_OFFSET      0x0024 /* Result Register */
#define SAM_ADC_SEQCTRL_OFFSET     0x0028 /* Sequence Control Register */
#define SAM_ADC_CALIB_OFFSET       0x002C /* Calibration Register */

/* ADC register addresses ***************************************************/

#define SAM_ADC_CTRLA              (SAM_ADC_BASE + SAM_ADC_CTRLA_OFFSET)
#define SAM_ADC_CTRLB              (SAM_ADC_BASE + SAM_ADC_CTRLB_OFFSET)
#define SAM_ADC_REFCTL             (SAM_ADC_BASE + SAM_ADC_REFCTL_OFFSET)
#define SAM_ADC_EVCTRL             (SAM_ADC_BASE + SAM_ADC_EVCTRL_OFFSET)
#define SAM_ADC_INTENCLR           (SAM_ADC_BASE + SAM_ADC_INTENCLR_OFFSET)
#define SAM_ADC_INTENSET           (SAM_ADC_BASE + SAM_ADC_INTENSET_OFFSET)
#define SAM_ADC_INTFLAG            (SAM_ADC_BASE + SAM_ADC_INTFLAG_OFFSET)
#define SAM_ADC_SEQSTATUS          (SAM_ADC_BASE + SAM_ADC_SEQSTATUS_OFFSET)
#define SAM_ADC_INPUTCTRL          (SAM_ADC_BASE + SAM_ADC_INPUTCTRL_OFFSET)
#define SAM_ADC_CTRLC              (SAM_ADC_BASE + SAM_ADC_CTRLC_OFFSET)
#define SAM_ADC_AVGCTRL            (SAM_ADC_BASE + SAM_ADC_AVGCTRL_OFFSET)
#define SAM_ADC_SAMPCTRL           (SAM_ADC_BASE + SAM_ADC_SAMPCTRL_OFFSET)
#define SAM_ADC_WINLT              (SAM_ADC_BASE + SAM_ADC_WINLT_OFFSET)
#define SAM_ADC_WINUT              (SAM_ADC_BASE + SAM_ADC_WINUT_OFFSET)
#define SAM_ADC_GAINCORR           (SAM_ADC_BASE + SAM_ADC_GAINCORR_OFFSET)
#define SAM_ADC_OFFSETCORR         (SAM_ADC_BASE + SAM_ADC_OFFSETCORR_OFFSET)
#define SAM_ADC_SWTRIG             (SAM_ADC_BASE + SAM_ADC_SWTRIG_OFFSET)
#define SAM_ADC_DBGCTRL            (SAM_ADC_BASE + SAM_ADC_DBGCTRL_OFFSET)
#define SAM_ADC_SYNCBUSY           (SAM_ADC_BASE + SAM_ADC_SYNCBUSY_OFFSET)
#define SAM_ADC_RESULT             (SAM_ADC_BASE + SAM_ADC_RESULT_OFFSET)
#define SAM_ADC_SEQCTRL            (SAM_ADC_BASE + SAM_ADC_SEQCTRL_OFFSET)
#define SAM_ADC_CALIB              (SAM_ADC_BASE + SAM_ADC_CALIB_OFFSET)

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_ADC_H */
