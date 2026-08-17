/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_adc_chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RTL8721F_AMEBA_ADC_CHIP_H
#define __ARCH_ARM_SRC_RTL8721F_AMEBA_ADC_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip ADC wiring for RTL8721F (amebagreen2).  The shared driver
 * (arch/arm/src/common/ameba/ameba_adc.c) includes this header to learn how
 * many channels the converter can list, the crossbar pad-mux code that turns
 * a pad into an analog input, and the peripheral-clock masks.  A port to
 * another Ameba chip supplies a same-named header on the chip include path;
 * the shared driver is never edited -- it reads only the macros below, and
 * it drives the ADC through the SDK fwlib API, which internally selects the
 * secure/non-secure register alias (via TrustZone_IsSecure()), so no
 * register base appears here at all.
 *
 * Values below are taken from the amebagreen2 fwlib headers (verified, not
 * guessed): PINMUX_FUNCTION_ADC=5 (ameba_pinmux.h), ADC_IRQ=42
 * (ameba_vector_table.h), APBPeriph_ADC and APBPeriph_ADC_CLOCK both
 * bit30|bit23 (sysreg_lsys.h).  RTL8721F does not route the ADC through the
 * cap-touch/CTC block, so no second clock domain is needed and
 * AMEBA_ADC_AUXCLK_* stay undefined.
 *
 * External channels CH0..CH7 map to pads PA20,PA19,PA18,PA17,PA15,PA14,
 * PA13,PA12 (from ameba_adc.h ADC_CHx_PIN); CH8..CH10 and CH14 are fixed
 * internal channels with no pad.  The 16-bit conversion word (channel id in
 * [19:16], data in [15:0]) is identical on every current Ameba chip, so the
 * driver extracts it directly and no macro is needed here.
 */

#define AMEBA_ADC_NCHAN           12    /* ADC_CH_NUM: CH0..CH7 ext, rest int  */
#define AMEBA_ADC_MAXLIST         16    /* Channel-switch list depth (Cvlist)  */
#define AMEBA_ADC_PINMUX_FID      5     /* PINMUX_FUNCTION_ADC                  */
#define AMEBA_ADC_IRQ             42    /* ADC_IRQ (unused by this polling drv) */

/* sizeof(fwlib ADC_InitTypeDef): OpMode/CvlistLen/Cvlist[16] then
 * RxThresholdLevel/SpecialCh/SamplePeriodUs(u16) -> 22 bytes.  The shared
 * driver sizes its stack mirror from this so ADC_StructInit() cannot
 * overflow it.
 */

#define AMEBA_ADC_INIT_SIZE       22

/* APBPeriph_ADC (function) and APBPeriph_ADC_CLOCK masks.  Equal on this
 * chip; both set the bit30 group selector and bit23 for the LP ADC block.
 * RTL8721F needs no second clock domain, so AMEBA_ADC_AUXCLK_* are left
 * undefined (amebalite/amebasmart define them to gate APBPeriph_CTC too).
 */

#define AMEBA_ADC_APBPERIPH       (((uint32_t)1 << 30) | ((uint32_t)1 << 23))
#define AMEBA_ADC_APBPERIPH_CLK   (((uint32_t)1 << 30) | ((uint32_t)1 << 23))

#endif /* __ARCH_ARM_SRC_RTL8721F_AMEBA_ADC_CHIP_H */
