/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_adc_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_ADC_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_ADC_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip ADC wiring for RTL8721DX (amebadplus).  The shared driver
 * (arch/arm/src/common/ameba/ameba_adc.c) includes this header to learn how
 * many channels the converter can list, the crossbar pad-mux code that turns
 * a pad into an analog input, and the peripheral-clock masks.  A port to
 * another Ameba chip supplies a same-named header on the chip include path;
 * the shared driver is never edited -- it reads only the macros below, and
 * it drives the ADC through the SDK fwlib API, which internally selects the
 * secure/non-secure register alias (via TrustZone_IsSecure()), so no
 * register base appears here at all.
 *
 * What differs per chip, from the SDK fwlib headers and the vendor
 * example_adc_ext.h (verified, not guessed):
 *
 *   chip         pad-mux code for analog in       extra clock to gate
 *   -----------  ------------------------------   -------------------
 *   amebadplus   PINMUX_FUNCTION_ADC       = 6    (none)
 *   amebalite    PINMUX_FUNCTION_AUXIN            APBPeriph_CTC
 *   amebasmart   PINMUX_FUNCTION_CAPTOUCH         APBPeriph_CTC
 *   amebagreen2  PINMUX_FUNCTION_ADC             (none)
 *   RTL8720F     PINMUX_FUNCTION_ADC             (none)
 *
 * Both differences are expressed as macros so a new chip only edits this
 * header: AMEBA_ADC_PINMUX_FID carries the pad-mux code, and a chip that
 * needs a second clock domain (amebalite/amebasmart route the ADC through
 * the cap-touch/CTC block) also defines AMEBA_ADC_AUXCLK_PERIPH / _CLK,
 * which the driver gates only when present.  amebadplus needs neither, so
 * it leaves those two undefined -- see the SPI/I2C/PWM chip headers for the
 * same data-driven, never-computed pattern.
 *
 * External channels CH0..CH6 map to pads PB19,PB18,PB17,PB16,PB15,PB14,PB13
 * (from ameba_adc.h ADC_CHx_PIN); CH7..CH10 are fixed internal channels with
 * no pad.  The 16-bit conversion word (channel id in [19:16], data in
 * [15:0]) is identical on every current Ameba chip, so the driver extracts
 * it directly and no macro is needed here.
 */

#define AMEBA_ADC_NCHAN           11    /* ADC_CH_NUM: CH0..CH6 ext, 7..10 int */
#define AMEBA_ADC_MAXLIST         16    /* Channel-switch list depth (Cvlist)  */
#define AMEBA_ADC_PINMUX_FID      6     /* PINMUX_FUNCTION_ADC                  */
#define AMEBA_ADC_IRQ             47    /* ADC_IRQ (unused by this polling drv) */

/* sizeof(fwlib ADC_InitTypeDef): OpMode/CvlistLen/Cvlist[16] then ClkDiv/
 * RxThresholdLevel/SpecialCh/ChanInType(u32) -> 28 bytes.  The shared driver
 * sizes its stack mirror from this so ADC_StructInit() cannot overflow it.
 */

#define AMEBA_ADC_INIT_SIZE       28

/* APBPeriph_ADC (function) and APBPeriph_ADC_CLOCK masks.  Equal on this
 * chip; both set the bit30 group selector and bit23 for the LP ADC block.
 * amebadplus needs no second clock domain, so AMEBA_ADC_AUXCLK_* are left
 * undefined (amebalite/amebasmart define them to gate APBPeriph_CTC too).
 */

#define AMEBA_ADC_APBPERIPH       (((uint32_t)1 << 30) | ((uint32_t)1 << 23))
#define AMEBA_ADC_APBPERIPH_CLK   (((uint32_t)1 << 30) | ((uint32_t)1 << 23))

/* amebadplus ADC_InitTypeDef has a ClkDiv field at offset 18; the other
 * Ameba chips (amebagreen2, RTL8720F) do not -- their offset 18 is
 * RxThresholdLevel.  The shared driver guards the ClkDiv write with this
 * macro so it only runs on chips that actually have the field.
 */

#define AMEBA_ADC_HAS_CLKDIV      1

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_ADC_CHIP_H */
