/****************************************************************************
 * arch/arm/src/rp2040/rp2040_adc.h
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

/****************************************************************************
 * Note:
 *    The ADC driver upper-half returns signed values of up to 32-bits to
 *    the user and expects the high-order bits in any result to be
 *    significant.  So, while the RP2040 hardware returns an unsigned value
 *    in the low-order 12 bits of the result register, we shift this
 *    value to the high-order bits.
 *
 *    The result is that to convert a 32-bit value returned from the ADC
 *    driver, you should use   V = ADC_AVDD * value / (2^31)  where ADC_AVDD
 *    is the analogue reference voltage supplied to the RP2040 chip.  If
 *    8 or 16 bit values are returned the divisor would be (2^15) or (2^7)
 *    respectively.
 *
 *    Also, if the conversion error bit was set for a particular sample,
 *    the return value will be negated.  Any negative return value should
 *    be treated as erroneous.
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_RP2040_ADC_H
#define __ARCH_ARM_SRC_RP2040_RP2040_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_RP2040_ADC

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ADC

/****************************************************************************
 * Name: rp2040_adc_setup
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 * Input Parameters:
 *   path      - Path to the ws2812 device  (e.g. "/dev/adc0")
 *   read_adc0 - This device reads ADC0
 *   read_adc1 - This device reads ADC1
 *   read_adc2 - This device reads ADC3
 *   read_adc3 - This device reads ADC4
 *   read_temp - This device reads the chip temperature.
 *
 * Returned Value:
 *   OK on success or an ERROR on failure
 ****************************************************************************/

int rp2040_adc_setup(const char *path,
                     bool        read_adc0,
                     bool        read_adc1,
                     bool        read_adc2,
                     bool        read_adc3,
                     bool        read_temp);

#else /* CONFIG_ADC */

/* ### TODO ### Add programmatic access function. */

#endif /* CONFIG_ADC */

#endif /* CONFIG_RP2040_ADC */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_RP2040_RP2040_ADC_H */
