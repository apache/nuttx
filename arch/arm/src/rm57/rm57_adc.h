/****************************************************************************
 * arch/arm/src/rm57/rm57_adc.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_ADC_H
#define __ARCH_ARM_SRC_RM57_RM57_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/analog/adc.h>

#include <arch/chip/rm57_adc_ioctl.h>

#include "hardware/rm57_adc.h"

#ifdef CONFIG_RM57_ADC

/* The ioctl commands this driver accepts, the types they take and the
 * constants that go in them are the application's interface, so they live in
 * the exported arch/chip header included above rather than here.  What
 * remains below is the in-kernel entry points the board glue calls.
 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rm57_adc_initialize
 *
 * Description:
 *   Initialize one MibADC conversion group and register it with the upper
 *   half ADC driver at 'devpath'.  The owning module is brought out of reset
 *   and configured on the first call that references it; later calls for the
 *   other groups of the same module reuse that configuration.
 *
 * Input Parameters:
 *   devpath - Character device path, e.g. "/dev/adc0"
 *   module  - RM57_ADC_MODULE1 or RM57_ADC_MODULE2
 *   group   - RM57_ADC_GROUP_EV, RM57_ADC_GROUP_G1 or RM57_ADC_GROUP_G2
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_adc_initialize(const char *devpath, int module, int group);

/****************************************************************************
 * Name: rm57_adc_calibrate
 *
 * Description:
 *   Run the offset error calibration sequence for one ADC module and load
 *   the resulting correction value into ADCALR.  No conversion group may be
 *   active while this runs; the caller's channel selections are preserved.
 *
 * Input Parameters:
 *   module - RM57_ADC_MODULE1 or RM57_ADC_MODULE2
 *   calib  - Receives the raw conversions and computed correction.  May be
 *            NULL if the caller only wants the correction applied.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_adc_calibrate(int module, struct rm57_adc_calib_s *calib);

/****************************************************************************
 * Name: rm57_adc_midpointcal
 *
 * Description:
 *   Run the differential mid-point calibration sequence (TRM SPNU562A
 *   section 22.2.6.1.3), which cancels the mismatch of the internal bridge
 *   resistors by averaging the two swapped mid-point measurements.
 *
 ****************************************************************************/

int rm57_adc_midpointcal(int module, struct rm57_adc_calib_s *calib);

/****************************************************************************
 * Name: rm57_adc_selftest
 *
 * Description:
 *   Drive ADREFHI and then ADREFLO onto one input channel through the
 *   internal test resistor and classify the channel as healthy, open, or
 *   shorted to ground or supply.
 *
 ****************************************************************************/

int rm57_adc_selftest(int module, struct rm57_adc_selftest_s *test);

/****************************************************************************
 * Name: rm57_adc_readtemp
 *
 * Description:
 *   Convert one of the three on-die temperature sensors and report the
 *   result, scaled by the factory calibration recorded in TI OTP.  The
 *   sensor's ADC channel is converted once on a borrowed conversion group,
 *   leaving that group's configuration untouched.
 *
 * Input Parameters:
 *   sensor - 0, 1 or 2
 *   temp   - Receives the raw reading and the calibrated temperature
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC_TEMPSENSOR
int rm57_adc_readtemp(int sensor, struct rm57_adc_temp_s *temp);
#endif

/****************************************************************************
 * Name: rm57_adc_tempcal_points
 *
 * Description:
 *   Report one sensor's factory calibration points as they were recorded in
 *   TI OTP: 'val' receives the three ADC readings and 'kelvin' the three
 *   temperatures they were taken at.  Both arrays must hold at least
 *   RM57_ADC_TEMPCAL_NPOINTS entries.
 *
 *   This is a bring-up aid.  A sensor that reports itself uncalibrated gives
 *   no clue whether its OTP is blank or whether the driver rejected the
 *   curve, and these are the only OTP reads the port makes at all - so
 *   plausible values here are also what shows that errata L2FMC#5 is not
 *   substituting instruction opcodes for the data.
 *
 * Returned Value:
 *   Zero if the calibration is usable, -ENODATA if it is not, or -EINVAL for
 *   a sensor that is not in this configuration.  The arrays are filled in
 *   either way when the sensor is valid.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC_TEMPSENSOR
int rm57_adc_tempcal_points(int sensor, uint16_t *val, uint16_t *kelvin);
#endif

/****************************************************************************
 * Name: rm57_adc_temp_register
 *
 * Description:
 *   Register one on-die temperature sensor as a device node.  With
 *   CONFIG_RM57_ADC_TEMP_UORB the node is the sensor framework's
 *   /dev/uorb/sensor_temp<devno>, publishing calibrated degrees Celsius;
 *   otherwise it is a character device, CONFIG_RM57_ADC_TEMP_PATH<devno>,
 *   whose read() returns a whole struct rm57_adc_temp_s.
 *
 * Input Parameters:
 *   sensor - 0, 1 or 2
 *   devno  - Device number, unique among temperature sensors
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC_TEMPSENSOR
int rm57_adc_temp_register(int sensor, int devno);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RM57_ADC */
#endif /* __ARCH_ARM_SRC_RM57_RM57_ADC_H */
