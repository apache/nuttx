/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_adc.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADC_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_ESP32S3_ADC
#  define ESP32S3_ADC1 1
#  define ESP32S3_ADC1_CHANNEL0 0
#  define ESP32S3_ADC1_CHANNEL1 1
#  define ESP32S3_ADC1_CHANNEL2 2
#  define ESP32S3_ADC1_CHANNEL3 3
#  define ESP32S3_ADC1_CHANNEL4 4
#  define ESP32S3_ADC1_CHANNEL5 5
#  define ESP32S3_ADC1_CHANNEL6 6
#  define ESP32S3_ADC1_CHANNEL7 7
#  define ESP32S3_ADC1_CHANNEL8 8
#  define ESP32S3_ADC1_CHANNEL9 9
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp32s3_adc_init
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   channel - ADC channel number
 *
 * Returned Value:
 *   ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

void esp32s3_adc_init(int adc_index, struct adc_dev_s *dev);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADC_H */
