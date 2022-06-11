/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_adc.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_ADC_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/tlsr82_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_CHAN_0               0
#define ADC_CHAN_1               1
#define ADC_CHAN_2               2
#define ADC_CHAN_3               3
#define ADC_CHAN_4               4
#define ADC_CHAN_5               5
#define ADC_CHAN_6               6
#define ADC_CHAN_7               7
#define ADC_CHAN_TEMP            253
#define ADC_CHAN_VBAT            254
#define ADC_CHAN_NONE            255

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_adc_init
 ****************************************************************************/

int tlsr82_adc_init(const char *devpath, int miror);

#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_ADC_H */
