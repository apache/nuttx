/****************************************************************************
 * arch/arm/include/samd2l2/sam_adc.h
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H
#define __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IOCTL definitions */

#define SAMD_ADC_IOCTL_START       0 /* start adc conversion */
#define SAMD_ADC_IOCTL_STOP        1 /* stop adc conversion */
#define SAMD_ADC_IOCTL_SET_PARAMS  2 /* set parameters when adc is stopped */
#define SAMD_ADC_IOCTL_GET_PARAMS  3 /* get parameters */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sam_adc_param_s
{
  uint8_t samplen;      /* sampling time length */
  uint8_t prescaler;    /* prescaler configuration */
  uint8_t averaging;    /* number of samples to be collected */
};

#endif /* __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H */
