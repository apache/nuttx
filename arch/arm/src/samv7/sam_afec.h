/****************************************************************************
<<<<<<< HEAD
 * arch/arm/src/samd2l2/sam_adc.h
=======
 * arch/arm/src/samv7/sam_afec.h
>>>>>>> d4a50033761a90a487eec7e191f02559a4d31359
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

<<<<<<< HEAD
#ifndef __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H
#define __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H
=======
#ifndef __ARCH_ARM_SRC_SAMV7_SAM_AFEC_H
#define __ARCH_ARM_SRC_SAMV7_SAM_AFEC_H
>>>>>>> d4a50033761a90a487eec7e191f02559a4d31359

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

<<<<<<< HEAD
#include <stdint.h>
#include <stdbool.h>

#include "sam_config.h"


#include "hardware/sam_afec.h"

=======
#include "chip.h"
#include "hardware/sam_afec.h"

#if defined(CONFIG_ADC) && (defined(CONFIG_SAMV7_AFEC0) || \
    defined(CONFIG_SAMV7_AFEC1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port numbers for use with sam_mcan_initialize() */

#define AFEC0 0
#define AFEC1 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
>>>>>>> d4a50033761a90a487eec7e191f02559a4d31359

/****************************************************************************
 * Public Function Prototypes
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
<<<<<<< HEAD
 * Name: sam_adcinitialize
 *
 * Description:
 *   Initialize the ADC. See sam_adc.c for more details.
 *
 * Input Parameters:
 *   genclk      - Number of the Clock Generator to use.
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *sam_adcinitialize(int genclk);
=======
 * Name: sam_afec_initialize
 *
 * Description:
 *   Initialize the adc
 *
 * Input Parameters:
 *   intf      - ADC number (1 or 2)
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *sam_afec_initialize(int intf,
                                          FAR const uint8_t *chanlist,
                                          int nchannels);
>>>>>>> d4a50033761a90a487eec7e191f02559a4d31359

#undef EXTERN
#if defined(__cplusplus)
}
#endif
<<<<<<< HEAD
#endif /* __ARCH_ARM_SRC_SAMD2L2_SAM_ADC_H */
=======

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ADC && (CONFIG_SAMV7_AFEC0 || CONFIG_SAMV7_AFEC1) */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_AFEC_H */
>>>>>>> d4a50033761a90a487eec7e191f02559a4d31359
