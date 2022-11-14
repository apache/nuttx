/****************************************************************************
 * arch/arm/include/samv7/sam_afec.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMV7_SAM_AFEC_H
#define __ARCH_ARM_INCLUDE_SAMV7_SAM_AFEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/analog/adc.h>

/* IOCTL Commands ***********************************************************
 *
 * Cmd: SAMV7_AFEC_IOCTRL_GAIN           Arg: Channel and Gain
 *
 */

#define ANIOC_SAMV7_AFEC_IOCTRL_GAIN           _ANIOC(AN_SAMV7_AFEC_FIRST + 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct sam_afec_gain_param_s
{
  uint8_t channel;  /* channel number of aefc instance */
  uint8_t gain;     /* gain to be set for the channel 0:1, 1:2, 3:4 */
} sam_afec_gain_param_tds;

#ifndef __ASSEMBLY__

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_SAMV7_SAM_AFEC_H */
