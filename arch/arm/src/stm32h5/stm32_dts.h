/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dts.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_DTS_H
#define __ARCH_ARM_SRC_STM32H5_STM32_DTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/sensor.h>      /* For struct sensor_lowerhalf_s */
#include <nuttx/sensors/ioctl.h>       /* SNIOC_* if needed */
#include <nuttx/uorb.h>               /* SENSOR_TYPE_AMBIENT_TEMPERATURE */

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct stm32_dts_cal_s
{
  uint16_t fmt0;
  uint16_t ramp;
  float t0;
};

struct stm32_dts_cfg_s
{
  uint8_t samples;
  bool lse;
  uint32_t clk_frequency;
};

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

int stm32h5_dts_register(int devno);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_DTS_H */
