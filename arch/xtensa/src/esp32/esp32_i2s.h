/****************************************************************************
 * arch/xtensa/src/esp32/esp32_i2s.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_I2S_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/audio/i2s.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ESP32_I2S

#ifdef CONFIG_ESP32_I2S0
  #define ESP32_I2S0 0
#endif /* CONFIG_ESP32_I2S0 */

#ifdef CONFIG_ESP32_I2S1
  #define ESP32_I2S1 1
#endif /* CONFIG_ESP32_I2S1 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *esp32_i2sbus_initialize(int port);

#endif /* CONFIG_ESP32_I2S */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_I2S_H */
