/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_temperature_sensor.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TEMPERATURE_SENSOR_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TEMPERATURE_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

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
 * Public Types
 ****************************************************************************/

struct esp_temp_sensor_config_t
{
    int range_min;   /* The minimum value of the temperature you want to test */
    int range_max;   /* The maximum value of the temperature you want to test */
};

#define TEMPERATURE_SENSOR_CONFIG(min, max) \
{                                           \
    .range_min = min,                       \
    .range_max = max                        \
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_temperature_sensor_initialize
 *
 * Description:
 *   This function initializes the internal temperature sensor with the
 *   provided configuration.
 *
 * Input Parameters:
 *   cfg - Configuration of measurement range for the temperature sensor
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_temperature_sensor_initialize(struct esp_temp_sensor_config_t cfg);

/****************************************************************************
 * Name: esp_temperature_sensor_uninitialize
 *
 * Description:
 *   This function uninitializes the internal temperature sensor.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_temperature_sensor_uninitialize(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TEMPERATURE_SENSOR_H */
