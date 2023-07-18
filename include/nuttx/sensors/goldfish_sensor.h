/****************************************************************************
 * include/nuttx/sensors/goldfish_sensor.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_GOLDFISH_SENSOR_H
#define __INCLUDE_NUTTX_SENSORS_GOLDFISH_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: goldfish_sensor_init
 *
 * Description:
 *   Goldfish Multi-Sensors driver entrypoint.
 *
 * Input Parameters:
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int goldfish_sensor_init(int devno, uint32_t batch_number);

#ifdef __cplusplus
}
#endif

#endif
