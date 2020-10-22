/****************************************************************************
 * include/nuttx/sensors/wtgahrs2.h
 * Driver for the Wit-Motion WTGAHRS2 accelerometer, gyroscope, magnetic,
 * angle, barometer, temperature, gps sensors by serial interface with host
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

#ifndef __INCLUDE_NUTTX_SENSORS_WTGAHRS2_H
#define __INCLUDE_NUTTX_SENSORS_WTGAHRS2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: wtgahrs2_initialize
 *
 * Description:
 *   Initialize wrgahrs2 sensor module, it will create accelerometer,
 *   gyroscope, magnetic, barometer, gps character device.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to read data source by serial tty.
 *
 * Returned Value:
 *   OK if the driver was successfully initialize; A negated errno value is
 *   returned on any failure.
 ****************************************************************************/

int wtgahrs2_initialize(FAR const char *path);

#ifdef __cplusplus
}
#endif

#endif
