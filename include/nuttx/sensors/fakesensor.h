/****************************************************************************
 * include/nuttx/sensors/fakesensor.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_FAKESENSOR_H
#define __INCLUDE_NUTTX_SENSORS_FAKESENSOR_H

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
 * Name: fakesensor_init
 *
 * Description:
 *   This function generates a sensor node under /dev/uorb/. And then
 *   report the data from csv file.
 *
 * Input Parameters:
 *   type        - The type of sensor and defined in <nuttx/sensors/sensor.h>
 *   file_name   - The name of csv name and the file structure is as follows:
 *                    First row : set interval, unit millisecond
 *                    Second row: csv file header
 *                    third row : data
 *                    (Each line should not exceed 50 characters)
 *                    For example:
 *                    interval:12
 *                    x,y,z
 *                    2.1234,3.23443,2.23456
 *                    ...
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch
 ****************************************************************************/

int fakesensor_init(int type, FAR const char *file_name,
                    int devno, uint32_t batch_number);

#ifdef __cplusplus
}
#endif

#endif
