/****************************************************************************
 * include/nuttx/sensors/max31855.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MAX31855_H
#define __INCLUDE_NUTTX_SENSORS_MAX31855_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#if defined(CONFIG_SENSORS_MAX31855)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#define MAX31855_SPI_MAXFREQ 4000000

struct spi_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: max31855_register
 *
 * Description:
 *   This function will register the max31855 driver as /dev/tempN
 *   where N is the minor device number.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register.  E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31855
 *   devid   - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int max31855_register(FAR const char *devpath,
                      FAR struct spi_dev_s *spi,
                      uint16_t devid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MAX31855 */
#endif /* __INCLUDE_NUTTX_SENSORS_MAX31855_H */
