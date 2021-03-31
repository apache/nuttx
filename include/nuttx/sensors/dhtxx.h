/****************************************************************************
 * include/nuttx/sensors/dhtxx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_DHTXX_H
#define __INCLUDE_NUTTX_SENSORS_DHTXX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum dhtxx_type_e
{
  DHTXX_DHT11,
  DHTXX_DHT12,
  DHTXX_DHT21,
  DHTXX_DHT22,
  DHTXX_DHT33,
  DHTXX_DHT44
};

struct dhtxx_config_s
{
  CODE void (*config_data_pin)(FAR struct dhtxx_config_s *state, bool mode);
  CODE void (*set_data_pin)(FAR struct dhtxx_config_s *state, bool value);
  CODE bool (*read_data_pin)(FAR struct dhtxx_config_s *state);
  CODE int64_t (*get_clock)(FAR struct dhtxx_config_s *state);
  enum dhtxx_type_e type;
};

enum dhtxx_status_e
{
  /* Timeout occurred waiting for data. */

  DHTXX_TIMEOUT,

  /* Checksum sent and calculated differ. */

  DHTXX_CHECKSUM_ERROR,

  /* Data read exceeds the sensor's measurement range. */

  DHTXX_READ_ERROR,

  /* Data read successfully. */

  DHTXX_SUCCESS
};

struct dhtxx_sensor_data_s
{
  float hum;
  float temp;
  enum dhtxx_status_e status;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: dhtxx_register
 *
 * Description:
 *   Register the Dhtxx character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/dht0"
 *   config  - The dhtxx config.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int dhtxx_register(FAR const char *devpath,
                   FAR struct dhtxx_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_DHTXX_H */
