/****************************************************************************
 * include/nuttx/sensors/dhtxx.h
 *
 *   Copyright (C) 2018 Abdelatif GUETTOUCHE. All rights reserved.
 *   Author: Abdelatif GUETTOUCHE <abdelatif.guettouche@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  /* Timeout accured waiting for data. */ 

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
