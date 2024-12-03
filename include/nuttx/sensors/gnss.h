/****************************************************************************
 * include/nuttx/sensors/gnss.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_GNSS_H
#define __INCLUDE_NUTTX_SENSORS_GNSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SENSOR_GNSS_IDX_GNSS               0
#define SENSOR_GNSS_IDX_GNSS_SATELLITE     1
#define SENSOR_GNSS_IDX_GNSS_MEASUREMENT   2
#define SENSOR_GNSS_IDX_GNSS_CLOCK         3
#define SENSOR_GNSS_IDX_GNSS_GEOFENCE      4
#define SENSOR_GNSS_IDX_GNSS_MAX           5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The GNSS lower half driver interface */

struct gnss_lowerhalf_s;
struct gnss_ops_s
{
  /**************************************************************************
   * Name: activate
   *
   * Description:
   *   Enable or disable GNSS device. when enable GNSS, GNSS will
   *   work in current mode(if not set, use default mode). when disable
   *   GNSS, it will disable sense path and stop work.
   *
   * Input Parameters:
   *   lower  - The instance of lower half GNSS driver
   *   filep  - The pointer of file, represents each user using the GNSS.
   *   enable - true(enable) and false(disable)
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*activate)(FAR struct gnss_lowerhalf_s *lower,
                       FAR struct file *filep, bool enable);

  /**************************************************************************
   * Name: set_interval
   *
   * Description:
   *   Set the GNSS output data period in microseconds for a given GNSS.
   *   If *period_us > max_delay it will be truncated to max_delay and if
   *   *period_us < min_delay it will be replaced by min_delay.
   *
   *   The lower-half can update update *period_us to reflect the actual
   *   period in case the value is rounded up to nearest supported value.
   *
   *   Before changing the interval, you need to push the prepared data to
   *   ensure that they are not lost.
   *
   * Input Parameters:
   *   lower     - The instance of lower half GNSS driver.
   *   filep     - The pointer of file, represents each user using GNSS.
   *   period_us - the time between samples, in us, it may be overwrite by
   *               lower half driver.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*set_interval)(FAR struct gnss_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR uint32_t *period_us);

  /**************************************************************************
   * Name: control
   *
   * With this method, the user can set some special config for the GNSS,
   * such as changing the custom mode, setting the custom resolution, reset,
   * etc, which are all parsed and implemented by lower half driver.
   *
   * Input Parameters:
   *   lower      - The instance of lower half GNSS driver.
   *   filep      - The pointer of file, represents each user using GNSS.
   *   cmd        - The special cmd for GNSS.
   *   arg        - The parameters associated with cmd.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *   -ENOTTY    - The cmd don't support.
   *
   **************************************************************************/

  CODE int (*control)(FAR struct gnss_lowerhalf_s *lower,
                      FAR struct file *filep,
                      int cmd, unsigned long arg);

  /**************************************************************************
   * Name: inject_data
   *
   * With this method, the user can inject some data to GNSS driver,
   * such as inject lto data, utc data, etc.
   *
   * Input Parameters:
   *   lower      - The instance of lower half GNSS driver.
   *   filep      - The pointer of file, represents each user using GNSS.
   *   buffer     - The buffer of inject data.
   *   buflen     - The length of buffer.
   *
   * Returned Value:
   *   The written bytes returned on success;
   *   A negated errno value on failure.
   *
   **************************************************************************/

  CODE ssize_t (*inject_data)(FAR struct gnss_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR const void *buffer, size_t buflen);
};

/* This structure is the generic form of state structure used by lower half
 * GNSS driver.
 */

typedef CODE void (*gnss_push_data_t)(FAR void *priv, FAR const void *data,
                                      size_t bytes, bool is_nmea);

typedef CODE void (*gnss_push_event_t)(FAR void *priv, FAR const void *data,
                                       size_t bytes, int type);

struct gnss_lowerhalf_s
{
  /* The lower half GNSS driver operations */

  FAR const struct gnss_ops_s *ops;

  /* Lower half driver pushes raw data by calling this function.
   * It is provided by upper half driver to lower half driver,
   * if paramenter is_nmea is true, the data includes nmea message.
   */

  gnss_push_data_t push_data;

  /* Lower half driver pushes GNSS event by calling this function.
   * It is provided by upper half driver to lower half driver.
   * lower half can use type to description the data type, eg:
   * SENSOR_TYPE_GNSS, SENSOR_TYPE_GNSS_SATELLITE
   * SENSOR_TYPE_GNSS_MEASUREMENT, SENSOR_TYPE_GNSS_CLOCK
   */

  gnss_push_event_t push_event;

  /* The private opaque pointer to be passed to upper-layer during callback */

  FAR void *priv;
};

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
 * Name: gnss_register
 *
 * Description:
 *   This function binds an instance of a "lower half" GNSS driver with the
 *   "upper half" GNSS device and registers that device so that can be used
 *   by application code.
 *
 * Input Parameters:
 *   dev     - A pointer to an instance of lower half GNSS driver. This
 *             instance is bound to the GNSS driver and must persist as long
 *             as the driver persists.
 *   devno   - The user specifies which device of this type, from 0. If the
 *             devno alerady exists, -EEXIST will be returned.
 *   nbuffer - The number of events that the circular buffer can hold.
 *   count   - The array size of nbuffer.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int gnss_register(FAR struct gnss_lowerhalf_s *dev, int devno,
                  uint32_t nbuffer[], size_t count);

/****************************************************************************
 * Name: gnss_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the gnss_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half GNSS driver. This
 *           instance is bound to the GNSS driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 *
 ****************************************************************************/

void gnss_unregister(FAR struct gnss_lowerhalf_s *dev, int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_GNSS_H */
