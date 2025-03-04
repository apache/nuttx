/****************************************************************************
 * include/nuttx/sensors/sensor.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_SENSOR_H
#define __INCLUDE_NUTTX_SENSORS_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <nuttx/fs/fs.h>
#include <nuttx/clock.h>
#include <nuttx/uorb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Body coordinate system position P0:
 *
 *          +y
 *          |
 *          |
 *          |
 *          |
 *          .------>+x
 *         /
 *        /
 *       /
 *      /
 *     +z
 *
 */

#define SENSOR_BODY_COORDINATE_P0                   0

/* Body coordinate system position P1:
 *
 *          .------>+y
 *         /|
 *        / |
 *       /  |
 *      /   |
 *     +z  -x
 *
 */

#define SENSOR_BODY_COORDINATE_P1                   1

/* Body coordinate system position P2:
 *
 * -x<------.
 *         /|
 *        / |
 *       /  |
 *      /   |
 *     +z  -y
 *
 */

#define SENSOR_BODY_COORDINATE_P2                   2

/* Body coordinate system position P3:
 *
 *          +x
 *          |
 *          |
 *          |
 *          |
 * -y<------.
 *         /
 *        /
 *       /
 *      /
 *     +z
 *
 */

#define SENSOR_BODY_COORDINATE_P3                   3

/* Body coordinate system position P4:
 *
 *          +y  -z
 *          |   /
 *          |  /
 *          | /
 *          |/
 * -x<------.
 *
 */

#define SENSOR_BODY_COORDINATE_P4                   4

/* Body coordinate system position P5:
 *
 *          +y  -z
 *          |   /
 *          |  /
 *          | /
 *          |/
 * -x<------.
 *
 */

#define SENSOR_BODY_COORDINATE_P5                   5

/* Body coordinate system position P6:
 *
 *              -z
 *              /
 *             /
 *            /
 *           /
 * -x<------.
 *          |
 *          |
 *          |
 *          |
 *         -y
 *
 */

#define SENSOR_BODY_COORDINATE_P6                   6

/* Body coordinate system position P7:
 *
 *         +x   -z
 *          |   /
 *          |  /
 *          | /
 *          |/
 *          .------->y
 *
 */

#define SENSOR_BODY_COORDINATE_P7                   7

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t sensor_get_timestamp(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The sensor lower half driver interface */

struct sensor_lowerhalf_s;
struct sensor_ops_s
{
  /**************************************************************************
   * Name: open
   *
   * Description:
   *   The open method differs from the activate method with true because
   *   it's called and turned off every times, and it receives the pointer
   *   of file and the instance of lowerhalf sensor driver. It uses to do
   *   something about initialize for every user.
   *
   * Input Parameters:
   *   lower - The instance of lower half sensor driver
   *   filep - The pointer of file, represents each user using the sensor
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*open)(FAR struct sensor_lowerhalf_s *lower,
                   FAR struct file *filep);

  /**************************************************************************
   * Name: close
   *
   * Description:
   *   The close method differs from the activate method with false because
   *   it's called and turned off every times, and it receives the pointer
   *   of file and the instance of lowerhalf sensor driver. It uses to do
   *   something about uninitialize for every user.
   *
   * Input Parameters:
   *   lower - The instance of lower half sensor driver.
   *   filep - The pointer of file, represents each user using the sensor.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*close)(FAR struct sensor_lowerhalf_s *lower,
                    FAR struct file *filep);

  /**************************************************************************
   * Name: activate
   *
   * Description:
   *   Enable or disable sensor device. when enable sensor, sensor will
   *   work in  current mode(if not set, use default mode). when disable
   *   sensor, it will disable sense path and stop convert.
   *
   * Input Parameters:
   *   lower  - The instance of lower half sensor driver
   *   filep  - The pointer of file, represents each user using the sensor.
   *   enable - true(enable) and false(disable)
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*activate)(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, bool enable);

  /**************************************************************************
   * Name: set_interval
   *
   * Description:
   *   Set the sensor output data period in microseconds for a given sensor.
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
   *   lower     - The instance of lower half sensor driver.
   *   filep     - The pointer of file, represents each user using sensor.
   *   period_us - the time between samples, in us, it may be overwrite by
   *               lower half driver.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*set_interval)(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR uint32_t *period_us);

  /**************************************************************************
   * Name: batch
   *
   * Description:
   *   Set sensor's maximum report latency in microseconds.
   *
   *   This function can be called while the sensor is activated,
   *   in which case it must not cause any sensor measurements to be lost.
   *   So, it is necessary to flush fifo or read ready data from data
   *   register to prevent data lost before you using batch mode.
   *
   *   This sensor default mode isn't batch mode, so you need call this
   *   function and *latency_us != 0.
   *   If *latency_us > max_report_latency it will be truncated to
   *   max_report_latency and return *latency_us to user
   *   And you must flush fifo data to prevent data lost, then adjust
   *   latency.
   *
   *   You can exit batch mode by call this function with *latency_us = 0.
   *   And you must flush fifo data to prevent data lost, then stop batch.
   *
   *   If sensor doesn't support batching (FIFO size zero), set batch to
   *   NULL.
   *
   *   You must set interval by calling set_interval before calling batch(),
   *   othrewise, -EINVAL is returned.
   *
   *   The reason why you don't have flush operation is that you need to push
   *   the prepared data out before adjusting the latency to ensure that the
   *   data will not be lost.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *   latency_us - the time between batch data, in us. It may by overwrite
   *                by lower half driver.
   *
   * Returned Value:
   *   Zero (OK) or positive on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*batch)(FAR struct sensor_lowerhalf_s *lower,
                    FAR struct file *filep,
                    FAR uint32_t *latency_us);

  /**************************************************************************
   * Name: fetch
   *
   * Fetch sensor register data by this function. It will use buffer of
   * userspace provided and disables intermediate buffer of upper half. It's
   * recommend that the lowerhalf driver writer to use this function for
   * slower sensor ODR (output data rate) of sensor because this way saves
   * space and it's simple.
   *
   * If fetch isn't NULL, upper half driver will disable intermediate
   * buffer and userspace can't set buffer size by ioctl.
   *
   * You can call this function to read sensor register data by I2C/SPI bus
   * when open mode is non-block, and poll are always successful.
   * When you call this function and open mode is block, you will wait
   * until sensor data ready, then read sensor data.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *   buffer     - The buffer of receive sensor event, it's provided by
   *                file_operation::sensor_read.
   *   buflen     - The size of buffer.
   *
   * Returned Value:
   *   The size of read buffer returned on success; a negated errno value
   *   on failure.
   *
   **************************************************************************/

  CODE int (*fetch)(FAR struct sensor_lowerhalf_s *lower,
                    FAR struct file *filep,
                    FAR char *buffer, size_t buflen);

  /**************************************************************************
   * Name: flush
   *
   * When sensor data accumulates in the hardware buffer but does not
   * reach the watermark, the upper-layer application can immediately push
   * the fifo data to the upper layer circbuffer through the flush operation.
   *
   * The flush operation is an asynchronous operation. The lower half driver
   * must call push event with data is NULL and len is zero when the flush
   * action is completed, then upper half driver triggers the POLLPRI event,
   * and update user state event to tell application the flush complete
   * event.
   *
   * You can call the flush operation at any time. When the sensor is not
   * activated, flsuh returns -EINVAL. When the sensor does not support fifo,
   * it immediately returns the POLLPRI event, indicating that the flush
   * is completed.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*flush)(FAR struct sensor_lowerhalf_s *lower,
                    FAR struct file *filep);

  /**************************************************************************
   * Name: selftest
   *
   * Selftest allows for the testing of the mechanical and electrical
   * portions of the sensors. When the selftest is activated, the
   * electronics cause the sensors to be actuated and produce an output
   * signal. The output signal is used to observe the selftest response.
   * When the selftest response exceeds the min/max values,
   * the part is deemed to have failed selftest.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *   arg        - The parameters associated with selftest.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*selftest)(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep,
                       unsigned long arg);

  /**************************************************************************
   * Name: set_calibvalue
   *
   * The calibration value to be written in or the non-volatile memory of the
   * sensor or dedicated registers. At each power-on, so that the values read
   * from the sensor are already corrected. When the device is calibrated,
   * the absolute accuracy will be better than before.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *   arg        - The parameters associated with calibration value.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*set_calibvalue)(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             unsigned long arg);

  /**************************************************************************
   * Name: calibrate
   *
   * This operation can trigger the calibration operation, and if the
   * calibration operation is short-lived, the calibration result value can
   * be obtained at the same time, the calibration value to be written in or
   * the non-volatile memory of the sensor or dedicated registers. When the
   * upper-level application calibration is completed, the current
   * calibration value of the sensor needs to be obtained and backed up,
   * so that the last calibration value can be directly obtained after
   * power-on.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *   arg        - The parameters associated with calibration value.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *
   **************************************************************************/

  CODE int (*calibrate)(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        unsigned long arg);

  /**************************************************************************
   * Name: get_info
   *
   * With this method, the user can obtain information about the current
   * device. The name and vendor information cannot exceed
   * SENSOR_INFO_NAME_SIZE.
   *
   * Input Parameters:
   *   lower   - The instance of lower half sensor driver.
   *   filep   - The pointer of file, represents each user using sensor.
   *   info    - Device information structure pointer.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *   -ENOTTY - The cmd don't support.
   *
   **************************************************************************/

  CODE int (*get_info)(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep,
                       FAR struct sensor_device_info_s *info);

  /**************************************************************************
   * Name: control
   *
   * With this method, the user can set some special config for the sensor,
   * such as changing the custom mode, setting the custom resolution, reset,
   * etc, which are all parsed and implemented by lower half driver.
   *
   * Input Parameters:
   *   lower      - The instance of lower half sensor driver.
   *   filep      - The pointer of file, represents each user using sensor.
   *   cmd        - The special cmd for sensor.
   *   arg        - The parameters associated with cmd.
   *
   * Returned Value:
   *   Zero (OK) on success; a negated errno value on failure.
   *   -ENOTTY    - The cmd don't support.
   *
   **************************************************************************/

  CODE int (*control)(FAR struct sensor_lowerhalf_s *lower,
                      FAR struct file *filep,
                      int cmd, unsigned long arg);
};

/* This structure is the generic form of state structure used by lower half
 * Sensor driver.
 */

typedef CODE ssize_t (*sensor_push_event_t)(FAR void *priv,
                                            FAR const void *data,
                                            size_t bytes);
typedef CODE void (*sensor_notify_event_t)(FAR void *priv);

struct sensor_lowerhalf_s
{
  /* The type of sensor device */

  int type;

  /* The number of events that the circular buffer can hold.
   * This sensor circular buffer is used to slove issue that application
   * can't read sensor event in time. If this number of events is too large,
   * the latency of sensor event will be too larage. If the number of events
   * is too small, the event will be overwrite before application read them.
   * So, it's recommended to set according to sensor odr. If odr is low, you
   * can set to one. If odr is high, you can set to two or three.
   *
   * If device support batch mode, the number of events that hardware fifo
   * hold maximum number of samples, must be aligned with size of
   * struct sensor_xxx.
   */

  uint32_t nbuffer;

  /* The lower half sensor driver operations */

  FAR const struct sensor_ops_s *ops;

  union
    {
      /**********************************************************************
       * Name: push_event
       *
       * Description:
       *   Lower half driver pushes a sensor event by calling this function.
       *   It is provided by upper half driver to lower half driver.
       *
       * Input Parameters:
       *   priv   - Upper half driver handle.
       *   data   - The buffer of event, it can be all type of sensor events.
       *   bytes  - The number of bytes of sensor event.
       *
       * Returned Value:
       *   The bytes of push is returned when success;
       *   A negated errno value is returned on any failure.
       *
       **********************************************************************/

      sensor_push_event_t push_event;

      /**********************************************************************
       * Name: notify_event
       *
       * Description:
       *   Lower half driver notifies that sensor data is ready and can be
       *   read by fetch. It is provided by upper half driver to lower half
       *   driver.
       *
       *   This api is used when sensor_ops_s::fetch isn't NULL.
       *
       * Input Parameters:
       *   priv   - Upper half driver handle
       *
       **********************************************************************/

      sensor_notify_event_t notify_event;
    };

/****************************************************************************
 * Name: sensor_lock/sensor_unlock
 *
 * Description:
 *   Lower half driver can lock/unlock upper half driver by this interface.
 *
 * Input Parameters:
 *   priv   - Upper half driver handle
 *
 ****************************************************************************/

  CODE void (*sensor_lock)(FAR void * priv);
  CODE void (*sensor_unlock)(FAR void * priv);

  /* The private opaque pointer to be passed to upper-layer during callback */

  FAR void *priv;

  /* The flag is used to indicate that the validity of sensor data is
   * persistent, such as battery status information, switch information, etc.
   */

  bool persist;
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

/****************************************************************************
 * Name: sensor_remap_vector_raw16
 *
 * Description:
 *   This function remap the sensor data according to the place position on
 *   board. The value of place is determined base on g_remap_tbl.
 *
 * Input Parameters:
 *   in    - A pointer to input data need remap.
 *   out   - A pointer to output data.
 *   place - The place position of sensor on board,
 *           ex:SENSOR_BODY_COORDINATE_PX
 *
 ****************************************************************************/

void sensor_remap_vector_raw16(FAR const int16_t *in, FAR int16_t *out,
                               int place);

/****************************************************************************
 * "Upper Half" Sensor Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   You can register the character device by node name format based on the
 *   type of sensor. Multiple types of the same type are distinguished by
 *   numbers. eg: accel0, accel1. This type of sensor must be less than
 *   SENSOR_TYPE_COUNT. This API corresponds to the sensor_unregister.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persist as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0. If the
 *           devno alerady exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_register(FAR struct sensor_lowerhalf_s *dev, int devno);

/****************************************************************************
 * Name: sensor_custom_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Sensor driver with the
 *   "upper half" Sensor device and registers that device so that can be used
 *   by application code.
 *
 *   You can register the character device type by specific path and esize.
 *   This API corresponds to the sensor_custom_unregister.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persist as long
 *           as the driver persists.
 *   path  - The user specifies path of device. ex: /dev/uorb/xxx.
 *   esize - The element size of intermediate circular buffer.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sensor_custom_register(FAR struct sensor_lowerhalf_s *dev,
                           FAR const char *path, size_t esize);

/****************************************************************************
 * Name: sensor_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the sensor_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 *
 ****************************************************************************/

void sensor_unregister(FAR struct sensor_lowerhalf_s *dev, int devno);

/****************************************************************************
 * Name: sensor_custom_unregister
 *
 * Description:
 *   This function unregisters character node and releases all resource from
 *   upper half driver. This API corresponds to the sensor_custom_register.
 *
 * Input Parameters:
 *   dev   - A pointer to an instance of lower half sensor driver. This
 *           instance is bound to the sensor driver and must persists as long
 *           as the driver persists.
 *   path  - The user specifies path of device, ex: /dev/uorb/xxx
 *
 ****************************************************************************/

void sensor_custom_unregister(FAR struct sensor_lowerhalf_s *dev,
                              FAR const char *path);

/****************************************************************************
 * Name: usensor_initialize
 *
 * Description:
 *   This function registers usensor character node "/dev/usensor", so that
 *   application can register user sensor by this node. The node will
 *   manager all user sensor in this character dirver.
 *
 ****************************************************************************/

#ifdef CONFIG_USENSOR
int usensor_initialize(void);
#endif

/****************************************************************************
 * Name: sensor_rpmsg_register
 *
 * Description:
 *   This function registers rpmsg takeover for the real lower half, and
 *   initialize rpmsg resource.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   path  - The path of character node, ex: /dev/uorb/xxx.
 *
 * Returned Value:
 *   The takeover rpmsg lowerhalf returned on success, NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_RPMSG
FAR struct sensor_lowerhalf_s *sensor_rpmsg_register(
                                       FAR struct sensor_lowerhalf_s *lower,
                                       FAR const char *path);
#endif

/****************************************************************************
 * Name: sensor_rpmsg_unregister
 *
 * Description:
 *   This function unregisters rpmsg takeover for the real lower half, and
 *   release rpmsg resource. This API corresponds to the
 *   sensor_rpmsg_register.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_RPMSG
void sensor_rpmsg_unregister(FAR struct sensor_lowerhalf_s *lower);
#endif

/****************************************************************************
 * Name: sensor_rpmsg_initialize
 *
 * Description:
 *   This function initializes the context of sensor rpmsg, registers
 *   rpmsg callback and prepares enviroment to intercat with remote sensor.
 *
 * Returned Value:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_RPMSG
int sensor_rpmsg_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_SENSORS_SENSOR_H */
