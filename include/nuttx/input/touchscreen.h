/****************************************************************************
 * include/nuttx/input/touchscreen.h
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

/* The TOUCHSCREEN driver exports a standard character driver interface. By
 * convention, the touchscreen driver is registers as an input device at
 * /dev/inputN where N uniquely identifies the driver instance.
 *
 * This header file documents the generic interface that all NuttX
 * touchscreen devices must conform.  It adds standards and conventions on
 * top of the standard character driver interface.
 */

#ifndef __INCLUDE_NUTTX_INPUT_TOUCHSCREEN_H
#define __INCLUDE_NUTTX_INPUT_TOUCHSCREEN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/semaphore.h>
#include <time.h>

#ifdef CONFIG_INPUT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Common TSC IOCTL commands */

#define TSIOC_SETCALIB       _TSIOC(0x0001)  /* arg: Pointer to int calibration value */
#define TSIOC_GETCALIB       _TSIOC(0x0002)  /* arg: Pointer to int calibration value */
#define TSIOC_SETFREQUENCY   _TSIOC(0x0003)  /* arg: Pointer to uint32_t frequency value */
#define TSIOC_GETFREQUENCY   _TSIOC(0x0004)  /* arg: Pointer to uint32_t frequency value */
#define TSIOC_GETFWVERSION   _TSIOC(0x0005)  /* arg: Pointer to uint32_t firmware version value */

#define TSC_FIRST            0x0001          /* First common command */
#define TSC_NCMDS            5               /* Five common commands */

/* User defined ioctl commands are also supported.  However, the
 * TSC driver must reserve a block of commands as follows in order
 * prevent IOCTL command numbers from overlapping.
 *
 * This is generally done as follows.  The first reservation for
 * TSC driver A would look like:
 *
 *   TSC_A_FIRST         (TSC_FIRST + TSC_NCMDS)     <- First command
 *   TSC_A_NCMDS         42                          <- Number of commands
 *
 * IOCTL commands for TSC driver A would then be defined in a
 * TSC A header file like:
 *
 *   TSCIOC_A_CMD1       _TSIOC(TSC_A_FIRST + 0)
 *   TSCIOC_A_CMD2       _TSIOC(TSC_A_FIRST + 1)
 *   TSCIOC_A_CMD3       _TSIOC(TSC_A_FIRST + 2)
 *   ...
 *   TSCIOC_A_CMD42      _TSIOC(TSC_A_FIRST + 41)
 *
 * The next reservation would look like:
 *
 *   TSC_B_FIRST         (TSC_A_FIRST + TSC_A_NCMDS) <- Next command
 *   TSC_B_NCMDS         77                          <- Number of commands
 */

/* These definitions provide the meaning of all of the bits that may be
 * reported in the struct touch_point_s flags.
 */

#define TOUCH_DOWN           (1 << 0) /* A new touch contact is established */
#define TOUCH_MOVE           (1 << 1) /* Movement occurred with previously reported contact */
#define TOUCH_UP             (1 << 2) /* The touch contact was lost */
#define TOUCH_ID_VALID       (1 << 3) /* Touch ID is certain */
#define TOUCH_POS_VALID      (1 << 4) /* Hardware provided a valid X/Y position */
#define TOUCH_PRESSURE_VALID (1 << 5) /* Hardware provided a valid pressure */
#define TOUCH_SIZE_VALID     (1 << 6) /* Hardware provided a valid H/W contact size */
#define TOUCH_GESTURE_VALID  (1 << 7) /* Hardware provided a valid gesture */

/* These are definitions for touch gesture */

#define TOUCH_DOUBLE_CLICK   (0x00)
#define TOUCH_SLIDE_UP       (0x01)
#define TOUCH_SLIDE_DOWN     (0x02)
#define TOUCH_SLIDE_LEFT     (0x03)
#define TOUCH_SLIDE_RIGHT    (0x04)
#define TOUCH_PALM           (0x05)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure contains information about a single touch point.
 * Positional units are device specific.
 */

struct touch_point_s
{
  uint8_t  id;        /* Unique identifies contact; Same in all reports for the contact */
  uint8_t  flags;     /* See TOUCH_* definitions above */
  int16_t  x;         /* X coordinate of the touch point (uncalibrated) */
  int16_t  y;         /* Y coordinate of the touch point (uncalibrated) */
  int16_t  h;         /* Height of touch point (uncalibrated) */
  int16_t  w;         /* Width of touch point (uncalibrated) */
  uint16_t gesture;   /* Gesture of touchscreen contact */
  uint16_t pressure;  /* Touch pressure */
  uint64_t timestamp; /* Touch event time stamp, in microseconds */
};

/* The typical touchscreen driver is a read-only, input character device
 * driver.the driver write() method is not supported and any attempt to
 * open the driver in any mode other than read-only will fail.
 *
 * Data read from the touchscreen device consists only of touch events and
 * touch sample data.  This is reflected by struct touch_sample_s.  This
 * structure is returned by either the driver read method.
 *
 * On some devices, multiple touchpoints may be supported. So this top level
 * data structure is a struct touch_sample_s that "contains" a set of touch
 * points.  Each touch point is managed individually using an ID that
 * identifies a touch from first contact until the end of the contact.
 */

struct touch_sample_s
{
  int npoints;                   /* The number of touch points in point[] */
  struct touch_point_s point[1]; /* Actual dimension is npoints */
};

#define SIZEOF_TOUCH_SAMPLE_S(n) \
  (sizeof(struct touch_sample_s) + ((n) - 1) * sizeof(struct touch_point_s))

/* This structure is for touchscreen lower half driver */

struct touch_lowerhalf_s
{
  uint8_t       maxpoint;       /* Maximal point supported by the touchscreen */
  FAR void      *priv;          /* Save the upper half pointer */

  /**************************************************************************
   * Name: control
   *
   * Description:
   *   Users can use this interface to implement custom IOCTL.
   *
   * Arguments:
   *   lower   - The instance of lower half of touchscreen device.
   *   cmd     - User defined specific command.
   *   arg     - Argument of the specific command.
   *
   * Return Value:
   *   Zero(OK) on success; a negated errno value on failure.
   *   -ENOTTY - The command is not supported.
   **************************************************************************/

  CODE int (*control)(FAR struct touch_lowerhalf_s *lower,
                      int cmd, unsigned long arg);

  /**************************************************************************
   * Name: write
   *
   * Description:
   *   Users can use this interface to implement custom write.
   *
   * Arguments:
   *   lower   - The instance of lower half of touchscreen device.
   *   buffer  - User defined specific buffer.
   *   buflen  - User defined specific buffer size.
   *
   * Return Value:
   *   Number of bytes written；a negated errno value on failure.
   *
   **************************************************************************/

  CODE ssize_t (*write)(FAR struct touch_lowerhalf_s *lower,
                        FAR const char *buffer, size_t buflen);
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t touch_get_time(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: touch_event
 *
 * Description:
 *   The lower half driver pushes touchscreen events through this interface,
 *   provided by touchscreen upper half.
 *
 * Arguments:
 *   priv    - Upper half driver handle.
 *   sample  - pointer to data of touch point event.
 ****************************************************************************/

void touch_event(FAR void *priv, FAR const struct touch_sample_s *sample);

/****************************************************************************
 * Name: touch_register
 *
 * Description:
 *   This function registers a touchscreen device, the upper half binds
 *   with hardware device through the lower half instance.
 *
 * Arguments:
 *   lower     - A pointer of lower half instance.
 *   path      - The path of touchscreen device. such as "/dev/input0"
 *   nums      - Number of the touch points structure.
 *
 * Return:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int touch_register(FAR struct touch_lowerhalf_s *lower,
                   FAR const char *path, uint8_t nums);

/****************************************************************************
 * Name: touch_unregister
 *
 * Description:
 *   This function is used to touchscreen driver to unregister and
 *   release the occupied resources.
 *
 * Arguments:
 *   lower     - A pointer to an insatnce of touchscreen lower half driver.
 *   path      - The path of touchscreen device. such as "/dev/input0"
 ****************************************************************************/

void touch_unregister(FAR struct touch_lowerhalf_s *lower,
                      FAR const char *path);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT */
#endif /* __INCLUDE_NUTTX_INPUT_TOUCHSCREEN_H */
