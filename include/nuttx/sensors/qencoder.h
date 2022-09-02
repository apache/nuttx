/****************************************************************************
 * include/nuttx/sensors/qencoder.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_QENCODER_H
#define __INCLUDE_NUTTX_SENSORS_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * CONFIG_SENSORS_QENCODER - Enables support for the quadrature encoder upper
 * half
 */

/* IOCTL Commands ***********************************************************/

/* The Quadrature Encode module uses a standard character driver framework.
 * However, since the driver is a device control interface rather than a
 * data transfer interface, the majority of the functionality is implemented
 * in driver ioctl calls.  The PWM ioctl commands are listed below:
 *
 * QEIOC_POSITION - Get the current position from the encoder.
 *   Argument: int32_t pointer to the location to return the position.
 * QEIOC_RESET - Reset the position to zero.
 *   Argument: None
 * QEIOC_SETPOSMAX - Set the maximum position for the encoder.
 *   Argument: uint32_t maximum position
 * QEIOC_SETINDEX - Set the index position for the encoder.
 *   Argument: uint32_t index position
 * QEIOC_GETINDEX - Get the index position and count of the encoder.
 *   The structure also contains current position so QEIOC_POSITION
 *   is not required when QEIOC_GETINDEX is used.
 *   Argment: qe_index_s structure (refer below)
 */

#define QEIOC_POSITION     _QEIOC(0x0001) /* Arg: int32_t* pointer */
#define QEIOC_RESET        _QEIOC(0x0002) /* Arg: None */
#define QEIOC_SETPOSMAX    _QEIOC(0x0003) /* Arg: uint32_t */
#define QEIOC_SETINDEX     _QEIOC(0x0004) /* Arg: uint32_t */
#define QEIOC_GETINDEX     _QEIOC(0x0005) /* Arg: qe_index_s struct */

#define QE_FIRST           0x0001         /* First required command */
#define QE_NCMDS           5              /* 5 required commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half QE driver to the lower-half QE driver via the ioctl()
 * method of the QE lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See arch/arm/src/tiva/tiva_qencoder.h (Not usable at that location) */

#define QE_TIVA_FIRST      (QE_FIRST + QE_NCMDS)
#define QE_TIVA_NCMDS      5

/* See include/nuttx/sensors/as5048b.h */

#define QE_AS5048B_FIRST   (QE_TIVA_FIRST + QE_TIVA_NCMDS)
#define QE_AS5048B_NCMDS   4

/* See arch/arm/src/imxrt/imxrt_enc.h */

#define QE_IMXRT_FIRST     (QE_AS5048B_FIRST + QE_AS5048B_NCMDS)
#define QE_IMXRT_NCMDS     7

/* See include/nuttx/sensors/as5048a.h */

#define QE_AS5048A_FIRST   (QE_IMXRT_FIRST + QE_IMXRT_NCMDS)
#define QE_AS5048A_NCMDS   4

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the vtable that is used to by the upper half quadrature encoder
 * to call back into the lower half quadrature encoder.
 */

struct qe_lowerhalf_s;
struct qe_ops_s
{
  /* This method is called when the driver is opened.  The lower half driver
   * should configure and initialize the device so that it is ready for use.
   * The initial position value should be zero.
   */

  CODE int (*setup)(FAR struct qe_lowerhalf_s *lower);

  /* This method is called when the driver is closed.  The lower half driver
   * should stop data collection, free any resources, disable timer hardware,
   * and put the system into the lowest possible power usage state
   */

  CODE int (*shutdown)(FAR struct qe_lowerhalf_s *lower);

  /* Return the current position measurement. */

  CODE int (*position)(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos);

  /* Set the maximum encoder position. */

  CODE int (*setposmax)(FAR struct qe_lowerhalf_s *lower, uint32_t pos);

  /* Reset the position measurement to zero. */

  CODE int (*reset)(FAR struct qe_lowerhalf_s *lower);

  /* Set the index pin position */

  CODE int (*setindex)(FAR struct qe_lowerhalf_s *lower, uint32_t pos);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct qe_lowerhalf_s *lower,
                    int cmd, unsigned long arg);
};

/* Structure qe_index_s is used for QEIOC_GETINDEX call. This call returns
 * current encoder position, the last index position and number of index
 * occurances.
 */

struct qe_index_s
{
  int32_t qenc_pos;     /* Qencoder actual position */
  int32_t indx_pos;     /* Index last position */
  int16_t indx_cnt;     /* Number of index occurances */
};

/* This is the interface between the lower half quadrature encoder driver
 * and the upper half quadrature encoder driver.  A (device-specific)
 * instance of this structure is passed to the upper-half driver when the
 * quadrature encoder driver is registered.
 *
 * Normally that lower half logic will have its own, custom state structure
 * that is simply cast to struct qe_lowerhalf_s.  In order to perform such
 * casts, the initial fields of the custom state structure match the initial
 * fields of the following generic lower half state structure.
 */

struct qe_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;

  /* The custom timer state structure may include additional fields after
   * the pointer to the callback structure.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: qe_register
 *
 * Description:
 *   Register the Quadrature Encoder lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int qe_register(FAR const char *devpath, FAR struct qe_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_QENCODER */
#endif /* __INCLUDE_NUTTX_SENSORS_QENCODER_H */
