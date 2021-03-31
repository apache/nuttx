/****************************************************************************
 * arch/arm/include/cxd56xx/geofence.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_GEOFENCE_H
#define __ARCH_ARM_INCLUDE_CXD56XX_GEOFENCE_H

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Start geofence monitoring.
 * This command is used to start the geofence monitoring.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GEOFENCE_IOCTL_START 1

/* Stop geofence monitoring.
 * This command is used to stop the geofence monitoring.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GEOFENCE_IOCTL_STOP 2

/* Add region.
 * This command is used to add the region.
 *
 * param arg
 * Parameter is struct cxd56_geofence_region_s.
 */

#define CXD56_GEOFENCE_IOCTL_ADD 3

/* Modify region.
 * This command is used to modify the region.
 *
 * param arg
 * Parameter is struct cxd56_geofence_region_s.
 */

#define CXD56_GEOFENCE_IOCTL_MODIFY 4

/* Delete region.
 * This command is used to delete the region.
 *
 * param arg
 * Parameter is region id.
 */

#define CXD56_GEOFENCE_IOCTL_DELETE 5

/* Delete all region.
 * This command is used to delete all region.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 */

#define CXD56_GEOFENCE_IOCTL_ALL_DELETE 6

/* Get region data.
 * This command is used to get region data.
 *
 * param arg
 * Parameter is struct cxd56_geofence_region_s pointer
 * Latitude and longitude and radius data of specified id is stored.
 */

#define CXD56_GEOFENCE_IOCTL_GET_REGION_DATA 7

/* Get used id.
 * This command is used to get used region id.
 *
 * param arg
 * Parameter is uint32_t data pointer.
 * The used id is represented by bit field.
 * For example, when ID0 and ID19 are used,
 * since bit0 and bit19 are set, the return value is 0x00080001.
 */

#define CXD56_GEOFENCE_IOCTL_GET_USED_ID 8

/* Get all status.
 * This command is used to get all region status.
 *
 * param arg
 * Parameter is Unnecessary. Set Zero.
 * All region status will stored in next read data.
 */

#define CXD56_GEOFENCE_IOCTL_GET_ALL_STATUS 9

/* Set goefence operation mode
 * This command is used to set operation mode.
 *
 * param arg
 * Parameter is struct cxd56_geofence_mode_s.
 */

#define CXD56_GEOFENCE_IOCTL_SET_MODE 10

/* check macros for GNSS commands */

#define CXD56_GEOFENCE_IOCTL_INVAL 0
#define CXD56_GEOFENCE_IOCTL_MAX   11

/* The transition type indicating that the user exits the region. */

#define CXD56_GEOFENCE_TRANSITION_EXIT    0

/* The transition type indicating that the user enters the region. */

#define CXD56_GEOFENCE_TRANSITION_ENTER   1

/* The transition type indicating that the user enters and
 * dwells in region for a given period of time.
 */

#define CXD56_GEOFENCE_TRANSITION_DWELL   2

/* MAX number of region on the CXD56xx. */

#define CXD56_GEOFENCE_REGION_CAPACITY    20

/* Region center point and radius data
 *
 *  The latitude and longtitude data format is
 *  integer value multiplied by 1000000.
 *  Example: When latitude is 35.123456, specify 35123456.
 */

struct cxd56_geofence_region_s
{
  /* Region ID  The range of ID is 0 to 19. */

  uint8_t id;

  /* Latitude (degree) of the center position of the region. */

  long latitude;

  /* Longitude (degree) of the center position of the region. */

  long longitude;

  /* Radius (m) of the region. */

  uint16_t radius;
};

/* Geofence mode setting parameter */

struct cxd56_geofence_mode_s
{
  uint16_t deadzone;           /* dead zone [meter] */
  uint16_t dwell_detecttime;   /* Dewlling period time [sec] */
};

/* The transition data */

struct cxd56_geofence_trans_s
{
  /* Region ID */

  uint8_t id;

  /* Transition status.
   * The status is #CXD56_GEOFENCE_TRANSITION_EXIT or
   * #CXD56_GEOFENCE_TRANSITION_ENTER or #CXD56_GEOFENCE_TRANSITION_DWELL.
   */

  uint8_t status;
};

/* Geofence output data structure. */

struct cxd56_geofence_status_s
{
  /* Updated region ID count */

  uint8_t update;

  /* The detail data of updated region ID */

  struct cxd56_geofence_trans_s status[CXD56_GEOFENCE_REGION_CAPACITY];
};

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_GEOFENCE_H */
