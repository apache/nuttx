/****************************************************************************
 * drivers/input/ft5x06.h
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

/* References:
 *   "FT5x06", FocalTech Systems Co., Ltd, D-FT5x06-1212-V4.0, Revised
 *   Dec. 18, 2012
 */

/* The FT5x06 Series ICs are single-chip capacitive touch panel controller
 * ICs with a built-in 8 bit Micro-controller unit (MCU).  They adopt the
 * mutual capacitance approach, which supports true multi-touch capability.
 * In conjunction with a mutual capacitive touch panel, the FT5x06 have
 * user-friendly input functions, which can be applied on many portable
 * devices, such as cellular phones, MIDs, netbook and notebook personal
 * computers.
 */

#ifndef __DRIVERS_INPUT_FT5X06_H
#define __DRIVERS_INPUT_FT5X06_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WARNING: Some definitions may apply only to the FT5336 */

/* FT5x06 maximum number of simultaneously detected touches. */

#define FT5X06_MAX_TOUCHES                (5)

/* FT5x06 raw touch data length. */

#define FT5X06_TOUCH_DATA_LEN             (0x20)

/* FT5x06 register addresses */

#define FT5X06_TOUCH_MODE_REG             (0x00) /* Mode register */
#define FT5X06_TOUCH_GESTID_REG           (0x01) /* Gesture ID register */
#define FT5X06_TOUCH_STAT_REG             (0x02) /* Touch data status */
                                                 /* See struct ft5x06_touch_point_s */
#define FT5X06_TH_GROUP_REG               (0x80) /* Threshold for touch detection */
#define FT5X06_TH_DIFF_REG                (0x85) /* Filter function coefficients */
#define FT5X06_CTRL_REG                   (0x86) /* Control register */
#define FT5X06_TIMEENTERMONITOR_REG       (0x87) /* Time switching Active to Monitor */
#define FT5X06_PERIODACTIVE_REG           (0x88) /* Report rate in Active mode */
#define FT5X06_PERIODMONITOR_REG          (0x89) /* Report rate in Monitor mode */
#define FT5X06_RADIAN_VALUE_REG           (0x91) /* Minimum allowing angle */
#define FT5X06_OFFSET_LEFT_RIGHT_REG      (0x92) /* Minimum offset */
#define FT5X06_OFFSET_UP_DOWN_REG         (0x93) /* Maximum offset */
#define FT5X06_DISTANCE_LEFT_RIGHT_REG    (0x94) /* Minimum distance */
#define FT5X06_DISTANCE_UP_DOWN_REG       (0x95) /* Minimum distance */
#define FT5X06_DISTANCE_ZOOM_REG          (0x96) /* Maximum distance */
#define FT5X06_LIB_VER_H_REG              (0xa1) /* MS LIB version */
#define FT5X06_LIB_VER_L_REG              (0xa2) /* LS LIB version */
#define FT5X06_CIPHER_REG                 (0xa3) /* Chip selecting */
#define FT5X06_GMODE_REG                  (0xa4) /* Interrupt mode */
#define FT5X06_PWR_MODE_REG               (0xa5) /* Power mode */
#define FT5X06_FIRMID_REG                 (0xa6) /* Firmware version */
#define FT5X06_CHIP_ID_REG                (0xa8) /* Chip ID */
#define FT5X06_RELEASE_CODE_ID_REG        (0xaf) /* Release code version */
#define FT5X06_STATE_REG                  (0xbc) /* Current operating mode */

#define FT5X06_TOUCH_DATA_STARTREG        (1)    /* Address where data begins */

/* Possible values of the DEV_MODE register */

#define FT5X06_DEV_MODE_WORKING           (0x00)
#define FT5X06_DEV_MODE_FACTORY           (0x04)

/* Possible values of the GEST_ID register */

#define FT5X06_GEST_ID_NO_GESTURE         (0x00)
#define FT5X06_GEST_ID_MOVE_UP            (0x10)
#define FT5X06_GEST_ID_MOVE_RIGHT         (0x14)
#define FT5X06_GEST_ID_MOVE_DOWN          (0x18)
#define FT5X06_GEST_ID_MOVE_LEFT          (0x1c)
#define FT5X06_GEST_ID_SINGLE_CLICK       (0x20)
#define FT5X06_GEST_ID_DOUBLE_CLICK       (0x22)
#define FT5X06_GEST_ID_ROTATE_CLOCKWISE   (0x28)
#define FT5X06_GEST_ID_ROTATE_C_CLOCKWISE (0x29)
#define FT5X06_GEST_ID_ZOOM_IN            (0x40)
#define FT5X06_GEST_ID_ZOOM_OUT           (0x49)

/* Values related to FT5X06_CTRL_REG */

#define FT5X06_CTRL_KEEP_ACTIVE_MODE      (0x00)
#define FT5X06_CTRL_KEEP_AUTO_SWITCH_MONITOR_MODE (0x01)

/* Possible values of FT5X06_GMODE_REG */

#define FT5X06_G_MODE_INTERRUPT_POLLING   (0x00)
#define FT5X06_G_MODE_INTERRUPT_TRIGGER   (0x01)

/* Possible values of FT5X06_CHIP_ID_REG */

#define FT5X06_ID_VALUE                   (0x51)

/* Operations on struct ft5x06_touch_point_s */

#define TOUCH_POINT_GET_EVENT(t)          ((t).xh >> 6)
#define TOUCH_POINT_GET_ID(t)             ((t).yh >> 4)
#define TOUCH_POINT_GET_X(t)              ((((t).xh & 0x0f) << 8) | (t).xl)
#define TOUCH_POINT_GET_Y(t)              ((((t).yh & 0x0f) << 8) | (t).yl)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum touch_event_e
{
  FT5X06_DOWN    = 0,  /* The state changed to touched */
  FT5X06_UP      = 1,  /* The state changed to not touched */
  FT5X06_CONTACT = 2,  /* There is a continuous touch being detected */
  FT5X06_INVALID = 3   /* No touch information available */
};

/* Describes on touchpoint returned by the FT5x06 */

struct ft5x06_touch_point_s
{
  uint8_t xh;
  uint8_t xl;
  uint8_t yh;
  uint8_t yl;
  uint8_t weight;
  uint8_t area;
};

/* Describes all touch data returned by the FT5x06 */

struct ft5x06_touch_data_s
{
  uint8_t gestid;      /* Gesture ID */
  uint8_t tdstatus;    /* Touch status */
  struct ft5x06_touch_point_s touch[FT5X06_MAX_TOUCHES];
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_INPUT_FT5X06_H */
