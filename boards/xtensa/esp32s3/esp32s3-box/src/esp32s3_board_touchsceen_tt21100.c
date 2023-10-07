/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-box/src/esp32s3_board_touchsceen_tt21100.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <syslog.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/input/touchscreen.h>

#include "esp32s3_i2c.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include "esp32s3-box.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TT21100 maximum report frame size */

#define TT21100_BUFFER_SIZE     256

/* TT21100 touch report frame ID */

#define TT21100_ID_TOUCH        0x1

#define TT21100_PATH            CONFIG_ESP32S3_BOARD_TOUCHSCREEN_PATH
#define TT21100_WIDTH           CONFIG_ESP32S3_BOARD_TOUCHSCREEN_WIDTH
#define TT21100_HEIGHT          CONFIG_ESP32S3_BOARD_TOUCHSCREEN_HEIGHT
#define TT21100_WORK_DELAY      CONFIG_ESP32S3_BOARD_TOUCHSCREEN_SAMPLE_DELAYS
#define TT21100_SAMPLE_CACHES   CONFIG_ESP32S3_BOARD_TOUCHSCREEN_SAMPLE_CACHES

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one TT21100 driver instance */

struct tt21100_dev_s
{
  struct touch_lowerhalf_s  touch_lower;    /* Touchsrceen lowerhalf */

  struct i2c_master_s *i2c;                 /* I2C master port */
  struct work_s       work;                 /* Read sample data work */
  spinlock_t          lock;                 /* Device specific lock. */

  uint8_t buffer[TT21100_BUFFER_SIZE];      /* Read buffer */
};

/* This structure describes the frame of touchpoint */

struct tt21100_touchpoint_s
{
  uint8_t  type       : 3;                  /* Touch object type */
  uint8_t  reserved   : 5;                  /* Reserved */

  uint8_t  touch_id   : 5;                  /* Touch arbitrary ID tag */
  uint8_t  event_id   : 2;                  /* Touch action event ID */
  uint8_t  tip        : 1;                  /* Touch liftoff status */

  uint16_t x;                               /* Touch X-axis */
  uint16_t y;                               /* Touch Y-axis */

  uint8_t  pressure;                        /* Touch pressure */

  uint16_t major_axis_length;               /* Touch length of major axis */
  uint8_t  orientation;                     /* The angle between vertical axis and major axis */
}
__attribute__((packed));

/* This structure describes the frame of touch action */

struct tt21100_touch_s
{
  uint8_t  data_num       : 5;              /* Touchpoint data frame number */
  uint8_t  large_object   : 1;              /* Large object detected */
  uint8_t  reserved_0     : 2;              /* Reserved */

  uint8_t  noise_efect    : 3;              /* Charger noise effect level */
  uint8_t  reserved_1     : 3;              /* Reserved */
  uint8_t  report_counter : 2;              /* Report issue counter */

  struct tt21100_touchpoint_s touchpoint[0];  /* Touchpoint data */
}
__attribute__((packed));

/* This structure describes the frame of TT21100 report */

struct tt21100_data_s
{
  uint16_t  length;                         /* Report frame length */
  uint8_t   id;                             /* Report ID */
  uint16_t  time_stamp;                     /* Report Timestamp */
  struct tt21100_touch_s touch;             /* Report Tocuh data */
}
__attribute__((packed));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tt21100_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct tt21100_dev_s g_tt21100_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tt2110_touch_event
 *
 * Description:
 *   Process touch event. Read touchpoint data and send to touch event.
 *
 * Input Parameters:
 *   dev - TT2110 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tt2110_touch_event(struct tt21100_dev_s *dev)
{
  struct tt21100_data_s *data = (struct tt21100_data_s *)dev->buffer;
  struct tt21100_touch_s *touch = &data->touch;
  struct tt21100_touchpoint_s *touchpoint = touch->touchpoint;
  struct touch_sample_s sample;
  struct touch_point_s *point = sample.point;

  memset(&sample, 0, sizeof(sample));
  sample.npoints = 1;

  if (touch->data_num > 0)
    {
      point->timestamp = data->time_stamp;
      point->pressure  = touchpoint[0].pressure;
      point->flags     = TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;

#ifdef CONFIG_ESP32S3_BOARD_TOUCHSCREEN_X_MIRROR
      point->x         = TT21100_WIDTH - (int16_t)touchpoint[0].x;
#else
      point->x         = (int16_t)touchpoint[0].x;
#endif

#ifdef CONFIG_ESP32S3_BOARD_TOUCHSCREEN_Y_MIRROR
      point->y         = TT21100_HEIGHT - (int16_t)touchpoint[0].y;
#else
      point->y         = (int16_t)touchpoint[0].y;
#endif

      switch (touchpoint[0].event_id)
        {
          case 1:
            point->flags |= TOUCH_DOWN;
            break;
          case 2:
            point->flags |= TOUCH_MOVE;
            break;
          case 3:
            point->flags |= TOUCH_UP;
            break;
          default:
            ierr("ERROR: invalid event id: %d\n", touchpoint[0].event_id);
            break;
        }

      touch_event(dev->touch_lower.priv, &sample);
    }
}

/****************************************************************************
 * Name: tt21100_event
 *
 * Description:
 *   Process TT21100 event.
 *
 * Input Parameters:
 *   dev - TT2110 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tt21100_event(struct tt21100_dev_s *dev)
{
  struct tt21100_data_s *data = (struct tt21100_data_s *)dev->buffer;

  switch (data->id)
    {
      case TT21100_ID_TOUCH:
        tt2110_touch_event(dev);
        break;
      default:
        ierr("ERROR: id=%d is invalid\n", data->id);
        break;
    }
}

/****************************************************************************
 * Name: tt21100_worker
 *
 * Description:
 *   Process TT21100 work, read TT21100 report frame and process it.
 *
 * Input Parameters:
 *   arg - TT2110 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tt21100_worker(FAR void *arg)
{
  int ret;
  struct i2c_msg_s msg;
  uint16_t length;
  struct tt21100_dev_s *dev = (struct tt21100_dev_s *)arg;
  clock_t delay = TT21100_WORK_DELAY;

  msg.frequency = TOUCHSCEEN_CLOCK;
  msg.addr      = TOUCHSCEEN_ADDR;
  msg.flags     = I2C_M_READ;
  msg.buffer    = (uint8_t *)&length;
  msg.length    = sizeof(length);
  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  if (ret != 0)
    {
      ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
      goto exit;
    }

  if (length > TT21100_BUFFER_SIZE)
    {
      ierr("ERROR: tt21100 data length > %d\n", TT21100_BUFFER_SIZE);
      goto exit;
    }
  else if (length <= 2)
    {
      /* No valid data */

      goto exit;
    }

  msg.frequency = TOUCHSCEEN_CLOCK;
  msg.addr      = TOUCHSCEEN_ADDR;
  msg.flags     = I2C_M_READ;
  msg.buffer    = dev->buffer;
  msg.length    = length;
  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  if (ret != 0)
    {
      ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
      goto exit;
    }

  tt21100_event(dev);

  delay = 1;

exit:
  ret = work_queue(LPWORK, &dev->work, tt21100_worker, dev, delay);
  if (ret != 0)
    {
      ierr("ERROR: work_queue() failed: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_touchscreen_initialize
 *
 * Description:
 *   Initialize touchpad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_touchscreen_initialize(void)
{
  int ret;
  struct tt21100_dev_s *dev = &g_tt21100_dev;

  dev->i2c = esp32s3_i2cbus_initialize(TOUCHSCEEN_I2C);
  if (!dev->i2c)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C port %d\n",
             TOUCHSCEEN_I2C);
      return -ENODEV;
    }

  ret = touch_register(&dev->touch_lower, TT21100_PATH,
                       TT21100_SAMPLE_CACHES);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: touch_register() failed: %d\n", ret);
      return ret;
    }

  ret = work_queue(LPWORK, &dev->work, tt21100_worker,
                   dev, TT21100_WORK_DELAY);
  if (ret != 0)
    {
      syslog(LOG_ERR, "ERROR: work_queue() failed: %d\n", ret);
      return ret;
    }

  return 0;
}
