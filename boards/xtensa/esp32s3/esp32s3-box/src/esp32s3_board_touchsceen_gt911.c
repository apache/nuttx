/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-box/src/esp32s3_board_touchsceen_gt911.c
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

/* GT911 maximum report frame size */

#define GT911_BUFFER_SIZE     41
#define GT911_TOUCHPOINTS     5

/* GT911 board configuration */

#define GT911_ADDR            TOUCHSCEEN_ADDR
#define GT911_CLOCK           TOUCHSCEEN_CLOCK

#define GT911_PATH            CONFIG_ESP32S3_BOARD_TOUCHSCREEN_PATH
#define GT911_WORK_DELAY      CONFIG_ESP32S3_BOARD_TOUCHSCREEN_SAMPLE_DELAYS
#define GT911_SAMPLE_CACHES   CONFIG_ESP32S3_BOARD_TOUCHSCREEN_SAMPLE_CACHES

/* GT911 registers address */

#define GT911_READ_XY_REG     0x814e
#define GT911_READ_DATA_REG   0x814f
#define GT911_CONFIG_REG      0x8047
#define GT911_PRODUCT_ID_REG  0x8140

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one GT911 driver instance */

struct gt911_dev_s
{
  struct touch_lowerhalf_s  touch_lower;    /* Touchsrceen lowerhalf */

  bool                has_report;           /* Mark if report event */

  struct i2c_master_s *i2c;                 /* I2C master port */
  struct work_s       work;                 /* Read sample data work */
  spinlock_t          lock;                 /* Device specific lock. */

  uint8_t buffer[GT911_BUFFER_SIZE];        /* Read buffer */
};

/* This structure describes the frame of touchpoint */

struct gt911_touchpoint_s
{
  uint8_t id;                               /* Not used */
  uint16_t x;                               /* Touch X-axis */
  uint16_t y;                               /* Touch Y-axis */
  uint16_t pressure;                        /* Touch pressure */
  uint8_t  reserved;                        /* Not used */
}
__attribute__((packed));

/* This structure describes the frame of touchpoint */

struct gt911_data_s
{
  uint8_t touchpoints     : 4;              /* Touch point number */
  uint8_t has_key         : 1;              /* 1: key is inpressed */
  uint8_t proximity_valid : 1;              /* Not used */
  uint8_t large_detected  : 1;              /* 1: large-area touch */
  uint8_t buffer_status   : 1;              /* 1: input data is valid */

  struct gt911_touchpoint_s touchpoint[0];
}
__attribute__((packed));

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct gt911_dev_s g_gt911_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt911_write_reg
 *
 * Description:
 *   Read GT911 continuous registers value.
 *
 * Input Parameters:
 *   dev    - GT911 object pointer
 *   reg    - Register start address
 *   buf    - Register value buffer
 *   buflen - Register value buffer length
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gt911_read_reg(struct gt911_dev_s *dev,
                          uint16_t reg,
                          uint8_t *buf,
                          int buflen)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = GT911_CLOCK,
      .addr      = GT911_ADDR,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Receive the I2C Register Values */

      .frequency = GT911_CLOCK,
      .addr      = GT911_ADDR,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, buflen=%d\n", reg, buflen);
  DEBUGASSERT(dev && dev->i2c && buf);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Read failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt911_read_reg", buf, buflen);
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return 0;
}

/****************************************************************************
 * Name: gt911_write_reg
 *
 * Description:
 *   Write GT911 register value.
 *
 * Input Parameters:
 *   dev - GT911 object pointer
 *   reg - Register address
 *   val - Register value
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gt911_write_reg(struct gt911_dev_s *dev,
                           uint16_t reg,
                           uint8_t val)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[3] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff, /* Second Byte: LSB */
    val,
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[1] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = GT911_CLOCK,
      .addr      = GT911_ADDR,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, val=%d\n", reg, val);
  DEBUGASSERT(dev && dev->i2c);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Write failed: %d\n", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: gt911_touch_event
 *
 * Description:
 *   Process touch event. Read touchpoint data and send to touch event.
 *
 * Input Parameters:
 *   dev - GT911 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gt911_touch_event(struct gt911_dev_s *dev)
{
  struct gt911_data_s *data = (struct gt911_data_s *)dev->buffer;
  struct gt911_touchpoint_s *tp = data->touchpoint;
  struct touch_sample_s sample;
  struct touch_point_s *point = sample.point;

  memset(&sample, 0, sizeof(sample));
  sample.npoints = 1;

  point->x         = tp->x;
  point->y         = tp->y;
  point->pressure  = tp->pressure;
  point->flags     = TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;

  if (data->buffer_status)
    {
      point->flags |= TOUCH_DOWN;
      dev->has_report = true;
    }
  else
    {
      point->flags |= TOUCH_UP;
      dev->has_report = false;
    }

  touch_event(dev->touch_lower.priv, &sample);
}

/****************************************************************************
 * Name: gt911_event
 *
 * Description:
 *   Process GT911 event.
 *
 * Input Parameters:
 *   dev - GT911 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gt911_event(struct gt911_dev_s *dev)
{
  struct gt911_data_s *data = (struct gt911_data_s *)dev->buffer;

  if (!data->has_key)
    {
      gt911_touch_event(dev);
    }
  else
    {
      ierr("ERROR: event is invalid\n");
    }
}

/****************************************************************************
 * Name: gt911_worker
 *
 * Description:
 *   Process GT911 work, read GT911 report frame and process it.
 *
 * Input Parameters:
 *   arg - GT911 object pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gt911_worker(FAR void *arg)
{
  int ret;
  struct gt911_dev_s *dev = (struct gt911_dev_s *)arg;
  struct gt911_data_s *data = (struct gt911_data_s *)dev->buffer;
  clock_t delay = GT911_WORK_DELAY;
  bool touched = false;

  ret = gt911_read_reg(dev, GT911_READ_XY_REG, dev->buffer, 1);
  if (ret != 0)
    {
      ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
      goto exit;
    }

  if (data->buffer_status &&
      (data->touchpoints > 0) &&
      (data->touchpoints < GT911_TOUCHPOINTS))
    {
      ret = gt911_read_reg(dev, GT911_READ_DATA_REG,
                           &dev->buffer[1], data->touchpoints * 8);
      if (ret != 0)
        {
          ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
          goto exit;
        }

      touched = true;
    }
  else if (dev->has_report)
    {
      touched = true;
    }

  ret = gt911_write_reg(dev, GT911_READ_XY_REG, 0);
  if (ret != 0)
    {
      ierr("ERROR: I2C_TRANSFER() failed: %d\n", ret);
      goto exit;
    }

  if (touched)
    {
      gt911_event(dev);
      delay = 1;
    }

exit:
  ret = work_queue(LPWORK, &dev->work, gt911_worker, dev, delay);
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
  struct gt911_dev_s *dev = &g_gt911_dev;

  dev->i2c = esp32s3_i2cbus_initialize(TOUCHSCEEN_I2C);
  if (!dev->i2c)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C port %d\n",
             TOUCHSCEEN_I2C);
      return -ENODEV;
    }

  ret = touch_register(&dev->touch_lower, GT911_PATH,
                       GT911_SAMPLE_CACHES);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: touch_register() failed: %d\n", ret);
      return ret;
    }

  ret = work_queue(LPWORK, &dev->work, gt911_worker,
                   dev, GT911_WORK_DELAY);
  if (ret != 0)
    {
      syslog(LOG_ERR, "ERROR: work_queue() failed: %d\n", ret);
      return ret;
    }

  return 0;
}
