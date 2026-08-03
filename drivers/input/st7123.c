/****************************************************************************
 * drivers/input/st7123.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/bits.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>

#include <nuttx/input/st7123.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ST7123_TOUCH_FW_VERSION         0x00
#define ST7123_TOUCH_STATUS             0x01
#define ST7123_TOUCH_DEV_CTRL           0x02

/* XY Coordinate resolution, Maximum Number of Touches Register */

#define ST7123_TOUCH_MAX_X_COORD_H      0x05
#define ST7123_TOUCH_MAX_X_COORD_L      0x06
#define ST7123_TOUCH_MAX_Y_COORD_H      0x07
#define ST7123_TOUCH_MAX_Y_COORD_L      0x08
#define ST7123_TOUCH_MAX_TOUCHES        0x09
#define ST7123_TOUCH_SENSING_COUNTER_H  0x0a
#define ST7123_TOUCH_SENSING_COUNTER_L  0x0b
#define ST7123_TOUCH_FW_REV_3           0x0c
#define ST7123_TOUCH_FW_REV_2           0x0d
#define ST7123_TOUCH_FW_REV_1           0x0e
#define ST7123_TOUCH_FW_REV_0           0x0f
#define ST7123_TOUCH_ADV_TOUCH_INFO     0x10
#define ST7123_TOUCH_GESTURE_INFO       0x12
#define ST7123_TOUCH_KEYS               0x13
#define ST7123_TOUCH_MISC_INFO          0xf0
#define ST7123_TOUCH_MISC_CTRL          0xf1

/* Touch area registers */

#define ST7123_TOUCH_AREA_SIZE         7    /* There are 7 registers for each touch area */
#define ST7123_TOUCH_DATA_START        0x14

/* Maximum number of touch areas this driver is able to report.  The value
 * read back from ST7123_TOUCH_MAX_TOUCHES is clamped to this limit so that a
 * bogus register value can never overrun the frame or sample buffers.
 */

#define ST7123_MAX_TOUCH_AREAS         10

/* Registers 0x10 through 0x13 precede the per-area data at 0x14 and are read
 * as part of the same touch frame.
 */

#define ST7123_FRAME_HEADER_SIZE \
  (ST7123_TOUCH_DATA_START - ST7123_TOUCH_ADV_TOUCH_INFO)

/* Advanced Touch Info Masks */

#define ST7123_ADV_TOUCH_WITH_PROX     BIT(2)
#define ST7123_ADV_TOUCH_WITH_COORD    BIT(3)
#define ST7123_ADV_TOUCH_RST_CHIP      BIT(7)
#define ST7123_ADV_TOUCH_PROX_STATUS   0x70

/* Devices status obtained from STATUS register bits [3:0]
 * and error codes obtained from STATUS register bits [7:4].
 */

#define ST7123_STATUS_MASK 0x0f
#define ST7123_ERROR_MASK 0xf0
#define ST7123_ERROR_SHIFT 4

/* Miscellaneous information register */

#define ST7123_MISC_SUPPORT_COORD_CHKSUM     BIT(4)
#define ST7123_MISC_SUPPORT_PROXIMITY        BIT(5)
#define ST7123_MISC_SUPPORT_SMART_WAKEUP_EN  BIT(7)

/* Miscellaneous control register */

#define ST7123_MISC_CTRL_SMART_WAKEUP_EN_BIT BIT(7)

/* Device control register valid bits. All other bits must be set to 0. */

#define ST7123_DEVICE_CTRL_RESET_BIT         BIT(0)
#define ST7123_DEVICE_CTRL_POWER_DOWN_BIT    BIT(1)
#define ST7123_DEVICE_CTRL_PROXIMITY_EN_BIT  BIT(5)

/* Gesture codes */

#define ST7123_GESTURE_NONE             0x00
#define ST7123_GESTURE_DET_FAILED       0xff
#define ST7123_GESTURE_DOUBLE_TAP       0xb0
#define ST7123_GESTURE_SINGLE_TAP       0xb1
#define ST7123_GESTURE_LONG_PRESS       0xb2
#define ST7123_GESTURE_SWIPE_RIGHT      0xc0
#define ST7123_GESTURE_SWIPE_LEFT       0xc1
#define ST7123_GESTURE_SWIPE_DOWN       0xc2
#define ST7123_GESTURE_SWIPE_UP         0xc3
#define ST7123_GESTURE_ARROW_TOP        0xc4
#define ST7123_GESTURE_ARROW_RIGHT      0xc5
#define ST7123_GESTURE_ARROW_BOTTOM     0xc6
#define ST7123_GESTURE_ARROW_LEFT       0xc7
#define ST7123_GESTURE_TWO_FINGER_DOWN  0xc8

#define ST7123_I2C_ADDRLEN        7

/* Driver registration */

#define DEV_FORMAT     "/dev/input%d"
#define DEV_NAMELEN    16

/* Startup */

#define WAIT_TIMEOUT_STEP_US 100
#define WAIT_TIMEOUT_MAX_US (WAIT_TIMEOUT_STEP_US * 1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Raw area data with proper bit field for software parsing */

begin_packed_struct struct st7123_area_report_t
{
  uint8_t x_coord_h: 6;
  uint8_t reserved_s: 1;
  uint8_t valid: 1;
  uint8_t x_coord_l;
  uint8_t y_coord_h;
  uint8_t y_coord_l;
  uint8_t area_size;
  uint8_t intensity;
  uint8_t reserved_e;
} end_packed_struct;

/* Firmware revision from registers 0x0c through 0x0f */

begin_packed_struct struct st7123_fw_revision_reg_t
{
  uint8_t fw_rev_3;
  uint8_t fw_rev_2;
  uint8_t fw_rev_1;
  uint8_t fw_rev_0;
} end_packed_struct;

/* Advanced touch information from register 0x10 */

begin_packed_struct struct st7123_adv_touch_info_reg_t
{
  uint8_t reserved: 2;
  uint8_t with_prox: 1;
  uint8_t with_coord: 1;
  uint8_t prox_status: 3;
  uint8_t rst_chip: 1;
} end_packed_struct;

/* Miscellaneous information from register 0xf0 */

begin_packed_struct struct st7123_misc_info_reg_t
{
  uint8_t reserved_0: 3;
  uint8_t support_coord_chksum: 1;
  uint8_t support_proximity: 1;
  uint8_t reserved_1: 1;
  uint8_t support_smart_wakeup_en: 1;
} end_packed_struct;

/* Device capabilities from registers 0x05 through 0x09 */

begin_packed_struct struct st7123_device_caps_reg_t
{
  uint8_t max_x_coord_h: 5;
  uint8_t reserved_x: 2;
  uint8_t max_x_coord_l;
  uint8_t max_y_coord_h: 5;
  uint8_t reserved_y: 2;
  uint8_t max_y_coord_l;
  uint8_t max_touches;
} end_packed_struct;

/* A complete touch report, registers 0x10 upwards. The controller keeps the
 * interrupt line asserted until the whole frame has been consumed, so it
 * must be fetched in a single transaction.
 */

begin_packed_struct struct st7123_frame_t
{
  struct st7123_adv_touch_info_reg_t adv_touch_info;
  uint8_t reserved;
  uint8_t gesture;
  uint8_t keys;
  struct st7123_area_report_t areas[ST7123_MAX_TOUCH_AREAS];
} end_packed_struct;

/* Status codes */

typedef enum st7123_status_e
{
  ST7123_STATUS_NORMAL = 0,
  ST7123_STATUS_INIT = 1,
  ST7123_STATUS_ERROR = 2,
  ST7123_STATUS_SMART_WAKEUP = 3,
  ST7123_STATUS_IDLE = 4,
  ST7123_STATUS_POWER_DOWN = 5,
  ST7123_STATUS_PRE_SMART_WAKEUP = 6,
  ST7123_STATUS_PROXIMITY = 7,
  ST7123_STATUS_TRGT_PROXIMITY = 8,
} st7123_status_t;

/* Error codes */

typedef enum st7123_error_e
{
  ST7123_ERROR_NO_ERROR = 0,
  ST7123_ERROR_INVALID_ADDR = 1,
  ST7123_ERROR_INVALID_VALUE = 2,
  ST7123_ERROR_INVALID_PLATFORM = 3,
  ST7123_ERROR_DEV_NOT_FOUND = 4,
  ST7123_ERROR_DEV_STACK_OVERFLOW = 5,
  ST7123_ERROR_DEV_INVALID_FW_TABLE = 6,
} st7123_error_t;

/* Main device structure */

struct st7123_dev_t
{
  FAR struct touch_lowerhalf_s lower;
  FAR const struct st7123_config_s *config;
  struct i2c_master_s *i2c;
  struct i2c_config_s i2c_config;
  struct work_s work;
  mutex_t lock;

  struct st7123_frame_t frame;
  struct st7123_adv_touch_info_reg_t adv_touch_info;
  struct st7123_device_caps_reg_t device_caps;
  struct st7123_fw_revision_reg_t fw_revision;
  struct st7123_misc_info_reg_t misc_info;
  uint8_t fw_version;

  /* Contacts reported as down by the previous frame, one bit per area.  Used
   * to turn the per-frame valid bits into DOWN/MOVE/UP transitions.
   */

  uint16_t downmap;

  /* Last known position of each contact, so that the release event can be
   * reported at the position where the contact was lost.
   */

  int16_t lastx[ST7123_MAX_TOUCH_AREAS];
  int16_t lasty[ST7123_MAX_TOUCH_AREAS];

  /* Scratch buffer for touch_event() samples. */

  uint8_t sample_buf[SIZEOF_TOUCH_SAMPLE_S(ST7123_MAX_TOUCH_AREAS)];
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int st7123_open(struct touch_lowerhalf_s *lower);
static int st7123_close(struct touch_lowerhalf_s *lower);
static int st7123_control(struct touch_lowerhalf_s *lower, int cmd,
                          unsigned long arg);
static int st7123_write(struct touch_lowerhalf_s *lower,
                        FAR const char *buffer, size_t buflen);
static int st7123_read_reg(struct st7123_dev_t *dev, uint8_t reg,
                           uint8_t *value);
static int st7123_write_reg(struct st7123_dev_t *dev, uint8_t reg,
                            uint8_t value);
static int st7123_read_sequential(struct st7123_dev_t *dev, uint8_t reg,
                                  uint8_t *value, size_t count);
static int st7123_read_status(struct st7123_dev_t *dev,
                              FAR st7123_status_t *status);
static int st7123_read_error(struct st7123_dev_t *dev,
                             FAR st7123_error_t *error);
static int st7123_probe_device(struct st7123_dev_t *dev);
static void st7123_data_worker(FAR void *arg);
static int st7123_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Static Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7123_control
 *
 * Description:
 *   Handle device-specific ioctl commands for the ST7123 lower half.
 *   No commands are currently supported.
 *
 * Input Parameters:
 *   lower - Touchscreen lower-half state
 *   cmd   - ioctl command
 *   arg   - ioctl argument
 *
 * Returned Value:
 *   -ENOTTY is always returned.
 *
 ****************************************************************************/

static int st7123_control(struct touch_lowerhalf_s *lower, int cmd,
                          unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: st7123_write
 *
 * Description:
 *   Write to the ST7123 lower half.  Writes are not supported.
 *
 * Input Parameters:
 *   lower  - Touchscreen lower-half state
 *   buffer - Data to write
 *   buflen - Number of bytes to write
 *
 * Returned Value:
 *   -ENOTTY is always returned.
 *
 ****************************************************************************/

static int st7123_write(struct touch_lowerhalf_s *lower,
                        FAR const char *buffer, size_t buflen)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: st7123_open
 *
 * Description:
 *   Bring the ST7123 out of power-down, optionally disable smart wakeup,
 *   and wait until the device reports the NORMAL status.  Clears the
 *   contact bitmap so the first frame after open does not emit stale
 *   release events.
 *
 * Input Parameters:
 *   lower - Touchscreen lower-half state
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_open(struct touch_lowerhalf_s *lower)
{
  DEBUGASSERT(lower != NULL);

  struct st7123_dev_t *dev = (struct st7123_dev_t *) lower;
  st7123_status_t status = ST7123_STATUS_INIT;
  st7123_error_t error;
  uint32_t timeout_cnt = 0;
  uint8_t regval;
  int ret;

  /* Power up the and reset the device */

  ret = st7123_write_reg(dev, ST7123_TOUCH_DEV_CTRL, 0x0);
  if (ret != OK)
    {
      return ret;
    }

  /* Disable smart wakeup if available, for now */

  if (dev->misc_info.support_smart_wakeup_en)
    {
      ret = st7123_read_reg(dev, ST7123_TOUCH_MISC_CTRL, &regval);
      if (ret == OK)
        {
          regval &= ~ST7123_MISC_CTRL_SMART_WAKEUP_EN_BIT;
          ret = st7123_write_reg(dev, ST7123_TOUCH_MISC_CTRL, regval);
          if (ret != OK)
            {
              ierr("failed to disable smart wakeup: %d", ret);
              return ret;
            }
        }
    }

  while (timeout_cnt < WAIT_TIMEOUT_MAX_US)
    {
      ret = st7123_read_status(dev, &status);
      if (ret == OK && status == ST7123_STATUS_NORMAL)
        {
          break;
        }

      nxsched_usleep(WAIT_TIMEOUT_STEP_US);
      timeout_cnt += WAIT_TIMEOUT_STEP_US;
    }

  if (status != ST7123_STATUS_NORMAL)
    {
      if (st7123_read_error(dev, &error) == OK)
        {
          ierr("%s open timeout. Status: %d, error: %d", __func__,
               status, error);
        }

      return -EIO;
    }

  /* No contact is down on a freshly opened device */

  dev->downmap = 0;
  memset(&dev->lastx, 0, sizeof(dev->lastx));
  memset(&dev->lasty, 0, sizeof(dev->lasty));

  iinfo("opened");

  return OK;
}

/****************************************************************************
 * Name: st7123_close
 *
 * Description:
 *   Power down the ST7123 and verify that the STATUS register reports
 *   POWER_DOWN.
 *
 * Input Parameters:
 *   lower - Touchscreen lower-half state
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_close(struct touch_lowerhalf_s *lower)
{
  DEBUGASSERT(lower != NULL);

  struct st7123_dev_t *dev = (struct st7123_dev_t *) lower;
  st7123_status_t status;
  int ret;

  /* Power down the device */

  ret = st7123_write_reg(dev, ST7123_TOUCH_DEV_CTRL,
                         ST7123_DEVICE_CTRL_POWER_DOWN_BIT);
  if (ret != OK)
    {
      return ret;
    }

  nxsched_usleep(WAIT_TIMEOUT_STEP_US);

  ret = st7123_read_status(dev, &status);
  if (ret != OK)
    {
      return ret;
    }

  if (status != ST7123_STATUS_POWER_DOWN)
    {
      ierr("%s failed to power down. Status: %d", __func__, status);
      return -EIO;
    }

  iinfo("%s powered down", __func__);
  return OK;
}

/****************************************************************************
 * Name: st7123_read_reg
 *
 * Description:
 *   Read a single ST7123 register over I2C.  The register address is
 *   written as a two-byte Start Reg H / Start Reg L pair, then one byte
 *   is read back.
 *
 * Input Parameters:
 *   dev   - ST7123 device state
 *   reg   - Register address to read
 *   value - Location to receive the register value
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_read_reg(struct st7123_dev_t *dev, uint8_t reg,
                           uint8_t *value)
{
  int ret;
  uint8_t write_buffer[2];

  write_buffer[0] = 0;
  write_buffer[1] = reg;

  nxmutex_lock(&dev->lock);
  ret = i2c_writeread(dev->i2c, &dev->i2c_config, write_buffer, 2, value, 1);
  nxmutex_unlock(&dev->lock);
  if (ret != OK)
    {
      ierr("Failed to read register %02x: %d", reg, ret);
    }
  return ret;
}

/****************************************************************************
 * Name: st7123_read_sequential
 *
 * Description:
 *   Read a contiguous run of ST7123 registers over I2C, starting at the
 *   given register address.  Used to fetch multi-byte structures such as
 *   the firmware revision, device capabilities, and the full touch frame.
 *
 * Input Parameters:
 *   dev   - ST7123 device state
 *   reg   - First register address to read
 *   value - Buffer to receive the register values
 *   count - Number of bytes to read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_read_sequential(struct st7123_dev_t *dev, uint8_t reg,
                                  uint8_t *value, size_t count)
{
  int ret;
  uint8_t write_buffer[2];

  write_buffer[0] = 0;
  write_buffer[1] = reg;

  nxmutex_lock(&dev->lock);
  ret = i2c_writeread(dev->i2c, &dev->i2c_config, write_buffer, 2,
                      value, count);
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: st7123_write_reg
 *
 * Description:
 *   Write a single ST7123 register over I2C.  The transfer carries the
 *   two-byte Start Reg H / Start Reg L address followed by the value.
 *
 * Input Parameters:
 *   dev   - ST7123 device state
 *   reg   - Register address to write
 *   value - Value to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_write_reg(struct st7123_dev_t *dev, uint8_t reg,
                            uint8_t value)
{
  int ret;
  uint8_t buffer[3];

  buffer[0] = 0;
  buffer[1] = reg;
  buffer[2] = value;

  nxmutex_lock(&dev->lock);
  ret = i2c_write(dev->i2c, &dev->i2c_config, buffer, 3);
  nxmutex_unlock(&dev->lock);
  if (ret != OK)
    {
      ierr("Failed to write register %02x", reg);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: st7123_data_worker
 *
 * Description:
 *   Work-queue handler that reads one touch frame from the ST7123 and
 *   converts the per-area valid bits into DOWN / MOVE / UP events for the
 *   touchscreen upper half.  Gesture codes from the controller are mapped
 *   onto the NuttX touch gesture values where possible.  Frames that carry
 *   no contact change are discarded.
 *
 * Input Parameters:
 *   arg - Pointer to the ST7123 device state
 *
 ****************************************************************************/

static void st7123_data_worker(FAR void *arg)
{
  FAR struct st7123_dev_t *dev = (FAR struct st7123_dev_t *)arg;
  struct touch_lowerhalf_s *lower = &dev->lower;
  FAR struct touch_sample_s *sample =
    (FAR struct touch_sample_s *)dev->sample_buf;
  FAR struct touch_point_s *point;
  FAR struct st7123_area_report_t *area;
  uint64_t timestamp;
  uint16_t downmap = 0;
  size_t framelen;
  int npoints = 0;
  int ret;
  int i;

  framelen = ST7123_FRAME_HEADER_SIZE +
             lower->maxpoint * ST7123_TOUCH_AREA_SIZE;

  ret = st7123_read_sequential(dev, ST7123_TOUCH_ADV_TOUCH_INFO,
                               (uint8_t *) &dev->frame, framelen);
  if (ret != OK)
    {
      ierr("failed to read touch frame: %d", ret);
      return;
    }

  timestamp = touch_get_time();
  memset(sample, 0, sizeof(dev->sample_buf));

  for (i = 0; i < lower->maxpoint; i++)
    {
      bool was_down = (dev->downmap & (1 << i)) != 0;

      area = &dev->frame.areas[i];

      if (area->valid)
        {
          dev->lastx[i] = (area->x_coord_h << 8) | area->x_coord_l;
          dev->lasty[i] = (area->y_coord_h << 8) | area->y_coord_l;
          downmap |= 1 << i;
        }
      else if (!was_down)
        {
          /* The area is idle and was idle in the previous frame too, so
           * there is nothing to report for it.
           */

          continue;
        }

      point            = &sample->point[npoints++];
      point->id        = i;
      point->x         = dev->lastx[i];
      point->y         = dev->lasty[i];
      point->timestamp = timestamp;

      if (area->valid)
        {
          point->pressure = area->intensity;
          switch (dev->frame.gesture)
            {
              case ST7123_GESTURE_DOUBLE_TAP:
                point->gesture = TOUCH_DOUBLE_CLICK;
                break;
              case ST7123_GESTURE_SWIPE_UP:
                point->gesture = TOUCH_SLIDE_UP;
                break;
              case ST7123_GESTURE_SWIPE_DOWN:
                point->gesture = TOUCH_SLIDE_DOWN;
                break;
              case ST7123_GESTURE_SWIPE_LEFT:
                point->gesture = TOUCH_SLIDE_LEFT;
                break;
              case ST7123_GESTURE_SWIPE_RIGHT:
                point->gesture = TOUCH_SLIDE_RIGHT;
                break;
              default:
                point->gesture = 0xff;
                break;
            }
          point->flags    = (was_down ? TOUCH_MOVE : TOUCH_DOWN) |
                            TOUCH_ID_VALID | TOUCH_POS_VALID |
                            TOUCH_PRESSURE_VALID;
        }
      else
        {
          /* The contact was lost.  Report the release at the last known
           * position so that consumers can act on it.
           */

          point->flags = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
    }

  dev->downmap = downmap;

  /* An interrupt without any contact change carries no information */

  if (npoints == 0)
    {
      return;
    }

  sample->npoints = npoints;
  touch_event(lower->priv, sample);
}

/****************************************************************************
 * Name: st7123_read_status
 *
 * Description:
 *   Read the STATUS register and return the device status from bits [3:0].
 *
 * Input Parameters:
 *   dev    - ST7123 device state
 *   status - Location to receive the decoded status
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_read_status(struct st7123_dev_t *dev,
                              FAR st7123_status_t *status)
{
  int ret;
  uint8_t regval;

  ret = st7123_read_reg(dev, ST7123_TOUCH_STATUS, &regval);
  if (ret != OK)
    {
      ierr("failed to read status: %d", ret);
      return ret;
    }

  *status = (st7123_status_t) (regval & ST7123_STATUS_MASK);
  return OK;
}

/****************************************************************************
 * Name: st7123_read_error
 *
 * Description:
 *   Read the STATUS register and return the error code from bits [7:4].
 *
 * Input Parameters:
 *   dev   - ST7123 device state
 *   error - Location to receive the decoded error code
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_read_error(struct st7123_dev_t *dev,
                             FAR st7123_error_t *error)
{
  int ret;
  uint8_t regval;

  ret = st7123_read_reg(dev, ST7123_TOUCH_STATUS, &regval);
  if (ret != OK)
    {
      ierr("failed to read error: %d", ret);
      return ret;
    }

  *error = (st7123_error_t) ((regval & ST7123_ERROR_MASK) >>
                             ST7123_ERROR_SHIFT);
  return OK;
}

/****************************************************************************
 * Name: st7123_probe_device
 *
 * Description:
 *   Read the firmware version, firmware revision, advanced touch info,
 *   miscellaneous capabilities, and coordinate / touch-count limits from
 *   the ST7123 and cache them in the device structure.
 *
 * Input Parameters:
 *   dev - ST7123 device state
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int st7123_probe_device(struct st7123_dev_t *dev)
{
  int ret;

  ret = st7123_read_reg(dev, ST7123_TOUCH_FW_VERSION, &dev->fw_version);
  ret |= st7123_read_reg(dev, ST7123_TOUCH_ADV_TOUCH_INFO,
                         (uint8_t *) &dev->adv_touch_info);
  ret |= st7123_read_reg(dev, ST7123_TOUCH_MISC_INFO,
                         (uint8_t *) &dev->misc_info);
  ret |= st7123_read_sequential(dev, ST7123_TOUCH_FW_REV_3,
                                (uint8_t *) &dev->fw_revision,
                                sizeof(dev->fw_revision));
  ret |= st7123_read_sequential(dev, ST7123_TOUCH_MAX_X_COORD_H,
                                (uint8_t *) &dev->device_caps,
                                sizeof(dev->device_caps));

  if (ret != OK)
    {
      ierr("failed to read device capabilities: %d", ret);
      return -EIO;
  }

  return OK;
}

/****************************************************************************
 * Name: st7123_interrupt
 *
 * Description:
 *   GPIO interrupt handler attached by the board through config->attach().
 *   Schedules st7123_data_worker() on the high-priority work queue.
 *
 * Input Parameters:
 *   irq     - Interrupt number (unused)
 *   context - Interrupt context (unused)
 *   arg     - Pointer to the ST7123 device state
 *
 * Returned Value:
 *   OK is always returned.
 *
 ****************************************************************************/

static int st7123_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct st7123_dev_t *priv = (FAR struct st7123_dev_t *)arg;
  int ret;

  DEBUGASSERT(priv != NULL);
  UNUSED(irq);
  UNUSED(context);

  ret = work_queue(HPWORK, &priv->work, st7123_data_worker, priv, 0);
  if (ret < 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7123_register
 *
 * Description:
 *   Probe the ST7123 over I2C, configure the touchscreen lower half from
 *   the reported resolution and touch-area count, register it as
 *   /dev/inputN, and attach the board interrupt via config->attach().
 *
 * Input Parameters:
 *   i2c    - I2C master used to talk to the ST7123
 *   minor  - Device minor number used to form /dev/inputN
 *   config - Persistent board configuration; config->attach must be valid
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int st7123_register(FAR struct i2c_master_s *i2c, uint8_t minor,
                    FAR const struct st7123_config_s *config)
{
  FAR struct st7123_dev_t *priv;
  char devname[DEV_NAMELEN];
  st7123_status_t status;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL && config->attach != NULL);

  if (config == NULL || config->attach == NULL)
    {
      ierr("ERROR: board config->attach is required\n");
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct st7123_dev_t));
  if (priv == NULL)
    {
      ierr("ERROR: kmm_zalloc(%zu) failed\n", sizeof(struct st7123_dev_t));
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.control = st7123_control;
  priv->lower.write   = st7123_write;
  priv->lower.open    = st7123_open;
  priv->lower.close   = st7123_close;

  priv->i2c = i2c;
  priv->i2c_config.frequency = CONFIG_INPUT_ST7123_I2C_FREQUENCY;
  priv->i2c_config.address = CONFIG_INPUT_ST7123_I2C_ADDRESS;
  priv->i2c_config.addrlen = ST7123_I2C_ADDRLEN;

  nxmutex_init(&priv->lock);

  ret = st7123_read_status(priv, &status);
  if (ret != OK)
    {
      goto errout_with_lock;
    }

  if (status == ST7123_STATUS_POWER_DOWN)
    {
      st7123_write_reg(priv, ST7123_TOUCH_DEV_CTRL, 0x0);
    }

  ret = st7123_probe_device(priv);
  if (ret != OK)
    {
      ierr("failed to probe st7123");
      ret = -EINVAL;
      goto errout_with_lock;
    }

  if (priv->device_caps.max_touches == 0)
    {
      ierr("device reports no touch areas");
      ret = -ENODEV;
      goto errout_with_lock;
    }

  priv->lower.maxpoint = priv->device_caps.max_touches;
  priv->lower.xres = ((priv->device_caps.max_x_coord_h << 8) +
                      priv->device_caps.max_x_coord_l);
  priv->lower.yres = ((priv->device_caps.max_y_coord_h << 8) +
                       priv->device_caps.max_y_coord_l);
  priv->lower.flags = 0;

  iinfo("probed: fw %u rev %u.%u.%u.%u", priv->fw_version,
        priv->fw_revision.fw_rev_3, priv->fw_revision.fw_rev_2,
        priv->fw_revision.fw_rev_1, priv->fw_revision.fw_rev_0);
  iinfo("resolution: %d x %d, max touches: %u, smart wakeup: %u",
        priv->lower.xres, priv->lower.yres, priv->lower.maxpoint,
        priv->misc_info.support_smart_wakeup_en);

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  ret = touch_register(&priv->lower, devname, priv->lower.maxpoint);
  if (ret < 0)
    {
      ierr("failed to register %s: %d", devname, ret);
      goto errout_with_lock;
    }

  /* Attach after the upper half is ready so the ISR can safely queue work. */

  ret = config->attach(config, st7123_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt: %d\n", ret);
      goto errout_with_register;
    }

  return OK;

errout_with_register:
  touch_unregister(&priv->lower, devname);

errout_with_lock:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
