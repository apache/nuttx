/****************************************************************************
 * drivers/input/cst92xx.c
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
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#include <nuttx/input/cst92xx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 16-bit register / command addresses.  Commands are issued by writing the
 * address alone, registers are read by writing the address and then
 * reading back with a repeated start.
 */

#define CST92XX_REG_POINTS        0xd000 /* Touch report, write ACK here */
#define CST92XX_CMD_INFO_MODE     0xd101 /* Enter debug/info mode */
#define CST92XX_CMD_NORMAL_MODE   0xd109 /* Return to normal reporting */
#define CST92XX_REG_RESOLUTION    0xd1f8 /* X/Y resolution, LE16 each */
#define CST92XX_REG_CHECKCODE     0xd1fc /* Firmware check code, LE32 */
#define CST92XX_REG_CHIPTYPE      0xd204 /* Project ID, chip type, LE16 */
#define CST92XX_REG_FWVERSION     0xd208 /* Firmware version, checksum */

#define CST92XX_ACK               0xab   /* Report marker and ACK value */
#define CST92XX_CHECKCODE_MASK    0xffff0000
#define CST92XX_CHECKCODE         0xcaca0000
#define CST92XX_FW_ERASED         0xa5a5a5a5

#define CST92XX_CHIP_CST9217      0x9217
#define CST92XX_CHIP_CST9220      0x9220

/* Touch report layout.  The first contact occupies bytes 0-4, followed by
 * a status byte with the contact count (byte 5) and the 0xab marker
 * (byte 6).  Further contacts follow at 7 + 5 * (n - 1).
 */

#define CST92XX_POINT_SIZE        5
#define CST92XX_COUNT_OFFSET      5
#define CST92XX_MARKER_OFFSET     6
#define CST92XX_COUNT_MASK        0x7f
#define CST92XX_COVER_MASK        0x80   /* In byte 4: palm/cover detect */
#define CST92XX_EVENT_MASK        0x0f
#define CST92XX_EVENT_PRESSED     0x06

#define CST92XX_POINT_OFFSET(n) \
  ((n) == 0 ? 0 : CST92XX_POINT_SIZE * (n) + 2)
#define CST92XX_FRAME_SIZE(n)     (CST92XX_POINT_SIZE * (n) + 2)

#define CST92XX_MAX_POINTS        CONFIG_INPUT_CST92XX_MAX_POINTS

/* Timing */

#define CST92XX_RESET_HOLD_US     10000
#define CST92XX_RESET_BOOT_US     50000
#define CST92XX_MODE_DELAY_US     10000

/* Driver registration */

#define DEV_FORMAT                "/dev/input%d"
#define DEV_NAMELEN               16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cst92xx_dev_s
{
  /* Must be the first member: the upper half passes this pointer back */

  struct touch_lowerhalf_s lower;

  FAR const struct cst92xx_config_s *config;
  FAR struct i2c_master_s *i2c;
  struct work_s work;
  mutex_t lock;
  uint8_t crefs;

  /* Contacts reported as down by the previous report, one bit per ID, and
   * their last transformed position so that the release can be reported
   * where the contact was lost.
   */

  uint16_t downmap;
  int16_t lastx[CST92XX_MAX_POINTS];
  int16_t lasty[CST92XX_MAX_POINTS];

  uint8_t frame[CST92XX_FRAME_SIZE(CST92XX_MAX_POINTS)];

  /* Report buffer, aligned for struct touch_sample_s */

  union
  {
    struct touch_sample_s sample;
    uint8_t buf[SIZEOF_TOUCH_SAMPLE_S(CST92XX_MAX_POINTS)];
  } report;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cst92xx_read
 *
 * Description:
 *   Read a block starting at a 16-bit register address.
 *
 ****************************************************************************/

static int cst92xx_read(FAR struct cst92xx_dev_s *priv, uint16_t reg,
                        FAR uint8_t *buffer, size_t buflen)
{
  struct i2c_config_s config;
  uint8_t addr[2];

  config.frequency = priv->config->frequency;
  config.address   = priv->config->address;
  config.addrlen   = 7;

  addr[0] = reg >> 8;
  addr[1] = reg & 0xff;

  return i2c_writeread(priv->i2c, &config, addr, sizeof(addr),
                       buffer, buflen);
}

/****************************************************************************
 * Name: cst92xx_write
 *
 * Description:
 *   Write a 16-bit register/command address optionally followed by one
 *   data byte.
 *
 ****************************************************************************/

static int cst92xx_write(FAR struct cst92xx_dev_s *priv, uint16_t reg,
                         FAR const uint8_t *data, size_t datalen)
{
  struct i2c_config_s config;
  uint8_t buffer[3];

  DEBUGASSERT(datalen <= 1);

  config.frequency = priv->config->frequency;
  config.address   = priv->config->address;
  config.addrlen   = 7;

  buffer[0] = reg >> 8;
  buffer[1] = reg & 0xff;
  if (datalen > 0)
    {
      buffer[2] = data[0];
    }

  return i2c_write(priv->i2c, &config, buffer, 2 + datalen);
}

/****************************************************************************
 * Name: cst92xx_transform
 *
 * Description:
 *   Apply the board swap/mirror configuration to a raw coordinate pair.
 *
 ****************************************************************************/

static void cst92xx_transform(FAR struct cst92xx_dev_s *priv,
                              uint16_t rawx, uint16_t rawy,
                              FAR int16_t *x, FAR int16_t *y)
{
  uint8_t flags = priv->config->flags;
  uint16_t xres = priv->lower.xres;
  uint16_t yres = priv->lower.yres;
  uint16_t tmp;

  if (flags & TOUCH_FLAG_SWAPXY)
    {
      tmp  = rawx;
      rawx = rawy;
      rawy = tmp;
    }

  if (xres > 0)
    {
      if (rawx >= xres)
        {
          rawx = xres - 1;
        }

      if (flags & TOUCH_FLAG_MIRRORX)
        {
          rawx = xres - 1 - rawx;
        }
    }

  if (yres > 0)
    {
      if (rawy >= yres)
        {
          rawy = yres - 1;
        }

      if (flags & TOUCH_FLAG_MIRRORY)
        {
          rawy = yres - 1 - rawy;
        }
    }

  *x = (int16_t)rawx;
  *y = (int16_t)rawy;
}

/****************************************************************************
 * Name: cst92xx_worker
 *
 * Description:
 *   Read one touch report, acknowledge it and convert the reported contacts
 *   into DOWN/MOVE/UP events for the upper half.  Runs on the high priority
 *   work queue, either from the interrupt handler or, in polled mode,
 *   periodically while the device is open.
 *
 ****************************************************************************/

static void cst92xx_worker(FAR void *arg)
{
  FAR struct cst92xx_dev_s *priv = arg;
  FAR struct touch_sample_s *sample = &priv->report.sample;
  FAR struct touch_point_s *point;
  FAR const uint8_t *frame = priv->frame;
  uint16_t rawx[CST92XX_MAX_POINTS];
  uint16_t rawy[CST92XX_MAX_POINTS];
  uint16_t pressed = 0;
  uint64_t timestamp;
  uint8_t maxpoint = priv->lower.maxpoint;
  uint8_t ack = CST92XX_ACK;
  int npoints = 0;
  int count;
  int ret;
  int i;

  memset(rawx, 0, sizeof(rawx));
  memset(rawy, 0, sizeof(rawy));

  if (nxmutex_lock(&priv->lock) < 0)
    {
      return;
    }

  ret = cst92xx_read(priv, CST92XX_REG_POINTS, priv->frame,
                     CST92XX_FRAME_SIZE(maxpoint));

  /* Always acknowledge, otherwise the controller stops reporting */

  cst92xx_write(priv, CST92XX_REG_POINTS, &ack, 1);

  if (ret < 0)
    {
      ierr("ERROR: Failed to read touch report: %d\n", ret);
      goto out;
    }

  /* The first byte still holds a previous ACK if there is no new report,
   * and the marker byte must be present for the report to be valid.
   */

  if (frame[0] == CST92XX_ACK || frame[CST92XX_MARKER_OFFSET] != CST92XX_ACK)
    {
      goto out;
    }

  if ((frame[4] & CST92XX_COVER_MASK) != 0)
    {
      /* The panel is covered (palm detect): treat as all contacts lost */

      iinfo("Cover detected\n");
    }
  else
    {
      count = frame[CST92XX_COUNT_OFFSET] & CST92XX_COUNT_MASK;
      if (count > maxpoint)
        {
          count = maxpoint;
        }

      for (i = 0; i < count; i++)
        {
          FAR const uint8_t *p = &frame[CST92XX_POINT_OFFSET(i)];
          uint8_t id = p[0] >> 4;

          if (id >= maxpoint ||
              (p[0] & CST92XX_EVENT_MASK) != CST92XX_EVENT_PRESSED)
            {
              continue;
            }

          rawx[id] = ((uint16_t)p[1] << 4) | (p[3] >> 4);
          rawy[id] = ((uint16_t)p[2] << 4) | (p[3] & 0x0f);
          pressed |= 1 << id;
        }
    }

  timestamp = touch_get_time();
  memset(&priv->report, 0, sizeof(priv->report));

  for (i = 0; i < maxpoint; i++)
    {
      bool was_down = (priv->downmap & (1 << i)) != 0;
      bool is_down  = (pressed & (1 << i)) != 0;

      if (!was_down && !is_down)
        {
          continue;
        }

      point = &sample->point[npoints++];

      if (is_down)
        {
          cst92xx_transform(priv, rawx[i], rawy[i],
                            &priv->lastx[i], &priv->lasty[i]);
          point->flags = was_down ? TOUCH_MOVE : TOUCH_DOWN;
        }
      else
        {
          point->flags = TOUCH_UP;
        }

      point->flags    |= TOUCH_ID_VALID | TOUCH_POS_VALID;
      point->id        = i;
      point->x         = priv->lastx[i];
      point->y         = priv->lasty[i];
      point->timestamp = timestamp;
    }

  priv->downmap = pressed;

  if (npoints > 0)
    {
      sample->npoints = npoints;
      touch_event(priv->lower.priv, sample);
    }

out:
#ifdef CONFIG_INPUT_CST92XX_POLLMODE
  if (priv->crefs > 0)
    {
      work_queue(HPWORK, &priv->work, cst92xx_worker, priv,
                 MSEC2TICK(CONFIG_INPUT_CST92XX_POLL_PERIOD));
    }
#endif

  nxmutex_unlock(&priv->lock);
}

#ifndef CONFIG_INPUT_CST92XX_POLLMODE
/****************************************************************************
 * Name: cst92xx_interrupt
 *
 * Description:
 *   INT GPIO handler attached by the board through config->attach().
 *
 ****************************************************************************/

static int cst92xx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cst92xx_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, cst92xx_worker, priv, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: cst92xx_open
 ****************************************************************************/

static int cst92xx_open(FAR struct touch_lowerhalf_s *lower)
{
  FAR struct cst92xx_dev_s *priv = (FAR struct cst92xx_dev_s *)lower;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs == UINT8_MAX)
    {
      nxmutex_unlock(&priv->lock);
      return -EMFILE;
    }

  if (priv->crefs++ == 0)
    {
      priv->downmap = 0;

#ifndef CONFIG_INPUT_CST92XX_POLLMODE
      priv->config->enable(priv->config, true);
#endif

      /* Drain any report that is already pending (and, in polled mode,
       * start polling).
       */

      work_queue(HPWORK, &priv->work, cst92xx_worker, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: cst92xx_close
 ****************************************************************************/

static int cst92xx_close(FAR struct touch_lowerhalf_s *lower)
{
  FAR struct cst92xx_dev_s *priv = (FAR struct cst92xx_dev_s *)lower;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs > 0 && --priv->crefs == 0)
    {
#ifndef CONFIG_INPUT_CST92XX_POLLMODE
      priv->config->enable(priv->config, false);
#endif
      work_cancel(HPWORK, &priv->work);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: cst92xx_probe
 *
 * Description:
 *   Reset the controller, read its identification and resolution in info
 *   mode and return it to normal reporting mode.
 *
 ****************************************************************************/

static int cst92xx_probe(FAR struct cst92xx_dev_s *priv,
                         FAR uint16_t *resx, FAR uint16_t *resy)
{
  FAR const struct cst92xx_config_s *config = priv->config;
  uint32_t checkcode;
  uint32_t fwversion;
  uint16_t chiptype;
  uint8_t buf[8];
  int ret;

  if (config->reset != NULL)
    {
      config->reset(config, true);
      nxsig_usleep(CST92XX_RESET_HOLD_US);
      config->reset(config, false);
      nxsig_usleep(CST92XX_RESET_BOOT_US);
    }

  ret = cst92xx_write(priv, CST92XX_CMD_INFO_MODE, NULL, 0);
  if (ret < 0)
    {
      ierr("ERROR: No response at 0x%02x: %d\n", config->address, ret);
      return ret;
    }

  nxsig_usleep(CST92XX_MODE_DELAY_US);

  ret = cst92xx_read(priv, CST92XX_REG_CHECKCODE, buf, 4);
  if (ret < 0)
    {
      goto errout;
    }

  checkcode = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) |
              ((uint32_t)buf[1] << 8) | buf[0];

  ret = cst92xx_read(priv, CST92XX_REG_RESOLUTION, buf, 4);
  if (ret < 0)
    {
      goto errout;
    }

  *resx = ((uint16_t)buf[1] << 8) | buf[0];
  *resy = ((uint16_t)buf[3] << 8) | buf[2];

  ret = cst92xx_read(priv, CST92XX_REG_CHIPTYPE, buf, 4);
  if (ret < 0)
    {
      goto errout;
    }

  chiptype = ((uint16_t)buf[3] << 8) | buf[2];

  ret = cst92xx_read(priv, CST92XX_REG_FWVERSION, buf, 8);
  if (ret < 0)
    {
      goto errout;
    }

  fwversion = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) |
              ((uint32_t)buf[1] << 8) | buf[0];

  iinfo("CST%04x: checkcode 0x%08" PRIx32 " fw 0x%08" PRIx32
        " resolution %ux%u\n", chiptype, checkcode, fwversion,
        *resx, *resy);

  if (fwversion == CST92XX_FW_ERASED)
    {
      ierr("ERROR: Controller has no firmware\n");
      ret = -ENODEV;
      goto errout;
    }

  if ((checkcode & CST92XX_CHECKCODE_MASK) != CST92XX_CHECKCODE)
    {
      ierr("ERROR: Bad firmware check code 0x%08" PRIx32 "\n", checkcode);
      ret = -ENODEV;
      goto errout;
    }

  if (chiptype != CST92XX_CHIP_CST9217 && chiptype != CST92XX_CHIP_CST9220)
    {
      ierr("ERROR: Unsupported chip type 0x%04x\n", chiptype);
      ret = -ENODEV;
      goto errout;
    }

  ret = cst92xx_write(priv, CST92XX_CMD_NORMAL_MODE, NULL, 0);
  nxsig_usleep(CST92XX_MODE_DELAY_US);
  return ret;

errout:
  cst92xx_write(priv, CST92XX_CMD_NORMAL_MODE, NULL, 0);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cst92xx_register
 *
 * Description:
 *   Reset and probe the CST92xx over I2C, configure the touchscreen lower
 *   half and register it as /dev/inputN where N is the given minor number.
 *
 * Input Parameters:
 *   i2c    - I2C master used to talk to the controller
 *   config - Persistent board configuration
 *   minor  - Device minor number used to form /dev/inputN
 *
 * Returned Value:
 *   Zero (OK) on success.  Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int cst92xx_register(FAR struct i2c_master_s *i2c,
                     FAR const struct cst92xx_config_s *config, int minor)
{
  FAR struct cst92xx_dev_s *priv;
  char devname[DEV_NAMELEN];
  uint16_t resx = 0;
  uint16_t resy = 0;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL);

#ifndef CONFIG_INPUT_CST92XX_POLLMODE
  if (config->attach == NULL || config->enable == NULL)
    {
      ierr("ERROR: attach() and enable() are required\n");
      return -EINVAL;
    }
#endif

  priv = kmm_zalloc(sizeof(struct cst92xx_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->config = config;
  priv->i2c    = i2c;
  nxmutex_init(&priv->lock);

  ret = cst92xx_probe(priv, &resx, &resy);
  if (ret < 0)
    {
      goto errout_with_priv;
    }

  if (config->flags & TOUCH_FLAG_SWAPXY)
    {
      uint16_t tmp = resx;

      resx = resy;
      resy = tmp;
    }

  priv->lower.maxpoint = CST92XX_MAX_POINTS;
  priv->lower.xres     = config->xres != 0 ? config->xres : resx;
  priv->lower.yres     = config->yres != 0 ? config->yres : resy;
  priv->lower.flags    = 0; /* Swap/mirror is handled by this driver */
  priv->lower.open     = cst92xx_open;
  priv->lower.close    = cst92xx_close;

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  ret = touch_register(&priv->lower, devname, CONFIG_INPUT_CST92XX_NBUFFERS);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register %s: %d\n", devname, ret);
      goto errout_with_priv;
    }

#ifndef CONFIG_INPUT_CST92XX_POLLMODE
  ret = config->attach(config, cst92xx_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt: %d\n", ret);
      touch_unregister(&priv->lower, devname);
      goto errout_with_priv;
    }

  /* Keep the interrupt disabled until the device is opened */

  config->enable(config, false);
#endif

  iinfo("Registered %s, %ux%u, %u points\n", devname,
        priv->lower.xres, priv->lower.yres, priv->lower.maxpoint);
  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
