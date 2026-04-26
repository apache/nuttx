/****************************************************************************
 * drivers/timers/dshot.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/dshot.h>

#ifdef CONFIG_DSHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSHOT_GCR_MASK       0x000fffffu
#define DSHOT_ERPMSCALE_NUM  600000u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dshot_upperhalf_s
{
  uint8_t crefs;
  mutex_t lock;
  FAR struct dshot_lowerhalf_s *dev;
  struct dshot_ch_telemetry_s telemetry[_DSHOT_NCHANNELS];
  bool bidir;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dshot_open(FAR struct file *filep);
static int dshot_close(FAR struct file *filep);
static ssize_t dshot_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t dshot_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int dshot_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dshotops =
{
  dshot_open,  /* open */
  dshot_close, /* close */
  dshot_read,  /* read */
  dshot_write, /* write */
  NULL,        /* seek */
  dshot_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dshot_crc
 *
 * Description:
 *   Calculate the 4-bit CRC for a DShot packet.
 *
 * Input Parameters:
 *   packet - The DShot packet to calculate CRC for.
 *   bidir  - Boolean indicating if bidirectional DShot is enabled.
 *
 * Returned Value:
 *   The calculated 4-bit CRC value.
 *
 ****************************************************************************/

static uint16_t dshot_crc(uint16_t packet, bool bidir)
{
  uint16_t crc;

  crc = packet >> 4;
  crc = crc ^ (crc >> 4) ^ (crc >> 8);

  if (bidir)
    {
      crc = ~crc;
    }

  return crc & 0xf;
}

/****************************************************************************
 * Name: dshot_build_packet
 *
 * Description:
 *   Build a complete DShot packet from throttle, telemetry, and bidir
 *   values.
 *
 * Input Parameters:
 *   throttle   - Throttle value (0-2047).
 *   telemetry  - Boolean indicating if telemetry request is enabled.
 *   bidir      - Boolean indicating if bidirectional DShot is enabled.
 *
 * Returned Value:
 *   The constructed 16-bit DShot packet.
 *
 ****************************************************************************/

static uint16_t dshot_build_packet(uint16_t throttle, bool telemetry,
                                   bool bidir)
{
  uint16_t packet = 0;
  packet |= throttle << 5;
  packet |= (telemetry ? 1 << 4 : 0);
  packet |= dshot_crc(packet, bidir);

  return packet;
}

/****************************************************************************
 * Name: dshot_parse_raw_packet
 *
 * Description:
 *   Parse raw pattern from the line into a 16-bit packet and verify CRC.
 *   Steps:
 *   1. Extract GCR20 value from the raw value
 *   2. Extract the 16-bit packet from the GCR20
 *   3. Check the packet CRC
 *
 * Input Parameters:
 *   raw - The raw encoded packet value to parse.
 *
 * Returned Value:
 *   On success, returns the 12-bit payload as a positive integer.
 *   On error, returns a negative error code.
 *
 ****************************************************************************/

static int dshot_parse_raw_packet(uint32_t raw)
{
  static const int8_t gcr_decode[32] =
    {
      -1, -1, -1, -1, -1, -1, -1, -1,
      -1,  9, 10, 11, -1, 13, 14, 15,
      -1, -1,  2,  3, -1,  5,  6,  7,
      -1,  0,  8,  1, -1,  4, 12, -1
    };

  uint32_t gcr20 = (raw ^ (raw >> 1)) & DSHOT_GCR_MASK;
  uint16_t packet = 0;
  int i;

  for (i = 0; i < 4; i++)
    {
      int16_t nibble = gcr_decode[gcr20 & 0x1f];

      if (nibble < 0)
        {
          return -EINVAL;
        }

      packet |= nibble << (4 * i);
      gcr20 >>= 5;
    }

  if (dshot_crc(packet, true) != (packet & 0xf))
    {
      return -EINVAL;
    }

  return packet >> 4;
}

/****************************************************************************
 * Name: dshot_copy_telemetry
 *
 * Description:
 *   Copy telemetry data for selected channels from upper-half to output.
 *
 * Input Parameters:
 *   upper   - Pointer to the upper-half driver state.
 *   out     - Pointer to output telemetry array.
 *   ch_mask - Bitmask indicating which channels to copy.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void dshot_copy_telemetry(FAR struct dshot_upperhalf_s *upper,
                                 FAR struct dshot_ch_telemetry_s *out,
                                 uint16_t ch_mask)
{
  int i;

  for (i = 0; i < _DSHOT_NCHANNELS; i++)
    {
      if ((ch_mask & (1u << i)) != 0)
        {
          out[i] = upper->telemetry[i];
        }
    }
}

/****************************************************************************
 * Name: dshot_update_telemetry
 *
 * Description:
 *   Update telemetry data from lower-half driver for selected channels.
 *   Parses raw telemetry data and extracts EDT or eRPM information.
 *
 * Input Parameters:
 *   upper   - Pointer to the upper-half driver state.
 *   ch_mask - Bitmask indicating which channels to update.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dshot_update_telemetry(FAR struct dshot_upperhalf_s *upper,
                                  uint16_t ch_mask)
{
  FAR struct dshot_lowerhalf_s *lower = upper->dev;
  struct dshot_raw_telemetry_s raw[_DSHOT_NCHANNELS];
  int i;
  int ret;

  memset(raw, 0, sizeof(raw));

  DEBUGASSERT(lower->ops->get_raw_telemetry != NULL);
  ret = lower->ops->get_raw_telemetry(lower, raw, ch_mask);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < _DSHOT_NCHANNELS; i++)
    {
      if ((ch_mask & (1u << i)) != 0)
        {
          int value = dshot_parse_raw_packet(raw[i].raw);

          if (value > 0)
            {
              uint16_t telem = (uint16_t)value & 0x0fff;
              uint8_t exponent = telem >> 9;

              if ((exponent & 1) == 0)
                {
                  upper->telemetry[i].edt_type = (uint8_t)(telem >> 8);
                  upper->telemetry[i].edt_value = (uint8_t)(telem & 0xff);
                  upper->telemetry[i].timestamp = raw[i].timestamp;
                }
              else
                {
                  uint32_t mantissa = telem & 0x1ff;
                  uint32_t period = mantissa << exponent;
                  if (period != 0)
                    {
                      uint32_t rpm = DSHOT_ERPMSCALE_NUM / period;
                      upper->telemetry[i].erpm =
                        (uint16_t)((rpm > 0xffff) ? 0xffff : rpm);
                      upper->telemetry[i].timestamp = raw[i].timestamp;
                    }
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: dshot_open
 *
 * Description:
 *   This function is called whenever the DShot device is opened.
 *
 * Input Parameters:
 *   filep - A pointer to the file structure instance.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dshot_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct dshot_upperhalf_s *upper = inode->i_private;
  uint8_t tmp;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      nxmutex_unlock(&upper->lock);
      return -EMFILE;
    }

  if (tmp == 1)
    {
      DEBUGASSERT(upper->dev->ops->setup != NULL);
      ret = upper->dev->ops->setup(upper->dev);
      if (ret < 0)
        {
          nxmutex_unlock(&upper->lock);
          return ret;
        }
    }

  upper->crefs = tmp;

  nxmutex_unlock(&upper->lock);
  return OK;
}

/****************************************************************************
 * Name: dshot_close
 *
 * Description:
 *   This function is called when the DShot device is closed.
 *
 * Input Parameters:
 *   filep - A pointer to the file structure instance.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dshot_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct dshot_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      upper->crefs = 0;
      DEBUGASSERT(upper->dev->ops->shutdown != NULL);
      upper->dev->ops->shutdown(upper->dev);
    }

  nxmutex_unlock(&upper->lock);
  return OK;
}

/****************************************************************************
 * Name: dshot_read
 *
 * Description:
 *   A dummy read method. This is provided only to satisfy the VFS layer.
 *
 * Input Parameters:
 *   filep  - A pointer to the file structure instance.
 *   buffer - The user-provided buffer into which data will be returned.
 *   buflen - The size of the buffer in bytes.
 *
 * Returned Value:
 *   Always returns 0 (end-of-file).
 *
 ****************************************************************************/

static ssize_t dshot_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: dshot_write
 *
 * Description:
 *   A dummy write method. This is provided only to satisfy the VFS layer.
 *
 * Input Parameters:
 *   filep  - A pointer to the file structure instance.
 *   buffer - The user-provided buffer from which data will be written.
 *   buflen - The number of bytes to be written.
 *
 * Returned Value:
 *   Always returns -EPERM (operation not permitted).
 *
 ****************************************************************************/

static ssize_t dshot_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -EPERM;
}

/****************************************************************************
 * Name: dshot_ioctl
 *
 * Description:
 *   The standard ioctl method. This is used to perform DShot-specific
 *   operations including setting throttle, configuring channels, and
 *   retrieving telemetry data.
 *
 * Input Parameters:
 *   filep - A pointer to the file structure instance.
 *   cmd   - The ioctl command code.
 *   arg   - The argument provided with the ioctl command.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dshot_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct dshot_upperhalf_s *upper = inode->i_private;
  FAR struct dshot_lowerhalf_s *lower = upper->dev;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case DSHOTIOC_CONFIGURE:
        {
          FAR struct dshot_config_s *cfg =
            (FAR struct dshot_config_s *)((uintptr_t)arg);

          if (cfg == NULL)
            {
              ret = -EINVAL;
              break;
            }

          DEBUGASSERT(lower->ops->configure);
          ret = lower->ops->configure(lower, cfg);
          upper->bidir = cfg->bidir;
        }
        break;

      case DSHOTIOC_SET_THROTTLE:
        {
          FAR struct dshot_throttle_s *req =
            (FAR struct dshot_throttle_s *)((uintptr_t)arg);
          uint16_t packets[_DSHOT_NCHANNELS];
          int i;

          if (req == NULL)
            {
              ret = -EINVAL;
              break;
            }

          /* Retrieve the latest telemetry */

          if (req->telemetry_req != 0)
            {
              ret = dshot_update_telemetry(upper, req->telemetry_req);
              if (ret < 0)
                {
                  break;
                }

              dshot_copy_telemetry(upper, req->ch_telemetry,
                                   req->telemetry_req);
            }

          /* Build DShot packets for each channel */

          for (i = 0; i < _DSHOT_NCHANNELS; i++)
            {
              if ((req->ch_mask & (1u << i)) != 0)
                {
                  bool telemetry = (req->telemetry_req & (1u << i)) != 0;
                  packets[i] =
                    dshot_build_packet(req->throttle[i], telemetry,
                                       upper->bidir);
                }
            }

          /* Send the packets */

          DEBUGASSERT(lower->ops->send_command);
          ret = lower->ops->send_command(lower, packets, req->ch_mask);
        }
        break;

      case DSHOTIOC_GET_TELEMETRY:
        {
          FAR struct dshot_telemetry_s *tlm =
            (FAR struct dshot_telemetry_s *)((uintptr_t)arg);

          if (tlm == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = dshot_update_telemetry(upper, tlm->ch_mask);
          if (ret < 0)
            {
              break;
            }

          dshot_copy_telemetry(upper, tlm->ch_telemetry, tlm->ch_mask);
          ret = OK;
        }
        break;

      default:
        {
          if (lower->ops->ioctl != NULL)
            {
              ret = lower->ops->ioctl(lower, cmd, arg);
            }
          else
            {
              ret = -ENOTTY;
            }
        }
        break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dshot_register
 *
 * Description:
 *   This function binds an instance of a "lower half" DShot driver with the
 *   "upper half" DShot device and registers that device so that it can be
 *   used by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input Parameters:
 *   path - The full path to the driver to be registered in the NuttX pseudo-
 *     filesystem. The recommended convention is to name all DShot drivers
 *     as "/dev/dshot0", "/dev/dshot1", etc., where the driver path differs
 *     only in the "minor" number at the end of the device name.
 *   dev  - A pointer to an instance of lower half DShot driver. This
 *     instance is bound to the DShot driver and must persist as long as the
 *     driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int dshot_register(FAR const char *path, FAR struct dshot_lowerhalf_s *dev)
{
  FAR struct dshot_upperhalf_s *upper;
  int ret;

  if (dev == NULL || dev->ops == NULL)
    {
      return -EINVAL;
    }

  upper = kmm_zalloc(sizeof(struct dshot_upperhalf_s));
  if (upper == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&upper->lock);
  upper->dev = dev;

  ret = register_driver(path, &g_dshotops, 0666, upper);
  if (ret < 0)
    {
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_DSHOT */
