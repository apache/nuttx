/****************************************************************************
 * arch/sim/src/sim/posix/up_i2cbuslinux.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "up_i2cbus.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "up_i2cbuslinux: " fmt "\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        syslog(LOG_ERR, "up_i2cbuslinux: " fmt "\n", ##__VA_ARGS__)
#define DEBUG(fmt, ...)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct linux_i2cbus_master_s
{
  const struct i2c_ops_s *ops; /* I2C vtable */
  int file;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int linux_i2cbus_transfer(struct i2c_master_s *dev,
                                 struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int linux_i2cbus_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_ops_s i2c_linux_ops =
{
  .transfer = linux_i2cbus_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = linux_i2cbus_reset,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: linux_i2cbus_reset
 *
 * Description:
 *   Provide i2c reset
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int linux_i2cbus_reset(struct i2c_master_s *dev)
{
  return -1; /* Not implemented */
}
#endif

/****************************************************************************
 * Name: linux_i2cbus_transfer
 *
 * Description:
 *   Provide i2c transfer
 *
 ****************************************************************************/

static int linux_i2cbus_transfer(struct i2c_master_s *dev,
                                 struct i2c_msg_s *msgs, int count)
{
  int ret = 0;
  struct linux_i2cbus_master_s *priv = (struct linux_i2cbus_master_s *)dev;
  int file = priv->file;
  uint8_t *pack_buf = NULL;
  struct i2c_rdwr_ioctl_data ioctl_data;
  struct i2c_msg l_msgs[2];  /* We only support up to 2 messages */
  ioctl_data.msgs = l_msgs;

  /* Many i2c bus do not play well with combined messages via the Linux
   * interface, this makes stitching things together a little harder
   * because NuttX provides the ability to hold the bus without ending
   * with a STOP which is not ideal in general, and not possible with
   * Linux.
   */

  if ((msgs[0].flags & NUTTX_I2C_M_TEN) || (msgs[1].flags & NUTTX_I2C_M_TEN))
    {
      /* Linux also has somewhat poor support for 10bit addresses and they
       * are quite rare so we just don't support them for now here
       */

      return -1;
    }

  ioctl_data.msgs = l_msgs;

  if (count == 1)
    {
      if (msgs[0].flags & NUTTX_I2C_M_NOSTOP || \
          msgs[0].flags & NUTTX_I2C_M_NOSTART)
        {
          /* Do not support leaving the bus hanging or try to send
           * without first starting.
           */

          return -1;
        }

      l_msgs[0].addr = msgs[0].addr;
      l_msgs[0].buf = msgs[0].buffer;
      l_msgs[0].len = msgs[0].length;
      l_msgs[0].flags = 0;
      if (msgs[0].flags & NUTTX_I2C_M_READ)
        {
           l_msgs[0].flags |= I2C_M_RD;
        }

      ioctl_data.nmsgs = 1;
    }
  else if(count == 2)
    {
      /* Addresses should be the same */

      if (msgs[0].addr != msgs[1].addr)
        {
          return -1;
        }

      /* Check if we are about to do a read of a register
       * NuttX interface represents this as WRITE(NOSTOP) + READ
       * Linux interface represents this as WRITE + READ
       */

      if (msgs[0].flags & NUTTX_I2C_M_NOSTOP && \
          msgs[1].flags & NUTTX_I2C_M_READ)
        {
          l_msgs[0].addr = msgs[0].addr;
          l_msgs[0].flags = 0;
          l_msgs[0].buf = msgs[0].buffer;
          l_msgs[0].len = msgs[0].length;

          l_msgs[1].addr = msgs[1].addr;
          l_msgs[1].flags = I2C_M_RD;
          l_msgs[1].buf = msgs[1].buffer;
          l_msgs[1].len = msgs[1].length;
          ioctl_data.nmsgs = 2;
        }
      else if (!(msgs[0].flags & NUTTX_I2C_M_READ) && \
               !(msgs[1].flags & NUTTX_I2C_M_READ) && \
               (msgs[0].flags & NUTTX_I2C_M_NOSTOP) && \
               (msgs[1].flags & NUTTX_I2C_M_NOSTART))
        {
          /* These writes are actually just a single write in Linux
           * so we pack the data in a single buffer and the unpack
           * it at the end.  This could support for for more than just 2
           * messages, but in most cases it is just two because it is
           * connivent to write the register address and the data into two
           * different buffers.
           */

          pack_buf = malloc(msgs[0].length + msgs[1].length);
          if (pack_buf == NULL)
            {
              return -1;
            }

          memcpy(pack_buf, msgs[0].buffer, msgs[0].length);
          memcpy(pack_buf + msgs[0].length, msgs[1].buffer, msgs[1].length);
          l_msgs[0].len = msgs[0].length + msgs[1].length;
          l_msgs[0].flags = 0;
          l_msgs[0].addr = msgs[0].addr;
          ioctl_data.msgs[0].buf = pack_buf;
          ioctl_data.nmsgs = 1;
        }
      else
        {
          /* Many busses cannot handle more than 2 messages */

          return -1;
        }
    }
  else
    {
      return -1;
    }

  if (ioctl(file, I2C_RDWR, &ioctl_data) < 1)
    {
      ret = -1;
    }

  /* Unpack from buffer back to msg buffers if needed */

  if (pack_buf != NULL)
    {
      if (ret == 0)
        {
          int idx;
          uint8_t *msg_p = pack_buf;
          for (idx = 0; idx < count; idx++)
            {
              memcpy(msgs[idx].buffer, msg_p, msgs[idx].length);
              msg_p += msgs[idx].length;
            }
        }

      free(pack_buf);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *sim_i2cbus_initialize(int bus)
{
  struct linux_i2cbus_master_s *priv;
  char filename[20];

  priv = (struct linux_i2cbus_master_s *)malloc(sizeof(priv));
  if (priv == NULL)
    {
      ERROR("Failed to allocate private i2c master driver");
      return NULL;
    }

  snprintf(filename, 19, "/dev/i2c-%d", bus);
  priv->file = open(filename, O_RDWR);
  if (priv->file < 0)
    {
      ERROR("Failed to open %s: %d", filename, priv->file);
      free(priv);
      return NULL;
    }

  priv->ops = &i2c_linux_ops;
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: sim_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int sim_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct linux_i2cbus_master_s *priv = (struct linux_i2cbus_master_s *)dev;
  if (priv->file >= 0)
    {
      close(priv->file);
    }

  free(priv);
  return 0;
}
