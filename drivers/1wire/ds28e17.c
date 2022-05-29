/****************************************************************************
 * drivers/1wire/ds28e17.c
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
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/1wire/1wire_master.h>
#include <nuttx/1wire/1wire_crc.h>
#include <nuttx/1wire/1wire.h>

#include <nuttx/1wire/ds28e17.h>

#include "1wire_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DS28E17_I2C_FREQUENCY
#  define CONFIG_DS28E17_I2C_FREQUENCY 400000
#endif

/* DS28E17 device 1-Wire family ID */

#define DS_FAMILY                    0x19

/* DS28E17 device I2C commands */

#define DS_WRITE_DATA_WITH_STOP      0x4b
#define DS_WRITE_DATA_NO_STOP        0x5a
#define DS_WRITE_DATA_ONLY           0x69
#define DS_WRITE_DATA_ONLY_WITH_STOP 0x78
#define DS_READ_DATA_WITH_STOP       0x87
#define DS_WRITE_READ_DATA_WITH_STOP 0x2d
#define DS_WRITE_CONFIGURATION       0xd2
#define DS_READ_CONFIGURATION        0xe1
#define DS_ENABLE_SLEEP_MODE         0x1e
#define DS_READ_DEVICE_REVISION      0xc4

/* DS28E17 status bits */

#define DS_STATUS_CRC                0x01
#define DS_STATUS_ADDRESS            0x02
#define DS_STATUS_START              0x08

/* Maximum number of I2C bytes to transfer within one CRC16 protected onewire
 * command (same for either read and write).
 */

#define DS_DATA_LIMIT                255

/* Default I2C frequency to use when DS28E17 is detected. */

#define DS_DEFAULT_FREQUENCY         CONFIG_DS28E17_I2C_FREQUENCY

/* Default I2C stretch value. */

#define DS_DEFAULT_STRETCH           1

/* How many times to retry check for chip busy condition vanishing. */

#define DS_BUSYWAIT_RETRIES          500

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ds28e17_dev_s                   /* Must be cast-compatible with onewire_master_s */
{
  FAR struct onewire_dev_s *dev;       /* 1-wire interface */
};

/* I2C Device, Instance */

struct ds_i2c_inst_s                   /* Must be cast-compatible with i2c_master_s */
{
  FAR const struct i2c_ops_s *ops;     /* Standard I2C operations */

  /* 1-wire data */

  FAR struct onewire_master_s *master; /* 1-wire bus master */
  FAR struct onewire_slave_s slave;    /* Our slave data on 1-wire bus */

  /* I2C data */

  uint32_t frequency;                  /* Current I2C frequency */
  uint8_t  stretch;                    /* Current I2C stretch value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int ds_i2c_reset(FAR struct i2c_master_s *i2cdev);
#endif
static int ds_i2c_transfer(FAR struct i2c_master_s *i2cdev,
                           FAR struct i2c_msg_s *msgs, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device Structures, Instantiation */

static const struct i2c_ops_s ds_i2c_ops =
{
  .transfer = ds_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = ds_i2c_reset
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ****************************************************************************/

static inline int ds_i2c_sem_wait(FAR struct i2c_master_s *i2cdev)
{
  FAR struct ds_i2c_inst_s *inst = (FAR struct ds_i2c_inst_s *)i2cdev;
  FAR struct onewire_master_s *master = inst->master;

  return nxrmutex_lock(&master->devlock);
}

/****************************************************************************
 * Name: ds_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void ds_i2c_sem_post(FAR struct i2c_master_s *i2cdev)
{
  FAR struct ds_i2c_inst_s *inst = (FAR struct ds_i2c_inst_s *)i2cdev;
  FAR struct onewire_master_s *master = inst->master;

  nxrmutex_unlock(&master->devlock);
}

static int ds_error(uint8_t buf[])
{
  /* Warnings */

  if (buf[0] & DS_STATUS_CRC)
    {
      i2cwarn("crc16 match failed\n");
    }

  if ((buf[0] & (DS_STATUS_CRC | DS_STATUS_ADDRESS)) == 0 && buf[1] != 0)
    {
      i2cwarn("I2C short write, no ack for %d bytes\n", buf[1]);
    }

  /* Error conditions */

  if (buf[0] & DS_STATUS_ADDRESS)
    {
      i2cerr("I2C device not responding\n");
      return -ENXIO;
    }

  if (buf[0] & DS_STATUS_START)
    {
      i2cerr("I2C status start\n");
      return -EAGAIN;
    }

  if (buf[0] != 0 || buf[1] != 0)
    {
      i2cerr("I2C IO error\n");
      return -EIO;
    }

  return OK;
}

static inline int ds_busywait_time(FAR struct ds_i2c_inst_s *inst)
{
  int d;

  /* Return busywait time (us) for I2C speeds 100 kHz, 400 kHz
   * and 900 kHz, adjusted with current stretch value.
   */

  switch (inst->frequency)
    {
      case 100000:
        d = 90;
        break;

      case 400000:
      default:
        d = 23;
        break;

      case 900000:
        d = 10;
        break;
    }

  return d * inst->stretch;
}

static int ds_busywait(FAR struct ds_i2c_inst_s *inst, size_t length)
{
  FAR struct onewire_master_s *master = inst->master;
  int retries = DS_BUSYWAIT_RETRIES;
  int delay;
  int ret;
  uint8_t bit = 1;

  /* Calculate delay time from current I2C settings. */

  delay = ds_busywait_time(inst);

  do
    {
      ret = ONEWIRE_READBIT(master->dev, &bit);
      if (ret < 0)
        {
          i2cerr("ERROR: ONEWIRE_READBIT failed\n");
          return ret;
        }

      if (bit == 0)
        {
          return OK;
        }

      if (retries == DS_BUSYWAIT_RETRIES)
        {
          /* First time, after checking bit once, do a big delay
           * for I2C to process all bytes in message.
           */

          up_udelay(delay * length);
        }
      else
        {
          /* Otherwise, wait for one byte duration. */

         up_udelay(delay);
        }
    }
  while (retries-- > 0);

  i2cwarn("busywait timeout\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: ds_i2c_read
 *
 * Description:
 *   Read data from I2C slave.
 *
 ****************************************************************************/

static int ds_i2c_read(FAR struct ds_i2c_inst_s *inst, uint16_t i2c_addr,
                       FAR uint8_t *buffer, ssize_t length)
{
  FAR struct onewire_master_s *master = inst->master;
  uint16_t crc;
  int ret;
  uint8_t buf[5];

  if (length <= 0)
    {
      return -EINVAL;
    }

  if (length > DS_DATA_LIMIT)
    {
      i2cerr("ERROR: reading too many bytes!\n");
      return -EINVAL;
    }

  /* Send command to DS28E17. */

  buf[0] = DS_READ_DATA_WITH_STOP;
  buf[1] = i2c_addr << 1 | 0x01;
  buf[2] = length;

  crc = onewire_crc16(buf, 3, 0);
  buf[3] = ~(crc & 0xff);
  buf[4] = ~((crc >> 8) & 0xff);

  ret = ONEWIRE_WRITE(master->dev, buf, 5);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait busy indication to vanish. */

  ret = ds_busywait(inst, length + 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Read status from DS28E17. */

  ret = ONEWIRE_READ(master->dev, buf, 1);
  buf[1] = 0;

  /* Check error conditions. */

  ret = ds_error(buf);
  if (ret < 0)
    {
      return ret;
    }

  /* Read received I2C data from DS28E17. */

  return ONEWIRE_READ(master->dev, buffer, length);
}

/****************************************************************************
 * Name: ds_i2c_write
 *
 * Description:
 *   Write data to I2C slave.
 *
 ****************************************************************************/

static int ds_i2c_write(FAR struct ds_i2c_inst_s *inst, uint16_t i2c_addr,
                        FAR const uint8_t *buffer, ssize_t length, bool stop)
{
  FAR struct onewire_master_s *master = inst->master;
  uint16_t crc;
  int ret;
  uint8_t buf[3];

  if (length <= 0)
    {
      return -EINVAL;
    }

  if (length > DS_DATA_LIMIT)
    {
      /* TODO: split big writes into multiple chunks. */

      i2cerr("ERROR: writing too many bytes!\n");
      return -EINVAL;
    }

  /* Write command header. */

  buf[0] = stop ? DS_WRITE_DATA_WITH_STOP : DS_WRITE_DATA_NO_STOP;
  buf[1] = i2c_addr << 1;
  buf[2] = length;
  crc = onewire_crc16(buf, 3, 0);

  ret = ONEWIRE_WRITE(master->dev, buf, 3);
  if (ret < 0)
    {
      return ret;
    }

  /* Write payload I2C data to DS28E17. */

  crc = onewire_crc16(buffer, length, crc);

  ret = ONEWIRE_WRITE(master->dev, buffer, length);
  if (ret < 0)
    {
      return ret;
    }

  /* Write checksum. */

  buf[0] = ~(crc & 0xff);
  buf[1] = ~((crc >> 8) & 0xff);

  ret = ONEWIRE_WRITE(master->dev, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait busy indication to vanish. */

  ret = ds_busywait(inst, length + 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Read status from DS28E17. */

  ret = ONEWIRE_READ(master->dev, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  /* Check error conditions. */

  ret = ds_error(buf);
  if (ret < 0)
    {
      return ret;
    }

  /* Return count of bytes written. */

  return length;
}

/****************************************************************************
 * Name: ds_i2c_write_read
 *
 * Description:
 *   Write data to I2C slave and read from same address.
 *
 ****************************************************************************/

static int ds_i2c_write_read(FAR struct ds_i2c_inst_s *inst,
                             uint16_t i2c_addr,
                             FAR const uint8_t *wbuffer, ssize_t wlength,
                             FAR uint8_t *rbuffer, ssize_t rlength)
{
  FAR struct onewire_master_s *master = inst->master;
  uint16_t crc;
  int ret;
  uint8_t buf[3];

  if (wlength <= 0 || rlength <= 0)
    {
      return -EINVAL;
    }

  if (wlength > DS_DATA_LIMIT)
    {
      i2cerr("ERROR: writing too many bytes!\n");
      return -EINVAL;
    }

  if (rlength > DS_DATA_LIMIT)
    {
      i2cerr("ERROR: reading too many bytes!\n");
      return -EINVAL;
    }

  /* Write command header. */

  buf[0] = DS_WRITE_READ_DATA_WITH_STOP;
  buf[1] = i2c_addr << 1;
  buf[2] = wlength;
  crc    = onewire_crc16(buf, 3, 0);

  ret = ONEWIRE_WRITE(master->dev, buf, 3);
  if (ret < 0)
    {
      return ret;
    }

  /* Write payload I2C data to DS28E17. */

  crc = onewire_crc16(wbuffer, wlength, crc);

  ret = ONEWIRE_WRITE(master->dev, wbuffer, wlength);
  if (ret < 0)
    {
      return ret;
    }

  /* Write read length and checksum. */

  buf[0] = rlength;
  crc    = onewire_crc16(buf, 1, crc);
  buf[1] = ~(crc & 0xff);
  buf[2] = ~((crc >> 8) & 0xff);

  ret = ONEWIRE_WRITE(master->dev, buf, 3);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait busy indication to vanish. */

  ret = ds_busywait(inst, wlength + 1 + rlength + 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Read status from DS28E17. */

  ret = ONEWIRE_READ(master->dev, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  /* Check error conditions. */

  ret = ds_error(buf);
  if (ret < 0)
    {
      return ret;
    }

  /* Read received I2C data from DS28E17. */

  return ONEWIRE_READ(master->dev, rbuffer, rlength);
}

static int ds_i2c_setfrequency(FAR struct ds_i2c_inst_s *inst,
                               uint32_t frequency)
{
  FAR struct onewire_master_s *master = inst->master;
  uint8_t buf[2];
  int speed;
  int ret;

  switch (frequency)
    {
      case 100000:
        speed = 0;
        break;

      case 400000:
        speed = 1;
        break;

      case 900000:
        speed = 2;
        break;

      default:
        i2cerr("ERROR: bad I2C freq %lu\n", frequency);
        return -EINVAL;
    }

  i2cinfo("Changing I2C freq %lu -> %lu\n", inst->frequency, frequency);

  /* Select DS28E17 */

  ret = onewire_reset_select(master, inst->slave.romcode);
  if (ret < 0)
    {
      i2cerr("ERROR: cannot change I2C freq\n");
      return ret;
    }

  /* Write new speed to device */

  buf[0] = DS_WRITE_CONFIGURATION;
  buf[1] = speed;

  ret = ONEWIRE_WRITE(master->dev, buf, 2);
  if (ret < 0)
    {
      i2cerr("ERROR: cannot change I2C freq\n");
      return ret;
    }

  inst->frequency = frequency;
  return ret;
}

/****************************************************************************
 * Name: ds_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 *   Initiates a master mode transaction on the I2C bus to transfer the
 *   provided messages to and from the slave devices.
 *
 ****************************************************************************/

static int ds_i2c_process(FAR struct i2c_master_s *i2cdev,
                          FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct ds_i2c_inst_s *inst = (FAR struct ds_i2c_inst_s *)i2cdev;
  FAR struct onewire_master_s *master = inst->master;
  int ret;
  int i;

  /* Check from first message only, if we want to change I2C frequency. */

  if (inst->frequency != msgs[0].frequency)
    {
      ds_i2c_setfrequency(inst, msgs[0].frequency);
    }

  /* Select DS28E17 */

  i = onewire_reset_select(master, inst->slave.romcode);
  if (i < 0)
    {
      goto errout;
    }

  while (i < count)
    {
      /* First we try to use DS_WRITE_READ_DATA_WITH_STOP to optimize
       * the common case of write followed by read to a same address.
       */

      if (i < count - 1 && msgs[i].addr == msgs[i + 1].addr &&
          (msgs[i].flags & I2C_M_READ) == 0 &&
          (msgs[i + 1].flags & I2C_M_READ))
        {
          /* Write-read combined transfer. */

          ret = ds_i2c_write_read(inst, msgs[i].addr,
                                  msgs[i].buffer, msgs[i].length,
                                  msgs[i + 1].buffer, msgs[i + 1].length);
          if (ret < 0)
            {
              i = ret;
              goto errout;
            }

          /* We processed two messages here. */

          i += 2;
        }
      else if (msgs[i].flags & I2C_M_READ)
        {
          /* Read transfer. */

          ret = ds_i2c_read(inst, msgs[i].addr, msgs[i].buffer,
                            msgs[i].length);
          if (ret < 0)
            {
              i = ret;
              goto errout;
            }

          i++;
        }
      else
        {
          /* Write transfer. Stop condition only for last transfer. */

          ret = ds_i2c_write(inst, msgs[i].addr, msgs[i].buffer,
                             msgs[i].length, i == (count - 1));
          if (ret < 0)
            {
              i = ret;
              goto errout;
            }

          i++;
        }

      /* Any more messages to process? */

      if (i < count)
        {
          /* Yes. Resume to same DS28E17.
           * Oddness: Skip-ROM does not set RS-bit needed by resume.
           */

          if (master->nslaves > 1)
            {
              ret = onewire_reset_resume(master);
            }
          else
            {
              ret = onewire_reset_select(master, inst->slave.romcode);
            }

          if (ret < 0)
            {
              i = ret;
              goto errout;
            }
        }
    }

errout:
  return i;
}

/****************************************************************************
 * Name: ds_i2c_reset
 *
 * Description:
 *   Reset an I2C bus
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int ds_i2c_reset(FAR struct i2c_master_s *i2cdev)
{
  FAR struct ds_i2c_inst_s *inst = (FAR struct ds_i2c_inst_s *)i2cdev;
  FAR struct onewire_master_s *master = inst->master;
  int ret;

  ret = ds_i2c_sem_wait(i2cdev);
  if (ret < 0)
    {
      return ret;
    }

  ret = ONEWIRE_RESET(master->dev);
  ds_i2c_sem_post(i2cdev);

  return ret;
}
#endif

/****************************************************************************
 * Name: ds_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int ds_i2c_transfer(FAR struct i2c_master_s *i2cdev,
                           FAR struct i2c_msg_s *msgs,
                           int count)
{
  int ret;

  ret = ds_i2c_sem_wait(i2cdev);
  if (ret < 0)
    {
      return ret;
    }

  ret = ds_i2c_process(i2cdev, msgs, count);
  ds_i2c_sem_post(i2cdev);

  return ret;
}

/****************************************************************************
 * Name: ds28e17_selftest
 *
 * Description:
 *
 ****************************************************************************/

static int ds28e17_selftest(FAR struct ds_i2c_inst_s *inst)
{
  FAR struct onewire_master_s *master = inst->master;
  uint8_t txbuf[] =
  {
    ONEWIRE_CMD_READ_ROM
  };

  uint8_t rxbuf[8] =
  {
    0
  };

  uint64_t rom;
  uint8_t crc;
  int ret;

  /* Read ROM-code of single connected slave and
   * check its checksum.
   */

  ret = ONEWIRE_RESET(master->dev);
  if (ret < 0)
    {
      i2cerr("ERROR: ONEWIRE_RESET failed: %d\n", ret);
      return ret;
    }

  ret = ONEWIRE_WRITE(master->dev, txbuf, sizeof(txbuf));
  if (ret < 0)
    {
      i2cerr("ERROR: ONEWIRE_WRITE failed: %d\n", ret);
      return ret;
    }

  ret = ONEWIRE_READ(master->dev, rxbuf, sizeof(rxbuf));
  if (ret < 0)
    {
      i2cerr("ERROR: ONEWIRE_READ failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_DEBUG_I2C_INFO
  lib_dumpbuffer("ds28e17_selftest: rxbuf", rxbuf, sizeof(rxbuf));

#endif
  memcpy(&rom, rxbuf, 8);
  i2cinfo("recv rom: 0x%llx\n", rom);

  crc = onewire_crc8(rxbuf, sizeof(rxbuf)-1);
  i2cinfo("crc8=%d, recv crc8=%d\n", crc, (int)rxbuf[7]);
  if (crc != rxbuf[7])
    {
      i2cerr("ERROR: crc8 does not match!\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds28e17_search
 *
 * Description:
 *   Search all DS28E17 devices from a 1-wire network.
 *
 * Input Parameters:
 *   priv      - Pointer to the associated DS28E17
 *   cb_search - Callback to call on each device found
 *   arg       - Argument passed to cb_search
 *
 * Return Value:
 *   Number of DS28E17 devices present.
 *
 ****************************************************************************/

int ds28e17_search(FAR struct ds28e17_dev_s *priv,
                   void (*cb_search)(int family,
                                     uint64_t romcode,
                                     void *arg),
                   void *arg)
{
  FAR struct onewire_master_s *master = (FAR struct onewire_master_s *)priv;

  DEBUGASSERT(master != NULL && cb_search != NULL);

  return onewire_search(master, DS_FAMILY, false, cb_search, arg);
}

/****************************************************************************
 * Name: ds28e17_lower_half
 *
 * Description:
 *   Initialize the lower half of the DS28E17 by creating a i2c_master_s
 *   for the virtual i2c master and link it to the associated DS28E17 and
 *   its port.
 *
 * Input Parameters:
 *   dev     - Pointer to the associated DS28E17
 *   romcode - The unique 64-bit address in 1-wire network.
 *             Use zero for skip-ROM mode.
 *
 * Returned Value:
 *   i2c device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
  ds28e17_lower_half(FAR struct ds28e17_dev_s *priv, uint64_t romcode)
{
  FAR struct ds_i2c_inst_s *inst;  /* device, single instance */
  FAR struct onewire_master_s *master = (FAR struct onewire_master_s *)priv;
  int ret;

  DEBUGASSERT(master != NULL);

  /* Allocate instance */

  inst = kmm_zalloc(sizeof(struct ds_i2c_inst_s));
  if (inst == NULL)
    {
      i2cerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  /* Initialize instance */

  inst->ops       = &ds_i2c_ops;
  inst->master    = master;
  inst->frequency = 400000;             /* power-on frequency */
  inst->stretch   = DS_DEFAULT_STRETCH;
  inst->slave.romcode = romcode;

  /* We need a recursive lock as this may be called from a search callback. */

  ret = nxrmutex_lock(&master->devlock);
  if (ret < 0)
    {
      kmm_free(inst);
      i2cerr("ERROR: Failed to acquire lock\n");
      return NULL;
    }

  if (onewire_addslave(master, &inst->slave) < 0)
    {
      kmm_free(inst);
      i2cerr("ERROR: Failed to add slave\n");
      nxrmutex_unlock(&master->devlock);
      return NULL;
    }

  /* Should default speed be different from DS28E17 power-on frequency? */

  if (inst->frequency != DS_DEFAULT_FREQUENCY)
    {
      ds_i2c_setfrequency(inst, DS_DEFAULT_FREQUENCY);
    }

  /* TODO: better selftest */

  if (master->maxslaves == 1)
    {
      ds28e17_selftest(inst);
    }

  nxrmutex_unlock(&master->devlock);
  return (struct i2c_master_s *)inst;
}

/****************************************************************************
 * Name: ds28e17_lower_half_unregister
 *
 * Description:
 *   Put back the lower half of the DS28E17.
 *
 * Input Parameters:
 *   priv    - Pointer to the associated DS28E17
 *   i2cdev  - i2c device instance from ds28e17_lower_half()
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ds28e17_lower_half_unregister(FAR struct ds28e17_dev_s *priv,
                                  FAR struct i2c_master_s *i2cdev)
{
  FAR struct ds_i2c_inst_s *inst = (FAR struct ds_i2c_inst_s *)i2cdev;
  FAR struct onewire_master_s *master = inst->master;
  int ret;

  ret = nxrmutex_lock(&master->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = onewire_removeslave(master, &inst->slave);
  if (ret < 0)
    {
      kmm_free(inst);
      i2cerr("ERROR: Failed to remove slave\n");
      nxrmutex_unlock(&master->devlock);
      return ret;
    }

  kmm_free(inst);
  nxrmutex_unlock(&master->devlock);

  return OK;
}

/****************************************************************************
 * Name: ds28e17_initialize
 *
 * Description:
 *   Returns a common DS28E17 device from 1-wire lower half device
 *
 * Input Parameters:
 *   dev - The allocated 1-wire lower half
 *
 ****************************************************************************/

FAR struct ds28e17_dev_s *ds28e17_initialize(FAR struct onewire_dev_s *dev)
{
  FAR struct onewire_master_s *priv;

  priv = onewire_initialize(dev, DS_DEFAULT_MAXSLAVES);

  /* We do not have our own data fields so just cast it. */

  return (FAR struct ds28e17_dev_s *)priv;
}
