/****************************************************************************
 * drivers/sensors/cxd5602pwbimu.c
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
#include <sys/stat.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/circbuf.h>
#include <nuttx/clock.h>

#include <nuttx/sensors/cxd5602pwbimu.h>

#if defined(CONFIG_SENSORS_CXD5602PWBIMU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Address */

/* Firmware update related registers */

#define CXD5602PWBIMU_INJECTION_READY        (0x01)
#define CXD5602PWBIMU_UPDATE_RESULT          (0x02)
#define CXD5602PWBIMU_INJECT_BINARY          (0x03)
#define CXD5602PWBIMU_VERIFY_BINARY          (0x04)
#define CXD5602PWBIMU_CHANGE_TO_UPDATEMODE   (0x05)

#define CXD5602PWBIMU_FW_VER                 (0x10) /* Firmware version */
#define CXD5602PWBIMU_HW_REVISION            (0x11) /* HW Revision */
#define CXD5602PWBIMU_HW_UNIQUE_ID           (0x12) /* HW UID */
#define CXD5602PWBIMU_FSR                    (0x13) /* Full Scale */
#define CXD5602PWBIMU_ODR                    (0x14) /* Output Data Rate */
#define CXD5602PWBIMU_INTR_ENABLE            (0x15) /* Interrupt enable */
#define CXD5602PWBIMU_FIFO_THRESH            (0x17) /* FIFO threshold */
#define CXD5602PWBIMU_OUTPUT_ENABLE          (0x18) /* Output Enable */
#define CXD5602PWBIMU_USER_CALIB_COEF        (0x19) /* User calibration */
#define CXD5602PWBIMU_USER_CALIB_FLASH       (0x1a) /* Flash user calib value */

#define CXD5602PWBIMU_MODE                   (0xff) /* Output Mode */

/* FSR */

#define FSR_ACCEL_2_G       (0x00 << 4) /* Set ACCEL FullScale +/-2G */
#define FSR_ACCEL_4_G       (0x01 << 4) /* Set ACCEL FullScale +/-4G */
#define FSR_ACCEL_8_G       (0x02 << 4) /* Set ACCEL FullScale +/-8G */
#define FSR_ACCEL_16_G      (0x03 << 4) /* Set ACCEL FullScale +/-16G */

#define FSR_GYRO_125_DPS    (0x00) /* Set GYRO FullScale +/-125dps */
#define FSR_GYRO_250_DPS    (0x01) /* Set GYRO FullScale +/-250dps */
#define FSR_GYRO_500_DPS    (0x02) /* Set GYRO FullScale +/-500dps */
#define FSR_GYRO_1000_DPS   (0x03) /* Set GYRO FullScale +/-1000dps */
#define FSR_GYRO_2000_DPS   (0x04) /* Set GYRO FullScale +/-2000dps */
#define FSR_GYRO_4000_DPS   (0x05) /* Set GYRO FullScale +/-4000dps */

/* ODR configuration */

#define ODR_15HZ   (0x00) /* Set output data rate to 15Hz */
#define ODR_30HZ   (0x01) /* Set output data rate to 30Hz */
#define ODR_60HZ   (0x02) /* Set output data rate to 60Hz */
#define ODR_120HZ  (0x03) /* Set output data rate to 120Hz */
#define ODR_240HZ  (0x04) /* Set output data rate to 240Hz */
#define ODR_480HZ  (0x05) /* Set output data rate to 480Hz */
#define ODR_960HZ  (0x06) /* Set output data rate to 960Hz */
#define ODR_1920HZ (0x07) /* Set output data rate to 1920Hz */

/* Output ENABLE */

#define OUTPUT_DISABLE  (0x00) /* Disable 6axis data output */
#define OUTPUT_ENABLE   (0x01) /* Enable 6axis data output */

/* Update result status */

#define RESULT_NOEXEC   (0x00)
#define RESULT_SUCCESS  (0x01)
#define RESULT_FAILURE  (0x02)

/* I2C Clock Frequency */

#define I2C_CLK_FRERQ   (400000)

/* Default I2C Slave Addresses */

#define I2C_PRIMARY_ADDR0   (0x10)
#define I2C_PRIMARY_ADDR1   (0x11)
#define I2C_PRIMARY_ADDR2   (0x12)
#define I2C_PRIMARY_ADDR3   (0x13)
#define I2C_SECONDARY_ADDR0 (0x30)
#define I2C_SECONDARY_ADDR1 (0x31)
#define I2C_SECONDARY_ADDR2 (0x32)
#define I2C_SECONDARY_ADDR3 (0x33)

#define I2C_ADDR_NOLOC  (0xff)  /* Not located */

#ifdef CONFIG_SENSORS_CXD5602PWBIMU_I2C_ADDRS_SECONDARY
#define I2C_SLAVE_ADDR0 I2C_SECONDARY_ADDR0
#define I2C_SLAVE_ADDR1 I2C_SECONDARY_ADDR1
#define I2C_SLAVE_ADDR2 I2C_SECONDARY_ADDR2
#define I2C_SLAVE_ADDR3 I2C_SECONDARY_ADDR3
#else
#define I2C_SLAVE_ADDR0 I2C_PRIMARY_ADDR0
#define I2C_SLAVE_ADDR1 I2C_PRIMARY_ADDR1
#define I2C_SLAVE_ADDR2 I2C_PRIMARY_ADDR2
#define I2C_SLAVE_ADDR3 I2C_PRIMARY_ADDR3
#endif

/* Driver state */

#define STATE_INIT    (0)
#define STATE_READY   (1)
#define STATE_RUNNING (2)
#define STATE_UPDATE  (3)

#define NR_BUFFERS CONFIG_SENSORS_CXD5602PWBIMU_NR_BUFFERS
#define CIRCBUFSZ(priv) (NR_BUFFERS * (priv)->spi_xfersize)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd5602pwbimu_dev_s
{
  FAR struct spi_dev_s *spi;             /* SPI interface */
  uint32_t              spi_xfersize;    /* SPI TransferSize */

  FAR struct i2c_master_s *i2c;          /* I2C interface */
  uint8_t                  i2caddr[4];   /* I2C slave addresses */
  uint32_t                 i2cfreq;      /* I2C clock frequency */
  int                      nslaves;      /* Number of I2C slaves */

  FAR cxd5602pwbimu_config_t *config;    /* Board control interface */

  mutex_t            devlock;            /* Device exclusion control */
  FAR struct pollfd *fds[CONFIG_SENSORS_CXD5602PWBIMU_NPOLLWAITERS];

  sem_t            dataready;            /* for notify data ready */
  sem_t            bufsem;               /* lock for buffer is in use */
  struct circbuf_s buffer;               /* Store sensing data */
  struct work_s    work;                 /* Retrieve sensing data */
  int              state;                /* Driver state */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t cxd5602pwbimu_spigetreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                        uint8_t regaddr);
static void cxd5602pwbimu_spiputreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                     uint8_t regaddr, uint8_t regval);
static void cxd5602pwbimu_recv(FAR struct cxd5602pwbimu_dev_s *priv,
                               FAR uint8_t *buffer, int len);

static int cxd5602pwbimu_getregsn(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid, uint8_t regaddr,
                                  FAR uint8_t *buffer, uint8_t len);
static int cxd5602pwbimu_getregsn_woadr(FAR struct cxd5602pwbimu_dev_s *priv,
                                        int slaveid,
                                        FAR uint8_t *buffer, uint8_t len);
static int cxd5602pwbimu_putregsn(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid, uint8_t regaddr,
                                  FAR uint8_t *buffer, uint8_t len);
static int cxd5602pwbimu_putregs(FAR struct cxd5602pwbimu_dev_s *priv,
                                 uint8_t regaddr,
                                 FAR uint8_t *buffer, uint8_t len);
static int cxd5602pwbimu_putreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                 uint8_t regaddr, uint8_t regval);
static int cxd5602pwbimu_putreg8n(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid,
                                  uint8_t regaddr, uint8_t regval);

/* Device control methods */

static int cxd5602pwbimu_enable(FAR struct cxd5602pwbimu_dev_s *priv,
                                bool enable);
static int cxd5602pwbimu_setodr(FAR struct cxd5602pwbimu_dev_s *priv,
                                uint32_t rate);
static int cxd5602pwbimu_setcalib(FAR struct cxd5602pwbimu_dev_s *priv,
                                  FAR cxd5602pwbimu_calib_t *calib);
static int cxd5602pwbimu_setdrange(FAR struct cxd5602pwbimu_dev_s *priv,
                                   int accel, int gyro);

/* Firmware update related functions */

static int cxd5602pwbimu_updatemode(FAR struct cxd5602pwbimu_dev_s *priv);
static int cxd5602pwbimu_waitforready(FAR struct cxd5602pwbimu_dev_s *priv,
                                      int slaveid);
static int cxd5602pwbimu_sendfwchunk(FAR struct cxd5602pwbimu_dev_s *priv,
                                     int slaveid, FAR uint8_t *buf, int len);
static int cxd5602pwbimu_verifyfw(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid);
static int cxd5602pwbimu_updatefw(FAR struct cxd5602pwbimu_dev_s *priv,
                                  FAR cxd5602pwbimu_updatefw_t *param);

/* Character driver methods */

static int cxd5602pwbimu_open(FAR struct file *filep);
static int cxd5602pwbimu_close(FAR struct file *filep);
static ssize_t cxd5602pwbimu_read(FAR struct file *filep, FAR char *buffer,
                                  size_t len);
static int cxd5602pwbimu_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);
static int cxd5602pwbimu_poll(FAR struct file *filep,
                              FAR struct pollfd *fds, bool setup);

#ifdef CONFIG_SENSORS_CXD5602PWBIMU_I2C_ADDRS_AUTO
static int cxd5602pwbimu_detectaddrs(FAR struct cxd5602pwbimu_dev_s *priv);
#endif
static int cxd5602pwbimu_checkver(FAR struct cxd5602pwbimu_dev_s *priv);
static int cxd5602pwbimu_int_handler(int irq, FAR void *context,
                                     FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_cxd5602pwbimufops =
{
  cxd5602pwbimu_open,   /* open */
  cxd5602pwbimu_close,  /* close */
  cxd5602pwbimu_read,   /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  cxd5602pwbimu_ioctl,  /* ioctl */
  NULL,                 /* mmap */
  NULL,                 /* truncate */
  cxd5602pwbimu_poll    /* poll */
};

/****************************************************************************
 * Name: cxd5602pwbimu_configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void cxd5602pwbimu_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for CXD5602PWBIMU */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, 1000000);
}

/****************************************************************************
 * Name: cxd5602pwbimu_spigetreg8
 *
 * Description:
 *   Read from an 8-bit CXD5602PWBIMU register
 *
 ****************************************************************************/

static uint8_t cxd5602pwbimu_spigetreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                        uint8_t regaddr)
{
  FAR cxd5602pwbimu_config_t *config = priv->config;
  uint8_t regval[2];

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  cxd5602pwbimu_configspi(priv->spi);

  /* Select the CXD5602PWBIMU */

  /* put the code which control csx pin for CXD5602PWBIMU to low */

  config->csx(config, false);

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, 2);

  /* Deselect the CXD5602PWBIMU */

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), false);

  /* put the code which control csx pin for CXD5602PWBIMU to high */

  config->csx(config, true);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  /* The first byte has to be dropped */

  return regval[1];
}

/****************************************************************************
 * Name: cxd5602pwbimu_spiputreg8
 *
 * Description:
 *   Write a value to an 8-bit CXD5602PWBIMU register
 *
 ****************************************************************************/

static void cxd5602pwbimu_spiputreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                     uint8_t regaddr, uint8_t regval)
{
  FAR cxd5602pwbimu_config_t *config = priv->config;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  cxd5602pwbimu_configspi(priv->spi);

  /* Select the CXD5602PWBIMU */

  /* put the code which control csx pin for CXD5602PWBIMU to low */

  config->csx(config, false);

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the CXD5602PWBIMU */

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), false);

  /* put the code which control csx pin for CXD5602PWBIMU to high */

  config->csx(config, true);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  up_mdelay(1);
}

static void cxd5602pwbimu_recv(FAR struct cxd5602pwbimu_dev_s *priv,
                               FAR uint8_t *buffer, int len)
{
  FAR cxd5602pwbimu_config_t *config = priv->config;

  /* Clear receive buffer and set 1 to first byte MSB */

  memset(buffer, 0, len);
  buffer[0] = 0x80;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  cxd5602pwbimu_configspi(priv->spi);

  /* Select the CXD5602PWBIMU */

  /* Put the code which control CSX pin for CXD5602PWBIMU to low */

  config->csx(config, false);

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), true);

  /* Exchange N bytes */

  SPI_EXCHANGE(priv->spi, buffer, buffer, len);

  /* Deselect the CXD5602PWBIMU */

  SPI_SELECT(priv->spi, SPIDEV_IMU(0), false);

  /* Put the code which control CSX pin for CXD5602PWBIMU to high */

  config->csx(config, true);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: cxd5602pwbimu_getregsn
 *
 * Description:
 *   Read value from CXD5602PWBIMU via I2C
 *
 ****************************************************************************/

static int cxd5602pwbimu_getregsn(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid, uint8_t regaddr,
                                  FAR uint8_t *buffer, uint8_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->i2cfreq;
  msg[0].addr      = priv->i2caddr[slaveid];
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->i2cfreq;
  msg[1].addr      = priv->i2caddr[slaveid];
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_getregsn_woadr
 *
 * Description:
 *   Read value from CXD5602PWBIMU via I2C with primary slave
 *   This function performs a read operation without a preceding
 *   write operation to the register address.
 ****************************************************************************/

static int cxd5602pwbimu_getregsn_woadr(FAR struct cxd5602pwbimu_dev_s *priv,
                                         int slaveid,
                                         FAR uint8_t *buffer, uint8_t len)
{
  struct i2c_msg_s msg;
  int ret;

  msg.frequency = priv->i2cfreq;
  msg.addr      = priv->i2caddr[slaveid];
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = len;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_putregsn
 *
 * Description:
 *   Write value to CXD5602PWBIMU via i2c.
 *
 ****************************************************************************/

static int cxd5602pwbimu_putregsn(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid, uint8_t regaddr,
                                  FAR uint8_t *buffer, uint8_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->i2cfreq;
  msg[0].addr      = priv->i2caddr[slaveid];
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->i2cfreq;
  msg[1].addr      = priv->i2caddr[slaveid];
  msg[1].flags     = 0;
  msg[1].buffer    = buffer;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_putregs
 *
 * Description:
 *   Write value to CXD5602PWBIMU via i2c with primary slave.
 *
 ****************************************************************************/

static int cxd5602pwbimu_putregs(FAR struct cxd5602pwbimu_dev_s *priv,
                                 uint8_t regaddr,
                                 FAR uint8_t *buffer, uint8_t len)
{
  return cxd5602pwbimu_putregsn(priv, 0, regaddr, buffer, len);
}

/****************************************************************************
 * Name: cxd5602pwbimu_putreg8
 *
 * Description:
 *   Write value to CXD5602PWBIMU via i2c with primary slave.
 *
 ****************************************************************************/

static int cxd5602pwbimu_putreg8(FAR struct cxd5602pwbimu_dev_s *priv,
                                 uint8_t regaddr, uint8_t regval)
{
  return cxd5602pwbimu_putregsn(priv, 0, regaddr, &regval, 1);
}

/****************************************************************************
 * Name: cxd5602pwbimu_putreg8n
 *
 * Description:
 *   Write value to CXD5602PWBIMU via i2c with specified slave address.
 *
 ****************************************************************************/

static int cxd5602pwbimu_putreg8n(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid,
                                  uint8_t regaddr, uint8_t regval)
{
  return cxd5602pwbimu_putregsn(priv, slaveid, regaddr, &regval, 1);
}

#ifdef CONFIG_SENSORS_CXD5602PWBIMU_I2C_ADDRS_AUTO
static int cxd5602pwbimu_detectaddrs(FAR struct cxd5602pwbimu_dev_s *priv)
{
  uint8_t val;
  int ret;

  if (priv->nslaves)
    {
      return 0;
    }

  /* First try to get register from primary PSoC.
   * If an address is determined to be either primary or secondary, the
   * other addresses are set in the same series for now.
   */

  priv->i2caddr[0] = I2C_PRIMARY_ADDR0;
  ret = cxd5602pwbimu_getregsn(priv, 0, CXD5602PWBIMU_FW_VER, &val, 1);
  if (ret)
    {
      priv->i2caddr[0] = I2C_SECONDARY_ADDR0;
      ret = cxd5602pwbimu_getregsn(priv, 0, CXD5602PWBIMU_FW_VER, &val, 1);
      if (ret)
        {
          /* If no response from primary and secondary address, the device
           * not found.
           */

          return -ENODEV;
        }

      priv->i2caddr[1] = I2C_SECONDARY_ADDR1;
      priv->i2caddr[2] = I2C_SECONDARY_ADDR2;
      priv->i2caddr[3] = I2C_SECONDARY_ADDR3;
    }
  else
    {
      priv->i2caddr[1] = I2C_PRIMARY_ADDR1;
      priv->i2caddr[2] = I2C_PRIMARY_ADDR2;
      priv->i2caddr[3] = I2C_PRIMARY_ADDR3;
    }

  sninfo("Detected address 0x%02x\n", priv->i2caddr[0]);

  return 0;
}
#endif

static int cxd5602pwbimu_checkaddrs(FAR struct cxd5602pwbimu_dev_s *priv)
{
  int i;
  uint8_t val;
  int nslaves;
  int ret;

  /* Counting available PSoCs */

  nslaves = 0;
  for (i = 0; i < 4; i++)
    {
      ret = cxd5602pwbimu_getregsn(priv, i, CXD5602PWBIMU_FW_VER, &val, 1);
      if (ret)
        {
          priv->i2caddr[i] = I2C_ADDR_NOLOC;
        }
      else
        {
          nslaves++;
        }
    }

  if (nslaves == 0)
    {
      return -ENODEV;
    }

  priv->nslaves = nslaves;

  /* Check that the address dip switches are valid. */

  for (i = 2; i < 4; i++)
    {
      if (priv->i2caddr[i] == I2C_ADDR_NOLOC)
        {
          priv->i2caddr[i] = ((priv->i2caddr[i - 2] & 0x30) ^ 0x20) + i;
          ret = cxd5602pwbimu_getregsn(priv, i, CXD5602PWBIMU_FW_VER,
                                       &val, 1);
          priv->i2caddr[i] = I2C_ADDR_NOLOC;
          if (ret == 0)
            {
              snerr("Dip switch setting mismatch detected.\n");
              return -EFAULT;
            }
        }
    }

  sninfo("%d slaves are detected.\n", priv->nslaves);

  return 0;
}

/****************************************************************************
 * Name: cxd5602pwbimu_checkver
 *
 * Description:
 *   Read and verify the CXD5602PWBIMU firmware versions
 *
 ****************************************************************************/

static int cxd5602pwbimu_checkver(FAR struct cxd5602pwbimu_dev_s *priv)
{
  uint8_t ver[4];
  uint8_t val = 0;
  int ret;
  int i;

  ret = cxd5602pwbimu_getregsn(priv, 0, CXD5602PWBIMU_MODE, &val, 1);
  if (ret != OK)
    {
      return -1;
    }

  if (val == 0)
    {
      /* Now in update mode */

      return 1;
    }

  /* Read and verify 4 firmware versions.
   *
   * 1 add-on board has 2 chips, and this add-on board can stack up to
   * 2 add-on boards, so maximum chips are 4.
   * They must have the same firmware version.
   */

  memset(ver, 0, sizeof(ver));
  for (i = 0; i < priv->nslaves; i++)
    {
      ret = cxd5602pwbimu_getregsn(priv, i, CXD5602PWBIMU_FW_VER, &val, 1);
      if (ret == 0)
        {
          ver[i] = val;
          sninfo("[%d]  Version: %02x\n", i, val);
        }
    }

  if (ver[0] == 0)
    {
      /* If primary chip not found, the board not connected. */

      return -1;
    }

  if (ver[0] != ver[1])
    {
      /* Primary and secondary firmwares are not matched, need to update */

      return 1;
    }

  if (priv->nslaves > 2)
    {
      if (ver[0] != ver[2] || ver[0] != ver[3])
        {
          /* 2nd board firmware does not matched with primary firmware,
           * need to update.
           */

          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: cxd5602pwbimu_enable
 *
 * Description:
 *   Start sensing
 *
 ****************************************************************************/

static int cxd5602pwbimu_enable(FAR struct cxd5602pwbimu_dev_s *priv,
                                bool enable)
{
  int ret = OK;

  if (priv->state == STATE_UPDATE)
    {
      return -EBUSY;
    }

  if (enable)
    {
      if (priv->state == STATE_RUNNING)
        {
          return OK; /* Already running, ignore. */
        }

      ret = cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_OUTPUT_ENABLE,
                                  OUTPUT_ENABLE);
      if (!ret)
        {
          priv->state = STATE_RUNNING;
        }
    }
  else
    {
      if (priv->state == STATE_READY)
        {
          return OK; /* Already stopped, ignore. */
        }

      cxd5602pwbimu_spiputreg8(priv, CXD5602PWBIMU_OUTPUT_ENABLE,
                               OUTPUT_DISABLE);
      priv->state = STATE_READY;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_setodr
 *
 * Description:
 *   Set sampling rate
 *
 ****************************************************************************/

static int cxd5602pwbimu_setodr(FAR struct cxd5602pwbimu_dev_s *priv,
                                uint32_t rate)
{
  uint8_t val;

  if (priv->state != STATE_READY)
    {
      return -EBUSY;
    }

  switch (rate)
    {
      case 15:
        val = ODR_15HZ;
        break;

      case 30:
        val = ODR_30HZ;
        break;

      case 60:
        val = ODR_60HZ;
        break;

      case 120:
        val = ODR_120HZ;
        break;

      case 240:
        val = ODR_240HZ;
        break;

      case 480:
        val = ODR_480HZ;
        break;

      case 960:
        val = ODR_960HZ;
        break;

      case 1920:
        val = ODR_1920HZ;
        break;

      default:
        return -EINVAL;
    }

  return cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_ODR, val);
}

/****************************************************************************
 * Name: cxd5602pwbimu_setcalib
 *
 * Description:
 *   Set calibration
 *
 ****************************************************************************/

static int cxd5602pwbimu_setcalib(FAR struct cxd5602pwbimu_dev_s *priv,
                                  FAR cxd5602pwbimu_calib_t *calib)
{
  int ret;

  if (priv->state != STATE_READY)
    {
      return -EBUSY;
    }

  ret = cxd5602pwbimu_putregs(priv, CXD5602PWBIMU_USER_CALIB_COEF,
                              (FAR uint8_t *)calib,
                              sizeof(cxd5602pwbimu_calib_t));
  if (ret == 0)
    {
      ret = cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_USER_CALIB_FLASH, 1);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_setdrange
 *
 * Description:
 *   Set dynamic range setting for accelerometer and gyro.
 *
 ****************************************************************************/

static int cxd5602pwbimu_setdrange(FAR struct cxd5602pwbimu_dev_s *priv,
                                   int accel, int gyro)
{
  int val;

  if (priv->state != STATE_READY)
    {
      return -EBUSY;
    }

  switch (accel)
    {
      case 2:
        val = FSR_ACCEL_2_G;
        break;

      case 4:
        val = FSR_ACCEL_4_G;
        break;

      case 8:
        val = FSR_ACCEL_8_G;
        break;

      case 16:
        val = FSR_ACCEL_16_G;
        break;

      default:
        return -EINVAL;
    }

  switch (gyro)
    {
      case 125:
        val |= FSR_GYRO_125_DPS;
        break;

      case 250:
        val |= FSR_GYRO_250_DPS;
        break;

      case 500:
        val |= FSR_GYRO_500_DPS;
        break;

      case 1000:
        val |= FSR_GYRO_1000_DPS;
        break;

      case 2000:
        val |= FSR_GYRO_2000_DPS;
        break;

      case 4000:
        val |= FSR_GYRO_4000_DPS;
        break;

      default:
        return -EINVAL;
    }

  return cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_FSR, val);
}

/****************************************************************************
 * Name: cxd5602pwbimu_setfifothresh
 *
 * Description:
 *   Set FIFO threshold. Driver resize the circbuf by configured
 *   threshold.
 *
 ****************************************************************************/

static int cxd5602pwbimu_setfifothresh(FAR struct cxd5602pwbimu_dev_s *priv,
                                       int thresh)
{
  size_t size;
  int ret;

  if (priv->state != STATE_READY)
    {
      return -EBUSY;
    }

  if (thresh < 1 || thresh > 4)
    {
      return -EINVAL;
    }

  ret = cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_FIFO_THRESH, thresh);
  if (!ret)
    {
      priv->spi_xfersize = sizeof(cxd5602pwbimu_data_t) * thresh;
      size = (NR_BUFFERS / thresh) * thresh * sizeof(cxd5602pwbimu_data_t);
      sninfo("Resize circbuf in %d bytes\n", size);
      ret = circbuf_resize(&priv->buffer, size);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_updatemode
 *
 * Description:
 *   Change CXD5602PWBIMU mode to update mode
 *
 ****************************************************************************/

static int cxd5602pwbimu_updatemode(FAR struct cxd5602pwbimu_dev_s *priv)
{
  int i;
  int ret;
  uint8_t val;

  ret = cxd5602pwbimu_getregsn(priv, 0, CXD5602PWBIMU_MODE, &val, 1);
  if (ret)
    {
      return ret;
    }

  if (val == 0)
    {
      /* Already in update mode */

      return 0;
    }

  for (i = 0; i < priv->nslaves; i++)
    {
      ret = cxd5602pwbimu_putreg8n(priv, i,
                                   CXD5602PWBIMU_CHANGE_TO_UPDATEMODE, 1);
      if (ret)
        {
          return ret;
        }
    }

  up_mdelay(100);

  return 0;
}

/****************************************************************************
 * Name: cxd5602pwbimu_waitforready
 *
 * Description:
 *   Wait for ready to updater can take the firmware binary data.
 *
 ****************************************************************************/

static int cxd5602pwbimu_waitforready(FAR struct cxd5602pwbimu_dev_s *priv,
                                      int slaveid)
{
  int retry;
  uint8_t val;
  int ret;

  val = 0;
  retry = 1000;
  do
    {
      ret = cxd5602pwbimu_getregsn(priv, slaveid,
                                   CXD5602PWBIMU_INJECTION_READY,
                                   &val, 1);
      if (ret)
        {
          return ret;
        }
    }
  while (val == 0 && --retry);

  return val == 1 ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: cxd5602pwbimu_sendfwchunk
 *
 * Description:
 *   Send chunk of CXD5602PWBIMU Add-on board firmware.
 *
 ****************************************************************************/

static int cxd5602pwbimu_sendfwchunk(FAR struct cxd5602pwbimu_dev_s *priv,
                                     int slaveid, FAR uint8_t *buf, int len)
{
  int ret;

  ret = cxd5602pwbimu_waitforready(priv, slaveid);
  if (ret)
    {
      return ret;
    }

  ret = cxd5602pwbimu_putregsn(priv, slaveid, CXD5602PWBIMU_INJECT_BINARY,
                               buf, len);

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_verifyfw
 *
 * Description:
 *   Verify updated CXD5602PWBIMU Add-on board firmwares.
 *
 ****************************************************************************/

static int cxd5602pwbimu_verifyfw(FAR struct cxd5602pwbimu_dev_s *priv,
                                  int slaveid)
{
  int retry;
  uint8_t val;
  int ret;

  ret = cxd5602pwbimu_waitforready(priv, slaveid);
  if (ret)
    {
      return ret;
    }

  val = 1;
  ret = cxd5602pwbimu_putregsn(priv, slaveid, CXD5602PWBIMU_VERIFY_BINARY,
                               &val, 1);
  if (ret)
    {
      return ret;
    }

  val = 0;
  retry = 1000;
  do
    {
      ret = cxd5602pwbimu_getregsn(priv, slaveid,
                                   CXD5602PWBIMU_UPDATE_RESULT,
                                   &val, 1);
      if (ret)
        {
          return ret;
        }
    }
  while (val == RESULT_NOEXEC && --retry);

  if (val == RESULT_SUCCESS)
    {
      ret = OK;
    }
  else if (val == RESULT_FAILURE)
    {
      ret = -EFAULT;
    }
  else
    {
      ret = -ETIMEDOUT;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_updatefw
 *
 * Description:
 *   Update CXD5602PWBIMU Add-on board firmwares.
 *
 ****************************************************************************/

static int cxd5602pwbimu_updatefw(FAR struct cxd5602pwbimu_dev_s *priv,
                                  FAR cxd5602pwbimu_updatefw_t *param)
{
  struct file finfo;
  struct stat stat;
  FAR uint8_t *buf;
  off_t size;
  off_t total;
  int i;
  int len;
  int ret;

  priv->state = STATE_UPDATE;

  buf = (FAR uint8_t *)kmm_malloc(128);
  if (!buf)
    {
      return -ENOMEM;
    }

  ret = file_open(&finfo, param->path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      kmm_free(buf);
      return ret;
    }

  ret = file_fstat(&finfo, &stat);
  if (ret < 0)
    {
      goto errout;
    }

  total = stat.st_size;

  sninfo("Change update mode\n");
  ret = cxd5602pwbimu_updatemode(priv);
  if (ret)
    {
      goto errout;
    }

  size = 0;
  sninfo("Updating");
  for (; ; )
    {
      ret = file_read(&finfo, buf, 128);
      if (ret <= 0)
        {
          /* break loop when error or EOF */

          break;
        }

      len = ret;

      for (i = 0; i < priv->nslaves; i++)
        {
          sninfo("Send %d bytes to %d ", len, i);
          ret = cxd5602pwbimu_sendfwchunk(priv, i, buf, len);
          if (ret)
            {
              goto errout;
            }

          sninfo("OK\n");
        }

      /* Inform current progress to caller */

      size += len;
      if (param->progress)
        {
          param->progress(size, total);
        }
    }

  sninfo("Verifying firmwares");

  for (i = 0; i < priv->nslaves; i++)
    {
      ret = cxd5602pwbimu_verifyfw(priv, i);
      if (ret)
        {
          break;
        }
    }

  sninfo("OK\n");

errout:
  kmm_free(buf);
  file_close(&finfo);

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int cxd5602pwbimu_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv = inode->i_private;
  FAR cxd5602pwbimu_config_t *config = priv->config;
  int ret;

  /* Power on and reset device */

  config->power(config, true);
  config->reset(config, true);
  up_udelay(20);
  config->reset(config, false);
  up_mdelay(150);

#ifdef CONFIG_SENSORS_CXD5602PWBIMU_I2C_ADDRS_AUTO
  ret = cxd5602pwbimu_detectaddrs(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = cxd5602pwbimu_checkaddrs(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = cxd5602pwbimu_checkver(priv);
  if (ret < 0)
    {
      return -ENODEV;
    }

  if (ret > 0)
    {
      /* If return value is positive, firmware mismatch has been detected.
       * The driver can be used for only updating firmwares.
       * Additionally, change each PSoCs to update mode for indicate on board
       * LEDs to in update mode.
       */

      cxd5602pwbimu_updatemode(priv);
      priv->state = STATE_UPDATE;

      return OK;
    }

  circbuf_init(&priv->buffer, NULL, CIRCBUFSZ(priv));

  /* Enable data ready interrupt */

  cxd5602pwbimu_putreg8(priv, CXD5602PWBIMU_INTR_ENABLE, 1);
  config->irq_attach(config, cxd5602pwbimu_int_handler, priv);
  config->irq_enable(config, true);

  priv->state = STATE_READY;

  return OK;
}

/****************************************************************************
 * Name: cxd5602pwbimu_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int cxd5602pwbimu_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv = inode->i_private;
  FAR cxd5602pwbimu_config_t *config = priv->config;

  /* Stop output 6axis data and power down */

  config->irq_enable(config, false);
  cxd5602pwbimu_enable(priv, false);

  up_mdelay(100);

  config->reset(config, true);

  if (circbuf_is_init(&priv->buffer))
    {
      circbuf_uninit(&priv->buffer);
    }

  priv->state = STATE_INIT;

  return OK;
}

/****************************************************************************
 * Name: cxd5602pwbimu_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t cxd5602pwbimu_read(FAR struct file *filep, FAR char *buffer,
                                  size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (circbuf_is_empty(&priv->buffer))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&priv->devlock);
          return -EAGAIN;
        }

      nxmutex_unlock(&priv->devlock);
      ret = nxsem_wait_uninterruptible(&priv->dataready);
      if (ret)
        {
          return ret;
        }

      nxmutex_lock(&priv->devlock);
    }

  ret = nxsem_wait_uninterruptible(&priv->bufsem);
  if (ret)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }

  ret = circbuf_read(&priv->buffer, buffer, len);
  nxsem_post(&priv->bufsem);

  nxmutex_unlock(&priv->devlock);

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int cxd5602pwbimu_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cxd5602pwbimu_dev_s *priv  = inode->i_private;
  int ret = OK;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->state == STATE_RUNNING && cmd != SNIOC_ENABLE)
    {
      nxmutex_unlock(&priv->devlock);
      return -EBUSY;
    }

  switch (cmd)
    {
      case SNIOC_SETDATASIZE:
        if (priv->state != STATE_READY)
          {
            ret = -EBUSY;
          }
        else
          {
            priv->spi_xfersize = arg;
            circbuf_resize(&priv->buffer, CIRCBUFSZ(priv));
          }
        break;

      case SNIOC_SSAMPRATE:
        {
          uint32_t rate = (uint32_t)arg;

          ret = cxd5602pwbimu_setodr(priv, rate);
        }
        break;

      case SNIOC_SDRANGE:
        {
          FAR cxd5602pwbimu_range_t *r = (FAR cxd5602pwbimu_range_t *)arg;

          ret = cxd5602pwbimu_setdrange(priv, r->accel, r->gyro);
        }
        break;

      case SNIOC_SCALIB:
        {
          FAR cxd5602pwbimu_calib_t *c = (FAR cxd5602pwbimu_calib_t *)arg;

          ret = cxd5602pwbimu_setcalib(priv, c);
        }
        break;

      case SNIOC_ENABLE:
        ret = cxd5602pwbimu_enable(priv, arg == 1);
        break;

      case SNIOC_SFIFOTHRESH:
        ret = cxd5602pwbimu_setfifothresh(priv, arg);
        break;

      case SNIOC_UPDATEFW:
        {
          FAR cxd5602pwbimu_updatefw_t *p =
            (FAR cxd5602pwbimu_updatefw_t *)arg;

          ret = cxd5602pwbimu_updatefw(priv, p);
        }
        break;

      case SNIOC_WREGSPI:
        {
          FAR cxd5602pwbimu_regs_t *p = (FAR cxd5602pwbimu_regs_t *)arg;

          cxd5602pwbimu_spiputreg8(priv, p->addr, *p->value);
        }
        break;

      case SNIOC_RREGSPI:
        {
          FAR cxd5602pwbimu_regs_t *p = (FAR cxd5602pwbimu_regs_t *)arg;

          p->value[0] = cxd5602pwbimu_spigetreg8(priv, p->addr);
        }
        break;

      case SNIOC_WREGS:
        {
          FAR cxd5602pwbimu_regs_t *p = (FAR cxd5602pwbimu_regs_t *)arg;

          ret = cxd5602pwbimu_putregsn(priv, p->slaveid,
                                       p->addr, p->value, p->len);
        }
        break;

      case SNIOC_RREGS:
        {
          FAR cxd5602pwbimu_regs_t *p = (FAR cxd5602pwbimu_regs_t *)arg;

          ret = cxd5602pwbimu_getregsn(priv, p->slaveid,
                                       p->addr, p->value, p->len);
        }
        break;

      case SNIOC_RREGS_WOADR:
        {
          FAR cxd5602pwbimu_regs_t *p = (FAR cxd5602pwbimu_regs_t *)arg;

          ret = cxd5602pwbimu_getregsn_woadr(priv, p->slaveid,
                                             p->value, p->len);
        }
        break;

      case SNIOC_GETBNUM:
        {
          ret = priv->nslaves == 2 ? 1 :
                priv->nslaves == 4 ? 2 : 0;
        }
        break;

      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);

  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_poll
 *
 * Description:
 *   Polling method for CXD5602PWBIMU data ready
 *
 ****************************************************************************/

static int cxd5602pwbimu_poll(FAR struct file *filep, FAR struct pollfd *fds,
                              bool setup)
{
  FAR struct inode *inode;
  FAR struct cxd5602pwbimu_dev_s *priv;
  uint32_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_SENSORS_CXD5602PWBIMU_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_SENSORS_CXD5602PWBIMU_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      flags = enter_critical_section();
      if (!circbuf_is_empty(&priv->buffer))
        {
          poll_notify(priv->fds,
                      CONFIG_SENSORS_CXD5602PWBIMU_NPOLLWAITERS, POLLIN);
        }

      leave_critical_section(flags);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: cxd5602pwbimu_worker
 *
 * Description:
 *   Work queue function for retrieve sensing data.
 *
 ****************************************************************************/

static void cxd5602pwbimu_worker(FAR void *arg)
{
  FAR struct cxd5602pwbimu_dev_s *priv =
    (FAR struct cxd5602pwbimu_dev_s *)arg;
  FAR cxd5602pwbimu_config_t *config = priv->config;
  FAR void *ptr;
  size_t size;
  int ret;

  /* 500us is the maximum sampling rate */

  ret = nxsem_tickwait_uninterruptible(&priv->bufsem, USEC2TICK(500));
  if (ret)
    {
      snerr("ERROR: Data buffer is locked too long time.\n");
      return;
    }

  /* Receive 1 sensing data.
   * If two or more data are ready, re-enter this routine after interrupt
   * enable, and continue this sequence until sensing data on the device
   * is empty.
   */

#ifndef CONFIG_SENSORS_CXD5602PWBIMU_OVERWRITE
  if (circbuf_is_full(&priv->buffer))
    {
      cxd5602pwbimu_data_t data;

      /* Drain sensor data but not copy into circular buffer */

      cxd5602pwbimu_recv(priv, (FAR uint8_t *)&data, priv->spi_xfersize);
    }
  else
#endif
    {
#ifdef CONFIG_SENSORS_CXD5602PWBIMU_OVERWRITE
      if (circbuf_is_full(&priv->buffer))
        {
          /* Advance the read pointer by the transfer size.
           * We need to do it for overwrite feature in circbuf.
           */

          circbuf_readcommit(&priv->buffer, priv->spi_xfersize);
        }
#endif

      ptr = circbuf_get_writeptr(&priv->buffer, &size);
      cxd5602pwbimu_recv(priv, ptr, priv->spi_xfersize);
      circbuf_writecommit(&priv->buffer, priv->spi_xfersize);
    }

  nxsem_post(&priv->bufsem);

  config->irq_enable(config, true);

  /* Notify data get ready to user */

  nxsem_post(&priv->dataready);
  poll_notify(priv->fds, CONFIG_SENSORS_CXD5602PWBIMU_NPOLLWAITERS, POLLIN);
}

/****************************************************************************
 * Name: cxd5602pwbimu_int_handler
 *
 * Description:
 *   Interrupt handler for CXD5602PWBIMU data ready
 *
 ****************************************************************************/

static int cxd5602pwbimu_int_handler(int irq, FAR void *context,
                                     FAR void *arg)
{
  FAR struct cxd5602pwbimu_dev_s *priv =
    (FAR struct cxd5602pwbimu_dev_s *)arg;
  FAR cxd5602pwbimu_config_t *config = priv->config;
  int ret;

  config->irq_enable(config, false);

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, cxd5602pwbimu_worker, priv, 0);
      if (ret < 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd5602pwbimu_register
 *
 * Description:
 *   Register the CXD5602PWBIMU character device as 'devpath'
 *
 * Input Parameters:
 *   devpath   - The full path to the driver to register. E.g., "/dev/imu0"
 *   dev_spi   - An instance of the SPI interface to use to communicate
 *               with CXD5602PWBIMU
 *   dev_i2c   - An instance of the I2C interface to use to communicate
 *               with CXD5602PWBIMU
 *   config    - An instance of the interrupt configuration data structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cxd5602pwbimu_register(FAR const char *devpath,
                           FAR struct spi_dev_s *dev_spi,
                           FAR struct i2c_master_s *dev_i2c,
                           FAR cxd5602pwbimu_config_t *config)
{
  FAR struct cxd5602pwbimu_dev_s *priv;
  size_t size = sizeof(struct cxd5602pwbimu_dev_s);
  int ret;

  priv = (FAR struct cxd5602pwbimu_dev_s *)kmm_zalloc(size);
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi = dev_spi;
  priv->spi_xfersize = sizeof(cxd5602pwbimu_data_t);

  priv->i2c = dev_i2c;
  priv->i2cfreq = I2C_CLK_FRERQ;
  priv->i2caddr[0] = I2C_SLAVE_ADDR0;
  priv->i2caddr[1] = I2C_SLAVE_ADDR1;
  priv->i2caddr[2] = I2C_SLAVE_ADDR2;
  priv->i2caddr[3] = I2C_SLAVE_ADDR3;
  priv->nslaves = 0;
  priv->state = STATE_INIT;

  priv->config = config;
  nxmutex_init(&priv->devlock);

  nxsem_init(&priv->dataready, 0, 0);
  nxsem_init(&priv->bufsem, 0, 1);

  ret = register_driver(devpath, &g_cxd5602pwbimufops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("CXD5602PWBIMU driver loaded successfully!\n");
  return OK;
}

#endif /* CONFIG_SENSORS_CXD5602PWBIMU */
