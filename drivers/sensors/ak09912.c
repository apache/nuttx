/****************************************************************************
 * drivers/sensors/ak09912.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <arch/types.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/ak09912.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AK09912)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AK09911_DEVID       0x05
#define AK09912_DEVID       0x04
#define AK09912_ADDR        0x0C
#define AK09912_FREQ        400000

/* REGISTER: WIA
 * Who I am.
 */

#define AK09912_WIA1       0x00
#define AK09912_WIA2       0x01

/* REGISTER: CNTL2
 * Power mode
 */

#define POWER_MODE_ADDR     0x31

/* REGISTER: ASAX
 * Sensitivity values
 */

#define AK09912_ASAX        0x60

/* REGISTER: CNTL1
 * Enable or disable temparator measure or enable or disable Noise
 * suppression filter.
 */

#define AK09912_CTRL1       0x30

/* REGISTER: HXL
 * The start address of data registers.
 */

#define AK09912_HXL         0x11

/* Polling timeout
 * The unit is 10 millisecond.
 */

#define AK09912_POLLING_TIMEOUT (1)  // 10 ms

/* The parameter for compensating. */

#define AK09912_SENSITIVITY               (128)
#define AK09912_SENSITIVITY_DIV           (256)

/* Noise Suppression Filter */

#define AK09912_NSF_NONE                  0b00
#define AK09912_NSF_LOW                   0b01
#define AK09912_NSF_MIDDLE                0b10
#define AK09912_NSF_HIGH                  0b11

/* Power mode */

#define AKM_POWER_DOWN_MODE                0b0000
#define AKM_SINGL_MEAS_MODE                0b00001
#define AKM_CONT_MEAS_1                    0b00010
#define AKM_CONT_MEAS_2                    0b00100
#define AKM_CONT_MEAS_3                    0b00110
#define AKM_CONT_MEAS_4                    0b01000
#define AKM_EXT_TRIG_MODE                  0b01010
#define AKM_SELF_TEST_MODE                 0b10000
#define AKM_FUSE_ROM_MODE                  0b11111

/* REGISTER: ST1
 * DRDY: Data ready bit. 0: not ready, 1: ready
 * DOR: Data overrun. 0: Not overrun, 1: overrun
 */

#define AK09912_ST1 0x10

#define SET_BITSLICE(s, v, offset, mask) \
do {                                     \
    s &= mask;                           \
    s |= (v << offset) & mask;} while(0)

#define MERGE_BYTE(low, high) \
    ((low & 0xff) | ((high << 8) & ~0xff))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for compensating data. */

struct sensi_data_s
{
  uint8_t x;
  uint8_t y;
  uint8_t z;
};

/* Structure for ak09912 device */

struct ak09912_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /*  I2C address */
  int freq;                     /* Frequency <= 3.4MHz */
  int compensated;              /* 0: uncompensated, 1:compensated */
  struct sensi_data_s asa_data; /* sensitivity data */
  uint8_t mode;                 /* power mode */
  uint8_t nsf;                  /* noise suppression filter setting */
  struct wdog_s wd;
  sem_t wait;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int ak09912_open(FAR struct file *filep);
static int ak09912_close(FAR struct file *filep);
static ssize_t ak09912_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ak09912_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int ak09912_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ak09912fops =
{
  ak09912_open,                  /* open */
  ak09912_close,                 /* close */
  ak09912_read,                  /* read */
  ak09912_write,                 /* write */
  0,                             /* seek */
  ak09912_ioctl,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                             /* poll */
#endif
  0                              /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ak09912_getreg8
 *
 * Description:
 *   Read from an 8-bit BMP280 register
 *
 ****************************************************************************/

static uint8_t ak09912_getreg8(FAR struct ak09912_dev_s *priv,
                               uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: ak09912_putreg8
 *
 * Description:
 *   Write to an 8-bit BMP280 register
 *
 ****************************************************************************/

static int ak09912_putreg8(FAR struct ak09912_dev_s *priv,
                           uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09912_getreg
 *
 * Description:
 *   Read cnt bytes from a ak09912 register
 *
 ****************************************************************************/

static int32_t ak09912_getreg(FAR struct ak09912_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t *buffer,
                              uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = cnt;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ak09912_delay_msec
 *
 * Description:
 *   System level delay
 *
 ****************************************************************************/

void  ak09912_delay_msek(uint16_t msek)
{
  /* This is used to short delay without schedule. */

  up_udelay(msek * 1000);
}

/****************************************************************************
 * Name: ak09912_set_power_mode
 *
 * Description:
 *   set POWER MODE for ak09912
 *
 ****************************************************************************/

static int ak09912_set_power_mode(FAR struct ak09912_dev_s *priv,
                                  uint32_t mode)
{
  int ret = 0;
  ret = ak09912_putreg8(priv, POWER_MODE_ADDR, (uint8_t)mode);
  ak09912_delay_msek(1);
  return ret;
}

/****************************************************************************
 * Name: ak09912_read_sensitivity_data
 *
 * Description:
 *   read sensitivity date in fuse mode.
 *
 ****************************************************************************/

static int ak09912_read_sensitivity_data(FAR struct ak09912_dev_s *priv,
                                         FAR struct sensi_data_s *asa_data)
{
  int ret = 0;
  uint8_t buffer[3];

  ret = ak09912_getreg(priv, AK09912_ASAX, buffer, sizeof(buffer));
  if (ret == 0)
    {
      asa_data->x = buffer[0];
      asa_data->y = buffer[1];
      asa_data->z = buffer[2];
    }

  return ret;
}

/****************************************************************************
 * Name: ak09912_set_noise_suppr_flt
 *
 * Description:
 *   set noise suppression filter for ak09912
 *
 ****************************************************************************/

static int ak09912_set_noise_suppr_flt(FAR struct ak09912_dev_s *priv,
                                       uint32_t nsf)
{
  int ret = 0;
  uint8_t ctrl1 = 0;

  ctrl1 = ak09912_getreg8(priv, AK09912_CTRL1);
  SET_BITSLICE(ctrl1, nsf, 5, 0x60);
  ret = ak09912_putreg8(priv, AK09912_CTRL1, ctrl1);

  return ret;
}

/****************************************************************************
 * Name: ak09912_wd_timeout
 *
 * Description:
 *   a timer to check if data is ready.
 *
 ****************************************************************************/

static void ak09912_wd_timeout(wdparm_t arg)
{
  struct ak09912_dev_s *priv = (struct ak09912_dev_s *)arg;
  irqstate_t flags = enter_critical_section();
  nxsem_post(&priv->wait);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ak09912_read_mag_uncomp_data
 *
 * Description:
 *   read mag uncompensating data.
 *
 ****************************************************************************/

static int ak09912_read_mag_uncomp_data(FAR struct ak09912_dev_s *priv,
                                        FAR struct mag_data_s *mag_data)
{
  int ret = 0;
  uint8_t state = 0;
  uint8_t buffer[8];  /* TMPS and ST2 is read, but the value is omitted. */

  wd_start(&priv->wd, AK09912_POLLING_TIMEOUT,
           ak09912_wd_timeout, (wdparm_t)priv);
  state = ak09912_getreg8(priv, AK09912_ST1);
  while (! (state & 0x1))
    {
      nxsem_wait(&priv->wait);
    }

  wd_cancel(&priv->wd);
  ret = ak09912_getreg(priv,  AK09912_HXL,  buffer, sizeof(buffer));

  mag_data->x = MERGE_BYTE(buffer[0], buffer[1]);
  mag_data->y = MERGE_BYTE(buffer[2], buffer[3]);
  mag_data->z = MERGE_BYTE(buffer[4], buffer[5]);

  return ret;
}

/****************************************************************************
 * Name: ak09912_read_mag_data
 *
 * Description:
 *   read mag data with compensation
 *
 ****************************************************************************/

static int ak09912_read_mag_data(FAR struct ak09912_dev_s *priv,
                                 FAR struct mag_data_s *mag_data)
{
  int ret = 0;

  ret = ak09912_read_mag_uncomp_data(priv, mag_data);
  if (ret < 0)
    {
      snerr("Failed to read mag data from device.\n");
      return ret;
    }

  mag_data->x = (int16_t)(mag_data->x *
                ((int32_t)priv->asa_data.x + AK09912_SENSITIVITY) /
                 AK09912_SENSITIVITY_DIV);
  mag_data->y = (int16_t)(mag_data->y *
               ((int32_t)priv->asa_data.y + AK09912_SENSITIVITY) /
                 AK09912_SENSITIVITY_DIV);
  mag_data->z = (int16_t)(mag_data->z *
               ((int32_t)priv->asa_data.z + AK09912_SENSITIVITY) /
                 AK09912_SENSITIVITY_DIV);

  return ret;
}

/****************************************************************************
 * Name: ak09912_checkid
 *
 * Description:
 *   Read and verify the AK09912 chip ID
 *
 ****************************************************************************/

static int ak09912_checkid(FAR struct ak09912_dev_s *priv)
{
  uint16_t devid = 0;

  /* Read device ID */

  devid = ak09912_getreg8(priv, AK09912_WIA1);
  devid += ak09912_getreg8(priv, AK09912_WIA2) << 8;
  sninfo("devid: 0x%02x\n", devid);

  if (devid != AK09911_DEVID && devid != AK09912_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: ak09912_updatecaldata
 *
 * Description:
 *   Update Calibration Coefficient Data
 *
 ****************************************************************************/

static int ak09912_initialize(FAR struct ak09912_dev_s *priv)
{
  int ret = 0;

  ret += ak09912_set_power_mode(priv, AKM_FUSE_ROM_MODE);
  ret += ak09912_read_sensitivity_data(priv, &priv->asa_data);
  ret += ak09912_set_power_mode(priv, AKM_POWER_DOWN_MODE);
  ret += ak09912_set_noise_suppr_flt(priv, priv->nsf);
  return ret;
}

/****************************************************************************
 * Name: ak09912_open
 *
 * Description:
 *   This function is called whenever the AK09912 device is opened.
 *
 ****************************************************************************/

static int ak09912_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;
  int ret = 0;

  ret = ak09912_set_power_mode(priv, priv->mode);
  if (ret < 0)
    {
      snerr("Failed to set power mode to %d.\n", priv->mode);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ak09912_close
 *
 * Description:
 *   This routine is called when the AK09912 device is closed.
 *
 ****************************************************************************/

static int ak09912_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;
  int ret = 0;

  ret = ak09912_set_power_mode(priv, AKM_POWER_DOWN_MODE);
  if (ret < 0)
    {
      snerr("Failed to set power mode to %d.\n", AKM_POWER_DOWN_MODE);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ak09912_read
 ****************************************************************************/

static ssize_t ak09912_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv = inode->i_private;
  int32_t ret = 0;
  FAR struct mag_data_s *mag_data = (FAR struct mag_data_s *)buffer;

  if (! buffer)
    {
      snerr("Buffer is null\n");
      return -1;
    }

  if (buflen != sizeof(struct mag_data_s))
    {
      snerr("You can't read something other than 32 bits (4 bytes)\n");
      return -1;
    }

  if (priv->compensated)
    {
      irqstate_t flags = enter_critical_section();
      ret = ak09912_read_mag_data(priv, mag_data);
      leave_critical_section(flags);
    }
  else
    {
      irqstate_t flags = enter_critical_section();
      ret = ak09912_read_mag_uncomp_data(priv, mag_data);
      leave_critical_section(flags);
    }

  if (ret < 0)
    {
      snerr("Failed to read data from ak09912.\n");
      return ret;
    }

  return sizeof(struct mag_data_s);
}

/****************************************************************************
 * Name: ak09912_write
 ****************************************************************************/

static ssize_t ak09912_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ak09912_write
 ****************************************************************************/

static int ak09912_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ak09912_dev_s *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case SNIOC_ENABLE_COMPENSATED:
        priv->compensated = (int)arg;
        break;
      default:
        snerr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ak09912_register
 *
 * Description:
 *   Register the AK09912 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/compass0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09912_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct ak09912_dev_s *priv;
  char path[16];
  int ret;

  /* Initialize the AK09912 device structure */

  priv = kmm_zalloc(sizeof(struct ak09912_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = AK09912_ADDR;
  priv->freq = AK09912_FREQ;
  priv->compensated = ENABLE_COMPENSATED;
  nxsem_init(&priv->wait, 0, 0);

  /* set default noise suppression filter. */

  priv->nsf = AK09912_NSF_LOW;

  /* set default power mode */

  priv->mode = AKM_CONT_MEAS_4;

  /* Check Device ID */

  ret = ak09912_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  ret = ak09912_initialize(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize physical device ak09912:%d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  snprintf(path, sizeof(path), "%s%d", devpath, 0);
  ret = register_driver(path, &g_ak09912fops, 0666, priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("AK09912 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AK09912 */
