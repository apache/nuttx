/****************************************************************************
 * drivers/sensors/ms5805.c
 * Character driver for the MEAS MS5805 Altimeter
 *
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <nuttx/sensors/ms5805.h>

#if defined(CONFIG_I2C) && defined(CONFIG_MS5805)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* I2C Address **************************************************************/

#define MS5805_ADDR      0x76

/* Register Definitions *****************************************************/
/* Register Addresses */

#define MS5805_RESET_REG 0x1e /* Reset Register */
#define MS5805_PRESS_REG 0x40 /* Pressure Register */
#define MS5805_TEMP_REG  0x50 /* Temperature Register */
#define MS5805_ADC_REG   0x00 /* ADC Register */
#define MS5805_PROM_REG  0xa0 /* PROM Register */

/* PROM Definitions *********************************************************/

#define MS5805_PROM_LEN  8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ms5805_dev_s
{
  FAR struct i2c_dev_s *i2c;   /* I2C interface */
  int32_t               temp;  /* Uncompensated temperature (degrees Centigrade) */
  int32_t               press; /* Uncompensated pressure (millibar) */
  uint8_t               osr;   /* Oversampling ratio bits */
  useconds_t            delay; /* Oversampling ratio delay */

  /* Calibration coefficients */

  uint16_t              c1;
  uint16_t              c2;
  uint16_t              c3;
  uint16_t              c4;
  uint16_t              c5;
  uint16_t              c6;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* CRC Calculation */

static uint8_t ms5805_crc(FAR const uint8_t *src, size_t len);

/* I2C Helpers */

static int ms5805_readu16(FAR struct ms5805_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval);
static int ms5805_readadc(FAR struct ms5805_dev_s *priv, FAR uint32_t *adc);
static int ms5805_setosr(FAR struct ms5805_dev_s *priv, uint16_t osr);
static int ms5805_reset(FAR struct ms5805_dev_s *priv);
static int ms5805_readprom(FAR struct ms5805_dev_s *priv);
static int ms5805_convert(FAR struct ms5805_dev_s *priv, uint8_t regaddr,
                          FAR uint32_t *regval);
static int ms5805_measure(FAR struct ms5805_dev_s *priv);

/* Character Driver Methods */

static int     ms5805_open(FAR struct file *filep);
static int     ms5805_close(FAR struct file *filep);
static ssize_t ms5805_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ms5805_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     ms5805_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  ms5805_open,
  ms5805_close,
  ms5805_read,
  ms5805_write,
  NULL,
  ms5805_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5805_crc
 *
 * Description:
 *   Calculate the CRC.
 *
 ****************************************************************************/

static uint8_t ms5805_crc(FAR const uint8_t *src, size_t len)
{
  uint16_t crc = 0;
  size_t   i;
  int      j;

  for (i = 0; i < len; i++)
    {
      crc ^= src[i];

      for (j = 0; j < 8; j++)
        {
          bool xor = (crc & 0x8000) != 0;

          crc <<= 1;
          if (xor)
            {
              crc ^= 0x3000;
            }
        }
    }

  return (uint8_t)(crc >> 12);
}

/****************************************************************************
 * Name: ms5805_readu16
 *
 * Description:
 *   Read from a 16-bit register.
 *
 ****************************************************************************/

static int ms5805_readu16(FAR struct ms5805_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval)
{
  uint8_t buffer[2];
  int ret;

  sndbg("addr: %02x\n", regaddr);

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, MS5805_ADDR, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      sndbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16 bits from the register */

  ret = I2C_READ(priv->i2c, buffer, sizeof(buffer));
  if (ret < 0)
    {
      sndbg("I2C_READ failed: %d\n", ret);
      return ret;
    }

  *regval = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  sndbg("value: %04x ret: %d\n", *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: ms5805_readadc
 *
 * Description:
 *   Read from the ADC register.
 *
 ****************************************************************************/

static int ms5805_readadc(FAR struct ms5805_dev_s *priv, FAR uint32_t *adc)
{
  uint8_t regaddr;
  uint8_t buffer[3];
  int ret;

  regaddr = MS5805_ADC_REG;
  sndbg("addr: %02x\n", regaddr);

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, MS5805_ADDR, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      sndbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 24 bits from the register */

  ret = I2C_READ(priv->i2c, buffer, sizeof(buffer));
  if (ret < 0)
    {
      sndbg("I2C_READ failed: %d\n", ret);
      return ret;
    }

  *adc = (uint32_t)buffer[0] << 16 |
         (uint32_t)buffer[1] << 8 |
         (uint32_t)buffer[2];
  sndbg("adc: %06x ret: %d\n", *adc, ret);
  return ret;
}

/****************************************************************************
 * Name: ms5805_setosr
 *
 * Description:
 *   Set the oversampling ratio.
 *
 ****************************************************************************/

static int ms5805_setosr(FAR struct ms5805_dev_s *priv, uint16_t osr)
{
	int ret = OK;

  sndbg("osr: %04x\n", osr);

  switch (osr)
    {
      case 256:
        priv->delay = 540;
        break;

      case 512:
        priv->delay = 1060;
        break;

      case 1024:
        priv->delay = 2080;
        break;

      case 2048:
        priv->delay = 4130;
        break;

      case 4096:
        priv->delay = 8220;
        break;

      case 8192:
        priv->delay = 16440;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (ret == OK)
    {
      priv->osr = (osr / 256 - 1) * 2;
    }

  return ret;
}

/****************************************************************************
 * Name: ms5805_readprom
 *
 * Description:
 *   Read from the PROM.
 *
 ****************************************************************************/

static int ms5805_readprom(FAR struct ms5805_dev_s *priv)
{
  uint16_t prom[MS5805_PROM_LEN];
  uint8_t  regaddr;
  uint8_t  crc;
  int      ret;
  int      i;

  regaddr = MS5805_PROM_REG;
  for (i = 0; i < MS5805_PROM_LEN-1; i++)
    {
      ret = ms5805_readu16(priv, regaddr, prom+i);
      if (ret < 0)
        {
          sndbg("ms5805_readu16 failed: %d\n", ret);
          return ret;
        }

      regaddr += 2;
    }

  crc = (uint8_t)(prom[0] >> 12);

  prom[0]                 = 0;
  prom[MS5805_PROM_LEN-1] = 0;

  if (crc != ms5805_crc((uint8_t *)prom, sizeof(prom)))
    {
      sndbg("crc mismatch\n");
      return -ENODEV;
    }

  priv->c1 = prom[1];
  priv->c2 = prom[2];
  priv->c3 = prom[3];
  priv->c4 = prom[4];
  priv->c5 = prom[5];
  priv->c6 = prom[6];

  return ret;
}

/****************************************************************************
 * Name: ms5805_reset
 *
 * Description:
 *   Reset the device.
 *
 ****************************************************************************/

static int ms5805_reset(FAR struct ms5805_dev_s *priv)
{
  uint8_t regaddr;
	int ret;

  regaddr = MS5805_RESET_REG;
  sndbg("addr: %02x\n", regaddr);

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, MS5805_ADDR, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      sndbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Check the CRC and read the calibration coefficients */

  ret = ms5805_readprom(priv);
  if (ret < 0)
    {
      sndbg("ms5805_readprom failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ms5805_convert
 *
 * Description:
 *   Measure the uncompensated temperature or the uncompensated pressure.
 *
 ****************************************************************************/

static int ms5805_convert(FAR struct ms5805_dev_s *priv, uint8_t regaddr,
                          FAR uint32_t *regval)
{
	int ret;

  regaddr |= priv->osr;
  sndbg("addr: %02x\n", regaddr);

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, MS5805_ADDR, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      sndbg("I2C_WRITE failed: %d\n", ret);
    }

  /* Wait for the conversion to end */

  up_udelay(priv->delay);

  /* Read the value from the ADC */

  ret = ms5805_readadc(priv, regval);
  if (ret < 0)
    {
      sndbg("ms5805_readadc failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ms5805_measure
 *
 * Description:
 *   Measure the compensated temperature and the compensated pressure.
 *
 ****************************************************************************/

static int ms5805_measure(FAR struct ms5805_dev_s *priv)
{
  uint32_t rawpress;
  uint32_t rawtemp;
  int32_t diff;
  int32_t temp;
  int64_t off;
  int64_t sens;
  int32_t press;
	int ret;

  ret = ms5805_convert(priv, MS5805_PRESS_REG, &rawpress);
  if (ret < 0)
    {
      sndbg("ms5805_convert failed: %d\n", ret);
      return ret;
    }

  ret = ms5805_convert(priv, MS5805_TEMP_REG, &rawtemp);
  if (ret < 0)
    {
      sndbg("ms5805_convert failed: %d\n", ret);
      return ret;
    }

  diff = (int32_t)rawtemp - (int32_t)priv->c5 / ((int32_t)1 << 8);
  temp = (int32_t)((int64_t)20 +
                   (int64_t)diff * (int64_t)priv->c6 / ((int64_t)1 << 23));

  off  = (int64_t)priv->c2 * ((int64_t)1 << 17) +
         (int64_t)priv->c4 * (int64_t)diff / ((int64_t)1 << 8);
  sens = (int64_t)priv->c1 * ((int64_t)1 << 16) +
         (int64_t)priv->c3 * (int64_t)diff / ((int64_t)1 << 7);

  if (temp < 2000)
    {
      int32_t ti;
      int64_t offi;
      int64_t sensi;

      ti    = (int32_t)((int64_t)11 * (int64_t)diff *
                        (int64_t)diff / ((int64_t)1 << 35));
      offi  = (int64_t)31 * ((int64_t)temp - (int64_t)2000) *
              ((int64_t)temp - (int64_t)2000) / ((int64_t)1 << 3);
      sensi = (int64_t)63 * ((int64_t)temp - (int64_t)2000) *
              ((int64_t)temp - (int64_t)2000) / ((int64_t)1 << 5);

      temp -= ti;
      off  -= offi;
      sens -= sensi;
    }

  press = (int32_t)(((int64_t)rawpress * sens / ((int64_t)1 << 21) - off) /
                    ((int64_t)1 << 15));

  priv->temp = temp / (int32_t)100;
  priv->press = press / (int32_t)100;
  return ret;
}

/****************************************************************************
 * Name: ms5805_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/

static int ms5805_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ms5805_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/

static int ms5805_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ms5805_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/

static ssize_t ms5805_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: ms5805_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t ms5805_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ms5805_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int ms5805_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ms5805_dev_s *priv  = inode->i_private;
  int                      ret   = OK;

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Measure the temperature and the pressure. Arg: None. */

      case SNIOC_MEASURE:
        DEBUGASSERT(arg == 0);
        ret = ms5805_measure(priv);
        break;

      /* Return the temperature last measured. Arg: int32_t* pointer. */

      case SNIOC_TEMPERATURE:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->temp;
          sndbg("temp: %08x\n", *ptr);
        }
        break;

      /* Return the pressure last measured. Arg: int32_t* pointer. */

      case SNIOC_PRESSURE:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->press;
          sndbg("press: %08x\n", *ptr);
        }
        break;

      /* Reset the device. Arg: None. */

      case SNIOC_RESET:
        DEBUGASSERT(arg == 0);
        ret = ms5805_reset(priv);
        break;

      /* Change the oversampling ratio. Arg: uint16_t value. */

      case SNIOC_OVERSAMPLING:
        ret = ms5805_setosr(priv, (uint16_t)arg);
        sndbg("osr: %04x ret: %d\n", *(uint16_t *)arg, ret);
        break;

      /* Unrecognized commands */

      default:
        sndbg("Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5805_register
 *
 * Description:
 *   Register the MS5805 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   osr     - The oversampling ratio.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms5805_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                    uint16_t osr)
{
  FAR struct ms5805_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct ms5805_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      sndbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c   = i2c;
  priv->temp  = 0;
  priv->press = 0;

  ret = ms5805_setosr(priv, osr);
  if (ret < 0)
    {
      sndbg("ms5805_setosr failed: %d\n", ret);
      goto err;
    }

  ret = ms5805_reset(priv);
  if (ret < 0)
    {
      sndbg("ms5805_reset failed: %d\n", ret);
      goto err;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      sndbg("Failed to register driver: %d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_MS5805 */
