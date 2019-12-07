/****************************************************************************
 * drivers/sensors/ms58xx.c
 * Character driver for MEAS MS58XX Altimeters
 *
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *   Updated by: Karim Keddam <karim.keddam@polymtl.ca>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ms58xx.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MS58XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MS58XX_I2C_FREQUENCY
#  define CONFIG_MS58XX_I2C_FREQUENCY 400000
#endif

/* Register Definitions *****************************************************/

/* Register Addresses */

#define MS58XX_RESET_REG 0x1e /* Reset Register */
#define MS58XX_PRESS_REG 0x40 /* Pressure Register */
#define MS58XX_TEMP_REG  0x50 /* Temperature Register */
#define MS58XX_ADC_REG   0x00 /* ADC Register */
#define MS58XX_PROM_REG  0xa0 /* PROM Register */

/* PROM Definitions *********************************************************/

#define MS58XX_PROM_LEN  8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ms58xx_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t               addr;   /* I2C address */

  enum ms58xx_model_e   model;
  uint8_t               crcindex;
  uint8_t               crcshift;

  int32_t               temp;   /* Uncompensated temperature (degrees Centigrade) */
  int32_t               press;  /* Uncompensated pressure (millibar) */

  uint8_t               osr;    /* Oversampling ratio bits */
  useconds_t            delay;  /* Oversampling ratio delay */

  /* Calibration coefficients */

  uint16_t              c1;
  uint16_t              c2;
  uint16_t              c3;
  uint16_t              c4;
  uint16_t              c5;
  uint16_t              c6;
  uint8_t               c7;
  uint8_t               c8;

  /* Constants used when calculating the temperature and the pressure */

  uint8_t               c1s;
  uint8_t               c2s;
  uint8_t               c3s;
  uint8_t               c4s;

  uint8_t               diffmull;
  uint8_t               diffdivls;
  uint8_t               offmull;
  uint8_t               offdivls;
  uint8_t               sensmull;
  uint8_t               sensdivls;

  uint8_t               offmulvl;
  uint8_t               sensmulvl;

  uint8_t               diffmulh;
  uint8_t               diffdivhs;
  uint8_t               offmulh;
  uint8_t               offdivhs;
  uint8_t               sensmulh;
  uint8_t               sensdivhs;

  uint8_t               pressdivs;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CRC Calculation */

static uint8_t ms58xx_crc(FAR uint16_t *src, uint8_t crcndx, uint16_t crcmask);

/* I2C Helpers */

static int ms58xx_i2c_write(FAR struct ms58xx_dev_s *priv,
                             FAR const uint8_t *buffer, int buflen);
static int ms58xx_i2c_read(FAR struct ms58xx_dev_s *priv,
                            FAR uint8_t *buffer, int buflen);
static int ms58xx_readu16(FAR struct ms58xx_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval);
static int ms58xx_readadc(FAR struct ms58xx_dev_s *priv, FAR uint32_t *adc);
static int ms58xx_setosr(FAR struct ms58xx_dev_s *priv, uint16_t osr);
static int ms58xx_reset(FAR struct ms58xx_dev_s *priv);
static int ms58xx_readprom(FAR struct ms58xx_dev_s *priv);
static int ms58xx_convert(FAR struct ms58xx_dev_s *priv, uint8_t regaddr,
                          FAR uint32_t *regval);
static int ms58xx_measure(FAR struct ms58xx_dev_s *priv);

/* Character Driver Methods */

static int     ms58xx_open(FAR struct file *filep);
static int     ms58xx_close(FAR struct file *filep);
static ssize_t ms58xx_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ms58xx_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     ms58xx_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
  ms58xx_open,
  ms58xx_close,
  ms58xx_read,
  ms58xx_write,
  NULL,
  ms58xx_ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms58xx_crc
 *
 * Description:
 *   Calculate the CRC.
 *
 ****************************************************************************/

static uint8_t ms58xx_crc(FAR uint16_t *src, uint8_t crcndx, uint16_t crcmask)
{
  uint16_t cnt;
  uint16_t n_rem;
  uint16_t crc_read;
  uint8_t n_bit;

  n_rem = 0x00;
  crc_read = src[crcndx];
  src[crcndx] &= ~crcmask;

  for (cnt = 0; cnt < 16; cnt++)
    {
      if (cnt % 2 == 1)
        {
          n_rem ^= (uint16_t)((src[cnt >> 1]) & 0x00ff);
        }
      else
        {
          n_rem ^= (uint16_t)(src[cnt >> 1] >> 8);
        }

      for (n_bit = 8; n_bit > 0; n_bit--)
        {
          if (n_rem & (0x8000))
            {
              n_rem = (n_rem << 1) ^ 0x3000;
            }
          else
            {
              n_rem = (n_rem << 1);
            }
        }
    }

  n_rem = (0x000f & (n_rem >> 12));
  src[crcndx] = crc_read;
  return (n_rem ^ 0x00);
}

/****************************************************************************
 * Name: ms58xx_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int ms58xx_i2c_write(FAR struct ms58xx_dev_s *priv,
                             FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_MS58XX_I2C_FREQUENCY,
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: ms58xx_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int ms58xx_i2c_read(FAR struct ms58xx_dev_s *priv,
                            FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_MS58XX_I2C_FREQUENCY,
  msg.addr      = priv->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: ms58xx_readu16
 *
 * Description:
 *   Read from a 16-bit register.
 *
 ****************************************************************************/

static int ms58xx_readu16(FAR struct ms58xx_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval)
{
  uint8_t buffer[2];
  int ret;

  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = ms58xx_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16 bits from the register */

  ret = ms58xx_i2c_read(priv, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  *regval = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  sninfo("value: %04x ret: %d\n", *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: ms58xx_readadc
 *
 * Description:
 *   Read from the ADC register.
 *
 ****************************************************************************/

static int ms58xx_readadc(FAR struct ms58xx_dev_s *priv, FAR uint32_t *adc)
{
  uint8_t regaddr;
  uint8_t buffer[3];
  int ret;

  regaddr = MS58XX_ADC_REG;
  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = ms58xx_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 24 bits from the register */

  ret = ms58xx_i2c_read(priv, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  *adc = (uint32_t)buffer[0] << 16 |
         (uint32_t)buffer[1] << 8 |
         (uint32_t)buffer[2];

  sninfo("adc: %06x ret: %d\n", *adc, ret);
  return ret;
}

/****************************************************************************
 * Name: ms58xx_setosr_1
 *
 * Description:
 *   Set the oversampling ratio.
 *
 ****************************************************************************/

static int ms58xx_setosr_1(FAR struct ms58xx_dev_s *priv, uint16_t osr)
{
  int ret = OK;
  switch (osr)
    {
      case 256:
          priv->delay = 600;
          priv->osr   = 0x0;
          break;

      case 512:
          priv->delay = 1170;
          priv->osr   = 0x2;
          break;

      case 1024:
          priv->delay = 2280;
          priv->osr   = 0x4;
          break;

      case 2048:
          priv->delay = 4540;
          priv->osr   = 0x6;
          break;

      case 4096:
          priv->delay = 9040;
          priv->osr   = 0x8;
          break;

      case 8192:
          priv->delay = 18080;
          priv->osr   = 0xa;
          break;

      default:
          ret = -EINVAL;
          break;
    }

  return ret;
}

/****************************************************************************
 * Name: ms58xx_setosr_2
 *
 * Description:
 *   Set the oversampling ratio.
 *
 ****************************************************************************/

static int ms58xx_setosr_2(FAR struct ms58xx_dev_s *priv, uint16_t osr)
{
  int ret = OK;
  switch (osr)
    {
      case 256:
        priv->delay = 600;
        break;

      case 512:
        priv->delay = 1170;
        break;

      case 1024:
        priv->delay = 2280;
        break;

      case 2048:
        priv->delay = 4540;
        break;

      case 4096:
        priv->delay = 9040;
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
 * Name: ms58xx_setosr
 *
 * Description:
 *   Set the oversampling ratio.
 *
 ****************************************************************************/

static int ms58xx_setosr(FAR struct ms58xx_dev_s *priv, uint16_t osr)
{
  int ret = OK;

  sninfo("osr: %04x\n", osr);

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5805_02:
      case MS58XX_MODEL_MS5837_30:
          ret = ms58xx_setosr_1(priv, osr);
          break;

      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5803_05:
      case MS58XX_MODEL_MS5803_07:
      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5803_30:
      case MS58XX_MODEL_MS5806_02:
          ret = ms58xx_setosr_2(priv, osr);
          break;

      default:
         ret = -EINVAL;
         break;
    }

  return ret;
}

/****************************************************************************
 * Name: ms58xx_readprom
 *
 * Description:
 *   Read from the PROM.
 *
 ****************************************************************************/

static int ms58xx_readprom(FAR struct ms58xx_dev_s *priv)
{
  uint16_t prom[MS58XX_PROM_LEN];
  uint8_t  crc;
  uint8_t  crcindex;
  uint8_t  crcshift;
  uint16_t crcmask;
  int      ret;
  int      i;
  int      len = MS58XX_PROM_LEN;

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5805_02:
      case MS58XX_MODEL_MS5837_30:
        prom[MS58XX_PROM_LEN - 1] = 0;
        len--;
        crcindex = 0;
        crcshift = 12;
        break;

      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5803_05:
      case MS58XX_MODEL_MS5803_07:
      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5803_30:
      case MS58XX_MODEL_MS5806_02:
      default:
        crcindex = 7;
        crcshift = 0;
        break;
    }

  /* We have to wait before the prom is ready is be read */

  up_udelay(10000);
  for (i = 0; i < len; i++)
    {
      ret = ms58xx_readu16(priv, MS58XX_PROM_REG + i * 2, prom + i);
      if (ret < 0)
        {
          snerr("ERROR: ms58xx_readu16 failed: %d\n", ret);
          return ret;
        }
    }

  crcmask         = (uint16_t)0xf << crcshift;
  crc             = (uint8_t)((prom[crcindex] & crcmask) >> crcshift);

  if (crc != ms58xx_crc(prom, crcindex, crcmask))
    {
      snerr("ERROR: crc mismatch\n");
      return -ENODEV;
    }

  priv->c1 = prom[1];
  priv->c2 = prom[2];
  priv->c3 = prom[3];
  priv->c4 = prom[4];
  priv->c5 = prom[5];
  priv->c6 = prom[6];

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5803_07:
        priv->c7 = (uint8_t)((prom[7] & 0x03f0) >> 4);
        priv->c8 = (uint8_t)((prom[7] & 0xf700) >> 10);
        break;

      case MS58XX_MODEL_MS5806_02:
        priv->c7 = (uint8_t)((prom[7] & 0x0ff0) >> 4);
        priv->c8 = 0;
        break;

      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5803_05:
      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5803_30:
      case MS58XX_MODEL_MS5805_02:
      case MS58XX_MODEL_MS5837_30:
      default:
        priv->c7 = 0;
        priv->c8 = 0;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ms58xx_reset
 *
 * Description:
 *   Reset the device.
 *
 ****************************************************************************/

static int ms58xx_reset(FAR struct ms58xx_dev_s *priv)
{
  uint8_t regaddr;
  int ret;

  regaddr = MS58XX_RESET_REG;
  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = ms58xx_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Check the CRC and read the calibration coefficients */

  ret = ms58xx_readprom(priv);
  if (ret < 0)
    {
      snerr("ERROR: ms58xx_readprom failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ms58xx_convert
 *
 * Description:
 *   Measure the uncompensated temperature or the uncompensated pressure.
 *
 ****************************************************************************/

static int ms58xx_convert(FAR struct ms58xx_dev_s *priv, uint8_t regaddr,
                          FAR uint32_t *regval)
{
  int ret;

  regaddr |= priv->osr;
  sninfo("addr: %02x\n", regaddr);

  /* Write the register address */

  ret = ms58xx_i2c_write(priv, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  /* Wait for the conversion to end */

  up_udelay(priv->delay);

  /* Read the value from the ADC */

  ret = ms58xx_readadc(priv, regval);
  if (ret < 0)
    {
      snerr("ERROR: ms58xx_readadc failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ms58xx_measure
 *
 * Description:
 *   Measure the compensated temperature and the compensated pressure.
 *
 ****************************************************************************/

static int ms58xx_measure(FAR struct ms58xx_dev_s *priv)
{
  uint32_t rawpress;
  uint32_t rawtemp;
  int32_t diff;
  int32_t temp;
  int64_t off;
  int64_t sens;
  int32_t press;
  int64_t diffmul;
  int64_t diffdiv;
  int64_t offmula;
  int64_t offdiva;
  int64_t sensmula;
  int64_t sensdiva;
  int64_t offmulb;
  int64_t sensmulb;
  int64_t tm;
  int64_t tp;
  int ret;

  ret = ms58xx_convert(priv, MS58XX_PRESS_REG, &rawpress);
  if (ret < 0)
    {
      snerr("ERROR: ms58xx_convert failed: %d\n", ret);
      return ret;
    }

  ret = ms58xx_convert(priv, MS58XX_TEMP_REG, &rawtemp);
  if (ret < 0)
    {
      snerr("ERROR: ms58xx_convert failed: %d\n", ret);
      return ret;
    }

  add_sensor_randomness(rawpress ^ rawtemp);

  diff = (int32_t)rawtemp - (int32_t)priv->c5 * ((int32_t)1 << 8);
  temp = (int32_t)((int64_t)2000 +
                   (int64_t)diff * (int64_t)priv->c6 / ((int64_t)1 << 23));

  off  = (int64_t)priv->c2 * ((int64_t)1 << priv->c2s) +
         (int64_t)priv->c4 * (int64_t)diff / ((int64_t)1 << priv->c4s);
  sens = (int64_t)priv->c1 * ((int64_t)1 << priv->c1s) +
         (int64_t)priv->c3 * (int64_t)diff / ((int64_t)1 << priv->c3s);

  if (temp < 2000)
    {
      diffmul  = (int64_t)priv->diffmull;
      diffdiv  = (int64_t)1 << priv->diffdivls;
      offmula  = (int64_t)priv->offmull;
      offdiva  = (int64_t)1 << priv->offdivls;
      sensmula = (int64_t)priv->sensmull;
      sensdiva = (int64_t)1 << priv->sensdivls;

      if (temp < -1500)
        {
          offmulb  = (int64_t)priv->offmulvl;
          sensmulb = (int64_t)priv->sensmulvl;
        }
      else
        {
          offmulb  = 0;
          sensmulb = 0;
        }
    }
  else
    {
      diffmul  = (int64_t)priv->diffmulh;
      diffdiv  = (int64_t)1 << priv->diffdivhs;
      offmula  = (int64_t)priv->offmulh;
      offdiva  = (int64_t)1 << priv->offdivhs;
      sensmula = (int64_t)priv->sensmulh;
      sensdiva = (int64_t)1 << priv->sensdivhs;

      offmulb  = 0;
      sensmulb = 0;
    }

  tm = (int64_t)temp - (int64_t)2000;
  tm *= tm;

  tp = (int64_t)temp + (int64_t)1500;
  tp *= tp;

  off  -= offmula * tm / offdiva + offmulb * tp;
  sens -= sensmula * tm / sensdiva + sensmulb * tp;
  temp -= (int32_t)(diffmul * (int64_t)diff * (int64_t)diff / diffdiv);

  press = (int32_t)(((int64_t)rawpress * sens / ((int64_t)1 << 21) - off) /
                    ((int64_t)1 << priv->pressdivs));

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5803_07:
        if (press > 110000)
          {
            press += (int32_t)((((int64_t)priv->c7 - ((int64_t)1 << 5)) *
                               (int64_t)100 * ((int64_t)1 << 2) -
                               ((int64_t)priv->c8 - ((int64_t)1 << 5)) *
                               ((int64_t)temp - (int64_t)2000) /
                               ((int64_t)1 << 4)) *
                               ((int64_t)press - (int64_t)110000) /
                               (int64_t)49000000);
          }
        break;

      case MS58XX_MODEL_MS5806_02:
#if CONFIG_MS58XX_VDD >= 22 && CONFIG_MS58XX_VDD <= 30
        press += (int32_t)(((int64_t)30 - (int64_t)CONFIG_MS58XX_VDD) *
                           (int64_t)priv->c7 /
                           (((int64_t)1 << 6) * (int64_t)10));
#endif
        break;

      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5803_05:
      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5803_30:
      case MS58XX_MODEL_MS5805_02:
      case MS58XX_MODEL_MS5837_30:
        break;
    }

  priv->temp = temp;
  priv->press = press;
  return ret;
}

/****************************************************************************
 * Name: ms58xx_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/

static int ms58xx_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ms58xx_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/

static int ms58xx_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ms58xx_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/

static ssize_t ms58xx_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  ssize_t size;
  FAR struct inode *inode        = filep->f_inode;
  FAR struct ms58xx_dev_s *priv  = inode->i_private;
  FAR struct ms58xx_measure_s *p = (FAR struct ms58xx_measure_s *)buffer;

  size = buflen;
  while (size >= sizeof(*p))
    {
      if (ms58xx_measure(priv) < 0)
        {
          return -1;
        }

      p->temperature  = priv->temp;
      p->pressure     = priv->press;

      p++;
      size -= sizeof(*p);
    }

  return size;
}

/****************************************************************************
 * Name: ms58xx_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t ms58xx_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ms58xx_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int ms58xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct ms58xx_dev_s *priv  = inode->i_private;
  int                      ret   = OK;

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Measure the temperature and the pressure. Arg: None. */

      case SNIOC_MEASURE:
        DEBUGASSERT(arg == 0);
        ret = ms58xx_measure(priv);
        break;

      /* Return the temperature last measured. Arg: int32_t* pointer. */

      case SNIOC_TEMPERATURE:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->temp;
          sninfo("temp: %08x\n", *ptr);
        }
        break;

      /* Return the pressure last measured. Arg: int32_t* pointer. */

      case SNIOC_PRESSURE:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->press;
          sninfo("press: %08x\n", *ptr);
        }
        break;

      /* Reset the device. Arg: None. */

      case SNIOC_RESET:
        DEBUGASSERT(arg == 0);
        ret = ms58xx_reset(priv);
        break;

      /* Change the oversampling ratio. Arg: uint16_t value. */

      case SNIOC_OVERSAMPLING:
        ret = ms58xx_setosr(priv, (uint16_t)arg);
        sninfo("osr: %04x ret: %d\n", *(uint16_t *)arg, ret);
        break;

      /* Unrecognized commands */

      default:
        snerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms58xx_register
 *
 * Description:
 *   Register the MS58XX character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the MS58XX.
 *   osr     - The oversampling ratio.
 *   model   - The MS58XX model.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms58xx_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, uint16_t osr, enum ms58xx_model_e model)
{
  FAR struct ms58xx_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(((model == MS58XX_MODEL_MS5803_02 ||
                model == MS58XX_MODEL_MS5803_05 ||
                model == MS58XX_MODEL_MS5803_07 ||
                model == MS58XX_MODEL_MS5803_14 ||
                model == MS58XX_MODEL_MS5803_30 ||
                model == MS58XX_MODEL_MS5806_02) &&
               (addr == MS58XX_ADDR0 || addr == MS58XX_ADDR1)) ||
              ((model == MS58XX_MODEL_MS5805_02 ||
                model == MS58XX_MODEL_MS5837_30) &&
               addr == MS58XX_ADDR0));

  /* Initialize the device's structure */

  priv = (FAR struct ms58xx_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c   = i2c;
  priv->addr  = addr;
  priv->model = model;
  priv->temp  = 0;
  priv->press = 0;

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5805_02:
      case MS58XX_MODEL_MS5806_02:
      default:
        priv->c1s = 16;
        priv->c2s = 17;
        priv->c3s = 7;
        priv->c4s = 6;
        break;

      case MS58XX_MODEL_MS5803_05:
        priv->c1s = 17;
        priv->c2s = 18;
        priv->c3s = 7;
        priv->c4s = 5;
        break;

      case MS58XX_MODEL_MS5803_07:
        priv->c1s = 17;
        priv->c2s = 18;
        priv->c3s = 6;
        priv->c4s = 5;
        break;

      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5803_30:
      case MS58XX_MODEL_MS5837_30:
        priv->c1s = 15;
        priv->c2s = 16;
        priv->c3s = 8;
        priv->c4s = 7;
        break;
    }

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5806_02:
        priv->diffmull  = 1;
        priv->diffdivls = 31;
        priv->offmull   = 61;
        priv->offdivls  = 4;
        priv->sensmull  = 2;
        priv->sensdivls = 0;

        priv->offmulvl  = 20;
        priv->sensmulvl = 12;

        priv->diffmulh  = 0;
        priv->diffdivhs = 0;
        priv->offmulh   = 0;
        priv->offdivhs  = 0;
        priv->sensmulh  = 0;
        priv->sensdivhs = 0;
        break;

      case MS58XX_MODEL_MS5803_05:
      case MS58XX_MODEL_MS5803_07:
        priv->diffmull  = 3;
        priv->diffdivls = 33;
        priv->offmull   = 3;
        priv->offdivls  = 3;
        priv->sensmull  = 7;
        priv->sensdivls = 3;

        priv->offmulvl  = 0;
        priv->sensmulvl = 3;

        priv->diffmulh  = 0;
        priv->diffdivhs = 0;
        priv->offmulh   = 0;
        priv->offdivhs  = 0;
        priv->sensmulh  = 0;
        priv->sensdivhs = 0;
        break;

      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5803_30:
        priv->diffmull  = 3;
        priv->diffdivls = 33;
        priv->offmull   = 3;
        priv->offdivls  = 1;
        priv->sensmull  = 5;
        priv->sensdivls = 3;

        priv->offmulvl  = 7;
        priv->sensmulvl = 4;

        priv->diffmulh  = 7;
        priv->diffdivhs = 37;
        priv->offmulh   = 1;
        priv->offdivhs  = 4;
        priv->sensmulh  = 0;
        priv->sensdivhs = 0;
        break;

      case MS58XX_MODEL_MS5805_02:
        priv->diffmull  = 11;
        priv->diffdivls = 35;
        priv->offmull   = 31;
        priv->offdivls  = 3;
        priv->sensmull  = 63;
        priv->sensdivls = 5;

        priv->offmulvl  = 0;
        priv->sensmulvl = 0;

        priv->diffmulh  = 0;
        priv->diffdivhs = 0;
        priv->offmulh   = 0;
        priv->offdivhs  = 0;
        priv->sensmulh  = 0;
        priv->sensdivhs = 0;
        break;

      case MS58XX_MODEL_MS5837_30:
        priv->diffmull  = 3;
        priv->diffdivls = 33;
        priv->offmull   = 3;
        priv->offdivls  = 1;
        priv->sensmull  = 5;
        priv->sensdivls = 3;

        priv->offmulvl  = 7;
        priv->sensmulvl = 4;

        priv->diffmulh  = 2;
        priv->diffdivhs = 37;
        priv->offmulh   = 1;
        priv->offdivhs  = 4;
        priv->sensmulh  = 0;
        priv->sensdivhs = 0;
        break;
    }

  switch (priv->model)
    {
      case MS58XX_MODEL_MS5803_02:
      case MS58XX_MODEL_MS5803_05:
      case MS58XX_MODEL_MS5803_14:
      case MS58XX_MODEL_MS5805_02:
      case MS58XX_MODEL_MS5806_02:
      default:
        priv->pressdivs = 15;
        break;

      case MS58XX_MODEL_MS5803_30:
      case MS58XX_MODEL_MS5837_30:
        priv->pressdivs = 13;
        break;
    }

  ret = ms58xx_setosr(priv, osr);
  if (ret < 0)
    {
      snerr("ERROR: ms58xx_setosr failed: %d\n", ret);
      goto errout;
    }

  ret = ms58xx_reset(priv);
  if (ret < 0)
    {
      snerr("ERROR: ms58xx_reset failed: %d\n", ret);
      goto errout;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto errout;
    }

  return ret;

errout:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_MS58XX */
