/****************************************************************************
 * drivers/sensors/as5048b.c
 * Character driver for the AMS AS5048B Magnetic Rotary Encoder
 *
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/as5048b.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_QENCODER) && defined(CONFIG_SENSORS_AS5048B)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct as5048b_dev_s
{
  struct qe_lowerhalf_s    lower; /* AS5048B quadrature encoder lower half */
  FAR struct i2c_master_s *i2c;   /* I2C interface */
  uint8_t                  addr;  /* I2C address */
  uint32_t frequency;             /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static int as5048b_readu8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval);
static int as5048b_readu16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                           uint8_t regaddrlo, FAR uint16_t *regval);
static int as5048b_writeu8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static int as5048b_writeu16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                            uint8_t regaddrlo, uint16_t regval);
static int as5048b_readzero(FAR struct as5048b_dev_s *priv,
                            FAR uint16_t *zero);
static int as5048b_writezero(FAR struct as5048b_dev_s *priv, uint16_t zero);
static int as5048b_readagc(FAR struct as5048b_dev_s *priv, FAR uint8_t *agc);
static int as5048b_readdiag(FAR struct as5048b_dev_s *priv,
                            FAR uint8_t *diag);
static int as5048b_readmag(FAR struct as5048b_dev_s *priv, FAR uint16_t *mag);
static int as5048b_readang(FAR struct as5048b_dev_s *priv, FAR uint16_t *ang);

/* Character Driver Methods */

static int as5048b_setup(FAR struct qe_lowerhalf_s *lower);
static int as5048b_shutdown(FAR struct qe_lowerhalf_s *lower);
static int as5048b_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos);
static int as5048b_reset(FAR struct qe_lowerhalf_s *lower);
static int as5048b_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_qeops =
{
  as5048b_setup,
  as5048b_shutdown,
  as5048b_position,
  as5048b_reset,
  as5048b_ioctl
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as5048b_readu8
 *
 * Description:
 *   Read from an 8-bit register
 *
 ****************************************************************************/

static int as5048b_readu8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval)
{
  int ret;
  struct i2c_config_s config;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x ret: %d\n", regaddr, *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_readu16
 *
 * Description:
 *   Read from two 8-bit registers
 *
 ****************************************************************************/

static int as5048b_readu16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                           uint8_t regaddrlo, FAR uint16_t *regval)
{
  uint8_t hi, lo;
  int ret;

  /* Read the high 8 bits of the 13-bit value */

  ret = as5048b_readu8(priv, regaddrhi, &hi);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu8 failed: %d\n", ret);
      return ret;
    }

  /* Read the low 5 bits of the 13-bit value */

  ret = as5048b_readu8(priv, regaddrlo, &lo);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu8 failed: %d\n", ret);
      return ret;
    }

  *regval = (uint16_t)hi << 6 | (uint16_t)lo;
  sninfo("addrhi: %02x addrlo: %02x value: %04x ret: %d\n",
         regaddrhi, regaddrlo, *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_writeu8
 *
 * Description:
 *   Write from an 8-bit register
 *
 ****************************************************************************/

static int as5048b_writeu8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  sninfo("addr: %02x value: %02x\n", regaddr, regval);

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048b_writeu16
 *
 * Description:
 *   Write to two 8-bit registers
 *
 ****************************************************************************/

static int as5048b_writeu16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                            uint8_t regaddrlo, uint16_t regval)
{
  int ret;

  sninfo("addrhi: %02x addrlo: %02x value: %04x\n",
        regaddrhi, regaddrlo, regval);

  /* Write the high 8 bits of the 13-bit value */

  ret = as5048b_writeu8(priv, regaddrhi, (uint8_t)(regval >> 6));
  if (ret < 0)
    {
      snerr("ERROR: as5048b_writeu8 failed: %d\n", ret);
      return ret;
    }

  /* Write the low 5 bits of the 13-bit value */

  ret = as5048b_writeu8(priv, regaddrhi, (uint8_t)regval);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_writeu8 failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048b_readzero
 *
 * Description:
 *   Read from the zero position registers
 *
 ****************************************************************************/

static int as5048b_readzero(FAR struct as5048b_dev_s *priv,
                            FAR uint16_t *zero)
{
  int ret;

  ret = as5048b_readu16(priv, AS5048B_ZEROHI_REG, AS5048B_ZEROLO_REG, zero);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu16 failed: %d\n", ret);
      return ret;
    }

  sninfo("zero: %04x ret: %d\n", *zero, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_writezero
 *
 * Description:
 *   Write to the zero position registers
 *
 ****************************************************************************/

static int as5048b_writezero(FAR struct as5048b_dev_s *priv, uint16_t zero)
{
  int ret;

  sninfo("zero: %04x\n", zero);

  ret = as5048b_writeu16(priv, AS5048B_ZEROHI_REG, AS5048B_ZEROLO_REG, zero);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_writeu16 failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048b_readagc
 *
 * Description:
 *   Read from the automatic gain control register
 *
 ****************************************************************************/

static int as5048b_readagc(FAR struct as5048b_dev_s *priv, FAR uint8_t *agc)
{
  int ret;

  ret = as5048b_readu8(priv, AS5048B_AGC_REG, agc);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu8 failed: %d\n", ret);
      return ret;
    }

  sninfo("agc: %02x ret: %d\n", *agc, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_readdiag
 *
 * Description:
 *   Read from the diagnostics register
 *
 ****************************************************************************/

static int as5048b_readdiag(FAR struct as5048b_dev_s *priv, FAR uint8_t *diag)
{
  int ret;

  ret = as5048b_readu8(priv, AS5048B_DIAG_REG, diag);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu8 failed: %d\n", ret);
      return ret;
    }

  sninfo("diag: %02x ret: %d\n", *diag, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_readmag
 *
 * Description:
 *   Read from the magnitude registers
 *
 ****************************************************************************/

static int as5048b_readmag(FAR struct as5048b_dev_s *priv, FAR uint16_t *mag)
{
  int ret;

  ret = as5048b_readu16(priv, AS5048B_MAGHI_REG, AS5048B_MAGLO_REG, mag);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu16 failed: %d\n", ret);
      return ret;
    }

  sninfo("mag: %04x ret: %d\n", *mag, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_readang
 *
 * Description:
 *   Read from the angle registers
 *
 ****************************************************************************/

static int as5048b_readang(FAR struct as5048b_dev_s *priv, FAR uint16_t *ang)
{
  int ret;

  ret = as5048b_readu16(priv, AS5048B_ANGHI_REG, AS5048B_ANGLO_REG, ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readu16 failed: %d\n", ret);
      return ret;
    }

  sninfo("ang: %04x ret: %d\n", *ang, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_setup
 *
 * Description:
 *   This method is called when the driver is opened
 *
 ****************************************************************************/

static int as5048b_setup(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: as5048b_shutdown
 *
 * Description:
 *   This method is called when the driver is closed
 *
 ****************************************************************************/

static int as5048b_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: as5048b_position
 *
 * Description:
 *   Return the current position measurement
 *
 ****************************************************************************/

static int as5048b_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos)
{
  FAR struct as5048b_dev_s *priv = (FAR struct as5048b_dev_s *)lower;
  uint16_t ang;
  int ret;

  ret = as5048b_readang(priv, &ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readang failed: %d\n", ret);
      return ret;
    }

  *pos = (int32_t)ang;
  return ret;
}

/****************************************************************************
 * Name: as5048b_reset
 *
 * Description:
 *   Reset the position measurement to zero
 *
 ****************************************************************************/

static int as5048b_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct as5048b_dev_s *priv = (FAR struct as5048b_dev_s *)lower;
  uint16_t ang;
  int ret;

  ret = as5048b_writezero(priv, 0);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_writezero failed: %d\n", ret);
      return ret;
    }

  ret = as5048b_readang(priv, &ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_readang failed: %d\n", ret);
      return ret;
    }

  ret = as5048b_writezero(priv, ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048b_writezero failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048b_ioctl
 ****************************************************************************/

static int as5048b_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg)
{
  FAR struct as5048b_dev_s *priv = (FAR struct as5048b_dev_s *)lower;
  int                       ret  = OK;

  switch (cmd)
    {
      /* Read from the zero position registers. Arg: int32_t* pointer. */

      case QEIOC_ZEROPOSITION:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          uint16_t zero;
          DEBUGASSERT(ptr != NULL);
          ret = as5048b_readzero(priv, &zero);
          if (ret == OK)
            {
              *ptr = (int32_t)zero;
            }

          sninfo("zero: %04x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the automatic gain control register. Arg: uint8_t* pointer. */

      case QEIOC_AUTOGAINCTL:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = as5048b_readagc(priv, ptr);
          sninfo("agc: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the diagnostics register. Arg: uint8_t* pointer. */

      case QEIOC_DIAGNOSTICS:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = as5048b_readdiag(priv, ptr);
          sninfo("diag: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the magnitude registers. Arg: int32_t* pointer. */

      case QEIOC_MAGNITUDE:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          uint16_t mag;
          DEBUGASSERT(ptr != NULL);
          ret = as5048b_readmag(priv, &mag);
          if (ret == OK)
            {
              *ptr = (int32_t)mag;
            }

          sninfo("mag: %04x ret: %d\n", *ptr, ret);
        }
        break;

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
 * Name: as5048b_initialize
 *
 * Description:
 *   Initialize the AS5048B device.
 *
 * Input Parameters:
 *   i2c  - An I2C driver instance.
 *   addr - The I2C address of the AS5048B.
 *
 * Returned Value:
 *   A new lower half quadrature encoder interface for the AS5048B on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *as5048b_initialize(FAR struct i2c_master_s *i2c,
                                              uint8_t addr, uint32_t frequency)
{
  FAR struct as5048b_dev_s *priv;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct as5048b_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  priv->lower.ops = &g_qeops;
  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = frequency;

  return &priv->lower;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_QENCODER && CONFIG_SENSORS_AS5048B */
