/****************************************************************************
 * drivers/sensors/as5048b.c
 * Character driver for the AMS AS5048B Magnetic Rotary Encoder
 *
 *   Copyright (C) 2015 Alexandru Duru. All rights reserved.
 *   Author: Alexandru Duru <alexandruduru@gmail.com>
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
#include <nuttx/i2c.h>
#include <nuttx/sensors/as5048b.h>

#if defined(CONFIG_I2C) && defined(CONFIG_AS5048B)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct as5048b_dev_s
{
  FAR struct i2c_dev_s *i2c; /* I2C interface */
  uint8_t addr;              /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static int as5048b_readb8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval);
static int as5048b_readb16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                           uint8_t regaddrlo, FAR uint16_t *regval);
static int as5048b_writeb8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static int as5048b_writeb16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
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

static int     as5048b_open(FAR struct file *filep);
static int     as5048b_close(FAR struct file *filep);
static ssize_t as5048b_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t as5048b_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     as5048b_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_as5048bfops =
{
  as5048b_open,
  as5048b_close,
  as5048b_read,
  as5048b_write,
  NULL,
  as5048b_ioctl
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
 * Name: as5048b_readb8
 *
 * Description:
 *   Read from an 8-bit register
 *
 ****************************************************************************/

static int as5048b_readb8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval)
{
  uint8_t buffer;
  int ret;

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      sndbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = I2C_READ(priv->i2c, &buffer, sizeof(buffer));
  if (ret < 0)
    {
      sndbg("I2C_READ failed: %d\n", ret);
      return ret;
    }

  *regval = buffer;
  sndbg("addr: %02x value: %02x ret: %d\n", regaddr, *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_readb16
 *
 * Description:
 *   Read from two 8-bit registers
 *
 ****************************************************************************/

static int as5048b_readb16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                           uint8_t regaddrlo, FAR uint16_t *regval)
{
  uint8_t hi, lo;
  int ret;

  /* Read the high 8 bits of the 13-bit value */

  ret = as5048b_readb8(priv, regaddrhi, &hi);
  if (ret < 0)
    {
      sndbg("as5048b_readb8 failed: %d\n", ret);
      return ret;
    }

  /* Read the low 5 bits of the 13-bit value */

  ret = as5048b_readb8(priv, regaddrlo, &lo);
  if (ret < 0)
    {
      sndbg("as5048b_readb8 failed: %d\n", ret);
      return ret;
    }

  *regval = (uint16_t)hi << 6 | (uint16_t)lo;
  sndbg("addrhi: %02x addrlo: %02x value: %04x ret: %d\n",
        regaddrhi, regaddrlo, *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_writeb8
 *
 * Description:
 *   Write from an 8-bit register
 *
 ****************************************************************************/

static int as5048b_writeb8(FAR struct as5048b_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  uint8_t buffer[2];
	int ret;

  sndbg("addr: %02x value: %02x\n", regaddr, regval);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Write the register address followed by the data (no RESTART) */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
  ret = I2C_WRITE(priv->i2c, buffer, sizeof(buffer));
  if (ret < 0)
    {
      sndbg("I2C_WRITE failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048b_writeb16
 *
 * Description:
 *   Write to two 8-bit registers
 *
 ****************************************************************************/

static int as5048b_writeb16(FAR struct as5048b_dev_s *priv, uint8_t regaddrhi,
                            uint8_t regaddrlo, uint16_t regval)
{
  int ret;

  sndbg("addrhi: %02x addrlo: %02x value: %04x\n",
        regaddrhi, regaddrlo, regval);

  /* Write the high 8 bits of the 13-bit value */

  ret = as5048b_writeb8(priv, regaddrhi, (uint8_t)(regval >> 6));
  if (ret < 0)
    {
      sndbg("as5048b_writeb8 failed: %d\n", ret);
      return ret;
    }

  /* Write the low 5 bits of the 13-bit value */

  ret = as5048b_writeb8(priv, regaddrhi, (uint8_t)regval);
  if (ret < 0)
    {
      sndbg("as5048b_writeb8 failed: %d\n", ret);
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
  uint16_t buffer;
  int ret;

  ret = as5048b_readb16(priv, AS5048B_ZEROHI_REG, AS5048B_ZEROLO_REG,
                        &buffer);
  if (ret < 0)
    {
      sndbg("as5048b_readb16 failed: %d\n", ret);
      return ret;
    }

  *zero = buffer;
  sndbg("zero: %04x ret: %d\n", *zero, ret);
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

  sndbg("zero: %04x\n", zero);

  ret = as5048b_writeb16(priv, AS5048B_ZEROHI_REG, AS5048B_ZEROLO_REG, zero);
  if (ret < 0)
    {
      sndbg("as5048b_writeb16 failed: %d\n", ret);
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
  uint8_t buffer;
  int ret;

  ret = as5048b_readb8(priv, AS5048B_AGC_REG, &buffer);
  if (ret < 0)
    {
      sndbg("as5048b_readb8 failed: %d\n", ret);
      return ret;
    }

  *agc = buffer;
  sndbg("agc: %02x ret: %d\n", *agc, ret);
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
  uint8_t buffer;
  int ret;

  ret = as5048b_readb8(priv, AS5048B_DIAG_REG, &buffer);
  if (ret < 0)
    {
      sndbg("as5048b_readb8 failed: %d\n", ret);
      return ret;
    }

  *diag = buffer;
  sndbg("diag: %02x ret: %d\n", *diag, ret);
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
  uint16_t buffer;
  int ret;

  ret = as5048b_readb16(priv, AS5048B_MAGHI_REG, AS5048B_MAGLO_REG, &buffer);
  if (ret < 0)
    {
      sndbg("as5048b_readb16 failed: %d\n", ret);
      return ret;
    }

  *mag = buffer;
  sndbg("mag: %04x ret: %d\n", *mag, ret);
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
  uint16_t buffer;
  int ret;

  ret = as5048b_readb16(priv, AS5048B_ANGHI_REG, AS5048B_ANGLO_REG, &buffer);
  if (ret < 0)
    {
      sndbg("as5048b_readb16 failed: %d\n", ret);
      return ret;
    }

  *ang = buffer;
  sndbg("ang: %04x ret: %d\n", *ang, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048b_open
 *
 * Description:
 *   This function is called whenever the device is opened
 *
 ****************************************************************************/

static int as5048b_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: as5048b_close
 *
 * Description:
 *   This function is called whenever the device is closed
 *
 ****************************************************************************/

static int as5048b_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: as5048b_read
 ****************************************************************************/

static ssize_t as5048b_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct as5048b_dev_s *priv  = inode->i_private;
  FAR uint16_t             *ptr;
  ssize_t                   nsamples;
  ssize_t                   i;
  int                       ret;

  /* How many samples were requested? */

  ptr      = (FAR uint16_t *)buffer;
  nsamples = buflen / sizeof(*ptr);

  sndbg("buflen: %u nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      uint16_t ang = 0;

      /* Read the next uint16_t angle value */

      ret = as5048b_readang(priv, &ang);
      if (ret < 0)
        {
          sndbg("as5048b_readang failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the angle value in the user buffer */

      *ptr++ = ang;
    }

    return nsamples * sizeof(*ptr);
}

/****************************************************************************
 * Name: as5048b_write
 ****************************************************************************/

static ssize_t as5048b_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: as5048b_ioctl
 ****************************************************************************/

static int as5048b_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct as5048b_dev_s *priv  = inode->i_private;
  int                       ret   = OK;

  switch (cmd)
    {
      /* Read from the zero position registers. Arg: uint16_t* pointer. */

      case SNIOC_READZERO:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          ret = as5048b_readzero(priv, ptr);
          sndbg("zero: %04x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write to the zero position registers. Arg: uint16_t value. */

      case SNIOC_WRITEZERO:
        ret = as5048b_writezero(priv, (uint16_t)arg);
        sndbg("zero: %04x ret: %d\n", *(uint16_t *)arg, ret);
        break;

      /* Read from the automatic gain control register. Arg: uint8_t* pointer. */

      case SNIOC_READAGC:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          ret = as5048b_readagc(priv, ptr);
          sndbg("agc: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the diagnostics register. Arg: uint8_t* pointer. */

      case SNIOC_READDIAG:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          ret = as5048b_readdiag(priv, ptr);
          sndbg("diag: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the magnitude registers. Arg: uint16_t* pointer. */

      case SNIOC_READMAG:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          ret = as5048b_readmag(priv, ptr);
          sndbg("mag: %04x ret: %d\n", *ptr, ret);
        }
        break;

      default:
        sndbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as5048b_register
 *
 * Description:
 *   Register the AS5048B character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register,
 *             for example "/dev/angle0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the AS5048B.
 *   addr    - The I2C address of the AS5048B.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int as5048b_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t addr)
{
  FAR struct as5048b_dev_s *priv;
  int ret;

  /* Initialize the device's structure */

  priv = (FAR struct as5048b_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      sndbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_as5048bfops, 0666, priv);
  if (ret < 0)
    {
      sndbg("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_AS5048B */
