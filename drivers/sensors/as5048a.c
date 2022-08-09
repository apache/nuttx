/****************************************************************************
 * drivers/sensors/as5048a.c
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
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/as5048a.h>

#if defined(CONFIG_SENSORS_AS5048A)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct as5048a_dev_s
{
  struct qe_lowerhalf_s lower; /* AS5048A quadrature encoder lower half */
  FAR struct spi_dev_s  *spi;  /* SPI interface */

  /* Since multiple AS5048A can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired AS5048A chip via their chip select inputs.
   */

  int spi_devid;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int as5048a_writeu16(FAR struct as5048a_dev_s *priv,
                            uint16_t regaddr, uint16_t regval);
static int as5048a_exchange(FAR struct as5048a_dev_s *priv,
                            uint16_t regaddr, FAR uint16_t *regval);
static uint16_t calc_even_parity(uint16_t value);
static int as5048a_readzero(FAR struct as5048a_dev_s *priv,
                            FAR uint16_t *zero);
static int as5048a_writezero(FAR struct as5048a_dev_s *priv, uint16_t zero);
static int as5048a_readagc(FAR struct as5048a_dev_s *priv,
                           FAR uint16_t *agc);
static int as5048a_readdiag(FAR struct as5048a_dev_s *priv,
                            FAR uint16_t *diag);
static int as5048a_readmag(FAR struct as5048a_dev_s *priv,
                           FAR uint16_t *mag);
static int as5048a_readang(FAR struct as5048a_dev_s *priv,
                           FAR uint16_t *ang);

/* Character Driver Methods */

static int as5048a_setup(FAR struct qe_lowerhalf_s *lower);
static int as5048a_shutdown(FAR struct qe_lowerhalf_s *lower);
static int as5048a_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos);
static int as5048a_reset(FAR struct qe_lowerhalf_s *lower);
static int as5048a_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_qeops =
{
  as5048a_setup,    /* setup */
  as5048a_shutdown, /* shutdown */
  as5048a_position, /* position */
  NULL,             /* setposmax */
  as5048a_reset,    /* reset */
  NULL,             /* setindex */
  as5048a_ioctl     /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as5048a_configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void as5048a_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the AS5048A */

  SPI_SETMODE(spi, AS5048A_SPI_MODE);
  SPI_SETBITS(spi, 16);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, AS5048A_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: as5048a_writeu16
 *
 * Description:
 *   Write to two 8-bit registers
 *
 ****************************************************************************/

static int as5048a_writeu16(FAR struct as5048a_dev_s *priv,
                            uint16_t regaddr, uint16_t regval)
{
  uint16_t datout;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  as5048a_configspi(priv->spi);

  /* Select the AS5048A */

  SPI_SELECT(priv->spi, priv->spi_devid, true);

  /* Send register address and set the value */

  datout = AS5048A_CMD_WRITE | regaddr;
  datout |= calc_even_parity(datout) << 15;
  SPI_SEND(priv->spi, datout);

  datout = 0x3fff & regval;
  datout |= calc_even_parity(datout) << 15;
  SPI_SEND(priv->spi, datout);

  /* Send NOP command */

  SPI_SEND(priv->spi, 0x0000);

  /* Deselect the AS5048A */

  SPI_SELECT(priv->spi, priv->spi_devid, false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  return OK;
}

/****************************************************************************
 * Name: as5048a_exchange
 *
 * Description:
 *   Read from 16-bit registers
 *
 ****************************************************************************/

static int as5048a_exchange(FAR struct as5048a_dev_s *priv,
                            uint16_t regaddr, FAR uint16_t *regval)
{
  uint16_t ret;
  uint16_t dat;
  uint16_t temp;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  as5048a_configspi(priv->spi);

  /* Select the AS5048A */

  SPI_SELECT(priv->spi, priv->spi_devid, true);

  /* Send READ  command. Received data is thrown away
   * this data comes from the previous command (unknown)
   */

  dat = AS5048A_CMD_READ | regaddr;
  dat |= calc_even_parity(dat) << 15;

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, dat);

  /* Send NOP command. Received data is the value of regaddr
   * from the previous command
   */

  dat = 0x0000; /* NOP command */
  temp = SPI_SEND(priv->spi, dat);

  if (temp & 0x4000)
    {
      /* error flag set - need to reset it */

      dat = AS5048A_CMD_READ | AS5048A_CLRERR_REG;
      dat |= calc_even_parity(dat) << 15;
      SPI_SEND(priv->spi, dat);

      snerr("ERROR: as5048a error flag set, need to reset it! %02x\n", temp);
      ret = -1;
    }
  else
    {
      *regval = temp;
      ret = 0;
    }

  /* Deselect the AS5048A */

  SPI_SELECT(priv->spi, priv->spi_devid, false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  sninfo("addr: %02x value: %02x ret: %d\n", regaddr, *regval, ret);
  return ret;
}

/****************************************************************************
 * Name: calc_even_parity
 *
 * Description:
 *   get the even parity of input value
 *
 ****************************************************************************/

static uint16_t calc_even_parity(uint16_t value)
{
  uint16_t cnt = 0;
  uint8_t i;

  for (i = 0; i < 16; i++)
    {
      if (value & 0x1) cnt++;
      value >>= 1;
    }

  return cnt & 0x1;
}

/****************************************************************************
 * Name: as5048a_readzero
 *
 * Description:
 *   Read from the zero position registers
 *
 ****************************************************************************/

static int as5048a_readzero(FAR struct as5048a_dev_s *priv,
                            FAR uint16_t *zero)
{
  uint16_t hi;
  uint16_t lo;
  int ret;

  /* Read the high 8 bits of the 13-bit value */

  ret = as5048a_exchange(priv, AS5048A_ZEROHI_REG, &hi);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_readu8 failed: %d\n", ret);
      return ret;
    }

  /* Read the low 5 bits of the 13-bit value */

  ret = as5048a_exchange(priv, AS5048A_ZEROLO_REG, &lo);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_readu8 failed: %d\n", ret);
      return ret;
    }

  *zero = (uint16_t)hi << 6 | (uint16_t)lo;

  sninfo("zero: %04x ret: %d\n", *zero, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048a_writezero
 *
 * Description:
 *   Write to the zero position registers
 *
 ****************************************************************************/

static int as5048a_writezero(FAR struct as5048a_dev_s *priv, uint16_t zero)
{
  int ret;

  sninfo("zero: %04x\n", zero);

  ret = as5048a_writeu16(priv, AS5048A_ZEROHI_REG, (zero >> 6));
  if (ret < 0)
    {
      snerr("ERROR: as5048a_writeu16 failed: %d\n", ret);
    }

  ret = as5048a_writeu16(priv, AS5048A_ZEROLO_REG, zero);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_writeu16 failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048a_readagc
 *
 * Description:
 *   Read from the automatic gain control register
 *
 ****************************************************************************/

static int as5048a_readagc(FAR struct as5048a_dev_s *priv, FAR uint16_t *agc)
{
  int ret;

  ret = as5048a_exchange(priv, AS5048A_AGC_REG, agc);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_exchange failed: %d\n", ret);
      return ret;
    }

  sninfo("agc: %02x ret: %d\n", *agc, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048a_readdiag
 *
 * Description:
 *   Read from the diagnostics register
 *
 ****************************************************************************/

static int as5048a_readdiag(FAR struct as5048a_dev_s *priv,
                            FAR uint16_t *diag)
{
  int ret;

  ret = as5048a_exchange(priv, AS5048A_DIAG_REG, diag);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_exchange failed: %d\n", ret);
      return ret;
    }

  sninfo("diag: %02x ret: %d\n", *diag, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048a_readmag
 *
 * Description:
 *   Read from the magnitude registers
 *
 ****************************************************************************/

static int as5048a_readmag(FAR struct as5048a_dev_s *priv, FAR uint16_t *mag)
{
  int ret;

  ret = as5048a_exchange(priv, AS5048A_MAG_REG, mag);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_exchange failed: %d\n", ret);
      return ret;
    }

  sninfo("mag: %04x ret: %d\n", *mag, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048a_readang
 *
 * Description:
 *   Read from the angle registers
 *
 ****************************************************************************/

static int as5048a_readang(FAR struct as5048a_dev_s *priv, FAR uint16_t *ang)
{
  int ret;

  ret = as5048a_exchange(priv, AS5048A_ANGLE_REG, ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_exchange failed: %d\n", ret);
      return ret;
    }

  sninfo("ang: %04x ret: %d\n", *ang, ret);
  return ret;
}

/****************************************************************************
 * Name: as5048a_setup
 *
 * Description:
 *   This method is called when the driver is opened
 *
 ****************************************************************************/

static int as5048a_setup(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: as5048a_shutdown
 *
 * Description:
 *   This method is called when the driver is closed
 *
 ****************************************************************************/

static int as5048a_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: as5048a_position
 *
 * Description:
 *   Return the current position measurement
 *
 ****************************************************************************/

static int as5048a_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos)
{
  FAR struct as5048a_dev_s *priv = (FAR struct as5048a_dev_s *)lower;
  uint16_t ang;
  int ret;

  ret = as5048a_readang(priv, &ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_readang failed: %d\n", ret);
      return ret;
    }

  *pos = (int32_t)ang;
  return ret;
}

/****************************************************************************
 * Name: as5048a_reset
 *
 * Description:
 *   Reset the position measurement to zero
 *
 ****************************************************************************/

static int as5048a_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct as5048a_dev_s *priv = (FAR struct as5048a_dev_s *)lower;
  uint16_t ang;
  int ret;

  ret = as5048a_writezero(priv, 0);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_writezero failed: %d\n", ret);
      return ret;
    }

  ret = as5048a_readang(priv, &ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_readang failed: %d\n", ret);
      return ret;
    }

  ret = as5048a_writezero(priv, ang);
  if (ret < 0)
    {
      snerr("ERROR: as5048a_writezero failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: as5048a_ioctl
 ****************************************************************************/

static int as5048a_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg)
{
  FAR struct as5048a_dev_s *priv = (FAR struct as5048a_dev_s *)lower;
  int                       ret  = OK;

  switch (cmd)
    {
      /* Read from the zero position registers. Arg: int* pointer. */

      case QEIOC_AS5048A_ZEROPOSITION:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          uint16_t zero;
          DEBUGASSERT(ptr != NULL);
          ret = as5048a_readzero(priv, &zero);
          if (ret == OK)
            {
              *ptr = (int)zero;
            }

          sninfo("zero: %04x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the automatic gain control register */

      case QEIOC_AS5048A_AUTOGAINCTL:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = as5048a_readagc(priv, ptr);
          sninfo("agc: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the diagnostics register. Arg: uint8_t* pointer. */

      case QEIOC_AS5048A_DIAGNOSTICS:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = as5048a_readdiag(priv, ptr);
          sninfo("diag: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Read from the magnitude registers. Arg: int* pointer. */

      case QEIOC_AS5048A_MAGNITUDE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          uint16_t mag;
          DEBUGASSERT(ptr != NULL);
          ret = as5048a_readmag(priv, &mag);
          if (ret == OK)
            {
              *ptr = (int)mag;
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
 * Name: as5048a_initialize
 *
 * Description:
 *   Initialize the AS5048A device.
 *
 * Input Parameters:
 *   spi  - An SPI driver instance.
 *
 * Returned Value:
 *   A new lower half encoder interface for the AS5048A on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *as5048a_initialize(FAR struct spi_dev_s *spi,
                                              int spi_devid)
{
  FAR struct as5048a_dev_s *priv;

  DEBUGASSERT(spi != NULL);

  /* Initialize the device's structure */

  priv = (FAR struct as5048a_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  priv->lower.ops = &g_qeops;
  priv->spi       = spi;
  priv->spi_devid = spi_devid;

  return &priv->lower;
}

#endif /* CONFIG_SENSORS_AS5048A */
