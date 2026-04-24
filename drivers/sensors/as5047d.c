/****************************************************************************
 * drivers/sensors/as5047d.c
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

#include <assert.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <stdlib.h>
#include <stdint.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/as5047d.h>

#if defined(CONFIG_SENSORS_AS5047D)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct as5047d_dev_s
{
  struct qe_lowerhalf_s lower;  /* AS5047D quadrature encoder lower half */
  FAR struct spi_dev_s  *spi;   /* SPI interface */

  /* Since multiple AS5047D can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired AS5047D chip via their chip select inputs.
   */

  int spi_devid;

  /* AS5047D returns the response of the previous frame. Track which
   * register is currently staged in the pipeline.
   */

  bool     pipeline_valid;
  uint16_t pipeline_reg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int as5047d_exchange(FAR struct as5047d_dev_s *priv,
                            uint16_t regaddr, FAR uint16_t *regval);
static uint16_t as5047d_calc_even_parity(uint16_t value);
static int as5047d_readang(FAR struct as5047d_dev_s *priv,
                           FAR uint16_t *ang);
static int as5047d_readmag(FAR struct as5047d_dev_s *priv,
                           FAR uint16_t *mag);
static int as5047d_readdiag(FAR struct as5047d_dev_s *priv,
                            FAR uint16_t *diag);

/* Character Driver Methods */

static int as5047d_setup(FAR struct qe_lowerhalf_s *lower);
static int as5047d_shutdown(FAR struct qe_lowerhalf_s *lower);
static int as5047d_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos);
static int as5047d_reset(FAR struct qe_lowerhalf_s *lower);
static int as5047d_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_qeops =
{
  as5047d_setup,    /* setup */
  as5047d_shutdown, /* shutdown */
  as5047d_position, /* position */
  NULL,             /* setposmax */
  as5047d_reset,    /* reset */
  NULL,             /* setindex */
  as5047d_ioctl     /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: as5047d_configspi
 *
 * Description:
 *   Configure SPI for the AS5047D.
 *
 ****************************************************************************/

static inline void as5047d_configspi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, AS5047D_SPI_MODE);
  SPI_SETBITS(spi, 16);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, AS5047D_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: as5047d_calc_even_parity
 *
 * Description:
 *   Get the even parity of input value.
 *
 ****************************************************************************/

static uint16_t as5047d_calc_even_parity(uint16_t value)
{
  uint16_t cnt = 0;
  uint8_t i;

  for (i = 0; i < 16; i++)
    {
      if (value & 0x1)
        {
          cnt++;
        }

      value >>= 1;
    }

  return cnt & 0x1;
}

/****************************************************************************
 * Name: as5047d_exchange
 *
 * Description:
 *   Read from 16-bit registers.
 *
 ****************************************************************************/

static int as5047d_exchange(FAR struct as5047d_dev_s *priv,
                            uint16_t regaddr, FAR uint16_t *regval)
{
  uint16_t cmd;
  uint16_t tx;
  uint16_t rx;
  uint16_t sample;

  DEBUGASSERT(priv != NULL && regval != NULL);

  SPI_LOCK(priv->spi, true);
  as5047d_configspi(priv->spi);

  cmd = AS5047D_CMD_READ | (regaddr & AS5047D_VALUE_MASK);
  cmd |= as5047d_calc_even_parity(cmd) << 15;

  /* AS5047D expects framed 16-bit transfers. Keep the SPI lock but toggle
   * CS for each command/response frame.
   */

  SPI_SELECT(priv->spi, priv->spi_devid, true);
  tx = cmd;
  SPI_EXCHANGE(priv->spi, &tx, &rx, 1);
  SPI_SELECT(priv->spi, priv->spi_devid, false);

  if (priv->pipeline_valid && priv->pipeline_reg == regaddr)
    {
      sample = rx;
    }
  else
    {
      /* Pipeline is empty or points to a different register. Send one more
       * READ frame so received data corresponds to regaddr.
       */

      SPI_SELECT(priv->spi, priv->spi_devid, true);
      tx = cmd;
      SPI_EXCHANGE(priv->spi, &tx, &sample, 1);
      SPI_SELECT(priv->spi, priv->spi_devid, false);
    }

  priv->pipeline_valid = true;
  priv->pipeline_reg   = regaddr;

  SPI_LOCK(priv->spi, false);

  if ((sample & AS5047D_FLAG_ERR) != 0)
    {
      snerr("ERROR: as5047d error flag set: %04x reg: %04x\n",
            sample, regaddr);
      return -EIO;
    }

  *regval = sample & AS5047D_VALUE_MASK;
  return OK;
}

/****************************************************************************
 * Name: as5047d_readang
 ****************************************************************************/

static int as5047d_readang(FAR struct as5047d_dev_s *priv, FAR uint16_t *ang)
{
  int ret;

  ret = as5047d_exchange(priv, AS5047D_REG_ANGLEUNC, ang);
  if (ret < 0)
    {
      snerr("ERROR: as5047d_exchange failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: as5047d_readmag
 ****************************************************************************/

static int as5047d_readmag(FAR struct as5047d_dev_s *priv, FAR uint16_t *mag)
{
  int ret;

  ret = as5047d_exchange(priv, AS5047D_REG_MAG, mag);
  if (ret < 0)
    {
      snerr("ERROR: as5047d_exchange failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: as5047d_readdiag
 ****************************************************************************/

static int as5047d_readdiag(FAR struct as5047d_dev_s *priv,
                            FAR uint16_t *diag)
{
  int ret;

  ret = as5047d_exchange(priv, AS5047D_REG_DIAAGC, diag);
  if (ret < 0)
    {
      snerr("ERROR: as5047d_exchange failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: as5047d_setup
 ****************************************************************************/

static int as5047d_setup(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: as5047d_shutdown
 ****************************************************************************/

static int as5047d_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: as5047d_position
 *
 * Description:
 *   Return the current position measurement relative to reset position.
 *
 ****************************************************************************/

static int as5047d_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos)
{
  FAR struct as5047d_dev_s *priv = (FAR struct as5047d_dev_s *)lower;
  uint16_t rawang;
  int ret;

  ret = as5047d_readang(priv, &rawang);
  if (ret < 0)
    {
      snerr("ERROR: as5047d_readang failed: %d\n", ret);
      return ret;
    }

  *pos = (int32_t)(rawang & AS5047D_VALUE_MASK);
  return ret;
}

/****************************************************************************
 * Name: as5047d_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ****************************************************************************/

static int as5047d_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct as5047d_dev_s *priv = (FAR struct as5047d_dev_s *)lower;
  uint16_t rawang;
  int ret;

  ret = as5047d_readang(priv, &rawang);
  if (ret < 0)
    {
      snerr("ERROR: as5047d_readang failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: as5047d_ioctl
 ****************************************************************************/

static int as5047d_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                         unsigned long arg)
{
  FAR struct as5047d_dev_s *priv = (FAR struct as5047d_dev_s *)lower;
  int ret = OK;

  switch (cmd)
    {
      case QEIOC_AS5047D_DIAGNOSTICS:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = as5047d_readdiag(priv, ptr);
        }
        break;

      case QEIOC_AS5047D_MAGNITUDE:
        {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = as5047d_readmag(priv, ptr);
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
 * Name: as5047d_initialize
 *
 * Description:
 *   Initialize the AS5047D device.
 *
 * Input Parameters:
 *   spi  - An SPI driver instance.
 *
 * Returned Value:
 *   A new lower half encoder interface for the AS5047D on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *as5047d_initialize(FAR struct spi_dev_s *spi,
                                              int spi_devid)
{
  FAR struct as5047d_dev_s *priv;

  DEBUGASSERT(spi != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  priv->lower.ops   = &g_qeops;
  priv->spi         = spi;
  priv->spi_devid   = spi_devid;

  return &priv->lower;
}

#endif /* CONFIG_SENSORS_AS5047D */
