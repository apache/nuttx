/****************************************************************************
 * drivers/sensors/mt6816.c
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
#include <errno.h>
#include <nuttx/debug.h>
#include <stdlib.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/mt6816.h>

#if defined(CONFIG_SENSORS_MT6816)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MT6816_REG_ANGLE_LSB  0x03
#define MT6816_REG_ANGLE_MSB  0x04
#define MT6816_CMD_READ       0x80
#define MT6816_NO_MAG_WARN    0x02
#define MT6816_PARITY_CHECK   0x01

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mt6816_dev_s
{
  struct qe_lowerhalf_s lower; /* MT6816 quadrature encoder lower half */
  FAR struct spi_dev_s  *spi;  /* SPI interface */

  /* Since multiple MT6816 can be connected to the same SPI bus we need
   * to use multiple spi device ids which are employed by NuttX to select/
   * deselect the desired MT6816 chip via their chip select inputs.
   */

  int devid;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mt6816_exchange(FAR struct mt6816_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval);
static uint16_t calc_even_parity(uint16_t value);
static int mt6816_readang(FAR struct mt6816_dev_s *priv,
                          FAR uint16_t *ang);

/* Character Driver Methods */

static int mt6816_setup(FAR struct qe_lowerhalf_s *lower);
static int mt6816_shutdown(FAR struct qe_lowerhalf_s *lower);
static int mt6816_position(FAR struct qe_lowerhalf_s *lower,
                           FAR int32_t *pos);
static int mt6816_reset(FAR struct qe_lowerhalf_s *lower);
static int mt6816_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qe_ops_s g_qeops =
{
  mt6816_setup,    /* setup */
  mt6816_shutdown, /* shutdown */
  mt6816_position, /* position */
  NULL,            /* setposmax */
  mt6816_reset,    /* reset */
  NULL,            /* setindex */
  mt6816_ioctl     /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mt6816_configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void mt6816_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the MT6816 */

  SPI_SETMODE(spi, MT6816_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, MT6816_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Name: mt6816_exchange
 *
 * Description:
 *   Read from 8-bit registers
 *
 ****************************************************************************/

static int mt6816_exchange(FAR struct mt6816_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval)
{
  uint8_t dat;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  mt6816_configspi(priv->spi);

  /* Select the MT6816 */

  SPI_SELECT(priv->spi, SPIDEV_MAG_ENCODER(priv->devid), true);

  /* Send READ  command. Received data is thrown away
   * this data comes from the previous command (unknown)
   */

  dat = MT6816_CMD_READ | regaddr;

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, dat);

  /* Send NOP command. Received data is the value of regaddr
   * from the previous command
   */

  dat = 0x00; /* NOP command */
  *regval = SPI_SEND(priv->spi, dat);

  /* Deselect the MT6816 */

  SPI_SELECT(priv->spi, SPIDEV_MAG_ENCODER(priv->devid), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

  sninfo("addr: %02x value: %02x\n", regaddr, *regval);
  return OK;
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
      if (value & 0x1)
        {
          cnt++;
        }

      value >>= 1;
    }

  return cnt & 0x1;
}

/****************************************************************************
 * Name: mt6816_readang
 *
 * Description:
 *   Read from the angle position registers
 *
 ****************************************************************************/

static int mt6816_readang(FAR struct mt6816_dev_s *priv,
                           FAR uint16_t *ang)
{
  uint8_t hi;
  uint8_t lo;
  uint16_t value;
  bool no_mag;
  bool parity;
  int ret;

  /* Read the high 8 bits of the 14-bit value */

  ret = mt6816_exchange(priv, MT6816_REG_ANGLE_LSB, &hi);
  if (ret < 0)
    {
      snerr("ERROR: mt6816_exchange failed: %d\n", ret);
      return ret;
    }

  /* Read the low 6 bits of the 14-bit value */

  ret = mt6816_exchange(priv, MT6816_REG_ANGLE_MSB, &lo);
  if (ret < 0)
    {
      snerr("ERROR: mt6816_exchange failed: %d\n", ret);
      return ret;
    }

  /* Read No Magnetic Error and Parity Check bits first */

  no_mag = (lo & MT6816_NO_MAG_WARN) >> 1;
  parity = (lo & MT6816_PARITY_CHECK);

  value = ((uint16_t)hi << 8 | (uint16_t)lo);

  /* Check if parity is right:
   * - Clear the parity bit
   * - Check if the calculated parity match with internal parity
   */

  value &= ~MT6816_PARITY_CHECK;

  if (calc_even_parity(value) == parity)
    {
      sninfo("Parity is correct!\n");
    }
  else
    {
      snerr("Parity error: expected %d got %d", !parity, parity);
    }

  /* Save only the angle value */

  *ang = value >> 2;

  sninfo("angle: %04d, NO_MAG=%d, PC=%d ret: %d\n",
         *ang, no_mag, parity, ret);

  return ret;
}

/****************************************************************************
 * Name: mt6816_setup
 *
 * Description:
 *   This method is called when the driver is opened
 *
 ****************************************************************************/

static int mt6816_setup(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: mt6816_shutdown
 *
 * Description:
 *   This method is called when the driver is closed
 *
 ****************************************************************************/

static int mt6816_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Name: mt6816_position
 *
 * Description:
 *   Return the current position measurement
 *
 ****************************************************************************/

static int mt6816_position(FAR struct qe_lowerhalf_s *lower,
                            FAR int32_t *pos)
{
  FAR struct mt6816_dev_s *priv = (FAR struct mt6816_dev_s *)lower;
  uint16_t ang;
  int ret;

  ret = mt6816_readang(priv, &ang);
  if (ret < 0)
    {
      snerr("ERROR: mt6816_readang failed: %d\n", ret);
      return ret;
    }

  *pos = (int32_t)ang;

  return ret;
}

/****************************************************************************
 * Name: mt6816_reset
 *
 * Description:
 *   Reset the position measurement to zero
 *
 ****************************************************************************/

static int mt6816_reset(FAR struct qe_lowerhalf_s *lower)
{
  /* Seems like the MT6816 doesn't have a SPI writable register
   * to define the zero position. However it has a register to
   * flash the zero position using its memory programming (MTP)
   * but it requires supply high voltage (7V) to HVPP pin.
   */

  return OK;
}

/****************************************************************************
 * Name: mt6816_ioctl
 ****************************************************************************/

static int mt6816_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
                        unsigned long arg)
{
  /* There is nothing we can('t) do! */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mt6816_initialize
 *
 * Description:
 *   Initialize the MT6816 device.
 *
 * Input Parameters:
 *   spi  - An SPI driver instance.
 *
 * Returned Value:
 *   A new lower half encoder interface for the MT6816 on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *mt6816_initialize(FAR struct spi_dev_s *spi,
                                             uint16_t devid)
{
  FAR struct mt6816_dev_s *priv;

  DEBUGASSERT(spi != NULL);

  /* Initialize the device's structure */

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return NULL;
    }

  priv->lower.ops = &g_qeops;
  priv->spi       = spi;
  priv->devid     = devid;

  return &priv->lower;
}

#endif /* CONFIG_SENSORS_MT6816 */
