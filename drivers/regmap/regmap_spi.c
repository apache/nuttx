/****************************************************************************
 * drivers/regmap/regmap_spi.c
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

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/regmap/regmap.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>

#include <debug.h>

#include "internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Regmap spi bus configuration. */

struct regmap_bus_spi_s
{
  struct regmap_bus_s base;
  FAR struct spi_dev_s *spi; /* SPI bus handler. */
  struct spi_sequence_s seq; /* Sequence of SPI transactions. */
  struct spi_trans_s trans;  /* SPI transaction. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Regmap handle functions. */

static int regmap_spi_write(FAR struct regmap_bus_s *bus,
                            FAR const void *data, unsigned int count);

static int regmap_spi_read(FAR struct regmap_bus_s *bus,
                           FAR const void *reg, unsigned int reg_size,
                           FAR void *val, unsigned int val_size);

static int regmap_spi_reg_write(FAR struct regmap_bus_s *bus,
                                unsigned int regaddr, unsigned int value);

static int regmap_spi_reg_read(FAR struct regmap_bus_s *bus,
                               unsigned int regaddr, FAR void *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int regmap_spi_write(FAR struct regmap_bus_s *bus,
                            FAR const void *data, unsigned int count)
{
  FAR struct regmap_bus_spi_s *dev = (FAR struct regmap_bus_spi_s *)bus;
  int ret;

  /* Get the attributes of this transactione.
   * e.g.:
   *   data[dev] = {reg_addr, reg_val1, reg_val2, ...}
   * if write ok return count, otherwise return negative.
   */

  dev->trans.nwords   = count;
  dev->trans.txbuffer = data;
  dev->trans.rxbuffer = NULL;

  /* Perform the transfer */

  ret = spi_transfer(dev->spi, &dev->seq);

  /* Calculate trans length. */

  return ret >= 0 ? count : ret;
}

static int regmap_spi_read(FAR struct regmap_bus_s *bus,
                           FAR const void *reg, unsigned int reg_size,
                           FAR void *val, unsigned int val_size)
{
  FAR struct regmap_bus_spi_s *dev = (FAR struct regmap_bus_spi_s *)bus;
  int ret;

  /* Get the reference to the spi_transfer_s structure. */

  dev->trans.nwords = reg_size + val_size;
  dev->trans.txbuffer = reg;
  dev->trans.rxbuffer = val;

  /* Perform the transfer. */

  ret = spi_transfer(dev->spi, &dev->seq);

  /* Calculate trans length. */

  return ret >= 0 ? val_size : ret;
}

static int regmap_spi_reg_write(FAR struct regmap_bus_s *bus,
                                unsigned int regaddr, unsigned int value)
{
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = value;

  return regmap_spi_write(bus, txbuffer, 2);
}

static int regmap_spi_reg_read(FAR struct regmap_bus_s *bus,
                               unsigned int regaddr, FAR void *value)
{
  uint8_t tmp = regaddr;

  return regmap_spi_read(bus, &tmp, 1, value, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regmap_init_spi
 *
 * Description:
 *   Regmap init spi bus.
 *
 * Input Parameters:
 *   spi    - An instance of the SPI interface to use to communicate.
 *   freq   - SPI frequency (Hz).
 *   nbits  - Number of bits.
 *   dev_id - See enum spi_devtype_e.
 *   mode   - See enum spi_mode_e.
 *   config - regmap configuration.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

FAR struct regmap_s *
regmap_init_spi(FAR struct spi_dev_s *spi, uint32_t freq,
                uint32_t devid, enum spi_mode_e mode,
                FAR const struct regmap_config_s *config)
{
  FAR struct regmap_bus_spi_s *dev;
  FAR struct regmap_s *regmap;

  dev = kmm_zalloc(sizeof(struct regmap_bus_spi_s));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->base.reg_write = regmap_spi_reg_write;
  dev->base.reg_read  = regmap_spi_reg_read;
  dev->base.write     = regmap_spi_write;
  dev->base.read      = regmap_spi_read;

  dev->trans.deselect = true;        /* De-select after transfer. */
  dev->seq.dev        = devid;       /* SPI controler hard cs index. */
  dev->seq.mode       = mode;        /* See enum spi_mode_e. */
  dev->seq.nbits      = 8;           /* Number of bits, Only supports 8bit. */
  dev->seq.ntrans     = 1;           /* Number of transactions. */
  dev->seq.frequency  = freq;        /* SPI frequency (Hz). */
  dev->seq.trans      = &dev->trans; /* Init spi_sequence_s trans. */
  dev->spi            = spi;

  regmap = regmap_init(&dev->base, config);
  if (regmap == NULL)
    {
      kmm_free(dev);
    }

  return regmap;
}
