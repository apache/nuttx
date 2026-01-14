/****************************************************************************
 * include/nuttx/regmap/regmap.h
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

#ifndef  __INCLUDE_NUTTX_REGMAP_REGMAP_H
#define  __INCLUDE_NUTTX_REGMAP_REGMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct regmap_bus_s;

/* Single byte register read/write. */

typedef CODE int (*reg_read_t)(FAR struct regmap_bus_s *bus,
                               unsigned int reg,
                               FAR void *val);
typedef CODE int (*reg_write_t)(FAR struct regmap_bus_s *bus,
                                unsigned int reg,
                                unsigned int val);

/* Bulk read/write */

typedef CODE int (*read_t)(FAR struct regmap_bus_s *bus,
                           FAR const void *reg_buf, unsigned int reg_size,
                           FAR void *val_buf, unsigned int val_size);
typedef CODE int (*write_t)(FAR struct regmap_bus_s *bus,
                            FAR const void *data,
                            unsigned int count);

/* Resources destroyed. */

typedef CODE void (*exit_t)(FAR struct regmap_bus_s *bus);

/* Description of a hardware bus for the register map infrastructure. */

struct regmap_bus_s
{
  reg_read_t  reg_read;
  reg_write_t reg_write;
  read_t  read;
  write_t write;
  exit_t  exit;
};

/* Configuration for the register map of a device.
 * reg_bits and val_bits must be set.
 */

struct regmap_config_s
{
  /* Number of bits in a register address, mandatory. */

  int reg_bits;

  /* The register address stride. Valid register addresses are a multiple
   * of this value. If set to 0, a value of 1 will be used.
   */

  int reg_stride;

  /* Number of bits in a register value, mandatory. */

  int val_bits;

  /* This regmap is either protected by external means or is guaranteed
   * not to be accessed from multiple threads. Don't use any locking
   * mechanisms.
   */

  bool disable_locking;
};

struct regmap_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: regmap_init
 *
 * Description:
 *   Initialize the internal configuration of regmap. The first parameter
 *   must be the handle of the bus, and the second parameter is the
 *   configuration parameter of the bus. Finally, these two parameters will
 *   be transparent to the corresponding bus.
 *
 * Input Parameters:
 *   dev    - device handle.
 *   bus    - device configuration.
 *   config - regmap configuration.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

FAR struct regmap_s *regmap_init(FAR struct regmap_bus_s *bus,
                                 FAR const struct regmap_config_s *config);

/****************************************************************************
 * Name: regmap_init_i2c
 *
 * Description:
 *   Regmap init i2c bus.
 *
 * Input Parameters:
 *   i2c        - An instance of the I2C interface to use to communicate.
 *   i2c_config - i2c device configuration.
 *   config     - regmap configuration.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C
struct i2c_master_s;
struct i2c_config_s;

FAR struct regmap_s *
regmap_init_i2c(FAR struct i2c_master_s *i2c,
                FAR struct i2c_config_s *i2c_config,
                FAR const struct regmap_config_s *config);
#endif /* CONFIG_I2C */

/****************************************************************************
 * Name: regmap_init_spi
 *
 * Description:
 *   regmap init spi bus.
 *
 * Input Parameters:
 *   spi    - An instance of the SPI interface to use to communicate.
 *   freq   - SPI frequency (Hz)
 *   nbits  - Number of bits
 *   dev_id - See enum spi_devtype_e
 *   mode   - See enum spi_mode_e
 *   config - regmap configuration.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI
struct spi_dev_s;

FAR struct regmap_s *
regmap_init_spi(FAR struct spi_dev_s *spi, uint32_t freq,
                uint32_t devid, enum spi_mode_e mode,
                FAR const struct regmap_config_s *config);
#endif /* CONFIG_SPI */

/****************************************************************************
 * Name: regmap_exit
 *
 * Description:
 *   Regmap exit function.
 *
 ****************************************************************************/

void regmap_exit(FAR struct regmap_s *map);

/****************************************************************************
 * Name: regmap_write
 *
 * Description:
 *   Regmap write, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map - regmap handler, from regmap bus init function return.
 *   reg - register address to be write.
 *   val - write data.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_write(FAR struct regmap_s *map, unsigned int reg,
                 unsigned int val);

/****************************************************************************
 * Name: regmap_bulk_write
 *
 * Description:
 *   Regmap bulk write, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map       - regmap handler, from regmap bus init function return.
 *   reg       - register address to be write.
 *   val       - write data buffer.
 *   val_count - write data buffer size.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_bulk_write(FAR struct regmap_s *map, unsigned int reg,
                      FAR const void *val, unsigned int val_count);

/****************************************************************************
 * Name: regmap_read
 *
 * Description:
 *   Regmap read, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map - regmap handler, from regmap bus init function return.
 *   reg - register address to be read.
 *   val - read data buffer.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_read(FAR struct regmap_s *map, unsigned int reg,
                FAR void *val);

/****************************************************************************
 * Name: regmap_bulk_read
 *
 * Description:
 *   Regmap bulk read, called after initializing the regmap bus device.
 *   the first parameter is regmap pointer.
 *
 * Input Parameters:
 *   map       - regmap handler, from regmap bus init function return.
 *   reg       - register address to be read.
 *   val       - read data buffer.
 *   val_count - read data buffer size.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int regmap_bulk_read(FAR struct regmap_s *map, unsigned int reg,
                     FAR void *val, unsigned int val_count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_REGMAP_REGMAP_H */
