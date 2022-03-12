/****************************************************************************
 * drivers/i2c/pca9540bdp.h
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

#ifndef __DRIVERS_I2C_PCA9540BDP_H
#define __DRIVERS_I2C_PCA9540BDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_I2CMULTIPLEXER_PCA9540BDP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_PCA9549BDP_I2C_FREQUENCY
#  define CONFIG_PCA9549BDP_I2C_FREQUENCY   400000  /* 400 khz */
#endif

/* Bit Definitions to be passed to write calls */

#define PCA9540BDP_ENABLE                   0x4
#define PCA9540BDP_DISABLE                  0x0

/* Bit masks */

#define PCA9540BDP_CH_BITMASK               0x03
#define PCA9540BDP_ENABLE_BITMASK           0x04

#define PCA9540BDP_CH_BIT                   0
#define PCA9540BDP_CH_NONE_BIT              1
#define PCA9540BDP_ENABLE_BIT               2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pca9540bdp_dev_s
{
  FAR struct i2c_master_s *i2c;      /* I2C interface */
  uint16_t addr;
  uint8_t state;                     /* Control register state */
};

struct i2c_port_dev_s
{
  FAR struct i2c_master_s vi2c;      /* Nested structure to allow casting as
                                      * public i2c master */
  uint8_t port;                      /* Associated port on the mux */
  FAR struct pca9540bdp_dev_s *dev;  /* Associated device */
};

#endif /* CONFIG_I2CMULTIPLEXER_PCA9540BDP */
#endif /* __DRIVERS_I2CMULTIPLEXER_PCA9540BDP_H */
