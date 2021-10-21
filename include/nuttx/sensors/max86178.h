/****************************************************************************
 * include/nuttx/sensors/max86178.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MAX86178_H
#define __INCLUDE_NUTTX_SENSORS_MAX86178_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/ioexpander/ioexpander.h>
#ifdef CONFIG_SENSORS_MAX86178_I2C
#  include <nuttx/i2c/i2c_master.h>
#else /* CONFIG_SENSORS_MAX86178_SPI */
#  include <nuttx/spi/spi_transfer.h>
#endif

#ifdef CONFIG_SENSORS_MAX86178

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct max86178_config_s
{
  int                         freq;       /* Digital interface frequency */
  int                         intpin;     /* Interrupt pin */
#ifdef CONFIG_SENSORS_MAX86178_I2C
  FAR struct i2c_master_s     *i2c;       /* I2C interface instance */
  uint8_t                      i2c_addr;  /* I2C address. not left-shifted */
#else /* CONFIG_SENSORS_MAX86178_SPI */
  FAR struct spi_dev_s        *spi;       /* SPI interface instance */
  uint32_t                    cs;         /* SPI CS pin */
  uint32_t                    gpiocs;     /* SPI CS pin number as GPIO */
#endif
  FAR struct ioexpander_dev_s *ioedev;    /* Ioexpander device */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max86178_register
 *
 * Description:
 *   Register the MAX86178 character device as 'devno'
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   dev     - An instance of the I2C/SPI interface to communicate with
 *             MAX86178.
 *
 * Returned Value:
 *   Return 0 if the driver was successfully initialize; A negative errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none
 *
 ****************************************************************************/

int max86178_register(int devno, FAR const struct max86178_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MAX86178 */
#endif /* __INCLUDE_NUTTX_SENSORS_MAX86178_H */
