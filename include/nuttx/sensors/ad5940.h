/****************************************************************************
 * include/nuttx/sensors/ad5940.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AD5940_H
#define __INCLUDE_NUTTX_SENSORS_AD5940_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/spi/spi_transfer.h>

#ifdef CONFIG_SENSORS_AD5940

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ad5940_config_s
{
  int                         freq;       /* Digital interface frequency */
  int                         intpin;     /* Interrupt pin */
  FAR struct spi_dev_s        *spi;       /* SPI interface instance */
  uint32_t                    cs;         /* SPI CS pin */
  uint32_t                    gpiocs;     /* SPI CS pin as a GPIO pin */
  uint32_t                    gpiorst;    /* Reset pin */
  FAR struct ioexpander_dev_s *ioedev;    /* Ioexpander device */
  FAR struct ioexpander_dev_s *ioerpmsg;  /* Ioexpander device for PMU GPIO */
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
 * Name: ad5940_register
 *
 * Description:
 *   Register the AD5940 character device as 'devno'
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   dev     - An instance of the SPI port to communicate with AD5940.
 *
 * Returned Value:
 *   Return 0 if the driver was successfully initialize; A negative errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none
 *
 ****************************************************************************/

int ad5940_register(int devno, FAR const struct ad5940_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_AD5940 */
#endif /* __INCLUDE_NUTTX_SENSORS_AD5940_H */
