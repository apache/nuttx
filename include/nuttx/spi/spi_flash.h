/****************************************************************************
 * include/nuttx/spi/spi_flash.h
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

#ifndef __INCLUDE_NUTTX_SPI_SPI_FLASH_H
#define __INCLUDE_NUTTX_SPI_SPI_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_SPI_FLASH

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  spi_flash_initialize
 *
 * Description:
 *   Create an instance of the SPI flash emulated driver.
 *
 * Input Parameters:
 *   name - the flash model to be emulated.
 *
 * Returned Value:
 *   On success a non-NULL, initialized SPI driver instance is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_FLASH
FAR struct spi_dev_s *spi_flash_initialize(FAR const char *name);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SPI_FLASH */
#endif /* __INCLUDE_NUTTX_SPI_SPI_FLASH_H */
