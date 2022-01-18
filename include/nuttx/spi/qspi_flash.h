/****************************************************************************
 * include/nuttx/spi/qspi_flash.h
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

#ifndef __INCLUDE_NUTTX_SPI_QSPI_FLASH_H
#define __INCLUDE_NUTTX_SPI_QSPI_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/qspi.h>

#ifdef CONFIG_QSPI_FLASH

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
 * Name:  qspi_flash_initialize
 *
 * Description:
 *   Create an instance of the QSPI flash emulated driver.
 *
 * Returned Value:
 *   On success a non-NULL, initialized QSPI driver instance is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_QSPI_FLASH
FAR struct qspi_dev_s *qspi_flash_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_QSPI_FLASH */
#endif /* __INCLUDE_NUTTX_SPI_QSPI_FLASH_H */
