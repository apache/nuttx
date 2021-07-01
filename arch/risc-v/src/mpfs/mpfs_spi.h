/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_spi.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_SPI_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *mpfs_spibus_initialize(int port);

/****************************************************************************
 * Name: mpfs_spibus_uninitialize
 *
 * Description:
 *   Uninitialize an SPI bus
 *
 ****************************************************************************/

int mpfs_spibus_uninitialize(struct spi_dev_s *dev);

/****************************************************************************
 * Name:  mpfs_spi0/1_select
 *
 * Description:
 *   The external function, mpfs_spi0/1_select
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   devid - The SPI CS or device number
 *   selected - true: assert CS, false de-assert CS
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_SPI0
void weak_function mpfs_spi0_select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
#endif

#ifdef CONFIG_MPFS_SPI1
void weak_function mpfs_spi1_select(FAR struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_SPI_H */
