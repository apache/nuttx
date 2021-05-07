/****************************************************************************
 * include/nuttx/mmcsd.h
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

#ifndef __INCLUDE_NUTTX_MMCSD_H
#define __INCLUDE_NUTTX_MMCSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mmcsd_slotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See nuttx/sdio.h */
int mmcsd_slotinitialize(int minor, FAR struct sdio_dev_s *dev);

/****************************************************************************
 * Name: mmcsd_spislotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the SPI MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for
 *     architectures that support multiple MMC/SD slots. This value must be
 *     in the range {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   spi - And instance of an SPI interface obtained by called the
 *     appropriate xyz_spibus_initialize() function for the MCU "xyz" with
 *     the appropriate port number.
 *
 ****************************************************************************/

struct spi_dev_s; /* See nuttx/spi/spi.h */
int mmcsd_spislotinitialize(int minor,
                            int slotno,
                            FAR struct spi_dev_s *spi);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MMCSD_H */
