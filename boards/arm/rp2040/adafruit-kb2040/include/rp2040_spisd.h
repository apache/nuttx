/****************************************************************************
 * boards/arm/rp2040/adafruit-kb2040/include/rp2040_spisd.h
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

#ifndef __BOARDS_ARM_RP2040_ADAFRUIT_KB2040_INCLUDE_RP2040_SPISD_H
#define __BOARDS_ARM_RP2040_ADAFRUIT_KB2040_INCLUDE_RP2040_SPISD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_spisd_initialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPISD
int board_spisd_initialize(int minor, int bus);
#endif

/****************************************************************************
 * Name: board_spisd_status
 *
 * Description:
 *   Get the status whether SD Card is present or not.
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_SPISD
uint8_t board_spisd_status(struct spi_dev_s *dev, uint32_t devid);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RP2040_ADAFRUIT_KB2040_INCLUDE_RP2040_SPISD_H */
