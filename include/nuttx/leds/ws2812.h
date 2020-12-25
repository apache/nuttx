/****************************************************************************
 * include/nuttx/leds/ws2812.h
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

#ifndef __INCLUDE_NUTTX_LEDS_WS2812_H
#define __INCLUDE_NUTTX_LEDS_WS2812_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_WS2812

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ws2812_leds_register
 *
 * Description:
 *   Initialize the WS2812 device as a LEDS interface.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leds0"
 *   spi     - An instance of the SPI interface to use to communicate
 *             with the WS2812.
 *   nleds   - Number of addressable LEDs
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ws2812_leds_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                         uint16_t nleds);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_WS2812 */
#endif /* __INCLUDE_NUTTX_LEDS_WS2812_H */
