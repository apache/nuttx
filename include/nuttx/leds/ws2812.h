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
#include <nuttx/fs/fs.h>

#ifdef CONFIG_WS2812_NON_SPI_DRIVER
#include <nuttx/mutex.h>
#else /* CONFIG_WS2812_NON_SPI_DRIVER */
#include <nuttx/spi/spi.h>
#endif /* CONFIG_WS2812_NON_SPI_DRIVER */
#ifdef CONFIG_WS2812

/****************************************************************************
 * ######## ATTENTION #######
 * This file contains code that supports two separate ws2812 upper-half
 * models:
 *
 * If CONFIG_WS2812_NON_SPI_DRIVER is NOT defined the older upper-half
 * code that uses SPI to send the serial data will be built.
 *
 * If WS2812_NEW_MODEL_DRIVER is defined the upper-half code that does
 * not relay on SPI will be built.
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

struct ws2812_dev_s
{
  /* --------------------------------------------------------
   * Each board that implements ws2812 must initialize the
   * following fields before calling ws2812_register.
   *
   * The close and read pointer may be initialized to NULL
   * and the upper-half will do the right thing.
   */

  FAR int       (*open)(FAR struct file *filep);

  FAR int       (*close)(FAR struct file *filep);

  FAR ssize_t   (*write)(FAR struct file *filep,
                         FAR const char  *data,
                         size_t           len);

  FAR ssize_t   (*read)(FAR struct file *filep,
                        FAR char        *data,
                        size_t           len);

  void                  *private;    /* Private data for opened device */
  uint32_t               clock;
  mutex_t                lock;
  int                    port;
  uint16_t               nleds;
  bool                   has_white;
};

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

/****************************************************************************
 * Name: ws2812_register
 *
 * Description:
 *   Initialize a ws2812 device as a LEDs interface.
 *
 * Input Parameters:
 *   dev_path  - The full path to the driver to register. E.g., "/dev/leds0"
 *   dev_data  - The partially initialize ws2812 structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ws2812_register(FAR const char          *dev_path,
                    FAR struct ws2812_dev_s *dev_data);

#else /* CONFIG_WS2812_NON_SPI_DRIVER */

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

int ws2812_leds_register(FAR const char  *devpath, FAR struct spi_dev_s *spi,
                         uint16_t nleds);

#endif /* CONFIG_WS2812_NON_SPI_DRIVER */

/****************************************************************************
 * Name: ws2812_hsv_to_rgb
 *
 * Description:
 *   Convert a set of hue, saturation and value numbers to an RGB pixel.
 *
 *   Representative "hue" values:
 *     Red       0
 *     Yellow   42
 *     Green    86
 *     Cyan    128
 *     Blue    170
 *     Magenta 212
 *
 *   "Saturation" values run from 0 (gray) to 255 (pure color)
 *
 *   "Value" values run from 0 (black) to 255 (full brightness)
 *
 * Input Parameters:
 *   hue        in range (0-255) (red -> 0, green -> 85, blue -> 170)
 *   saturation in range (0-255)
 *   value      in range (0-255)
 *
 * Returned Value:
 *   A 32-bit pixel in 0x00RRGGBB format.
 *
 ****************************************************************************/

uint32_t ws2812_hsv_to_rgb(uint8_t hue,
                           uint8_t saturation,
                           uint8_t value);

/****************************************************************************
 * Name: ws2812_gamma_correct
 *
 * Description:
 *   Applies a gamma correction to the supplied pixel.
 *
 *   This applies the function  y = 255 * (x / 255)^2.6 to each color
 *   component of the pixel.
 *
 * Input Parameters:
 *   a 32-bit pixel with 8-bit color components.
 *
 * Returned Value:
 *   A 32-bit gamma corrected pixel.
 *
 ****************************************************************************/

uint32_t ws2812_gamma_correct(uint32_t pixel);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_WS2812 */
#endif /* __INCLUDE_NUTTX_LEDS_WS2812_H */
