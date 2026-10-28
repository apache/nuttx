/****************************************************************************
 * include/nuttx/leds/ws2812.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdbool.h>
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

/* Although the LED might be RGB-only, the LED data is packed in a 32-bit
 * long variable and, then, this is the default size of a LED pixel.
 */

#define WS2812_RW_PIXEL_SIZE  4

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_WS2812_NON_SPI_DRIVER

#  ifdef CONFIG_WS2812_COMPONENT_TRANSFORM

enum ws2812_component_transform_e
{
  /* Supposedly, the LED expects color bytes in GRB order. However,
   * variant that expect RGB order exists as well. Upper-half can
   * transform the data from userspace (coming in RGB order)
   * so the lower-half (which expects GRB) does the correct thing.
   *
   * NONE transformation is always available and default
   * if the configuration struct is allocated with zalloc.
   */

  WS2812_COMPONENT_TRANSFORM_NONE = 0

#    ifdef CONFIG_WS2812_COMPONENT_TRANSFORM_RGB

  , WS2812_COMPONENT_TRANSFORM_RGB /* LED expects bytes in RGB order */

#    endif
};

#  endif /* CONFIG_WS2812_COMPONENT_TRANSFORM */

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

  FAR void              *private;    /* Private data for opened device */
  uint32_t               clock;
  mutex_t                lock;
  int                    port;
  uint16_t               nleds;      /* Count of WS2812s in the chain */
  bool                   has_white;  /* Set true if chain has LED with
                                      * white component */

#  ifdef CONFIG_WS2812_COMPONENT_TRANSFORM

  /* This is the byte order transformation the LED needs */

  enum ws2812_component_transform_e byte_order_transformation;

  /* This is a buffer used for the byte order transformation.
   * Must be allocated by the caller of ws2812_register and must have
   * enough room for WS2812_RW_PIXEL_SIZE * nleds bytes.
   *
   */

  FAR char *byte_order_buffer;

  /* Using single buffer per driver instance. Must prevent concurrent
   * access but without using the "lock" mutex above. Lower half
   * drivers are using that one. Must be initialized by caller
   * of ws2812_register as well.
   */

  mutex_t byte_order_lock;

#  endif
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

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_WS2812 */
#endif /* __INCLUDE_NUTTX_LEDS_WS2812_H */
