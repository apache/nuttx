/****************************************************************************
 * include/nuttx/leds/max7219.h
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

#ifndef __INCLUDE_NUTTX_LEDS_MAX7219_H
#define __INCLUDE_NUTTX_LEDS_MAX7219_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_SPI - Enables support for SPI drivers
 * CONFIG_LEDS_MAX7219 - Enables support for the MAX7219 driver
 */

#if defined(CONFIG_SPI) && defined(CONFIG_LEDS_MAX7219)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MAX7219 register addresses */

#define MAX7219_NOOP          (0x00)   /* No Operation          */
#define MAX7219_DIGIT0        (0x01)   /* Digit 0 register      */
#define MAX7219_DIGIT1        (0x02)   /* Digit 1 register      */
#define MAX7219_DIGIT2        (0x03)   /* Digit 2 register      */
#define MAX7219_DIGIT3        (0x04)   /* Digit 3 register      */
#define MAX7219_DIGIT4        (0x05)   /* Digit 4 register      */
#define MAX7219_DIGIT5        (0x06)   /* Digit 5 register      */
#define MAX7219_DIGIT6        (0x07)   /* Digit 6 register      */
#define MAX7219_DIGIT7        (0x08)   /* Digit 7 register      */
#define MAX7219_DECODE_MODE   (0x09)   /* Decode Moder register */
#define MAX7219_INTENSITY     (0x0a)   /* Intensity register    */
#define MAX7219_SCAN_LIMIT    (0x0b)   /* Scan Limit register   */
#define MAX7219_SHUTDOWN      (0x0c)   /* Shutdown register     */
#define MAX7219_DISPLAY_TEST  (0x0f)   /* Display Test register */

/* Default register values */

#define ENABLE_DECODE         (0xff)    /* Enable decoding of all 7-seg */
#define DISABLE_DECODE        (0x00)    /* Disable 7-seg decode         */
#define DEFAULT_SCAN_LIMIT    (0x07)    /* Display all digits           */
#define DISPLAY_INTENSITY(n)  (n & 0xf) /* low nimble defines it        */

/* Only two power settings are supported: */

#define MAX7219_POWER_OFF     0
#define MAX7219_POWER_ON      1

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
 * Name: max7219_leds_register
 *
 * Description:
 *   Initialize the MAX7219 device as a LEDS interface.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use to communicate
 *           with the MAX7219.
 *   devno - Device number to identify current display.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct spi_dev_s;  /* Forward reference */

int max7219_leds_register(FAR const char *devpath,
                          FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_MAX7219 */
#endif /* __INCLUDE_NUTTX_LEDS_MAX7219_H */
