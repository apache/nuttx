/****************************************************************************
 * include/nuttx/leds/max7219.h
 *
 *   Copyright (C) 2018 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
int max7219_leds_register(FAR const char *devpath, FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_MAX7219 */
#endif /* __INCLUDE_NUTTX_LEDS_MAX7219_H */
