/****************************************************************************
 * boards/avr/at90usb/teensy-2.0/src/at90usb_spi.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <avr/io.h>

#include "up_arch.h"
#include "chip.h"
#include "at90usb.h"
#include "teensy-20.h"

#ifdef CONFIG_AVR_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Teensy SPI Connection
 *
 * -- ---- -- ------------------------- -------
 * J2 NAME PIN NAME                     PAD
 * -- ---- -- ------------------------- -------
 *  1 VIN  -- Connected to USB +RV
 *  2 GND  -- Connected to USB GND
 *  3 3V3  -- Not used                  ---
 *  4 NC   -- Not used
 *  5 CS   10 (SS/PCINT0) PB0           Pad B0
 *  6 DI   12 (PDI/PCINT2/MOSI) PB2     Pad B2
 *  7 SCK  11 (PCINT1/SCLK) PB1         Pad B1
 *  8 DO   13 (PDO/PCINT3/MISO) PB3     Pad B3
 *  9 IRQ  -- Not used                  ---
 * 10 CD   14 (PCINT4/OC.2A) PB4        Pad B4
 * 11 WP   15 (PCINT5/OC.1A) PB5        Pad B5
 * -- ---- -- ------------------------- -------
 */

#define TEENSY_CS (1 << 0)
#define TEENSY_CD (1 << 4)
#define TEENSY_WP (1 << 5)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at90usb_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC1766-STK.
 *
 ****************************************************************************/

void weak_function at90usb_spidev_initialize(void)
{
  /* The Teensy board has no dedicated SPI devices so we assume that SS is
   * used for chip select:
   *
   *   "When the SPI is configured as a Master (MSTR in SPCR is set), the
   *    user can determine the direction of the SS pin.
   *    If SS is configured as an output, the pin is a general output pin
   *    which does not affect the SPI system. ...
   *
   *   "If SS is configured as an input, it must be held high to ensure
   *    Master SPI operation. If the SS pin is driven low by peripheral
   *    circuitry when the SPI is configured as a Master with the SS pin
   *    defined as an input, the SPI system interprets this as another
   *    master selecting the SPI ...
   */

  DDRB  |= TEENSY_CS;                 /* B0 is an output */
  PORTB |= TEENSY_CS;                 /* Low de-selects */
  DDRB  &= ~(TEENSY_CD | TEENSY_WP);  /* B4 and B5 are inputs */
  PORTB |= (TEENSY_CD | TEENSY_WP);   /* Pull high */
}

/****************************************************************************
 * Name:  avr_spiselect and avr_spistatus
 *
 * Description:
 *   The external functions, avr_spiselect and avr_spistatus  must be
 *   provided by board-specific logic.  They are implementations of the
 *   select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including avr_spibus_initialize()) are provided by
 *   common AVR logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in avr_spidev_initialize() to configure SPI
 *      chip select pins.
 *   2. Provide avr_spiselect() and avr_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations in the way your board is configured.
 *   3. Add a calls to at90usb_spidev_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by avr_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic
 *      (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 *
 ****************************************************************************/

void  avr_spiselect(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
           selected ? "assert" : "de-assert");

  /* Assert/de-assert the CS pin to the card */

  if (selected)
    {
       PORTB &= ~TEENSY_CS;
    }
  else
    {
       PORTB |= TEENSY_CS;
    }
}

uint8_t avr_spistatus(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;
  uint8_t regval = PINB;

  /* Both the CD and WP pins are pull high by the AT90USB and will be
   * grounded it a card is inserted or write protected.
   */

  if ((regval & TEENSY_CD) == 0)
    {
      ret |= SPI_STATUS_PRESENT;
    }

  if ((regval & TEENSY_WP) == 0)
    {
      ret |= SPI_STATUS_WRPROTECTED;
    }

  spiinfo("Returning %02x\n", ret);
  return ret;
}

#endif /* CONFIG_AVR_SPI */
