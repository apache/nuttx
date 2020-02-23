/****************************************************************************
 * include/nuttx/eeprom/m25xx.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __INCLUDE_NUTTX_EEPROM_M25XX_H
#define __INCLUDE_NUTTX_EEPROM_M25XX_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DO NOT CHANGE ORDER, IT MATCHES CODE IN drivers/eeprom/spieeprom.c */

enum eeprom_25xx_e
{
  /* Microchip geometries */

  EEPROM_25xx010,
  EEPROM_25xx020,
  EEPROM_25xx040,
  EEPROM_25xx080A, /* 16 bytes pages */
  EEPROM_25xx080B, /* 32 bytes pages */
  EEPROM_25xx160A, /* 16 bytes pages */
  EEPROM_25xx160B, /* 32 bytes pages */
  EEPROM_25xx320,
  EEPROM_25xx640,
  EEPROM_25xx128,
  EEPROM_25xx256,
  EEPROM_25xx512,
  EEPROM_25xx1024,

  /* Atmel geometries */

  EEPROM_AT25010B,
  EEPROM_AT25020B,
  EEPROM_AT25040B,

  /* STM geometries */

  EEPROM_M95M02,

  /* Aliases (devices similar to previously defined ones) */

  EEPROM_AT25080B  = EEPROM_25xx080B,
  EEPROM_AT25160B  = EEPROM_25xx160B,
  EEPROM_AT25320B  = EEPROM_25xx320,
  EEPROM_AT25640B  = EEPROM_25xx640,
  EEPROM_AT25128B  = EEPROM_25xx128,
  EEPROM_AT225256B = EEPROM_25xx256,
  EEPROM_AT25512   = EEPROM_25xx512,
  EEPROM_AT25M02   = EEPROM_25xx1024,
  EEPROM_M95010    = EEPROM_25xx010,
  EEPROM_M95020    = EEPROM_25xx020,
  EEPROM_M95040    = EEPROM_25xx040,
  EEPROM_M95080    = EEPROM_25xx080B,
  EEPROM_M95160    = EEPROM_25xx160B,
  EEPROM_M95320    = EEPROM_25xx320,
  EEPROM_M95640    = EEPROM_25xx640,
  EEPROM_M95128    = EEPROM_25xx128,
  EEPROM_M95256    = EEPROM_25xx256,
  EEPROM_M95512    = EEPROM_25xx512,
  EEPROM_M95M01    = EEPROM_25xx1024,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_initialize
 *
 * Description: Bind a EEPROM driver to an SPI bus. The user MUST provide
 * a description of the device geometry, since it is not possible to read
 * this information from the device (contrary to the SPI flash devices).
 *
 ****************************************************************************/

struct spi_dev_s;
int ee25xx_initialize(FAR struct spi_dev_s *dev, FAR char *devname,
                      int devtype, int readonly);

#endif /* __INCLUDE__NUTTX_EEPROM_M25XX_H */
