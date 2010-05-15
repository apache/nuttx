/****************************************************************************
 * config/lm3s6965-ek/src/up_oled.c
 * arch/arm/src/board/up_oled.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/p14201.h>

#include "lm3s_internal.h"
#include "lm3s6965ek_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: up_nxdrvinit
 *
 * Description:
 *   Called NX initialization logic to configure the OLED.
 *
 ************************************************************************************/

FAR struct lcd_dev_s *up_nxdrvinit(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

  /* Configure the OLED D/Cn GPIO */

  lm3s_configgpio(OLEDDC_GPIO);

  /* Get the SPI port */

  spi = up_spiinitialize(0);
  if (!spi)
    {
      glldbg("Failed to initialize SPI port 0\n");
    }
  else
    {
      /* Bind the SPI port to the OLED */

      dev = rit_initialize(spi, devno);
      if (!dev)
        {
          glldbg("Failed to bind SPI port 0 to OLED %d: %d\n", devno);
        }
     else
        {
          gllvdbg("Bound SPI port 0 to OLED %d\n", devno);
          return dev;
        }
    }
  return NULL;
}

/**************************************************************************************
 * Name:  rit_seldata
 *
 * Description:
 *   Set or clear the SD1329 D/Cn bit to select data (true) or command (false).  This
 *   function must be provided by platform-specific logic.
 *
 * Input Parameters:
 *
 *   devno - A value in the range of 0 throuh CONFIG_P14201_NINTERFACES-1.  This allows
 *   support for multiple OLED devices.
 *   data - true: select data; false: select command
 *
 * Returned Value:
 *   None
 *
 **************************************************************************************/

void rit_seldata(unsigned int devno, bool data)
{
  /* Set GPIO to 1 for data, 0 for command */

  lm3s_gpiowrite(OLEDDC_GPIO, data);
}
