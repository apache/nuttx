/************************************************************************************
 * configs/bambino-200e/src/lpc43_max31855.c
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/sensors/max31855.h>

#include "bambino-200e.h"

#include <nuttx/spi/spi.h>


#include "lpc43_spi.h"
#include "lpc43_ssp.h"

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX31855)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc43_max31855initialize
 *
 * Description:
 *   Initialize and register the MAX31855 Temperature Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register.  E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31855
 *   devid   - Minor device number. E.g., 0, 1, 2, etc.
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int lpc43_max31855initialize(FAR const char *devpath, int bus, uint16_t devid)
{
  FAR struct spi_dev_s *spi;
  spi = lpc43_sspbus_initialize(bus);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize SSP%d\n", bus);
      return -ENODEV;
    }

  /* Then register the temperature sensor */

  int ret = max31855_register(devpath, spi, devid);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX31855\n");
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX31855 */
