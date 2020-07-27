/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bmi160_scu.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmi160.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_SENSORS_BMI160_SCU_SPI)
#include "cxd56_spi.h"
#else
#include "cxd56_i2c.h"
#endif

#ifdef CONFIG_SENSORS_BMI160_SCU_DECI_GYRO
#  define GYRO_NR_SEQS 3
#else
#  define GYRO_NR_SEQS 1
#endif

#ifdef CONFIG_SENSORS_BMI160_SCU_DECI_ACCEL
#  define ACCEL_NR_SEQS 3
#else
#  define ACCEL_NR_SEQS 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_SENSORS_BMI160_SCU)
#if defined(CONFIG_SENSORS_BMI160_SCU_SPI)

int board_bmi160_initialize(int bus)
{
  int ret;
  FAR struct spi_dev_s *spi;

  sninfo("Initializing BMI160..\n");

  /* Initialize spi device */

  spi = cxd56_spibus_initialize(bus);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize spi%d.\n", bus);
      return -ENODEV;
    }

  int i;

  ret = bmi160_init(spi);
  if (ret < 0)
    {
      snerr("Error initialize BMI160\n");
      return ret;
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < GYRO_NR_SEQS; i++)
    {
      ret = bmi160gyro_register("/dev/gyro", i, spi);
      if (ret < 0)
        {
          snerr("Error registering gyroscope. %d\n", ret);
          return ret;
        }
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < ACCEL_NR_SEQS; i++)
    {
      ret = bmi160accel_register("/dev/accel", i, spi);
      if (ret < 0)
        {
          snerr("Error registering accelerometer. %d\n", ret);
          return ret;
        }
    }

  return ret;
}

#else /* !CONFIG_SENSORS_BMI160_SCU_SPI */

int board_bmi160_initialize(int bus)
{
  int ret;
  FAR struct i2c_master_s *i2c;

  sninfo("Initializing BMI160..\n");

  /* Initialize i2c deivce */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  int i;

  ret = bmi160_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize BMI160\n");
      return ret;
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < GYRO_NR_SEQS; i++)
    {
      ret = bmi160gyro_register("/dev/gyro", i, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering gyroscope. %d\n", ret);
          return ret;
        }
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < ACCEL_NR_SEQS; i++)
    {
      ret = bmi160accel_register("/dev/accel", i, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering accelerometer. %d\n", ret);
          return ret;
        }
    }

  return ret;
}

#endif  /* CONFIG_SENSORS_BMI160_SCU_SPI */
#endif  /* CONFIG_SENSORS_BMI160_SCU */
