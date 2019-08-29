/****************************************************************************
 * include/nuttx/sensors/bmi160.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI160_H
#define __INCLUDE_NUTTX_SENSORS_BMI160_H

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#if defined(CONFIG_SENSORS_BMI160) || defined(CONFIG_SENSORS_BMI160_SCU)

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
#define CONFIG_SENSORS_BMI160_I2C
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI160_SPI_MAXFREQUENCY 10000000

/* Configuration ************************************************************/

/* Power mode */

#define BMI160_PM_SUSPEND     (0x00)
#define BMI160_PM_NORMAL      (0x01)
#define BMI160_PM_LOWPOWER    (0x02)
#define BMI160_PM_FASTSTARTUP (0x03)

/* Output data rate */

#define BMI160_ACCEL_ODR_0_78HZ (0x01)
#define BMI160_ACCEL_ODR_1_56HZ (0x02)
#define BMI160_ACCEL_ODR_3_12HZ (0x03)
#define BMI160_ACCEL_ODR_6_25HZ (0x04)
#define BMI160_ACCEL_ODR_12_5HZ (0x05)
#define BMI160_ACCEL_ODR_25HZ   (0x06)
#define BMI160_ACCEL_ODR_50HZ   (0x07)
#define BMI160_ACCEL_ODR_100HZ  (0x08)
#define BMI160_ACCEL_ODR_200HZ  (0x09)
#define BMI160_ACCEL_ODR_400HZ  (0x0A)
#define BMI160_ACCEL_ODR_800HZ  (0x0B)
#define BMI160_ACCEL_ODR_1600HZ (0x0C)

/* IOCTL Commands ***********************************************************/

#define SNIOC_ENABLESC     _SNIOC(0x0001) /* Arg: uint8_t value */
#define SNIOC_READSC       _SNIOC(0x0002) /* Arg: int16_t* pointer */
#define SNIOC_SETACCPM     _SNIOC(0x0003) /* Arg: uint8_t value */
#define SNIOC_SETACCODR    _SNIOC(0x0004) /* Arg: uint8_t value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * struct 6-axis data
 ****************************************************************************/
struct accel_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gyro_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct accel_gyro_st_s
{
  struct gyro_t  gyro;
  struct accel_t accel;
  uint32_t sensor_time;
};

struct spi_dev_s;
struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_SENSORS_BMI160_SCU

#  ifdef CONFIG_SENSORS_BMI160_I2C
int bmi160_register(FAR const char *devpath, FAR struct i2c_master_s *dev);
#  else /* CONFIG_BMI160_SPI */
int bmi160_register(FAR const char *devpath, FAR struct spi_dev_s *dev);
#  endif

#else /* CONFIG_SENSORS_BMI160_SCU */

#  ifdef CONFIG_SENSORS_BMI160_I2C
int bmi160_init(FAR struct i2c_master_s *dev, int port);
int bmi160gyro_register(FAR const char *devpath, int minor,
                        FAR struct i2c_master_s *dev, int port);
int bmi160accel_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *dev, int port);
#  else /* CONFIG_SENSORS_BMI160_SPI */
int bmi160_init(FAR struct spi_dev_s *dev);
int bmi160gyro_register(FAR const char *devpath, int minor,
                        FAR struct spi_dev_s *dev);
int bmi160accel_register(FAR const char *devpath, int minor,
                         FAR struct spi_dev_s *dev);
#  endif

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_BMI160 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMI160_H */
