# ##############################################################################
# drivers/sensors/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_SENSORS)
  set(SRCS sensor.c)

  if(CONFIG_USENSOR)
    list(APPEND SRCS usensor.c)
  endif()

  if(CONFIG_SENSORS_RPMSG)
    list(APPEND SRCS sensor_rpmsg.c)
  endif()

  if(CONFIG_SENSORS_GPS)
    set_source_files_properties(
      gps_uorb.c DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
      PROPERTIES INCLUDE_DIRECTORIES ${NUTTX_DIR}/libs/libc/gpsutils)
    list(APPEND SRCS gps_uorb.c)
  endif()

  if(CONFIG_SENSORS_WTGAHRS2)
    list(APPEND SRCS wtgahrs2_uorb.c)
  endif()

  if(CONFIG_SENSORS_FAKESENSOR)
    list(APPEND SRCS fakesensor_uorb.c)
  endif()

  if(CONFIG_SENSORS_GOLDFISH_GPS)
    list(APPEND SRCS goldfish_gps_uorb.c)
  endif()

  if(CONFIG_SENSORS_HCSR04)
    list(APPEND SRCS hc_sr04.c)
  endif()

  if(CONFIG_SENSORS_ADXL345)
    list(APPEND SRCS adxl345_base.c)
  endif()

  if(CONFIG_SENSORS_ADXL362)
    list(APPEND SRCS adxl362_uorb.c)
  endif()

  if(CONFIG_SENSORS_BH1749NUC)
    list(APPEND SRCS bh1749nuc_base.c)
    if(CONFIG_SENSORS_BH1749NUC_UORB)
      list(APPEND SRCS bh1749nuc_uorb.c)
    else()
      list(APPEND SRCS bh1749nuc.c)
    endif()
  endif()

  if(CONFIG_SENSORS_DHTXX)
    list(APPEND SRCS dhtxx.c)
  endif()

  if(CONFIG_SENSORS_BMI270)
    list(APPEND SRCS bmi270_base.c)
    if(CONFIG_SENSORS_BMI270_UORB)
      list(APPEND SRCS bmi270_uorb.c)
    else()
      list(APPEND SRCS bmi270.c)
    endif()
  endif()

  # These drivers depend on I2C support

  if(CONFIG_I2C)

    if(CONFIG_SENSORS_APDS9960)
      list(APPEND SRCS apds9960.c)
    endif()

    if(CONFIG_SENSORS_AK09912)
      list(APPEND SRCS ak09912.c)
    endif()

    if(CONFIG_SENSORS_AS5048B)
      list(APPEND SRCS as5048b.c)
    endif()

    if(CONFIG_SENSORS_AS726X)
      list(APPEND SRCS as726x.c)
    endif()

    if(CONFIG_SENSORS_FXOS8700CQ)
      list(APPEND SRCS fxos8700cq.c)
    endif()

    if(CONFIG_SENSORS_HYT271)
      list(APPEND SRCS hyt271_uorb.c)
    endif()

    if(CONFIG_SENSORS_KXTJ9)
      list(APPEND SRCS kxtj9.c)
    endif()

    if(CONFIG_SENSORS_LIS2DH)
      list(APPEND SRCS lis2dh.c)
    endif()

    if(CONFIG_LIS331DL)
      list(APPEND SRCS lis331dl.c)
    endif()

    if(CONFIG_SENSORS_LSM303AGR)
      list(APPEND SRCS lsm303agr.c)
    endif()

    if(CONFIG_SENSORS_LSM6DSL)
      list(APPEND SRCS lsm6dsl.c)
    endif()

    if(CONFIG_SENSORS_LSM9DS1)
      list(APPEND SRCS lsm9ds1_base.c)
      if(CONFIG_SENSORS_LSM9DS1_UORB)
        list(APPEND SRCS lsm9ds1_uorb.c)
      else()
        list(APPEND SRCS lsm9ds1.c)
      endif()
    endif()

    if(CONFIG_SENSORS_MSA301)
      list(APPEND SRCS msa301.c)
    endif()

    if(CONFIG_SENSORS_LPS25H)
      list(APPEND SRCS lps25h.c)
    endif()

    if(CONFIG_ADXL345_I2C)
      list(APPEND SRCS adxl345_i2c.c)
    endif()

    if(CONFIG_SENSORS_BH1750FVI)
      list(APPEND SRCS bh1750fvi.c)
    endif()

    if(CONFIG_SENSORS_BMG160)
      list(APPEND SRCS bmg160.c)
    endif()

    if(CONFIG_SENSORS_BMI160)
      list(APPEND SRCS bmi160_base.c)
      if(CONFIG_SENSORS_BMI160_UORB)
        list(APPEND SRCS bmi160_uorb.c)
      else()
        list(APPEND SRCS bmi160.c)
      endif()
    endif()

    if(CONFIG_SENSORS_BMP180)
      list(APPEND SRCS bmp180.c)
    endif()

    if(CONFIG_SENSORS_BMP280)
      list(APPEND SRCS bmp280_uorb.c)
    endif()

    if(CONFIG_SENSORS_BME680)
      list(APPEND SRCS bme680_uorb.c)
    endif()

    if(CONFIG_SENSORS_ISL29023)
      list(APPEND SRCS isl29023.c)
    endif()

    if(CONFIG_SENSORS_HTS221)
      list(APPEND SRCS hts221.c)
    endif()

    if(CONFIG_LM75_I2C)
      list(APPEND SRCS lm75.c)
    endif()

    if(CONFIG_SENSORS_LM92)
      list(APPEND SRCS lm92.c)
    endif()

    if(CONFIG_SENSORS_MAX44009)
      list(APPEND SRCS max44009.c)
    endif()

    if(CONFIG_SENSORS_MB7040)
      list(APPEND SRCS mb7040.c)
    endif()

    if(CONFIG_SENSORS_MCP9844)
      list(APPEND SRCS mcp9844.c)
    endif()

    if(CONFIG_SENSORS_MLX90393)
      list(APPEND SRCS mlx90393.c)
    endif()

    if(CONFIG_SENSORS_MLX90614)
      list(APPEND SRCS mlx90614.c)
    endif()

    if(CONFIG_SENSORS_MS56XX)
      list(APPEND SRCS ms56xx_uorb.c)
    endif()

    if(CONFIG_SENSORS_MS58XX)
      list(APPEND SRCS ms58xx.c)
    endif()

    if(CONFIG_SENSORS_LTC4151)
      list(APPEND SRCS ltc4151.c)
    endif()

    if(CONFIG_SENSORS_INA219)
      list(APPEND SRCS ina219.c)
    endif()

    if(CONFIG_SENSORS_INA226)
      list(APPEND SRCS ina226.c)
    endif()

    if(CONFIG_SENSORS_INA3221)
      list(APPEND SRCS ina3221.c)
    endif()

    if(CONFIG_SENSORS_SCD30)
      list(APPEND SRCS scd30.c)
    endif()

    if(CONFIG_SENSORS_SCD41)
      list(APPEND SRCS scd41.c)
    endif()

    if(CONFIG_SENSORS_SGP30)
      list(APPEND SRCS sgp30.c)
    endif()

    if(CONFIG_SENSORS_AHT10)
      list(APPEND SRCS aht10.c)
    endif()

    if(CONFIG_SENSORS_SHT21)
      list(APPEND SRCS sht21.c)
    endif()

    if(CONFIG_SENSORS_SHT3X)
      list(APPEND SRCS sht3x.c)
    endif()

    if(CONFIG_SENSORS_SPS30)
      list(APPEND SRCS sps30.c)
    endif()

    if(CONFIG_SENSORS_T67XX)
      list(APPEND SRCS t67xx.c)
    endif()

    if(CONFIG_SENSORS_LTR308)
      list(APPEND SRCS ltr308_uorb.c)
    endif()

    if(CONFIG_SENSORS_BMM150)
      list(APPEND SRCS bmm150_uorb.c)
    endif()

  endif() # CONFIG_I2C

  # These drivers depend on SPI support

  if(CONFIG_SPI)

    if(CONFIG_ADXL345_SPI)
      list(APPEND SRCS adxl345_spi.c)
    endif()

    if(CONFIG_SENSORS_ADXL372)
      if(CONFIG_SENSORS_ADXL372_UORB)
        list(APPEND SRCS adxl372_uorb.c)
      else()
        list(APPEND SRCS adxl372.c)
      endif()
    endif()

    if(CONFIG_LIS3DSH)
      list(APPEND SRCS lis3dsh.c)
    endif()

    if(CONFIG_LIS3DH)
      list(APPEND SRCS lis3dh.c)
    endif()

    if(CONFIG_SENSORS_MAX31855)
      list(APPEND SRCS max31855.c)
    endif()

    if(CONFIG_SENSORS_MAX6675)
      list(APPEND SRCS max6675.c)
    endif()

    if(CONFIG_SENSORS_MPL115A)
      list(APPEND SRCS mpl115a.c)
    endif()

    if(CONFIG_SENSORS_LIS3MDL)
      list(APPEND SRCS lis3mdl.c)
    endif()

    if(CONFIG_SENSORS_LSM330SPI)
      list(APPEND SRCS lsm330_spi.c)
    endif()

    if(CONFIG_SENSORS_L3GD20)
      list(APPEND SRCS l3gd20_uorb.c)
    endif()

    if(CONFIG_SENSORS_ADT7320)
      list(APPEND SRCS adt7320.c)
    endif()

    if(CONFIG_SENSORS_AS5048A)
      list(APPEND SRCS as5048a.c)
    endif()

  endif() # CONFIG_SPI

  # These drivers depend on 1WIRE support

  if(CONFIG_1WIRE)

    if(CONFIG_SENSORS_DS18B20)
      list(APPEND SRCS ds18b20_uorb.c)
    endif()

  endif() # CONFIG_1WIRE

  if(CONFIG_SENSORS_MPU60X0)
    list(APPEND SRCS mpu60x0.c)
  endif()

  if(CONFIG_SENSORS_MPU9250)
    list(APPEND SRCS mpu9250_uorb.c)
  endif()

  # Quadrature encoder upper half

  if(CONFIG_SENSORS_QENCODER)
    list(APPEND SRCS qencoder.c)
  endif()

  # 3-phase Hall effect sensor upper half

  if(CONFIG_SENSORS_HALL3PHASE)
    list(APPEND SRCS hall3ph.c)
  endif()

  # Vishay VEML6070

  if(CONFIG_SENSORS_VEML6070)
    list(APPEND SRCS veml6070.c)
  endif()

  # ST VL53L1X

  if(CONFIG_SENSORS_VL53L1X)
    list(APPEND SRCS vl53l1x.c)
  endif()

  # Sensixs XEN1210

  if(CONFIG_SENSORS_XEN1210)
    list(APPEND SRCS xen1210.c)
  endif()

  # Zero Cross upper half

  if(CONFIG_SENSORS_ZEROCROSS)
    list(APPEND SRCS zerocross.c)
  endif()

  # TI HDC1008

  if(CONFIG_SENSORS_HDC1008)
    list(APPEND SRCS hdc1008.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
