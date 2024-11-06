# ##############################################################################
# boards/arm/cxd56xx/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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

if(CONFIG_ARCH_BOARD_COMMON)
  set(SRCS)

  list(APPEND SRCS src/cxd56_boot.c)

  if(CONFIG_AUDIO_CXD56 OR CONFIG_CXD56_AUDIO)
    list(APPEND SRCS src/cxd56_audio.c)
  endif()

  if(CONFIG_MODEM_ALT1250)
    list(APPEND SRCS src/cxd56_alt1250.c)
  endif()

  if(CONFIG_BOARDCTL_UNIQUEID)
    list(APPEND SRCS src/cxd56_uid.c)
  endif()

  if(CONFIG_CXD56_BACKUPLOG)
    list(APPEND SRCS src/cxd56_crashdump.c)
  endif()

  if(CONFIG_SENSORS)
    list(APPEND SRCS src/cxd56_sensors.c)
  endif()

  if(CONFIG_SENSORS_AK09912)
    list(APPEND SRCS src/cxd56_ak09912_i2c.c)
  endif()

  if(CONFIG_SENSORS_AK09912_SCU)
    list(APPEND SRCS src/cxd56_ak09912_scu.c)
  endif()

  if(CONFIG_SENSORS_APDS9930_SCU)
    list(APPEND SRCS src/cxd56_apds9930_scu.c)
  endif()

  if(CONFIG_SENSORS_APDS9960)
    list(APPEND SRCS src/cxd56_apds9960_i2c.c)
  endif()

  if(CONFIG_SENSORS_BH1721FVC_SCU)
    list(APPEND SRCS src/cxd56_bh1721fvc_scu.c)
  endif()

  if(CONFIG_SENSORS_BH1745NUC_SCU)
    list(APPEND SRCS src/cxd56_bh1745nuc_scu.c)
  endif()

  if(CONFIG_SENSORS_BM1383GLV_SCU)
    list(APPEND SRCS src/cxd56_bm1383glv_scu.c)
  endif()

  if(CONFIG_SENSORS_BM1422GMV_SCU)
    list(APPEND SRCS src/cxd56_bm1422gmv_scu.c)
  endif()

  if(CONFIG_SENSORS_BMI160_I2C)
    list(APPEND SRCS src/cxd56_bmi160_i2c.c)
  endif()

  if(CONFIG_SENSORS_BMI270_I2C)
    list(APPEND SRCS src/cxd56_bmi270_i2c.c)
  endif()

  if(CONFIG_SENSORS_BMI160_SCU)
    list(APPEND SRCS src/cxd56_bmi160_scu.c)
  endif()

  if(CONFIG_SENSORS_BMI160_SPI)
    list(APPEND SRCS src/cxd56_bmi160_spi.c)
  endif()

  if(CONFIG_SENSORS_BMP280)
    list(APPEND SRCS src/cxd56_bmp280_i2c.c)
  endif()

  if(CONFIG_SENSORS_BMP280_SCU)
    list(APPEND SRCS src/cxd56_bmp280_scu.c)
  endif()

  if(CONFIG_SENSORS_KX022_SCU)
    list(APPEND SRCS src/cxd56_kx022_scu.c)
  endif()

  if(CONFIG_SENSORS_LT1PA01_SCU)
    list(APPEND SRCS src/cxd56_lt1pa01_scu.c)
  endif()

  if(CONFIG_SENSORS_RPR0521RS_SCU)
    list(APPEND SRCS src/cxd56_rpr0521rs_scu.c)
  endif()

  if(CONFIG_SENSORS_SCD41)
    list(APPEND SRCS src/cxd56_scd41_i2c.c)
  endif()

  if(CONFIG_NETDEVICES)
    list(APPEND SRCS src/cxd56_netinit.c)
  endif()

  if(CONFIG_WL_GS2200M)
    list(APPEND SRCS src/cxd56_gs2200m.c)
  endif()

  if(CONFIG_LCD_ILI9340)
    list(APPEND SRCS src/cxd56_ili9340.c)
  endif()

  if(CONFIG_LCD_LPM013M091A)
    list(APPEND SRCS src/cxd56_lpm013m091a.c)
  endif()

  if(CONFIG_CXD56_SFC)
    list(APPEND SRCS src/cxd56_flash.c)
  endif()

  if(CONFIG_USBMSC)
    list(APPEND SRCS src/cxd56_usbmsc.c)
  endif()

  if(CONFIG_CXD56_I2C_DRIVER)
    list(APPEND SRCS src/cxd56_i2cdev.c)
  endif()

  if(CONFIG_I2C_BITBANG)
    list(APPEND SRCS src/cxd56_i2cdev_bitbang.c)
  endif()

  if(CONFIG_CXD56_SPI_DRIVER)
    list(APPEND SRCS src/cxd56_spidev.c)
  endif()

  if(CONFIG_VIDEO_ISX012)
    list(APPEND SRCS src/cxd56_isx012.c)
  endif()

  if(CONFIG_VIDEO_ISX019)
    list(APPEND SRCS src/cxd56_isx019.c)
  endif()

  if(CONFIG_CXD56_IMAGEPROC)
    list(APPEND SRCS src/cxd56_imageproc.c)
  endif()

  if(CONFIG_BCM20706)
    list(APPEND SRCS src/cxd56_bcm20706.c)
  endif()

  if(CONFIG_CXD56_EMMC)
    list(APPEND SRCS src/cxd56_emmcdev.c)
  endif()

  if(CONFIG_CXD56_SPISD)
    list(APPEND SRCS src/cxd56_spisd.c)
  endif()

  if(CONFIG_BOARD_USBDEV_SERIALSTR)
    list(APPEND SRCS src/cxd56_usbdevserialstr.c)
  endif()

  if(CONFIG_PM)
    list(APPEND SRCS src/cxd56_pm.c)
  endif()

  if(CONFIG_CXD56_GNSS_ADDON)
    list(APPEND SRCS src/cxd56_gnss_addon.c)
  endif()

  target_sources(board PRIVATE ${SRCS})
endif()
