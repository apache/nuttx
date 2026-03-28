# ##############################################################################
# boards/risc-v/esp32p4/common/src/CMakeLists.txt
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

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)

  if(CONFIG_ESPRESSIF_ADC)
    list(APPEND SRCS esp_board_adc.c)
  endif()

  if(CONFIG_ESPRESSIF_LEDC)
    list(APPEND SRCS esp_board_ledc.c)
  endif()

  if(CONFIG_ESP_RMT)
    list(APPEND SRCS esp_board_rmt.c)
  endif()

  if(CONFIG_ESPRESSIF_SPI)
    list(APPEND SRCS esp_board_spi.c)
  endif()

  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp_board_spidev.c)
  endif()

  if(CONFIG_ESPRESSIF_SPI AND CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS esp_board_spislavedev.c)
  endif()

  if(CONFIG_I2C_DRIVER)
    list(APPEND SRCS esp_board_i2c.c)
  endif()

  if(CONFIG_ESPRESSIF_I2S)
    list(APPEND SRCS esp_board_i2s.c)
  endif()

  if(CONFIG_ESPRESSIF_SPIFLASH)
    list(APPEND SRCS esp_board_spiflash.c)
  endif()

  if(CONFIG_ESPRESSIF_TWAI)
    list(APPEND SRCS esp_board_twai.c)
  endif()

  if(CONFIG_SENSORS_BMP180)
    list(APPEND SRCS esp_board_bmp180.c)
  endif()

  if(CONFIG_ESP_MCPWM)
    list(APPEND SRCS esp_board_mcpwm.c)
  endif()

  if(CONFIG_ESP_PCNT)
    list(APPEND SRCS esp_board_pcnt.c)
  endif()

endif()

target_sources(board PRIVATE ${SRCS})
