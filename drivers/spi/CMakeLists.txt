# ##############################################################################
# drivers/spi/CMakeLists.txt
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

if(CONFIG_SPI)
  set(SRCS)

  if(CONFIG_SPI_EXCHANGE)
    list(APPEND SRCS spi_transfer.c)

    if(CONFIG_SPI_DRIVER)
      list(APPEND SRCS spi_driver.c)
    endif()
  endif()

  if(CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS spi_slave_driver.c)
  endif()

  # Include the selected SPI drivers

  if(CONFIG_SPI_BITBANG)
    list(APPEND SRCS spi_bitbang.c)
  endif()

  if(CONFIG_SPI_FLASH)
    list(APPEND SRCS spi_flash.c)
  endif()

  if(CONFIG_QSPI_FLASH)
    list(APPEND SRCS qspi_flash.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
  target_include_directories(drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()
