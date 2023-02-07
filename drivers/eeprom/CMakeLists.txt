# ##############################################################################
# drivers/eeprom/CMakeLists.txt
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

if(CONFIG_EEPROM)
  set(SRCS)

  # Include the Microchip/Atmel/ST xx25xx driver for SPI
  if(CONFIG_SPI_EE_25XX)
    list(APPEND SRCS spi_xx25xx.c)
  endif()

  # Include the Microchip/Atmel/ST xx24xx driver for I2C
  if(CONFIG_I2C_EE_24XX)
    list(APPEND SRCS i2c_xx24xx.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
