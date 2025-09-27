# ##############################################################################
# arch/arm64/src/bcm2711/CMakeLists.txt
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

# BCM2711 specific C source files

set(SRCS bcm2711_boot.c bcm2711_mailbox.c bcm2711_serial.c bcm2711_gpio.c
         bcm2711_timer.c)

# Early boot logging

if(CONFIG_ARCH_EARLY_PRINT)
  list(APPEND SRCS bcm2711_lowputc.c)
endif()

# I2C interfaces

if(CONFIG_BCM2711_I2C)
  list(APPEND SRCS bcm2711_i2c.c)
endif()

# SPI interfaces
#
if(CONFIG_BCM2711_SPI)
  list(APPEND SRCS bcm2711_spi.c)
endif()

target_sources(arch PRIVATE ${SRCS})
