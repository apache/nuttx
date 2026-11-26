# ##############################################################################
# boards/xtensa/esp32s3/esp32s3-r8/src/CMakeLists.txt
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
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS esp32s3_boot.c esp32s3_bringup.c)

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS esp32s3_reset.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32s3_buttons.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32s3_gpio.c)
endif()

if(CONFIG_ESP32S3_SPI)
  list(APPEND SRCS esp32s3_board_spi.c)
endif()

if(CONFIG_ESP32S3_R8_LCD)
  list(APPEND SRCS esp32s3_board_lcd.c)
endif()

target_sources(board PRIVATE ${SRCS})
