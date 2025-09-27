# ##############################################################################
# boards/arm64/bcm2711/raspberrypi-4b/src/CMakeLists.txt
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

set(SRCS rpi4b_boardinitialize.c rpi4b_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS rpi4b_autoleds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS rpi4b_appinit.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS rpi4b_gpio.c)
endif()

if(CONFIG_BCM2711_I2C_DRIVER)
  list(APPEND SRCS bcm2711_i2cdev.c)
endif()

target_sources(board PRIVATE ${SRCS})
