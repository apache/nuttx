# ##############################################################################
# drivers/ioexpander/CMakeLists.txt
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

# Check if I/O expander support is enabled

if(CONFIG_IOEXPANDER)
  # Include the selected I/O expander drivers

  if(CONFIG_IOEXPANDER_RPMSG)
    list(APPEND SRCS ioe_rpmsg.c)
  endif()

  if(CONFIG_IOEXPANDER_DUMMY)
    list(APPEND SRCS ioe_dummy.c)
  endif()

  if(CONFIG_IOEXPANDER_PCA9555)
    list(APPEND SRCS pca9555.c)
  endif()

  if(CONFIG_IOEXPANDER_PCA9538)
    list(APPEND SRCS pca9538.c)
  endif()

  if(CONFIG_IOEXPANDER_TCA64XX)
    list(APPEND SRCS tca64xx.c)
  endif()

  if(CONFIG_IOEXPANDER_PCF8574)
    list(APPEND SRCS pcf8574.c)
  endif()

  if(CONFIG_IOEXPANDER_PCF8575)
    list(APPEND SRCS pcf8575.c)
  endif()

  if(CONFIG_IOEXPANDER_MCP23X17)
    list(APPEND SRCS mcp23x17.c)
  endif()

  if(CONFIG_IOEXPANDER_SX1509)
    list(APPEND SRCS sx1509.c)
  endif()
endif()

# GPIO test device driver (independent of IOEXPANDERS)

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS gpio.c)

  if(CONFIG_GPIO_LOWER_HALF)
    list(APPEND SRCS gpio_lower_half.c)
  endif()
endif()

target_sources(drivers PRIVATE ${SRCS})
