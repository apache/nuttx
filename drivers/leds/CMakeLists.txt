# ##############################################################################
# drivers/leds/CMakeLists.txt
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

if(CONFIG_USERLED)
  list(APPEND SRCS userled_upper.c)

  if(CONFIG_USERLED_LOWER)
    list(APPEND SRCS userled_lower.c)
  endif()
endif()

if(CONFIG_LEDS_APA102)
  list(APPEND SRCS apa102.c)
endif()

if(CONFIG_LEDS_MAX7219)
  list(APPEND SRCS max7219.c)
endif()

if(CONFIG_RGBLED)
  list(APPEND SRCS rgbled.c)
endif()

if(CONFIG_LP503X)
  list(APPEND SRCS lp503x.c)
endif()

if(CONFIG_PCA9635PW)
  list(APPEND SRCS pca9635pw.c)
endif()

if(CONFIG_NCP5623C)
  list(APPEND SRCS ncp5623c.c)
endif()

if(CONFIG_WS2812)
  list(APPEND SRCS ws2812.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
