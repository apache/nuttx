# ##############################################################################
# boards/arm/nrf53/common/src/CMakeLists.txt
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

  if(CONFIG_NRF53_TIMER)
    list(APPEND SRCS nrf53_timer.c)
  endif()

  if(CONFIG_BOARDCTL_BOOT_IMAGE)
    list(APPEND SRCS nrf53_boot_image.c)
  endif()

  if(CONFIG_NRF53_PROGMEM)
    list(APPEND SRCS nrf53_progmem.c)
  endif()

  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS nrf53_reset.c)
  endif()

  target_sources(board PRIVATE ${SRCS})

endif()
