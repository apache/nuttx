# ##############################################################################
# boards/risc-v/ab32vg1/ab32vg1-generic/src/CMakeLists.txt
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

set(SRCS ab32vg1_boot.c ab32vg1_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS ab32vg1_autoleds.c)
endif()

if(CONFIG_USERLED)
  list(APPEND SRCS ab32vg1_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS ab32vg1_buttons.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS ab32vg1_appinit.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS ab32vg1_reset.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
