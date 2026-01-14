# ##############################################################################
# boards/arm/tiva/launchxl-cc1312r1/src/CMakeLists.txt
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

set(SRCS cc1312_boot.c cc1312_pinconfig.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS cc1312_appinit.c cc1312_bringup.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS cc1312_bringup.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS cc1312_autoleds.c)
else()
  list(APPEND SRCS cc1312_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS cc1312_buttons.c)
endif()

if(CONFIG_TIVA_SSI)
  list(APPEND SRCS cc1312_ssi.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/sram.ld")
endif()
