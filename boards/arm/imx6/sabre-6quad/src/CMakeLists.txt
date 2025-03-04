# ##############################################################################
# boards/arm/imx6/sabre-6quad/src/CMakeLists.txt
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

set(SRCS imx_boardinit.c imx_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS imx_appinit.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS imx_autoleds.c)
else()
  list(APPEND SRCS imx_userleds.c)
endif()

target_sources(board PRIVATE ${SRCS})

# TODO: make this the default and then allow boards to redefine
set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/dramboot.ld")
