# ##############################################################################
# boards/arm/kinetis/twr-k64f120m/src/CMakeLists.txt
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

set(SRCS k64_boot.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS k64_leds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS k64_appinit.c)
endif()

if(CONFIG_KINETIS_SDHC)
  list(APPEND SRCS k64_sdhc.c)
  if(CONFIG_FS_AUTOMOUNTER)
    list(APPEND SRCS k64_automount.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
