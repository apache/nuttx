# ##############################################################################
# boards/arm/imxrt/fmu-v6xrt/src/CMakeLists.txt
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

set(SRCS imxrt_boot.c imxrt_flexspi_nor_boot.c imxrt_flexspi_nor_flash.c
         imxrt_clockconfig.c imxrt_bringup.c)

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
elseif(CONFIG_BOOT_RUNFROMISRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/flash-ocram.ld")
endif()
