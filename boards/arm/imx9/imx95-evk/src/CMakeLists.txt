# ##############################################################################
# boards/arm/imx9/imx95-evk/src/CMakeLists.txt
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

set(SRCS imx95_boardinit.c imx95_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS imx95_appinit.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS imx95_pwm.c)
endif()

if(CONFIG_IMX9_LPI2C)
  list(APPEND SRCS imx95_i2c.c)
endif()

if(CONFIG_IMX9_LPSPI)
  list(APPEND SRCS imx95_spi.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_IMX95_RUN_FROM_ITCM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/itcm.ld")
elseif(CONFIG_IMX95_RUN_FROM_DDR)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ddr.ld")
elseif(CONFIG_IMX95_RUN_FROM_FLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
endif()
