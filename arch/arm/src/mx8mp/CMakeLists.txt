# ##############################################################################
# arch/arm/src/mx8mp/CMakeLists.txt
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

# Required mx8mp files

list(
  APPEND
  SRCS
  mx8mp_start.c
  mx8mp_allocateheap.c
  mx8mp_iomuxc.c
  mx8mp_lowputc.c
  mx8mp_clrpend.c
  mx8mp_irq.c
  mx8mp_mpuinit.c
  mx8mp_ccm.c
  mx8mp_serial.c
  mx8mp_clockconfig.c
  mx8mp_gpio.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS mx8mp_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS mx8mp_timerisr.c)
endif()

if(CONFIG_I2C)
  list(APPEND SRCS mx8mp_i2c.c)
endif()

if(CONFIG_SPI)
  list(APPEND SRCS mx8mp_ecspi.c)
endif()

if(CONFIG_MX8MP_RPMSG)
  list(APPEND SRCS mx8mp_rptun.c mx8mp_ipc.c mx8mp_rsctable.c)
endif()

target_sources(arch PRIVATE ${SRCS})
