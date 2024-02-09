# ##############################################################################
# arch/arm/src/xmc4/CMakeLists.txt
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

list(
  APPEND
  SRCS
  xmc4_allocateheap.c
  xmc4_clockconfig.c
  xmc4_clockutils.c
  xmc4_clrpend.c
  xmc4_gpio.c
  xmc4_irq.c
  xmc4_lowputc.c
  xmc4_serial.c
  xmc4_start.c
  xmc4_usic.c
  xmc4_timerisr.c)

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS xmc4_userspace.c xmc4_mpuinit.c)
endif()

if(CONFIG_ECAT)
  list(APPEND SRCS xmc4_ecat.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS xmc4_idle.c)
endif()

if(CONFIG_XMC4_USCI_SPI)
  list(APPEND SRCS xmc4_spi.c)
endif()

target_sources(arch PRIVATE ${SRCS})
