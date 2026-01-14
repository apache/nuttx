# ##############################################################################
# arch/arm/src/imx6/CMakeLists.txt
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

set(SRCS
    imx_boot.c
    imx_memorymap.c
    imx_clockconfig.c
    imx_irq.c
    imx_timerisr.c
    imx_gpio.c
    imx_iomuxc.c
    imx_serial.c
    imx_lowputc.c
    imx_idle.c)

if(CONFIG_SMP)
  list(APPEND SRCS imx_cpuboot.c)
endif()

if(CONFIG_IMX6_ECSPI)
  list(APPEND SRCS imx_ecspi.c)
endif()

if(CONFIG_IMX6_ENET)
  list(APPEND SRCS imx_enet.c)
endif()

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS imx_pgalloc.c)
endif()

target_sources(arch PRIVATE ${SRCS})
