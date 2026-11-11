# ##############################################################################
# arch/arm/src/rzv2h/CMakeLists.txt
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
    rzv2h_start.c
    rzv2h_irq.c
    rzv2h_timerisr.c
    rzv2h_lowputc.c
    rzv2h_clockconfig.c
    rzv2h_mpu_region.c
    rzv2h_fsp_err.c)

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS rzv2h_gpio.c)
endif()

if(CONFIG_RZV2H_SCI_B)
  list(APPEND SRCS rzv2h_serial.c)
endif()

target_sources(arch PRIVATE ${SRCS})
