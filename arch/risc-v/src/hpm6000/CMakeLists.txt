# ##############################################################################
# arch/risc-v/src/hpm6000/CMakeLists.txt
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

set(SRCS hpm_head.S)

list(
  APPEND
  SRCS
  hpm_clockconfig.c
  hpm_gpio.c
  hpm_ioc.c
  hpm_irq.c
  hpm_irq_dispatch.c
  hpm_lowputc.c
  hpm_serial.c
  hpm_start.c
  hpm_timerisr.c)

target_sources(arch PRIVATE ${SRCS})
