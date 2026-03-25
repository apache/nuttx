# ##############################################################################
# arch/risc-v/src/hpm6750/CMakeLists.txt
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

# Specify our HEAD assembly file.  This will be linked as the first object file,
# so it will appear at address 0

list(
  APPEND
  SRCS
  hpm6750_head.S
  hpm6750_clockconfig.c
  hpm6750_irq.c
  hpm6750_irq_dispatch.c
  hpm6750_lowputc.c
  hpm6750_serial.c
  hpm6750_start.c
  hpm6750_timerisr.c)

target_sources(arch PRIVATE ${SRCS})
