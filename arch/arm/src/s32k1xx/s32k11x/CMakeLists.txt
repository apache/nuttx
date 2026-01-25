# ##############################################################################
# arch/arm/src/s32k1xx/s32k11x/CMakeLists.txt
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

# Source file specific to the S32k11x family.

list(APPEND S32k11xSRCS s32k11x_irq.c s32k11x_clockmapping.c
     s32k11x_periphfeatures.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND S32k11xSRCS s32k11x_timerisr.c)
endif()

target_sources(arch PRIVATE ${S32k11xSRCS})
