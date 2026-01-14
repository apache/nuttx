# ##############################################################################
# boards/sim/sim/sim/CMakeLists.txt
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

add_subdirectory(src)

if(CONFIG_HOST_LINUX)
  add_custom_target(
    nuttx_post_build
    DEPENDS nuttx
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory sim-pac
    COMMAND ${CMAKE_COMMAND} -E make_directory sim-pac/libs
    COMMAND ${CMAKE_COMMAND} -E copy nuttx sim-pac/nuttx
    COMMAND ldd sim-pac/nuttx | grep \"=> /\" | awk '{print $$3}' | xargs -I
            '{}' cp -v '{}' sim-pac/libs
    COMMAND readelf -l nuttx | grep "program interpreter" | awk -F':' '{print
                $$2}' | cut -d"]" -f1 | xargs -I '{}' cp -v '{}' sim-pac
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/tools/simlaunch.sh
            sim-pac
    COMMAND tar -czf nuttx.tgz sim-pac
    COMMENT "Pac SIM with dynamic libs in nuttx.tgz")
endif()
