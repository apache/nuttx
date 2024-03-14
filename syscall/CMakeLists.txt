# ##############################################################################
# syscall/CMakeLists.txt
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
# Parse CSV file to get expected output source files for each target
file(STRINGS syscall.csv SYSCALLS)
list(TRANSFORM SYSCALLS REPLACE "^\"([^,]+)\",.+" "\\1")
list(TRANSFORM SYSCALLS APPEND ".c")

# generate the host tool

add_custom_target(
  mksyscall
  COMMAND cmake -B ${CMAKE_BINARY_DIR}/bin -S ${CMAKE_SOURCE_DIR}/tools && cmake
          --build ${CMAKE_BINARY_DIR}/bin --target mksyscall
  SOURCES ${CMAKE_SOURCE_DIR}/tools/mksyscall.c)

if(CONFIG_LIB_SYSCALL)
  add_subdirectory(proxies)
  add_subdirectory(stubs)

  target_sources(stubs PRIVATE syscall_stublookup.c)
endif()

# TODO: should CONFIG_SCHED_INSTRUMENTATION_SYSCALL depend on
# CONFIG_LIB_SYSCALL?
if(CONFIG_SCHED_INSTRUMENTATION_SYSCALL)
  if(CONFIG_LIB_SYSCALL)
    target_sources(proxies PRIVATE syscall_names.c)
  else()
    target_sources(proxies PRIVATE syscall_names.c)
  endif()
endif()

if(CONFIG_SCHED_INSTRUMENTATION_SYSCALL)
  add_subdirectory(wraps)
endif()
