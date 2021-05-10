############################################################################
# libs/libxx/libcxx.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
###########################################################################

set(UCLIBCXX_VERSION 0.2.5)

FetchContent_Declare(uclibcxx
  URL https://git.busybox.net/uClibc++/snapshot/uClibc++-${UCLIBCXX_VERSION}.tar.gz
	#$(Q) $(COPYFILE) $(CURDIR)/system_configuration.h $(TOPDIR)/include/uClibc++
)
FetchContent_Populate(uclibcxx)
FetchContent_GetProperties(uclibcxx SOURCE_DIR UCLIBCXX_SOURCE_DIR)

set(SRCS
  algorithm.cpp associative_base.cpp bitset.cpp char_traits.cpp
  complex.cpp del_op.cpp del_opnt.cpp del_ops.cpp del_opv.cpp
  del_opvnt.cpp del_opvs.cpp deque.cpp exception.cpp fstream.cpp
  func_exception.cpp iomanip.cpp ios.cpp iostream.cpp istream.cpp
  iterator.cpp limits.cpp list.cpp locale.cpp map.cpp new_handler.cpp
  new_op.cpp new_opnt.cpp new_opv.cpp new_opvnt.cpp numeric.cpp
  ostream.cpp queue.cpp set.cpp sstream.cpp stack.cpp stdexcept.cpp
  streambuf.cpp string.cpp typeinfo.cpp utility.cpp valarray.cpp
  vector.cpp)

list(TRANSFORM SRCS PREPEND ${UCLIBCXX_SOURCE_DIR}/src/)
target_sources(xx PRIVATE ${SRCS})

execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${UCLIBCXX_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include_cxx
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/system_configuration.h ${CMAKE_BINARY_DIR}/include
)

target_include_directories(xx PRIVATE ${CMAKE_CURRENT_LIST_DIR})
