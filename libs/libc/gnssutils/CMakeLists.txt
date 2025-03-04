# ##############################################################################
# libs/libc/gnssutils/CMakeLists.txt
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
if(CONFIG_GNSSUTILS_MINMEA_LIB)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/minmea)

    set(MINMEA_VERSION db46128e73cee26d6a6eb0482dcba544ee1ea9f5)

    FetchContent_Declare(
      minmea
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/kosma/minmea/archive/${MINMEA_VERSION}.zip"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/minmea
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libc/gnssutils/minmea
          CONFIGURE_COMMAND
          ""
          BUILD_COMMAND
          ""
          INSTALL_COMMAND
          ""
          TEST_COMMAND
          ""
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(minmea)

    if(NOT minmea_POPULATED)
      FetchContent_Populate(minmea)
    endif()
  endif()

  if(NOT EXISTS ${CMAKE_BINARY_DIR}/include/minmea)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/minmea)
  endif()

  configure_file(${CMAKE_CURRENT_LIST_DIR}/minmea/minmea.h
                 ${CMAKE_BINARY_DIR}/include/minmea/minmea.h COPYONLY)

  target_sources(c PRIVATE minmea/minmea.c)

endif()
