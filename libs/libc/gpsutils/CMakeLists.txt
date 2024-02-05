# ##############################################################################
# libs/libc/gpsutils/CMakeLists.txt
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

nuttx_generate_kconfig(MENUDESC "GPS Utilities")

if(CONFIG_GPSUTILS_MINMEA_LIB)

  set(MINMEA_DIR ${CMAKE_CURRENT_LIST_DIR}/minmea)

  if(NOT EXISTS ${MINMEA_DIR})
    set(MINMEA_URL https://github.com/kosma/minmea/archive)
    set(MINMEA_VERSION db46128e73cee26d6a6eb0482dcba544ee1ea9f5)

    FetchContent_Declare(
      minmea_fetch
      URL ${MINMEA_URL}/${MINMEA_VERSION}.zip SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/minmea BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libc/gpsutils/minmea
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(minmea_fetch)

    if(NOT minmea_fetch_POPULATED)
      FetchContent_Populate(minmea_fetch)
    endif()
  endif()

  target_sources(c PRIVATE minmea/minmea.c)

endif()
