# ##############################################################################
# libs/libc/audio/libsrc/CMakeLists.txt
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
if(CONFIG_AUDIO_SRC)
  set(LIBSRC_VERSION 0.2.2)

  FetchContent_Declare(
    libsrc URL https://codeload.github.com/libsndfile/libsamplerate/zip/master)
  FetchContent_Populate(libsrc)
  FetchContent_GetProperties(libsrc)

  set(LIBSRC_DEFINITIONS -DPACKAGE="libsamplerate"
                         -DVERSION="${LIBSRC_VERSION}")

  if(CONFIG_SINC_FAST_CONVERTER)
    list(APPEND LIBSRC_DEFINITIONS -DENABLE_SINC_FAST_CONVERTER)
  endif()

  if(CONFIG_SINC_MEDIUM_CONVERTER)
    list(APPEND LIBSRC_DEFINITIONS -DENABLE_SINC_MEDIUM_CONVERTER)
  endif()

  if(CONFIG_SINC_BEST_CONVERTER)
    list(APPEND LIBSRC_DEFINITIONS -DENABLE_SINC_BEST_CONVERTER)
  endif()

  set(SRCS samplerate.c src_sinc.c src_linear.c src_zoh.c)
  list(TRANSFORM SRCS PREPEND ${libsrc_SOURCE_DIR}/src/)
  target_sources(c PRIVATE ${SRCS})

  target_compile_definitions(c PRIVATE ${LIBSRC_DEFINITIONS})
  target_include_directories(c PRIVATE ${libsrc_SOURCE_DIR}/include)
endif()
