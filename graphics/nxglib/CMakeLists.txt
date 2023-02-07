# ##############################################################################
# graphics/nxglib/CMakeLists.txt
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

nuttx_add_aux_library(nxglib)
set(SRCS)

set(OPERATIONS setpixel fillrectangle getrectangle filltrapezoid moverectangle
               copyrectangle)
set(BPPS
    1
    2
    8
    4
    16
    24
    32)

if(CONFIG_NX_LCDDRIVER)
  set(BLITDIR lcd)
else()
  set(BLITDIR fb)
endif()

foreach(op ${OPERATIONS})
  foreach(bpp ${BPPS})
    configure_file(${BLITDIR}/nxglib_${op}.c.in nxglib_${op}_${bpp}bpp.c)
    set_source_files_properties(
      ${CMAKE_CURRENT_BINARY_DIR}/nxglib_${op}_${bpp}bpp.c
      PROPERTIES COMPILE_FLAGS -DNXGLIB_BITSPERPIXEL=${bpp})
    list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/nxglib_${op}_${bpp}bpp.c)
  endforeach()
endforeach()

if(CONFIG_NX_RAMBACKED)
  foreach(op ${OPERATIONS})
    foreach(bpp ${BPPS})
      configure_file(pwfb/pwfb_${op}.c.in pwfb_${op}_${bpp}bpp.c)
      set_source_files_properties(
        ${CMAKE_CURRENT_BINARY_DIR}/pwfb_${op}_${bpp}bpp.c
        PROPERTIES COMPILE_FLAGS -DNXGLIB_BITSPERPIXEL=${bpp})
      list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/pwfb_${op}_${bpp}bpp.c)
    endforeach()
  endforeach()
endif()

if(CONFIG_NX_SWCURSOR)
  set(CURSOR_OPS draw erase backup)
  set(CURSOR_BPPS 8 16 24 32)

  foreach(op ${CURSOR_OPS})
    foreach(bpp ${CURSOR_BPPS})
      configure_file(cursor/nxglib_cursor_${op}.c.in
                     nxglib_cursor_${op}_${bpp}bpp.c)
      set_source_files_properties(
        ${CMAKE_CURRENT_BINARY_DIR}/nxglib_cursor_${op}_${bpp}bpp.c
        PROPERTIES COMPILE_FLAGS -DNXGLIB_BITSPERPIXEL=${bpp})
      list(APPEND SRCS
           ${CMAKE_CURRENT_BINARY_DIR}/nxglib_cursor_${op}_${bpp}bpp.c)
    endforeach()
  endforeach()
endif()

target_sources(nxglib PRIVATE ${SRCS})
target_include_directories(nxglib PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(graphics PRIVATE nxglib)
