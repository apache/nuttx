# ##############################################################################
# libs/libnx/nxfonts/CMakeLists.txt
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
function(add_font name id)
  string(REPLACE "-" "_" prefix "g_${name}_")
  configure_file(nxfonts_bitmaps.c nxfonts_bitmaps_${name}.c)
  target_sources(nxfonts
                 PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/nxfonts_bitmaps_${name}.c)
  set(CUSTOM_DEFINES NXFONTS_FONTID=${id} NXFONTS_PREFIX=${prefix})
  set_property(
    SOURCE ${CMAKE_CURRENT_BINARY_DIR}/nxfonts_bitmaps_${name}.c
    APPEND
    PROPERTY COMPILE_DEFINITIONS ${CUSTOM_DEFINES})
endfunction()

if(CONFIG_NXFONTS)
  nuttx_add_aux_library(nxfonts)
  target_include_directories(nxfonts PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_include_directories(nxfonts PRIVATE ${NUTTX_DIR}/libs/libnx)

  # Font conversion operations ###############################################

  set(SRCS nxfonts_getfont.c nxfonts_cache.c)

  set(BPPS
      1
      2
      4
      8
      16
      24
      32)
  foreach(bpp ${BPPS})
    set(BPP_SUFFIX _${bpp}bpp)
    configure_file(nxfonts_convert.c nxfonts_convert${BPP_SUFFIX}.c)
    set(CUSTOM_DEFINES NXFONTS_BITSPERPIXEL=${bpp} NXFONTS_SUFFIX=${BPP_SUFFIX})
    set_property(
      SOURCE ${CMAKE_CURRENT_BINARY_DIR}/nxfonts_convert${BPP_SUFFIX}.c
      APPEND
      PROPERTY COMPILE_DEFINITIONS ${CUSTOM_DEFINES})
    list(APPEND SRCS ${CMAKE_CURRENT_BINARY_DIR}/nxfonts_convert${BPP_SUFFIX}.c)
  endforeach()

  target_sources(nxfonts PRIVATE ${SRCS})

  # Fonts bitmaps ############################################################

  # Monospace fonts

  if(CONFIG_NXFONT_MONO5X8)
    add_font(mono5x8 18)
  endif()

  # Sans serif fonts

  if(CONFIG_NXFONT_SANS17X22)
    add_font(sans17x22 14)
  endif()

  if(CONFIG_NXFONT_SANS20X26)
    add_font(sans20x26 15)
  endif()

  if(CONFIG_NXFONT_SANS23X27)
    add_font(sans23x27 1)
  endif()

  if(CONFIG_NXFONT_SANS22X29)
    add_font(sans22x29 2)
  endif()

  if(CONFIG_NXFONT_SANS28X37)
    add_font(sans28x37 3)
  endif()

  if(CONFIG_NXFONT_SANS39X48)
    add_font(sans39x48 4)
  endif()

  # Sans serif bold fonts

  if(CONFIG_NXFONT_SANS17X23B)
    add_font(sans17x23b 16)
  endif()

  if(CONFIG_NXFONT_SANS20X27B)
    add_font(sans20x27b 17)
  endif()

  if(CONFIG_NXFONT_SANS22X29B)
    add_font(sans22x29b 5)
  endif()

  if(CONFIG_NXFONT_SANS28X37B)
    add_font(sans28x37b 6)
  endif()

  if(CONFIG_NXFONT_SANS40X49B)
    add_font(sans40x49b 7)
  endif()

  # Serif fonts

  if(CONFIG_NXFONT_SERIF22X29)
    add_font(serif22x29 8)
  endif()

  if(CONFIG_NXFONT_SERIF29X37)
    add_font(serif29x37 9)
  endif()

  if(CONFIG_NXFONT_SERIF38X48)
    add_font(serif38x48 10)
  endif()

  # Serif bold fonts

  if(CONFIG_NXFONT_SERIF22X28B)
    add_font(serif22x28b 11)
  endif()

  if(CONFIG_NXFONT_SERIF27X38B)
    add_font(serif27x38b 12)
  endif()

  if(CONFIG_NXFONT_SERIF38X49B)
    add_font(serif38x49b 13)
  endif()

  # Pixel fonts

  if(CONFIG_NXFONT_PIXEL_UNICODE)
    add_font(pixel-unicode 19)
  endif()

  if(CONFIG_NXFONT_PIXEL_LCD_MACHINE)
    add_font(pixel-lcd-machine 20)
  endif()

  # X11 misc fixed fonts

  if(CONFIG_NXFONT_X11_MISC_FIXED_4X6)
    add_font(x11-misc-fixed-4x6 21)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_5X7)
    add_font(x11-misc-fixed-5x7 22)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_5X8)
    add_font(x11-misc-fixed-5x8 23)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_6X9)
    add_font(x11-misc-fixed-6x9 24)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_6X10)
    add_font(x11-misc-fixed-6x10 25)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_6X12)
    add_font(x11-misc-fixed-6x12 26)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_6X13)
    add_font(x11-misc-fixed-6x13 27)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_6X13B)
    add_font(x11-misc-fixed-6x13b 28)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_6X13O)
    add_font(x11-misc-fixed-6x13o 29)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_7X13)
    add_font(x11-misc-fixed-7x13 30)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_7X13B)
    add_font(x11-misc-fixed-7x13b 31)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_7X13O)
    add_font(x11-misc-fixed-7x13o 32)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_7X14)
    add_font(x11-misc-fixed-7x14 33)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_7X14B)
    add_font(x11-misc-fixed-7x14b 34)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_8X13)
    add_font(x11-misc-fixed-8x13 35)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_8X13B)
    add_font(x11-misc-fixed-8x13b 36)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_8X13O)
    add_font(x11-misc-fixed-8x13o 37)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_9X15)
    add_font(x11-misc-fixed-9x15 38)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_9X15B)
    add_font(x11-misc-fixed-9x15b 39)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_9X18)
    add_font(x11-misc-fixed-9x18 40)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_9X18B)
    add_font(x11-misc-fixed-9x18b 41)
  endif()

  if(CONFIG_NXFONT_X11_MISC_FIXED_10X20)
    add_font(x11-misc-fixed-10x20 42)
  endif()

  # Tom Thumb mono-space 4x6 font

  if(CONFIG_NXFONT_TOM_THUMB_4X6)
    add_font(tom-thumb-4x6 43)
  endif()

  target_link_libraries(nx PRIVATE nxfonts)
endif()
