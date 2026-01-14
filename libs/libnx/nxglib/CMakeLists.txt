# ##############################################################################
# libs/libnx/nxglib/CMakeLists.txt
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

# Expose NXGL interfaces to applications

if(CONFIG_NXGLIB)

  # Files needed by both NX and NXFONTS
  set(SRCS nxglib_rgbblend.c)

  # Files needed only by NX

  if(CONFIG_NX)
    list(
      APPEND
      SRCS
      nxgl_area2rect.c
      nxglib_circlepts.c
      nxglib_circletraps.c
      nxglib_intersecting.c
      nxglib_nonintersecting.c
      nxglib_nullrect.c
      nxgl_rect2area.c
      nxglib_rectadd.c
      nxglib_rectcopy.c
      nxglib_rectinside.c
      nxglib_rectintersect.c
      nxglib_rectoffset.c
      nxglib_rectoverlap.c
      nxglib_rectsize.c
      nxglib_rectunion.c
      nxglib_rgb2yuv.c
      nxglib_runcopy.c
      nxglib_runoffset.c
      nxglib_splitline.c
      nxglib_trapcopy.c
      nxglib_trapoffset.c
      nxglib_vectoradd.c
      nxglib_vectsubtract.c
      nxglib_yuv2rgb.c)

    if(NOT CONFIG_NX_NPLANES EQUAL 1)
      list(APPEND SRCS nxglib_colorcmp.c nxglib_colorcopy.c)
    endif()
  endif()

  target_sources(nx PRIVATE ${SRCS})
endif()
