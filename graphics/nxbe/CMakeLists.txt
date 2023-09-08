# ##############################################################################
# graphics/nxbe/CMakeLists.txt
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

set(SRCS
    nxbe_bitmap.c
    nxbe_configure.c
    nxbe_colormap.c
    nxbe_clipper.c
    nxbe_closewindow.c
    nxbe_redraw.c
    nxbe_redrawbelow.c
    nxbe_setposition.c
    nxbe_move.c
    nxbe_getrectangle.c
    nxbe_fill.c
    nxbe_filltrapezoid.c
    nxbe_setpixel.c
    nxbe_lower.c
    nxbe_raise.c
    nxbe_modal.c
    nxbe_isvisible.c
    nxbe_setsize.c
    nxbe_setvisibility.c)

if(CONFIG_NX_RAMBACKED)
  list(APPEND SRCS nxbe_flush.c)
endif()

if(CONFIG_NX_SWCURSOR)
  list(APPEND SRCS nxbe_cursor.c nxbe_cursor_backupdraw.c)
elseif(CONFIG_NX_HWCURSOR)
  list(APPEND SRCS nxbe_cursor.c)
endif()

if(CONFIG_NX_UPDATE)
  list(APPEND SRCS nxbe_notify_rectangle.c)
endif()

target_sources(graphics PRIVATE ${SRCS})
