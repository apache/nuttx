# ##############################################################################
# libs/libnx/nxmu/CMakeLists.txt
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

# All of user-space hooks must be built in for the multi-user NX implementation
# so that they are available to the application in the kernel build (single use
# mode cannot be used with the kernel build)

if(CONFIG_NX)
  set(SRCS
      nxmu_sendserver.c
      nx_connect.c
      nx_disconnect.c
      nx_eventhandler.c
      nx_eventnotify.c
      nx_block.c
      nx_synch.c
      nx_kbdchin.c
      nx_kbdin.c
      nx_mousein.c
      nx_releasebkgd.c
      nx_requestbkgd.c
      nx_setbgcolor.c
      nxmu_sendwindow.c
      nx_closewindow.c
      nx_constructwindow.c
      nx_bitmap.c
      nx_fill.c
      nx_filltrapezoid.c
      nx_getposition.c
      nx_getrectangle.c
      nx_lower.c
      nx_modal.c
      nx_move.c
      nx_openwindow.c
      nx_raise.c
      nx_redrawreq.c
      nx_setpixel.c
      nx_setposition.c
      nx_setsize.c
      nx_setvisibility.c)

  if(CONFIG_NX_HWCURSOR)
    list(APPEND SRCS nx_cursor.c)
  elseif(CONFIG_NX_SWCURSOR)
    list(APPEND SRCS nx_cursor.c)
  endif()

  target_sources(nx PRIVATE ${SRCS})
endif()
