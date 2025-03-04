# ##############################################################################
# libs/libnx/nxtk/CMakeLists.txt
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

if(CONFIG_NX)
  set(SRCS
      nxtk_setsubwindows.c
      nxtk_events.c
      nxtk_block.c
      nxtk_synch.c
      nxtk_subwindowclip.c
      nxtk_containerclip.c
      nxtk_subwindowmove.c
      nxtk_drawframe.c
      nxtk_raise.c
      nxtk_lower.c
      nxtk_modal.c
      nxtk_setvisibility.c
      nxtk_ishidden.c
      nxtk_setposition.c
      nxtk_getposition.c
      nxtk_setsize.c
      nxtk_openwindow.c
      nxtk_closewindow.c
      nxtk_fillwindow.c
      nxtk_getwindow.c
      nxtk_filltrapwindow.c
      nxtk_movewindow.c
      nxtk_bitmapwindow.c
      nxtk_drawcirclewindow.c
      nxtk_drawlinewindow.c
      nxtk_fillcirclewindow.c
      nxtk_opentoolbar.c
      nxtk_closetoolbar.c
      nxtk_filltoolbar.c
      nxtk_gettoolbar.c
      nxtk_filltraptoolbar.c
      nxtk_movetoolbar.c
      nxtk_bitmaptoolbar.c
      nxtk_drawcircletoolbar.c
      nxtk_drawlinetoolbar.c
      nxtk_fillcircletoolbar.c
      nxtk_toolbarbounds.c)

  target_sources(nx PRIVATE ${SRCS})
endif()
