# ##############################################################################
# graphics/nxterm/CMakeLists.txt
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

if(CONFIG_NXTERM)
  set(SRCS
      nx_register.c
      nxterm_driver.c
      nxterm_font.c
      nxterm_putc.c
      nxterm_redraw.c
      nxterm_resize.c
      nxterm_register.c
      nxterm_scroll.c
      nxterm_vt100.c
      nxtk_register.c
      nxtool_register.c
      nxterm_clear.c)

  if(NOT CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
    list(APPEND SRCS nxterm_unregister.c)
  endif()

  if(CONFIG_NXTERM_NXKBDIN)
    list(APPEND SRCS nxterm_kbdin.c)
  endif()

  target_sources(graphics PRIVATE ${SRCS})
endif()
