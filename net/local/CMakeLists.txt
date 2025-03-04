# ##############################################################################
# net/local/CMakeLists.txt
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

# Unix domain socket source files

if(CONFIG_NET_LOCAL)
  set(SRCS
      local_conn.c
      local_release.c
      local_bind.c
      local_fifo.c
      local_recvmsg.c
      local_sendpacket.c
      local_recvutils.c
      local_sockif.c
      local_netpoll.c
      local_sendmsg.c)

  if(CONFIG_NET_LOCAL_STREAM)
    list(APPEND SRCS local_connect.c local_listen.c local_accept.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
