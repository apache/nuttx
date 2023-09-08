# ##############################################################################
# net/can/CMakeLists.txt
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
if(CONFIG_NET_CAN)
  set(SRCS)

  # Socket layer
  list(APPEND SRCS can_sockif.c can_sendmsg.c can_recvmsg.c)

  if(CONFIG_NET_CAN_NOTIFIER)
    list(APPEND SRCS can_notifier.c)
  endif()

  if(CONFIG_NET_CANPROTO_OPTIONS)
    list(APPEND SRCS can_setsockopt.c can_getsockopt.c)
  endif()

  list(APPEND SRCS can_conn.c can_input.c can_callback.c can_poll.c)

  target_sources(net PRIVATE ${SRCS})
endif()
