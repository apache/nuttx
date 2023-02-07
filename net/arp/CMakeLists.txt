# ##############################################################################
# net/arp/CMakeLists.txt
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

if(CONFIG_NET_ARP)
  set(SRCS arp_input.c arp_out.c arp_format.c arp_table.c)

  if(CONFIG_NET_ARP_IPIN)
    list(APPEND SRCS arp_ipin.c)
  endif()

  if(CONFIG_NET_ARP_SEND)
    list(APPEND SRCS arp_send.c arp_poll.c arp_notify.c)
  endif()

  if(CONFIG_NET_ARP_DUMP)
    list(APPEND SRCS arp_dump.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
