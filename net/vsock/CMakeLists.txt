# ##############################################################################
# net/vsock/CMakeLists.txt
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

# Virtual Socket source files

if(CONFIG_NET_VSOCK)
  set(SRCS vsock_sockif.c vsock_addr.c)

  if(CONFIG_NET_VSOCK_VIRTIO)
    list(APPEND SRCS vsock_virtio.c)
  endif()

  if(CONFIG_NET_VSOCK_VHOST)
    list(APPEND SRCS vsock_vhost.c)
  endif()

  if(CONFIG_NET_VSOCK_LOCAL)
    list(APPEND SRCS vsock_local.c)
  endif()

  target_sources(net PRIVATE ${SRCS})
endif()
