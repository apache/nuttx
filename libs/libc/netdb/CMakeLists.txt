# ##############################################################################
# libs/libc/netdb/CMakeLists.txt
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

if(CONFIG_LIBC_NETDB)
  set(SRCS)

  # Add the netdb C files to the build

  list(
    APPEND
    SRCS
    lib_netdb.c
    lib_gethostbyname.c
    lib_gethostbynamer.c
    lib_gethostbyname2.c
    lib_gethostbyname2r.c
    lib_gethostentbynamer.c
    lib_gethostbyaddr.c
    lib_gethostbyaddrr.c
    lib_getservbyname.c
    lib_getservbynamer.c
    lib_getservbyport.c
    lib_getservbyportr.c
    lib_gaistrerror.c
    lib_freeaddrinfo.c
    lib_getaddrinfo.c
    lib_getnameinfo.c
    lib_rexec.c
    lib_dn.c
    lib_proto.c
    lib_protor.c)

  # Add host file support

  if(CONFIG_NETDB_HOSTFILE)
    list(APPEND SRCS lib_parsehostfile.c)
  endif()

  # Add DNS lookup support

  if(CONFIG_NETDB_DNSCLIENT)
    list(APPEND SRCS lib_dnsinit.c lib_dnsbind.c lib_dnsquery.c)
    list(APPEND SRCS lib_dnsaddserver.c lib_dnsdefaultserver.c)
    list(APPEND SRCS lib_dnsforeach.c lib_dnsnotify.c)
    list(APPEND SRCS lib_dnsqueryfamily.c)

    if(NOT CONFIG_NETDB_DNSCLIENT_ENTRIES EQUAL 0)
      list(APPEND SRCS lib_dnscache.c)
    endif()
  endif()

  target_sources(c PRIVATE ${SRCS})
endif()
