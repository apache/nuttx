############################################################################
# libs/libc/netdb/Make.defs
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

if (CONFIG_LIBC_NETDB)
  set(SRCS)

  # Add the netdb C files to the build

  list(APPEND SRCS
    lib_netdb.c lib_gethostbyname.c lib_gethostbynamer.c
    lib_gethostbyname2.c lib_gethostbyname2r.c
    lib_gethostentbynamer.c
    lib_gethostbyaddr.c lib_gethostbyaddrr.c
    lib_getservbyname.c lib_getservbynamer.c
    lib_getservbyport.c lib_getservbyportr.c
    lib_gaistrerror.c lib_freeaddrinfo.c lib_getaddrinfo.c
    lib_getnameinfo.c)

  # Add host file support

  if (CONFIG_NETDB_HOSTFILE)
    list(APPEND SRCS lib_parsehostfile.c)
  endif()

  # Add DNS lookup support

  if (CONFIG_NETDB_DNSCLIENT)
    list(APPEND SRCS lib_dnsinit.c lib_dnsbind.c lib_dnsquery.c)
    list(APPEND SRCS lib_dnsaddserver.c lib_dnsdefaultserver.c)
    list(APPEND SRCS lib_dnsforeach.c lib_dnsnotify.c)

    if (NOT CONFIG_NETDB_DNSCLIENT_ENTRIES EQUAL 0)
      list(APPEND SRCS lib_dnscache.c)
    endif()
  endif()

  target_sources(c PRIVATE ${SRCS})
endif()
