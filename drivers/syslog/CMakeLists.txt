# ##############################################################################
# drivers/syslog/CMakeLists.txt
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

# Include SYSLOG Infrastructure

set(SRCS)

if(CONFIG_SYSLOG)
  list(APPEND SRCS vsyslog.c syslog_channel.c syslog_write.c syslog_flush.c)
endif()

if(CONFIG_SYSLOG_INTBUFFER)
  list(APPEND SRCS syslog_intbuffer.c)
endif()

if(NOT CONFIG_ARCH_SYSLOG)
  list(APPEND SRCS syslog_initialize.c)
endif()

# The RAMLOG device is usable as a system logging device or standalone

if(CONFIG_RAMLOG)
  list(APPEND SRCS ramlog.c)
  if(CONFIG_RAMLOG_BUFFER_SECTION)
    target_compile_definitions(
      drivers PRIVATE -DRAMLOG_BUFFER_SECTION="${CONFIG_RAMLOG_BUFFER_SECTION}")
  endif()
endif()

# Include SYSLOG drivers (only one should be enabled)

# System logging to a character device (or file)

if(CONFIG_SYSLOG_CONSOLE
   OR CONFIG_SYSLOG_CHAR
   OR CONFIG_SYSLOG_FILE)
  list(APPEND SRCS syslog_device.c)
endif()

if(CONFIG_SYSLOG_CHAR)
  list(APPEND SRCS syslog_devchannel.c)
endif()

if(CONFIG_SYSLOG_CONSOLE)
  list(APPEND SRCS syslog_consolechannel.c)
endif()

if(CONFIG_SYSLOG_FILE)
  list(APPEND SRCS syslog_filechannel.c)
endif()

if(CONFIG_SYSLOG_CHARDEV)
  list(APPEND SRCS syslog_chardev.c)
endif()

if(CONFIG_SYSLOG_RPMSG)
  list(APPEND SRCS syslog_rpmsg.c)
endif()

if(CONFIG_SYSLOG_RPMSG_SERVER)
  list(APPEND SRCS syslog_rpmsg_server.c)
endif()

if(CONFIG_CONSOLE_SYSLOG)
  list(APPEND SRCS syslog_console.c)
endif()

if(CONFIG_SYSLOG_STREAM)
  list(APPEND SRCS syslog_stream.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
