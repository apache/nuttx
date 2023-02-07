# ##############################################################################
# libs/libc/grp/CMakeLists.txt
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
set(SRCS lib_getgrgid.c lib_getgrgidr.c lib_getgrnam.c lib_getgrnamr.c
         lib_initgroups.c)

if(CONFIG_LIBC_GROUP_FILE)
  list(APPEND SRCS lib_find_grpfile.c lib_grp_globals.c)
else()
  list(APPEND SRCS lib_getgrbuf.c lib_getgrbufr.c)
endif()

target_sources(c PRIVATE ${SRCS})
