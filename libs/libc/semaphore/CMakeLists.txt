# ##############################################################################
# libs/libc/semaphore/CMakeLists.txt
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

set(SRCS
    sem_init.c
    sem_setprotocol.c
    sem_getprotocol.c
    sem_getvalue.c
    sem_destroy.c
    sem_wait.c
    sem_trywait.c
    sem_timedwait.c
    sem_clockwait.c
    sem_post.c)

if(CONFIG_FS_NAMED_SEMAPHORES)
  list(APPEND SRCS sem_open.c sem_close.c sem_unlink.c)
endif()

target_sources(c PRIVATE ${SRCS})
