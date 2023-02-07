# ##############################################################################
# cmake/nuttx_redefine_symbols.cmake
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

set(NXSYMBOLS
    __cxa_atexit
    abort
    accept
    access
    atexit
    backtrace
    bind
    calloc
    chmod
    chown
    clock_gettime
    close
    closedir
    connect
    dlsym
    dup
    exit
    fchmod
    fchown
    fclose
    fcntl
    fdopen
    fopen
    fprintf
    fread
    free
    fseek
    fstat
    fsync
    ftell
    ftruncate
    futimens
    fwrite
    getpeername
    getsockname
    getenv
    getpid
    getsockopt
    if_nametoindex
    ioctl
    listen
    longjmp
    lseek
    malloc
    malloc_size
    malloc_usable_size
    memcpy
    mkdir
    mmap
    mprotect
    munmap
    open
    opendir
    perror
    poll
    posix_memalign
    pthread_attr_init
    pthread_attr_setstack
    pthread_attr_destroy
    pthread_cond_destroy
    pthread_cond_init
    pthread_cond_signal
    pthread_cond_wait
    pthread_create
    pthread_getspecific
    pthread_key_create
    pthread_kill
    pthread_mutex_destroy
    pthread_mutex_init
    pthread_mutex_lock
    pthread_mutex_unlock
    pthread_setspecific
    pthread_sigmask
    puts
    read
    readdir
    readv
    realloc
    recvfrom
    rename
    rewinddir
    rmdir
    sched_yield
    select
    sendmsg
    sendto
    setitimer
    setbuf
    setjmp
    setsockopt
    shutdown
    sigaction
    sigaddset
    sigemptyset
    sigfillset
    sleep
    socket
    stat
    statvfs
    stderr
    strcat
    strchr
    strerror
    strlen
    strtol
    sysconf
    syslog
    tcgetattr
    tcsetattr
    unlink
    usleep
    utimensat
    write
    writev)

set(NXSYMBOL_RENAMES)
foreach(NXSYMBOL ${NXSYMBOLS})
  if(APPLE OR (CYGWIN AND CONFIG_SIM_CYGWIN_DECORATED))
    list(APPEND NXSYMBOL_RENAMES "_${NXSYMBOL} NX${NXSYMBOL}")
  else()
    list(APPEND NXSYMBOL_RENAMES "${NXSYMBOL} NX${NXSYMBOL}")
  endif()
endforeach()
string(REPLACE ";" "\n" NXSYMBOL_RENAMES "${NXSYMBOL_RENAMES}")
file(WRITE ${CMAKE_BINARY_DIR}/nuttx-names.dat "${NXSYMBOL_RENAMES}\n")
