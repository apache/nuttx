/****************************************************************************
 * arch/xtensa/include/simcall.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_INCLUDE_SIMCALL_H
#define __ARCH_XTENSA_INCLUDE_SIMCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIMCALL_SYS_READ    3
#define SIMCALL_SYS_WRITE   4
#define SIMCALL_SYS_OPEN    5
#define SIMCALL_SYS_CLOSE   6
#define SIMCALL_SYS_LSEEK   19

/* fcntl O_xxx constants for simcall.
 *
 * Someone with the official spec should fix this and qemu.
 * I made this CONFIG_HOST_xxx dependant because:
 *  - the qemu implementation just pass them to the host OS
 *  - I don't have an official documentation.
 *  - I build NuttX and run qemu on the same host.
 * I know this is wrong. But it works for me.
 */

#if CONFIG_HOST_MACOS
#define SIMCALL_O_RDONLY   0x0000
#define SIMCALL_O_WRONLY   0x0001
#define SIMCALL_O_RDWR     0x0003
#define SIMCALL_O_ACCMODE  0x0003
#define SIMCALL_O_APPEND   0x0008
#define SIMCALL_O_CREAT    0x0200
#define SIMCALL_O_TRUNC    0x0400
#define SIMCALL_O_EXCL     0x0800
#else

/* Assume Linux */

#define SIMCALL_O_RDONLY   0x0000
#define SIMCALL_O_WRONLY   0x0001
#define SIMCALL_O_RDWR     0x0002
#define SIMCALL_O_ACCMODE  0x0003
#define SIMCALL_O_APPEND   0x0400
#define SIMCALL_O_CREAT    0x0040
#define SIMCALL_O_TRUNC    0x0200
#define SIMCALL_O_EXCL     0x0080
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int simcall(int nr, int param1, int param2, int param3, int *errp);

#endif /* __ARCH_XTENSA_INCLUDE_SIMCALL_H */
