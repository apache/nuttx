/****************************************************************************
 * include/pty.h
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

#ifndef __INCLUDE_PTY_H
#define __INCLUDE_PTY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <termios.h>
#include <sys/ioctl.h>

#if defined(CONFIG_SERIAL_TERMIOS) && defined(CONFIG_PSEUDOTERM)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Create pseudo tty pair with name and set terminal attributes according
 * to term and win and return handles for both ends in master and slave.
 ****************************************************************************/

int openpty(FAR int *master, FAR int *slave, FAR char *name,
            FAR const struct termios *term, FAR const struct winsize *win);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SERIAL_TERMIOS && CONFIG_PSEUDOTERM */
#endif /* __INCLUDE_PTY_H */
