/****************************************************************************
 * include/err.h
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

#ifndef __INCLUDE_ERR_H
#define __INCLUDE_ERR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdarg.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_LIBC_ERR

/* Print "pid: ", FORMAT, ": ", the standard error string for errno,
 * and a newline, on stderr.
 */

void warn(FAR const char *fmt, ...);
void vwarn(FAR const char *fmt, va_list ap);

/* Likewise, but without ": " and the standard error string.  */

void warnx(FAR const char *fmt, ...);
void vwarnx(FAR const char *fmt, va_list ap);

/* Likewise, and then exit with STATUS.  */

void err(int status, FAR const char *fmt, ...);
void verr(int status, FAR const char *fmt, va_list ap);
void errx(int status, FAR const char *fmt, ...);
void verrx(int status, FAR const char *, va_list ap);

#endif /* CONFIG_LIBC_ERR */
#endif /* __INCLUDE_ERR_H */
