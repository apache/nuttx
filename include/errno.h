/****************************************************************************
 * include/errno.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_ERRNO_H
#define __INCLUDE_ERRNO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include "errno_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Convenience/compatibility definition.  If the errno is accessed from the
 * internal OS code, then the OS code should use the set_errno() and
 * get_errno().  Currently, those are just placeholders but would be needed
 * in the KERNEL mode build in order to instantiate the process address
 * environment as necessary to access the TLS-based errno variable.
 */

#define errno *__errno()
#define set_errno(e) \
  do \
    { \
      errno = (int)(e); \
    } \
  while (0)
#define get_errno() errno

/* Definitions of error numbers and the string that would be
 * returned by strerror().
 *
 * Note: most of them are defined in errno_defs.h, generated from errno.csv.
 */

#define EWOULDBLOCK         EAGAIN
#define EWOULDBLOCK_STR     "Operation would block"

#define __ELASTERROR        2000                        /* Users can add values starting here */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

/* Return a pointer to the thread specific errno. */

FAR int *__errno(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_ERRNO_H */
