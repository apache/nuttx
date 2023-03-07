/****************************************************************************
 * include/iconv.h
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

#ifndef __INCLUDE_ICONV_H
#define __INCLUDE_ICONV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef FAR void *iconv_t;

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

iconv_t iconv_open(FAR const char *to, FAR const char *from);
size_t iconv(iconv_t cd, FAR char **in, FAR size_t *inb,
             FAR char **out, FAR size_t *outb);
int iconv_close(iconv_t cd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_ICONV_H */
