/****************************************************************************
 * include/resolv.h
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

#ifndef __INCLUDE_RESOLV_H
#define __INCLUDE_RESOLV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int b64_ntop(FAR const unsigned char *src, size_t srclen,
             FAR char *target, size_t targsize);

int b64_pton(FAR const char *src,
             FAR unsigned char *target, size_t targsize);

/* dn resolution */

int dn_comp(FAR const char *src, FAR unsigned char *dst, int space,
            FAR unsigned char **dnptrs, FAR unsigned char **lastdnptr);

int dn_expand(FAR const unsigned char *base, FAR const unsigned char *end,
              FAR const unsigned char *src, FAR char *dest, int space);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_RESOLV_H */
