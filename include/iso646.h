/****************************************************************************
 * include/iso646.h
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

#ifndef __INCLUDE_ISO646_H
#define __INCLUDE_ISO646_H

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The <iso646.h> header shall define the following eleven macros
 * (on the left) that expand to the corresponding tokens (on the right):
 *
 * and          &&
 * and_eq       &=
 * bitand       &
 * bitor        |
 * compl        ~
 * not          !
 * not_eq       !=
 * or           ||
 * or_eq        |=
 * xor          ^
 * xor_eq       ^=
 *
 * Reference: Opengroup.org
 */

#define and         &&
#define and_eq      &=
#define bitand      &
#define bitor       |
#define compl       ~
#define not         !
#define not_eq      !=
#define or          ||
#define or_eq       |=
#define xor         ^
#define xor_eq      ^=

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ISO646_H */