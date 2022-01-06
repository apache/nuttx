/****************************************************************************
 * include/nl_types.h
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

#ifndef __INCLUDE_NL_TYPES_H
#define __INCLUDE_NL_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NL_SETD       1
#define NL_CAT_LOCALE 1

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef int       nl_item;
typedef FAR void *nl_catd;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

nl_catd catopen(FAR const char *name, int oflag);
FAR char *catgets(nl_catd catd, int set_id, int msg_id, FAR const char *s);
int catclose(nl_catd catd);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NL_TYPES_H */
