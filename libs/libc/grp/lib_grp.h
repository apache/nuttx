/****************************************************************************
 * libs/libc/grp/lib_grp.h
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

#ifndef __LIBS_LIBC_GRP_LIB_GRP_H
#define __LIBS_LIBC_GRP_LIB_GRP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <grp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROOT_GID 0
#define ROOT_NAME "root"
#define ROOT_PASSWD "x"

/* Reserver space for a NULL terminated list for group member names */

#define MEMBER_SIZE ((CONFIG_LIBC_GROUP_NUSERS + 1) * sizeof(FAR char *))

/* Reserve space for the maximum line in the group file PLUS space for an
 * array of Member names.
 */

#define GRPBUF_RESERVE_SIZE (CONFIG_LIBC_GROUP_LINESIZE + MEMBER_SIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_LIBC_GROUP_FILE
/* Data for non-reentrant group functions */

EXTERN struct group g_group;
EXTERN char g_group_buffer[GRPBUF_RESERVE_SIZE];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct group *getgrbuf(gid_t gid, FAR const char *name,
                           FAR const char *passwd);
int getgrbuf_r(gid_t gid, FAR const char *name, FAR const char *passwd,
               FAR struct group *grp, FAR char *buf, size_t buflen,
               FAR struct group **result);

#ifdef CONFIG_LIBC_GROUP_FILE
int grp_findby_name(FAR const char *gname, FAR struct group *entry,
                    FAR char *buffer, size_t buflen);
int grp_findby_gid(gid_t gid, FAR struct group *entry, FAR char *buffer,
                   size_t buflen);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __LIBS_LIBC_GRP_LIB_GRP_H */
