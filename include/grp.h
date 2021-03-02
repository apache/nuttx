/****************************************************************************
 * include/grp.h
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

#ifndef __INCLUDE_GRP_H
#define __INCLUDE_GRP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct group
{
  FAR char  *gr_name;
  FAR char  *gr_passwd;
  gid_t      gr_gid;
  FAR char **gr_mem;
};

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

FAR struct group *getgrnam(FAR const char *name);
FAR struct group *getgrgid(gid_t gid);
int getgrnam_r(FAR const char *name,
               FAR struct group *grp,
               FAR char *buf,
               size_t buflen,
               FAR struct group **result);
int getgrgid_r(gid_t gid, FAR struct group *grp,
               FAR char *buf, size_t buflen,
               FAR struct group **result);
int initgroups(FAR const char *user, gid_t group);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_GRP_H */
