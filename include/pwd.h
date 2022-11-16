/****************************************************************************
 * include/pwd.h
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

#ifndef __INCLUDE_PWD_H
#define __INCLUDE_PWD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_PASSWD_LINESIZE
# define NSS_BUFLEN_PASSWD CONFIG_LIBC_PASSWD_LINESIZE
#else
# define NSS_BUFLEN_PASSWD 256
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct passwd
{
  FAR char *pw_name;
  uid_t     pw_uid;
  gid_t     pw_gid;
  FAR char *pw_gecos;
  FAR char *pw_dir;
  FAR char *pw_shell;
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

FAR struct passwd *getpwnam(FAR const char *name);
FAR struct passwd *getpwuid(uid_t uid);
int getpwnam_r(FAR const char *name, FAR struct passwd *pwd, FAR char *buf,
               size_t buflen, FAR struct passwd **result);
int getpwuid_r(uid_t uid, FAR struct passwd *pwd, FAR char *buf,
               size_t buflen, FAR struct passwd **result);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_PWD_H */
