/****************************************************************************
 * libs/libc/grp/lib_grp.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
