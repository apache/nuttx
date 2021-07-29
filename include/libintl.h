/****************************************************************************
 * include/libintl.h
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

#ifndef __INCLUDE_LIBINTL_H
#define __INCLUDE_LIBINTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

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

FAR char *gettext(FAR const char *msgid) formatlike(1);
FAR char *dgettext(FAR const char *domainname,
                   FAR const char *msgid) formatlike(2);
FAR char *dcgettext(FAR const char *domainname,
                    FAR const char *msgid,
                    int category) formatlike(2);
FAR char *ngettext(FAR const char *msgid1,
                   FAR const char *msgid2,
                   unsigned long int n) formatlike(1) formatlike(2);
FAR char *dngettext(FAR const char *domainname,
                    FAR const char *msgid1,
                    FAR const char *msgid2,
                    unsigned long int n) formatlike(2) formatlike(3);
FAR char *dcngettext(FAR const char *domainname,
                     FAR const char *msgid1,
                     FAR const char *msgid2,
                     unsigned long int n,
                     int category) formatlike(2) formatlike(3);

FAR char *textdomain(FAR const char *domainname);

FAR char *bindtextdomain(FAR const char *domainname,
                         FAR const char *dirname);
FAR char *bind_textdomain_codeset(FAR const char *domainname,
                                  FAR const char *codeset);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_LIBINTL_H */
