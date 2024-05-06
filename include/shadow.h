/****************************************************************************
 * include/shadow.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_SHADOW_H
#define __INCLUDE_SHADOW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* Structure of the password file */

struct spwd
{
  FAR char *sp_namp;          /* Login name */
  FAR char *sp_pwdp;          /* Encrypted password */
  long int sp_lstchg;         /* Date of last change */
  long int sp_min;            /* Minimum number of days between changes */
  long int sp_max;            /* Maximum number of days between changes */
  long int sp_warn;           /* Number of days to warn user to change the password */
  long int sp_inact;          /* Number of days the account may be inactive */
  long int sp_expire;         /* Number of days since 1970-01-01 until account expires */
  unsigned long int sp_flag;  /* Reserved */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Get shadow entry matching NAME */

FAR struct spwd *getspnam(FAR const char *name);
int getspnam_r(FAR const char *name, FAR struct spwd *sp, FAR char *buf,
               size_t size, FAR struct spwd **res);

#endif /* __INCLUDE_SHADOW_H */