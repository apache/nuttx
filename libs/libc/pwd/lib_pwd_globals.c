/****************************************************************************
 * libs/libc/pwd/lib_pwd_globals.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pwd/lib_pwd.h"

#ifdef CONFIG_LIBC_PASSWD_FILE

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Data for non-reentrant group functions */

struct passwd g_passwd;
char g_passwd_buffer[CONFIG_LIBC_PASSWD_LINESIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_LIBC_GROUP_FILE */
