/****************************************************************************
 * libs/libc/unistd/lib_getcwd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_current_dir_name
 *
 * Description:
 *   get_current_dir_name() will allocate an buffer to hold the current
 *   working directory info, and this buffer will be returned to user.
 *   the user will be responsible to free the buffer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, get_current_dir_name() returns a pointer to a string
 *   containing the pathname of the current working directory.
 *   Otherwise, get_current_dir_name() returns a null pointer and
 *   sets errno to indicate the error.
 *   the errno can refer to: getcwd() function's comments.
 *
 ****************************************************************************/

FAR char *get_current_dir_name(void)
{
  return getcwd(NULL, 0);
}
