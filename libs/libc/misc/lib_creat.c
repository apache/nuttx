/****************************************************************************
 * libs/libc/misc/lib_creat.c
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

#include <fcntl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: creat
 *
 * Description:
 *   The creat() function is just a simple wrapper of open() function
 *
 * Input Parameters:
 *   path  - The path for the file to create on
 *   mode  - the access mode
 *
 * Returned Value:
 *   Return a file descriptor of the file that created success,
 *   return -1 if error occurd on creat new file, and errno is setup
 *
 ****************************************************************************/

int creat(FAR const char *path, mode_t mode)
{
  return open(path, O_WRONLY | O_CREAT | O_TRUNC, mode);
}
