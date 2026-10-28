/****************************************************************************
 * libs/libc/unistd/lib_setpgrp.c
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

#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setpgrp
 *
 * Description:
 *   Make the calling process the leader of a process group whose ID is
 *   the process ID of the caller, as setpgid(0, 0) does (XSI).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The process group ID of the caller is returned on success.  On failure
 *   -1 is returned and errno is set as setpgid() sets it.
 *
 ****************************************************************************/

pid_t setpgrp(void)
{
  if (setpgid(0, 0) < 0)
    {
      return -1;
    }

  return getpgrp();
}
