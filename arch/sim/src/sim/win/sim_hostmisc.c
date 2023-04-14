/****************************************************************************
 * arch/sim/src/sim/win/sim_hostmisc.c
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

#include <direct.h>
#include <stdio.h>
#include <string.h>

#include <io.h>
#include <windows.h>
#include <libloaderapi.h>

#include "sim_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_abort
 *
 * Description:
 *   Abort the simulation
 *
 * Input Parameters:
 *   status - Exit status to set
 ****************************************************************************/

void host_abort(int status)
{
  ExitProcess(status);
}

int host_backtrace(void** array, int size)
{
  return CaptureStackBackTrace(0, size, array, NULL);
}

/****************************************************************************
 * Name: host_init_cwd
 ****************************************************************************/

#ifdef CONFIG_SIM_IMAGEPATH_AS_CWD
void host_init_cwd(void)
{
  char *name;
  char path[MAX_PATH];

  /* Get the absolute path of the executable file */

  if (GetModuleFileNameA(GetModuleHandleA(NULL), path, MAX_PATH) == 0)
    {
      perror("GetModuleFileNameA failed");
      return;
    }

  name = strrchr(path, '/');
  if (name == NULL)
    {
      name = strrchr(path, '\\');
    }

  *++name = '\0';
  _chdir(path);
}
#endif
