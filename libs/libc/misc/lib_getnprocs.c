/****************************************************************************
 * libs/libc/misc/lib_getnprocs.c
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
#include <sys/sysinfo.h>
#include <sys/types.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_nprocs_conf
 *
 * Description:
 *   Retrieve the number of configured processors in the system.
 *   The get_nprocs_conf() function returns the number of processors (CPUs)
 *   configured in the system, regardless of whether they are currently
 *   enabled or available. This function is useful for determining the
 *   processor configuration in multiprocessor systems.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the number of configured processors is returned.
 *   If the system does not define a processor configuration, it returns 1.
 *
 ****************************************************************************/

int get_nprocs_conf(void)
{
  return CONFIG_SMP_NCPUS;
}

/****************************************************************************
 * Name: get_nprocs
 *
 * Description:
 *   Retrieve the number of online processors in the system.
 *   The get_nprocs() function returns the number of processors (CPUs) that
 *   are currently online and available for use in the system. This function
 *   can be useful for determining the number of processors that can be used
 *   by applications or the operating system for parallel processing.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the number of online processors is returned.
 *   If the system does not define a processor configuration, it returns 1.
 *
 ****************************************************************************/

int get_nprocs(void)
{
  return CONFIG_SMP_NCPUS;
}
