/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_perms.c
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

#include <nuttx/arch.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_text_enable_write
 *
 * Description:
 *   Temporarily enable write access to the .text section. This must be
 *   called prior to loading the process code into memory.
 *
 * Input Parameters:
 *   addrenv - The address environment to be modified.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_text_enable_write(group_addrenv_t *addrenv)
{
  /* Nothing needs to be done */

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_text_disable_write
 *
 * Description:
 *   Disable write access to the .text section. This must be called after the
 *   process code is loaded into memory.
 *
 * Input Parameters:
 *   addrenv - The address environment to be modified.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_text_disable_write(group_addrenv_t *addrenv)
{
  /* Nothing needs to be done */

  return OK;
}
