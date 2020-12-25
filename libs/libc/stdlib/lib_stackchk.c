/****************************************************************************
 * libs/libc/stdlib/lib_stackchk.c
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

#include <assert.h>

#ifdef CONFIG_STACK_CANARIES

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR const void *const __stack_chk_guard = &__stack_chk_guard;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __stack_chk_fail
 *
 * Description:
 *   The interface __stack_chk_fail() shall abort the function that called
 *   it with a message that a stack overflow has been detected. The program
 *   that called the function shall then exit.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void __stack_chk_fail(void)
{
  PANIC();
}

#endif /* CONFIG_STACK_CANARIES */
