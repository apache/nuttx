/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_main.c
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
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* This function is provided outside exported SDK, therefore here is defined
 * as weak symbol to avoid link error.
 */

int nsh_main(int argc, char *argv[]);

int weak_function spresense_main(int argc, char *argv[])
{
  return nsh_main(argc, argv);
}

#ifdef CONFIG_SYSTEMTICK_HOOK
void weak_function board_timerhook(void)
{
}
#endif
