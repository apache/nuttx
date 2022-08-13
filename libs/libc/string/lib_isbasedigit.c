/****************************************************************************
 * libs/libc/string/lib_isbasedigit.c
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

#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_isbasedigit
 *
 * Description:
 *   Given an ASCII character, ch, and a base (1-36) do two
 *   things:  1) Determine if ch is a valid character, and 2)
 *   convert ch to its binary value.
 *
 ****************************************************************************/

bool lib_isbasedigit(int ch, int base, int *value)
{
  bool ret = false;
  int  tmp = 0;

  if (base <= 10)
    {
      if (ch >= '0' && ch <= base + '0' - 1)
        {
          tmp = ch - '0';
          ret = true;
        }
    }
  else if (base <= 36)
    {
      if (ch >= '0' && ch <= '9')
        {
          tmp = ch - '0';
          ret = true;
        }
      else if (ch >= 'a' && ch <= 'a' + base - 11)
        {
          tmp = ch - 'a' + 10;
          ret = true;
        }
      else if (ch >= 'A' && ch <= 'A' + base - 11)
        {
          tmp = ch - 'A' + 10;
          ret = true;
        }
    }

  if (value)
    {
      *value = tmp;
    }

  return ret;
}
