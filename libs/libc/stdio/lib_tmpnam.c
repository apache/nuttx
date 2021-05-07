/****************************************************************************
 * libs/libc/stdio/lib_tmpnam.c
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmpnam
 *
 * Description:
 *   The tmpnam() function generates a string that is a valid filename and
 *   that is not the same as the name of an existing file. The function is
 *   potentially capable of generating TMP_MAX different strings, but any or
 *   all of them may already be in use by existing files and thus not be
 *   suitable return values.
 *
 *   The tmpnam() function generates a different string each time it is
 *   called from the same process, up to {TMP_MAX} times. If it is called
 *   more than {TMP_MAX} times, the behavior is implementation-defined.
 *
 * Returned Value:
 *   Upon successful completion, tmpnam() returns a pointer to a string. I
 *   no suitable string can be generated, the tmpnam() function will
 *   return a null pointer.
 *
 *   If the argument s is a null pointer, tmpnam() will leave its result
 *   in an internal static object and return a pointer to that object.
 *   Subsequent calls to tmpnam() may modify the same object. If the
 *   argument s is not a null pointer, it is presumed to point to an
 *   array of at least L_tmpnam chars; tmpnam() will write its result in
 *   that array and will return the argument as its value.
 *
 ****************************************************************************/

FAR char *tmpnam(FAR char *s)
{
  static char path[L_tmpnam];

  if (s == NULL)
    {
      s = path;
    }

  snprintf(s, L_tmpnam, "%s/XXXXXX", P_tmpdir);
  return mktemp(s);
}
