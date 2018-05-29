/****************************************************************************
 * libs/libc/stdio/lib_tmpnam.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  int ret;

  if (s == NULL)
    {
      s = path;
    }

  (void)snprintf(s, L_tmpnam, "%s/XXXXXX.tmp", P_tmpdir);
  ret = mktemp(s);
  return (ret == OK) ? s : NULL;
}
