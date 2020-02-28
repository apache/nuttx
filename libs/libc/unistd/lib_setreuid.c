/****************************************************************************
 * libs/libc/unistd/lib_setreuid.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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

#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setreuid
 *
 * Description:
 *   The setreuid() function sets the real user ID and/or the effective user
 *   ID of the calling task group to ruid and/or euid.
 *
 * Input Parameters:
 *   ruid - Real user identity to set.  The special value (uid_t)-1
 *          indicates that the real user ID should not be changed.
 *   ruid - Effective user identity to set.  The special value (uid_t)-1
 *          indicates that the effective user ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setreuid(uid_t ruid, uid_t euid)
{
  int ret = OK;

  if (ruid != (uid_t)-1)
    {
      /* Set the real user ID.  CAREFUL:  This exploits non-standard behavior
       * of setuid():  setuid() should set the real, effective, and saved
       * user ID.  Here we depend on it setting only the real user ID.
       */

      ret = setuid(ruid);
    }

  if (ret >= 0 && euid != (uid_t)-1)
    {
      /* Set the effective user ID */

      ret = seteuid(euid);
    }

  return ret;
}
