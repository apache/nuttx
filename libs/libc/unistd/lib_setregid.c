/****************************************************************************
 * libs/libc/unistd/lib_setregid.c
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
 * Name: setregid
 *
 * Description:
 *   The setregid() function sets the real group ID and/or the effective group
 *   ID of the calling task group to rgid and/or egid.
 *
 * Input Parameters:
 *   rgid - Real group identity to set.  The special value (gid_t)-1
 *          indicates that the real group ID should not be changed.
 *   rgid - Effective group identity to set.  The special value (gid_t)-1
 *          indicates that the effective group ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setregid(gid_t rgid, gid_t egid)
{
  int ret = OK;

  if (rgid != (gid_t)-1)
    {
      /* Set the real group ID.  CAREFUL:  This exploits non-standard behavior
       * of setgid():  setgid() should set the real, effective, and saved
       * group ID.  Here we depend on it setting only the real group ID.
       */

      ret = setgid(rgid);
    }

  if (ret >= 0 && egid != (gid_t)-1)
    {
      /* Set the effective group ID */

      ret = setegid(egid);
    }

  return ret;
}
