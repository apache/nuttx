/****************************************************************************
 * sched/module/mod_modhandle.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#ifdef CONFIG_MODULE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modhandle
 *
 * Description:
 *   modhandle() returns the module handle for the installed module with the
 *   provided name.  A secondary use of this function is to determin if a
 *   module has been loaded or not.
 *
 * Input Parameters:
 *   name   - A pointer to the module name string.
 *
 * Returned Value:
 *   The non-NULL module handle previously returned by insmod() is returned
 *   on success.  If no module with that name is installed, modhandle() will
 *   return a NULL handle and the errno variable will be set appropriately.
 *
 ****************************************************************************/

FAR void *modhandle(FAR const char *name)
{
  FAR struct module_s *modp;

  DEBUGASSERT(name != NULL);

  /* Get exclusive access to the module registry */

  modlib_registry_lock();

  /* Find the module entry for this name in the registry */

  modp = modlib_registry_find(name);
  if (modp == NULL)
    {
      berr("ERROR: Failed to find module %s: %d\n", name, modp);
      set_errno(ENOENT);
    }

  modlib_registry_unlock();
  return (FAR void *)modp;
}

#endif /* CONFIG_MODULE */
