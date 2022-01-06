/****************************************************************************
 * binfmt/binfmt_loadmodule.c
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

#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/envpath.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: load_default_priority
 *
 * Description:
 *   Set the default priority of the module to be loaded.  This may be
 *   changed (1) by the actions of the binary format's load() method if
 *   the binary format contains priority information, or (2) by the user
 *   between calls to load_module() and exec_module().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; Otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int load_default_priority(FAR struct binary_s *bin)
{
  struct sched_param param;
  int ret;

  /* Get the priority of this thread */

  ret = nxsched_get_param(0, &param);
  if (ret < 0)
    {
      berr("ERROR: nxsched_get_param failed: %d\n", ret);
      return ret;
    }

  /* Save that as the priority of child thread */

  bin->priority = param.sched_priority;
  if (bin->priority <= 0)
    {
      bin->priority = SCHED_PRIORITY_DEFAULT;
    }

  return ret;
}

/****************************************************************************
 * Name: load_absmodule
 *
 * Description:
 *   Load a module into memory, bind it to an exported symbol take, and
 *   prep the module for execution.  filename is known to be an absolute
 *   path to the file to be loaded.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int load_absmodule(FAR struct binary_s *bin, FAR const char *filename,
                          FAR const struct symtab_s *exports, int nexports)
{
  FAR struct binfmt_s *binfmt;
  int ret = -ENOENT;

  binfo("Loading %s\n", filename);

  /* Disabling pre-emption should be sufficient protection while accessing
   * the list of registered binary format handlers.
   */

  sched_lock();

  /* Traverse the list of registered binary format handlers.  Stop
   * when either (1) a handler recognized and loads the format, or
   * (2) no handler recognizes the format.
   */

  for (binfmt = g_binfmts; binfmt; binfmt = binfmt->next)
    {
      /* Use this handler to try to load the format */

      ret = binfmt->load(bin, filename, exports, nexports);
      if (ret == OK)
        {
          /* Successfully loaded -- break out with ret == 0 */

          binfo("Successfully loaded module %s\n", filename);

          /* Save the unload method for use by unload_module */

          bin->unload = binfmt->unload;
          binfmt_dumpmodule(bin);
          break;
        }
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: load_module
 *
 * Description:
 *   Load a module into memory, bind it to an exported symbol take, and
 *   prep the module for execution.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int load_module(FAR struct binary_s *bin, FAR const char *filename,
                FAR const struct symtab_s *exports, int nexports)
{
  int ret = -EINVAL;

  /* Verify that we were provided something to work with */

#ifdef CONFIG_DEBUG_FEATURES
  if (bin && filename)
#endif
    {
      /* Set the default priority of the new program. */

      ret = load_default_priority(bin);
      if (ret < 0)
        {
          return ret;
        }

      /* Were we given a relative path?  Or an absolute path to the file to
       * be loaded?  Absolute paths start with '/'.
       */

#ifdef CONFIG_LIBC_ENVPATH
      if (filename[0] != '/')
        {
          FAR char *fullpath;
          ENVPATH_HANDLE handle;

          ret = -ENOENT;

          /* Initialize to traverse the PATH variable */

          handle = envpath_init("PATH");
          if (handle)
            {
              /* Get the next absolute file path */

              while ((fullpath = envpath_next(handle, filename)) != NULL)
                {
                  /* Try to load the file at this path */

                  ret = load_absmodule(bin, fullpath, exports, nexports);

                  /* Free the allocated fullpath */

                  kmm_free(fullpath);

                  /* Break out of the loop with ret == OK on success */

                  if (ret == OK)
                    {
                      break;
                    }
                }

              /* Release the traversal handle */

              envpath_release(handle);
            }
        }
      else
#endif
        {
          /* We already have the one and only absolute path to the file to
           * be loaded.
           */

          ret = load_absmodule(bin, filename, exports, nexports);
        }
    }

  return ret;
}

#endif /* CONFIG_BINFMT_DISABLE */
