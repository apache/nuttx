/****************************************************************************
 * binfmt/builtin.c
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

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/binfmt/binfmt.h>
#include <nuttx/lib/builtin.h>

#ifdef CONFIG_BUILTIN

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int builtin_loadbinary(FAR struct binary_s *binp,
                              FAR const char *filename,
                              FAR const struct symtab_s *exports,
                              int nexports);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_builtin_binfmt =
{
  NULL,               /* next */
  builtin_loadbinary, /* load */
  NULL,               /* unload */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_loadbinary
 *
 * Description:
 *   Verify that the file is an builtin binary.
 *
 ****************************************************************************/

static int builtin_loadbinary(FAR struct binary_s *binp,
                              FAR const char *filename,
                              FAR const struct symtab_s *exports,
                              int nexports)
{
  FAR const struct builtin_s *builtin;
  FAR char *name;
  int index;

  binfo("Loading file: %s\n", filename);

  name = strrchr(filename, '/');
  if (name != NULL)
    {
      filename = name + 1;
    }

  /* Looking up the index to this name in g_builtins[] */

  index = builtin_isavail(filename);
  if (index < 0)
    {
      berr("ERROR: %s is not a builtin application\n", filename);
      return index;
    }

  /* Return the load information.  NOTE: that there is no way to configure
   * the priority.  That is a bug and needs to be fixed.
   */

  builtin         = builtin_for_index(index);
  if (builtin == NULL)
    {
      berr("ERROR: %s is not a builtin application\n", filename);
      return -ENOENT;
    }

  binp->entrypt   = builtin->main;
  binp->stacksize = builtin->stacksize;
  binp->priority  = builtin->priority;
#ifdef CONFIG_SCHED_USER_IDENTITY
  binp->uid       = builtin->uid;
  binp->gid       = builtin->gid;
  binp->mode      = builtin->mode;
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: builtin_initialize
 *
 * Description:
 *   In order to use the builtin binary format, this function must be called
 *   during system initialize to register the builtin binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int builtin_initialize(void)
{
  int ret;

  /* Register ourselves as a binfmt loader */

  binfo("Registering Builtin Loader\n");

  ret = register_binfmt(&g_builtin_binfmt);
  if (ret != 0)
    {
      berr("Failed to register binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: builtin_uninitialize
 *
 * Description:
 *   Unregister the builtin binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void builtin_uninitialize(void)
{
  unregister_binfmt(&g_builtin_binfmt);
}

#endif /* CONFIG_BUILTIN */
