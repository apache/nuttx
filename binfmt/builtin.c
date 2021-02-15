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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/lib/builtin.h>

#ifdef CONFIG_FS_BINFS

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
  struct file file;
  int index;
  int ret;

  binfo("Loading file: %s\n", filename);

  /* Open the binary file for reading (only) */

  ret = file_open(&file, filename, O_RDONLY);
  if (ret < 0)
    {
      berr("ERROR: Failed to open binary %s: %d\n", filename, ret);
      return ret;
    }

  /* If this file is a BINFS file system, then we can recover the name of
   * the file using the FIOC_FILENAME ioctl() call.
   */

  ret = file_ioctl(&file, FIOC_FILENAME,
                   (unsigned long)((uintptr_t)&filename));
  if (ret < 0)
    {
      berr("ERROR: FIOC_FILENAME ioctl failed: %d\n", ret);
      file_close(&file);
      return ret;
    }

  /* Other file systems may also support FIOC_FILENAME, so the real proof
   * is that we can look up the index to this name in g_builtins[].
   */

  index = builtin_isavail(filename);
  if (index < 0)
    {
      berr("ERROR: %s is not a builtin application\n", filename);
      file_close(&file);
      return index;
    }

  /* Return the load information.  NOTE: that there is no way to configure
   * the priority.  That is a bug and needs to be fixed.
   */

  builtin         = builtin_for_index(index);
  binp->entrypt   = builtin->main;
  binp->stacksize = builtin->stacksize;
  binp->priority  = builtin->priority;
  file_close(&file);
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

#endif /* CONFIG_FS_BINFS */
