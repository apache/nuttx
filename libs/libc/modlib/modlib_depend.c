/****************************************************************************
 * libs/libc/modlib/modlib_depend.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_depend
 *
 * Description:
 *   Set up module dependencies between the exporter and the importer of a
 *   symbol.  The happens when the module is installed via insmod and a
 *   symbol is imported from another module.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 * Assumptions:
 *   Caller holds the registry lock.
 *
 ****************************************************************************/

int modlib_depend(FAR struct module_s *importer,
                  FAR struct module_s *exporter)
{
#if CONFIG_MODLIB_MAXDEPEND > 0
  int freendx;
  int i;

  DEBUGASSERT(importer != NULL && exporter != NULL);

  /* First checker if the exported is already in our list if dependencies.
   * This would happen if we are importing multiple symbols from the same
   * exporting module.  In that case, the module would already be in the
   * list of dependencies.
   *
   * The list dependency list is a a dumb, upacked array of pointers.  This
   * should not be too inefficient if the number of CONFIG_MODLIB_MAXDEPEND
   * is small.  Otherwise, a more dynamic data structure would be in order.
   */

  for (i = 0, freendx = -1; i < CONFIG_MODLIB_MAXDEPEND; i++)
    {
      FAR const struct module_s *modp;

      /* Check if this dependency slot is available. */

      modp = importer->dependencies[i];
      if (modp == NULL)
        {
          /* Remember this slot for use the module is NOT already in the
           * list of dependencies.
           */

          freendx = i;
        }
      else if (modp == exporter)
        {
          /* Yes, we are already importing symbols from this module.  Nothing
           * more needs to be done.
           */

          return OK;
        }
    }

  /* If we get here, then (1) this is a new exporting module that does not
   * already appear in the list of dependencies, and (2) freendx is the
   * index to the last free slot in the dependency list.  If freendx is
   * negative, then the dependency list is full.
   */

  if (freendx >= 0)
    {
      /* Increment the count of dependencies on the exporter module */

      DEBUGASSERT(exporter->dependents < UINT8_MAX);
      if (exporter->dependents >= UINT8_MAX)
        {
          return -ENOSPC;
        }

      exporter->dependents++;

      /* And remember the exporter so that we can decrement the count of
       * dependents if the importer is removed.
       */

      DEBUGASSERT(importer->dependencies[freendx] == NULL);
      importer->dependencies[freendx] = exporter;
      return OK;
    }

  /* If we get there then the list of dependencies is full. */

  DEBUGPANIC();
  return -ENFILE;

#else
  return OK;
#endif
}

/****************************************************************************
 * Name: modlib_undepend
 *
 * Description:
 *   Tear down module dependencies between the exporters and the importer of
 *   symbols.  This happens when the module is removed via rmmod (and on
 *   error handling cases in insmod).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 * Assumptions:
 *   Caller holds the registry lock.
 *
 ****************************************************************************/

int modlib_undepend(FAR struct module_s *importer)
{
#if CONFIG_MODLIB_MAXDEPEND > 0
  FAR struct module_s *exporter;
  int i;

  DEBUGASSERT(importer != NULL && importer->dependents == 0);

  /* Decrement the dependency count on each of exporters of symbols used by
   * this importer module.  This is an upacked array of pointers.  This
   * should not be too inefficient if the number of CONFIG_MODLIB_MAXDEPEND
   * is small.  Otherwise, a more dynamic data structure would be in order.
   */

  for (i = 0; i < CONFIG_MODLIB_MAXDEPEND; i++)
    {
      exporter = importer->dependencies[i];
      if (exporter != NULL)
        {
          DEBUGASSERT(exporter->dependents > 0);
          if (exporter->dependents > 0)
            {
              exporter->dependents--;
            }

          importer->dependencies[i] = NULL;
        }
    }
#endif

  return OK;
}
