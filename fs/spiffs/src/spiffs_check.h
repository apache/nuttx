/****************************************************************************
 * fs/spiffs/src/spiffs_check.h
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

#ifndef __FS_SPIFFS_SRC_SPIFFS_CHECK_H
#define __FS_SPIFFS_SRC_SPIFFS_CHECK_H

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Consistency check output */

#if !defined(CONFIG_SPIFFS_CHECK_OUTPUT)
#  define spiffs_checkinfo   _none
#elif defined(CONFIG_CPP_HAVE_VARARGS)
#  define spiffs_checkinfo(format, ...) \
     syslog(LOG_NOTICE, "SPIFFS: " format, ##__VA_ARGS__)
#else
#  define spiffs_checkinfo   _info
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spiffs_s; /* Forward reference */

/****************************************************************************
 * Name: spiffs_check_luconsistency
 *
 * Description:
 *   Scans all object look up. For each entry, corresponding page header is
 *   checked for validity.  If an object index header page is found, this is
 *   also checked
 *
 * Input Parameters:
 *   fs - A reference to the SPIFFS volume object instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int spiffs_check_luconsistency(FAR struct spiffs_s *fs);

/****************************************************************************
 * Name: spiffs_check_pgconsistency
 *
 * Description:
 *   Checks consistency amongst all pages and fixes irregularities
 *   Scans all pages (except lu pages), reserves 4 bits in working memory
 *   for each page
 *
 *     bit 0: 0 == FREE|DELETED, 1 == USED
 *     bit 1: 0 == UNREFERENCED, 1 == REFERENCED
 *     bit 2: 0 == NOT_INDEX,    1 == INDEX
 *     bit 3: unused
 *
 *   A consistent file system will have only pages being
 *
 *     - x000 free, unreferenced, not index
 *      - x011 used, referenced only once, not index
 *      - x101 used, unreferenced, index
 *
 *   The working memory might not fit all pages so several scans might be
 *   needed
 *
 * Input Parameters:
 *   fs - A reference to the SPIFFS volume object instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int spiffs_check_pgconsistency(FAR struct spiffs_s *fs);

/****************************************************************************
 * Name: spiffs_check_pgconsistency
 *
 * Description:
 *   Removes orphaned and partially deleted index pages.
 *   Scans for index pages. When an index page is found, corresponding index
 *   header is searched for.   If no such page exists, the index page cannot
 *   be reached as no index header exists and must be deleted.
 *
 * Input Parameters:
 *   fs - A reference to the SPIFFS volume object instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int spiffs_check_objidconsistency(FAR struct spiffs_s *fs);

/****************************************************************************
 * Name: spiffs_dump
 *
 * Description:
 *   Dump logical flash content
 *
 * Input Parameters:
 *   fs - A reference to the volume structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPIFFS_DUMP
int spiffs_dump(FAR struct spiffs_s *fs);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __FS_SPIFFS_SRC_SPIFFS_CHECK_H */
