/****************************************************************************
 * fs/spiffs/src/spiffs_check.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
