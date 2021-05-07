/****************************************************************************
 * binfmt/libnxflat/libnxflat_verify.c
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

#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/binfmt/nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_verifyheader
 *
 * Description:
 *   Given the header from a possible NXFLAT executable, verify that it
 *   is an NXFLAT executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_verifyheader(const struct nxflat_hdr_s *header)
{
  if (!header)
    {
      berr("NULL NXFLAT header!");
      return -ENOEXEC;
    }

  /* Check the FLT header -- magic number and revision.
   *
   * If the magic number does not match.  Just return
   * silently.  This is not our binary.
   */

  if (strncmp(header->h_magic, NXFLAT_MAGIC, 4) != 0)
    {
      berr("Unrecognized magic=\"%c%c%c%c\"\n",
      header->h_magic[0], header->h_magic[1],
      header->h_magic[2], header->h_magic[3]);
      return -ENOEXEC;
    }

  return OK;
}
