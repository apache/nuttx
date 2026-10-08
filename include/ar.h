/****************************************************************************
 * include/ar.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_AR_H
#define __INCLUDE_AR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The layout of an archive (.a) file, as written by ar(1): the magic string
 * ARMAG, then one struct ar_hdr per member, each followed by the member
 * data padded to an even size.  All header fields are ASCII, padded with
 * spaces.
 */

#define ARMAG   "!<arch>\n"   /* Magic string at the start of the file */
#define SARMAG  8             /* Length of ARMAG */
#define ARFMAG  "`\n"         /* Value of ar_fmag in every header */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ar_hdr
{
  char ar_name[16];  /* Member name, terminated by '/' (GNU) or a space */
  char ar_date[12];  /* Modification time, decimal seconds since the epoch */
  char ar_uid[6];    /* Owner user ID, decimal */
  char ar_gid[6];    /* Owner group ID, decimal */
  char ar_mode[8];   /* File mode, octal */
  char ar_size[10];  /* Size of the member data in bytes, decimal */
  char ar_fmag[2];   /* Always ARFMAG */
};

#endif /* __INCLUDE_AR_H */
