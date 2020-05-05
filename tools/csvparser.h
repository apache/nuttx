/****************************************************************************
 * tools/csvparser.h
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

#ifndef __TOOLS_CSVPARSER_H
#define __TOOLS_CSVPARSER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINESIZE      (PATH_MAX > 256 ? PATH_MAX : 256)

#define MAX_FIELDS    16
#define MAX_PARMSIZE  256

#define NAME_INDEX    0
#define HEADER_INDEX  1
#define COND_INDEX    2
#define RETTYPE_INDEX 3
#define PARM1_INDEX   4

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef char csvparm_t[MAX_PARMSIZE];

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern bool      g_debug;
extern char      g_line[LINESIZE + 1];
extern csvparm_t g_parm[MAX_FIELDS];
extern int       g_lineno;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

char *read_line(FILE *stream);
int parse_csvline(char *ptr);

#endif /* __TOOLS_CSVPARSER_H */
