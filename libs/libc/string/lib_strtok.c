/****************************************************************************
 * libs/libc/string/lib_strtok.c
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

#include <string.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_saveptr = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtok
 *
 * Description:
 *    The  strtok()  function  parses  a string into a
 *    sequence of tokens.  On the first call to strtok() the
 *    string to be parsed should be specified in 'str'.  In
 *    each subsequent call that should parse the same string,
 *    'str' should be NULL.
 *
 *    The 'delim' argument specifies a set of characters that
 *    delimit the tokens in the parsed string.  The caller
 *    may specify different strings in delim in successive
 *    calls that parse the same string.
 *
 *    Each call to strtok() returns a pointer to a null-
 *    terminated string containing the next token. This
 *    string  does not include the delimiting character.  If
 *    no more tokens are found, strtok() returns NULL.
 *
 *    A sequence of two or more contiguous delimiter
 *    characters in the parsed string is considered to be a
 *    single delimiter. Delimiter characters at the start or
 *    end of the string are ignored.  The tokens returned by
 *    strtok() are always non-empty strings.
 *
 * Returned Value:
 *    strtok() returns a pointer to the next token, or NULL
 *    if there are no more tokens.
 *
 ****************************************************************************/

#undef strtok /* See mm/README.txt */
char *strtok(char *str, const char *delim)
{
  return strtok_r(str, delim, &g_saveptr);
}
