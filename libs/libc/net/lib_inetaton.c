/****************************************************************************
 * libs/libc/net/lib_inetaton.c
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

#include <stdint.h>
#include <arpa/inet.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_aton
 *
 * Description:
 *   The inet_aton() function converts the string pointed to by cp, in the
 *   standard IPv4 dotted decimal notation, to an integer value suitable for
 *   use as an Internet address.
 *
 * Note:
 *   inet_aton() returns nonzero if the address is valid, zero if not.
 *   Therefore macros OK and ERROR are not used here.
 *
 ****************************************************************************/

int inet_aton(FAR const char *cp, FAR struct in_addr *inp)
{
  int dots = 0;
  uint32_t num = 0;
  uint32_t addr = 0;
  int c;

  do
    {
      c = *cp;

      switch (c)
        {
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            num = 10 * num + (c - '0');
            break;

          case '.':
            dots++;
            if (dots > 3)
              {
                return 0;
              }

            /* no break */

          case '\0':
            if (num > 255)
              {
                return 0;
              }

            addr = addr << 8 | num;
            num = 0;
            break;

          default:
            return 0;
        }
    }
  while (*cp++);

  /* Normalize it */

  if (dots < 3)
    {
      addr <<= 8 * (3 - dots);
    }

  if (inp)
    {
      inp->s_addr = HTONL(addr);
    }

  return 1;
}
