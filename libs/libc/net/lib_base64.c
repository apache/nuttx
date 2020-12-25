/****************************************************************************
 * libs/libc/net/lib_base64.c
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

#include <ctype.h>
#include <resolv.h>
#include <string.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_base64[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static const char g_pad64 = '=';

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int b64_ntop(FAR const unsigned char *src, size_t srclen,
             FAR char *target, size_t targsize)
{
  size_t datalen = 0;

  while (srclen >= 3)
    {
      if ((datalen += 4) >= targsize)
        {
          return -1;
        }

      *target++ = g_base64[src[0] >> 2];
      *target++ = g_base64[((src[0] & 0x03) << 4) + (src[1] >> 4)];
      *target++ = g_base64[((src[1] & 0x0f) << 2) + (src[2] >> 6)];
      *target++ = g_base64[src[2] & 0x3f];

      src += 3;
      srclen -= 3;
    }

  if (srclen != 0)
    {
      if ((datalen += 4) >= targsize)
        {
          return -1;
        }

     *target++ = g_base64[src[0] >> 2];
     if (srclen == 1)
       {
         *target++ = g_base64[(src[0] & 0x03) << 4];
         *target++ = g_pad64;
       }
     else
       {
         *target++ = g_base64[((src[0] & 0x03) << 4) + (src[1] >> 4)];
         *target++ = g_base64[(src[1] & 0x0f) << 2];
       }

    *target++ = g_pad64;
  }

  *target = '\0';
  return datalen;
}

int b64_pton(FAR const char *src,
             FAR unsigned char *target, size_t targsize)
{
  FAR char *pos;
  size_t datalen = 0;
  int state = 0;
  int ch;

  while ((ch = *src++) != '\0')
    {
      if (isspace(ch))
        {
          continue;
        }

      if (ch == g_pad64)
        {
          break;
        }

      pos = strchr(g_base64, ch);
      if (pos == NULL)
        {
          return -1;
        }

      switch (state)
        {
          case 0:
            if (target)
              {
                if (datalen >= targsize)
                  {
                    return -1;
                  }

                *target = (pos - g_base64) << 2;
              }

            state = 1;
            break;

          case 1:
            if (target)
              {
                if ((datalen += 1) >= targsize)
                  {
                    return -1;
                  }

                *target++ |=  (pos - g_base64) >> 4;
                *target    = ((pos - g_base64) & 0x0f) << 4;
              }

            state = 2;
            break;

          case 2:
            if (target)
              {
                if ((datalen += 1) >= targsize)
                  {
                    return -1;
                  }

                *target++ |=  (pos - g_base64) >> 2;
                *target    = ((pos - g_base64) & 0x03) << 6;
              }

            state = 3;
            break;

          case 3:
            if (target)
              {
                *target++ |= pos - g_base64;
              }

            datalen++;
            state = 0;
            break;

          default:
            return -1;
        }
      }

  if (ch == g_pad64)
    {
      ch = *src++;
      switch (state)
        {
          case 0:
          case 1:
            return -1;

          case 2:
            for (; ch != '\0'; ch = *src++)
              {
                if (!isspace(ch))
                  {
                    break;
                  }
              }

            if (ch != g_pad64)
              {
                return -1;
              }

            ch = *src++;

            /* FALLTHROUGH */

          case 3:
            for (; ch != '\0'; ch = *src++)
              {
                if (!isspace(ch))
                  {
                    return -1;
                  }
              }

           if (target && *target != 0)
             {
               return -1;
             }
        }
    }
  else
    {
       if (state != 0)
         {
           return -1;
         }
    }

  return datalen;
}
