/****************************************************************************
 * drivers/serial/serial_termios.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/serial/serial.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uart_termios_getattr(FAR const struct uart_termios_s *tc,
                          FAR struct termios *termiosp)
{
  termiosp->c_iflag = tc->iflag;
  termiosp->c_oflag = tc->oflag;
  termiosp->c_lflag = tc->lflag;
}

void uart_termios_setattr(FAR struct uart_termios_s *tc,
                          FAR const struct termios *termiosp)
{
  tc->iflag = termiosp->c_iflag;
  tc->oflag = termiosp->c_oflag;
  tc->lflag = termiosp->c_lflag;
}

size_t uart_termios_in_copy(FAR const struct uart_termios_s *tc,
                            FAR char *dst, FAR const char *src,
                            size_t len)
{
  FAR char *odst = dst;

  while (len > 0)
    {
      FAR const char *special = NULL;
      size_t ncopy = len;
      char ch;

      /* Bound the second memchr by the first hit to stay linear */

      if (tc->iflag & (IGNCR | ICRNL))
        {
          special = memchr(src, '\r', ncopy);
          ncopy   = special != NULL ? (special - src) : ncopy;
        }

      if (tc->iflag & INLCR)
        {
          FAR const char *nl = memchr(src, '\n', ncopy);

          if (nl != NULL)
            {
              special = nl;
              ncopy   = special - src;
            }
        }

      /* memmove: in-place callers make dst trail src once a CR is dropped */

      if (ncopy != 0)
        {
          memmove(dst, src, ncopy);
          dst += ncopy;
          src += ncopy;
          len -= ncopy;
        }

      if (special == NULL)
        {
          break;
        }

      ch = *src++;
      len--;
      if (uart_termios_in_char(tc, &ch))
        {
          *dst++ = ch;
        }
    }

  return dst - odst;
}

size_t uart_termios_out_copy(FAR const struct uart_termios_s *tc,
                             FAR char *dst, size_t dstlen,
                             FAR const char *src, size_t srclen,
                             FAR size_t *consumed)
{
  FAR const char *osrc = src;
  FAR char *odst = dst;

  if ((tc->oflag & OPOST) == 0)
    {
      size_t len = MIN(srclen, dstlen);

      memcpy(dst, src, len);
      *consumed = len;
      return len;
    }

  while (srclen > 0 && dstlen > 0)
    {
      FAR const char *special = NULL;
      size_t len = srclen;

      if (tc->oflag & OCRNL)
        {
          special = memchr(src, '\r', len);
          len     = special != NULL ? (size_t)(special - src) : len;
        }

      if (tc->oflag & ONLCR)
        {
          FAR const char *nl = memchr(src, '\n', len);

          if (nl != NULL)
            {
              special = nl;
              len     = special - src;
            }
        }

      len = MIN(len, dstlen);
      if (len != 0)
        {
          memcpy(dst, src, len);
          dst    += len;
          dstlen -= len;
          src    += len;
          srclen -= len;
        }

      if (special == NULL || dstlen == 0)
        {
          break;
        }

      len = uart_termios_out_char(tc, *src, dst, dstlen);
      if (len == 0)
        {
          break;
        }

      dst    += len;
      dstlen -= len;
      src++;
      srclen--;
    }

  *consumed = src - osrc;
  return dst - odst;
}

FAR const char *
uart_termios_isig_find(FAR const struct uart_termios_s *tc,
                       FAR const char *src, size_t len, FAR int *signo)
{
  FAR const char *special = NULL;

  if ((tc->lflag & ISIG) == 0)
    {
      return NULL;
    }

#ifdef CONFIG_TTY_SIGINT
  special = memchr(src, CONFIG_TTY_SIGINT_CHAR, len);
  if (special != NULL)
    {
      *signo = SIGINT;
    }
#endif

#ifdef CONFIG_TTY_SIGTSTP
    {
      FAR const char *tmp;

      tmp = memchr(src, CONFIG_TTY_SIGTSTP_CHAR,
                   special != NULL ? (size_t)(special - src) : len);
      if (tmp != NULL)
        {
          special = tmp;
          *signo  = SIGTSTP;
        }
    }
#endif

  return special;
}
