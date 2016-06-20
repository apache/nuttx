/****************************************************************************
 * drivers/syslog/vlowsyslog.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>

#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_ARCH_LOWPUTC
/* The low-level SYSLOG functions can be used only if we have access to
 * either the low-level serial interface, up_putc(), and to syslog_putc()
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _lowvsyslog
 *
 * Description:
 *   _lowvsyslog() handles the system logging system calls. It is functionally
 *   equivalent to vlowsyslog() except that the pre-process priority filtering
 *   has already been performed and, hence, there is no priority argument.
 *
 *   NOTE:  The va_list parameter is passed by reference.  That is because
 *   the va_list is a structure in some compilers and passing of structures
 *   in the NuttX sycalls does not work.
 *
 ****************************************************************************/

int _lowvsyslog(FAR const IPTR char *fmt, FAR va_list *ap)
{
  struct lib_outstream_s stream;

  /* Wrap the stdout in a stream object and let lib_vsprintf do the work. */

  syslogstream((FAR struct lib_outstream_s *)&stream);
  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, *ap);
}

#endif /* CONFIG_ARCH_LOWPUTC */
