/****************************************************************************
 * libs/libc/termios/lib_tcsendbreak.c
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

#include <sys/ioctl.h>

#include <termios.h>
#include <errno.h>

#include <nuttx/serial/tioctl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcsendbreak
 *
 * Description:
 *   Transmit a serial line Break, which is a continuous stream of 0 bits.
 *
 *   A proper Break needs to last longer than a NUL ('\0') character.
 *   Effectively this causes a Framing Error at the receiver's UART. Most
 *   UARTs have the ability to distinguish between a Break and other types
 *   of Framing Errors and may treat it specially. Some devices utilize
 *   Breaks for things like synchronization.
 *
 *   Regarding the duration parameter and its portability between Unix-like
 *   systems: It seems that a duration parameter of 0 specifies a Break of
 *   0.25 to 0.5 seconds with consistency between various Unix flavors, but
 *   the interpretation of a nonzero duration parameter varies wildly from
 *   one operating system to another:
 *
 *     - Linux, AIX, Tru64: duration in milliseconds
 *     - BSDs, macOS, HP-UX: ignore duration
 *     - Solaris, UnixWare: behaves like tcdrain()
 *
 *   On FreeBSD and OpenBSD, a duration of 0 gives a Break of 0.4 seconds.
 *
 *   Sources of information about the duration parameter include: the man
 *   page for termios(3) as found in release 5.13 of the Linux man-pages
 *   project, the man page for tcsendbreak(3) as found in FreeBSD 13.1 and
 *   OpenBSD 7.2, and some StackOverflow search results.
 *
 *   In NuttX, we combine the above treatments of duration: Zero or negative
 *   values will give a Break of 0.4 seconds; positive values will be
 *   treated as a duration in milliseconds. We hope this combination will
 *   work well for programs that are ported from any of the above families.
 *
 *   NuttX-specific implementation details: tcsendbreak() calls IOCTL
 *   TCSBRK. This is handled in the upper half serial driver (see
 *   drivers/serial/serial.c), which treats its argument like 'duration'
 *   described here, except unsigned, supporting only 0 or positive values;
 *   0 maps to 400 and all other values are used as-is, in milliseconds.
 *   This, in turn, calls IOCTLs TIOCSBRK and TIOCCBRK to start and end a
 *   BSD-compatible Break, respectively, with a sleep of the given duration
 *   between them. For tcsendbreak() to work, the arch-specific lower half
 *   serial driver must implement TIOCSBRK and TIOCCBRK. Some architectures
 *   do not implement these at all. Others implement them but rely upon one
 *   or more Kconfig options being set, such as CONFIG_*_U[S]ART_BREAKS and,
 *   on some architectures, a separate CONFIG_*_SERIALBRK_BSDCOMPAT, to
 *   provide the required Break behavior. Furthermore, the sleep time will
 *   be at least the given duration, but may be substantially longer
 *   depending on the system's tick resolution (see CONFIG_USEC_PER_TICK)
 *   and other factors.
 *
 *   This function blocks for the duration of the Break.
 *
 * Input Parameters:
 *   fd       - The 'fd' argument is an open file descriptor associated
 *              with a terminal.
 *   duration - If 0 or negative, request a serial line Break lasting 0.4
 *              seconds. If non-zero, request a serial line Break lasting
 *              that duration in milliseconds.
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned. Otherwise, -1 is returned
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

int tcsendbreak(int fd, int duration)
{
  if (duration <= 0)
    {
      duration = 400;
    }

  return ioctl(fd, TCSBRK, (unsigned long)duration);
}
