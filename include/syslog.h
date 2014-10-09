/****************************************************************************
 * include/syslog.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYSLOG_H
#define __INCLUDE_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The option argument to openlog() is an OR of any of these:
 *
 *   LOG_CONS     - Write directly to system console if there is an error
 *                  while sending to system logger. 
 *   LOG_NDELAY   - Open the connection immediately (normally, the connection
 *                  is opened when the first message is logged). 
 *   LOG_NOWAIT   - Don't wait for child processes that may have been created
 *                  while logging the message.
 *   LOG_ODELAY   - The converse of LOG_NDELAY; opening of the connection is
 *                  delayed until syslog() is called. (This is the default,
 *                  and need not be specified.) 
 *   LOG_PERROR   - (Not in POSIX.1-2001 or POSIX.1-2008.) Print to stderr
 *                  as well (Linux).
 *   LOG_PID      - Include PID with each message. 
 */

/* Note: openlog() is not currently supported */

/* The facility argument is used to specify what type of program is logging
 * the message. This lets the configuration file specify that messages from
 * different facilities will be handled differently.
 *
 *   LOG_AUTH     - Security/authorization messages 
 *   LOG_AUTHPRIV - Security/authorization messages (private) 
 *   LOG_CRON     - Clock daemon (cron and at) 
 *   LOG_DAEMON   - System daemons without separate facility value 
 *   LOG_FTP      - Ftp daemon 
 *   LOG_KERN     - Lernel messages (these can't be generated from user
 *                  processes) 
 *   LOG_LOCAL0 through LOG_LOCAL7 - Reserved for local use 
 *   LOG_LPR      - Line printer subsystem 
 *   LOG_MAIL     - Mail subsystem 
 *   LOG_NEWS     - USENET news subsystem 
 *   LOG_SYSLOG   - Messages generated internally by syslogd(8) 
 *   LOG_USER     - Generic user-level messages (default) 
 *   LOG_UUCP     - UUCP subsystem 
 */

#define LOG_AUTH      0
#define LOG_AUTHPRIV  0
#define LOG_CRON      0
#define LOG_DAEMON    0
#define LOG_FTP       0
#define LOG_KERN      0
#define LOG_LOCAL0    0
#define LOG_LOCAL1    0
#define LOG_LOCAL2    0
#define LOG_LOCAL3    0
#define LOG_LOCAL4    0
#define LOG_LOCAL5    0
#define LOG_LOCAL6    0
#define LOG_LOCAL7    0
#define LOG_LPR       0
#define LOG_MAIL      0
#define LOG_NEWS      0
#define LOG_SYSLOG    0
#define LOG_USER      0
#define LOG_UUCP      0

/* This determines the importance of the message. The levels are, in order
 * of decreasing importance:
 */

#define LOG_EMERG     0  /* System is unusable */
#define LOG_ALERT     1  /* Action must be taken immediately */
#define LOG_CRIT      2  /* Critical conditions */
#define LOG_ERR       3  /* Error conditions */
#define LOG_WARNING   4  /* Warning conditions */
#define LOG_NOTICE    5  /* Normal, but significant, condition */
#define LOG_INFO      6  /* Informational message */
#define LOG_DEBUG     7  /* Debug-level message */

/* Used with setlogmask() */

#define LOG_MASK(p)   (1 << (p))
#define LOG_UPTO(p)   ((1 << (p)) - 1)
#define LOG_ALL       0xff

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

#if 0 /* Not supported */
void openlog(FAR const char *ident, int option, int facility);
void closelog(void);
#endif

/****************************************************************************
 * Name: syslog and vsyslog
 *
 * Description:
 *   syslog() generates a log message. The priority argument is formed by
 *   ORing the facility and the level values (see include/syslog.h). The
 *   remaining arguments are a format, as in printf and any arguments to the
 *   format.
 *
 *   The NuttX implementation does not support any special formatting
 *   characters beyond those supported by printf.
 *
 *   The function vsyslog() performs the same task as syslog() with the
 *   difference that it takes a set of arguments which have been obtained
 *   using the stdarg variable argument list macros.
 *
 ****************************************************************************/

int syslog(int priority, FAR const char *format, ...);
int vsyslog(int priority, FAR const char *src, va_list ap);

/****************************************************************************
 * Name: lowsyslog and lowvsyslog
 *
 * Description:
 *   syslog() generates a log message. The priority argument is formed by
 *   ORing the facility and the level values (see include/syslog.h). The
 *   remaining arguments are a format, as in printf and any arguments to the
 *   format.
 *
 *   This is a non-standard, low-level system logging interface.  The
 *   difference between syslog() and lowsyslog() is that the syslog()
 *   interface writes to the syslog device (usually fd=1, stdout) whereas
 *   lowsyslog() uses a lower level interface that works even from interrupt
 *   handlers.
 *
 *   If the platform cannot support lowsyslog, then we will substitute the
 *   standard syslogging functions.  These will, however, probably cause
 *   problems if called from interrupt handlers, depending upon the nature of
 *   the underlying syslog device.
 *
 *   The function lowvsyslog() performs the same task as lowsyslog() with
 *   the difference that it takes a set of arguments which have been
 *   obtained using the stdarg variable argument list macros.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LOWPUTC

int lowsyslog(int priority, FAR const char *format, ...);
int lowvsyslog(int priority, FAR const char *format, va_list ap);

#else

#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define lowsyslog(p,f,...) syslog(p,f,##__VA_ARGS__)
#  else
#    define lowsyslog (void)
#  endif
#  define lowvsyslog(p,f,a) vsyslog(p,f,a)

#endif

/****************************************************************************
 * Name: setlogmask
 *
 * Description:
 *   The setlogmask() function sets the logmask and returns the previous
 *   mask. If the mask argument is 0, the current logmask is not modified.
 *
 *   The SYSLOG priorities are: LOG_EMERG, LOG_ALERT, LOG_CRIT, LOG_ERR,
 *   LOG_WARNING, LOG_NOTICE, LOG_INFO, and LOG_DEBUG.  The bit corresponding
 *   to a priority p is LOG_MASK(p); LOG_UPTO(p) provides the mask of all
 *   priorities in the above list up to and including p.
 *
 *   REVISIT: Per POSIX the syslog mask should be a per-process value but in
 *   NuttX, the scope of the mask is dependent on the nature of the build.
 *
 *   Flat Build:  There is one, global SYSLOG mask that controls all output.
 *   Protected Build:  There are two SYSLOG masks.  One within the kernel
 *     that controls only kernel output.  And one in user-space that controls
 *     only user SYSLOG output.
 *   Kernel Build:  The kernel build is compliant with the POSIX requirement:
 *     There will be one mask for for each user process, controlling the
 *     SYSLOG output only form that process.  There will be a separate mask
 *     accessable only in the kernel code to control kernel SYSLOG output.
 *
 ****************************************************************************/

int setlogmask(int mask);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYSLOG_H */
