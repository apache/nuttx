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
/* Configuration ************************************************************/
/* Some interfaces in this file are currently only available within the
 * kernel.  They could are available to applications in the flat build and
 * could be made available in the protected and kernel builds IF system
 * calls were added.
 *
 * REVISIT: For example, I don't yet know how to pass a va_list in a system
 * call so none of those interfaces.
 *
 * NOTE: In protected and kernel builds, there may also be a limit to the
 * number of parameters that are supported in the variable parameter list.
 */

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
#  undef __KERNEL__
#  define __KERNEL__ 1
#endif

/* syslog interface *********************************************************/
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

/* These low-level debug APIs are provided by the NuttX library.  These are
 * normally accessed via the macros in debug.h.  If the cross-compiler's
 * C pre-processor supports a variable number of macro arguments, then those
 * macros below will map all debug statements to one or the other of the
 * following.
 *
 * NOTE: In protected and kernel builds, there may be a limit to the number
 * of parameter that are supported in the variable parameter list.
 */

int syslog(int priority, FAR const char *format, ...);
#ifdef __KERNEL__
int vsyslog(int priority, FAR const char *src, va_list ap);
#endif

#ifdef CONFIG_ARCH_LOWPUTC
/* These are non-standard, low-level system logging interface.  The
 * difference between syslog() and lowsyslog() is that the syslog()
 * interface writes to the syslog device (usually fd=1, stdout) whereas
 * lowsyslog() uses a lower level interface that works even from interrupt
 * handlers.
 *
 * NOTE: In protected and kernel builds, there may be a limit to the number
 * of parameters that are supported in the variable parameter list.
 */

int lowsyslog(int priority, FAR const char *format, ...);
#  ifdef __KERNEL__
int lowvsyslog(int priority, FAR const char *format, va_list ap);
#  endif

#else
/* If the platform cannot support lowsyslog, then we will substitute the
 * standard syslogging functions.  These will, however, probably cause
 * problems if called from interrupt handlers, depending upon the nature of
 * the underlying syslog device.
 */

#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define lowsyslog(p,f,...) syslog(p,f,##__VA_ARGS__)
#  else
#    define lowsyslog (void)
#  endif
#  ifdef __KERNEL__
#   define lowvsyslog(p,f,a) vsyslog(p,f,a)
#  endif
#endif

/* The setlogmask() function sets the logmask and returns the previous
 * mask. If the mask argument is 0, the current logmask is not modified.
 *
 * REVISIT: In the current implementation, the syslog interfaces are not
 * part of libc, but are part of the kernel logic.  Per POSIX the syslog
 * mask should be a per-process value but in this implementation it is
 * a single, kernel value.  This could easily fixed (at least in the
 * kernel build) by simply moving all of the syslog logic back into libc
 * (where it once was).
 */

int setlogmask(int mask);

#if defined(CONFIG_SYSLOG_ENABLE) && defined(__KERNEL__)
/* Non-standard interface to enable or disable syslog output */

void syslog_enable(bool enable);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYSLOG_H */
