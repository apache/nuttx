/****************************************************************************
 * include/syslog.h
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

#ifndef __INCLUDE_SYSLOG_H
#define __INCLUDE_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

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
 *   LOG_FTP      - FTP daemon
 *   LOG_KERN     - Kernel messages (these can't be generated from user
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
#define LOG_UPTO(p)   ((1 << ((p)+1)) - 1)
#define LOG_ALL       0xff

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: openlog
 *
 * Description:
 *   The openlog() function sets process attributes that affect subsequent
 *   calls to syslog(). The ident argument is a string that is prepended to
 *   every message. The logopt argument indicates logging options. Values
 *   for logopt are constructed by a bitwise-inclusive OR of zero or more of
 *   the following:
 *
 *     LOG_PID - Log the process ID with each message. This is useful for
 *       identifying specific processes.
 *
 *     LOG_CONS - Write messages to the system console if they cannot be
 *       sent to the logging facility. The syslog() function ensures that
 *       the process does not acquire the console as a controlling terminal
 *       in the process of writing the message.
 *
 *     LOG_NDELAY - Open the connection to the logging facility immediately.
 *       Normally the open is delayed until the first message is logged.
 *       This is useful for programs that need to manage the order in which
 *       file descriptors are allocated.
 *
 *     LOG_ODELAY - Delay open until syslog() is called.
 *
 *     LOG_NOWAIT - Do not wait for child processes that may have been
 *       created during the course of logging the message. This option
 *       should be used by processes that enable notification of child
 *       termination using SIGCHLD, since syslog() may otherwise block
 *       waiting for a child whose exit status has already been collected.
 *
 *   The facility argument encodes a default facility to be assigned to all
 *   messages that do not have an explicit facility already encoded. The
 *   initial default facility is LOG_USER.
 *
 *   It is not necessary to call openlog() prior to calling syslog().
 *
 ****************************************************************************/

#if 0 /* Not supported */
void openlog(FAR const char *ident, int option, int facility);
#endif

/****************************************************************************
 * Name: closelog
 *
 * Description:
 *   The openlog() and syslog() functions may allocate a file descriptor.
 *   The closelog() function will close any open file descriptors allocated
 *   by previous calls to openlog() or syslog().
 *
 ****************************************************************************/

#if 0 /* Not supported */
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

void syslog(int priority, FAR const IPTR char *fmt, ...) sysloglike(2, 3);
void vsyslog(int priority, FAR const IPTR char *fmt, va_list ap)
     sysloglike(2, 0);

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
 *   Per OpenGroup.org "If the maskpri argument is 0, the current log mask
 *   is not modified."  In this implementation, the value zero is permitted
 *   in order to disable all syslog levels.
 *
 *   NOTE:  setlogmask is not a thread-safe, re-entrant function.  Concurrent
 *   use of setlogmask() will have undefined behavior.
 *
 *   REVISIT: Per POSIX the syslog mask should be a per-process value but in
 *   NuttX, the scope of the mask is dependent on the nature of the build:
 *
 *   Flat Build:  There is one, global SYSLOG mask that controls all output.
 *   Protected Build:  There are two SYSLOG masks.  One within the kernel
 *     that controls only kernel output.  And one in user-space that controls
 *     only user SYSLOG output.
 *   Kernel Build:  The kernel build is compliant with the POSIX requirement:
 *     There will be one mask for for each user process, controlling the
 *     SYSLOG output only form that process.  There will be a separate mask
 *     accessible only in the kernel code to control kernel SYSLOG output.
 *
 ****************************************************************************/

int setlogmask(int mask);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYSLOG_H */
