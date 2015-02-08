/****************************************************************************
 * include/nuttx/syslog/ramlog.h
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville. All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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

#ifndef __INCLUDE_NUTTX_SYSLOG_SYSLOG_CONSOLE_H
#define __INCLUDE_NUTTX_SYSLOG_SYSLOG_CONSOLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_SYSLOG_CONSOLE

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SYSLOG_CONSOLE - Use the syslog logging output as a system console.
 *   If this feature is enabled (along with CONFIG_DEV_CONSOLE), then all
 *   console output will be re-directed to a syslog_putc function.  This
 *   is useful, for example, if the only console is a Telnet console.  Then
 *   in that case, console output from non-Telnet threads will go to the
 *   syslog_putc output.
 */

#ifndef CONFIG_DEV_CONSOLE
#  undef CONFIG_SYSLOG_CONSOLE
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_console_init
 *
 * Description:
 *   Create the console logging device and register it at the '/dev/console' 
 *   path.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CONSOLE
int syslog_console_init(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SYSLOG_CONSOLE */
#endif /* __INCLUDE_NUTTX_SYSLOG_SYSLOG_CONSOLE_H */
