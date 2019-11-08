/****************************************************************************
 * include/nuttx/syslog/syslog.h
 * The NuttX SYSLOGing interface
 *
 *   Copyright (C) 2012, 2014-2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_SYSLOG_SYSLOG_H
#define __INCLUDE_NUTTX_SYSLOG_SYSLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdarg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_SYSLOG_INTBUFFER - Enables an interrupt buffer that will be used
 *   to serialize debug output from interrupt handlers.
 * CONFIG_SYSLOG_INTBUFSIZE - The size of the interrupt buffer in bytes.
 * CONFIG_SYSLOG_DEVPATH - The full path to the system logging device
 *
 * In addition, some SYSLOG device must also be enabled that will provide
 * the syslog output channel.  As of this writing, there are two SYSLOG
 * devices available:
 *
 *   1. A RAM SYSLOGing device that will log data into a circular buffer
 *      that can be dumped using the NSH dmesg command.  This device is
 *      described in the include/nuttx/syslog/ramlog.h header file.
 *
 *   2. And a generic character device that may be used as the SYSLOG.  The
 *      generic device interfaces are described in this file.  A disadvantage
 *      of using the generic character device for the SYSLOG is that it
 *      cannot handle debug output generated from interrupt level handlers.
 *
 * CONFIG_SYSLOG_CHAR - Enable the generic character device for the SYSLOG.
 *   The full path to the SYSLOG device is provided by CONFIG_SYSLOG_DEVPATH.
 *   A valid character device must exist at this path.  It will by opened
 *   by logic in syslog_initialize() based on the current configuration.
 *
 *   NOTE:  No more than one SYSLOG device should be configured.
 */

#if defined(CONFIG_SYSLOG_CHAR) && !defined(CONFIG_SYSLOG_DEVPATH)
#  define CONFIG_SYSLOG_DEVPATH "/dev/ttyS1"
#endif

#ifdef CONFIG_SYSLOG_INTBUFFER
#  ifndef CONFIG_SYSLOG_INTBUFSIZE
#    define CONFIG_SYSLOG_INTBUFSIZE 512
#  endif
#  if CONFIG_SYSLOG_INTBUFSIZE > 65535
#    undef  CONFIG_SYSLOG_INTBUFSIZE
#    define CONFIG_SYSLOG_INTBUFSIZE 65535
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Initialization phases */

enum syslog_init_e
{
  SYSLOG_INIT_RESET = 0, /* Power on SYSLOG initialization phase */
  SYSLOG_INIT_EARLY,     /* Early initialization in up_initialize() */
  SYSLOG_INIT_LATE       /* Late initialization in nx_start(). */
};

/* This structure provides the interface to a SYSLOG device */

typedef CODE ssize_t (*syslog_write_t)(FAR const char *buf, size_t buflen);
typedef CODE int (*syslog_putc_t)(int ch);
typedef CODE int (*syslog_flush_t)(void);

struct syslog_channel_s
{
  /* I/O redirection methods */

  syslog_putc_t  sc_putc;   /* Normal buffered output */
  syslog_putc_t  sc_force;  /* Low-level output for interrupt handlers */
  syslog_flush_t sc_flush;  /* Flush buffered output (on crash) */
#ifdef CONFIG_SYSLOG_WRITE
  syslog_write_t sc_write;  /* Write multiple bytes */
#endif

  /* Implementation specific logic may follow */
};

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
 * Name: syslog_channel
 *
 * Description:
 *   Configure the SYSLOGging function to use the provided channel to
 *   generate SYSLOG output.
 *
 * Input Parameters:
 *   channel - Provides the interface to the channel to be used.
 *
 * Returned Value:
 *   Zero (OK)is returned on  success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int syslog_channel(FAR const struct syslog_channel_s *channel);

/****************************************************************************
 * Name: syslog_initialize
 *
 * Description:
 *   One power up, the SYSLOG facility is non-existent or limited to very
 *   low-level output.  This function is called later in the initialization
 *   sequence after full driver support has been initialized.  It installs
 *   the configured SYSLOG drivers and enables full SYSLOGing capability.
 *
 *   This function performs these basic operations:
 *
 *   - Initialize the SYSLOG device
 *   - Call syslog_channel() to begin using that device.
 *
 *   If CONFIG_ARCH_SYSLOG is selected, then the architecture-specifica
 *   logic will provide its own SYSLOG device initialize which must include
 *   as a minimum a call to syslog_channel() to use the device.
 *
 * Input Parameters:
 *   phase - One of {SYSLOG_INIT_EARLY, SYSLOG_INIT_LATE}
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_SYSLOG
int syslog_initialize(enum syslog_init_e phase);
#else
#  define syslog_initialize(phase)
#endif

/****************************************************************************
 * Name: syslog_file_channel
 *
 * Description:
 *   Configure to use a file in a mounted file system at 'devpath' as the
 *   SYSLOG channel.
 *
 *   This tiny function is simply a wrapper around syslog_dev_initialize()
 *   and syslog_channel().  It calls syslog_dev_initialize() to configure
 *   the character file at 'devpath then calls syslog_channel() to use that
 *   device as the SYSLOG output channel.
 *
 *   File SYSLOG channels differ from other SYSLOG channels in that they
 *   cannot be established until after fully booting and mounting the target
 *   file system.  This function would need to be called from board-specific
 *   bring-up logic AFTER mounting the file system containing 'devpath'.
 *
 *   SYSLOG data generated prior to calling syslog_file_channel will, of
 *   course, not be included in the file.
 *
 *   NOTE interrupt level SYSLOG output will be lost in this case unless
 *   the interrupt buffer is used.
 *
 * Input Parameters:
 *   devpath - The full path to the file to be used for SYSLOG output.
 *     This may be an existing file or not.  If the file exists,
 *     syslog_file_channel() will append new SYSLOG data to the end of the
 *     file.  If it does not, then syslog_file_channel() will create the
 *     file.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_FILE
int syslog_file_channel(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: syslog_flush
 *
 * Description:
 *   This is called by system crash-handling logic.  It must flush any
 *   buffered data to the SYSLOG device.
 *
 *   Interrupts are disabled at the time of the crash and this logic must
 *   perform the flush using low-level, non-interrupt driven logic.
 *
 *   REVISIT:  There is an implementation problem in that if a character
 *   driver is the underlying device, then there is no mechanism to flush
 *   the data buffered in the driver with interrupts disabled.
 *
 *   Currently, this function on (a) dumps the interrupt buffer (if the
 *   SYSLOG interrupt buffer is enabled), and (b) only the SYSLOG interface
 *   supports supports the 'sc_force()' method.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK)is returned on  success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int syslog_flush(void);

/****************************************************************************
 * Name: nx_vsyslog
 *
 * Description:
 *   nx_vsyslog() handles the system logging system calls. It is functionally
 *   equivalent to vsyslog() except that (1) the per-process priority
 *   filtering has already been performed and the va_list parameter is
 *   passed by reference.  That is because the va_list is a structure in
 *   some compilers and passing of structures in the NuttX sycalls does
 *   not work.
 *
 ****************************************************************************/

int nx_vsyslog(int priority, FAR const IPTR char *src, FAR va_list *ap);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_SYSLOG_SYSLOG_H */
