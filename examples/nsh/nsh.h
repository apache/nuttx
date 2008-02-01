/****************************************************************************
 * examples/nsh/nsh.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __NSH_H
#define __HSH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_EXAMPLES_NSH_TELNET
#else
# include <stdio.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define NSH_MAX_ARGUMENTS     6

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*cmd_t)(FAR void *handle, int argc, char **argv);

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const char g_nshprompt[];
extern const char g_fmtargrequired[];
extern const char g_fmtarginvalid[];
extern const char g_fmtcmdnotfound[];
extern const char g_fmtcmdnotimpl[];
extern const char g_fmtnosuch[];
extern const char g_fmttoomanyargs[];
extern const char g_fmtcmdfailed[];
extern const char g_fmtcmdoutofmemory[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Message handler */

extern int nsh_parse(FAR void *handle, char *cmdline);

/* I/O interfaces */

#ifdef CONFIG_EXAMPLES_NSH_TELNET

extern int nsh_telnetmain(void);
extern int nsh_telnetout(FAR void *handle, const char *fmt, ...);

# define nsh_main()              nsh_telnetmain()
# define nsh_output(handle, ...) nsh_telnetout(handle, __VA_ARGS__)

#else

extern int nsh_serialmain(void);

# define nsh_main()              nsh_serialmain()
# define nsh_output(handle, ...) printf(__VA_ARGS__)

#endif

/* Shell command handlers */

#if CONFIG_NFILE_DESCRIPTORS > 0
extern void cmd_cat(FAR void *handle, int argc, char **argv);
extern void cmd_cp(FAR void *handle, int argc, char **argv);
#endif
extern void cmd_echo(FAR void *handle, int argc, char **argv);
extern void cmd_exec(FAR void *handle, int argc, char **argv);
extern void cmd_exit(FAR void *handle, int argc, char **argv);
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
extern void cmd_ifconfig(FAR void *handle, int argc, char **argv);
#endif
#if CONFIG_NFILE_DESCRIPTORS > 0
extern void cmd_ls(FAR void *handle, int argc, char **argv);
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
extern void cmd_mkdir(FAR void *handle, int argc, char **argv);
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
extern void cmd_mount(FAR void *handle, int argc, char **argv);
#endif
#endif
extern void cmd_ps(FAR void *handle, int argc, char **argv);
#ifndef CONFIG_DISABLE_ENVIRON
extern void cmd_set(FAR void *handle, int argc, char **argv);
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
extern void cmd_rm(FAR void *handle, int argc, char **argv);
extern void cmd_rmdir(FAR void *handle, int argc, char **argv);
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
extern void cmd_umount(FAR void *handle, int argc, char **argv);
#endif
#endif
#ifndef CONFIG_DISABLE_ENVIRON
extern void cmd_unset(FAR void *handle, int argc, char **argv);
#endif

#endif /* __NSH_H */
