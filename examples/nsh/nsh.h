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

/* This is the maximum number of arguments that will be accepted for a command */

#define NSH_MAX_ARGUMENTS 6

/* strerror() produces much nicer output but is, however, quite large and
 * will only be used if CONFIG_EXAMPLES_NSH_STRERROR is defined.
 */

#ifdef CONFIG_EXAMPLES_NSH_STRERROR
#  define NSH_ERRNO strerror(errno)
#else
#  define NSH_ERRNO errno
#endif

/* Maximum size of one command line (telnet or serial) */

#ifndef CONFIG_EXAMPLES_NSH_LINELEN
#  define CONFIG_EXAMPLES_NSH_LINELEN 80
#endif

/* The following two settings are used only in the telnetd interface */

#ifndef CONFIG_EXAMPLES_NSH_IOBUFFER_SIZE
# define CONFIG_EXAMPLES_NSH_IOBUFFER_SIZE 512
#endif

/* As threads are created to handle each request, a stack must be allocated
 * for the thread.  Use a default if the user provided no stacksize.
 */

#ifndef CONFIG_EXAMPLES_NSH_STACKSIZE
# define CONFIG_EXAMPLES_NSH_STACKSIZE 4096
#endif

/* Define to enable dumping of all input/output buffers */

#undef CONFIG_EXAMPLES_NSH_TELNETD_DUMPBUFFER
#undef CONFIG_EXAMPLES_NSH_FULLPATH

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

extern int   nsh_parse(FAR void *handle, char *cmdline);

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
extern char *nsh_linebuffer(FAR void *handle);

/* Shell command handlers */

extern void cmd_echo(FAR void *handle, int argc, char **argv);
extern void cmd_exec(FAR void *handle, int argc, char **argv);
extern void cmd_exit(FAR void *handle, int argc, char **argv);
extern void cmd_ps(FAR void *handle, int argc, char **argv);

#if CONFIG_NFILE_DESCRIPTORS > 0
  extern void cmd_cat(FAR void *handle, int argc, char **argv);
  extern void cmd_cp(FAR void *handle, int argc, char **argv);
  extern void cmd_ls(FAR void *handle, int argc, char **argv);
# if CONFIG_NFILE_STREAMS > 0
  extern void cmd_sh(FAR void *handle, int argc, char **argv);
# endif  /* CONFIG_NFILE_STREAMS */
# ifndef CONFIG_DISABLE_MOUNTPOINT
    extern void cmd_mkdir(FAR void *handle, int argc, char **argv);
    extern void cmd_mkfifo(FAR void *handle, int argc, char **argv);
    extern void cmd_rm(FAR void *handle, int argc, char **argv);
    extern void cmd_rmdir(FAR void *handle, int argc, char **argv);
#   ifdef CONFIG_FS_FAT
      extern void cmd_mkfatfs(FAR void *handle, int argc, char **argv);
      extern void cmd_mount(FAR void *handle, int argc, char **argv);
      extern void cmd_umount(FAR void *handle, int argc, char **argv);
#   endif /* CONFIG_FS_FAT */
# endif /* !CONFIG_DISABLE_MOUNTPOINT */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
  extern void cmd_ifconfig(FAR void *handle, int argc, char **argv);
#endif

#ifndef CONFIG_DISABLE_ENVIRON
  extern void cmd_set(FAR void *handle, int argc, char **argv);
  extern void cmd_unset(FAR void *handle, int argc, char **argv);
#endif /* CONFIG_DISABLE_ENVIRON */

#ifndef CONFIG_DISABLE_SIGNALS
  extern void cmd_sleep(FAR void *handle, int argc, char **argv);
  extern void cmd_usleep(FAR void *handle, int argc, char **argv);
#endif /* CONFIG_DISABLE_SIGNALS */

#endif /* __NSH_H */
