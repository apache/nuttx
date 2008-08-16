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
#ifdef CONFIG_EXAMPLES_NSH_CONSOLE
# include <stdio.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#if !defined(CONFIG_EXAMPLES_NSH_CONSOLE) && !defined(CONFIG_EXAMPLES_NSH_TELNET)
#  error "No NSH front end defined"
#endif

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

#define nsh_clone(v)           (v)->clone(v)
#define nsh_addref(v)          (v)->addref(v)
#define nsh_release(v)         (v)->release(v)
#define nsh_linebuffer(v)      (v)->linebuffer(v)
#define nsh_redirect(v,f)      (v)->redirect(v,f)
#define nsh_undirect(v,d)      (v)->undirect(v,d)

#ifdef CONFIG_CPP_HAVE_VARARGS
# define nsh_output(v, fmt...) (v)->output(v, ##fmt)
#else
# define nsh_output            vtbl->output
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nsh_vtbl_s
{
  FAR struct nsh_vtbl_s *(*clone)(FAR struct nsh_vtbl_s *vtbl);
  void (*addref)(FAR struct nsh_vtbl_s *vtbl);
  void (*release)(FAR struct nsh_vtbl_s *vtbl);
  int (*output)(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);
  FAR char *(*linebuffer)(FAR struct nsh_vtbl_s *vtbl);
  FAR void *(*redirect)(FAR struct nsh_vtbl_s *vtbl, int fd);
  void (*undirect)(FAR struct nsh_vtbl_s *vtbl, FAR void *direct);
};

typedef void (*cmd_t)(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

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

extern int nsh_parse(FAR struct nsh_vtbl_s *vtbl, char *cmdline);

/* I/O interfaces */

#ifdef CONFIG_EXAMPLES_NSH_CONSOLE
extern int nsh_consolemain(int argc, char *argv[]);
#endif

#ifdef CONFIG_EXAMPLES_NSH_TELNET
extern int nsh_telnetmain(int argc, char *argv[]);
#endif

/* Shell command handlers */

extern void cmd_echo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
extern void cmd_exec(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
extern void cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
extern void cmd_ps(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

#if CONFIG_NFILE_DESCRIPTORS > 0
  extern void cmd_cat(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
  extern void cmd_cp(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
  extern void cmd_ls(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
# if CONFIG_NFILE_STREAMS > 0
  extern void cmd_sh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
# endif  /* CONFIG_NFILE_STREAMS */
# ifndef CONFIG_DISABLE_MOUNTPOINT
    extern void cmd_mkdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
    extern void cmd_mkfifo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
    extern void cmd_rm(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
    extern void cmd_rmdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   ifdef CONFIG_FS_FAT
      extern void cmd_mkfatfs(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
      extern void cmd_mount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
      extern void cmd_umount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif /* CONFIG_FS_FAT */
# endif /* !CONFIG_DISABLE_MOUNTPOINT */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
  extern void cmd_ifconfig(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif

#ifndef CONFIG_DISABLE_ENVIRON
  extern void cmd_set(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
  extern void cmd_unset(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif /* CONFIG_DISABLE_ENVIRON */

#ifndef CONFIG_DISABLE_SIGNALS
  extern void cmd_sleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
  extern void cmd_usleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif /* CONFIG_DISABLE_SIGNALS */

#endif /* __NSH_H */
