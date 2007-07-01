/****************************************************************************
 * nsh.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define errno (*get_errno_ptr())

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*cmd_t)(int argc, char **argv);

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

#if CONFIG_NFILE_DESCRIPTORS > 0
void cmd_cat(int argc, char **argv);
void cmd_cp(int argc, char **argv);
#endif
void cmd_exec(int argc, char **argv);
#if CONFIG_NFILE_DESCRIPTORS > 0
void cmd_ls(int argc, char **argv);
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_mkdir(int argc, char **argv);
void cmd_mount(int argc, char **argv);
#endif
void cmd_ps(int argc, char **argv);
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_umount(int argc, char **argv);
#endif

#endif /* __NSH_H */
