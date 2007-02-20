/************************************************************
 * fs_internal.h
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
 ************************************************************/

#ifndef __FS_INTERNAL_H
#define __FS_INTERNAL_H

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs.h>
#include <nuttx/compiler.h>

/************************************************************
 * Definitions
 ************************************************************/

#define FSNODEFLAG_DELETED 0x00000001

/************************************************************
 * Public Types
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS >0
extern struct file files[CONFIG_NFILE_DESCRIPTORS];
#endif
extern struct inode *root_inode;

/************************************************************
 * Pulblic Function Prototypes
 ************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* fs_inode.c ***********************************************/

EXTERN struct inode *inode_find(const char *path);
EXTERN void inode_addref(struct inode *inode);
EXTERN void inode_release(struct inode *inode);

/* fs_files.c ***********************************************/

#if CONFIG_NFILE_DESCRIPTORS >0
EXTERN void weak_function files_initialize(void);
EXTERN int  files_allocate(struct inode *inode, int oflags, off_t pos);
EXTERN void files_release(int filedes);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif 



#endif /* __FS_INTERNAL_H */
