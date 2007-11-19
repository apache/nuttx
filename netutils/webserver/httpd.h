/****************************************************************************
 * netutils/webserver/httpd.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2001-2005, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _NETUTILS_WEBSERVER_HTTPD_H
#define _NETUTILS_WEBSERVER_HTTPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define HTTPD_FS_STATISTICS 1
#define HTTPD_IOBUFFER_SIZE 512

#ifndef CONFIG_EXAMPLES_UIP_HTTPDSTACKSIZE
# define CONFIG_EXAMPLES_UIP_HTTPDSTACKSIZE 4096
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct httpd_fs_file
{
  char *data;
  int len;
};

struct httpd_state
{
  char ht_buffer[HTTPD_IOBUFFER_SIZE];
  char filename[20];
  struct httpd_fs_file file;
  int sockfd;                         /* The socket descriptor from accept() */
  int len;
  char *scriptptr;
  int scriptlen;

  unsigned short count;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef HTTPD_FS_STATISTICS
#if HTTPD_FS_STATISTICS == 1
extern uint16 httpd_fs_count(char *name);
#endif /* HTTPD_FS_STATISTICS */
#endif /* HTTPD_FS_STATISTICS */

/* file must be allocated by caller and will be filled in by the function. */

int  httpd_fs_open(const char *name, struct httpd_fs_file *file);
void httpd_fs_init(void);

#endif /* _NETUTILS_WEBSERVER_HTTPD_H */
