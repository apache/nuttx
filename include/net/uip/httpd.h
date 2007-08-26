/* httpd.h
 *
 * Copyright (c) 2001-2005, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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
 */

#ifndef __HTTPD_H__
#define __HTTPD_H__

#include <sys/types.h>
#include <net/uip/psock.h>

#define HTTPD_FS_STATISTICS 1

struct httpd_fs_file
{
  char *data;
  int len;
};

struct httpd_state
{
  unsigned char timer;
  struct psock sin, sout;
  char inputbuf[50];
  char filename[20];
  char state;
  struct httpd_fs_file file;
  int len;
  char *scriptptr;
  int scriptlen;

  unsigned short count;
};

#ifdef HTTPD_FS_STATISTICS
#if HTTPD_FS_STATISTICS == 1
extern uint16 httpd_fs_count(char *name);
#endif /* HTTPD_FS_STATISTICS */
#endif /* HTTPD_FS_STATISTICS */

void httpd_init(void);
void httpd_log(char *msg);
void httpd_log_file(uint16 *requester, char *file);

/* file must be allocated by caller and will be filled in
 * by the function.
 */

int httpd_fs_open(const char *name, struct httpd_fs_file *file);
void httpd_fs_init(void);


#endif /* __HTTPD_H__ */
