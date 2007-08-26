/* smtp.h
 * SMTP header file
 * Author: Adam Dunkels <adam@dunkels.com>
 *
 * Copyright (c) 2002, Adam Dunkels.
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

#ifndef __SMTP_H__
#define __SMTP_H__

#include <sys/types.h>

#include <net/uip/uipopt.h>

/* Error number that signifies a non-error condition. */

#define SMTP_ERR_OK 0

/* Callback function that is called when an e-mail transmission is
 * done.
 *
 * This function must be implemented by the module that uses the SMTP
 * module.
 *
 * error The number of the error if an error occured, or
 * SMTP_ERR_OK.
 */

void smtp_done(unsigned char error);

void smtp_init(void);
void smtp_configure(char *localhostname, uint16 *smtpserver);
unsigned char smtp_send(char *to, char *from,
			char *subject, char *msg,
			uint16 msglen);

struct smtp_state
{
  uint8 state;
  char *to;
  char *from;
  char *subject;
  char *msg;
  uint16 msglen;

  uint16 sentlen, textlen;
  uint16 sendptr;
};

#endif /* __SMTP_H__ */
