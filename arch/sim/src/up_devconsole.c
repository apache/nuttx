/************************************************************
 * up_devconsole.c
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

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdio.h>
#include <errno.h>

#include <nuttx/fs.h>

#include "up_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

#define READ   3
#define WRITE  4

/************************************************************
 * Private Function Prototypes
 ************************************************************/

static ssize_t devconsole_read(struct file *, char *, size_t);
static ssize_t devconsole_write(struct file *, const char *, size_t);

/************************************************************
 * Private Data
 ************************************************************/

static struct file_operations devconsole_fops =
{
  .read		= devconsole_read,
  .write	= devconsole_write,
};

/************************************************************
 * Private Functions
 ************************************************************/

static inline int up_read(int fd, void* buf, size_t count)
{
  uint32 result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (READ), "b" ((uint32)(fd)), "c" ((uint32)(buf)), "d" ((uint32)(count)) \
                    : "memory");

  return (int)result;
}

static inline int up_write(int fd, const void* buf, size_t count)
{
  uint32 result;

  __asm__ volatile ("int $0x80" \
                    : "=a" (result) \
                    : "0" (WRITE), "b" ((uint32)(fd)), "c" ((uint32)(buf)), "d" ((uint32)(count)) \
                    : "memory");

  return (int)result;
}

static inline int up_check_result(int result)
{
  if (result >= (uint32)(-(128 + 1)))
    {
      *get_errno_ptr() = -result;
      result = ERROR;
    }
  return result;
}

static ssize_t devconsole_read(struct file *filp, char *buffer, size_t len)
{
  return up_check_result(up_read(1, buffer, len));
}

static ssize_t devconsole_write(struct file *filp, const char *buffer, size_t len)
{
  return up_check_result(up_write(1, buffer, len));
}

/************************************************************
 * Public Funtions
 ************************************************************/

void up_devconsole(void)
{
  (void)register_inode("/dev/console", &devconsole_fops, 0666, NULL);
}

int up_putc(int ch)
{
  char b = ch;
  (void)up_write(1, &b, 1);
  return ch;
}
