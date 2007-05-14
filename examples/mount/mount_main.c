/****************************************************************************
 * mount_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/mount.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_source[]         = "/dev/blkdev";
static const char g_target[]         = "/mnt/fs";
static const char g_filesystemtype[] = "vfat";

static const char g_testfile1[]      = "/mnt/fs/nuttx-test/test-file.txt";
static const char g_testfile2[]      = "/mnt/fs/nuttx-test/write-test.txt";
static const char g_testmsg[]        = "This is a write test";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_initialize
 ****************************************************************************/

void user_initialize(void)
{
}

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  int ret;

  printf("main: mounting %s filesystem at target=%s with source=%s\n",
         g_filesystemtype, g_target, g_source);

  ret = mount(g_source, g_target, g_filesystemtype, 0, NULL);
  printf("main: mount() returned %d\n", ret);

  if (ret == 0)
    {
      printf("main: opening %s for reading\n", g_testfile1);

      int fd = open(g_testfile1, O_RDONLY);
      if (fd < 0)
        {
          printf("main: failed open %s, errno=%d\n", g_testfile1, *get_errno_ptr());
        }
      else
        {
          char buffer[128];
          int nbytes = read(fd, buffer, 128);
          if (nbytes < 0)
            {
              printf("main: failed to read from %s, errno=%d\n", g_testfile1, *get_errno_ptr());
            }
          else
          {
              buffer[127]='\0';
              printf("main: Read \"%s\" from %s\n", buffer, g_testfile1);
          }
          close(fd);
        }

      printf("main: opening %s for writing\n", g_testfile2);

      fd = open(g_testfile2, O_WRONLY|O_CREAT|O_TRUNC, 0644);
      if (fd < 0)
        {
          printf("main: failed open %s, errno=%d\n", g_testfile2, *get_errno_ptr());
        }
      else
        {
          int nbytes = write(fd, g_testmsg, strlen(g_testmsg));
          if (nbytes < 0)
            {
              printf("main: failed to write to %s, errno=%d\n", g_testfile2, *get_errno_ptr());
            }
          close(fd);
        }

      ret = umount(g_target);
      printf("main: umount() returned %d\n", ret);
    }
  
  fflush(stdout);
  return 0;
}
