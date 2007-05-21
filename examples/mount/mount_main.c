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

static const char g_testdir1[]       = "/mnt/fs/TestDir";
static const char g_testfile1[]      = "/mnt/fs/TestDir/TestFile.txt";
static const char g_testfile2[]      = "/mnt/fs/TestDir/WritTest.txt";
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
  char buffer[128];
  int  nbytes;
  int  ret;

  /* Mount the test file system (see arch/sim/src/up_deviceimage.c */

  printf("main: mounting %s filesystem at target=%s with source=%s\n",
         g_filesystemtype, g_target, g_source);

  ret = mount(g_source, g_target, g_filesystemtype, 0, NULL);
  printf("main: mount() returned %d\n", ret);

  if (ret == 0)
    {
      /* Read a test file that is already on the test file system image */

      printf("main: opening %s for reading\n", g_testfile1);

      int fd = open(g_testfile1, O_RDONLY);
      if (fd < 0)
        {
          printf("main: ERROR failed to open %s, errno=%d\n",
                 g_testfile1, *get_errno_ptr());
        }
      else
        {
          memset(buffer, 0, 128);
          nbytes = read(fd, buffer, 128);
          if (nbytes < 0)
            {
              printf("main: ERROR failed to read from %s, errno=%d\n",
                     g_testfile1, *get_errno_ptr());
            }
          else
          {
              buffer[127]='\0';
              printf("main: Read \"%s\" from %s\n", buffer, g_testfile1);
          }
          close(fd);
        }

      /* Write a test file into a pre-existing file on the test file system */

      printf("main: opening %s for writing\n", g_testfile2);

      fd = open(g_testfile2, O_WRONLY|O_CREAT|O_TRUNC, 0644);
      if (fd < 0)
        {
          printf("main: ERROR failed to open %s for writing, errno=%d\n",
                 g_testfile2, *get_errno_ptr());
        }
      else
        {
          int nbytes = write(fd, g_testmsg, strlen(g_testmsg));
          if (nbytes < 0)
            {
              printf("main: ERROR failed to write to %s, errno=%d\n",
                     g_testfile2, *get_errno_ptr());
            }
          else
            {
              printf("main: wrote %d bytes to %s\n", nbytes, g_testfile2);
            }
          close(fd);
        }

      /* Read the file that we just wrote */

      printf("main: opening %s for reading\n", g_testfile2);

      fd = open(g_testfile2, O_RDONLY);
      if (fd < 0)
        {
          printf("main: ERRORfailed to open %s for reading, errno=%d\n",
                 g_testfile2, *get_errno_ptr());
        }
      else
        {
          memset(buffer, 0, 128);
          nbytes = read(fd, buffer, 128);
          if (nbytes < 0)
            {
              printf("main: ERROR failed to read from %s, errno=%d\n",
                     g_testfile2, *get_errno_ptr());
            }
          else
          {
              buffer[127]='\0';
              printf("main: Read \"%s\" from %s\n", buffer, g_testfile2);
          }
          close(fd);
        }

      /* Try rmdir() against a file on the directory.  It should fail with ENOTDIR */

      printf("main: Try rmdir(%s)\n", g_testfile1);

      ret = rmdir(g_testfile1);
      if (ret == 0)
        {
          printf("main: ERROR rmdir(%s) succeeded\n", g_testfile1);
        }
      else if (*get_errno_ptr() != ENOTDIR)
      {
          printf("main: ERROR rmdir(%s) failed with errno=%d\n",
                     g_testfile1, *get_errno_ptr());
      }

      /* Try rmdir() against the test directory.  It should fail with ENOTEMPTY */

      printf("main: Try rmdir(%s)\n", g_testdir1);

      ret = rmdir(g_testdir1);
      if (ret == 0)
        {
          printf("main: ERROR rmdir(%s) succeeded\n", g_testdir1);
        }
      else if (*get_errno_ptr() != ENOTEMPTY)
        {
          printf("main: ERROR rmdir(%s) failed with errno=%d\n",
                     g_testdir1, *get_errno_ptr());
        }

      /* Try unlink() against the test directory.  It should fail with EISDIR */

      printf("main: Try unlink(%s)\n", g_testdir1);

      ret = unlink(g_testdir1);
      if (ret == 0)
        {
          printf("main: ERROR unlink(%s) succeeded\n", g_testdir1);
        }
      else if (*get_errno_ptr() != EISDIR)
        {
          printf("main: ERROR unlink(%s) failed with errno=%d\n",
                     g_testdir1, *get_errno_ptr());
        }

      /* Try unlink() against the test file1.  It should succeed. */

      printf("main: Try unlink(%s)\n", g_testfile1);

      ret = unlink(g_testfile1);
      if (ret != 0)
        {
          printf("main: ERROR unlink(%s) failed with errno=%d\n",
                     g_testfile1, *get_errno_ptr());
        }

      /* Attempt to open testfile1 should fail with ENOENT */

      printf("main: Try open(%s) for reading\n", g_testfile1);

      fd = open(g_testfile1, O_RDONLY);
      if (fd >= 0)
        {
          printf("main: ERROR open(%s) succeeded\n", g_testfile1);
          close(fd);
        }
      else if (*get_errno_ptr() != ENOENT)
        {
          printf("main: ERROR open(%s) failed with errno=%d\n",
                     g_testfile1, *get_errno_ptr());
        }

      /* Try rmdir() against the test directory.  It should still fail with ENOTEMPTY */

      printf("main: Try rmdir(%s)\n", g_testdir1);

      ret = rmdir(g_testdir1);
      if (ret == 0)
        {
          printf("main: ERROR rmdir(%s) succeeded\n", g_testdir1);
        }
      else if (*get_errno_ptr() != ENOTEMPTY)
        {
          printf("main: ERROR rmdir(%s) failed with errno=%d\n",
                     g_testdir1, *get_errno_ptr());
        }

      /* Try unlink() against the test file2.  It should succeed. */

      printf("main: Try unlink(%s)\n", g_testfile2);

      ret = unlink(g_testfile2);
      if (ret != 0)
        {
          printf("main: ERROR unlink(%s) failed with errno=%d\n",
                     g_testfile2, *get_errno_ptr());
        }

      /* Try rmdir() against the test directory.  It should now succeed. */

      printf("main: Try rmdir(%s)\n", g_testdir1);

      ret = rmdir(g_testdir1);
      if (ret != 0)
        {
          printf("main: ERROR rmdir(%s) failed with errno=%d\n",
                     g_testdir1, *get_errno_ptr());
        }

      /* Unmount the file system */

      printf("main: Try unmount(%s)\n", g_target);

      ret = umount(g_target);
      if (ret != 0)
        {
          printf("main: ERROR umount() failed, errno %d\n", *get_errno_ptr());
        }
    }
  
  fflush(stdout);
  return 0;
}
