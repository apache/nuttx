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
#include <sys/stat.h>

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
static const char g_testdir2[]       = "/mnt/fs/NewDir";
static const char g_testfile1[]      = "/mnt/fs/TestDir/TestFile.txt";
static const char g_testfile2[]      = "/mnt/fs/TestDir/WritTest.txt";
static const char g_testfile3[]      = "/mnt/fs/NewDir/WritTest.txt";
static const char g_testmsg[]        = "This is a write test";

static int        g_nerrors          = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fail_read_open
 ****************************************************************************/

static void fail_read_open(const char *path, int expectederror)
{
  int fd;

  printf("fail_read_open: Try open(%s) for reading\n", path);

  fd = open(path, O_RDONLY);
  if (fd >= 0)
    {
      printf("fail_read_open: ERROR open(%s) succeeded\n", path);
      g_nerrors++;
      close(fd);
    }
  else if (*get_errno_ptr() != expectederror)
    {
      printf("fail_read_open: ERROR open(%s) failed with errno=%d (expected %d)\n",
             path, *get_errno_ptr(), expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: read_test_file
 ****************************************************************************/

static void read_test_file(const char *path)
{
  char buffer[128];
  int  nbytes;
  int  fd;

  /* Read a test file that is already on the test file system image */

  printf("read_test_file: opening %s for reading\n", path);

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("read_test_file: ERROR failed to open %s, errno=%d\n",
             path, *get_errno_ptr());
      g_nerrors++;
    }
  else
    {
      memset(buffer, 0, 128);
      nbytes = read(fd, buffer, 128);
      if (nbytes < 0)
        {
          printf("read_test_file: ERROR failed to read from %s, errno=%d\n",
                 path, *get_errno_ptr());
          g_nerrors++;
        }
      else
        {
          buffer[127]='\0';
          printf("read_test_file: Read \"%s\" from %s\n", buffer, path);
        }
      close(fd);
    }
}

/****************************************************************************
 * Name: write_test_file
 ****************************************************************************/

static void write_test_file(const char *path)
{
  int fd;

  /* Write a test file into a pre-existing file on the test file system */

  printf("write_test_file: opening %s for writing\n", path);

  fd = open(path, O_WRONLY|O_CREAT|O_TRUNC, 0644);
  if (fd < 0)
    {
      printf("write_test_file: ERROR failed to open %s for writing, errno=%d\n",
             path, *get_errno_ptr());
      g_nerrors++;
    }
  else
    {
      int nbytes = write(fd, g_testmsg, strlen(g_testmsg));
      if (nbytes < 0)
        {
          printf("write_test_file: ERROR failed to write to %s, errno=%d\n",
                 path, *get_errno_ptr());
          g_nerrors++;
        }
      else
        {
          printf("write_test_file: wrote %d bytes to %s\n", nbytes, path);
        }
      close(fd);
    }
}

/****************************************************************************
 * Name: fail_mkdir
 ****************************************************************************/

static void fail_mkdir(const char *path, int expectederror)
{
  int ret;

  /* Try mkdir() against a file or directory.  It should fail with expectederror */

  printf("fail_mkdir: Try mkdir(%s)\n", path);

  ret = mkdir(path, 0666);
  if (ret == 0)
    {
      printf("fail_mkdir: ERROR mkdir(%s) succeeded\n", path);
      g_nerrors++;
    }
  else if (*get_errno_ptr() != expectederror)
    {
      printf("fail_mkdir: ERROR mkdir(%s) failed with errno=%d (expected %d)\n",
             path, *get_errno_ptr(), expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_mkdir
 ****************************************************************************/

static void succeed_mkdir(const char *path)
{
  int ret;

  printf("succeed_mkdir: Try mkdir(%s)\n", path);

  ret = mkdir(path, 0666);
  if (ret != 0)
    {
      printf("succeed_mkdir: ERROR mkdir(%s) failed with errno=%d\n",
             path, *get_errno_ptr());
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: fail_rmdir
 ****************************************************************************/

static void fail_rmdir(const char *path, int expectederror)
{
  int ret;

  /* Try rmdir() against a file or directory.  It should fail with expectederror */

  printf("fail_rmdir: Try rmdir(%s)\n", path);

  ret = rmdir(path);
  if (ret == 0)
    {
      printf("fail_rmdir: ERROR rmdir(%s) succeeded\n", path);
      g_nerrors++;
    }
  else if (*get_errno_ptr() != expectederror)
    {
      printf("fail_rmdir: ERROR rmdir(%s) failed with errno=%d (expected %d)\n",
             path, *get_errno_ptr(), expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_rmdir
 ****************************************************************************/

static void succeed_rmdir(const char *path)
{
  int ret;

  printf("succeed_rmdir: Try rmdir(%s)\n", path);

  ret = rmdir(path);
  if (ret != 0)
    {
      printf("succeed_rmdir: ERROR rmdir(%s) failed with errno=%d\n",
             path, *get_errno_ptr());
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: fail_unlink
 ****************************************************************************/

static void fail_unlink(const char *path, int expectederror)
{
  int ret;

  /* Try unlink() against a file or directory.  It should fail with expectederror */

  printf("fail_unlink: Try unlink(%s)\n", path);

  ret = unlink(path);
  if (ret == 0)
    {
      printf("fail_unlink: ERROR unlink(%s) succeeded\n", path);
      g_nerrors++;
    }
  else if (*get_errno_ptr() != expectederror)
    {
      printf("fail_unlink: ERROR unlink(%s) failed with errno=%d (expected %d)\n",
             path, *get_errno_ptr(), expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_unlink
 ****************************************************************************/

static void succeed_unlink(const char *path)
{
  int ret;

  /* Try unlink() against the test file.  It should succeed. */

  printf("succeed_unlink: Try unlink(%s)\n", path);

  ret = unlink(path);
  if (ret != 0)
    {
      printf("succeed_unlink: ERROR unlink(%s) failed with errno=%d\n",
             path, *get_errno_ptr());
      g_nerrors++;
    }
}

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
  int  ret;

  /* Mount the test file system (see arch/sim/src/up_deviceimage.c */

  printf("user_start: mounting %s filesystem at target=%s with source=%s\n",
         g_filesystemtype, g_target, g_source);

  ret = mount(g_source, g_target, g_filesystemtype, 0, NULL);
  printf("user_start: mount() returned %d\n", ret);

  if (ret == 0)
    {
      /* Read a test file that is already on the test file system image */

      read_test_file(g_testfile1);

      /* Write a test file into a pre-existing directory on the test file system */

      write_test_file(g_testfile2);

      /* Read the file that we just wrote */

      read_test_file(g_testfile2);

      /* Try rmdir() against a file on the directory.  It should fail with ENOTDIR */

      fail_rmdir(g_testfile1, ENOTDIR);

      /* Try rmdir() against the test directory.  It should fail with ENOTEMPTY */

      fail_rmdir(g_testdir1, ENOTEMPTY);

      /* Try unlink() against the test directory.  It should fail with EISDIR */

      fail_unlink(g_testdir1, EISDIR);

      /* Try unlink() against the test file1.  It should succeed. */

      succeed_unlink(g_testfile1);

      /* Attempt to open testfile1 should fail with ENOENT */

      fail_read_open(g_testfile1, ENOENT);

      /* Try rmdir() against the test directory.  It should still fail with ENOTEMPTY */

      fail_rmdir(g_testdir1, ENOTEMPTY);

      /* Try mkdir() against the test file2.  It should fail with EEXIST. */

      fail_mkdir(g_testfile2, EEXIST);

      /* Try unlink() against the test file2.  It should succeed. */

      succeed_unlink(g_testfile2);

      /* Try mkdir() against the test dir1.  It should fail with EEXIST. */

      fail_mkdir(g_testdir1, EEXIST);

      /* Try rmdir() against the test directory.  It should now succeed. */

      succeed_rmdir(g_testdir1);

      /* Try mkdir() against the test dir2.  It should succeed */

      succeed_mkdir(g_testdir2);

      /* Try mkdir() against the test dir2.  It should fail with EXIST */

      fail_mkdir(g_testdir2, EEXIST);

      /* Write a test file into a new directory on the test file system */

      write_test_file(g_testfile3);

      /* Read the file that we just wrote */

      read_test_file(g_testfile3);

      /* Unmount the file system */

      printf("user_start: Try unmount(%s)\n", g_target);

      ret = umount(g_target);
      if (ret != 0)
        {
          printf("user_start: ERROR umount() failed, errno %d\n", *get_errno_ptr());
          g_nerrors++;
        }

      printf("user_start: %d errors reported\n", g_nerrors);
    }
  
  fflush(stdout);
  return 0;
}
