/****************************************************************************
 * examples/pipe/redirect_test.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "pipe.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define READ_SIZE 37

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: redirect_reader
 ****************************************************************************/

static int redirect_reader(int argc, char *argv[])
{
  char buffer[READ_SIZE];
  int fd = (int)argv[0];
  int ret;
  int nbytes = 0;
 
  printf("redirect_reader: started with fd=%s\n", argv[1]);

  /* Convert the fd to binary */

  fd = atoi(argv[1]);

  /* Re-direct the fd to stdin */

  ret = dup2(fd, 0);
  if (ret != 0)
    {
        fprintf(stderr, "redirect_reader: dup2 failed: %d\n", errno);
        close(fd);
        return 1;
    }

  /* Close the original file descriptor */

  ret = close(fd);
  if (ret != 0)
    {
       fprintf(stderr, "redirect_reader: failed to close fd=%d\n", fd);
       return 2;
    }

  /* Then read from stdin until we hit the end of file */

  for (;;)
    {
      /* Read from stdin */

      ret = read(0, buffer, READ_SIZE);
      if (ret < 0 )
        {
           fprintf(stderr, "redirect_reader: read failed, errno=%d\n", errno);
           return 3;
        }
      else if (ret == 0)
        {
          break;
        }
      nbytes += ret;

      /* Echo to stdout */
 
      ret = write(1, buffer, ret);
      if (ret < 0)
        {
           fprintf(stderr, "redirect_reader: read failed, errno=%d\n", errno);
           return 4;
        }
    }

  printf("redirect_reader: %d bytes read\n", nbytes);
  ret = close(0);
  if (ret != 0)
    {
       fprintf(stderr, "redirect_reader: failed to close fd=%d\n", fd);
       return 5;
    }
  printf("redirect_reader: Returning success\n");
  return 0;
}

/****************************************************************************
 * Name: redirect_writer
 ****************************************************************************/

static int redirect_writer(int argc, char *argv[])
{
  int fd;
  int nbytes = 0;
  int ret;
 
  printf("redirect_writer: started with fd=%s\n", argv[1]);

  /* Convert the fd to binary */

  fd = atoi(argv[1]);

  /* Re-direct the fd to stdout */

  ret = dup2(fd, 1);
  if (ret != 0)
    {
        fprintf(stderr, "redirect_writer: dup2 failed: %d\n", errno);
        return 1;
    }

  /* Close the original file descriptor */

  ret = close(fd);
  if (ret != 0)
    {
       fprintf(stderr, "redirect_reader: failed to close fd=%d\n", fd);
       return 2;
    }

  /* Then write a bunch of stuff to stdout */

  nbytes += printf("\nFour score and seven years ago our fathers brought forth on this continent a new nation,\n");
  nbytes += printf("conceived in Liberty, and dedicated to the proposition that all men are created equal.\n");
  nbytes += printf("\nNow we are engaged in a great civil war, testing whether that nation, or any nation, so\n");
  nbytes += printf("conceived and so dedicated, can long endure. We are met on a great battle-field of that war.\n");
  nbytes += printf("We have come to dedicate a portion of that field, as a final resting place for those who here\n");
  nbytes += printf("gave their lives that that nation might live. It is altogether fitting and proper that we\n");
  nbytes += printf("should do this.\n");
  nbytes += printf("\nBut, in a larger sense, we can not dedicate—we can not consecrate—we can not hallow—this ground.\n");
  nbytes += printf("The brave men, living and dead, who struggled here, have consecrated it, far above our poor power\n");
  nbytes += printf("to add or detract. The world will little note, nor long remember what we say here, but it can\n");
  nbytes += printf("never forget what they did here. It is for us the living, rather, to be dedicated here to the\n");
  nbytes += printf("unfinished work which they who fought here have thus far so nobly advanced. It is rather for us to\n");
  nbytes += printf("be here dedicated to the great task remaining before us—that from these honored dead we take\n");
  nbytes += printf("increased devotion to that cause for which they gave the last full measure of devotion—that we\n");
  nbytes += printf("here highly resolve that these dead shall not have died in vain—that this nation, under God,\n");
  nbytes += printf("shall have a new birth of freedom—and that government of the people, by the people, for the\n");
  nbytes += printf("people, shall not perish from the earth.\n\n");
  printf("redirect_writer: %d bytes read\n", nbytes);

  ret = close(1);
  if (ret != 0)
    {
       fprintf(stderr, "redirect_writer: failed to close fd=%d\n", fd);
       return 3;
    }
  printf("redirect_writer: Returning success\n");
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: redirection_test
 ****************************************************************************/

int redirection_test(void)
{
  const char *argv[2];
  char buffer[8];
  int readerid;
  int writerid;
  int filedes[2];
  int ret;

  /* Create the pipe */

  ret = pipe(filedes);
  if (ret < 0)
    {
      fprintf(stderr, "redirection_test: pipe failed with errno=%d\n", errno);
      return 5;
    }
 
  /* Start redirect_reader thread */

  printf("redirection_test: Starting redirect_reader task with fd=%d\n", filedes[0]);
  sprintf(buffer, "%d", filedes[0]);
  argv[0] = buffer;
  argv[1] = NULL;
  readerid = task_create("redirect_reader", 50, CONFIG_EXAMPLES_PIPE_STACKSIZE, redirect_reader, argv);
  if (readerid < 0)
    {
      fprintf(stderr, "redirection_test: Failed to create redirect_writer task: %d\n", errno);
      return 1;
    }

  /* Start redirect_writer task */

  printf("redirection_test: Starting redirect_writer task with fd=%d\n", filedes[1]);
  sprintf(buffer, "%d", filedes[1]);
  argv[0] = buffer;
  argv[1] = NULL;
  writerid = task_create("redirect_writer", 50, CONFIG_EXAMPLES_PIPE_STACKSIZE, redirect_writer, argv);
  if (writerid < 0)
    {
      fprintf(stderr, "redirection_test: Failed to create redirect_writer task: %d\n", errno);
      ret = task_delete(readerid);
      if (ret != 0)
        {
          fprintf(stderr, "redirection_test: Failed to delete redirect_reader task %d\n", errno);
        }
      return 2;
    }

  /* Wait for redirect_writer thread to complete */

  printf("redirection_test: Waiting...\n");
  sleep(10);

  printf("redirection_test: returning %d\n", ret);
  return ret;
}

