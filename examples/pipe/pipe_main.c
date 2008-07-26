/****************************************************************************
 * pipe_main.c
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

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_FIFO_PATH
# define CONFIG_EXAMPLES_FIFO_PATH "/tmp/testfifo"
#endif

#define MAX_BYTE      13

#define WRITE_SIZE    MAX_BYTE
#define NWRITES       1400
#define NWRITE_BYTES (NWRITES * WRITE_SIZE)

#define READ_SIZE    (2*MAX_BYTE)
#define NREADS       (NWRITES / 2)
#define NREAD_BYTES   NWRITE_BYTES

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
 * Name: reader
 ****************************************************************************/

static void *reader(pthread_addr_t pvarg)
{
  char buffer[READ_SIZE];
  int fd = (int)pvarg;
  int ret;
  int nbytes;
  int value;
  int ndx;

  printf("reader: started\n");
  for (nbytes = 0, value = 0; nbytes < NREAD_BYTES;)
    {
      ret = read(fd, buffer, READ_SIZE);
      if (ret < 0 )
        {
           fprintf(stderr, "reader: read failed, errno=%d\n", errno);
           return (void*)1;
        }
      else if (ret == 0)
        {
          if (nbytes < NREAD_BYTES)
            {
              fprintf(stderr, "reader: Too few bytes read -- aborting: %d\n", nbytes);
              return (void*)2;
            }
          break;
        }
      for (ndx = 0; ndx < ret; ndx++)
        {
          if (value >= WRITE_SIZE)
            {
              value = 0;
            }
          if (buffer[ndx] != value)
            {
              fprintf(stderr, "reader: Byte %d, expected %d, found %d\n",
                      nbytes + ndx, value, buffer[ndx]);
              return (void*)3;
            }
          value++;
        }
      nbytes += ret;
      if (nbytes > NREAD_BYTES)
        {
          fprintf(stderr, "reader: Too many bytes read -- aborting: %d\n", nbytes);
          return (void*)3;
        }
    }
  printf("reader: %d bytes read\n", nbytes);
  return (void*)0;
}

/****************************************************************************
 * Name: writer
 ****************************************************************************/

static void *writer(pthread_addr_t pvarg)
{
  char buffer[WRITE_SIZE];
  int fd = (int)pvarg;
  int ret;
  int i;

  printf("writer: started\n");
  for (i = 0; i < WRITE_SIZE; i++)
    {
      buffer[i] = i;
    }

  for (i = 0; i < NWRITES; i++)
    {
      ret = write(fd, buffer, WRITE_SIZE);
      if (ret < 0 )
        {
           fprintf(stderr, "writer: write failed, errno=%d\n", errno);
           return (void*)1;
        }
      else if (ret != WRITE_SIZE)
        {
           fprintf(stderr, "writer: Unexpected write size=%d\n", ret);
           return (void*)2;
        }
    }
  printf("writer: %d bytes written\n", NWRITE_BYTES);
  return (void*)0;
}

/****************************************************************************
 * Name: perform_test
 ****************************************************************************/

static int perform_test(int fdin, int fdout)
{
  pthread_t readerid;
  pthread_t writerid;
  void *value;
  int tmp;
  int ret;

  /* Start reader thread */

  printf("perform_test: Starting reader thread\n");
  ret = pthread_create(&readerid, NULL, reader, (pthread_addr_t)fdin);
  if (ret != 0)
    {
      fprintf(stderr, "perform_test: Failed to create reader thread, error=%d\n", ret);
      return -1;
    }

  /* Start writer thread */

  printf("perform_test: Starting writer thread\n");
  ret = pthread_create(&writerid, NULL, writer, (pthread_addr_t)fdout);
  if (ret != 0)
    {
      fprintf(stderr, "perform_test: Failed to create writer thread, error=%d\n", ret);
      ret = pthread_cancel(readerid);
      if (ret != 0)
        {
          fprintf(stderr, "perform_test: Failed to cancel reader thread, error=%d\n", ret);
        }
      return -1;
    }

  /* Wait for writer thread to complete */

  printf("perform_test: Waiting for writer thread\n");
  ret = pthread_join(writerid, &value);
  if (ret != 0)
    {
      fprintf(stderr, "perform_test: pthread_join failed, error=%d\n", ret);
    }
  else
    {
      ret = (int)value;
      printf("perform_test: writer returned %d\n", ret);
    }

  /* Wait for reader thread to complete */

  printf("perform_test: Waiting for reader thread\n");
  tmp = pthread_join(readerid, &value);
  if (tmp != 0)
    {
      fprintf(stderr, "perform_test: pthread_join failed, error=%d\n", ret);
    }
  else
    {
      tmp = (int)value;
      printf("perform_test: reader returned %d\n", tmp);
    }

  if (ret == 0)
    {
      ret = tmp;
    }
  printf("perform_test: returning %d\n", ret);
  return ret;
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
  int filedes[2];
  int ret;

  /* Test FIFO logic */

  printf("user_start: Performing FIFO test\n");
  ret = mkfifo(CONFIG_EXAMPLES_FIFO_PATH, 0666);
  if (ret < 0)
    {
      fprintf(stderr, "user_start: mkfifo failed with errno=%d\n", errno);
      return 1;
    }

  /* Open open end of the FIFO for reading and the other end for writing.  NOTE:
   * the following would not work on most FIFO implementations because the attempt
   * to open just one end of the FIFO would block.  The NuttX FIFOs do not block.
   */

  filedes[1] = open(CONFIG_EXAMPLES_FIFO_PATH, O_WRONLY);
  if (filedes[1] < 0)
    {
      fprintf(stderr, "user_start: Failed to open FIFO %s for writing, errno=%d\n",
              CONFIG_EXAMPLES_FIFO_PATH, errno);
      close(filedes[0]);
      return 3;
    }

  filedes[0] = open(CONFIG_EXAMPLES_FIFO_PATH, O_RDONLY);
  if (filedes[0] < 0)
    {
      fprintf(stderr, "user_start: Failed to open FIFO %s for reading, errno=%d\n",
              CONFIG_EXAMPLES_FIFO_PATH, errno);
      return 2;
    }

  /* Then perform the test using those file descriptors */

  ret = perform_test(filedes[0], filedes[1]);
  close(filedes[0]);
  close(filedes[1]);
  unlink(CONFIG_EXAMPLES_FIFO_PATH);
  if (ret != 0)
    {
      fprintf(stderr, "user_start: FIFO test FAILED\n");
      return 4;
    }
  printf("user_start: FIFO test PASSED\n");

  /* Test PIPE logic */

  printf("user_start: Performing pipe test\n");
  ret = pipe(filedes);
  if (ret < 0)
    {
      fprintf(stderr, "user_start: pipe failed with errno=%d\n", errno);
      return 1;
    }

  /* Then perform the test using those file descriptors */

  ret = perform_test(filedes[0], filedes[1]);
  close(filedes[0]);
  close(filedes[1]);
  unlink(CONFIG_EXAMPLES_FIFO_PATH);
  if (ret != 0)
    {
      fprintf(stderr, "user_start: PIPE test FAILED\n");
      return 4;
    }
  printf("user_start: PIPE test PASSED\n");

  fflush(stdout);
  return 0;
}
