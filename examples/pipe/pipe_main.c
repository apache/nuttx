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

#define FIFO_PATH1 "/tmp/testfifo-1"
#define FIFO_PATH2 "/tmp/testfifo-2"

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
          return (void*)4;
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
 * Name: transfer_test
 ****************************************************************************/

static int transfer_test(int fdin, int fdout)
{
  pthread_t readerid;
  pthread_t writerid;
  void *value;
  int tmp;
  int ret;

  /* Start reader thread */

  printf("transfer_test: Starting reader thread\n");
  ret = pthread_create(&readerid, NULL, reader, (pthread_addr_t)fdin);
  if (ret != 0)
    {
      fprintf(stderr, "transfer_test: Failed to create reader thread, error=%d\n", ret);
      return 1;
    }

  /* Start writer thread */

  printf("transfer_test: Starting writer thread\n");
  ret = pthread_create(&writerid, NULL, writer, (pthread_addr_t)fdout);
  if (ret != 0)
    {
      fprintf(stderr, "transfer_test: Failed to create writer thread, error=%d\n", ret);
      pthread_detach(readerid);
      ret = pthread_cancel(readerid);
      if (ret != 0)
        {
          fprintf(stderr, "transfer_test: Failed to cancel reader thread, error=%d\n", ret);
        }
      return 2;
    }

  /* Wait for writer thread to complete */

  printf("transfer_test: Waiting for writer thread\n");
  ret = pthread_join(writerid, &value);
  if (ret != 0)
    {
      fprintf(stderr, "transfer_test: pthread_join failed, error=%d\n", ret);
    }
  else
    {
      ret = (int)value;
      printf("transfer_test: writer returned %d\n", ret);
    }

  /* Wait for reader thread to complete */

  printf("transfer_test: Waiting for reader thread\n");
  tmp = pthread_join(readerid, &value);
  if (tmp != 0)
    {
      fprintf(stderr, "transfer_test: pthread_join failed, error=%d\n", ret);
    }
  else
    {
      tmp = (int)value;
      printf("transfer_test: reader returned %d\n", tmp);
    }

  if (ret == 0)
    {
      ret = tmp;
    }
  printf("transfer_test: returning %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: null_writer
 ****************************************************************************/

static void *null_writer(pthread_addr_t pvarg)
{
  int fd;

  /* Wait a bit */

  printf("null_writer: started -- sleeping\n");
  sleep(5);

  /* Then open the FIFO for write access */

  printf("null_writer: Opening FIFO for write access\n");
  fd = open(FIFO_PATH2, O_WRONLY);
  if (fd < 0)
    {
      fprintf(stderr, "null_writer: Failed to open FIFO %s for writing, errno=%d\n",
              FIFO_PATH2, errno);
      return (void*)1;
    }

  /* Wait a bit more */

  printf("null_writer: Opened %s for writing -- sleeping\n", FIFO_PATH2);
  sleep(5);

  /* Then close the FIFO */

  printf("null_writer: Closing %s\n", FIFO_PATH2);
  close(fd);
  sleep(5);
  
  printf("null_writer: Returning success\n");
  return (void*)0;
}

/****************************************************************************
 * Name: interlock_test
 ****************************************************************************/

static int interlock_test(void)
{
  pthread_t writerid;
  void *value;
  char data[16];
  ssize_t nbytes;
  int fd;
  int ret;

  /* Create a FIFO */

  ret = mkfifo(FIFO_PATH2, 0666);
  if (ret < 0)
    {
      fprintf(stderr, "interlock_test: mkfifo failed with errno=%d\n", errno);
      return 1;
    }

  /* Start the null_writer_thread */

  printf("interlock_test: Starting null_writer thread\n");
  ret = pthread_create(&writerid, NULL, null_writer, (pthread_addr_t)NULL);
  if (ret != 0)
    {
      fprintf(stderr, "interlock_test: Failed to create null_writer thread, error=%d\n", ret);
      ret = 2;
      goto errout_with_fifo;
    }
 
  /* Open one end of the FIFO for reading.  This open call should block until the
   * null_writer thread opens the other end of the FIFO for writing.
   */

  printf("interlock_test: Opening FIFO for read access\n");
  fd = open(FIFO_PATH2, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "interlock_test: Failed to open FIFO %s for reading, errno=%d\n",
              FIFO_PATH2, errno);
      ret = 3;
      goto errout_with_thread;
    }

  /* Attempt to read one byte from the FIFO.  This should return end-of-file because
   * the null_writer closes the FIFO without writing anything.
   */

  printf("interlock_test: Reading from %s\n", FIFO_PATH2);
  nbytes = read(fd, data, 16);
  if (nbytes < 0 )
    {
      fprintf(stderr, "interlock_test: read failed, errno=%d\n", errno);
      ret = 4;
      goto errout_with_file;
    }
  else if (ret != 0)
    {
      fprintf(stderr, "interlock_test: Read %d bytes of data -- aborting: %d\n", nbytes);
      ret = 5;
      goto errout_with_file;
    }

  /* Close the file */

  printf("interlock_test: Closing %s\n", FIFO_PATH2);
  close(fd);

  /* Wait for null_writer thread to complete */

  printf("interlock_test: Waiting for null_writer thread\n");
  ret = pthread_join(writerid, &value);
  if (ret != 0)
    {
      fprintf(stderr, "interlock_test: pthread_join failed, error=%d\n", ret);
      ret = 6;
      goto errout_with_fifo;
    }
  else
    {
      printf("interlock_test: writer returned %d\n", (int)value);
      if (value != (void*)0)
        {
          ret = 7;
          goto errout_with_fifo;
        }
    }

  /* unlink(FIFO_PATH2); */
  printf("interlock_test: Returning success\n");
  return 0;

errout_with_file:
  close(fd);
errout_with_thread:
  pthread_detach(writerid);
  pthread_cancel(writerid);
errout_with_fifo:
  /* unlink(FIFO_PATH2); */
  printf("interlock_test: Returning %d\n", ret);
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
  ret = mkfifo(FIFO_PATH1, 0666);
  if (ret < 0)
    {
      fprintf(stderr, "user_start: mkfifo failed with errno=%d\n", errno);
      return 1;
    }

  /* Open one end of the FIFO for reading and the other end for writing.  NOTE:
   * the following might not work on most FIFO implementations because the attempt
   * to open just one end of the FIFO for writing might block.  The NuttX FIFOs block
   * only on open for read-only (see interlock_test()).
   */

  filedes[1] = open(FIFO_PATH1, O_WRONLY);
  if (filedes[1] < 0)
    {
      fprintf(stderr, "user_start: Failed to open FIFO %s for writing, errno=%d\n",
              FIFO_PATH1, errno);
      return 2;
    }

  filedes[0] = open(FIFO_PATH1, O_RDONLY);
  if (filedes[0] < 0)
    {
      fprintf(stderr, "user_start: Failed to open FIFO %s for reading, errno=%d\n",
              FIFO_PATH1, errno);
      close(filedes[1]);
      return 3;
    }

  /* Then perform the test using those file descriptors */

  ret = transfer_test(filedes[0], filedes[1]);
  close(filedes[0]);
  close(filedes[1]);
  /* unlink(FIFO_PATH1); fails */
  if (ret != 0)
    {
      fprintf(stderr, "user_start: FIFO test FAILED (%d)\n", ret);
      return 4;
    }
  printf("user_start: FIFO test PASSED\n");

  /* Test PIPE logic */

  printf("user_start: Performing pipe test\n");
  ret = pipe(filedes);
  if (ret < 0)
    {
      fprintf(stderr, "user_start: pipe failed with errno=%d\n", errno);
      return 5;
    }

  /* Then perform the test using those file descriptors */

  ret = transfer_test(filedes[0], filedes[1]);
  close(filedes[0]);
  close(filedes[1]);
  /* unlink(FIFO_PATH1); fails */
  if (ret != 0)
    {
      fprintf(stderr, "user_start: PIPE test FAILED (%d)\n", ret);
      return 6;
    }
  printf("user_start: PIPE test PASSED\n");

  /* Then perform the FIFO interlock test */
  ret = interlock_test();
  if (ret != 0)
    {
      fprintf(stderr, "user_start: FIFO interlock test FAILED (%d)\n", ret);
      return 7;
    }
  printf("user_start: PIPE interlock test PASSED\n");

  fflush(stdout);
  return 0;
}
