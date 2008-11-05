/****************************************************************************
 * config/us7032evb1/shterm/shterm.c
 *
 *   Copyright(C) 2008 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
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
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Size of the circular buffer used for interrupt I/O */

#define MAX_FILEPATH 255

#define ENQ          5
#define ACK          6

#define DEFAULT_BAUD 9600

#define dbg(format, arg...) if (debug > 0) printf(format, ##arg)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sendfile(int fdtarg, char *filename, int verify);
static void receivefile(int fdtarg, char *filename);
static void getfilename(int fd, char *name);
static int readbyte(int fd, char *ch);
static void writebyte(int fd, char byte);
static void close_tty(void);
static void interrupt(int signo);
static void show_usage(const char *progname, int exitcode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int debug = 0;
static int g_fd = -1;
static int g_fdnb = -1;
static const char g_dfttydev[] = "/dev/ttyS0";
static const char *g_ttydev = g_dfttydev;
static int g_baud = DEFAULT_BAUD;
static struct termios g_termios;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendfile
 ****************************************************************************/

static void sendfile(int fdtarg, char *filename, int verify)
{
  char chin;
  char chout;
  int  fdin;
  int  nbytes;
  int  ndots;
  int  ret;

  fdin = open(filename, O_RDONLY);
  if (fdin < 0)
    {
      fprintf(stderr, "ERROR: Failed to open '%s' for reading\n", filename);
      (void)writebyte(fdin, '>');
      return;
    }

  if (verify)
    {
      printf("Verifying file '%s':\n", filename);
    }
  else
    {
      printf("Loading file '%s':\n", filename);
    }
  fflush(stdout);

  while ((ret = readbyte(fdin, &chout)) == 1)
    {
      if (++nbytes > 256)
        {
          nbytes = 0;
          putchar('.');
          if (++ndots > 72)
            {
               putchar('\n');
               fflush(stdout);
               ndots = 0;
            }
        }

      writebyte(fdtarg, chout);

      do
        {
          ret = readbyte(fdtarg, &chin);
          if (ret == 1 && chin == '>')
            {
              close(fdin);
              writebyte(fdtarg, ACK);
              return;
            }
        }
      while (ret == 1 && chin != chout);
    }

  writebyte(fdtarg, '>');
  do
   {
     ret = readbyte(fdtarg, &chin);
   }
  while (ret == 1 && chin != ENQ);
  close(fdin);
  writebyte(fdtarg, ACK);
}

/****************************************************************************
 * Name: receivefile
 ****************************************************************************/

static void receivefile(int fdtarg, char *filename)
{
  char ch;
  int  fdout;
  int  nbytes;
  int  ndots;
  int  ret;

  fdout = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fdout < 0)
    {
      fprintf(stderr, "ERROR: Failed to open '%s' for writing\n", filename);
      (void)writebyte(fdtarg, '>');
      return;
    }

  printf("Receiving file '%s':\n", filename);
  fflush(stdout);
  (void)writebyte(fdtarg, '+');

  /* Synchronize */

  do
    {
      ret = readbyte(fdtarg, &ch);
    }
  while (ret == 1 && ch != 'S' && ch != 'Q');

  nbytes = 0;
  ndots = 0;

  /* Receive the file */

  while (ret == 1)
    {
      /* Check for end-of-file */

      if (ch == '>')
        {
          close(fdout);
          return;
        }

      writebyte(fdout, ch);

      if (++nbytes > 256)
        {
          nbytes = 0;
          putchar('.');
          if (++ndots > 72)
            {
               putchar('\n');
               ndots = 0;
            }
            fflush(stdout);
        }

      ret = readbyte(fdtarg, &ch);
      if (ch == '\r')
        {
          writebyte(fdtarg, '+');
        }
    }

  close (fdout);
}

/****************************************************************************
 * Name: getfilename
 ****************************************************************************/

static void getfilename(int fd, char *name)
{
  char ch;
  int ret;

  /* Skip over spaces */

  do
    {
      ret = readbyte(fd, &ch);
    }
  while(ch == ' ' && ret == 1);

  /* Concatenate the filename */

  while(ret == 1 && ch > ' ')
    {
      *name++ = ch;
       ret = readbyte(fd, &ch);
    }
  *name++ = 0;
}

/****************************************************************************
 * Name: readbyte
 ****************************************************************************/

static int readbyte(int fd, char *ch)
{
  int ret;

  /* Read characters from the console, and echo them to the target tty */

  ret = read(fd, ch, 1);
  if(ret < 0)
    {
      if(errno != EAGAIN)
        {
          printf("ERROR: Failed to read from fd=%d: %s\n", fd, strerror(errno));
          close_tty();
          exit(12);
        }
      return -EAGAIN;
    }
  else if(ret > 1)
    {
      printf("ERROR: Unexpected number of bytes read(%d) from fd=%d\n", ret, fd);
      close_tty();
      exit(13);
    }
  return ret;
}

/****************************************************************************
 * Name: writebyte
 ****************************************************************************/

static void writebyte(int fd, char byte)
{
  int ret = write(fd, &byte, 1);
  if(ret < 0)
    {
      printf("ERROR: Failed to write to fd=%d: %s\n", fd, strerror(errno));
      close_tty();
      exit(14);
    }
}

/****************************************************************************
 * Name: close_tty
 ****************************************************************************/

static void close_tty(void)
{
  int ret;

  if (g_fdnb >= 0)
    {
      (void)close(g_fdnb);
    }

  if (g_fd >= 0)
    {
      ret = tcsetattr(g_fd, TCSANOW, &g_termios);
      if (ret < 0)
        {
          printf("ERROR: Failed to restore termios for %s: %s\n", g_ttydev, strerror(errno));
        }
      (void)close(g_fd);
    }
}

/****************************************************************************
 * Name: interrupt
 ****************************************************************************/

static void interrupt(int signo)
{
  printf("Exit-ing...\n");
  close_tty();
  exit(0);
}

/****************************************************************************
 * Name: interrupt
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s [-h] [-t <ttyname>] [-b <baud>]\n", progname);
  fprintf(stderr, "\nWhere:\n");
  fprintf(stderr, "\t-h: Prints this message then exit.\n");
  fprintf(stderr, "\t-t <ttyname>:  Use <ttyname> device instead of %s.\n", g_dfttydev);
  fprintf(stderr, "\t-b <baud>: Use <baud> instead of %d.\n", DEFAULT_BAUD);
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct termios tty;
  char filename[MAX_FILEPATH];
  char ch;
  int  speed;
  int  opt;
  int  oflags;
  int  ret;

  while((opt = getopt(argc, argv, ":t:b:h")) != -1)
    {
      switch(opt)
        {
        case 'd':
          debug++;
          break;

        case 't':
          g_ttydev = optarg;
          break;

        case 'b':
          g_baud = atoi(optarg);
          break;

        case ':':
          fprintf(stderr, "ERROR: Missing argument to option '%c'\n", optopt);
          show_usage(argv[0], 1);
          break;

        case '?':
          fprintf(stderr, "ERROR: Unrecognized option '%c'\n", optopt);
          show_usage(argv[0], 2);
          break;
        }
    }

  if(optind < argc)
    {
      fprintf(stderr, "ERROR: Unexpected arguments at end of line\n");
      show_usage(argv[0], 3);
    }

  switch(g_baud)
    {
    case 0:      speed = B0;      break;
    case 50:     speed = B50;     break;
    case 75:     speed = B75;     break;
    case 110:    speed = B110;    break;
    case 134:    speed = B134;    break;
    case 150:    speed = B150;    break;
    case 200:    speed = B200;    break;
    case 300:    speed = B300;    break;
    case 600:    speed = B600;    break;
    case 1200:   speed = B1200;   break;
    case 1800:   speed = B1800;   break;
    case 2400:   speed = B2400;   break;
    case 4800:   speed = B4800;   break;
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;

    default:
      fprintf(stderr, "ERROR: Unsupported BAUD=%d\n", g_baud);
      show_usage(argv[0], 4);
    }

  /* Set the host stdin to O_NONBLOCK */

  oflags = fcntl(0, F_GETFL, 0);
  if(oflags == -1)
    {
      fprintf(stderr, "ERROR: fnctl(F_GETFL) failed: %s\n", strerror(errno));
      return 5;
    }

  ret = fcntl(0, F_SETFL, oflags | O_NONBLOCK);
  if(ret < 0)
    {
      fprintf(stderr, "ERROR: fnctl(F_SETFL) failed: %s\n", strerror(errno));
      return 6;
    }

  /* Open the selected serial port (blocking)*/

  g_fd = open(g_ttydev, O_RDWR);
  if(g_fd < 0)
    {
      printf("ERROR: Failed to open %s: %s\n", g_ttydev, strerror(errno));
      return 7;
    }

  /* Configure the serial port in at the selected baud in 8-bit, no-parity, raw mode
   * and turn off echo, etc.
   */

  ret = tcgetattr(g_fd, &g_termios);
  if(ret < 0)
    {
      printf("ERROR: Failed to get termios for %s: %s\n", g_ttydev, strerror(errno));
      close(g_fd);
      return 8;
    }

  memcpy(&tty, &g_termios, sizeof(struct termios));
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  tty.c_cflag &= ~(CSIZE|PARENB);
  tty.c_cflag |= CS8;

 (void)cfsetispeed(&tty, speed);
 (void)cfsetospeed(&tty, speed);

  ret = tcsetattr(g_fd, TCSANOW, &tty);
  if(ret < 0)
    {
      printf("ERROR: Failed to set termios for %s: %s\n", g_ttydev, strerror(errno));
      close(g_fd);
      return 9;
    }

#if 1
  /* Open the selected serial port (non-blocking)*/

  g_fdnb = open(g_ttydev, O_RDONLY | O_NONBLOCK);
  if(g_fdnb < 0)
    {
      printf("ERROR: Failed to open %s: %s\n", g_ttydev, strerror(errno));
      return 9;
    }
#else
  /* Create a non-blocking copy of the configure tty descriptor */

  g_fdnb = dup(g_fd);
  if (g_fdnb < 0)
    {
      printf("ERROR: Failed to dup %s fd=%d: %s\n", g_ttydev, g_fd, strerror(errno));
      close_tty();
      return 9;
    }

  oflags = fcntl(g_fdnb, F_GETFL, 0);
  if(oflags == -1)
    {
      fprintf(stderr, "ERROR: fnctl(F_GETFL) failed: %s\n", strerror(errno));
      close_tty();
      return 10;
    }

  ret = fcntl(g_fdnb, F_SETFL, oflags | O_NONBLOCK);
  if(ret < 0)
    {
      fprintf(stderr, "ERROR: fnctl(F_SETFL) failed: %s\n", strerror(errno));
      close_tty();
      return 11;
    }
#endif

  /* Catch attempts to control-C out of the program so that we can restore
   * the TTY settings.
   */

  signal(SIGINT, interrupt);

  /* Loopo until control-C */

  for(;;)
    {
      /* Read characters from the console, and echo them to the target tty */

      ret = readbyte(0, &ch);
      if (ret == 0)
        {
          printf("End-of-file: exitting\n");
          close_tty();
          return 0;
        }
      else if (ret == 1)
        {
          writebyte(g_fd, ch);
        }

      /* Read characters from target TTY and echo them on the console */

      ret = readbyte(g_fdnb, &ch);
      if (ret == 0)
        {
          printf("ERROR: Unexpected number of bytes read(%d) from %s\n", ret, g_ttydev);
          close_tty();
          return 15;
        }
      else if (ret == 1)
        {
          if (ch == ENQ)
            {
              char ch1;
              char ch2;

              writebyte(g_fd, '*');
              ret = readbyte(g_fd, &ch1);
              if (ret != 1)
                {
                  printf("ERROR: Unexpected number of bytes read(%d) from %s\n", ret, g_ttydev);
                  close_tty();
                  return 16;
                }
              ret = readbyte(g_fd, &ch2);
              if (ret != 1)
                {
                  printf("ERROR: Unexpected number of bytes read(%d) from %s\n", ret, g_ttydev);
                  close_tty();
                  return 17;
                }

              getfilename(g_fd, filename);

              if (ch1 == 'l' || ch1 == 'L')
                {
                  sendfile(g_fd, filename, 0);
                }
              else if (ch1 == 'v' || ch1 == 'v')
                {
                  sendfile(g_fd, filename, 1);
                }
              else if (ch1 == 's' || ch1 == 'S')
                {
                  receivefile(g_fd, filename);
                }
            }
          else
            {
              writebyte(1, ch);
            }
        }
    }
  return 0;
}
