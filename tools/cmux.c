/****************************************************************************
 * tools/cmux.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>

#include <linux/gsmmux.h>
#include <linux/tty.h>

/* Maximum transfert unit (MTU), value in bytes */

#define MTU 127
#define MRU 127

/* Name of the virtual TTYs to create */

#define DEVICE_NAME "/dev/ttyGSM"

/* Name of the driver, used to get the major number */

#define DRIVER_NAME "gsmtty"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_major
 *
 * Description:
 *   Gets the major number of the driver device.
 *   Returns the major number on success, -1 on failure.
 *
 ****************************************************************************/

static int get_major(const char *driver)
{
  char device[NAME_MAX];
  char *line = NULL;
  int major = -1;
  size_t len = 0;
  ssize_t read;
  FILE *fp;

  /* Open /proc/devices file */

  if ((fp = fopen("/proc/devices", "r")) == NULL)
    {
      perror("Cannot open /proc/devices\n");
      return -1;
    }

  while ((major == -1) && (read = getline(&line, &len, fp)) != -1)
    {
      if (strstr(line, driver) != NULL)
        {
          if (sscanf(line, "%d %s\n", &major, device) != 2)
            {
              major = -1;
            }
        }

      /* Free the line before getting a new one */

      free(line);
      line = NULL;
    }

  fclose(fp);
  return major;
}

/****************************************************************************
 * Name: make_nodes
 *
 * Description:
 *   Creates the virtual TTY nodes.
 *   Returns the number of nodes created.
 *
 ****************************************************************************/

int make_nodes(int major, const char *basename, int number)
{
  char devname[PATH_MAX];
  int created = 0;
  mode_t oldmask;
  dev_t device;
  int minor;

  /* Set a new mask to get 666 mode and stores the old one */

  oldmask = umask(0);

  for (minor = 1; minor <= number; minor++)
    {
      /* Append the minor number to the base name */

      sprintf(devname, "%s%d", basename, minor);

      /* Store a device info with major and minor */

      device = makedev(major, minor);

      /* Create the actual character node */

      if (mknod(devname, S_IFCHR | 0666, device) != 0)
        {
          fprintf(stderr, "Cannot create %s errer:%s. \n",
                  devname, strerror(errno));
        }
      else
        {
          created++;
          printf("Created %s\n", devname);
        }
    }

  /* Revert the mask to the old one */

  umask(oldmask);
  return created;
}

/****************************************************************************
 * Name: remove_nodes
 *
 * Description:
 *   Removes previously created TTY nodes.
 *   Returns nothing, it doesn't really matter if it fails.
 *
 ****************************************************************************/

void remove_nodes(const char *basename, int number)
{
  char devname[PATH_MAX];
  int node;

  printf("Remove nodes\n");
  for (node = 1; node <= number; node++)
    {
      sprintf(devname, "%s%d", basename, node);

      /* Unlink the actual character node */

      if (unlink(devname) == -1)
        {
          fprintf(stderr, "Cannot remove %s, error: %s\n",
                  devname, strerror(errno));
        }
    }
}

/****************************************************************************
 * Name: setrawmode
 *
 * Description:
 *   Sets the serial port in raw mode.
 *
 ****************************************************************************/

int setrawmode(int fd, int baudrate)
{
  struct termios term;
  int ret;

  ret = tcgetattr(fd, &term);
  if (ret < 0)
    {
      perror("Error from tcgetattr\n");
      return ret;
    }

  cfmakeraw(&term);
  if (baudrate != 0)
    {
      speed_t speed;
      switch (baudrate)
        {
        case 9600:
          speed = B9600;
          break;
        case 19200:
          speed = B19200;
          break;
        case 38400:
          speed = B38400;
          break;
        case 57600:
          speed = B57600;
          break;
        case 115200:
          speed = B115200;
          break;
        case 230400:
          speed = B230400;
          break;
        case 460800:
          speed = B460800;
          break;
        case 500000:
          speed = B500000;
          break;
        case 576000:
          speed = B576000;
          break;
        case 921600:
          speed = B921600;
          break;
        case 1000000:
          speed = B1000000;
          break;
        case 1152000:
          speed = B1152000;
          break;
        case 1500000:
          speed = B1500000;
          break;
        case 2000000:
          speed = B2000000;
        case 2500000:
          speed = B2500000;
          break;
        default:
          fprintf(stderr, "Unsupported baudrate: %d\n", baudrate);
          return -1;
        }

      cfsetospeed(&term, speed);
      cfsetispeed(&term, speed);
    }

  ret = tcsetattr(fd, TCSANOW, &term);
  if (ret != 0)
    {
      perror("Error from tcsetattr\n");
      return ret;
    }

  return 0;
}

static void signal_handler(int signum)
{
  printf("Caught signal %d\n", signum);
}

static void usage(const char *progname)
{
  fprintf(stderr, "Usage: %s -d <device> -n <number> -b <baudrate>\n",
          progname);
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  const char *devname = NULL;
  char ttyname[PATH_MAX];
  struct gsm_config gsm;
  int ldisc = N_GSM0710;
  int baudrate = 0;
  int number = 1;
  int created;
  int major;
  int ret;
  int fd;

  if (argc < 2)
    {
      usage(argv[0]);
    }

  while ((ret = getopt(argc, argv, "d:n:b:")) != -1)
    {
      switch (ret)
        {
        case 'd':
          devname = optarg;
          break;
        case 'n':
          number = atoi(optarg);
          break;
        case 'b':
          baudrate = atoi(optarg);
          break;
        default:
          usage(argv[0]);
        }
    }

  if (devname == NULL)
    {
      perror("No device specified\n");
      usage(argv[0]);
    }

  fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0)
    {
      fprintf(stderr, "Error opening %s: %s\n", devname, strerror(errno));
      return EXIT_FAILURE;
    }

  printf("serial: %s, fd: %d\n", devname, fd);
  if (setrawmode(fd, baudrate) < 0)
    {
      goto errout;
    }

  if (ioctl(fd, TIOCSETD, &ldisc) < 0)
    {
      perror("Cannot set line dicipline\n");
      goto errout;
    }

  /* Get n_gsm configuration */

  if (ioctl(fd, GSMIOC_GETCONF, &gsm) < 0)
    {
      perror("Cannot get GSM multiplex parameters\n");
      goto errout;
    }

  /* We are initiator and need encoding 0 (basic) */

  gsm.encapsulation = 0;
  gsm.initiator = 1;

  /* Set the MTU and MRU */

  gsm.mru = MRU;
  gsm.mtu = MTU;

  if (ioctl(fd, GSMIOC_SETCONF, &gsm) < 0)
    {
      perror("Cannot set GSM multiplex parameters\n");
      goto errout;
    }

  /* Wait for the line discipline to be set */

  sleep(2);
  printf("Line dicipline set\n");

  /* Create the virtual TTYs */

  major = get_major(DRIVER_NAME);
  if (major < 0)
    {
      perror("Cannot get major number\n");
      goto errout;
    }

  created = make_nodes(major, DEVICE_NAME, number);
  if (created == 0)
    {
      perror("No nodes have been created\n");
      goto errout;
    }
  else if (created < number)
    {
      fprintf(stderr, "Cannot create all nodes, only %d/%d "
              "have been created.\n", created, number);
    }

  /* Check if the device is accessible */

  ret = ioctl(fd, GSMIOC_GETFIRST, &major);
  snprintf(ttyname, sizeof(ttyname), "%s%d", DEVICE_NAME, major);
  ret = open(ttyname, O_RDWR | O_NOCTTY | O_NDELAY);
  if (ret < 0)
    {
      fprintf(stderr, "Error opening %s: %s\n", ttyname, strerror(errno));
      remove_nodes(DEVICE_NAME, number);
      goto errout;
    }

  close(ret);

  /* Detach from the terminal if needed */

  printf("Going to background\n");
  if (daemon(0, 0) != 0)
    {
      fprintf(stderr, "Cannot daemonize\n");
    }

  /* Wait to keep the line discipline enabled, wake it up with a signal */

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  pause();

  remove_nodes(DEVICE_NAME, number);

errout:
  close(fd);
  return EXIT_FAILURE;
}
