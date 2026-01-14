/****************************************************************************
 * libs/libc/unistd/lib_getpass.c
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

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_password[128];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *getpass(FAR const char *prompt)
{
  struct termios s;
  struct termios t;
  ssize_t total_bytes_read = 0;
  ssize_t bytes_read;
  int fd;

  if ((fd = open("/dev/console", O_RDONLY | O_NOCTTY)) < 0)
    {
      fd = STDIN_FILENO;
    }

  tcgetattr(fd, &t);
  s = t;
  t.c_lflag &= ~(ECHO | ISIG);
  t.c_lflag |= ICANON;
  t.c_iflag &= ~(INLCR | IGNCR);
  t.c_iflag |= ICRNL;
  tcsetattr(fd, TCSAFLUSH, &t);
  tcdrain(fd);

  if (write(STDERR_FILENO, prompt, strlen(prompt)) != strlen(prompt))
    {
      return 0;
    }

  while ((bytes_read = read(fd, g_password + total_bytes_read,
                            sizeof(g_password) - total_bytes_read)) > 0)
    {
      if (bytes_read > 0 && g_password[total_bytes_read] == '\n')
        {
            break;
        }

      total_bytes_read += bytes_read;
    }

  if (total_bytes_read >= 0)
    {
      if (total_bytes_read > 0 && g_password[total_bytes_read - 1] == '\n')
        {
          total_bytes_read--;
        }

      g_password[total_bytes_read] = 0;
    }

  tcsetattr(fd, TCSAFLUSH, &s);

  if (fd > STDERR_FILENO)
    {
      close(fd);
    }

  return g_password;
}
