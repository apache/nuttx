/****************************************************************************
 * arch/sim/src/sim/win/sim_hostuart.c
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

#include <stdbool.h>
#include <windows.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static HANDLE g_stdin_handle;
static HANDLE g_stdout_handle;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_uart_start
 ****************************************************************************/

void host_uart_start(void)
{
  DWORD mode;

  g_stdin_handle = GetStdHandle(STD_INPUT_HANDLE);
  if (GetConsoleMode(g_stdin_handle, &mode))
    {
      SetConsoleMode(g_stdin_handle, mode & ~(ENABLE_MOUSE_INPUT |
                                              ENABLE_WINDOW_INPUT |
                                              ENABLE_ECHO_INPUT |
                                              ENABLE_LINE_INPUT));
      FlushConsoleInputBuffer(g_stdin_handle);
    }

  g_stdout_handle = GetStdHandle(STD_OUTPUT_HANDLE);
}

/****************************************************************************
 * Name: host_uart_open
 ****************************************************************************/

int host_uart_open(const char *pathname)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_uart_close
 ****************************************************************************/

void host_uart_close(int fd)
{
}

/****************************************************************************
 * Name: host_uart_putc
 ****************************************************************************/

int host_uart_putc(int fd, int ch)
{
  DWORD nwritten;

  if (WriteConsole(g_stdout_handle, &ch, 1, &nwritten, NULL))
    {
      return ch;
    }

  return -EIO;
}

/****************************************************************************
 * Name: host_uart_getc
 ****************************************************************************/

int host_uart_getc(int fd)
{
  unsigned char ch;
  DWORD nread;

  if (ReadConsole(g_stdin_handle, &ch, 1, &nread, 0))
    {
      return ch;
    }

  return -EIO;
}

/****************************************************************************
 * Name: host_uart_getcflag
 ****************************************************************************/

int host_uart_getcflag(int fd, unsigned int *cflag)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_uart_setcflag
 ****************************************************************************/

int host_uart_setcflag(int fd, unsigned int cflag)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_uart_checkc
 ****************************************************************************/

bool host_uart_checkc(int fd)
{
  DWORD size;

  if (GetNumberOfConsoleInputEvents(g_stdin_handle, &size) && size > 1)
    {
      return true;
    }

  return false;
}
