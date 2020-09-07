/****************************************************************************
 * libs/libc/termios/lib_cfspeed.c
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

#include <sys/types.h>
#include <termios.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CBAUD          0010017  /* Baud speed mask (not in POSIX) */
#define CBAUDEX        0010000  /* Extra baud speed mask, included in CBAUD.
                                 * (not in POSIX) */

#define ARRAYSIZE(a)   (sizeof((a))/sizeof(a[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Routine which returns the baud rate of the tty
 *
 * Note that the baud_table needs to be kept in sync with the
 * include/termios.h file.
 */

static const speed_t g_baud_table[] =
{
  0,       50,      75,      110,     134,
  150,     200,     300,     600,     1200,
  1800,    2400,    4800,    9600,    19200,
  38400,   57600,   115200,  230400,  460800,
  500000,  576000,  921600,  1000000, 1152000,
  1500000, 2000000, 2500000, 3000000, 3500000,
  4000000
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int baud_mask(speed_t speed)
{
  speed_t idx = 0;

  for (; idx < ARRAYSIZE(g_baud_table); idx++)
    {
      if (speed == g_baud_table[idx])
        {
          break;
        }
    }

  /* we don't find the speed value, it could be mask */

  if (idx == ARRAYSIZE(g_baud_table))
    {
      return (speed & ~CBAUD) ? -1 : speed;
    }

  /* If idx > B38400, we should will idx minus 15, and or CBAUDEX */

  if (idx > B38400)
    {
      idx -= B38400;
      idx |= CBAUDEX;
    }

  return idx;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cfsetspeed
 *
 * Description:
 *   The cfsetspeed() function is a non-POSIX function that sets the baud
 *   stored in the structure pointed to by termiosp to speed.
 *
 *   There is no effect on the baud set in the hardware until a subsequent
 *   successful call to tcsetattr() on the same termios structure.
 *
 *   NOTE 1: NuttX does not control input/output baud independently.  Both
 *   must be the same.  The POSIX standard interfaces, cfisetispeed() and
 *   cfisetospeed() are defined to be cfsetspeed() in termios.h.
 *
 *   NOTE 3: A consequence of NOTE 1 is that you should never attempt to
 *   set the input and output baud to different values.
 *
 *   Also, the following POSIX requirement cannot be supported: "If the input
 *   baud rate stored in the termios structure pointed to by termios_p is 0,
 *   the input baud rate given to the hardware will be the same as the output
 *   baud rate stored in the termios structure."
 *
 *   NOTE 2. In Nuttx, the speed_t is defined to be unsigned int and the baud
 *   encodings of termios.h are baud value mask. And their corresponding
 *   values are in array g_baud_table. However, if you do so, your code will
 *   *NOT* be portable to other environments where speed_t is smaller and
 *   where the termios.h baud values are encoded! To avoid portability
 *   issues, use the baud definitions in termios.h!
 *
 *   Linux, for example, would require this (also non-portable) sequence:
 *
 *     cfsetispeed(termiosp, BOTHER);
 *     termiosp->c_ispeed = baud;
 *
 *     cfsetospeed(termiosp, BOTHER);
 *     termiosp->c_ospeed = baud;
 *
 * Input Parameters:
 *   termiosp - The termiosp argument is a pointer to a termios structure.
 *   speed - The new input speed. It could be baud rate or could be mask.
 *
 * Returned Value:
 *   Baud is returned. If speed don't match g_baud_table and mask in
 *   termios.h, -1 is returned and set errno EINVAL.
 *
 ****************************************************************************/

int cfsetspeed(FAR struct termios *termiosp, speed_t speed)
{
  int mask = baud_mask(speed);

  DEBUGASSERT(termiosp);
  if (mask == -1)
    {
      set_errno(EINVAL);
      return mask;
    }

  termiosp->c_cflag &= ~CBAUD;
  termiosp->c_cflag |= mask;
  return 0;
}

/****************************************************************************
 * Name: cfgetspeed
 *
 * Description:
 *   The cfgetspeed() function is a non-POSIX function will extract the baud
 *   from the termios structure to which the termiosp argument points.
 *
 *   This function will return exactly the value in the termios data
 *   structure, without interpretation.
 *
 *   NOTE 1: NuttX does not control input/output baud independently.  Both
 *   must be the same.  The POSIX standard interfaces, cfisetispeed() and
 *   cfisetospeed() are defined to be cfgetspeed() in termios.h.
 *   NOTE 2.  In Nuttx, the speed_t is defined to be uint32_t and the baud
 *   encodings of termios.h are the actual baud values themselves. Therefore,
 *   any baud value may be returned here... not just those enumerated in
 *   termios.h
 *
 * Input Parameters:
 *   termiosp - The termiosp argument is a pointer to a termios structure.
 *
 * Returned Value:
 *   Encoded baud value from the termios structure.
 *
 ****************************************************************************/

speed_t cfgetspeed(FAR const struct termios *termiosp)
{
  int idx;

  DEBUGASSERT(termiosp);
  idx = termiosp->c_cflag & CBAUD & ~CBAUDEX;
  idx += (termiosp->c_cflag & CBAUDEX) ? 15 : 0;
  return g_baud_table[idx];
}
