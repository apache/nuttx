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
#include <sys/param.h>
#include <termios.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CBAUD          0010017  /* Baud speed mask (not in POSIX) */
#define BOTHER         0010000  /* Magic token for custom baud rate */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct speed_s
{
  speed_t value;
  speed_t mask;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Routine which returns the baud rate of the tty
 *
 * Note that the baud_table needs to be kept in sync with the
 * include/termios.h file.
 */

static const struct speed_s g_baud_table[] =
{
  { 0,       B0 },
  { 50,      B50 },
  { 75,      B75 },
  { 110,     B110 },
  { 134,     B134 },
  { 150,     B150 },
  { 200,     B200 },
  { 300,     B300 },
  { 600,     B600 },
  { 1200,    B1200 },
  { 1800,    B1800 },
  { 2400,    B2400 },
  { 4800,    B4800 },
  { 9600,    B9600 },
  { 19200,   B19200 },
  { 38400,   B38400 },
  { 57600,   B57600 },
  { 115200,  B115200 },
  { 230400,  B230400 },
  { 460800,  B460800 },
  { 500000,  B500000 },
  { 576000,  B576000 },
  { 921600,  B921600 },
  { 1000000, B1000000 },
  { 1152000, B1152000 },
  { 1500000, B1500000 },
  { 2000000, B2000000 },
  { 2500000, B2500000 },
  { 3000000, B3000000 },
  { 3500000, B3500000 },
  { 4000000, B4000000 }
};

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
 *   NOTE 2. In NuttX, the speed_t is defined to be unsigned int and the baud
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
  size_t idx;

  DEBUGASSERT(termiosp);
  for (idx = 0; idx < nitems(g_baud_table); idx++)
    {
      if (speed == g_baud_table[idx].mask)
        {
          termiosp->c_speed = g_baud_table[idx].value;
          break;
        }
      else if (speed == g_baud_table[idx].value)
        {
          termiosp->c_speed = speed;
          speed = g_baud_table[idx].mask;
          break;
        }
    }

  if (idx == nitems(g_baud_table))
    {
      termiosp->c_speed = speed;
      speed = BOTHER;
    }

  termiosp->c_cflag &= ~CBAUD;
  termiosp->c_cflag |= speed;

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
 *   NOTE 2.  In NuttX, the speed_t is defined to be uint32_t and the baud
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
  DEBUGASSERT(termiosp);
  return termiosp->c_speed;
}
