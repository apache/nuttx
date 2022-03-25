/****************************************************************************
 * libs/libc/misc/lib_getrandom.c
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

#include <sys/random.h>
#include <fcntl.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getrandom
 *
 * Description:
 *   Fill a buffer of arbitrary length with randomness. This is the
 *   preferred interface for getting random numbers. The traditional
 *   /dev/random approach is susceptible for things like the attacker
 *   exhausting file descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Input Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested.
 *   flags  - Bit mask that can contain zero or more of the ORed values
 *            together.
 *
 * Returned Value:
 *   On success, getrandom() returns the number of bytes that were copied
 *   to the buffer bytes.  This may be less than the number of bytes
 *   requested via nbytes if either GRND_RANDOM was specified in flags and
 *   insufficient entropy was present in the random source or the system
 *   call was interrupted by a signal.
 *
 *   On error, -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

ssize_t getrandom(FAR void *bytes, size_t nbytes, unsigned int flags)
{
  int oflags = O_RDONLY;
  FAR const char *dev;
  int fd;

  if ((flags & GRND_NONBLOCK) != 0)
    {
      oflags |= O_NONBLOCK;
    }

  if ((flags & GRND_RANDOM) != 0)
    {
      dev = "/dev/random";
    }
  else
    {
      dev = "/dev/urandom";
    }

  fd = open(dev, oflags);
  if (fd < 0)
    {
      return fd;
    }

  nbytes = read(fd, bytes, nbytes);
  close(fd);

  return nbytes;
}
