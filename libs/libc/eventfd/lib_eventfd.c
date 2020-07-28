/****************************************************************************
 * libs/libc/lib_eventfd.c
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

#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int eventfd_read(int fd, FAR eventfd_t *value)
{
  return read(fd, value, sizeof (eventfd_t)) != sizeof (eventfd_t) ? -1 : 0;
}

int eventfd_write(int fd, eventfd_t value)
{
  return write(fd, &value,
      sizeof (eventfd_t)) != sizeof (eventfd_t) ? -1 : 0;
}

int eventfd_get_minor(int fd)
{
  int ret;
  int minor;

  ret = ioctl(fd, EFD_FIOC_MINOR, &minor);

  if (ret < 0)
    {
      return ret;
    }

  return minor;
}
