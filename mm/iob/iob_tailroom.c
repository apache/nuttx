/****************************************************************************
 * mm/iob/iob_tailroom.c
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

#include <nuttx/config.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_tailroom
 *
 * Description:
 *  Return the number of bytes at the tail of the I/O buffer chain which
 *  can be used to append data without additional allocations.
 *
 ****************************************************************************/

unsigned int iob_tailroom(FAR struct iob_s *iob)
{
  while (iob->io_flink != NULL)
    {
      iob = iob->io_flink;
    }

  return CONFIG_IOB_BUFSIZE - (iob->io_offset + iob->io_len);
}
