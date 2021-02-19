/****************************************************************************
 * net/udp/udp_wrbuffer_dump.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "udp/udp.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_wrbuffer_dump
 *
 * Description:
 *  Dump the contents of a write buffer
 *
 ****************************************************************************/

void udp_wrbuffer_dump(FAR const char *msg, FAR struct udp_wrbuffer_s *wrb,
                       unsigned int len, unsigned int offset)
{
  syslog(LOG_DEBUG, "%s: wrb=%p pktlen=%d\n",
         msg, wrb, wrb->wb_iob->io_pktlen);
  iob_dump("I/O Buffer Chain", wrb->wb_iob, len, offset);
}

#endif /* CONFIG_DEBUG_FEATURES */
