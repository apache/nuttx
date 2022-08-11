/****************************************************************************
 * libs/libc/netdb/lib_dnsinit.c
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

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <arpa/inet.h>

#include <nuttx/sched.h>
#include <nuttx/mutex.h>
#include "netdb/lib_dns.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Protects DNS cache, nameserver list and notify list. */

static rmutex_t g_dns_lock = NXRMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_semtake
 *
 * Description:
 *   Take the DNS lock, ignoring errors due to the receipt of signals.
 *
 ****************************************************************************/

void dns_semtake(void)
{
  nxrmutex_lock(&g_dns_lock);
}

/****************************************************************************
 * Name: dns_semgive
 *
 * Description:
 *   Release the DNS lock
 *
 ****************************************************************************/

void dns_semgive(void)
{
  nxrmutex_unlock(&g_dns_lock);
}
