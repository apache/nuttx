/****************************************************************************
 * libs/libc/net/lib_htonl.c
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
#include <arpa/inet.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t htonl(uint32_t hl)
{
#ifdef CONFIG_ENDIAN_BIG
  return hl;
#else
  return (((hl) >> 24) |
          (((hl) >>  8) & 0x0000ff00) |
          (((hl) <<  8) & 0x00ff0000) |
           ((hl) << 24));
#endif
}

uint32_t ntohl(uint32_t nl)
{
#ifdef CONFIG_ENDIAN_BIG
  return nl;
#else
  return htonl(nl);
#endif
}
