/****************************************************************************
 * libs/libc/netdb/lib_resmkquery.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdlib.h>
#include <string.h>
#include <resolv.h>

#include <arpa/nameser.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: res_mkquery
 *
 * Description:
 *   Build a DNS query message with one question for the domain name dname
 *   of the given class and type.  The data and newrr arguments are not
 *   used.
 *
 * Input Parameters:
 *   op      - The operation code, normally QUERY.
 *   dname   - The domain name to query.
 *   rclass  - The class of the query, for example C_IN.
 *   type    - The type of the query, for example T_MX.
 *   data    - Not used.
 *   datalen - Not used.
 *   newrr   - Not used.
 *   buf     - The buffer that receives the message.
 *   buflen  - The size of buf.
 *
 * Returned Value:
 *   The size of the message on success, or -1 on failure.
 *
 ****************************************************************************/

int res_mkquery(int op, FAR const char *dname, int rclass, int type,
                FAR const unsigned char *data, int datalen,
                FAR const unsigned char *newrr,
                FAR unsigned char *buf, int buflen)
{
  FAR unsigned char *cp;
  uint16_t id;
  int n;

  UNUSED(data);
  UNUSED(datalen);
  UNUSED(newrr);

  if (op < 0 || op > 15 || rclass < 0 || rclass > 0xffff ||
      type < 0 || type > 0xffff || buf == NULL ||
      buflen < NS_HFIXEDSZ + 1 + NS_QFIXEDSZ)
    {
      return -1;
    }

  /* Encode the domain name after the header */

  n = dn_comp(dname, buf + NS_HFIXEDSZ,
              buflen - NS_HFIXEDSZ - NS_QFIXEDSZ, NULL, NULL);
  if (n < 0)
    {
      return -1;
    }

  /* The header: a random ID, recursion desired and one question */

  memset(buf, 0, NS_HFIXEDSZ);
  id     = arc4random();
  buf[0] = id >> 8;
  buf[1] = id;
  buf[2] = (op << 3) | 0x01;
  buf[5] = 1;

  /* The type and class of the question */

  cp = buf + NS_HFIXEDSZ + n;
  NS_PUT16(type, cp);
  NS_PUT16(rclass, cp);

  return cp - buf;
}
