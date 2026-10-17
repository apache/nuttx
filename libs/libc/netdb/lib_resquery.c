/****************************************************************************
 * libs/libc/netdb/lib_resquery.c
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

#include <netdb.h>
#include <resolv.h>

#include <arpa/nameser.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: res_query
 *
 * Description:
 *   Query the name servers for records of the given class and type for
 *   the domain name dname, and copy the raw answer into the buffer
 *   answer.
 *
 * Input Parameters:
 *   dname  - The domain name to query.
 *   rclass - The class of the query, for example C_IN.
 *   type   - The type of the query, for example T_SRV.
 *   answer - The buffer that receives the answer message.
 *   anslen - The size of answer.
 *
 * Returned Value:
 *   The size of the answer on success.  On failure, -1 is returned and
 *   h_errno is set to HOST_NOT_FOUND, NO_DATA, NO_RECOVERY or TRY_AGAIN.
 *
 ****************************************************************************/

int res_query(FAR const char *dname, int rclass, int type,
              FAR unsigned char *answer, int anslen)
{
  unsigned char query[NS_HFIXEDSZ + NS_MAXCDNAME + NS_QFIXEDSZ];
  int ret;

  ret = res_mkquery(QUERY, dname, rclass, type, NULL, 0, NULL,
                    query, sizeof(query));
  if (ret < 0)
    {
      h_errno = NO_RECOVERY;
      return -1;
    }

  ret = res_send(query, ret, answer, anslen);
  if (ret < NS_HFIXEDSZ)
    {
      h_errno = TRY_AGAIN;
      return -1;
    }

  switch (answer[3] & 0x0f)
    {
      case ns_r_noerror:
        if (answer[6] == 0 && answer[7] == 0)
          {
            h_errno = NO_DATA;
            return -1;
          }

        return ret;

      case ns_r_nxdomain:
        h_errno = HOST_NOT_FOUND;
        break;

      case ns_r_servfail:
        h_errno = TRY_AGAIN;
        break;

      default:
        h_errno = NO_RECOVERY;
        break;
    }

  return -1;
}

/****************************************************************************
 * Name: res_search
 *
 * Description:
 *   Same as res_query().  NuttX has no search domain list, so the name is
 *   queried as given.
 *
 ****************************************************************************/

int res_search(FAR const char *dname, int rclass, int type,
               FAR unsigned char *answer, int anslen)
{
  return res_query(dname, rclass, type, answer, anslen);
}
