/****************************************************************************
 * net/netlink/netlink_attr.c
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

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/net/netlink.h>

#include "netlink.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NETLINK_VALIDATE_POLICY
static const uint8_t g_nla_attr_len[NLA_TYPE_MAX + 1] =
{
  0,
  sizeof(uint8_t),
  sizeof(uint16_t),
  sizeof(uint32_t),
  sizeof(uint64_t),
  0, 0, 0, 0, 0, 0, 0,
  sizeof(int8_t),
  sizeof(int16_t),
  sizeof(int32_t),
  sizeof(int64_t),
};

static const uint8_t g_nla_attr_minlen[NLA_TYPE_MAX + 1] =
{
  0,
  sizeof(uint8_t),
  sizeof(uint16_t),
  sizeof(uint32_t),
  sizeof(uint64_t),
  0, 0,
  sizeof(uint64_t),
  NLA_HDRLEN,
  0, 0, 0,
  sizeof(int8_t),
  sizeof(int16_t),
  sizeof(int32_t),
  sizeof(int64_t),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int validate_nla_bitfield32(FAR const struct nlattr *nla,
                                   FAR uint32_t *valid_flags_mask)
{
  FAR const struct nla_bitfield32 *bf = nla_data(nla);

  if (!valid_flags_mask)
    {
      return -EINVAL;
    }

  /* disallow invalid bit selector */

  if (bf->selector & ~*valid_flags_mask)
    {
      return -EINVAL;
    }

  /* disallow invalid bit values */

  if (bf->value & ~*valid_flags_mask)
    {
      return -EINVAL;
    }

  /* disallow valid bit values that are not selected */

  if (bf->value & ~bf->selector)
    {
      return -EINVAL;
    }

  return 0;
}

static int validate_nla(FAR const struct nlattr *nla, int maxtype,
                        FAR const struct nla_policy *policy)
{
  FAR const struct nla_policy *pt;
  int minlen = 0;
  int attrlen = nla_len(nla);
  int type = nla_type(nla);

  if (type <= 0 || type > maxtype)
    {
      return -EINVAL;
    }

  pt = &policy[type];

  DEBUGASSERT(pt->type <= NLA_TYPE_MAX);

  if (g_nla_attr_len[pt->type] && attrlen != g_nla_attr_len[pt->type])
    {
      nwarn("netlink: '%d': attribute type %d has an invalid length.\n",
            getpid(), type);
      return -EINVAL;
    }

  switch (pt->type)
    {
      case NLA_FLAG:
        if (attrlen > 0)
          {
            return -ERANGE;
          }
        break;

      case NLA_BITFIELD32:
        if (attrlen != sizeof(struct nla_bitfield32))
          {
            return -ERANGE;
          }

        if (validate_nla_bitfield32(nla, pt->validation_data) != 0)
          {
            return -EINVAL;
          }
        break;

      case NLA_NUL_STRING:
        if (pt->len)
          {
            minlen = MIN(attrlen, pt->len + 1);
          }
        else
          {
            minlen = attrlen;
          }

        if (!minlen || memchr(nla_data(nla), '\0', minlen) == NULL)
          {
            return -EINVAL;
          }

      /* fall through */

      case NLA_STRING:
        if (attrlen < 1)
          {
            return -ERANGE;
          }

        if (pt->len)
          {
            FAR char *buf = nla_data(nla);

            if (buf[attrlen - 1] == '\0')
              {
                attrlen--;
              }

            if (attrlen > pt->len)
              {
                return -ERANGE;
              }
          }
        break;

      case NLA_BINARY:
        if (pt->len && attrlen > pt->len)
          {
            return -ERANGE;
          }
        break;

      case NLA_NESTED_COMPAT:
        if (attrlen < pt->len)
          {
            return -ERANGE;
          }

        if (attrlen < NLA_ALIGN(pt->len))
          {
            break;
          }

        if (attrlen < NLA_ALIGN(pt->len) + NLA_HDRLEN)
          {
            return -ERANGE;
          }

        nla = nla_data(nla) + NLA_ALIGN(pt->len);
        if (attrlen < NLA_ALIGN(pt->len) + NLA_HDRLEN + nla_len(nla))
          {
            return -ERANGE;
          }
        break;

      case NLA_NESTED:
        /* a nested attributes is allowed to be empty; if its not,
         * it must have a size of at least NLA_HDRLEN.
         */

        if (attrlen == 0)
          {
            break;
          }

      default:
        if (pt->len)
          {
            minlen = pt->len;
          }
        else if (pt->type != NLA_UNSPEC)
          {
            minlen = g_nla_attr_minlen[pt->type];
          }

        if (attrlen < minlen)
          {
            return -ERANGE;
          }
    }

  return 0;
}
#else
#  define validate_nla(nla, maxtype, policy) 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nla_next
 *
 * Description:
 *   Next netlink attribute in attribute stream.
 *
 * Input Parameters:
 *   nla - netlink attribute.
 *   remaining - number of bytes remaining in attribute stream.
 *
 * Returned Value:
 *   Returns the next netlink attribute in the attribute stream and
 *   decrements remaining by the size of the current attribute.
 *
 ****************************************************************************/

FAR struct nlattr *nla_next(FAR const struct nlattr *nla,
                            FAR int *remaining)
{
  unsigned int totlen = NLA_ALIGN(nla->nla_len);

  *remaining -= totlen;
  return (FAR struct nlattr *)((FAR char *)nla + totlen);
}

/****************************************************************************
 * Name: nla_parse
 *
 * Description:
 *   Parse the nested netlink attribute.
 *
 ****************************************************************************/

int nla_parse(FAR struct nlattr **tb, int maxtype,
              FAR const struct nlattr *head,
              int len, FAR const struct nla_policy *policy,
              FAR struct netlink_ext_ack *extack)
{
  FAR const struct nlattr *nla;
  int rem;
  int err = 0;

  memset(tb, 0, sizeof(FAR struct nlattr *) * (maxtype + 1));

  nla_for_each_attr(nla, head, len, rem)
    {
      uint16_t type = nla_type(nla);

      if (type > 0 && type <= maxtype)
        {
          err = validate_nla(nla, maxtype, policy);
          if (err < 0)
            {
              nl_set_err_msg_attr(extack, nla,
                                  "Attribute failed policy validation");
              goto errout;
            }

          tb[type] = (FAR struct nlattr *)nla;
        }
    }

  if (rem > 0)
    {
      nwarn("netlink: %d bytes leftover after parsing attributes in "
            "pid `%d'.\n", rem, getpid());
    }

errout:
  return err;
}
