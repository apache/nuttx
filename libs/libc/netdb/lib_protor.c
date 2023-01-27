/****************************************************************************
 * libs/libc/netdb/lib_protor.c
 *
 * Copyright © 2005-2020 Rich Felker, et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <netdb.h>
#include <string.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const unsigned char g_protos[] =
{
  "\000ip\0"
  "\001icmp\0"
  "\002igmp\0"
  "\003ggp\0"
  "\004ipencap\0"
  "\005st\0"
  "\006tcp\0"
  "\010egp\0"
  "\014pup\0"
  "\021udp\0"
  "\024hmp\0"
  "\026xns-idp\0"
  "\033rdp\0"
  "\035iso-tp4\0"
  "\044xtp\0"
  "\045ddp\0"
  "\046idpr-cmtp\0"
  "\051ipv6\0"
  "\053ipv6-route\0"
  "\054ipv6-frag\0"
  "\055idrp\0"
  "\056rsvp\0"
  "\057gre\0"
  "\062esp\0"
  "\063ah\0"
  "\071skip\0"
  "\072ipv6-icmp\0"
  "\073ipv6-nonxt\0"
  "\074ipv6-opts\0"
  "\111rspf\0"
  "\121vmtp\0"
  "\131ospf\0"
  "\136ipip\0"
  "\142encap\0"
  "\147pim\0"
  "\377raw"
};

static const char *g_aliases;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void setprotoent_r(int stayopen, FAR struct protoent *result_buf)
{
  result_buf->idx = 0;
}

void endprotoent_r(FAR struct protoent *result_buf)
{
  result_buf->idx = 0;
}

int getprotoent_r(FAR struct protoent *result_buf, FAR char *buf,
                  size_t buflen, FAR struct protoent **result)
{
  int idx = result_buf->idx;
  if (idx >= sizeof(g_protos))
    {
      *result = NULL;
      return ENOENT;
    }

  result_buf->p_proto = g_protos[idx];
  result_buf->p_name = (char *)&g_protos[idx + 1];
  result_buf->p_aliases = (char **)&g_aliases;
  idx += strlen(result_buf->p_name) + 2;
  result_buf->idx = idx;
  *result = result_buf;

  return OK;
}

int getprotobyname_r(FAR const char *name,
                     FAR struct protoent *result_buf, FAR char *buf,
                     size_t buflen, FAR struct protoent **result)
{
  int ret;
  endprotoent_r(result_buf);
  do
    {
      ret = getprotoent_r(result_buf, buf, buflen, result);
    }
  while (ret == OK && strcmp(name, result_buf->p_name));

  return ret;
}

int getprotobynumber_r(int proto,
                       FAR struct protoent *result_buf, FAR char *buf,
                       size_t buflen, FAR struct protoent **result)
{
  int ret;
  endprotoent_r(result_buf);
  do
    {
      ret = getprotoent_r(result_buf, buf, buflen, result);
    }
  while (ret == OK && result_buf->p_proto != proto);

  return ret;
}
