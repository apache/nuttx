/* uip-resolv.c
 * DNS host name to IP address resolver.
 *
 * The uIP DNS resolver functions are used to lookup a hostname and
 * map it to a numerical IP address. It maintains a list of resolved
 * hostnames that can be queried with the resolv_lookup()
 * function. New hostnames can be resolved using the resolv_query()
 * function.
 *
 * When a hostname has been resolved (or found to be non-existant),
 * the resolver code calls a callback function called resolv_found()
 * that must be implemented by the module that uses the resolver.
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based heavily on portions of uIP:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <string.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <net/uip/resolv.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_RESOLV_ENTRIES
#define RESOLV_ENTRIES 4
#else /* CONFIG_NET_RESOLV_ENTRIES */
#define RESOLV_ENTRIES CONFIG_NET_RESOLV_ENTRIES
#endif /* CONFIG_NET_RESOLV_ENTRIES */

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

/* The maximum number of retries when asking for a name */

#define MAX_RETRIES 8

#define DNS_FLAG1_RESPONSE        0x80
#define DNS_FLAG1_OPCODE_STATUS   0x10
#define DNS_FLAG1_OPCODE_INVERSE  0x08
#define DNS_FLAG1_OPCODE_STANDARD 0x00
#define DNS_FLAG1_AUTHORATIVE     0x04
#define DNS_FLAG1_TRUNC           0x02
#define DNS_FLAG1_RD              0x01
#define DNS_FLAG2_RA              0x80
#define DNS_FLAG2_ERR_MASK        0x0f
#define DNS_FLAG2_ERR_NONE        0x00
#define DNS_FLAG2_ERR_NAME        0x03

#define STATE_UNUSED 0
#define STATE_NEW    1
#define STATE_ASKING 2
#define STATE_DONE   3
#define STATE_ERROR  4

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The DNS message header */

struct dns_hdr
{
  uint16 id;
  uint8 flags1, flags2;
  uint16 numquestions;
  uint16 numanswers;
  uint16 numauthrr;
  uint16 numextrarr;
};

/* The DNS answer message structure */

struct dns_answer
{
  /* DNS answer record starts with either a domain name or a pointer
   * to a name already present somewhere in the packet.
   */

  uint16 type;
  uint16 class;
  uint16 ttl[2];
  uint16 len;
  uip_ipaddr_t ipaddr;
};

struct namemap
{
  uint8 state;
  uint8 tmr;
  uint8 retries;
  uint8 seqno;
  uint8 err;
  char name[32];
  uip_ipaddr_t ipaddr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct namemap names[RESOLV_ENTRIES];
static uint8 seqno;
static struct uip_udp_conn *resolv_conn = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Walk through a compact encoded DNS name and return the end of it. */

static unsigned char *parse_name(unsigned char *query)
{
  unsigned char n;

  do
    {
      n = *query++;

      while(n > 0)
        {
          ++query;
          --n;
        }
    }
  while(*query != 0);
  return query + 1;
}

/* Runs through the list of names to see if there are any that have
 * not yet been queried and, if so, sends out a query.
 */

static void check_entries(void)
{
  register struct dns_hdr *hdr;
  char *query, *nptr, *nameptr;
  static uint8 i;
  static uint8 n;
  register struct namemap *namemapptr;

  for(i = 0; i < RESOLV_ENTRIES; ++i)
    {
      namemapptr = &names[i];
      if (namemapptr->state == STATE_NEW ||
          namemapptr->state == STATE_ASKING)
        {
          if (namemapptr->state == STATE_ASKING)
            {
              if (--namemapptr->tmr == 0)
                {
                  if (++namemapptr->retries == MAX_RETRIES)
                    {
                      namemapptr->state = STATE_ERROR;
                      resolv_found(namemapptr->name, NULL);
                      continue;
                    }
                  namemapptr->tmr = namemapptr->retries;
                }
              else
                {
                  /* Its timer has not run out, so we move on to next entry. */
                  continue;
                }
            }
          else
            {
              namemapptr->state = STATE_ASKING;
              namemapptr->tmr = 1;
              namemapptr->retries = 0;
            }
          hdr = (struct dns_hdr *)uip_appdata;
          memset(hdr, 0, sizeof(struct dns_hdr));
          hdr->id = htons(i);
          hdr->flags1 = DNS_FLAG1_RD;
          hdr->numquestions = HTONS(1);
          query = (char *)uip_appdata + 12;
          nameptr = namemapptr->name;
          --nameptr;

          /* Convert hostname into suitable query format. */
          do
            {
              ++nameptr;
              nptr = query;
              ++query;
              for (n = 0; *nameptr != '.' && *nameptr != 0; ++nameptr)
                {
                  *query = *nameptr;
                  ++query;
                  ++n;
                }
              *nptr = n;
            }
          while(*nameptr != 0);
            {
              static unsigned char endquery[] = {0,0,1,0,1};
              memcpy(query, endquery, 5);
            }
          uip_udp_send((unsigned char)(query + 5 - (char *)uip_appdata));
          break;
        }
    }
}

/* Called when new UDP data arrives */

static void newdata(void)
{
  unsigned char *nameptr;
  struct dns_answer *ans;
  struct dns_hdr *hdr;
  static uint8 nquestions, nanswers;
  static uint8 i;
  register struct namemap *namemapptr;

  hdr = (struct dns_hdr *)uip_appdata;

  dbg( "ID %d\n", htons(hdr->id));
  dbg( "Query %d\n", hdr->flags1 & DNS_FLAG1_RESPONSE);
  dbg( "Error %d\n", hdr->flags2 & DNS_FLAG2_ERR_MASK);
  dbg( "Num questions %d, answers %d, authrr %d, extrarr %d\n",
       htons(hdr->numquestions), htons(hdr->numanswers),
       htons(hdr->numauthrr), htons(hdr->numextrarr));

  /* The ID in the DNS header should be our entry into the name
   * table.
   */

  i = htons(hdr->id);
  namemapptr = &names[i];
  if (i < RESOLV_ENTRIES && namemapptr->state == STATE_ASKING)
    {
      /* This entry is now finished */

      namemapptr->state = STATE_DONE;
      namemapptr->err = hdr->flags2 & DNS_FLAG2_ERR_MASK;

      /* Check for error. If so, call callback to inform */

      if (namemapptr->err != 0)
        {
          namemapptr->state = STATE_ERROR;
          resolv_found(namemapptr->name, NULL);
          return;
        }

      /* We only care about the question(s) and the answers. The authrr
       * and the extrarr are simply discarded.
       */

      nquestions = htons(hdr->numquestions);
      nanswers = htons(hdr->numanswers);

      /* Skip the name in the question. XXX: This should really be
       * checked agains the name in the question, to be sure that they
       * match.
       */

      nameptr = parse_name((unsigned char *)uip_appdata + 12) + 4;

      while(nanswers > 0)
        {
          /* The first byte in the answer resource record determines if it
           * is a compressed record or a normal one.
           */

          if (*nameptr & 0xc0)
            {
              /* Compressed name. */

              nameptr +=2;
              dbg("Compressed anwser\n");
            }
          else
            {
              /* Not compressed name. */
              nameptr = parse_name(nameptr);
            }

          ans = (struct dns_answer *)nameptr;
          dbg("Answer: type %x, class %x, ttl %x, length %x\n",
              htons(ans->type), htons(ans->class), (htons(ans->ttl[0]) << 16) | htons(ans->ttl[1]),
              htons(ans->len));

          /* Check for IP address type and Internet class. Others are discarded. */

          if (ans->type == HTONS(1) && ans->class == HTONS(1) && ans->len == HTONS(4))
            {
              dbg("IP address %d.%d.%d.%d\n",
                  htons(ans->ipaddr[0]) >> 8, htons(ans->ipaddr[0]) & 0xff,
                  htons(ans->ipaddr[1]) >> 8, htons(ans->ipaddr[1]) & 0xff);

              /* XXX: we should really check that this IP address is the one
               * we want.
               */

              namemapptr->ipaddr[0] = ans->ipaddr[0];
              namemapptr->ipaddr[1] = ans->ipaddr[1];

              resolv_found(namemapptr->name, namemapptr->ipaddr);
              return;
            }
          else
            {
              nameptr = nameptr + 10 + htons(ans->len);
            }
          --nanswers;
        }
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

void uip_interrupt_udp_event(void)
{
  if (uip_udp_conn->rport == HTONS(53))
    {
      if (uip_poll())
        {
          check_entries();
        }
      if (uip_newdata())
        {
          newdata();
        }
    }
}

/* Queues a name so that a question for the name will be sent out. */

void resolv_query(char *name)
{
  static uint8 i;
  static uint8 lseq, lseqi;
  register struct namemap *nameptr;

  lseq = lseqi = 0;

  for(i = 0; i < RESOLV_ENTRIES; ++i)
    {
      nameptr = &names[i];
      if (nameptr->state == STATE_UNUSED)
        {
          break;
        }
      if (seqno - nameptr->seqno > lseq)
        {
          lseq = seqno - nameptr->seqno;
          lseqi = i;
        }
    }

  if (i == RESOLV_ENTRIES)
    {
      i = lseqi;
      nameptr = &names[i];
    }

  dbg("Using entry %d\n", i);

  strcpy(nameptr->name, name);
  nameptr->state = STATE_NEW;
  nameptr->seqno = seqno;
  ++seqno;
}

/* Look up a hostname in the array of known hostnames.
 *
 * Note: This function only looks in the internal array of known
 * hostnames, it does not send out a query for the hostname if none
 * was found. The function resolv_query() can be used to send a query
 * for a hostname.
 *
 * Return A pointer to a 4-byte representation of the hostname's IP
 * address, or NULL if the hostname was not found in the array of
 * hostnames.
 */

uint16 *resolv_lookup(char *name)
{
  static uint8 i;
  struct namemap *nameptr;

  /* Walk through the list to see if the name is in there. If it is
   * not, we return NULL.
   */

  for(i = 0; i < RESOLV_ENTRIES; ++i)
    {
      nameptr = &names[i];
      if (nameptr->state == STATE_DONE && strcmp(name, nameptr->name) == 0)
        {
          return nameptr->ipaddr;
        }
    }
  return NULL;
}

/* Obtain the currently configured DNS server.
 *
 * Return: A pointer to a 4-byte representation of the IP address of
 * the currently configured DNS server or NULL if no DNS server has
 * been configured.
 */

uint16 *resolv_getserver(void)
{
  if (resolv_conn == NULL)
    {
      return NULL;
    }
  return resolv_conn->ripaddr;
}

/* Configure which DNS server to use for queries.
 *
 * dnsserver A pointer to a 4-byte representation of the IP
 * address of the DNS server to be configured.
 */

void resolv_conf(uint16 *dnsserver)
{
  if (resolv_conn != NULL)
    {
      uip_udp_remove(resolv_conn);
    }

  resolv_conn = uip_udp_new(dnsserver, HTONS(53));
}

/* Initalize the resolver. */

void resolv_init(void)
{
  static uint8 i;

  for(i = 0; i < RESOLV_ENTRIES; ++i)
    {
      names[i].state = STATE_DONE;
    }
}
