/****************************************************************************
 * examples/uip/main.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
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

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <net/if.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-lib.h>

/* Here we include the header file for the application(s) we use in
 * our project as defined in the config/<board-name>/defconfig file
 */

/* DHCPC may be used in conjunction with any other feature (or not) */

#if defined(CONFIG_EXAMPLE_UIP_DHCPC)
# include <net/uip/resolv.h>
# include <net/uip/dhcpc.h>
#endif

/* Pick the netutils feature under test (which may be DHCPC) */

#if defined(CONFIG_EXAMPLE_UIP_SMTP)
# include <net/uip/smtp.h>
#elif defined(CONFIG_EXAMPLE_UIP_TELNETD)
# include <net/uip/telnetd.h>
#elif defined(CONFIG_EXAMPLE_UIP_WEBSERVER)
# include <net/uip/httpd.h>
#elif !defined(CONFIG_EXAMPLE_UIP_DHCPC)
# error "No network application specified"
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#  else
#    define message (void)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_EXAMPLE_UIP_SMTP)
static const char g_host_name[] = "localhost";
static const char g_recipient[] = "spudmonkey@racsa.co.cr";
static const char g_sender[]    = "nuttx-testing@example.com";
static const char g_subject[]   = "Testing SMTP from NuttX";
static const char g_msg_body[]  = "Test message sent by NuttX\r\n";
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_initialize
 ****************************************************************************/

#ifndef CONFIG_HAVE_WEAKFUNCTIONS
void user_initialize(void)
{
  /* Stub that must be provided only if the toolchain does
   * not support weak functions.
   */
}
#endif

/****************************************************************************
 * user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  struct in_addr addr;
#if defined(CONFIG_EXAMPLE_UIP_DHCPC) || defined(CONFIG_EXAMPLE_UIP_NOMAC)
  uint8 mac[IFHWADDRLEN];
#endif
#if defined(CONFIG_EXAMPLE_UIP_DHCPC) || defined(CONFIG_EXAMPLE_UIP_SMTP)
  void *handle;
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLE_UIP_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xb0;
  mac[3] = 0x0b;
  mac[4] = 0xba;
  mac[5] = 0xbe;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

#if !defined(CONFIG_EXAMPLE_UIP_DHCPC)
  addr.s_addr = HTONL(CONFIG_EXAMPLE_UIP_IPADDR);
#else
  addr.s_addr = 0;
#endif
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_UIP_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_UIP_NETMASK);
  uip_setnetmask("eth0", &addr);

#if defined(CONFIG_EXAMPLE_UIP_DHCPC)
  /* Set up the resolver */

  resolv_init();
#endif

#if defined(CONFIG_EXAMPLE_UIP_DHCPC)
  /* Get the MAC address of the NIC */

  uip_getmacaddr("eth0", mac);

  /* Set up the DHCPC modules */

  handle = dhcpc_open(&mac, IFHWADDRLEN);

  /* Get an IP address */

  printf("Getting IP address\n");
  if (handle)
    {
        struct dhcpc_state ds;
        (void)dhcpc_request(handle, &ds);
        uip_sethostaddr("eth1", &ds.ipaddr);
        if (ds.netmask.s_addr != 0)
          {
            uip_setnetmask("eth0", &ds.netmask);
          }
        if (ds.default_router.s_addr != 0)
          {
            uip_setdraddr("eth0", &ds.default_router);
          }
        if (ds.dnsaddr.s_addr != 0)
          {
            resolv_conf(&ds.dnsaddr);
          }
        dhcpc_close(handle);
    }
#endif

#if defined(CONFIG_EXAMPLE_UIP_WEBSERVER)
  printf("Starting webserver\n");
  httpd_init();
  httpd_listen();
#elif defined(CONFIG_EXAMPLE_UIP_TELNETD)
  printf("Starting telnetd\n");
  telnetd_init();
#elif defined(CONFIG_EXAMPLE_UIP_SMTP)
  printf("Sending mail\n");
  uip_ipaddr(addr.s_addr, 127, 0, 0, 1);
  handle = smtp_open();
  if (handle)
    {
      smtp_configure(handle, g_host_name, &addr.s_addr);
      smtp_send(handle, g_recipient, NULL, g_sender, g_subject,
                g_msg_body, strlen(g_msg_body));
      smtp_close(handle);
    }
#endif

  while(1)
    {
      sleep(3);
      printf("main: Still running\n");
#if CONFIG_NFILE_DESCRIPTORS > 0
      fflush(stdout);
#endif
    }
  return 0;
}
