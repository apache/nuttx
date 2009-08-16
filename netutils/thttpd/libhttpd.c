/****************************************************************************
 * netutils/thttpd/libhttpd.c
 * HTTP Protocol Library
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995,1998,1999,2000,2001 by Jef Poskanzer <jef@mail.acme.com>.
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <fcntl.h>
#include <dirent.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/regex.h>
#include <nuttx/symtab.h>
#include <nuttx/binfmt.h>
#include <net/uip/thttpd.h>

#include "config.h"
#include "libhttpd.h"
#include "timers.h"
#include "tdate_parse.h"
#include "fdwatch.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef STDIN_FILENO
#  define STDIN_FILENO 0
#endif
#ifndef STDOUT_FILENO
#  define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#  define STDERR_FILENO 2
#endif

#define NAMLEN(dirent) strlen((dirent)->d_name)

extern char *crypt(const char *key, const char *setting);

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/* Conditional macro to allow two alternate forms for use in the built-in
 * error pages.  If EXPLICIT_ERROR_PAGES is defined, the second and more
 * explicit error form is used; otherwise, the first and more generic
 * form is used.
 */

#ifdef EXPLICIT_ERROR_PAGES
#  define ERROR_FORM(a,b) b
#else
#  define ERROR_FORM(a,b) a
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_THTTPD_CGI_PATTERN
enum cgi_outbuffer_e
{
  CGI_OUTBUFFER_READHEADER = 0,  /* Reading header from HTTP client */
  CGI_OUTBUFFER_HEADERREAD,      /* Header has been read */
  CGI_OUTBUFFER_HEADERSENT,      /* Header has been sent to the CGI program */
  CGI_OUTBUFFER_READDATA,        /* Transferring data from CGI to client */
  CGI_OUTBUFFER_DONE,            /* Finished */
};

struct cgi_outbuffer_s
{
  enum cgi_outbuffer_e state;    /* State of the transfer */
  char  *buffer;                 /* Allocated I/O buffer */
  size_t size;                   /* Size of the allocation */
  size_t len;                    /* Amount of valid data in the allocated buffer */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void free_httpd_server(httpd_server *hs);
static int  initialize_listen_socket(httpd_sockaddr *saP);
static void add_response(httpd_conn *hc, const char *str);
static void send_mime(httpd_conn *hc, int status, const char *title, const char *encodings,
                      const char *extraheads, const char *type, off_t length, time_t mod);
static void send_response(httpd_conn *hc, int status, const char *title,
                          const char *extraheads, const char *form, const char *arg);
static void send_response_tail(httpd_conn *hc);
static void defang(const char *str, char *dfstr, int dfsize);
#ifdef CONFIG_THTTPD_ERROR_DIRECTORY
static int  send_err_file(httpd_conn *hc, int status, char *title,
                           char *extraheads, char *filename);
#endif
#ifdef CONFIG_THTTPD_AUTH_FILE
static void send_authenticate(httpd_conn *hc, char *realm);
static int  b64_decode(const char *str, unsigned char *space, int size);
static int  auth_check(httpd_conn *hc, char *dirname);
static int  auth_check2(httpd_conn *hc, char *dirname);
#endif
static void send_dirredirect(httpd_conn *hc);
static int  hexit(char c);
static void strdecode(char *to, char *from);
#ifdef CONFIG_THTTPD_GENERATE_INDICES
static void strencode(char *to, int tosize, char *from);
#endif
#ifdef CONFIG_THTTPD_TILDE_MAP1
static int CONFIG_THTTPD_TILDE_MAP1(httpd_conn *hc);
#endif
#ifdef CONFIG_THTTPD_TILDE_MAP2
static int CONFIG_THTTPD_TILDE_MAP2(httpd_conn *hc);
#endif
#ifdef CONFIG_THTTPD_VHOST
static int  vhost_map(httpd_conn *hc);
#endif
static char *expand_filename(char *path, char **restP, boolean tildemapped);
static char *bufgets(httpd_conn *hc);
static void de_dotdot(char *file);
static void init_mime(void);
static void figure_mime(httpd_conn *hc);
#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
static void cgi_kill2(ClientData client_data, struct timeval *nowP);
static void cgi_kill(ClientData client_data, struct timeval *nowP);
#endif
#ifdef CONFIG_THTTPD_GENERATE_INDICES
static void ls_child(int argc, char **argv);
static int  ls(httpd_conn *hc);
#endif
#ifdef SERVER_NAME_LIST
static char *hostname_map(char *hostname);
#endif

/* CGI Support */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static void create_environment(httpd_conn *hc);
static char **make_argp(httpd_conn *hc);
static inline int cgi_interpose_input(httpd_conn *hc, int wfd, char *buffer);
static inline int cgi_interpose_output(httpd_conn *hc, int rfd, char *inbuffer,
                                          struct cgi_outbuffer_s *hdr);
static int  cgi_child(int argc, char **argv);
static int  cgi(httpd_conn *hc);
#endif

static int  check_referer(httpd_conn *hc);
#ifdef CONFIG_THTTPD_URLPATTERN
static int  really_check_referer(httpd_conn *hc);
#endif
#ifdef CONFIG_DEBUG
static int  sockaddr_check(httpd_sockaddr *saP);
#else
#  define sockaddr_check(saP) (1)
#endif
static size_t sockaddr_len(httpd_sockaddr *saP);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This global keeps track of whether we are in the main process or a
 * sub-process.  The reason is that httpd_write_response() can get called
 * in either context; when it is called from the main process it must use
 * non-blocking I/O to avoid stalling the server, but when it is called
 * from a sub-process it wants to use blocking I/O so that the whole
 * response definitely gets written.  So, it checks this variable.  A bit
 * of a hack but it seems to do the right thing.
 */

static pid_t  main_thread;
static int    str_alloc_count = 0;
static size_t str_alloc_size  = 0;

/* This is the 'root' of the Filesystem as seen by the HTTP client */

static const char httpd_root[] = CONFIG_THTTPD_PATH;

/* Include MIME encodings and types */

#include "mime_types.h"

/****************************************************************************
 * HTTP Strings
 ****************************************************************************/

static const char ok200title[]  = "OK";
static const char ok206title[]  = "Partial Content";

static const char err302title[] = "Found";
static const char err302form[]  = "The actual URL is '%s'.\n";

static const char err304title[] = "Not Modified";

const char httpd_err400title[]  = "Bad Request";
const char httpd_err400form[]   = "Your request has bad syntax or is inherently impossible to satisfy.\n";

#ifdef CONFIG_THTTPD_AUTH_FILE
static const char err401title[] = "Unauthorized";
static const char err401form[]  = "Authorization required for the URL '%s'.\n";
#endif

static const char err403title[] = "Forbidden";
#ifndef EXPLICIT_ERROR_PAGES
static const char err403form[]  = "You do not have permission to get URL '%s' from this server.\n";
#endif

static const char err404title[] = "Not Found";
static const char err404form[]  = "The requested URL '%s' was not found on this server.\n";

const char httpd_err408title[]  = "Request Timeout";
const char httpd_err408form[]   = "No request appeared within a reasonable time period.\n";

static const char err500title[] = "Internal Error";
static const char err500form[]  = "There was an unusual problem serving the requested URL '%s'.\n";

static const char err501title[] = "Not Implemented";
static const char err501form[]  = "The requested method '%s' is not implemented by this server.\n";

static const char httpd_err503title[] = "Service Temporarily Overloaded";
static const char httpd_err503form[]  = "The requested URL '%s' is temporarily overloaded.  Please try again later.\n";

static const char html_crlf[]      = "\r\n";
static const char html_html[]      = "<HTML>\r\n";
static const char html_endhtml[]   = "</HTML>\r\n";
static const char html_hdtitle[]   = "<HEAD><TITLE>";
static const char html_titlehd[]   = "</TITLE></HEAD>\r\n";
static const char html_body[]      = "<BODY BGCOLOR=\"#99cc99\" TEXT=\"#000000\" LINK=\"#2020ff\" VLINK=\"#4040cc\">\r\n";
static const char html_endbody[]   = "</BODY>\r\n";
static const char html_hdr2[]      = "<H2>";
static const char html_endhdr2[]   = "</H2>";

static const char *index_names[]   = { CONFIG_THTTPD_INDEX_NAMES };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void free_httpd_server(httpd_server * hs)
{
  if (hs)
    {
      if (hs->hostname)
        {
          free(hs->hostname);
        }

      free(hs);
    }
}

static int initialize_listen_socket(httpd_sockaddr *saP)
{
  int listen_fd;
  int on;
  int flags;

  /* Check sockaddr. */

#ifdef CONFIG_DEBUG
  if (!sockaddr_check(saP))
    {
      ndbg("unknown sockaddr family on listen socket\n");
      return -1;
    }
#endif

  /* Create socket. */

  nvdbg("Create listen socket\n");
  listen_fd = socket(saP->sin_family, SOCK_STREAM, 0);
  if (listen_fd < 0)
    {
      ndbg("socket failed: %d\n", errno);
      return -1;
    }

  /* Allow reuse of local addresses. */

  on = 1;
  if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on)) < 0)
    {
      ndbg("setsockopt(SO_REUSEADDR) failed: %d\n", errno);
    }

  /* Bind to it. */

  if (bind(listen_fd, (struct sockaddr*)saP, sockaddr_len(saP)) < 0)
    {
      ndbg("bind to %s failed: %d\n", httpd_ntoa(saP), errno);
      (void)close(listen_fd);
      return -1;
    }

  /* Set the listen file descriptor to no-delay / non-blocking mode. */

  flags = fcntl(listen_fd, F_GETFL, 0);
  if (flags == -1)
    {
      ndbg("fcntl(F_GETFL) failed: %d\n", errno);
      (void)close(listen_fd);
      return -1;
    }

  if (fcntl(listen_fd, F_SETFL, flags | O_NDELAY) < 0)
    {
      ndbg("fcntl(O_NDELAY) failed: %d\n", errno);
      (void)close(listen_fd);
      return -1;
    }

  /* Start a listen going. */

  if (listen(listen_fd, CONFIG_THTTPD_LISTEN_BACKLOG) < 0)
    {
      ndbg("listen failed: %d\n", errno);
      (void)close(listen_fd);
      return -1;
    }

  return listen_fd;
}

/* Append a string to the buffer waiting to be sent as response. */

static void add_response(httpd_conn *hc, const char *str)
{
  int resplen;
  int len;

  len     = strlen(str);
  resplen = hc->buflen + len;

  if (resplen > CONFIG_THTTPD_IOBUFFERSIZE)
    {
      ndbg("resplen(%d) > buffer size(%d)\n", resplen, CONFIG_THTTPD_IOBUFFERSIZE);
      resplen = CONFIG_THTTPD_IOBUFFERSIZE;
      len     = resplen - hc->buflen;
    }

  memcpy(&(hc->buffer[hc->buflen]), str, len);
  hc->buflen = resplen;
}

static void send_mime(httpd_conn *hc, int status, const char *title, const char *encodings,
                      const char *extraheads, const char *type, off_t length, time_t mod)
{
  struct timeval now;
  const char *rfc1123fmt = "%a, %d %b %Y %H:%M:%S GMT";
  char tmbuf[72];
#ifdef CONFIG_THTTPD_MAXAGE
  time_t expires;
  char expbuf[72];
#endif
  char fixed_type[72];
  char buf[128];
  int partial_content;
  int s100;

  hc->status = status;
  hc->bytes_to_send = length;
  if (hc->mime_flag)
    {
      if (status == 200 && hc->got_range &&
          (hc->range_end >= hc->range_start) &&
          ((hc->range_end != length - 1) ||
           (hc->range_start != 0)) &&
          (hc->range_if == (time_t) - 1 || hc->range_if == hc->sb.st_mtime))
        {
          partial_content = 1;
          hc->status = status = 206;
          title = ok206title;
        }
      else
        {
          partial_content = 0;
          hc->got_range = FALSE;
        }

      gettimeofday(&now, NULL);
      if (mod == (time_t)0)
        {
          mod = now.tv_sec;
        }

      (void)snprintf(fixed_type, sizeof(fixed_type), type, CONFIG_THTTPD_CHARSET);
      (void)snprintf(buf, sizeof(buf), "%.20s %d %s\r\n", hc->protocol, status, title);
      add_response(hc, buf);
      (void)snprintf(buf, sizeof(buf), "Server: %s\r\n", "thttpd");
      add_response(hc, buf);
      (void)snprintf(buf, sizeof(buf), "Content-Type: %s\r\n", fixed_type);
      add_response(hc, buf);
      (void)strftime(tmbuf, sizeof(tmbuf), rfc1123fmt, gmtime(&now.tv_sec));
      (void)snprintf(buf, sizeof(buf), "Date: %s\r\n", tmbuf);
      add_response(hc, buf);
      (void)strftime(tmbuf, sizeof(tmbuf), rfc1123fmt, gmtime(&mod));
      (void)snprintf(buf, sizeof(buf), "Last-Modified: %s\r\n", tmbuf);
      add_response(hc, buf);
      add_response(hc, "Accept-Ranges: bytes\r\n");
      add_response(hc, "Connection: close\r\n");

      s100 = status / 100;
      if (s100 != 2 && s100 != 3)
        {
          (void)snprintf(buf, sizeof(buf), "Cache-Control: no-cache,no-store\r\n");
          add_response(hc, buf);
        }

      if (encodings[0] != '\0')
        {
          (void)snprintf(buf, sizeof(buf), "Content-Encoding: %s\r\n", encodings);
          add_response(hc, buf);
        }

      if (partial_content)
        {
          (void)snprintf(buf, sizeof(buf),"Content-Range: bytes %ld-%ld/%ld\r\n",
                         (long)hc->range_start, (long)hc->range_end, (long)length);
          add_response(hc, buf);
          (void)snprintf(buf, sizeof(buf),"Content-Length: %ld\r\n",
                         (long)(hc->range_end - hc->range_start + 1));
          add_response(hc, buf);
        }
      else if (length >= 0)
        {
          (void)snprintf(buf, sizeof(buf), "Content-Length: %ld\r\n", (long)length);
          add_response(hc, buf);
        }

#ifdef CONFIG_THTTPD_P3P
      (void)snprintf(buf, sizeof(buf), "P3P: %s\r\n", CONFIG_THTTPD_P3P);
      add_response(hc, buf);
#endif

#ifdef CONFIG_THTTPD_MAXAGE
      expires = now + CONFIG_THTTPD_MAXAGE;
      (void)strftime(expbuf, sizeof(expbuf), rfc1123fmt, gmtime(&expires));
      (void)snprintf(buf, sizeof(buf),
                     "Cache-Control: max-age=%d\r\nExpires: %s\r\n",
                     CONFIG_THTTPD_MAXAGE, expbuf);
      add_response(hc, buf);
#endif

      if (extraheads[0] != '\0')
        {
          add_response(hc, extraheads);
        }
      add_response(hc, "\r\n");
    }
}

static void send_response(httpd_conn *hc, int status, const char *title, const char *extraheads,
                          const char *form, const char *arg)
{
  char defanged[72];
  char buf[128];

  nvdbg("title: \"%s\" form: \"%s\"\n", title, form);

  send_mime(hc, status, title, "", extraheads, "text/html; charset=%s", (off_t)-1, (time_t)0);
  add_response(hc, html_html);
  add_response(hc, html_hdtitle);
  (void)snprintf(buf, sizeof(buf), "%d %s", status, title);
  add_response(hc, buf);
  add_response(hc, html_titlehd);
  add_response(hc, html_body);
  add_response(hc, html_hdr2);
  add_response(hc, buf);
  add_response(hc, html_endhdr2);

  defang(arg, defanged, sizeof(defanged));
  (void)snprintf(buf, sizeof(buf), form, defanged);
  add_response(hc, buf);

  if (match("**MSIE**", hc->useragent))
    {
      int n;
      add_response(hc, "<!--\n");
      for (n = 0; n < 6; ++n)
        add_response(hc,
                     "Padding so that MSIE deigns to show this error instead of its own canned one.\n");
      add_response(hc, "-->\n");
    }

  send_response_tail(hc);
}

static void send_response_tail(httpd_conn *hc)
{
  add_response(hc, "<HR>\r\n<ADDRESS><A HREF=\"");
  add_response(hc, CONFIG_THTTPD_SERVER_ADDRESS);
  add_response(hc, "\">");
  add_response(hc, "thttpd");
  add_response(hc, "</A></ADDRESS>\r\n");
  add_response(hc, html_endbody);
  add_response(hc, html_endhtml);
}

static void defang(const char *str, char *dfstr, int dfsize)
{
  const char *cp1;
  char *cp2;

  for (cp1 = str, cp2 = dfstr;
       *cp1 != '\0' && cp2 - dfstr < dfsize - 5; ++cp1, ++cp2)
    {
      switch (*cp1)
        {
        case '<':
          *cp2++ = '&';
          *cp2++ = 'l';
          *cp2++ = 't';
          *cp2 = ';';
          break;
        case '>':
          *cp2++ = '&';
          *cp2++ = 'g';
          *cp2++ = 't';
          *cp2 = ';';
          break;
        default:
          *cp2 = *cp1;
          break;
        }
    }
  *cp2 = '\0';
}

#ifdef CONFIG_THTTPD_ERROR_DIRECTORY
static int send_err_file(httpd_conn *hc, int status, char *title, char *extraheads,
                          char *filename)
{
  FILE *fp;
  char buf[1000];
  size_t nread;

  fp = fopen(filename, "r");
  if (fp == NULL)
    {
      return 0;
    }

  send_mime(hc, status, title, "", extraheads, "text/html; charset=%s",
            (off_t)-1, (time_t)0);
  for (;;)
    {
      nread = fread(buf, 1, sizeof(buf) - 1, fp);
      if (nread == 0)
        break;
      buf[nread] = '\0';
      add_response(hc, buf);
    }
  (void)fclose(fp);

#ifdef ERR_APPEND_SERVER_INFO
  send_response_tail(hc);
#endif

  return 1;
}
#endif

#ifdef CONFIG_THTTPD_AUTH_FILE
static void send_authenticate(httpd_conn *hc, char *realm)
{
  static char *header;
  static size_t maxheader = 0;
  static char headstr[] = "WWW-Authenticate: Basic realm=\"";

  httpd_realloc_str(&header, &maxheader, sizeof(headstr) + strlen(realm) + 3);
  (void)snprintf(header, maxheader, "%s%s\"\r\n", headstr, realm);
  httpd_send_err(hc, 401, err401title, header, err401form, hc->encodedurl);

  /* If the request was a POST then there might still be data to be read, so 
   * we need to do a lingering close.
   */

  if (hc->method == METHOD_POST)
    {
      hc->should_linger = TRUE;
    }
}

/* Base-64 decoding.  This represents binary data as printable ASCII
 * characters.  Three 8-bit binary bytes are turned into four 6-bit
 * values, like so:
 *
 *   [11111111][22222222][33333333] -> [111111][112222][222233][333333]
 *
 * Then the 6-bit values are represented using the characters "A-Za-z0-9+/".
 */

static inline b64_charmap(char *ch)
{
  char bin6;

       bin6 = -1;
       if (c == 0x20)                         /* ' ' */
         {
           bin6 = 62;                          /* ' '  maps to 62 */
         }
       elseif (c == 0x2f)                      /* '/' */
         {
           bin6 = 63;                          /* '/'  maps to 63 */
         }
       else if (c >=  0x30)                    /* '0' */
         {
           else if (c <= 0x39)                 /* '9' */
             {
               bin6 = (c - 0x39 + 52);         /* '0'-'9' maps to 52-61 */
             }
           else if (c >= 0x41)                 /* 'A' */
             {
               if (c <= 0x5a)                  /* 'Z' */
                 {
                   bin6 = c - 0x41;            /* 'A'-'Z' map to 0 - 25 */
                 }
               else if (c >= 0x61)             /* 'a' */
                 {
                   if (c <= 0x7a)              /* 'z' */
                     {
                       bin6 = c - 0x61 + 26;   /* 'a'-'z' map to 0 - 25 */
                     }
                 }
             }
         }

}

/* Do base-64 decoding on a string.  Ignore any non-base64 bytes.
 * Return the actual number of bytes generated.  The decoded size will
 * be at most 3/4 the size of the encoded, and may be smaller if there
 * are padding characters (blanks, newlines).
 */

static int b64_decode(const char *str, unsigned char *space, int size)
{
  const char *cp;
  int ndx;
  int phase;
  int decoded;
  int prev_decoded = 0;
  unsigned char packed;

  ndx = 0;
  phase = 0;
  for (cp = str; *cp != '\0', ndx < size; cp++)
    {
       /* Decode base-64 */

      decoded = b64_charmap(*cp);  /* Decode ASCII representations to 6-bit binary */
      if (decoded != -1)
        {
          switch (phase)
            {
            case 0:
              phase = 1;
              break;

            case 1:
              space[ndx++] = ((prev_decoded << 2) | ((decoded & 0x30) >> 4));
              phase = 2;
              break;

            case 2:
              space[ndx++] =(((prev_decoded & 0xf) << 4) | ((decoded & 0x3packed) >> 2));
              phase = 3;
              break;

            case 3:
              space[ndx++] =(((prev_decoded & 0x03) << 6) | decoded);
              phase = 0;
              break;
            }
          prev_decoded = decoded;
        }
    }
  return ndx;
}

/* Returns -1 == unauthorized, 0 == no auth file, 1 = authorized. */

static int auth_check(httpd_conn *hc, char *dirname)
{
#ifdef CONFIG_THTTPD_GLOBALPASSWD
  char *topdir;

#ifdef CONFIG_THTTPD_VHOST
  if (hc->hostdir[0] != '\0')
    {
      topdir = hc->hostdir;
    }
  else
#endif
    {
      topdir = httpd_root;
    }

  switch (auth_check2(hc, topdir))
    {
    case -1:
      return -1;
    case 1:
      return 1;
    }
#endif
  return auth_check2(hc, dirname);
}

/* Returns -1 == unauthorized, 0 == no auth file, 1 = authorized. */

static int auth_check2(httpd_conn *hc, char *dirname)
{
  static char *authpath;
  static size_t maxauthpath = 0;
  struct stat sb;
  char authinfo[500];
  char *authpass;
  char *colon;
  int l;
  FILE *fp;
  char line[500];
  char *cryp;
  static char *prevauthpath;
  static size_t maxprevauthpath = 0;
  static time_t prevmtime;
  static char *prevuser;
  static size_t maxprevuser = 0;
  static char *prevcryp;
  static size_t maxprevcryp = 0;

  /* Construct auth filename. */

  httpd_realloc_str(&authpath, &maxauthpath,
                    strlen(dirname) + 1 + sizeof(CONFIG_THTTPD_AUTH_FILE));
  (void)snprintf(authpath, maxauthpath, "%s/%s", dirname, CONFIG_THTTPD_AUTH_FILE);

  /* Does this directory have an auth file? */

  if (stat(authpath, &sb) < 0)
    {
      /* Nope, let the request go through. */

      return 0;
    }

  /* Does this request contain basic authorization info? */

  if (hc->authorization[0] == '\0' ||  strncmp(hc->authorization, "Basic ", 6) != 0)
    {
      /* Nope, return a 401 Unauthorized. */

      send_authenticate(hc, dirname);
      return -1;
    }

  /* Decode it. */

  l = b64_decode(&(hc->authorization[6]), (unsigned char *)authinfo, sizeof(authinfo) - 1);
  authinfo[l] = '\0';

  /* Split into user and password. */

  authpass = strchr(authinfo, ':');
  if (!authpass)
    {
      /* No colon? Bogus auth info. */

      send_authenticate(hc, dirname);
      return -1;
    }
  *authpass++ = '\0';

  /* If there are more fields, cut them off. */

  colon = strchr(authpass, ':');
  if (colon)
    {
      *colon = '\0';
    }

  /* See if we have a cached entry and can use it. */

  if (maxprevauthpath != 0 &&
      strcmp(authpath, prevauthpath) == 0 &&
      sb.st_mtime == prevmtime && strcmp(authinfo, prevuser) == 0)
    {
      /* Yes.  Check against the cached encrypted password. */

      if (strcmp(crypt(authpass, prevcryp), prevcryp) == 0)
        {
          /* Ok! */

          httpd_realloc_str(&hc->remoteuser, &hc->maxremoteuser,
                            strlen(authinfo));
          (void)strcpy(hc->remoteuser, authinfo);
          return 1;
        }
      else
        {
          /* No. */

          send_authenticate(hc, dirname);
          return -1;
        }
    }

  /* Open the password file. */

  fp = fopen(authpath, "r");
  if (fp == NULL)
    {
      /* The file exists but we can't open it? Disallow access. */

      ndbg("%s auth file %s could not be opened: %d\n",
             httpd_ntoa(&hc->client_addr), authpath, errno);

      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "The requested URL '%s' is protected by an authentication file, "
                                "but the authentication file cannot be opened.\n"),
                     hc->encodedurl);
      return -1;
    }

  /* Read it. */

  while (fgets(line, sizeof(line), fp) != NULL)
    {
      /* Nuke newline. */

      l = strlen(line);
      if (line[l - 1] == '\n')
        {
          line[l - 1] = '\0';
        }

      /* Split into user and encrypted password. */

      cryp = strchr(line, ':');
      if (!cryp)
        {
          continue;
        }
      *cryp++ = '\0';

      /* Is this the right user? */

      if (strcmp(line, authinfo) == 0)
        {
          /* Yes. */

          (void)fclose(fp);

          /* So is the password right? */

          if (strcmp(crypt(authpass, cryp), cryp) == 0)
            {
              /* Ok! */

              httpd_realloc_str(&hc->remoteuser, &hc->maxremoteuser, strlen(line));
              (void)strcpy(hc->remoteuser, line);

              /* And cache this user's info for next time. */

              httpd_realloc_str(&prevauthpath, &maxprevauthpath, strlen(authpath));
              (void)strcpy(prevauthpath, authpath);
              prevmtime = sb.st_mtime;
              httpd_realloc_str(&prevuser, &maxprevuser, strlen(authinfo));
              (void)strcpy(prevuser, authinfo);
              httpd_realloc_str(&prevcryp, &maxprevcryp, strlen(cryp));
              (void)strcpy(prevcryp, cryp);
              return 1;
            }
          else
            {
              /* No. */

              send_authenticate(hc, dirname);
              return -1;
            }
        }
    }

  /* Didn't find that user.  Access denied. */

  (void)fclose(fp);
  send_authenticate(hc, dirname);
  return -1;
}
#endif /* CONFIG_THTTPD_AUTH_FILE */

static void send_dirredirect(httpd_conn *hc)
{
  static char *location;
  static char *header;
  static size_t maxlocation = 0;
  static size_t maxheader = 0;
  static char headstr[] = "Location: ";

  if (hc->query[0] != '\0')
    {
      char *cp = strchr(hc->encodedurl, '?');
      if (cp)
        {
          *cp = '\0';
        }

      httpd_realloc_str(&location, &maxlocation, strlen(hc->encodedurl) + 2 + strlen(hc->query));
      (void)snprintf(location, maxlocation, "%s/?%s", hc->encodedurl, hc->query);
    }
  else
    {
      httpd_realloc_str(&location, &maxlocation, strlen(hc->encodedurl) + 1);
      (void)snprintf(location, maxlocation, "%s/", hc->encodedurl);
    }

  httpd_realloc_str(&header, &maxheader, sizeof(headstr) + strlen(location));
  (void)snprintf(header, maxheader, "%s%s\r\n", headstr, location);
  send_response(hc, 302, err302title, header, err302form, location);
}

static int hexit(char nibble)
{
  if (nibble >= '0' && nibble <= '9')
    {
      return nibble - '0';
    }
  else if (nibble >= 'a' && nibble <= 'f')
    {
      return nibble - 'a' + 10;
    }
  else if (nibble >= 'A' && nibble <= 'F')
    {
      return nibble - 'A' + 10;
    }
  return 0;
}

/* Copies and decodes a string.  It's ok for from and to to be the
 * same string.
 */

static void strdecode(char *to, char *from)
{
  for (; *from != '\0'; ++to, ++from)
    {
      if (from[0] == '%' && isxdigit(from[1]) && isxdigit(from[2]))
        {
          *to = hexit(from[1]) * 16 + hexit(from[2]);
          from += 2;
        }
      else
        {
          *to = *from;
        }
    }
  *to = '\0';
}

/* Copies and encodes a string. */

#ifdef CONFIG_THTTPD_GENERATE_INDICES
static void strencode(char *to, int tosize, char *from)
{
  int tolen;

  for (tolen = 0; *from != '\0' && tolen + 4 < tosize; ++from)
    {
      if (isalnum(*from) || strchr("/_.-~", *from) != NULL)
        {
          *to = *from;
          ++to;
          ++tolen;
        }
      else
        {
          (void)sprintf(to, "%%%02x", (int)*from & 0xff);
          to += 3;
          tolen += 3;
        }
    }
  *to = '\0';
}
#endif /* CONFIG_THTTPD_GENERATE_INDICES */

/* Map a ~username/whatever URL into <prefix>/username. */

#ifdef CONFIG_THTTPD_TILDE_MAP1
static intCONFIG_THTTPD_TILDE_MAP1(httpd_conn *hc)
{
  static char *temp;
  static size_t maxtemp = 0;
  int len;
  static char *prefix =CONFIG_THTTPD_TILDE_MAP1;

  len = strlen(hc->expnfilename) - 1;
  httpd_realloc_str(&temp, &maxtemp, len);
  (void)strcpy(temp, &hc->expnfilename[1]);

  httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename, strlen(prefix) + 1 + len);
  (void)strcpy(hc->expnfilename, prefix);

  if (prefix[0] != '\0')
    {
      (void)strcat(hc->expnfilename, "/");
    }

  (void)strcat(hc->expnfilename, temp);
  return 1;
}
#endif

/* Map a ~username/whatever URL into <user's homedir>/<postfix>. */

#ifdef CONFIG_THTTPD_TILDE_MAP2
static intCONFIG_THTTPD_TILDE_MAP2(httpd_conn *hc)
{
  static char *temp;
  static size_t maxtemp = 0;
  static char *postfix =CONFIG_THTTPD_TILDE_MAP2;
  char *cp;
  struct passwd *pw;
  char *alt;
  char *rest;

  /* Get the username. */

  httpd_realloc_str(&temp, &maxtemp, strlen(hc->expnfilename) - 1);
  (void)strcpy(temp, &hc->expnfilename[1]);

  cp = strchr(temp, '/');
  if (cp)
    {
      *cp++ = '\0';
    }
  else
    {
      cp = "";
    }

  /* Get the passwd entry. */

  pw = getpwnam(temp);
  if (!pw)
    {
      return 0;
    }

  /* Set up altdir. */

  httpd_realloc_str(&hc->altdir, &hc->maxaltdir, strlen(pw->pw_dir) + 1 + strlen(postfix));
  (void)strcpy(hc->altdir, pw->pw_dir);
  if (postfix[0] != '\0')
    {
      (void)strcat(hc->altdir, "/");
      (void)strcat(hc->altdir, postfix);
    }

  alt = expand_filename(hc->altdir, &rest, TRUE);
  if (rest[0] != '\0')
    {
     return 0;
    }

  httpd_realloc_str(&hc->altdir, &hc->maxaltdir, strlen(alt));
  (void)strcpy(hc->altdir, alt);

  /* And the filename becomes altdir plus the post-~ part of the original. */

  httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename, strlen(hc->altdir) + 1 + strlen(cp));
  (void)snprintf(hc->expnfilename, hc->maxexpnfilename, "%s/%s", hc->altdir, cp);

  /* For this type of tilde mapping, we want to defeat vhost mapping. */

  hc->tildemapped = TRUE;
  return 1;
}
#endif

/* Virtual host mapping. */

#ifdef CONFIG_THTTPD_VHOST
static int vhost_map(httpd_conn *hc)
{
  httpd_sockaddr sa;
  socklen_t sz;
  static char *tempfilename;
  static size_t maxtempfilename = 0;
  char *cp1;
  int len;
#ifdef VHOST_DIRLEVELS
  int i;
  char *cp2;
#endif

  /* Figure out the virtual hostname. */

  if (hc->reqhost[0] != '\0')
    {
      hc->vhostname = hc->reqhost;
    }
  else if (hc->hdrhost[0] != '\0')
    {
      hc->vhostname = hc->hdrhost;
    }
  else
    {
      sz = sizeof(sa);
      if (getsockname(hc->conn_fd, &sa.sa, &sz) < 0)
        {
          ndbg("getsockname: %d\n", errno);
          return 0;
        }
      hc->vhostname = httpd_ntoa(&sa);
    }

  /* Pound it to lower case. */

  for (cp1 = hc->vhostname; *cp1 != '\0'; ++cp1)
    {
      if (isupper(*cp1))
        {
          *cp1 = tolower(*cp1);
        }
    }

  if (hc->tildemapped)
    {
      return 1;
    }

  /* Figure out the host directory. */

#ifdef VHOST_DIRLEVELS
  httpd_realloc_str(&hc->hostdir, &hc->maxhostdir, strlen(hc->vhostname) + 2 * VHOST_DIRLEVELS);
  if (strncmp(hc->vhostname, "www.", 4) == 0)
    {
      cp1 = &hc->vhostname[4];
    }
  else
    {
      cp1 = hc->vhostname;
    }

  for (cp2 = hc->hostdir, i = 0; i < VHOST_DIRLEVELS; ++i)
    {
      /* Skip dots in the hostname.  If we don't, then we get vhost
       * directories in higher level of filestructure if dot gets involved
       * into path construction.  It's `while' used here instead of `if' for 
       * it's possible to have a hostname formed with two dots at the end of 
       * it.
       */

      while (*cp1 == '.')
        {
          ++cp1;
        }

      /* Copy a character from the hostname, or '_' if we ran out. */

      if (*cp1 != '\0')
        {
          *cp2++ = *cp1++;
        }
      else
        {
          *cp2++ = '_';
        }

      /* Copy a slash. */

      *cp2++ = '/';
    }
  (void)strcpy(cp2, hc->vhostname);
#else /* VHOST_DIRLEVELS */
  httpd_realloc_str(&hc->hostdir, &hc->maxhostdir, strlen(hc->vhostname));
  (void)strcpy(hc->hostdir, hc->vhostname);
#endif                                 /* VHOST_DIRLEVELS */

  /* Prepend hostdir to the filename. */

  len = strlen(hc->expnfilename);
  httpd_realloc_str(&tempfilename, &maxtempfilename, len);
  (void)strcpy(tempfilename, hc->expnfilename);
  httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename, strlen(hc->hostdir) + 1 + len);
  (void)strcpy(hc->expnfilename, hc->hostdir);
  (void)strcat(hc->expnfilename, "/");
  (void)strcat(hc->expnfilename, tempfilename);
  return 1;
}
#endif

/* Expands filename, deleting ..'s and leading /'s.
 * Returns the expanded path (pointer to static string), or NULL on
 * errors.  Also returns, in the string pointed to by restP, any trailing
 * parts of the path that don't exist.
 */

static char *expand_filename(char *path, char **restP, boolean tildemapped)
{
  static char *checked;
  static char *rest;
  static size_t maxchecked = 0, maxrest = 0;
  size_t checkedlen, restlen, prevcheckedlen, prevrestlen;
#if 0 // REVISIT
  struct stat sb;
#endif
  int nlinks, i;
  char *r;
  char *cp1;
  char *cp2;

  nvdbg("path: \"%s\"\n", path);
#if 0 // REVISIT
  /* We need to do the pathinfo check.  we do a single stat() of the whole
   * filename - if it exists, then we return it as is with nothing in restP.
   * If it doesn't exist, we fall through to the existing code.
   */

  if (stat(path, &sb) != -1)
    {
      checkedlen = strlen(path);
      httpd_realloc_str(&checked, &maxchecked, checkedlen);
      (void)strcpy(checked, path);

      /* Trim trailing slashes. */

      while (checked[checkedlen - 1] == '/')
        {
          checked[checkedlen - 1] = '\0';
          --checkedlen;
        }

      httpd_realloc_str(&rest, &maxrest, 0);
      rest[0] = '\0';
      *restP = rest;
      return checked;
    }
#endif

  /* Handle leading /, or . or by copying the default directory into checked */

  if (path[0] == '.' || (path[0] == '/' && strncmp(path, httpd_root, strlen(httpd_root)) != 0))
    {
      /* Start out with httpd_root in checked */
 
      checkedlen = strlen(httpd_root) + 1;
      httpd_realloc_str(&checked, &maxchecked, checkedlen);
      strcpy(checked, httpd_root);

      /* Skip over leading '.' */

      if (path[0] == '.')
        {
          path++;
        }
    }
  else
    {
      /* Start out with nothing in checked */

      httpd_realloc_str(&checked, &maxchecked, 1);
      checked[0] = '\0';
      checkedlen = 0;
    }       

  /* Copy the whole filename (minus the leading '.') into rest. */

  restlen = strlen(path);
  httpd_realloc_str(&rest, &maxrest, restlen);
  (void)strcpy(rest, path);

  /* trim trailing slash */

  if (rest[restlen - 1] == '/')
    {
      rest[--restlen] = '\0';
    }

  r = rest;
  nlinks = 0;

  /* While there are still components to check... */

  while (restlen > 0)
    {
      /* Save current checkedlen.  Save current restlen in case we get a non-existant component. */

      prevcheckedlen = checkedlen;
      prevrestlen    = restlen;

      /* Grab one component from r and transfer it to checked. */

      cp1 = strchr(r, '/');
      if (cp1)
        {
          i = cp1 - r;
          if (i == 0)
            {
              /* Special case for absolute paths. */

              httpd_realloc_str(&checked, &maxchecked, checkedlen + 1);
              (void)strncpy(&checked[checkedlen], r, 1);
              checkedlen += 1;
            }
          else if (strncmp(r, "..", MAX(i, 2)) == 0)
            {
              /* Ignore ..'s that go above the start of the path. */

              if (checkedlen != 0)
                {
                  cp2 = strrchr(checked, '/');
                  if (!cp2)
                    {
                      checkedlen = 0;
                    }
                  else if (cp2 == checked)
                    {
                      checkedlen = 1;
                    }
                  else
                    {
                      checkedlen = cp2 - checked;
                    }
                }
            }
          else
            {
              httpd_realloc_str(&checked, &maxchecked, checkedlen + 1 + i);
              if (checkedlen > 0 && checked[checkedlen - 1] != '/')
                {
                  checked[checkedlen++] = '/';
                }

              (void)strncpy(&checked[checkedlen], r, i);
              checkedlen += i;
            }

          checked[checkedlen] = '\0';
          r += i + 1;
          restlen -= i + 1;
        }
      else
        {
          /* No slashes remaining, r is all one component. */

          if (strcmp(r, "..") == 0)
            {
              /* Ignore ..'s that go above the start of the path. */

              if (checkedlen != 0)
                {
                  cp2 = strrchr(checked, '/');
                  if (!cp2)
                    {
                      checkedlen = 0;
                    }
                  else if (cp2 == checked)
                    {
                      checkedlen = 1;
                    }
                  else
                    {
                      checkedlen = cp2 - checked;
                    }

                  checked[checkedlen] = '\0';
                }
            }
          else
            {
              httpd_realloc_str(&checked, &maxchecked, checkedlen + 1 + restlen);
              if (checkedlen > 0 && checked[checkedlen - 1] != '/')
                {
                  checked[checkedlen++] = '/';
                }

              (void)strcpy(&checked[checkedlen], r);
              checkedlen += restlen;
            }

          r += restlen;
          restlen = 0;
        }
    }

  /* Ok. */

  *restP = r;
  if (checked[0] == '\0')
    {
      (void)strcpy(checked, httpd_root);
    }

  nvdbg("checked: \"%s\"\n", checked);
  return checked;
}

static char *bufgets(httpd_conn *hc)
{
  int i;
  char c;

  for (i = hc->checked_idx; hc->checked_idx < hc->read_idx; ++hc->checked_idx)
    {
      c = hc->read_buf[hc->checked_idx];
      if (c == '\012' || c == '\015')
        {
          hc->read_buf[hc->checked_idx] = '\0';
          ++hc->checked_idx;
          if (c == '\015' && hc->checked_idx < hc->read_idx &&
              hc->read_buf[hc->checked_idx] == '\012')
            {
              hc->read_buf[hc->checked_idx] = '\0';
              ++hc->checked_idx;
            }
          return &(hc->read_buf[i]);
        }
    }
  return NULL;
}

static void de_dotdot(char *file)
{
  char *cp;
  char *cp2;
  int l;

  /* Collapse any multiple / sequences. */

  while ((cp = strstr(file, "//")) != NULL)
    {
      for (cp2 = cp + 2; *cp2 == '/'; ++cp2)
        {
          continue;
        }

      (void)strcpy(cp + 1, cp2);
    }

  /* Remove leading ./ and any /./ sequences. */

  while (strncmp(file, "./", 2) == 0)
    {
      (void)strcpy(file, file + 2);
    }

  while ((cp = strstr(file, "/./")) != NULL)
    {
    (void)strcpy(cp, cp + 2);
    }

  /* Alternate between removing leading ../ and removing xxx/../ */

  for (;;)
    {
      while (strncmp(file, "../", 3) == 0)
        {
          (void)strcpy(file, file + 3);
        }

      cp = strstr(file, "/../");
      if (!cp)
        {
          break;
        }

      for (cp2 = cp - 1; cp2 >= file && *cp2 != '/'; --cp2)
        {
          continue;
        }

      (void)strcpy(cp2 + 1, cp + 4);
    }

  /* Also elide any xxx/.. at the end. */

  while ((l = strlen(file)) > 3 && strcmp((cp = file + l - 3), "/..") == 0)
    {
      for (cp2 = cp - 1; cp2 >= file && *cp2 != '/'; --cp2)
        {
          continue;
        }

      if (cp2 < file)
        {
          break;
        }

      *cp2 = '\0';
    }
}

static void init_mime(void)
{
  int i;

  /* Fill in the lengths. */

  for (i = 0; i < n_enc_tab; ++i)
    {
      enc_tab[i].ext_len = strlen(enc_tab[i].ext);
      enc_tab[i].val_len = strlen(enc_tab[i].val);
    }

  for (i = 0; i < n_typ_tab; ++i)
    {
      typ_tab[i].ext_len = strlen(typ_tab[i].ext);
      typ_tab[i].val_len = strlen(typ_tab[i].val);
    }
}

/* Figure out MIME encodings and type based on the filename.  Multiple
 * encodings are separated by commas, and are listed in the order in
 * which they were applied to the file.
 */

static void figure_mime(httpd_conn *hc)
{
  char *prev_dot;
  char *dot;
  char *ext;
  int me_indexes[100], n_me_indexes;
  size_t ext_len, encodings_len;
  int i, top, bot, mid;
  int r;
  char *default_type = "text/plain; charset=%s";

  /* Peel off encoding extensions until there aren't any more. */

  n_me_indexes = 0;
  for (prev_dot = &hc->expnfilename[strlen(hc->expnfilename)];; prev_dot = dot)
    {
      for (dot = prev_dot - 1; dot >= hc->expnfilename && *dot != '.'; --dot)
        ;

      if (dot < hc->expnfilename)
        {
          /* No dot found.  No more encoding extensions, and no type
           * extension either.
           */

          hc->type = default_type;
          goto done;
        }

      ext = dot + 1;
      ext_len = prev_dot - ext;

      /* Search the encodings table.  Linear search is fine here, there are
       * only a few entries.
       */

      for (i = 0; i < n_enc_tab; ++i)
        {
          if (ext_len == enc_tab[i].ext_len &&
              strncasecmp(ext, enc_tab[i].ext, ext_len) == 0)
            {
              if (n_me_indexes < sizeof(me_indexes) / sizeof(*me_indexes))
                {
                  me_indexes[n_me_indexes] = i;
                  ++n_me_indexes;
                }
              goto next;
            }
        }

      /* No encoding extension found.  Break and look for a type extension. */

      break;

    next:;
    }

  /* Binary search for a matching type extension. */

  top = n_typ_tab - 1;
  bot = 0;
  while (top >= bot)
    {
      mid = (top + bot) / 2;
      r = strncasecmp(ext, typ_tab[mid].ext, ext_len);
      if (r < 0)
        {
          top = mid - 1;
        }
      else if (r > 0)
        {
          bot = mid + 1;
        }
      else if (ext_len < typ_tab[mid].ext_len)
        {
          top = mid - 1;
        }
      else if (ext_len > typ_tab[mid].ext_len)
        {
          bot = mid + 1;
        }
      else
        {
          hc->type = typ_tab[mid].val;
          goto done;
        }
    }
  hc->type = default_type;

done:

  /* The last thing we do is actually generate the mime-encoding header. */

  hc->encodings[0] = '\0';
  encodings_len = 0;
  for (i = n_me_indexes - 1; i >= 0; --i)
    {
      httpd_realloc_str(&hc->encodings, &hc->maxencodings,
                        encodings_len + enc_tab[me_indexes[i]].val_len + 1);
      if (hc->encodings[0] != '\0')
        {
          (void)strcpy(&hc->encodings[encodings_len], ",");
          ++encodings_len;
        }

      (void)strcpy(&hc->encodings[encodings_len], enc_tab[me_indexes[i]].val);
      encodings_len += enc_tab[me_indexes[i]].val_len;
    }
}

#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
static void cgi_kill2(ClientData client_data, struct timeval *nowP)
{
  pid_t pid;

  pid = (pid_t) client_data.i;
  if (kill(pid, SIGKILL) == 0)
    {
      ndbg("hard-killed CGI process %d\n", pid);
    }
}

static void cgi_kill(ClientData client_data, struct timeval *nowP)
{
  pid_t pid;

  pid = (pid_t) client_data.i;
  if (kill(pid, SIGINT) == 0)
    {
      ndbg("killed CGI process %d\n", pid);

      /* In case this isn't enough, schedule an uncatchable kill. */

      if (tmr_create(nowP, cgi_kill2, client_data, 5 * 1000L, 0) == NULL)
        {
          ndbg("tmr_create(cgi_kill2) failed\n");
          exit(1);
        }
    }
}
#endif

/* qsort comparison routine. */

#ifdef CONFIG_THTTPD_GENERATE_INDICES
static int name_compare(char **a, char **b)
{
  return strcmp(*a, *b);
}

static void ls_child(int argc, char **argv)
{
  FAR httpd_conn *hc = (FAR httpd_conn*)strtoul(argv[1], NULL, 16);
  DIR *dirp;
  struct dirent *de;
  int namlen;
  static int maxnames = 0;
  int nnames;
  static char *names;
  static char **nameptrs;
  static char *name;
  static size_t maxname = 0;
  static char *rname;
  static size_t maxrname = 0;
  static char *encrname;
  static size_t maxencrname = 0;
  FILE *fp;
  int i, r;
  struct stat sb;
  struct stat lsb;
  char modestr[20];
  char *linkprefix;
  char link[MAXPATHLEN + 1];
  char *fileclass;
  time_t now;
  char *timestr;
  ClientData client_data;

  httpd_unlisten(hc->hs);
  send_mime(hc, 200, ok200title, "", "", "text/html; charset=%s",
            (off_t) - 1, hc->sb.st_mtime);
  httpd_write_response(hc);

  /* Open a stdio stream so that we can use fprintf, which is more
   * efficient than a bunch of separate write()s.  We don't have to
   * worry about double closes or file descriptor leaks cause we're
   * in a subprocess.
   */

  fp = fdopen(hc->conn_fd, "w");
  if (fp == NULL)
    {
      ndbg("fdopen: %d\n", errno);
      INTERNALERROR("fdopen");
      httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
      httpd_write_response(hc);
      closedir(dirp);
      exit(1);
    }

  fputs(html_html, fp);
  fputs(html_hdtitle, fp);
  (void)fprintf(fp, "Index of %s", hc->encodedurl, hc->encodedurl);
  fputs(html_titlehd, fp);
  fputs(html_body, fp);
  fputs(html_hdr2, fp);
  (void)fprintf(fp, "Index of %s", hc->encodedurl, hc->encodedurl);
  fputs(html_endhdr2, fp);
  fputs(html_crlf, fp);
  fputs("<PRE>\r\nmode  links  bytes  last-changed  name\r\n<HR>", fp);

  /* Read in names. */

  nnames = 0;
  while ((de = readdir(dirp)) != 0)     /* dirent or direct */
    {
      if (nnames >= maxnames)
        {
          if (maxnames == 0)
            {
              maxnames = 100;
              names    = NEW(char, maxnames * (MAXPATHLEN + 1));
              nameptrs = NEW(char *, maxnames);
            }
          else
            {
              maxnames *= 2;
              names     = RENEW(names, char, maxnames * (MAXPATHLEN + 1));
              nameptrs  = RENEW(nameptrs, char *, maxnames);
            }

          if (!names || !nameptrs)
            {
              ndbg("out of memory reallocating directory names\n");
              exit(1);
            }

          for (i = 0; i < maxnames; ++i)
            {
              nameptrs[i] = &names[i * (MAXPATHLEN + 1)];
            }
        }

      namlen = NAMLEN(de);
      (void)strncpy(nameptrs[nnames], de->d_name, namlen);
      nameptrs[nnames][namlen] = '\0';
      ++nnames;
    }
  closedir(dirp);

  /* Sort the names. */

  qsort(nameptrs, nnames, sizeof(*nameptrs), name_compare);

  /* Generate output. */

  for (i = 0; i < nnames; ++i)
    {
      httpd_realloc_str(&name, &maxname,
                        strlen(hc->expnfilename) + 1 +
                        strlen(nameptrs[i]));
      httpd_realloc_str(&rname, &maxrname,
                        strlen(hc->origfilename) + 1 +
                        strlen(nameptrs[i]));

      if (hc->expnfilename[0] == '\0' || strcmp(hc->expnfilename, ".") == 0)
        {
          (void)strcpy(name, nameptrs[i]);
          (void)strcpy(rname, nameptrs[i]);
        }
      else
        {
          (void)snprintf(name, maxname, "%s/%s", hc->expnfilename, nameptrs[i]);
          if (strcmp(hc->origfilename, ".") == 0)
            {
              (void)snprintf(rname, maxrname, "%s", nameptrs[i]);
            }
          else
            {
              (void)snprintf(rname, maxrname, "%s%s", hc->origfilename, nameptrs[i]);
            }
        }

      httpd_realloc_str(&encrname, &maxencrname, 3 * strlen(rname) + 1);
      strencode(encrname, maxencrname, rname);

      if (stat(name, &sb) < 0 || lstat(name, &lsb) < 0)
        {
          continue;
        }

      linkprefix = "";
      link[0] = '\0';

      /* Break down mode word.  First the file type. */

      switch (lsb.st_mode & S_IFMT)
        {
        case S_IFIFO:
          modestr[0] = 'p';
          break;

        case S_IFCHR:
          modestr[0] = 'c';
          break;

        case S_IFDIR:
          modestr[0] = 'd';
          break;

        case S_IFBLK:
          modestr[0] = 'b';
          break;

        case S_IFREG:
          modestr[0] = '-';
          break;

        case S_IFSOCK:
          modestr[0] = 's';
          break;

        case S_IFLNK:
        default:
          modestr[0] = '?';
          break;
        }

      /* Now the world permissions.  Owner and group permissions are 
       * not of interest to web clients.
       */

      modestr[1] = (lsb.st_mode & S_IROTH) ? 'r' : '-';
      modestr[2] = (lsb.st_mode & S_IWOTH) ? 'w' : '-';
      modestr[3] = (lsb.st_mode & S_IXOTH) ? 'x' : '-';
      modestr[4] = '\0';

      /* We also leave out the owner and group name */

      /* Get time string. */

      now = time(NULL);
      timestr = ctime(&lsb.st_mtime);
      timestr[0] = timestr[4];
      timestr[1] = timestr[5];
      timestr[2] = timestr[6];
      timestr[3] = ' ';
      timestr[4] = timestr[8];
      timestr[5] = timestr[9];
      timestr[6] = ' ';

      if (now - lsb.st_mtime > 60 * 60 * 24 * 182)      /* 1/2 year */
        {
          timestr[7] = ' ';
          timestr[8] = timestr[20];
          timestr[9] = timestr[21];
          timestr[10] = timestr[22];
          timestr[11] = timestr[23];
        }
      else
        {
          timestr[7] = timestr[11];
          timestr[8] = timestr[12];
          timestr[9] = ':';
          timestr[10] = timestr[14];
          timestr[11] = timestr[15];
        }
      timestr[12] = '\0';

      /* The ls -F file class. */

      switch (sb.st_mode & S_IFMT)
        {
        case S_IFDIR:
          fileclass = "/";
          break;

        case S_IFSOCK:
          fileclass = "=";
          break;

        case S_IFLNK:
          fileclass = "@";
          break;

        default:
          fileclass = (sb.st_mode & S_IXOTH) ? "*" : "";
          break;
        }

      /* And print. */

      (void)fprintf(fp, "%s %3ld  %10lld  %s  <A HREF=\"/%.500s%s\">%s</A>%s%s%s\n",
                    modestr, (long)lsb.st_nlink, (sint16) lsb.st_size,
                    timestr, encrname, S_ISDIR(sb.st_mode) ? "/" : "",
                    nameptrs[i], linkprefix, link, fileclass);
    }

  fputs("</PRE>", fp);
  fputs(html_endbody, fp);
  fputs(html_endhtml, fp);
  (void)fclose(fp);
  exit(0);
}

static int ls(httpd_conn *hc)
{
  DIR *dirp;
  struct dirent *de;
  int namlen;
  static int maxnames = 0;
  int nnames;
  static char *names;
  static char **nameptrs;
  static char *name;
  static size_t maxname = 0;
  static char *rname;
  static size_t maxrname = 0;
  static char *encrname;
  static size_t maxencrname = 0;
  FILE *fp;
  int i, child;
  struct stat sb;
  struct stat lsb;
  char modestr[20];
  char *linkprefix;
  char link[MAXPATHLEN + 1];
  char *fileclass;
  time_t now;
  char *timestr;
  char arg[16];
  char *argv[1];
#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
  ClientData client_data;
#endif

  dirp = opendir(hc->expnfilename);
  if (dirp == NULL)
    {
      ndbg("opendir %s: %d\n", hc->expnfilename, errno);
      httpd_send_err(hc, 404, err404title, "", err404form, hc->encodedurl);
      return -1;
    }

  if (hc->method == METHOD_HEAD)
    {
      closedir(dirp);
      send_mime(hc, 200, ok200title, "", "", "text/html; charset=%s",
                (off_t) - 1, hc->sb.st_mtime);
    }
  else if (hc->method == METHOD_GET)
    {
#ifdef CONFIG_THTTPD_CGILIMIT
      if (hc->hs->cgi_count >= CONFIG_THTTPD_CGILIMIT)
        {
          closedir(dirp);
          httpd_send_err(hc, 503, httpd_err503title, "", httpd_err503form,
                         hc->encodedurl);
          return -1;
        }
#endif
      ++hc->hs->cgi_count;

      /* Start the child task. */

      snprintf(arg, 16, "%p", hc); /* task_create doesn't handle binary arguments. */
      argv[0] = arg;

#ifndef CONFIG_CUSTOM_STACK
      child = task_create("CGI child", CONFIG_THTTPD_CGI_PRIORITY,
                          CONFIG_THTTPD_CGI_STACKSIZE,
                          (main_t)ls_child, (const char **)argv);
#else
      child = task_create("CGI child", CONFIG_THTTPD_CGI_PRIORITY,
                          (main_t)ls_child, (const char **)argv);
#endif
      if (child < 0)
        {
          ndbg("task_create: %d\n", errno);
          closedir(dirp);
          INTERNALERROR("task_create");
          httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
          return -1;
        }

      closedir(dirp);
      ndbg("spawned indexing process %d for directory '%s'\n", child, hc->expnfilename);

      /* Schedule a kill for the child process, in case it runs too long */

#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
      client_data.i = child;
      if (tmr_create(NULL, cgi_kill, client_data, CONFIG_THTTPD_CGI_TIMELIMIT * 1000L, 0) == NULL)
        {
          ndbg("tmr_create(cgi_kill ls) failed\n");
          exit(1);
        }
#endif

      hc->status = 200;
      hc->bytes_sent = CONFIG_THTTPD_CGI_BYTECOUNT;
      hc->should_linger = FALSE;
    }
  else
    {
      closedir(dirp);
      NOTIMPLEMENTED(httpd_method_str(hc->method));
      httpd_send_err(hc, 501, err501title, "", err501form, httpd_method_str(hc->method));
      return -1;
    }

  return 0;
}
#endif /* CONFIG_THTTPD_GENERATE_INDICES */

/* Set up environment variables. Be real careful here to avoid
 * letting malicious clients overrun a buffer.  We don't have
 * to worry about freeing stuff since we're a sub-process.
 */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static void create_environment(httpd_conn *hc)
{
  char *cp;
  char buf[256];

  setenv("PATH", CONFIG_THTTPD_CGI_PATH, TRUE);
#ifdef CGI_LD_LIBRARY_PATH
  setenv("LD_LIBRARY_PATH", CGI_LD_LIBRARY_PATH, TRUE);
#endif /* CGI_LD_LIBRARY_PATH */

  setenv("SERVER_SOFTWARE",  CONFIG_THTTPD_SERVER_SOFTWARE, TRUE);

  /* If vhosting, use that server-name here. */
#ifdef CONFIG_THTTPD_VHOST
  if (hc->vhostname)
    {
      cp = hc->vhostname;
    }
  else
#endif
    {
      cp = hc->hs->hostname;
    }

  if (cp)
    {
      setenv("SERVER_NAME", cp, TRUE);
    }

  setenv("GATEWAY_INTERFACE", "CGI/1.1", TRUE);
  setenv("SERVER_PROTOCOL", hc->protocol, TRUE);

  (void)snprintf(buf, sizeof(buf), "%d", (int)CONFIG_THTTPD_PORT);
  setenv("SERVER_PORT", buf, TRUE);

  setenv("REQUEST_METHOD", httpd_method_str(hc->method), TRUE);

  if (hc->pathinfo[0] != '\0')
    {
      char *cp2;
      size_t l;

      (void)snprintf(buf, sizeof(buf), "/%s", hc->pathinfo);
      setenv("PATH_INFO", buf, TRUE);

      l = strlen(httpd_root) + strlen(hc->pathinfo) + 1;
      cp2 = NEW(char, l);
      if (cp2)
        {
          (void)snprintf(cp2, l, "%s%s", httpd_root, hc->pathinfo);
          setenv("PATH_TRANSLATED", cp2, TRUE);
        }
    }

  (void)snprintf(buf, sizeof(buf), "/%s",strcmp(hc->origfilename, ".") == 0 ? "" : hc->origfilename);
  setenv("SCRIPT_NAME", buf, TRUE);

  if (hc->query[0] != '\0')
    {
      setenv("QUERY_STRING", hc->query, TRUE);
    }

  setenv("REMOTE_ADDR", httpd_ntoa(&hc->client_addr), TRUE);
  if (hc->referer[0] != '\0')
    {
      setenv("HTTP_REFERER", hc->referer, TRUE);
    }

  if (hc->useragent[0] != '\0')
    {
      setenv("HTTP_USER_AGENT", hc->useragent, TRUE);
    }

  if (hc->accept[0] != '\0')
    {
      setenv("HTTP_ACCEPT", hc->accept, TRUE);
    }

  if (hc->accepte[0] != '\0')
    {
      setenv("HTTP_ACCEPT_ENCODING", hc->accepte, TRUE);
    }

  if (hc->acceptl[0] != '\0')
    {
      setenv("HTTP_ACCEPT_LANGUAGE", hc->acceptl, TRUE);
    }

  if (hc->cookie[0] != '\0')
    {
      setenv("HTTP_COOKIE", hc->cookie, TRUE);
    }

  if (hc->contenttype[0] != '\0')
    {
      setenv("CONTENT_TYPE", hc->contenttype, TRUE);
    }

  if (hc->hdrhost[0] != '\0')
    {
      setenv("HTTP_HOST", hc->hdrhost, TRUE);
    }

  if (hc->contentlength != -1)
    {
      (void)snprintf(buf, sizeof(buf), "%lu", (unsigned long)hc->contentlength);
      setenv("CONTENT_LENGTH", buf, TRUE);
    }

  if (hc->remoteuser[0] != '\0')
    {
      setenv("REMOTE_USER", hc->remoteuser, TRUE);
    }

  if (hc->authorization[0] != '\0')
    {
      setenv("AUTH_TYPE", "Basic", TRUE);
    }

  /* We only support Basic auth at the moment. */

  if (getenv("TZ") != NULL)
    {
      setenv("TZ", getenv("TZ"), TRUE);
    }

  setenv("CGI_PATTERN", CONFIG_THTTPD_CGI_PATTERN, TRUE);
}
#endif

/* Set up argument vector */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static FAR char **make_argp(httpd_conn *hc)
{
  FAR char **argp;
  int argn;
  char *cp1;
  char *cp2;

  /* By allocating an arg slot for every character in the query, plus one
   * for the filename and one for the NULL, we are guaranteed to have
   * enough.  We could actually use strlen/2.
   */

  argp = NEW(char *, strlen(hc->query) + 2);
  if (!argp)
    {
      return NULL;
    }

  argp[0] = strrchr(hc->expnfilename, '/');
  if (argp[0])
    {
      ++argp[0];
    }
  else
    {
      argp[0] = hc->expnfilename;
    }
  argn = 1;

  /* According to the CGI spec at http://hoohoo.ncsa.uiuc.edu/cgi/cl.html,
   * "The server should search the query information for a non-encoded =
   * character to determine if the command line is to be used, if it finds
   * one, the command line is not to be used."
   */

  if (strchr(hc->query, '=') == NULL)
    {
      for (cp1 = cp2 = hc->query; *cp2 != '\0'; ++cp2)
        {
          if (*cp2 == '+')
            {
              *cp2 = '\0';
              strdecode(cp1, cp1);
              argp[argn++] = cp1;
              cp1 = cp2 + 1;
            }
        }

      if (cp2 != cp1)
        {
          strdecode(cp1, cp1);
          argp[argn++] = cp1;
        }
    }

  argp[argn] = NULL;
  return argp;
}
#endif

/* Data is available from the client socket. This routine is used only for POST
 * requests.  It reads the data from the client and sends it to the child thread.
 */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static inline int cgi_interpose_input(httpd_conn *hc, int wfd, char *buffer)
{
  size_t nbytes;
  ssize_t nbytes_read;
  ssize_t nbytes_written;

  nbytes = hc->read_idx - hc->checked_idx;
  if (nbytes > 0)
    {
      if (httpd_write(wfd, &(hc->read_buf[hc->checked_idx]), nbytes) != nbytes)
        {
          return 1;
        }
    }

  if (nbytes < hc->contentlength)
    {
      do
        {
          nbytes_read = read(hc->conn_fd, buffer, MIN(sizeof(buffer), hc->contentlength - nbytes));
          if (nbytes_read < 0)
            {
              if (errno != EINTR)
                {
                  return 1;
                }
            }
        }
      while (nbytes_read < 0);

      if (nbytes_read > 0)
        {
          nbytes_written = httpd_write(wfd, buffer, nbytes_read);
          if (nbytes_written != nbytes_read)
            {
              return 1;
            }
        }
    }

  if (nbytes >= hc->contentlength)
    {
      /* Special hack to deal with broken browsers that send a LF or CRLF
       * after POST data, causing TCP resets - we just read and discard up
       * to 2 bytes.  Unfortunately this doesn't fix the problem for CGIs
       * which avoid the interposer process due to their POST data being
       * short.  Creating an interposer process for all POST CGIs is
       * unacceptably expensive.  The eventual fix will come when interposing
       * gets integrated into the main loop as a tasklet instead of a process.
       */

      /* Turn on no-delay mode in case we previously cleared it. */

      httpd_set_ndelay(hc->conn_fd);

      /* And read up to 2 bytes. */

      (void)read(hc->conn_fd, buffer, sizeof(buffer));
      return 1;
    }
  return 0;
}
#endif

/* This routine is used for parsed-header CGIs.  The idea here is that the
 * CGI can return special headers such as "Status:" and "Location:" which
 * change the return status of the response.  Since the return status has to
 * be the very first line written out, we have to accumulate all the headers
 * and check for the special ones before writing the status.  Then we write
 * out the saved headers and proceed to echo the rest of the response.
 */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static inline int cgi_interpose_output(httpd_conn *hc, int rfd, char *inbuffer,
                                          struct cgi_outbuffer_s *hdr)
{
  ssize_t nbytes_read;
  char *br = NULL;
  int status;
  const char *title;
  char *cp;

  /* Make sure the connection is in blocking mode.  It should already be
   * blocking, but we might as well be sure.
   */

  httpd_clear_ndelay(hc->conn_fd);

  /* Loop while there are things we can do without waiting for more input */

  switch (hdr->state)
    {
      case CGI_OUTBUFFER_READHEADER:
        {
          /* Slurp in all headers as they become available from the client. */

          do
            {
              /* Read until we successfully read data or until an error occurs.
               * EAGAIN is not an error, but it is still cause to return.
               */

              nbytes_read = read(hc->conn_fd, inbuffer, sizeof(inbuffer));
              nvdbg("Read %d bytes from fd %d\n", nbytes_read, hc->conn_fd);

              if (nbytes_read < 0)
                {
                  if (errno != EINTR)
                    {
                      if (errno != EAGAIN)
                        {
                          ndbg("read: %d\n", errno);
                        }
                      return 1;
                    }
                }
            }
          while (nbytes_read < 0);

          /* Check for end-of-file */

          if (nbytes_read <= 0)
            {
              nvdbg("End-of-file\n");
              br         = &(hdr->buffer[hdr->len]);
              hdr->state = CGI_OUTBUFFER_HEADERREAD;
            }
          else
            {
              /* Accumulate more header data */

              httpd_realloc_str(&hdr->buffer, &hdr->size, hdr->len + nbytes_read);
              (void)memcpy(&(hdr->buffer[hdr->len]), inbuffer, nbytes_read);
              hdr->len += nbytes_read;
              hdr->buffer[hdr->len] = '\0';
              nvdbg("Header bytes accumulated: %d\n", hdr->len);

              /* Check for end of header */

              if ((br = strstr(hdr->buffer, "\r\n\r\n")) != NULL ||
                  (br = strstr(hdr->buffer, "\012\012")) != NULL)
                {
                  nvdbg("End-of-header\n");
                  hdr->state = CGI_OUTBUFFER_HEADERREAD;
                }
              else
                {
                  /* Return.  We will be called again when more data is available
                   * on the socket.
                   */

                  return 0;
                }
            }
        }
        break;

      case CGI_OUTBUFFER_HEADERREAD:
        {
          /* If there were no headers, bail. */

          if (hdr->buffer[0] == '\0')
            {
              hdr->state = CGI_OUTBUFFER_DONE;
              return 1;
            }

          /* Figure out the status.  Look for a Status: or Location: header; else if 
           * there's an HTTP header line, get it from there; else default to 200.
           */

          status = 200;
          if (strncmp(hdr->buffer, "HTTP/", 5) == 0)
            {
              cp = hdr->buffer;
              cp += strcspn(cp, " \t");
              status = atoi(cp);
            }

          if ((cp = strstr(hdr->buffer, "Status:")) != NULL &&
              cp < br && (cp == hdr->buffer || *(cp - 1) == '\012'))
            {
              cp += 7;
              cp += strspn(cp, " \t");
              status = atoi(cp);
            }

          if ((cp = strstr(hdr->buffer, "Location:")) != NULL &&
              cp < br && (cp == hdr->buffer || *(cp - 1) == '\012'))
            {
              status = 302;
            }

          /* Write the status line. */

          nvdbg("Status: %d\n", status);
          switch (status)
            {
            case 200:
              title = ok200title;
              break;

            case 302:
              title = err302title;
              break;

            case 304:
              title = err304title;
              break;

            case 400:
              BADREQUEST("status");
              title = httpd_err400title;
              break;

#ifdef CONFIG_THTTPD_AUTH_FILE
            case 401:
              title = err401title;
              break;
#endif

            case 403:
              title = err403title;
              break;

            case 404:
              title = err404title;
             break;

           case 408:
              title = httpd_err408title;
              break;

            case 500:
              INTERNALERROR("status");
              title = err500title;
              break;

            case 501:
              NOTIMPLEMENTED("status");
              title = err501title;
              break;

            case 503:
              title = httpd_err503title;
              break;

            default:
              title = "Something";
              break;
            }

          (void)snprintf(inbuffer, sizeof(inbuffer), "HTTP/1.0 %d %s\r\n", status, title);
          (void)httpd_write(hc->conn_fd, inbuffer, strlen(inbuffer));

          /* Write the saved hdr->buffer to the client. */

          (void)httpd_write(hc->conn_fd, hdr->buffer, hdr->len);
        }

        /* Then set up to read the data following the header from the CGI program and
         * pass it back to the client. We return now; we will be called again when
         * data is available on the pipe.
         */

        hdr->state = CGI_OUTBUFFER_READDATA;
        return 0;

      case CGI_OUTBUFFER_READDATA:
        {
          /* Read data from the pipe. */

          do
            {
              /* Read until we successfully read data or until an error occurs.
               * EAGAIN is not an error, but it is still cause to return.
               */

              nbytes_read = read(rfd, inbuffer, sizeof(inbuffer));
              nvdbg("Read %d bytes from fd %d\n", nbytes_read, rfd);

              if (nbytes_read < 0)
                {
                  if (errno != EINTR)
                    {
                      if (errno != EAGAIN)
                        {
                          ndbg("read: %d\n", errno);
                        }
                      return 1;
                    }
                }
            }
          while (nbytes_read < 0);

          /* Check for end of file */

          if (nbytes_read == 0)
            {
              nvdbg("End-of-file\n");
              close(hc->conn_fd);
              close(rfd);
              hdr->state = CGI_OUTBUFFER_DONE;
              return 1;
            }
          else
            {
               /* Forward the data from the CGI program to the client */

             (void)httpd_write(hc->conn_fd, inbuffer, strlen(inbuffer));
            }
        }
        break;

      case CGI_OUTBUFFER_DONE:
      default:
        return 1;
    }
  return 0;
}
#endif

/* CGI child process. */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static int cgi_child(int argc, char **argv)
{
  FAR httpd_conn *hc = (FAR httpd_conn*)strtoul(argv[1], NULL, 16);
#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
  ClientData client_data;
#endif
  FAR char **argp;
  struct cgi_outbuffer_s hdr;
  struct fdwatch_s *fw;
  char   *buffer;
  char   *binary;
  char   *directory;
  boolean indone;
  boolean outdone;
  int     child;
  int     pipefd[2];
  int     wfd = -1;
  int     rfd = -1;
  int     fd;
  int     ret;
  int     err = 1;

  /* Update all of the environment variable settings, these will be inherited
   * by the CGI task.
   */

  create_environment(hc);

  /* Make the argument vector. */

  argp = make_argp(hc);

  /* Close all file descriptors (including stdio, stdin, stdout) EXCEPT for
   * stderr and hc->conn_fd
   */

  for (fd = 0; fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS); fd++)
    {
       /* Keep stderr open for debug; keep hc->conn_fd open for obvious reasons */

       if (fd != 2 && fd != hc->conn_fd)
         {
           close(fd);
         }
    }

  /* Create pipes that will be interposed between the CGI task's stdin or
   * stdout and the socket.
   *
   *
   * Setup up the STDIN pipe - a pipe to transfer data received on the
   * socket to the CGI program.
   */

  ret = pipe(pipefd);
  if (ret < 0)
    {
      ndbg("STDIN pipe: %d\n", errno);
      goto errout_with_descriptors;
    }
  else
    {
      /* Then map the receiving end the pipe to stdin, save the sending end, and
       * closing the original receiving end
       */

      ret = dup2(pipefd[0], 0);
      if (ret < 0)
        {
          ndbg("STDIN dup2: %d\n", errno);
          close(pipefd[1]);
          goto errout_with_descriptors;
        }

      wfd = pipefd[1];
      close(pipefd[0]);
    }

  /* Set up the STDOUT pipe - a pipe to transfer data received from the CGI program
   * to the client.
   */

  if (ret == 0)
    {
      ret = pipe(pipefd);
      if (ret < 0)
        {
          ndbg("STDOUT pipe: %d\n", errno);
          goto errout_with_descriptors;
        }
      else
        {
          /* Then map the sending end the pipe to stdout, save the receiving end, and
           * closing the original sending end
           */

          ret = dup2(pipefd[1], 1);
          if (ret < 0)
            {
              ndbg("STDOUT dup2: %d\n", errno);
              close(pipefd[1]);
              goto errout_with_descriptors;
             }

          rfd = pipefd[0];
          close(pipefd[1]);
        }
    }

  /* Split the program into directory and binary, so we can chdir() to the
   * program's own directory.  This isn't in the CGI 1.1 spec, but it's what 
   * other HTTP servers do.
   */

  directory = strdup(hc->expnfilename);
  if (!directory)
    {
      binary = hc->expnfilename;  /* ignore errors */
    }
  else
    {
      binary = strrchr(directory, '/');
      if (!binary)
        {
          binary = hc->expnfilename;
        }
      else
        {
          *binary++ = '\0';
          (void)chdir(directory);       /* ignore errors */
        }
    }

  /* Allocate memory for buffering */

  memset(&hdr, 0, sizeof(struct cgi_outbuffer_s));
  httpd_realloc_str(&hdr.buffer, &hdr.size, 500);
  if (!hdr.buffer)
    {
      ndbg("hdr allocation failed\n");
      goto errout_with_descriptors;
    }

  buffer = (char*)malloc(CONFIG_THTTPD_IOBUFFERSIZE);
  if (!buffer)
    {
      ndbg("buffer allocation failed\n");
      goto errout_with_header;
    }

  /* Create fdwatch structures */

  fw = fdwatch_initialize(2);
  if (!fw)
    {
      ndbg("fdwatch allocation failed\n");
      goto errout_with_buffer;
    }

  /* Run the CGI program. */

  child = exec(binary, (FAR const char **)argp, g_thttpdsymtab, g_thttpdnsymbols);
  if (child < 0)
    {
      /* Something went wrong. */

      ndbg("execve %s: %d\n", hc->expnfilename, errno);
      goto errout_with_watch;
   }

  /* Schedule a kill for the child process, in case it runs too long. */

#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
  client_data.i = child;
  if (tmr_create(NULL, cgi_kill, client_data, CONFIG_THTTPD_CGI_TIMELIMIT * 1000L, 0) == NULL)
    {
      ndbg("tmr_create(cgi_kill child) failed\n");
      goto errout_with_watch;
    }
#endif

  /* Add the read descriptors to the watch */

  fdwatch_add_fd(fw, hc->conn_fd, NULL);
  fdwatch_add_fd(fw, rfd, NULL);

  /* Then perform the interposition */

  indone  = FALSE;
  outdone = FALSE;

  do
    {
      (void)fdwatch(fw, 5000);

      if (!indone && fdwatch_check_fd(fw, hc->conn_fd))
        {
          /* Transfer data from the client to the CGI program (POST) */

          indone = cgi_interpose_input(hc, wfd, buffer);
          if (indone)
            {
              fdwatch_del_fd(fw, hc->conn_fd);
            }
        }

      if (fdwatch_check_fd(fw, rfd))
        {
          /* Handle receipt of headers and CGI program response (GET) */

          outdone = cgi_interpose_output(hc, rfd, buffer, &hdr);
        }
  }
  while (!outdone);
  err = 0;

  /* Get rid of watch structures */

errout_with_watch:
  fdwatch_uninitialize(fw);

  /* Free memory */

errout_with_buffer:
  free(buffer);
errout_with_header:
  free(hdr.buffer);

  /* Close all descriptors */

errout_with_descriptors:
  close(wfd);
  close(rfd);
  close(hc->conn_fd);

  INTERNALERROR("errout");
  httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
  httpd_write_response(hc);
  return err;
}
#endif /* CONFIG_THTTPD_CGI_PATTERN */

#ifdef CONFIG_THTTPD_CGI_PATTERN
static int cgi(httpd_conn *hc)
{
  char arg[16];
  char *argv[1];
  pid_t child;

  if (hc->method == METHOD_GET || hc->method == METHOD_POST)
    {
#ifdef CONFIG_THTTPD_CGILIMIT
      if (hc->hs->cgi_count >= CONFIG_THTTPD_CGILIMIT)
        {
          httpd_send_err(hc, 503, httpd_err503title, "", httpd_err503form,
                         hc->encodedurl);
          return -1;
        }
#endif
      ++hc->hs->cgi_count;
      httpd_clear_ndelay(hc->conn_fd);

      /* Start the child task.  We use a trampoline task here so that we can
       * safely muck with the file descriptors before actually started the CGI
       * task.
       */

      snprintf(arg, 16, "%p", hc); /* task_create doesn't handle binary arguments. */
      argv[0] = arg;

#ifndef CONFIG_CUSTOM_STACK
      child = task_create("CGI child", CONFIG_THTTPD_CGI_PRIORITY,
                          CONFIG_THTTPD_CGI_STACKSIZE,
                          (main_t)cgi_child, (const char **)argv);
#else
      child = task_create("CGI child", CONFIG_THTTPD_CGI_PRIORITY,
                          (main_t)cgi_child, (const char **)argv);
#endif
      if (child < 0)
        {
          ndbg("task_create: %d\n", errno);
          INTERNALERROR("task_create");
          httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
          return -1;
        }

      ndbg("started CGI process %d for file '%s'\n", child, hc->expnfilename);

      hc->status        = 200;
      hc->bytes_sent    = CONFIG_THTTPD_CGI_BYTECOUNT;
      hc->should_linger = FALSE;
    }
  else
    {
      NOTIMPLEMENTED("CGI");
      httpd_send_err(hc, 501, err501title, "", err501form, httpd_method_str(hc->method));
      return -1;
    }

  return 0;
}
#endif

/* Returns 1 if ok to serve the url, 0 if not. */

static int check_referer(httpd_conn *hc)
{
  /* Are we doing referer checking at all? */

#ifdef CONFIG_THTTPD_URLPATTERN
  int r;
  char *cp;

  child = really_check_referer(hc);

  if (!r)
    {
#ifdef CONFIG_THTTPD_VHOST
      if (hc->vhostname != NULL)
        {
          cp = hc->vhostname;
        }
      else
#endif
        {
          cp = hc->hs->hostname;
        }

      if (cp == NULL)
        {
          cp = "";
        }

      ndbg("%s non-local referer \"%s%s\" \"%s\"\n",
             httpd_ntoa(&hc->client_addr), cp, hc->encodedurl, hc->referer);
      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "You must supply a local referer to get URL '%s' from this server.\n"),
                     hc->encodedurl);
    }
  return r;
#else
  return 1;
#endif
}

/* Returns 1 if ok to serve the url, 0 if not. */

#ifdef CONFIG_THTTPD_URLPATTERN
static int really_check_referer(httpd_conn *hc)
{
  httpd_server *hs;
  char *cp1;
  char *cp2;
  char *cp3;
  static char *refhost = NULL;
  static size_t refhost_size = 0;
  char *lp;

  hs = hc->hs;

  /* Check for an empty referer. */

  if (hc->referer == NULL || hc->referer[0] == '\0' ||
      (cp1 = strstr(hc->referer, "//")) == NULL)
    {
      /* Disallow if the url matches. */

      if (match(CONFIG_THTTPD_URLPATTERN, hc->origfilename))
        {
          return 0;
        }

      /* Otherwise ok. */

      return 1;
    }

  /* Extract referer host. */

  cp1 += 2;
  for (cp2 = cp1; *cp2 != '/' && *cp2 != ':' && *cp2 != '\0'; ++cp2)
    {
      continue;
    }

  httpd_realloc_str(&refhost, &refhost_size, cp2 - cp1);
  for (cp3 = refhost; cp1 < cp2; ++cp1, ++cp3)
    if (isupper(*cp1))
      {
        *cp3 = tolower(*cp1);
      }
    else
      {
        *cp3 = *cp1;
      }
  *cp3 = '\0';

  /* Local pattern? */

#ifdef CONFIG_THTTPD_LOCALPATTERN
  lp = CONFIG_THTTPD_LOCALPATTERN;
#else

  /* No local pattern.  What's our hostname? */

#ifndef CONFIG_THTTPD_VHOST
  /* Not vhosting, use the server name. */

  lp = hs->hostname;
  if (!lp)
    {
      /* Couldn't figure out local hostname - give up. */

     return 1;
    }

#else
  /* We are vhosting, use the hostname on this connection. */

  lp = hc->vhostname;
  if (!lp)
    {
      /* Oops, no hostname.  Maybe it's an old browser that doesn't
       * send a Host: header.  We could figure out the default
       * hostname for this IP address, but it's not worth it for the
       * few requests like this.
       */

      return 1;
    }
#endif
#endif /* CONFIG_THTTPD_LOCALPATTERN */

  /* If the referer host doesn't match the local host pattern, and the
   * filename does match the url pattern, it's an illegal reference.
   */

#ifdef CONFIG_THTTPD_URLPATTERN
  if (!match(lp, refhost) && match(CONFIG_THTTPD_URLPATTERN, hc->origfilename))
    {
      return 0;
    }
#endif

  /* Otherwise ok. */

  return 1;
}
#endif

#ifdef CONFIG_DEBUG
static int sockaddr_check(httpd_sockaddr *saP)
{
  switch (saP->sin_family)
    {
    case AF_INET:
      return 1;

#ifdef  CONFIG_NET_IPv6
    case AF_INET6:
      return 1;
#endif

    default:
      return 0;
    }
}
#endif

static size_t sockaddr_len(httpd_sockaddr *saP)
{
  switch (saP->sin_family)
    {
    case AF_INET:
      return sizeof(struct sockaddr_in);

#ifdef  CONFIG_NET_IPv6
    case AF_INET6:
      return sizeof(struct sockaddr_in6);
#endif

    default:
      break;
    }
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR httpd_server *httpd_initialize(FAR httpd_sockaddr *sa)
{
  FAR httpd_server *hs;

  /* Save the PID of the main thread */

  main_thread = getpid();

  /* Allocate the server structure */

  hs = (FAR httpd_server *)zalloc(sizeof(httpd_server));
  if (!hs)
    {
      ndbg("out of memory allocating an httpd_server\n");
      return NULL;
    }

#ifdef CONFIG_THTTPD_HOSTNAME
  hs->hostname = strdup(CONFIG_THTTPD_HOSTNAME);
#else
  hs->hostname = strdup(httpd_ntoa(sa));
#endif
  nvdbg("hostname: %s\n", hs->hostname);

  if (!hs->hostname)
    {
      ndbg("out of memory copying hostname\n");
      return NULL;
    }

  hs->cgi_count = 0;

  /* Initialize listen sockets */

  hs->listen_fd = initialize_listen_socket(sa);
  if (hs->listen_fd == -1)
    {
      ndbg("Failed to create listen socket\n");
      free_httpd_server(hs);
      return NULL;
    }

  init_mime();

  /* Done initializing. */

  ndbg("%s starting on port %d\n", CONFIG_THTTPD_SERVER_SOFTWARE, (int)CONFIG_THTTPD_PORT);
  return hs;
}

void httpd_terminate(httpd_server * hs)
{
  httpd_unlisten(hs);
  free_httpd_server(hs);
}

void httpd_unlisten(httpd_server * hs)
{
  if (hs->listen_fd != -1)
    {
      (void)close(hs->listen_fd);
      hs->listen_fd = -1;
    }
}

/* Send the buffered response. */

void httpd_write_response(httpd_conn *hc)
{
  /* If we are in a sub-process, turn off no-delay mode. */

  if (main_thread != getpid())
    {
       httpd_clear_ndelay(hc->conn_fd);
    }

  /* Send the response, if necessary. */

  if (hc->buflen > 0)
    {
      (void)httpd_write(hc->conn_fd, hc->buffer, hc->buflen);
      hc->buflen = 0;
    }
}

/* Set no-delay / non-blocking mode on a socket. */

void httpd_set_ndelay(int fd)
{
  int flags, newflags;

  flags = fcntl(fd, F_GETFL, 0);
  if (flags != -1)
    {
      newflags = flags | (int)O_NDELAY;
      if (newflags != flags)
        (void)fcntl(fd, F_SETFL, newflags);
    }
}

/* Clear no-delay / non-blocking mode on a socket. */

void httpd_clear_ndelay(int fd)
{
  int flags, newflags;

  flags = fcntl(fd, F_GETFL, 0);
  if (flags != -1)
    {
      newflags = flags & ~(int)O_NDELAY;
      if (newflags != flags)
        {
          (void)fcntl(fd, F_SETFL, newflags);
        }
    }
}

void httpd_realloc_str(char **strP, size_t * maxsizeP, size_t size)
{
  if (*maxsizeP == 0)
    {
      *maxsizeP       = MAX(CONFIG_THTTPD_IOBUFFERSIZE, size + CONFIG_THTTPD_REALLOCINCR);
      *strP           = NEW(char, *maxsizeP + 1);
      ++str_alloc_count;
      str_alloc_size += *maxsizeP;
    }
  else if (size > *maxsizeP)
    {
      str_alloc_size -= *maxsizeP;
      *maxsizeP       = MAX(*maxsizeP * 2, size * 5 / 4);
      *strP = RENEW(*strP, char, *maxsizeP + 1);
      str_alloc_size += *maxsizeP;
    }
  else
    {
      return;
    }

  if (!*strP)
    {
      ndbg("out of memory reallocating a string to %d bytes\n", *maxsizeP);
      exit(1);
    }
}

void httpd_send_err(httpd_conn *hc, int status, const char *title, const char *extraheads,
                    const char *form, const char *arg)
{
#ifdef CONFIG_THTTPD_ERROR_DIRECTORY
  char filename[1000];

  /* Try virtual host error page. */

  ndbg("title: \"%s\" form: \"%s\"\n", title, form);

#ifdef CONFIG_THTTPD_VHOST
  if (hc->hostdir[0] != '\0')
    {
      (void)snprintf(filename, sizeof(filename),
                     "%s/%s/err%d.html", hc->hostdir, CONFIG_THTTPD_ERROR_DIRECTORY, status);
      if (send_err_file(hc, status, title, extraheads, filename))
        {
          nvdbg("Sent VHOST error file\n");
          return;
        }
    }
#endif

  /* Try server-wide error page. */

  (void)snprintf(filename, sizeof(filename), "%s/err%d.html", CONFIG_THTTPD_ERROR_DIRECTORY, status);
  if (send_err_file(hc, status, title, extraheads, filename))
    {
      nvdbg("Sent server-wide error page\n");
      return;
    }

  /* Fall back on built-in error page. */

  send_response(hc, status, title, extraheads, form, arg);

#else

  send_response(hc, status, title, extraheads, form, arg);

#endif
}

const char *httpd_method_str(int method)
{
  switch (method)
    {
    case METHOD_GET:
      return "GET";

    case METHOD_HEAD:
      return "HEAD";

    case METHOD_POST:
      return "POST";

    default:
      return "UNKNOWN";
    }
}

int httpd_get_conn(httpd_server *hs, int listen_fd, httpd_conn *hc)
{
  httpd_sockaddr sa;
  socklen_t sz;

  if (!hc->initialized)
    {
      hc->read_size = 0;
      httpd_realloc_str(&hc->read_buf, &hc->read_size, CONFIG_THTTPD_IOBUFFERSIZE);
      hc->maxdecodedurl =
        hc->maxorigfilename = hc->maxexpnfilename = hc->maxencodings =
        hc->maxpathinfo = hc->maxquery = hc->maxaccept =
        hc->maxaccepte = hc->maxreqhost = hc->maxhostdir =
        hc->maxremoteuser = 0;
#ifdef CONFIG_THTTPD_TILDE_MAP2
      hc->maxaltdir = 0;
#endif
      httpd_realloc_str(&hc->decodedurl, &hc->maxdecodedurl, 1);
      httpd_realloc_str(&hc->origfilename, &hc->maxorigfilename, 1);
      httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename, 0);
      httpd_realloc_str(&hc->encodings, &hc->maxencodings, 0);
      httpd_realloc_str(&hc->pathinfo, &hc->maxpathinfo, 0);
      httpd_realloc_str(&hc->query, &hc->maxquery, 0);
      httpd_realloc_str(&hc->accept, &hc->maxaccept, 0);
      httpd_realloc_str(&hc->accepte, &hc->maxaccepte, 0);
      httpd_realloc_str(&hc->reqhost, &hc->maxreqhost, 0);
      httpd_realloc_str(&hc->hostdir, &hc->maxhostdir, 0);
      httpd_realloc_str(&hc->remoteuser, &hc->maxremoteuser, 0);
#ifdef CONFIG_THTTPD_TILDE_MAP2
      httpd_realloc_str(&hc->altdir, &hc->maxaltdir, 0);
#endif
      hc->initialized = 1;
    }

  /* Accept the new connection. */

  nvdbg("accept() new connection on listen_fd %d\n", listen_fd);
  sz = sizeof(sa);
  hc->conn_fd = accept(listen_fd, (struct sockaddr*)&sa, &sz);
  if (hc->conn_fd < 0)
    {
      if (errno == EWOULDBLOCK)
        {
          return GC_NO_MORE;
        }

      ndbg("accept failed: %d\n", errno);
      return GC_FAIL;
    }

#ifdef CONFIG_DEBUG
  if (!sockaddr_check(&sa))
    {
      ndbg("unknown sockaddr family\n");
      close(hc->conn_fd);
      hc->conn_fd = -1;
      return GC_FAIL;
    }
#endif

  hc->hs = hs;
  (void)memset(&hc->client_addr, 0, sizeof(hc->client_addr));
  (void)memmove(&hc->client_addr, &sa, sockaddr_len(&sa));
  hc->read_idx          = 0;
  hc->checked_idx       = 0;
  hc->checked_state     = CHST_FIRSTWORD;
  hc->method            = METHOD_UNKNOWN;
  hc->status            = 0;
  hc->bytes_to_send     = 0;
  hc->bytes_sent        = 0;
  hc->encodedurl        = "";
  hc->decodedurl[0]     = '\0';
  hc->protocol          = "UNKNOWN";
  hc->origfilename[0]   = '\0';
  hc->expnfilename[0]   = '\0';
  hc->encodings[0]      = '\0';
  hc->pathinfo[0]       = '\0';
  hc->query[0]          = '\0';
  hc->referer           = "";
  hc->useragent         = "";
  hc->accept[0]         = '\0';
  hc->accepte[0]        = '\0';
  hc->acceptl           = "";
  hc->cookie            = "";
  hc->contenttype       = "";
  hc->reqhost[0]        = '\0';
  hc->hdrhost           = "";
  hc->hostdir[0]        = '\0';
  hc->authorization     = "";
  hc->remoteuser[0]     = '\0';
  hc->buffer[0]         = '\0';
#ifdef CONFIG_THTTPD_TILDE_MAP2
  hc->altdir[0]         = '\0';
#endif
  hc->buflen = 0;
  hc->if_modified_since = (time_t) - 1;
  hc->range_if          = (time_t)-1;
  hc->contentlength     = -1;
  hc->type = "";
#ifdef CONFIG_THTTPD_VHOST
  hc->vhostname         = NULL;
#endif
  hc->mime_flag         = TRUE;
  hc->one_one           = FALSE;
  hc->got_range         = FALSE;
  hc->tildemapped       = FALSE;
  hc->range_start       = 0;
  hc->range_end         = -1;
  hc->keep_alive        = FALSE;
  hc->should_linger     = FALSE;
  hc->file_fd           = -1;

  nvdbg("New connection accepted on %d\n", hc->conn_fd);
  return GC_OK;
}

/* Checks hc->read_buf to see whether a complete request has been read so far;
 * either the first line has two words (an HTTP/0.9 request), or the first
 * line has three words and there's a blank line present.
 *
 * hc->read_idx is how much has been read in; hc->checked_idx is how much we
 * have checked so far; and hc->checked_state is the current state of the
 * finite state machine.
 */

int httpd_got_request(httpd_conn *hc)
{
  char c;

  for (; hc->checked_idx < hc->read_idx; ++hc->checked_idx)
    {
      c = hc->read_buf[hc->checked_idx];
      switch (hc->checked_state)
        {
        case CHST_FIRSTWORD:
          switch (c)
            {
            case ' ':
            case '\t':
              hc->checked_state = CHST_FIRSTWS;
              break;

            case '\012':
            case '\015':
              hc->checked_state = CHST_BOGUS;
              return GR_BAD_REQUEST;
            }
          break;

        case CHST_FIRSTWS:
          switch (c)
            {
            case ' ':
            case '\t':
              break;

            case '\012':
            case '\015':
              hc->checked_state = CHST_BOGUS;
              return GR_BAD_REQUEST;

            default:
              hc->checked_state = CHST_SECONDWORD;
              break;
            }
          break;

        case CHST_SECONDWORD:
          switch (c)
            {
            case ' ':
            case '\t':
              hc->checked_state = CHST_SECONDWS;
              break;

            case '\012':
            case '\015':
              /* The first line has only two words - an HTTP/0.9 request. */
              return GR_GOT_REQUEST;
            }
          break;

        case CHST_SECONDWS:
          switch (c)
            {
            case ' ':
            case '\t':
              break;

            case '\012':
            case '\015':
              hc->checked_state = CHST_BOGUS;
              return GR_BAD_REQUEST;

            default:
              hc->checked_state = CHST_THIRDWORD;
              break;
            }
          break;

        case CHST_THIRDWORD:
          switch (c)
            {
            case ' ':
            case '\t':
              hc->checked_state = CHST_THIRDWS;
              break;

            case '\012':
              hc->checked_state = CHST_LF;
              break;

            case '\015':
              hc->checked_state = CHST_CR;
              break;
            }
          break;

        case CHST_THIRDWS:
          switch (c)
            {
            case ' ':
            case '\t':
              break;

            case '\012':
              hc->checked_state = CHST_LF;
              break;

            case '\015':
              hc->checked_state = CHST_CR;
              break;

            default:
              hc->checked_state = CHST_BOGUS;
              return GR_BAD_REQUEST;
            }
          break;

        case CHST_LINE:
          switch (c)
            {
            case '\012':
              hc->checked_state = CHST_LF;
              break;

            case '\015':
              hc->checked_state = CHST_CR;
              break;
            }
          break;
 
       case CHST_LF:
          switch (c)
            {
            case '\012':
              /* Two newlines in a row - a blank line - end of request. */

              return GR_GOT_REQUEST;

            case '\015':
              hc->checked_state = CHST_CR;
              break;

            default:
              hc->checked_state = CHST_LINE;
              break;
            }
          break;

        case CHST_CR:
          switch (c)
            {
            case '\012':
              hc->checked_state = CHST_CRLF;
              break;

            case '\015':
              /* Two returns in a row - end of request. */

              return GR_GOT_REQUEST;

            default:
              hc->checked_state = CHST_LINE;
              break;
            }
          break;

        case CHST_CRLF:
          switch (c)
            {
            case '\012':
              /* Two newlines in a row - end of request. */

              return GR_GOT_REQUEST;

            case '\015':
              hc->checked_state = CHST_CRLFCR;
              break;

            default:
              hc->checked_state = CHST_LINE;
              break;
            }
          break;

        case CHST_CRLFCR:
          switch (c)
            {
            case '\012':
            case '\015':
              /* Two CRLFs or two CRs in a row - end of request. */

              return GR_GOT_REQUEST;

            default:
              hc->checked_state = CHST_LINE;
              break;
            }
          break;

        case CHST_BOGUS:
          return GR_BAD_REQUEST;
        }
    }
  return GR_NO_REQUEST;
}

int httpd_parse_request(httpd_conn *hc)
{
  char *buf;
  char *method_str;
  char *url;
  char *protocol;
  char *reqhost;
  char *eol;
  char *cp;
  char *pi;

  hc->checked_idx = 0;          /* reset */
  method_str      = bufgets(hc);
  nvdbg("method_str: \"%s\"\n", method_str);

  url = strpbrk(method_str, " \t\012\015");
  if (!url)
    {
      BADREQUEST("url-1");
      httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
      return -1;
    }

  *url++ = '\0';
  url   += strspn(url, " \t\012\015");
  nvdbg("url: \"%s\"\n", url);

  protocol = strpbrk(url, " \t\012\015");
  nvdbg("protocol: \"%s\"\n", protocol ? protocol : "<null>");

  if (!protocol)
    {
      protocol      = "HTTP/0.9";
      hc->mime_flag = FALSE;
    }
  else
    {
      *protocol++ = '\0';
      protocol += strspn(protocol, " \t\012\015");
      if (*protocol != '\0')
        {
          eol = strpbrk(protocol, " \t\012\015");
          if (eol)
            {
              *eol = '\0';
            }

          if (strcasecmp(protocol, "HTTP/1.0") != 0)
            {
              hc->one_one = TRUE;
            }
        }
    }
  hc->protocol = protocol;

  /* Check for HTTP/1.1 absolute URL. */

  if (strncasecmp(url, "http://", 7) == 0)
    {
      if (!hc->one_one)
        {
          BADREQUEST("one_one");
          httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
          return -1;
        }

      reqhost = url + 7;
      url     = strchr(reqhost, '/');
      if (!url)
        {
          BADREQUEST("reqhost-1");
          httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
          return -1;
        }
      *url = '\0';

      if (strchr(reqhost, '/') != NULL || reqhost[0] == '.')
        {
          BADREQUEST("reqhost-2");
          httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
          return -1;
        }

      httpd_realloc_str(&hc->reqhost, &hc->maxreqhost, strlen(reqhost));
      (void)strcpy(hc->reqhost, reqhost);
      *url = '/';
    }

  if (*url != '/')
    {
      BADREQUEST("url-2");
      httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
      return -1;
    }

  if (strcasecmp(method_str, httpd_method_str(METHOD_GET)) == 0)
    {
      hc->method = METHOD_GET;
    }
  else if (strcasecmp(method_str, httpd_method_str(METHOD_HEAD)) == 0)
    {
      hc->method = METHOD_HEAD;
    }
  else if (strcasecmp(method_str, httpd_method_str(METHOD_POST)) == 0)
    {
      hc->method = METHOD_POST;
    }
  else
    {
      NOTIMPLEMENTED(method_str);
      httpd_send_err(hc, 501, err501title, "", err501form, method_str);
      return -1;
    }

  hc->encodedurl = url;
  httpd_realloc_str(&hc->decodedurl, &hc->maxdecodedurl, strlen(hc->encodedurl));
  strdecode(hc->decodedurl, hc->encodedurl);

  httpd_realloc_str(&hc->origfilename, &hc->maxorigfilename, strlen(hc->decodedurl));
  (void)strcpy(hc->origfilename, &hc->decodedurl[1]);

  /* Special case for top-level URL. */

  if (hc->origfilename[0] == '\0')
    {
      (void)strcpy(hc->origfilename, ".");
    }

  /* Extract query string from encoded URL. */

  cp = strchr(hc->encodedurl, '?');
  if (cp)
    {
      ++cp;
      httpd_realloc_str(&hc->query, &hc->maxquery, strlen(cp));
      (void)strcpy(hc->query, cp);

      /* Remove query from (decoded) origfilename. */

      cp = strchr(hc->origfilename, '?');
      if (cp)
        {
          *cp = '\0';
        }
    }

  de_dotdot(hc->origfilename);
  if (hc->origfilename[0] == '/' ||
      (hc->origfilename[0] == '.' && hc->origfilename[1] == '.' &&
       (hc->origfilename[2] == '\0' || hc->origfilename[2] == '/')))
    {
      BADREQUEST("origfilename");
      httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
      return -1;
    }

  if (hc->mime_flag)
    {
      /* Read the MIME headers. */
      while ((buf = bufgets(hc)) != NULL)
        {
          if (buf[0] == '\0')
            {
              break;
            }

          if (strncasecmp(buf, "Referer:", 8) == 0)
            {
              cp = &buf[8];
              cp += strspn(cp, " \t");
              hc->referer = cp;
            }
          else if (strncasecmp(buf, "User-Agent:", 11) == 0)
            {
              cp = &buf[11];
              cp += strspn(cp, " \t");
              hc->useragent = cp;
            }
          else if (strncasecmp(buf, "Host:", 5) == 0)
            {
              cp = &buf[5];
              cp += strspn(cp, " \t");
              hc->hdrhost = cp;
              cp = strchr(hc->hdrhost, ':');
              if (cp)
                {
                  *cp = '\0';
                }

              if (strchr(hc->hdrhost, '/') != NULL || hc->hdrhost[0] == '.')
                {
                  BADREQUEST("hdrhost");
                  httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
                  return -1;
                }
            }
          else if (strncasecmp(buf, "Accept:", 7) == 0)
            {
              cp = &buf[7];
              cp += strspn(cp, " \t");
              if (hc->accept[0] != '\0')
                {
                  if (strlen(hc->accept) > CONFIG_THTTPD_MAXREALLOC)
                    {
                      ndbg("%s way too much Accept: data\n",
                             httpd_ntoa(&hc->client_addr));
                      continue;
                    }
                  httpd_realloc_str(&hc->accept, &hc->maxaccept, strlen(hc->accept) + 2 + strlen(cp));
                  (void)strcat(hc->accept, ", ");
                }
              else
                {
                  httpd_realloc_str(&hc->accept, &hc->maxaccept, strlen(cp));
                }
              (void)strcat(hc->accept, cp);
            }
          else if (strncasecmp(buf, "Accept-Encoding:", 16) == 0)
            {
              cp = &buf[16];
              cp += strspn(cp, " \t");
              if (hc->accepte[0] != '\0')
                {
                  if (strlen(hc->accepte) > CONFIG_THTTPD_MAXREALLOC)
                    {
                      ndbg("%s way too much Accept-Encoding: data\n",
                             httpd_ntoa(&hc->client_addr));
                      continue;
                    }
                  httpd_realloc_str(&hc->accepte, &hc->maxaccepte, strlen(hc->accepte) + 2 + strlen(cp));
                  (void)strcat(hc->accepte, ", ");
                }
              else
                {
                  httpd_realloc_str(&hc->accepte, &hc->maxaccepte, strlen(cp));
                }
             (void)strcpy(hc->accepte, cp);
            }
          else if (strncasecmp(buf, "Accept-Language:", 16) == 0)
            {
              cp = &buf[16];
              cp += strspn(cp, " \t");
              hc->acceptl = cp;
            }
          else if (strncasecmp(buf, "If-Modified-Since:", 18) == 0)
            {
              cp = &buf[18];
              hc->if_modified_since = tdate_parse(cp);
              if (hc->if_modified_since == (time_t) - 1)
                ndbg("unparsable time: %s\n", cp);
            }
          else if (strncasecmp(buf, "Cookie:", 7) == 0)
            {
              cp = &buf[7];
              cp += strspn(cp, " \t");
              hc->cookie = cp;
            }
          else if (strncasecmp(buf, "Range:", 6) == 0)
            {
              /* Only support %d- and %d-%d, not %d-%d,%d-%d or -%d. */
              if (strchr(buf, ',') == NULL)
                {
                  char *cp_dash;
                  cp = strpbrk(buf, "=");
                  if (cp)
                    {
                      cp_dash = strchr(cp + 1, '-');
                      if (cp_dash != NULL && cp_dash != cp + 1)
                        {
                          *cp_dash = '\0';
                          hc->got_range = TRUE;
                          hc->range_start = atoll(cp + 1);
                          if (hc->range_start < 0)
                            {
                              hc->range_start = 0;
                            }

                          if (isdigit((int)cp_dash[1]))
                            {
                              hc->range_end = atoll(cp_dash + 1);
                              if (hc->range_end < 0)
                                hc->range_end = -1;
                            }
                        }
                    }
                }
            }
          else if (strncasecmp(buf, "Range-If:", 9) == 0 ||
                   strncasecmp(buf, "If-Range:", 9) == 0)
            {
              cp = &buf[9];
              hc->range_if = tdate_parse(cp);
              if (hc->range_if == (time_t) - 1)
                {
                  ndbg("unparsable time: %s\n", cp);
                }
            }
          else if (strncasecmp(buf, "Content-Type:", 13) == 0)
            {
              cp = &buf[13];
              cp += strspn(cp, " \t");
              hc->contenttype = cp;
            }
          else if (strncasecmp(buf, "Content-Length:", 15) == 0)
            {
              cp = &buf[15];
              hc->contentlength = atol(cp);
            }
          else if (strncasecmp(buf, "Authorization:", 14) == 0)
            {
              cp = &buf[14];
              cp += strspn(cp, " \t");
              hc->authorization = cp;
            }
          else if (strncasecmp(buf, "Connection:", 11) == 0)
            {
              cp = &buf[11];
              cp += strspn(cp, " \t");
              if (strcasecmp(cp, "keep-alive") == 0)
               {
                 hc->keep_alive = TRUE;
               }
           }
#ifdef LOG_UNKNOWN_HEADERS
          else if (strncasecmp(buf, "Accept-Charset:", 15) == 0 ||
                   strncasecmp(buf, "Accept-Language:", 16) == 0 ||
                   strncasecmp(buf, "Agent:", 6) == 0 ||
                   strncasecmp(buf, "Cache-Control:", 14) == 0 ||
                   strncasecmp(buf, "Cache-Info:", 11) == 0 ||
                   strncasecmp(buf, "Charge-To:", 10) == 0 ||
                   strncasecmp(buf, "Client-IP:", 10) == 0 ||
                   strncasecmp(buf, "Date:", 5) == 0 ||
                   strncasecmp(buf, "Extension:", 10) == 0 ||
                   strncasecmp(buf, "Forwarded:", 10) == 0 ||
                   strncasecmp(buf, "From:", 5) == 0 ||
                   strncasecmp(buf, "HTTP-Version:", 13) == 0 ||
                   strncasecmp(buf, "Max-Forwards:", 13) == 0 ||
                   strncasecmp(buf, "Message-Id:", 11) == 0 ||
                   strncasecmp(buf, "MIME-Version:", 13) == 0 ||
                   strncasecmp(buf, "Negotiate:", 10) == 0 ||
                   strncasecmp(buf, "Pragma:", 7) == 0 ||
                   strncasecmp(buf, "Proxy-Agent:", 12) == 0 ||
                   strncasecmp(buf, "Proxy-Connection:", 17) == 0 ||
                   strncasecmp(buf, "Security-Scheme:", 16) == 0 ||
                   strncasecmp(buf, "Session-Id:", 11) == 0 ||
                   strncasecmp(buf, "UA-Color:", 9) == 0 ||
                   strncasecmp(buf, "UA-CPU:", 7) == 0 ||
                   strncasecmp(buf, "UA-Disp:", 8) == 0 ||
                   strncasecmp(buf, "UA-OS:", 6) == 0 ||
                   strncasecmp(buf, "UA-Pixels:", 10) == 0 ||
                   strncasecmp(buf, "User:", 5) == 0 ||
                   strncasecmp(buf, "Via:", 4) == 0 ||
                   strncasecmp(buf, "X-", 2) == 0)
            ;                   /* ignore */
          else
            {
              ndbg("unknown request header: %s\n", buf);
            }
#endif
        }
    }

  if (hc->one_one)
    {
      /* Check that HTTP/1.1 requests specify a host, as required. */

      if (hc->reqhost[0] == '\0' && hc->hdrhost[0] == '\0')
        {
          BADREQUEST("reqhost-3");
          httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");
          return -1;
        }

      /* If the client wants to do keep-alives, it might also be doing
       * pipelining.  There's no way for us to tell.  Since we don't
       * implement keep-alives yet, if we close such a connection there
       * might be unread pipelined requests waiting.  So, we have to do a
       * lingering close.
       */

      if (hc->keep_alive)
        {
          hc->should_linger = TRUE;
        }
    }

  /* Ok, the request has been parsed.  Now we resolve stuff that may require 
   * the entire request.
   */

  /* Copy original filename to expanded filename. */

  httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename,
                    strlen(hc->origfilename));
  (void)strcpy(hc->expnfilename, hc->origfilename);

  /* Tilde mapping. */

  if (hc->expnfilename[0] == '~')
    {
#ifdef CONFIG_THTTPD_TILDE_MAP1
      if (!tilde_map_1(hc))
        {
          httpd_send_err(hc, 404, err404title, "", err404form, hc->encodedurl);
          return -1;
        }
#endif                                 /*CONFIG_THTTPD_TILDE_MAP1 */
#ifdef CONFIG_THTTPD_TILDE_MAP2
      if (!tilde_map_2(hc))
        {
          httpd_send_err(hc, 404, err404title, "", err404form, hc->encodedurl);
          return -1;
        }
#endif                                 /*CONFIG_THTTPD_TILDE_MAP2 */
    }

  /* Virtual host mapping. */

#ifdef CONFIG_THTTPD_VHOST
    if (!vhost_map(hc))
      {
        INTERNALERROR("VHOST");
        httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
        return -1;
      }
#endif

  /* Expand the filename */

  cp = expand_filename(hc->expnfilename, &pi, hc->tildemapped);
  if (!cp)
    {
      INTERNALERROR(hc->expnfilename);
      httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
      return -1;
    }

  httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename, strlen(cp));
  (void)strcpy(hc->expnfilename, cp);
  httpd_realloc_str(&hc->pathinfo, &hc->maxpathinfo, strlen(pi));
  (void)strcpy(hc->pathinfo, pi);
  nvdbg("expnfilename: \"%s\" pathinfo: \"%s\"\n", hc->expnfilename, hc->pathinfo);

  /* Remove pathinfo stuff from the original filename too. */

  if (hc->pathinfo[0] != '\0')
    {
      int i;
      i = strlen(hc->origfilename) - strlen(hc->pathinfo);
      if (i > 0 && strcmp(&hc->origfilename[i], hc->pathinfo) == 0)
        {
          hc->origfilename[i - 1] = '\0';
        }
    }

  /* If the expanded filename is an absolute path, check that it's still
   * within the current directory or the alternate directory.
   */

  if (hc->expnfilename[0] == '/')
    {
      if (strncmp(hc->expnfilename, httpd_root, strlen(httpd_root)) == 0)
        {
        }
#ifdef CONFIG_THTTPD_TILDE_MAP2
      else if (hc->altdir[0] != '\0' &&
               (strncmp(hc->expnfilename, hc->altdir, strlen(hc->altdir)) == 0 &&
                (hc->expnfilename[strlen(hc->altdir)] == '\0' ||
                 hc->expnfilename[strlen(hc->altdir)] == '/')))
        {
        }
#endif
      else
        {
          ndbg("%s URL \"%s\" goes outside the web tree\n",
                 httpd_ntoa(&hc->client_addr), hc->encodedurl);
          httpd_send_err(hc, 403, err403title, "",
                         ERROR_FORM(err403form,
                                    "The requested URL '%s' resolves to a file outside the permitted web server directory tree.\n"),
                         hc->encodedurl);
          return -1;
        }
    }

  return 0;
}

void httpd_close_conn(httpd_conn *hc, struct timeval *nowP)
{
  if (hc->file_fd >= 0)
    {
      (void)close(hc->file_fd);
      hc->file_fd = -1;
    }

  if (hc->conn_fd >= 0)
    {
      (void)close(hc->conn_fd);
      hc->conn_fd = -1;
    }
}

void httpd_destroy_conn(httpd_conn *hc)
{
  if (hc->initialized)
    {
      free((void *)hc->read_buf);
      free((void *)hc->decodedurl);
      free((void *)hc->origfilename);
      free((void *)hc->expnfilename);
      free((void *)hc->encodings);
      free((void *)hc->pathinfo);
      free((void *)hc->query);
      free((void *)hc->accept);
      free((void *)hc->accepte);
      free((void *)hc->reqhost);
      free((void *)hc->hostdir);
      free((void *)hc->remoteuser);
      free((void *)hc->buffer);
#ifdef CONFIG_THTTPD_TILDE_MAP2
      free((void *)hc->altdir);
#endif                                 /*CONFIG_THTTPD_TILDE_MAP2 */
      hc->initialized = 0;
    }
}

int httpd_start_request(httpd_conn *hc, struct timeval *nowP)
{
  static char *indexname;
  static size_t maxindexname = 0;
#ifdef CONFIG_THTTPD_AUTH_FILE
  static char *dirname;
  static size_t maxdirname = 0;
#endif                                 /* CONFIG_THTTPD_AUTH_FILE */
  size_t expnlen, indxlen;
  char *cp;
  char *pi;
  int i;

  nvdbg("File: \"%s\"\n", hc->expnfilename);
  expnlen = strlen(hc->expnfilename);

  if (hc->method != METHOD_GET && hc->method != METHOD_HEAD &&
      hc->method != METHOD_POST)
    {
      NOTIMPLEMENTED("start");
      httpd_send_err(hc, 501, err501title, "", err501form,
                     httpd_method_str(hc->method));
      return -1;
    }

  /* Stat the file. */

  if (stat(hc->expnfilename, &hc->sb) < 0)
    {
      INTERNALERROR(hc->expnfilename);
      httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
      return -1;
    }

  /* Is it world-readable or world-executable? We check explicitly instead
   * of just trying to open it, so that no one ever gets surprised by a file 
   * that's not set world-readable and yet somehow is readable by the HTTP
   * server and therefore the *whole* world.
   */

  if (!(hc->sb.st_mode & (S_IROTH | S_IXOTH)))
    {
      ndbg("%s URL \"%s\" resolves to a non world-readable file\n",
           httpd_ntoa(&hc->client_addr), hc->encodedurl);
      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "The requested URL '%s' resolves to a file that is not world-readable.\n"),
                     hc->encodedurl);
      return -1;
    }

  /* Is it a directory? */

  if (S_ISDIR(hc->sb.st_mode))
    {
      /* If there's pathinfo, it's just a non-existent file. */

      if (hc->pathinfo[0] != '\0')
        {
          httpd_send_err(hc, 404, err404title, "", err404form, hc->encodedurl);
          return -1;
        }

      /* Special handling for directory URLs that don't end in a slash. We
       * send back an explicit redirect with the slash, because otherwise
       * many clients can't build relative URLs properly.
       */

      if (strcmp(hc->origfilename, "") != 0 &&
          strcmp(hc->origfilename, ".") != 0 &&
          hc->origfilename[strlen(hc->origfilename) - 1] != '/')
        {
          send_dirredirect(hc);
          return -1;
        }

      /* Check for an index file. */

      for (i = 0; i < sizeof(index_names) / sizeof(char *); ++i)
        {
          httpd_realloc_str(&indexname, &maxindexname,
                            expnlen + 1 + strlen(index_names[i]));
          (void)strcpy(indexname, hc->expnfilename);
          indxlen = strlen(indexname);
          if (indxlen == 0 || indexname[indxlen - 1] != '/')
            {
              (void)strcat(indexname, "/");
            }

          if (strcmp(indexname, "./") == 0)
            {
              indexname[0] = '\0';
            }

          (void)strcat(indexname, index_names[i]);
          if (stat(indexname, &hc->sb) >= 0)
            {
              goto got_one;
            }
        }

      /* Nope, no index file, so it's an actual directory request. */
#ifdef CONFIG_THTTPD_GENERATE_INDICES
      /* Directories must be readable for indexing. */
      if (!(hc->sb.st_mode & S_IROTH))
        {
          ndbg("%s URL \"%s\" tried to index a directory with indexing disabled\n",
                 httpd_ntoa(&hc->client_addr), hc->encodedurl);
          httpd_send_err(hc, 403, err403title, "",
                         ERROR_FORM(err403form,
                                    "The requested URL '%s' resolves to a directory that has indexing disabled.\n"),
                         hc->encodedurl);
          return -1;
        }
#  ifdef CONFIG_THTTPD_AUTH_FILE
      /* Check authorization for this directory. */

      if (auth_check(hc, hc->expnfilename) == -1)
        {
          return -1;
        }
#  endif /* CONFIG_THTTPD_AUTH_FILE */

      /* Referer check. */

      if (!check_referer(hc))
        {
          return -1;
        }

      /* Ok, generate an index. */
      return ls(hc);
#else
      ndbg("%s URL \"%s\" tried to index a directory\n",
             httpd_ntoa(&hc->client_addr), hc->encodedurl);
      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "The requested URL '%s' is a directory, and directory indexing is disabled on this server.\n"),
                     hc->encodedurl);
      return -1;
#endif

    got_one:

      /* Got an index file.  Expand again.  More pathinfo means
       * something went wrong.
       */

      cp = expand_filename(indexname, &pi, hc->tildemapped);
      if (cp == NULL || pi[0] != '\0')
        {
          INTERNALERROR(indexname);
          httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
          return -1;
        }

      expnlen = strlen(cp);
      httpd_realloc_str(&hc->expnfilename, &hc->maxexpnfilename, expnlen);
      (void)strcpy(hc->expnfilename, cp);

      /* Now, is the index version world-readable or world-executable? */

      if (!(hc->sb.st_mode & (S_IROTH | S_IXOTH)))
        {
          ndbg("%s URL \"%s\" resolves to a non-world-readable index file\n",
                 httpd_ntoa(&hc->client_addr), hc->encodedurl);
          httpd_send_err(hc, 403, err403title, "",
                         ERROR_FORM(err403form,
                                    "The requested URL '%s' resolves to an index file that is not world-readable.\n"),
                         hc->encodedurl);
          return -1;
        }
    }

  /* Check authorization for this directory. */

#ifdef CONFIG_THTTPD_AUTH_FILE
  httpd_realloc_str(&dirname, &maxdirname, expnlen);
  (void)strcpy(dirname, hc->expnfilename);
  cp = strrchr(dirname, '/');
  if (!cp)
    {
      (void)strcpy(dirname, httpd_root);
    }
  else
    {
      *cp = '\0';
    }

  if (auth_check(hc, dirname) == -1)
    {
      return -1;
    }

  /* Check if the filename is the CONFIG_THTTPD_AUTH_FILE itself - that's verboten. */

  if (expnlen == sizeof(CONFIG_THTTPD_AUTH_FILE) - 1)
    {
      if (strcmp(hc->expnfilename, CONFIG_THTTPD_AUTH_FILE) == 0)
        {
          ndbg("%s URL \"%s\" tried to retrieve an auth file\n",
                 httpd_ntoa(&hc->client_addr), hc->encodedurl);
          httpd_send_err(hc, 403, err403title, "",
                         ERROR_FORM(err403form,
                                    "The requested URL '%s' is an authorization file, retrieving it is not permitted.\n"),
                         hc->encodedurl);
          return -1;
        }
    }
  else if (expnlen >= sizeof(CONFIG_THTTPD_AUTH_FILE) &&
           strcmp(&(hc->expnfilename[expnlen - sizeof(CONFIG_THTTPD_AUTH_FILE) + 1]),
                  CONFIG_THTTPD_AUTH_FILE) == 0 &&
           hc->expnfilename[expnlen - sizeof(CONFIG_THTTPD_AUTH_FILE)] == '/')
    {
      ndbg("%s URL \"%s\" tried to retrieve an auth file\n",
             httpd_ntoa(&hc->client_addr), hc->encodedurl);
      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "The requested URL '%s' is an authorization file, retrieving it is not permitted.\n"),
                     hc->encodedurl);
      return -1;
    }
#endif

  /* Referer check. */

  if (!check_referer(hc))
    return -1;

  /* Is it in the CGI area? */

#ifdef CONFIG_THTTPD_CGI_PATTERN
  if (match(CONFIG_THTTPD_CGI_PATTERN, hc->expnfilename))
    {
      return cgi(hc);
    }
#endif

  /* It's not CGI.  If it's executable or there's pathinfo, someone's trying 
   * to either serve or run a non-CGI file as CGI.  Either case is
   * prohibited.
   */

  if (hc->sb.st_mode & S_IXOTH)
    {
      ndbg("%s URL \"%s\" is executable but isn't CGI\n",
             httpd_ntoa(&hc->client_addr), hc->encodedurl);
      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "The requested URL '%s' resolves to a file which is marked executable but is not a CGI file; retrieving it is forbidden.\n"),
                     hc->encodedurl);
      return -1;
    }

  if (hc->pathinfo[0] != '\0')
    {
      ndbg("%s URL \"%s\" has pathinfo but isn't CGI\n",
             httpd_ntoa(&hc->client_addr), hc->encodedurl);
      httpd_send_err(hc, 403, err403title, "",
                     ERROR_FORM(err403form,
                                "The requested URL '%s' resolves to a file plus CGI-style pathinfo, but the file is not a valid CGI file.\n"),
                     hc->encodedurl);
      return -1;
    }

  /* Fill in range_end, if necessary. */

  if (hc->got_range &&
      (hc->range_end == -1 || hc->range_end >= hc->sb.st_size))
    {
      hc->range_end = hc->sb.st_size - 1;
    }

  figure_mime(hc);

  if (hc->method == METHOD_HEAD)
    {
      send_mime(hc, 200, ok200title, hc->encodings, "", hc->type,
                hc->sb.st_size, hc->sb.st_mtime);
    }
  else if (hc->if_modified_since != (time_t) - 1 &&
           hc->if_modified_since >= hc->sb.st_mtime)
    {
      send_mime(hc, 304, err304title, hc->encodings, "", hc->type, (off_t) - 1,
                hc->sb.st_mtime);
    }
  else
    {
      hc->file_fd = open(hc->expnfilename, O_RDONLY);
      if (!hc->file_fd < 0)
        {
          INTERNALERROR(hc->expnfilename);
          httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
          return -1;
        }
      send_mime(hc, 200, ok200title, hc->encodings, "", hc->type,
                hc->sb.st_size, hc->sb.st_mtime);
    }

  return 0;
}

char *httpd_ntoa(httpd_sockaddr *saP)
{
#ifdef  CONFIG_NET_IPv6
  static char str[200];

  if (getnameinfo
      (&saP->sa, sockaddr_len(saP), str, sizeof(str), 0, 0,
       NI_NUMERICHOST) != 0)
    {
      str[0] = '?';
      str[1] = '\0';
    }
  else if (IN6_IS_ADDR_V4MAPPED(&saP->sa_in6.sin6_addr) &&
           strncmp(str, "::ffff:", 7) == 0)
    {
      /* Elide IPv6ish prefix for IPv4 addresses. */

      (void)strcpy(str, &str[7]);
    }

  return str;

#else /* CONFIG_NET_IPv6 */

  return inet_ntoa(saP->sin_addr);

#endif
}

/* Read to requested buffer, accounting for interruptions and EOF */

int httpd_read(int fd, const void *buf, size_t nbytes)
{
  ssize_t nread;
  int ntotal;

  ntotal = 0;
  do
    {
      nread = read(fd, (char*)buf + ntotal, nbytes - ntotal);
      if (nread < 0)
        {
          if (errno == EAGAIN)
            {
              usleep(100000); /* 100MS */
            }
          else if (errno != EINTR)
            {
              ndbg("Error sending: %d\n", errno);
              return nread;
            }
        }
      else
        {
          ntotal += nread;
        }
    }
  while (ntotal < nbytes && nread != 0);
  return ntotal;
}

/* Write the requested buffer completely, accounting for interruptions */

int httpd_write(int fd, const void *buf, size_t nbytes)
{
  ssize_t nwritten;
  int ntotal;

  ntotal = 0;
  do
    {
      nwritten = write(fd, (char*)buf + ntotal, nbytes - ntotal);
      if (nwritten < 0)
        {
          if (errno == EAGAIN)
            {
              usleep(100000); /* 100MS */
            }
          else if (errno != EINTR)
            {
              ndbg("Error sending: %d\n", errno);
              return nwritten;
            }
        }
      else
        {
          ntotal += nwritten;
        }
    }
  while (ntotal < nbytes);
  return ntotal;
}

/* Generate debugging statistics */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
void httpd_logstats(long secs)
{
  if (str_alloc_count > 0)
    {
      ndbg("  libhttpd - %d strings allocated, %lu bytes (%g bytes/str)\n",
           str_alloc_count, (unsigned long)str_alloc_size,
           (float)str_alloc_size / str_alloc_count);
    }
}
#endif
#endif /* CONFIG_THTTPD */

