/**
 * webclient.h
 * Header file for the HTTP client.
 * Author: Adam Dunkels <adam@dunkels.com>
 *
 * Copyright (c) 2002, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
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
 */

#ifndef __WEBCLIENT_H__
#define __WEBCLIENT_H__

#include <sys/types.h>
#include <net/uip/uipopt.h>

#define WEBCLIENT_CONF_MAX_URLLEN 100

struct webclient_state
{
  uint8 timer;
  uint8 state;
  uint8 httpflag;

  uint16 port;
  char host[40];
  char file[WEBCLIENT_CONF_MAX_URLLEN];
  uint16 getrequestptr;
  uint16 getrequestleft;

  char httpheaderline[200];
  uint16 httpheaderlineptr;

  char mimetype[32];
};

/* Callback function that is called from the webclient code when HTTP
 * data has been received.
 *
 * This function must be implemented by the module that uses the
 * webclient code. The function is called from the webclient module
 * when HTTP data has been received. The function is not called when
 * HTTP headers are received, only for the actual data.
 *
 * Note: This function is called many times, repetedly, when data is
 * being received, and not once when all data has been received.
 *
 * data A pointer to the data that has been received.
 * len The length of the data that has been received.
 */

void webclient_datahandler(char *data, uint16 len);

/* Callback function that is called from the webclient code when the
 * HTTP connection has been connected to the web server.
 *
 * This function must be implemented by the module that uses the
 * webclient code.
 */

void webclient_connected(void);

/* Callback function that is called from the webclient code if the
 * HTTP connection to the web server has timed out.
 *
 * This function must be implemented by the module that uses the
 * webclient code.
 */

void webclient_timedout(void);

/* Callback function that is called from the webclient code if the
 * HTTP connection to the web server has been aborted by the web
 * server.
 *
 * This function must be implemented by the module that uses the
 * webclient code.
 */

void webclient_aborted(void);

/* Callback function that is called from the webclient code when the
 * HTTP connection to the web server has been closed.
 *
 * This function must be implemented by the module that uses the
 * webclient code.
 */

void webclient_closed(void);

/* Initialize the webclient module. */

void webclient_init(void);

/* Open an HTTP connection to a web server and ask for a file using
 * the GET method.
 *
 * This function opens an HTTP connection to the specified web server
 * and requests the specified file using the GET method. When the HTTP
 * connection has been connected, the webclient_connected() callback
 * function is called and when the HTTP data arrives the
 * webclient_datahandler() callback function is called.
 *
 * The callback function webclient_timedout() is called if the web
 * server could not be contacted, and the webclient_aborted() callback
 * function is called if the HTTP connection is aborted by the web
 * server.
 *
 * When the HTTP request has been completed and the HTTP connection is
 * closed, the webclient_closed() callback function will be called.
 *
 * Note: If the function is passed a host name, it must already be in
 * the resolver cache in order for the function to connect to the web
 * server. It is therefore up to the calling module to implement the
 * resolver calls and the signal handler used for reporting a resolv
 * query answer.
 *
 * host A pointer to a string containing either a host name or
 * a numerical IP address in dotted decimal notation (e.g., 192.168.23.1).
 *
 * port The port number to which to connect, in host byte order.
 *
 * file A pointer to the name of the file to get.
 *
 * Return:  0 if the host name could not be found in the cache, or
 * if a TCP connection could not be created.
 *
 * Return:  1 if the connection was initiated.
 */

unsigned char webclient_get(const char *host, uint16 port, char *file);

/* Close the currently open HTTP connection. */

void webclient_close(void);

/* Obtain the MIME type of the current HTTP data stream.
 *
 * Return:  A pointer to a string contaning the MIME type. The string
 * may be empty if no MIME type was reported by the web server.
 */

char *webclient_mimetype(void);

/* Obtain the filename of the current HTTP data stream.
 *
 * The filename of an HTTP request may be changed by the web server,
 * and may therefore not be the same as when the original GET request
 * was made with webclient_get(). This function is used for obtaining
 * the current filename.
 *
 * Return:  A pointer to the current filename.
 */

char *webclient_filename(void);

/* Obtain the hostname of the current HTTP data stream.
 *
 * The hostname of the web server of an HTTP request may be changed
 * by the web server, and may therefore not be the same as when the
 * original GET request was made with webclient_get(). This function
 * is used for obtaining the current hostname.
 *
 * Return:  A pointer to the current hostname.
 */

char *webclient_hostname(void);

/* Obtain the port number of the current HTTP data stream.
 *
 * The port number of an HTTP request may be changed by the web
 * server, and may therefore not be the same as when the original GET
 * request was made with webclient_get(). This function is used for
 * obtaining the current port number.
 *
 * Return:  The port number of the current HTTP data stream, in host byte order.
 */

unsigned short webclient_port(void);

#endif /* __WEBCLIENT_H__ */
