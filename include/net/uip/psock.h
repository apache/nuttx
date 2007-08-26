/* psock.h
 * Protosocket library header file
 * Author: Adam Dunkels <adam@sics.se>
 *
 * Copyright (c) 2004, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* psock Protosockets library
 *
 * The protosocket library provides an interface to the uIP stack that is
 * similar to the traditional BSD socket interface. Unlike programs
 * written for the ordinary uIP event-driven interface, programs
 * written with the protosocket library are executed in a sequential
 * fashion and does not have to be implemented as explicit state
 * machines.
 *
 * Protosockets only work with TCP connections.
 *
 * The protosocket library uses \ref pt protothreads to provide
 * sequential control flow. This makes the protosockets lightweight in
 * terms of memory, but also means that protosockets inherits the
 * functional limitations of protothreads. Each protosocket lives only
 * within a single function. Automatic variables (stack variables) are
 * not retained across a protosocket library function call.
 *
 * \note Because the protosocket library uses protothreads, local
 * variables will not always be saved across a call to a protosocket
 * library function. It is therefore advised that local variables are
 * used with extreme care.
 *
 * The protosocket library provides functions for sending data without
 * having to deal with retransmissions and acknowledgements, as well
 * as functions for reading data without having to deal with data
 * being split across more than one TCP segment.
 *
 * Because each protosocket runs as a protothread, the protosocket has to be
 * started with a call to PSOCK_BEGIN() at the start of the function
 * in which the protosocket is used. Similarly, the protosocket protothread can
 * be terminated by a call to PSOCK_EXIT().
 *
 */

#ifndef __PSOCK_H__
#define __PSOCK_H__

#include <sys/types.h>
#include <net/uip/uipopt.h>

/* The structure that holds the state of a buffer.
 *
 * This structure holds the state of a uIP buffer. The structure has
 * no user-visible elements, but is used through the functions
 * provided by the library.
 *
 */

struct psock_buf
{
  uint8 *ptr;
  unsigned short left;
};

/* The representation of a protosocket.
 *
 * The protosocket structrure is an opaque structure with no user-visible
 * elements.
 */

struct psock
{
  const uint8 *sendptr;    /* Pointer to the next data to be sent. */
  uint8 *readptr;          /* Pointer to the next data to be read. */
  uint8 *bufptr;           /* Pointer to the buffer used for buffering incoming data. */
  uint16 sendlen;          /* The number of bytes left to be sent. */
  uint16 readlen;          /* The number of bytes left to be read. */
  struct psock_buf buf;  /* The structure holding the state of the input buffer. */
  unsigned int bufsize;    /* The size of the input buffer. */
  unsigned char state;     /* The state of the protosocket. */
};

/* Initialize a protosocket.
 *
 * Initializes a protosocket and must be called before the
 * protosocket is used. The initialization also specifies the input buffer
 * for the protosocket.
 *
 * psock (struct psock *) A pointer to the protosocket to be
 * initialized
 *
 * buffer (char *) A pointer to the input buffer for the
 * protosocket.
 *
 * buffersize (unsigned int) The size of the input buffer.
 */

extern void psock_init(struct psock *psock, char *buffer, unsigned int buffersize);

/* Send data.
 *
 * This macro sends data over a protosocket. The protosocket protothread blocks
 * until all data has been sent and is known to have been received by
 * the remote end of the TCP connection.
 *
 * psock (struct psock *) A pointer to the protosocket over which
 * data is to be sent.
 *
 * data (char *) A pointer to the data that is to be sent.
 *
 * datalen (unsigned int) The length of the data that is to be
 * sent.
 */

extern void psock_send(struct psock *psock, const char *buf, unsigned int len);

/*Send a null-terminated string.
 *
 * psock Pointer to the protosocket.
 * str  The string to be sent.
 *
 *             This function sends a null-terminated string over the
 *             protosocket.
 */

#define PSOCK_SEND_STR(psock, str) psock_send(psock, str, strlen(str))

/* Generate data with a function and send it
 *
 * psock Pointer to the protosocket.
 * generator Pointer to the generator function
 * arg   Argument to the generator function
 *
 *             This function generates data and sends it over the
 *             protosocket. This can be used to dynamically generate
 *             data for a transmission, instead of generating the data
 *             in a buffer beforehand. This function reduces the need for
 *             buffer memory. The generator function is implemented by
 *             the application, and a pointer to the function is given
 *             as an argument with the call to PSOCK_GENERATOR_SEND().
 *
 *             The generator function should place the generated data
 *             directly in the uip_appdata buffer, and return the
 *             length of the generated data. The generator function is
 *             called by the protosocket layer when the data first is
 *             sent, and once for every retransmission that is needed.
 */

extern void psock_generator_send(struct psock *psock, unsigned short (*f)(void *), void *arg);

/* Close a protosocket.
 *
 * This macro closes a protosocket and can only be called from within the
 * protothread in which the protosocket lives.
 *
 * psock (struct psock *) A pointer to the protosocket that is to
 * be closed.
 */

#define PSOCK_CLOSE(psock) uip_close()

/* Read data until the buffer is full.
 *
 * This macro will block waiting for data and read the data into the
 * input buffer specified with the call to PSOCK_INIT(). Data is read
 * until the buffer is full..
 *
 * psock (struct psock *) A pointer to the protosocket from which
 * data should be read.
 */

extern void psock_readbuf(struct psock *psock);

/* Read data up to a specified character.
 *
 * This macro will block waiting for data and read the data into the
 * input buffer specified with the call to PSOCK_INIT(). Data is only
 * read until the specifieed character appears in the data stream.
 *
 * psock (struct psock *) A pointer to the protosocket from which
 * data should be read.
 *
 * c (char) The character at which to stop reading.
 */

extern void psock_readto(struct psock *psock, unsigned char c);

/* The length of the data that was previously read.
 *
 * Returns the length of the data that was previously read
 * using PSOCK_READTO() or PSOCK_READ().
 *
 * psock (struct psock *) A pointer to the protosocket holding the data.
 */

extern uint16 psock_datalen(struct psock *psock);

/* Check if there is new data has arrived on a protosocket without blocking
 *
 * psock (struct psock *) A pointer to the protosocket.
 */

extern boolean psock_checknewdata(struct psock *s);

/* Block until new data has arrived on a protosocket.
 *
 * psock (struct psock *) A pointer to the protosocket.
 */

extern void psock_waitnewdata(struct psock *s);

#endif /* __PSOCK_H__ */

/** @} */
