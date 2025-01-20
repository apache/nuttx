/****************************************************************************
 * libs/libc/gdbstub/lib_gdbstub.c
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

#include <ctype.h>
#include <elf.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <syslog.h>
#include <sys/param.h>
#include <sys/types.h>

#include <nuttx/nuttx.h>
#include <nuttx/sched.h>
#include <nuttx/ascii.h>
#include <nuttx/gdbstub.h>
#include <nuttx/memoryregion.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIB_GDBSTUB_DEBUG
#  define GDB_DEBUG(...) syslog(LOG_DEBUG, ##__VA_ARGS__)
#  define GDB_ASSERT() __assert(__FILE__, __LINE__, 0)
#else
#  define GDB_DEBUG(...)
#  define GDB_ASSERT() do {} while (0)
#endif

#define BUFSIZE CONFIG_LIB_GDBSTUB_PKTSIZE

#ifdef CONFIG_BOARD_MEMORY_RANGE
static const struct memory_region_s g_memory_region[] =
  {
    CONFIG_BOARD_MEMORY_RANGE
  };
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gdb_state_s
{
  gdb_send_func_t send;                   /* Send buffer to gdb */
  gdb_recv_func_t recv;                   /* Recv buffer from gdb */
  FAR void *priv;                         /* Private data for transport */
  gdb_monitor_func_t monitor;             /* Monitor will be called when gdb
                                           * receive a monitor command
                                           */
  int last_stopreason;                    /* Last stop reason */
  FAR void *last_stopaddr;                /* Last stop address */
  pid_t pid;                              /* Gdb current thread */
  FAR char *pkt_next;                     /* Pointer to next byte in packet */
  char pkt_buf[BUFSIZE];                  /* Packet buffer */
  size_t pkt_len;                         /* Packet send and receive length */
  uint8_t running_regs[XCPTCONTEXT_SIZE]; /* Registers of running thread */
  size_t size;                            /* Size of registers */
  uintptr_t registers[0];                 /* Registers of other threads */
};

struct gdb_debugpoint_s
{
  int type;
  FAR void *addr;
  size_t size;
  debug_callback_t callback;
  FAR void *arg;
};

typedef CODE ssize_t (*gdb_format_func_t)(FAR void *buf, size_t buf_len,
                                          FAR const void *data,
                                          size_t data_len);

/* Calculate remaining space in packet from ptr_next position. */

#define gdb_remaining_len(state) ((state)->pkt_len - \
                                  ((state)->pkt_next - (state)->pkt_buf))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* System functions, supported by all stubs */

static int gdb_getchar(FAR struct gdb_state_s *state);
static int gdb_putchar(FAR struct gdb_state_s *state, int ch);

/* Packet functions */

static int gdb_send_packet(FAR struct gdb_state_s *state);
static int gdb_recv_packet(FAR struct gdb_state_s *state);
static int gdb_checksum(FAR const char *buf, size_t len);
static int gdb_recv_ack(FAR struct gdb_state_s *state);

/* Data format */

static ssize_t gdb_bin2hex(FAR void *buf, size_t buf_len,
                           FAR const void *data, size_t data_len);
static ssize_t gdb_hex2bin(FAR void *buf, size_t buf_len,
                           FAR const void *data, size_t data_len);
static ssize_t gdb_bin2bin(FAR void *buf, size_t buf_len,
                           FAR const void *data, size_t data_len);
static size_t gdb_encode_rle(FAR void *data, size_t data_len);

/* Packet creation helpers */

static int gdb_send_ok_packet(FAR struct gdb_state_s *state);
static int gdb_send_error_packet(FAR struct gdb_state_s *state,
                                 unsigned char error);

/* Command functions */

static ssize_t gdb_get_memory(FAR struct gdb_state_s *state,
                              FAR void *buf, size_t buf_len,
                              uintptr_t addr, size_t len,
                              gdb_format_func_t format);
static ssize_t gdb_put_memory(FAR struct gdb_state_s *state,
                              FAR const void *buf, size_t buf_len,
                              uintptr_t addr, size_t len,
                              gdb_format_func_t format);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Packet buffer Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gdb_expect_seperator
 *
 * Description:
 *   Check if the next byte in the packet is a seperator.
 *
 * Input Parameters:
 *   state - The pointer to the GDB state structure.
 *   c     - The expected seperator.
 *
 * Returned Value:
 *   0 on success, -EINVAL on error.
 *
 ****************************************************************************/

static int gdb_expect_seperator(FAR struct gdb_state_s *state, char c)
{
  if (!state->pkt_next || *state->pkt_next != c)
    {
      return -EINVAL;
    }
  else
    {
      state->pkt_next++;
    }

  return 0;
}

/****************************************************************************
 * Name: gdb_expect_integer
 *
 * Description:
 *   Parse an integer argument from the packet.
 *
 * Input Parameters:
 *   state - The pointer to the GDB state structure.
 *   arg   - The parsed integer argument.
 *
 * Returned Value:
 *   0 on success
 *
 ****************************************************************************/

static int gdb_expect_integer(FAR struct gdb_state_s *state,
                              FAR uintptr_t *arg)
{
  size_t len = gdb_remaining_len(state);
  char tmp = state->pkt_next[len];

  state->pkt_next[len] = '\0';
  *arg = strtoul(state->pkt_next, &state->pkt_next, 16);
  state->pkt_next[len] = tmp;
  return 0;
}

/****************************************************************************
 * Name: gdb_expect_addr_lenth
 *
 * Description:
 *   Parse an address and length argument from the packet.
 *
 * Input Parameters:
 *   state - The pointer to the GDB state structure.
 *   addr  - The parsed address.
 *   length - The parsed length.
 *
 * Returned Value:
 *   0 on success, -EINVAL on error.
 *
 ****************************************************************************/

static int gdb_expect_addr_lenth(FAR struct gdb_state_s *state,
                                 FAR uintptr_t *addr, FAR size_t *length)
{
  int ret;

  ret = gdb_expect_integer(state, addr);
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_expect_seperator(state, ',');
  if (ret < 0)
    {
      return ret;
    }

  return gdb_expect_integer(state, length);
}

/****************************************************************************
 * Packet Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gdb_putchar
 *
 * Description:
 *   Put a character to the GDB server.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *   ch      - The character to be sent.
 *
 * Returned Value:
 *   The character sent to the GDB server.
 *
 ****************************************************************************/

static int gdb_putchar(FAR struct gdb_state_s *state, int ch)
{
  char tmp = ch & 0xff;
  ssize_t ret;

  ret = state->send(state->priv, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  return tmp;
}

/****************************************************************************
 * Name: gdb_getchar
 *
 * Description:
 *  Get a character from the GDB server.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   The character read from the GDB server.
 *
 ****************************************************************************/

static int gdb_getchar(FAR struct gdb_state_s *state)
{
  unsigned char tmp;
  ssize_t ret;

  ret = state->recv(state->priv, &tmp, 1);
  if (ret < 0)
    {
      return ret;
    }

  return tmp;
}

/****************************************************************************
 * Name: gdb_send_packet
 *
 * Description:
 *   Transmits a packet of data.
 *   Packets are of the form: $<packet-data>#<checksum>
 *
 * Input Parameters:
 *   state - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0       if the packet was transmitted and acknowledged.
 *   1       if the packet was transmitted but not acknowledged.
 *   Negative value on error.
 *
 ****************************************************************************/

static int gdb_send_packet(FAR struct gdb_state_s *state)
{
  char buf[3];
  char csum;
  int ret;

  ret = gdb_putchar(state, '$'); /* Send packet start */
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_LIB_GDBSTUB_DEBUG
    {
      size_t p;
      GDB_DEBUG("-> ");
      for (p = 0; p < state->pkt_len; p++)
        {
          if (isprint(state->pkt_buf[p]))
            {
              GDB_DEBUG("%c", state->pkt_buf[p]);
            }
          else
            {
              GDB_DEBUG("\\x%02x", state->pkt_buf[p] & 0xff);
            }
        }

      GDB_DEBUG("\n");
    }
#endif

  state->pkt_len = gdb_encode_rle(state->pkt_buf, state->pkt_len);

  /* Send packet data */

  ret = state->send(state->priv, state->pkt_buf, state->pkt_len);
  if (ret < 0)
    {
      return ret;
    }

  /* Send the checksum */

  buf[0] = '#';
  csum = gdb_checksum(state->pkt_buf, state->pkt_len);
  ret = gdb_bin2hex(buf + 1, sizeof(buf) - 1, &csum, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = state->send(state->priv, buf, sizeof(buf));
  if (ret < 0)
    {
      return ret;
    }

  return gdb_recv_ack(state);
}

/****************************************************************************
 * Name: gdb_recv_packet
 *
 * Description:
 *   Receives a packet of data, assuming a 7-bit clean connection.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0          if the packet was received.
 *   -EIO       if the packet was not received.
 *   -EOVERFLOW if the packet was too long.
 *   Negative value on error.
 *
 ****************************************************************************/

static int gdb_recv_packet(FAR struct gdb_state_s *state)
{
  char buf[2];
  char csum;
  int ret;

  /* Wait for packet start */

  do
    {
      while (1)
        {
          ret = gdb_getchar(state);
          if (ret < 0)
            {
              return ret;
            }
          else if (ret == '$')
            {
              /* Detected start of packet. */

              break;
            }
          else if (ret == ASCII_ETX)
            {
              state->pkt_buf[0] = ASCII_ETX;
              state->pkt_len = 1;
              return 0;
            }
        }

      /* Read until checksum */

      state->pkt_len = 0;
      while (1)
        {
          ret = gdb_getchar(state);
          if (ret < 0) /* Error receiving character */
            {
              return ret;
            }
          else if (ret == '#') /* End of packet */
            {
              break;
            }
          else /* Check for space */
            {
              if (state->pkt_len >= sizeof(state->pkt_buf))
                {
                  GDB_DEBUG("packet buffer overflow\n");
                  return -EOVERFLOW;
                }

              /* Store character and update checksum */

              state->pkt_buf[state->pkt_len++] = (char)ret;
            }
        }
    }
  while (state->pkt_len == 0); /* Ignore empty packets */

  ret = state->recv(state->priv, buf, 2); /* Receive the checksum */
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_LIB_GDBSTUB_DEBUG
    {
      size_t p;
      GDB_DEBUG("<- ");
      for (p = 0; p < state->pkt_len; p++)
        {
          if (isprint(state->pkt_buf[p]))
            {
              GDB_DEBUG("%c", state->pkt_buf[p]);
            }
          else
            {
              GDB_DEBUG("\\x%02x", state->pkt_buf[p] & 0xff);
            }
        }

      GDB_DEBUG("\n");
    }
#endif

  ret = gdb_hex2bin(&csum, 1, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  if (csum != gdb_checksum(state->pkt_buf, state->pkt_len)) /* Verify */
    {
      GDB_DEBUG("received packet with bad checksum\n");
      gdb_putchar(state, '-'); /* Send packet nack */
      return -EIO;
    }

  gdb_putchar(state, '+'); /* Send packet ack */
  state->pkt_next = state->pkt_buf;
  return 0;
}

/****************************************************************************
 * Name: gdb_checksum
 *
 * Description:
 *   Calculate 8-bit checksum of a buffer.
 *
 * Input Parameters:
 *   buf - The buffer to checksum.
 *
 * Returned Value:
 *   8-bit checksum
 *
 ****************************************************************************/

static int gdb_checksum(FAR const char *buf, size_t len)
{
  char csum = 0;

  while (len--)
    {
      csum += *buf++;
    }

  return csum;
}

/****************************************************************************
 * Name: gdb_recv_ack
 *
 * Description:
 *   Receive a packet acknowledgment.
 *
 * Input Parameters:
 *   state - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0       if an ACK (+) was received
 *   1       if a NACK (-) was received
 *   -EINVAL if a bad response was received
 *
 ****************************************************************************/

static int gdb_recv_ack(FAR struct gdb_state_s *state)
{
  int response;

  switch (response = gdb_getchar(state)) /* Wait for packet ack */
    {
      case '+': /* Packet acknowledged */
        return 0;
      case '-': /* Packet negative acknowledged */
        return 1;
      default: /* Bad response! */
        GDB_DEBUG("received bad packet response: 0x%2x\n", response);
        return -EINVAL;
    }
}

/****************************************************************************
 * Data Encoding/Decoding
 ****************************************************************************/

/****************************************************************************
 * Name: gdb_bin2hex
 *
 * Description:
 *   Encode data to its hex-value representation in a buffer.
 *
 * Input Parameters:
 *   buf      - The buffer to encode to.
 *   buf_len  - The length of the buffer.
 *   data     - The data to encode.
 *   data_len - The length of the data to encode.
 *
 * Returned Value:
 *    The number of bytes written to buf on success.
 *    -EOVERFLOW if the buffer is too small.
 *
 ****************************************************************************/

static ssize_t gdb_bin2hex(FAR void *buf, size_t buf_len,
                           FAR const void *data, size_t data_len)
{
  FAR const char *in = data;
  FAR char *out = buf;
  size_t pos;

  if (buf_len < data_len * 2)
    {
      return -EOVERFLOW; /* Buffer too small */
    }

  for (pos = 0; pos < data_len; pos++)
    {
      itoa(in[pos] >> 4 & 0xf, out++, 16);
      itoa(in[pos] & 0xf, out++, 16);
    }

  return data_len * 2;
}

/****************************************************************************
 * Name: gdb_hex2bin
 *
 * Description:
 *   Decode data from its hex-value representation to a buffer.
 *
 * Input Parameters:
 *   buf      - The buffer containing the encoded data.
 *   buf_len  - The length of the buffer.
 *   data     - The buffer to store the decoded data.
 *   data_len - The length of the data to decode.
 *
 * Returned Value:
 *   The number of bytes written to data on success.
 *   -EOVERFLOW if the buffer is too small.
 *   Negative value on error.
 *
 ****************************************************************************/

static ssize_t gdb_hex2bin(FAR void *buf, size_t buf_len,
                           FAR const void *data, size_t data_len)
{
  FAR const char *in = data;
  FAR char *out = buf;
  size_t pos;

  if (buf_len * 2 < data_len)
    {
      return -EOVERFLOW; /* Buffer too small */
    }

  for (pos = 0; pos < data_len; pos += 2)
    {
      char ch[3] =
        {
          in[pos], in[pos + 1], 0
        };

      set_errno(0);
      out[pos / 2] = strtoul(ch, NULL, 16); /* Decode high nibble */
      if (out[pos / 2] == 0 && get_errno())
        {
          GDB_ASSERT();
          return -get_errno(); /* Buffer contained junk. */
        }
    }

  return data_len / 2;
}

/****************************************************************************
 * Name: gdb_bin2bin
 *
 * Description:
 *   Decode data from its bin-value representation to a buffer.
 *
 * Input Parameters:
 *   buf      - The buffer containing the encoded data.
 *   buf_len  - The length of the buffer.
 *   data     - The buffer to store the decoded data.
 *   data_len - The length of the data to decode.
 *
 * Returned Value:
 *     The number of bytes written to data on success.
 *    -EOVERFLOW if the output buffer is too small.
 *    -EINVAL if the input buffer is invalid.
 *
 ****************************************************************************/

static ssize_t gdb_bin2bin(FAR void *buf, size_t buf_len,
                           FAR const void *data, size_t data_len)
{
  FAR const char *in = data;
  FAR char *out = buf;
  size_t out_pos = 0;
  size_t in_pos;

  for (in_pos = 0; in_pos < data_len; in_pos++)
    {
      if (out_pos >= buf_len)
        {
          GDB_ASSERT();
          return -EOVERFLOW; /* Output buffer overflow */
        }

      if (in[in_pos] == '}') /* The next byte is escaped! */
        {
          if (in_pos + 1 >= data_len)
            {
              /* There's an escape character, but no escaped character
               * following the escape character.
               */

              GDB_ASSERT();
              return -EINVAL;
            }

          in_pos++;
          out[out_pos++] = in[in_pos] ^ 0x20;
        }
      else
        {
          out[out_pos++] = in[in_pos];
        }
    }

  return out_pos;
}

/****************************************************************************
 * Name: gdb_count_repeat
 *
 * Description:
 *   Get how many bytes are repeated in coming data.
 *
 * Input Parameters:
 *   data     - The buffer containing the encoded data.
 *   data_len - The length of the data to decode.
 *
 * Returned Value:
 *     The number of bytes repeated.
 *
 ****************************************************************************/

static size_t gdb_count_repeat(FAR const char *data, size_t data_len)
{
  char c = data[0];
  size_t i = 1;

  while (i < data_len && data[i] == c)
    {
      i++;
    }

  return i;
}

/****************************************************************************
 * Name: gdb_encode_rle
 *
 * Description:
 *   Encode data in place using GDB RLE encoding.
 *
 * Input Parameters:
 *   data     - The buffer containing the encoded data.
 *   data_len - The length of the data to decode.
 *
 * Returned Value:
 *     The number of bytes written to data on success.
 *
 ****************************************************************************/

static size_t gdb_encode_rle(FAR void *data, size_t data_len)
{
  static const int max_count = 127 - 29;
  FAR char *buf = data;
  size_t widx = 0;
  size_t ridx = 0;

  while (ridx < data_len)
    {
      size_t count = gdb_count_repeat(buf + ridx, data_len - ridx);
      char c = buf[ridx];

      ridx += count;
      while (count >= max_count)
        {
          buf[widx++] = c;
          buf[widx++] = '*';
          buf[widx++] = max_count - 1 + 29;
          count -= max_count;
        }

      if (count <= 3)
        {
          while (count > 0)
            {
              buf[widx++] = c;
              count--;
            }

          continue;
        }

      buf[widx++] = c;
      count--;
      if (count + 29 == '$')
        {
          buf[widx++] = c;
          buf[widx++] = c;
          count -= 2;
        }
      else if (count + 29 == '#')
        {
          buf[widx++] = c;
          count -= 1;
        }

      buf[widx++] = '*';
      buf[widx++] = count + 29;
    }

  return widx;
}

/****************************************************************************
 * Name: gdb_is_valid_region
 * Description:
 *   Check if the address is in the memory region.
 *
 ****************************************************************************/

static bool gdb_is_valid_region(FAR struct gdb_state_s *state,
                                uintptr_t addr, size_t len, uint32_t flags)
{
#ifdef CONFIG_BOARD_MEMORY_RANGE
  FAR const struct memory_region_s *region = g_memory_region;

  while (region->start < region->end)
    {
      if (addr >= region->start &&
          addr <= region->end - len &&
          (region->flags & flags) == flags)
        {
          return true;
        }

      region++;
    }

  return false;
#else
  return true;
#endif
}

/****************************************************************************
 * Command Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gdb_get_memory
 *
 * Description:
 *  Get data from memory.
 *
 * Input Parameters:
 *   state   - The state structure.
 *   buf     - The buffer to store the encoded data.
 *   buf_len - The length of the buffer.
 *   addr    - The address to read from.
 *   len     - The length of the data to read.
 *   format  - The formatting function to use.
 *
 * Returned Value:
 *   The number of bytes written to buf on success.
 *   Negative value on error.
 *
 ****************************************************************************/

static ssize_t gdb_get_memory(FAR struct gdb_state_s *state,
                              FAR void *buf, size_t buf_len,
                              uintptr_t addr, size_t len,
                              gdb_format_func_t format)
{
  ssize_t ret = -EINVAL;

  if (gdb_is_valid_region(state, addr, len, PF_R))
    {
      return format(buf, buf_len, (FAR const void *)addr, len);
    }

  return ret;
}

/****************************************************************************
 * Name: gdb_put_memory
 *
 * Description:
 *   Put data from buf into memory.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *   buf     - The buffer containing the encoded data.
 *   buf_len - The length of the buffer.
 *   addr    - The address to write to.
 *   len     - The length of the data to write.
 *   format  - The formatting function to use.
 *
 * Returned Value:
 *   The number of bytes written to addr on success.
 *   Negative value on error.
 *
 ****************************************************************************/

static ssize_t gdb_put_memory(FAR struct gdb_state_s *state,
                              FAR const void *buf, size_t buf_len,
                              uintptr_t addr, size_t len,
                              gdb_format_func_t format)
{
  ssize_t ret = -EINVAL;

  if (gdb_is_valid_region(state, addr, len, PF_W))
    {
      return format((FAR void *)addr, len, buf, buf_len);
    }

  return ret;
}

/****************************************************************************
 * Packet Creation Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: gdb_send_ok_packet
 *
 * Description:
 *   Send OK packet.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *    Zero on success.
 *    Negative value on error.
 *
 ****************************************************************************/

static int gdb_send_ok_packet(FAR struct gdb_state_s *state)
{
  state->pkt_buf[0] = 'O';
  state->pkt_buf[1] = 'K';
  state->pkt_len = 2;

  return gdb_send_packet(state);
}

/****************************************************************************
 * Name: gdb_send_signal_packet
 *
 * Description:
 *   Send a signal packet (S AA).
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *   signal  - The signal to send.
 *
 * Returned Value:
 *   Zero on success.
 *   Negative value on error.
 *
 ****************************************************************************/

static int gdb_send_signal_packet(FAR struct gdb_state_s *state,
                                  unsigned char signal)
{
  int ret;

  state->pkt_buf[0] = 'S';
  ret = gdb_bin2hex(&state->pkt_buf[1], sizeof(state->pkt_buf) - 1,
                    &signal, 1);
  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = 1 + ret;
  return gdb_send_packet(state);
}

/****************************************************************************
 * Name: gdb_send_error_packet
 *
 * Description:
 *   Send a error packet (E AA).
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *   error   - The error to send.
 *
 * Returned Value:
 *   Zero on success.
 *   Negative value on error.
 *
 ****************************************************************************/

static int gdb_send_error_packet(FAR struct gdb_state_s *state,
                                 unsigned char error)
{
  int ret;

  state->pkt_buf[0] = 'E';
  ret = gdb_bin2hex(&state->pkt_buf[1], sizeof(state->pkt_buf) - 1,
                    &error, 1);
  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = 1 + ret;
  return gdb_send_packet(state);
}

/****************************************************************************
 * Communication Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gdb_get_registers
 *
 * Description:
 *   Get the registers of the specified task.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 ****************************************************************************/

static void gdb_get_registers(FAR struct gdb_state_s *state)
{
  FAR struct tcb_s *tcb = nxsched_get_tcb(state->pid);
  FAR uint8_t *reg;
  int i;

  reg = (FAR uint8_t *)tcb->xcp.regs;
  if (state->pid == _SCHED_GETTID())
    {
      if (up_interrupt_context())
        {
          reg = (FAR uint8_t *)running_regs();
        }
      else
        {
          up_saveusercontext(state->running_regs);
          reg = state->running_regs;
        }
    }

  for (i = 0; i < state->size / sizeof(uintptr_t); i++)
    {
      if (g_tcbinfo.reg_off.p[i] == UINT16_MAX)
        {
          state->registers[i] = 0;
        }
      else
        {
          state->registers[i] =
            *(FAR uintptr_t *)(reg + g_tcbinfo.reg_off.p[i]);
        }
    }
}

/****************************************************************************
 * Name: gdb_read_registers
 *
 * Description:
 *   Read Registers Command Format: g.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   Zero on success.
 *   Negative value on error.
 *
 * Note: Comand Format: g.
 *       Response Format: xxxxxxxxyyyyyyyyy...
 ****************************************************************************/

static int gdb_read_registers(FAR struct gdb_state_s *state)
{
  int ret;

  gdb_get_registers(state);
  ret = gdb_bin2hex(state->pkt_buf, sizeof(state->pkt_buf),
                    state->registers, state->size);
  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = ret;
  gdb_send_packet(state);

  return 0;
}

/****************************************************************************
 * Name: gdb_write_registers
 *
 * Description:
 *   Write Registers Command Format: G XX...
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   Zero on success.
 *   Negative value on error.
 *
 *   Note: This function is not really change the register values.
 *         Comand Format: Gxxxxxxxxyyyyyyyyy
 *         Response Format: OK
 ****************************************************************************/

static int gdb_write_registers(FAR struct gdb_state_s *state)
{
  int ret;

  ret = gdb_hex2bin(state->registers, sizeof(state->registers),
                    state->pkt_buf + 1, state->pkt_len - 1);
  if (ret < 0)
    {
      return ret;
    }

  gdb_send_ok_packet(state);
  return 0;
}

/****************************************************************************
 * Name: gdb_write_register
 *
 * Description:
 *   Write a Register Command Format: P n.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   Zero on success.
 *   Negative value on error.
 *
 * Note: Comand Format: Pn.
 *       Response Format: OK
 ****************************************************************************/

static int gdb_read_register(FAR struct gdb_state_s *state)
{
  uintptr_t addr;
  int ret;

  state->pkt_next++;
  ret = gdb_expect_integer(state, &addr);
  if (ret < 0)
    {
      return ret;
    }

  gdb_get_registers(state);
  if (addr >= state->size)
    {
      return 0;
    }

  ret = gdb_bin2hex(state->pkt_buf, sizeof(state->pkt_buf),
                    &state->registers[addr], sizeof(state->registers[addr]));
  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = ret;
  gdb_send_packet(state);
  return 0;
}

/****************************************************************************
 * Name: gdb_read_memory
 *
 * Description:
 *   Read Memory Command Format: m addr,length
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   The number of bytes read if successful.
 *   Negative value on error.
 *
 * Note: Comand Format: mAAAAAAAAA,LLLLLLLL
 *       Response Format: XXXXXXXXYYYYYYYYY...
 ****************************************************************************/

static int gdb_read_memory(FAR struct gdb_state_s *state)
{
  uintptr_t addr;
  size_t length;
  int ret;

  state->pkt_next++;
  ret = gdb_expect_addr_lenth(state, &addr, &length);
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_get_memory(state, state->pkt_buf, sizeof(state->pkt_buf),
                       addr, length, gdb_bin2hex);
  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = ret;
  gdb_send_packet(state);

  return ret;
}

/****************************************************************************
 * Name: gdb_write_memory
 *
 * Description:
 *   Write Memory Command Format: M addr,length:XX..
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   The number of bytes written if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: MAAAAAAAAA,LLLLLLLL:XXXXXXXXX...
 *        Response Format: OK
 ****************************************************************************/

static int gdb_write_memory(FAR struct gdb_state_s *state)
{
  uintptr_t addr;
  size_t length;
  int ret;

  state->pkt_next++;
  ret = gdb_expect_addr_lenth(state, &addr, &length);
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_put_memory(state, state->pkt_next, gdb_remaining_len(state),
                       addr, length, gdb_hex2bin);
  if (ret < 0)
    {
      return ret;
    }

  gdb_send_ok_packet(state);
  return ret;
}

/****************************************************************************
 * Name: gdb_write_bin_memory
 *
 * Description:
 *   Write Memory (Binary) Command Format: X addr,length:XX..
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   The number of bytes written if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: XAAAAAAAAA,LLLLLLLL:XXXXXXXXX...
 *        Response Format: OK
 ****************************************************************************/

static int gdb_write_bin_memory(FAR struct gdb_state_s *state)
{
  uintptr_t addr;
  size_t length;
  int ret;

  state->pkt_next++;
  ret = gdb_expect_addr_lenth(state, &addr, &length);
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_expect_seperator(state, ':');
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_put_memory(state, state->pkt_next, gdb_remaining_len(state),
                       addr, length, gdb_bin2bin);
  if (ret < 0)
    {
      return ret;
    }

  gdb_send_ok_packet(state);
  return ret;
}

/****************************************************************************
 * Name: gdb_get_thread
 *
 * Description:
 *   Get the all thread id.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 ****************************************************************************/

static void gdb_get_thread(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct gdb_state_s *state = arg;
  int pid = tcb->pid;

  /* Gdb pid start from 1, so add it */

  state->pkt_len += sprintf(&state->pkt_buf[state->pkt_len], "%x,", pid + 1);
}

/****************************************************************************
 * Name: gdb_query
 *
 * Description:
 *  The query packet is used by GDB to request information from the stub.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: qSTRING
 *        STRING:is the query string.
 ****************************************************************************/

static int gdb_query(FAR struct gdb_state_s *state)
{
  static const char thread_extra_info[] = "ThreadExtraInfo";
  static const char fthread_info[] = "fThreadInfo";
  static const char sthread_info[] = "sThreadInfo";
  static const char supported_info[] = "Supported";
  static const char r_cmd[] = "Rcmd";

  if (memcmp(&state->pkt_buf[1], fthread_info,
              sizeof(fthread_info) - 1) == 0)
    {
      state->pkt_buf[0] = 'm';
      state->pkt_len = 1;
      nxsched_foreach(gdb_get_thread, state);
      state->pkt_len--;
      state->pkt_buf[state->pkt_len] = 0;
      gdb_send_packet(state);
    }
  else if (memcmp(&state->pkt_buf[1], sthread_info,
                   sizeof(sthread_info) - 1) == 0)
    {
      state->pkt_buf[0] = 'l';
      state->pkt_len = 1;
      gdb_send_packet(state);
    }
  else if (memcmp(&state->pkt_buf[1], supported_info,
                  sizeof(supported_info) - 1) == 0)
    {
#ifdef CONFIG_ARCH_HAVE_DEBUG
      state->pkt_len = sprintf(state->pkt_buf,
                               "hwbreak+;PacketSize=%zx",
                               sizeof(state->pkt_buf));
#else
      state->pkt_len = sprintf(state->pkt_buf,
                               "PacketSize=%zx",
                               sizeof(state->pkt_buf));
#endif
      gdb_send_packet(state);
    }
  else if (memcmp(&state->pkt_buf[1], thread_extra_info,
                   sizeof(thread_extra_info) - 1) == 0)
    {
      FAR struct tcb_s *tcb;
      char thread_state[32];
      char thread_info[128];
      uintptr_t pid;
      int ret;

      state->pkt_next += sizeof(thread_extra_info) + 1;
      ret = gdb_expect_integer(state, &pid);
      if (ret < 0)
        {
          return ret;
        }

      tcb = nxsched_get_tcb(pid - 1);
      if (tcb == NULL)
        {
          return -EINVAL;
        }

      nxsched_get_stateinfo(tcb, thread_state, sizeof(thread_state));
      snprintf(thread_info, sizeof(thread_info),
               "Name: %s, State: %s, Priority: %d, Stack: %zu",
                get_task_name(tcb), thread_state, tcb->sched_priority,
                tcb->adj_stack_size);

      ret = gdb_bin2hex(state->pkt_buf, sizeof(state->pkt_buf),
                        thread_info, strlen(thread_info));

      state->pkt_len = ret;
      gdb_send_packet(state);
    }
  else if (state->monitor != NULL &&
           memcmp(&state->pkt_buf[1], r_cmd, sizeof(r_cmd) - 1) == 0)
    {
      ssize_t len = state->pkt_len - sizeof(r_cmd) - 1; /* skip the 'Rcmd,' */
      char cmd[128];
      int ret;

      len = gdb_hex2bin(cmd, len / 2,
                        &state->pkt_buf[sizeof(r_cmd) + 1], len);
      if (len < 0)
        {
          return len;
        }

      cmd[len] = '\0';
      ret = state->monitor(state, cmd);
      if (ret < 0)
        {
          return ret;
        }
      else
        {
          gdb_send_ok_packet(state);
        }
    }
  else
    {
      return -EPROTONOSUPPORT;
    }

  return 0;
}

/****************************************************************************
 * Name: gdb_is_thread_active
 *
 * Description:
 *  The is thread active packet is used by GDB to request information from
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: T<id>
 *        id:is the thread id.
 *        Rsponse Format: OK
 ****************************************************************************/

static int gdb_is_thread_active(FAR struct gdb_state_s *state)
{
  FAR struct tcb_s *tcb;
  uintptr_t pid;
  int ret;

  state->pkt_next++;
  ret = gdb_expect_integer(state, &pid);
  if (ret < 0)
    {
      return ret;
    }

  tcb = nxsched_get_tcb(pid - 1);
  if (tcb == NULL)
    {
      return -EINVAL;
    }

  state->pid = pid - 1;
  gdb_send_ok_packet(state);
  return ret;
}

/****************************************************************************
 * Name: gdb_thread_context
 *
 * Description:
 *  The thread context packet is used by GDB to request information from
 *  the stub.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: Hg<id>
 *                       Hc-<id>
 *        Rsponse Format: OK
 ****************************************************************************/

static int gdb_thread_context(FAR struct gdb_state_s *state)
{
  FAR struct tcb_s *tcb;
  uintptr_t pid;
  int ret;

  if (state->pkt_buf[1] == 'g')
    {
      state->pkt_next += 2;
    }
  else if  (state->pkt_buf[1] == 'c')
    {
      state->pkt_next += 3;
    }
  else
    {
      return -EINVAL;
    }

  ret = gdb_expect_integer(state, &pid);
  if (ret < 0)
    {
      return ret;
    }

  if (pid != 0)
    {
      tcb = nxsched_get_tcb(pid - 1);
      if (tcb == NULL)
        {
          return -EINVAL;
        }

      state->pid = pid - 1;
    }

  gdb_send_ok_packet(state);
  return 0;
}

/****************************************************************************
 * Name: gdb_send_stop
 *
 * Description:
 *  The stop packet is used by GDB to request information from the stub.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Rsponse Format: T AA n1:r1;n2:r2;...
 *        The program received signal number AA
 *        n is thread id in current.
 *        r is stop reason.
 *
 ****************************************************************************/

static int gdb_send_stop(FAR struct gdb_state_s *state, int stopreason,
                         FAR void *stopaddr)
{
  int ret;

  state->pid = _SCHED_GETTID();
retry:
  switch (stopreason)
    {
      case GDB_STOPREASON_WATCHPOINT_RO:
        ret = sprintf(state->pkt_buf, "T05thread:%x;rwatch:%" PRIxPTR ";",
                      state->pid + 1, (uintptr_t)stopaddr);
        break;
      case GDB_STOPREASON_WATCHPOINT_WO:
        ret = sprintf(state->pkt_buf, "T05thread:%x;awatch:%" PRIxPTR ";",
                      state->pid + 1, (uintptr_t)stopaddr);
        break;
      case GDB_STOPREASON_WATCHPOINT_RW:
        ret = sprintf(state->pkt_buf, "T05thread:%x;watch:%" PRIxPTR ";",
                      state->pid + 1, (uintptr_t)stopaddr);
        break;
      case GDB_STOPREASON_BREAKPOINT:
        ret = sprintf(state->pkt_buf, "T05thread:%x;hwbreak:;",
                      state->pid + 1);
        break;
      case GDB_STOPREASON_STEPPOINT:
        if (state->last_stopreason == GDB_STOPREASON_WATCHPOINT_RW ||
            state->last_stopreason == GDB_STOPREASON_WATCHPOINT_WO)
          {
            stopreason = state->last_stopreason;
            stopaddr = state->last_stopaddr;
            goto retry;
          }

      case GDB_STOPREASON_CTRLC:
      default:
        ret = sprintf(state->pkt_buf, "T05thread:%x;", state->pid + 1);
    }

  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = ret;
  return gdb_send_packet(state);
}

#ifdef CONFIG_ARCH_HAVE_DEBUG

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: gdb_smp_debugpoint_add
 *
 * Description:
 *  Add debug point to all cpu.
 *
 ****************************************************************************/

static int gdb_smp_debugpoint_add(FAR void *arg)
{
  FAR struct gdb_debugpoint_s *point = arg;
  return up_debugpoint_add(point->type, point->addr, point->size,
                           point->callback, point->arg);
}

/****************************************************************************
 * Name: gdb_smp_debugpoint_remove
 *
 * Description:
 *  Remove debug point to all cpu.
 *
 ****************************************************************************/

static int gdb_smp_debugpoint_remove(FAR void *arg)
{
  FAR struct gdb_debugpoint_s *point = arg;
  return up_debugpoint_remove(point->type, point->addr, point->size);
}
#endif

/****************************************************************************
 * Name: gdb_debugpoint_callback
 *
 * Description:
 *  The debugpoint callback is used by GDB to request.
 *
 ****************************************************************************/

static void gdb_debugpoint_callback(int type, FAR void *addr,
                                    size_t size, FAR void *arg)
{
  int stopreason;

  switch (type)
    {
      case DEBUGPOINT_BREAKPOINT:
        stopreason = GDB_STOPREASON_BREAKPOINT;
        break;
      case DEBUGPOINT_WATCHPOINT_RO:
        stopreason = GDB_STOPREASON_WATCHPOINT_RO;
        break;
      case DEBUGPOINT_WATCHPOINT_WO:
        stopreason = GDB_STOPREASON_WATCHPOINT_WO;
        break;
      case DEBUGPOINT_WATCHPOINT_RW:
        stopreason = GDB_STOPREASON_WATCHPOINT_RW;
        break;
      case DEBUGPOINT_STEPPOINT:
        stopreason = GDB_STOPREASON_STEPPOINT;
        gdb_debugpoint_remove(GDB_STOPREASON_STEPPOINT, NULL, 0);
        break;
      default:
        return;
    }

  gdb_process(arg, stopreason, addr);
}

/****************************************************************************
 * Name: gdb_debugpoint
 *
 * Description:
 *   The debugpoint packet is used by GDB to request information from
 *
 * Input Parameters:
 *   state  - The pointer to the GDB state structure.
 *   enable - Enable or disable debugpoint.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: Z/z type,addr,length
 *        Rsponse Format: OK
 *   Z is set breakpoint.
 *   z is clear breakpoint.
 *   type: 0 is software breakpoint.
 *         1 is hardware breakpoint.
 *         2 is write watchpoint.
 *         3 is read watchpoint.
 *         4 is read/write watchpoint.
 *   length: is the length of watchpoint.
 *           if is breakpoint, length is instruction type.
 *
 ****************************************************************************/

static int gdb_debugpoint(FAR struct gdb_state_s *state, bool enable)
{
  uintptr_t type;
  uintptr_t addr;
  uintptr_t size;
  int ret;

  state->pkt_next += 1;
  ret = gdb_expect_integer(state, &type);
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_expect_seperator(state, ',');
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_expect_integer(state, &addr);
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_expect_seperator(state, ',');
  if (ret < 0)
    {
      return ret;
    }

  ret = gdb_expect_integer(state, &size);
  if (ret < 0)
    {
      return ret;
    }

  switch (type)
    {
      case 0: /* just use hardware break */
      case 1:
        type = DEBUGPOINT_BREAKPOINT;
        break;
      case 2:
        type = DEBUGPOINT_WATCHPOINT_WO;
        break;
      case 3:
        type = DEBUGPOINT_WATCHPOINT_RO;
        break;
      case 4:
        type = DEBUGPOINT_WATCHPOINT_RW;
        break;
      default:
        return -EPROTONOSUPPORT;
    }

  if (enable)
    {
      ret = gdb_debugpoint_add(type, (FAR void *)addr, size,
                               gdb_debugpoint_callback, state);
    }
  else
    {
      ret = gdb_debugpoint_remove(type, (FAR void *)addr, size);
    }

  if (ret < 0)
    {
      return ret;
    }

  return gdb_send_ok_packet(state);
}

/****************************************************************************
 * Name: gdb_step
 *
 * Description:
 *   The handle step packet is used by GDB to request information from
 *
 * Input Parameters:
 *   state  - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: s
 *        Rsponse Format: OK
 *
 ****************************************************************************/

static int gdb_step(FAR struct gdb_state_s *state)
{
  int ret = gdb_debugpoint_add(GDB_STOPREASON_STEPPOINT, NULL, 0,
                               gdb_debugpoint_callback, state);

  if (ret < 0)
    {
      return ret;
    }

  return gdb_send_ok_packet(state);
}

/****************************************************************************
 * Name: gdb_continue
 *
 * Description:
 *   The handle continue packet is used by GDB to request information from
 *
 * Input Parameters:
 *   state  - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   0  if successful.
 *   Negative value on error.
 *
 * Note : Comand Format: c
 *        Rsponse Format: OK
 *
 ****************************************************************************/

static int gdb_continue(FAR struct gdb_state_s *state)
{
  return gdb_send_ok_packet(state);
}

#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct tcbinfo_s g_tcbinfo;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_DEBUG

/****************************************************************************
 * Name: gdbstub_debugpoint_add
 ****************************************************************************/

int gdb_debugpoint_add(int type, FAR void *addr, size_t size,
                       debug_callback_t callback, FAR void *arg)
{
#ifdef CONFIG_SMP
  struct gdb_debugpoint_s point;
  point.type = type;
  point.addr = addr;
  point.size = size;
  point.callback = callback;
  point.arg = arg;
  return nxsched_smp_call((1 << CONFIG_SMP_NCPUS) - 1,
                          gdb_smp_debugpoint_add, &point);
#else
  return up_debugpoint_add(type, addr, size, callback, arg);
#endif
}

/****************************************************************************
 * Name: gdb_debugpoint_remove
 ****************************************************************************/

int gdb_debugpoint_remove(int type, FAR void *addr, size_t size)
{
#ifdef CONFIG_SMP
  struct gdb_debugpoint_s point;
  point.type = type;
  point.addr = addr;
  point.size = size;

  return nxsched_smp_call((1 << CONFIG_SMP_NCPUS) - 1,
                          gdb_smp_debugpoint_remove, &point);
#else
  return up_debugpoint_remove(type, addr, size);
#endif
}

#endif

/****************************************************************************
 * Name: gdb_state_init
 *
 * Description:
 *   Initialize the GDB state structure.
 *
 * Input Parameters:
 *   send    - The pointer to the send function.
 *   recv    - The pointer to the receive function.
 *   monitor - The pointer to the monitor_cmd function.
 *   priv    - The pointer to the private data.
 *
 * Returned Value:
 *   The pointer to the GDB state structure on success.
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct gdb_state_s *gdb_state_init(gdb_send_func_t send,
                                       gdb_recv_func_t recv,
                                       gdb_monitor_func_t monitor,
                                       FAR void *priv)
{
  size_t size = g_tcbinfo.regs_num * sizeof(uintptr_t);
  FAR struct gdb_state_s *state = lib_zalloc(sizeof(*state) + size);

  if (state == NULL)
    {
      return NULL;
    }

  state->size = size;
  state->send = send;
  state->recv = recv;
  state->priv = priv;
  state->monitor = monitor;

  return state;
}

/****************************************************************************
 * Name: gdb_state_uninit
 *
 * Description:
 *   Uninitialize the GDB state structure.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 ****************************************************************************/

void gdb_state_uninit(FAR struct gdb_state_s *state)
{
  if (state != NULL)
    {
      lib_free(state);
    }
}

/****************************************************************************
 * Name: gdb_console_message
 *
 * Description:
 *   Send a message to the GDB console (via O XX... packet).
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *    Zero on success.
 *    Negative value on error.
 *
 ****************************************************************************/

int gdb_console_message(FAR struct gdb_state_s *state, FAR const char *msg)
{
  ssize_t ret;

  if (state == NULL || msg == NULL)
    {
      return -EINVAL;
    }

  state->pkt_buf[0] = 'O';
  ret = gdb_bin2hex(&state->pkt_buf[1], sizeof(state->pkt_buf) - 1,
                    msg, strlen(msg));
  if (ret < 0)
    {
      return ret;
    }

  state->pkt_len = 1 + ret;
  return gdb_send_packet(state);
}

/****************************************************************************
 * Name: gdb_process
 *
 * Description:
 *   Main debug loop. Handles commands.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   Zero if successful.
 *   Negative value on error.
 *
 ****************************************************************************/

int gdb_process(FAR struct gdb_state_s *state, int stopreason,
                FAR void *stopaddr)
{
  int ret;

  if (stopreason != GDB_STOPREASON_NONE)
    {
      gdb_send_stop(state, stopreason, stopaddr);
    }

  while ((ret = gdb_recv_packet(state)) >= 0)
    {
      /* Handle one letter commands */

      switch (state->pkt_buf[0])
        {
          case '?': /* gdbserial status */
            ret = gdb_send_signal_packet(state, 0x00);
            break;
          case 'g': /* Read registers */
            ret = gdb_read_registers(state);
            break;
          case 'G': /* Write registers */
            ret = gdb_write_registers(state);
            break;
          case 'p': /* Read one register */
            ret = gdb_read_register(state);
            break;
          case 'm': /* Read memory */
            ret = gdb_read_memory(state);
            break;
          case 'M': /* Write memory */
            ret = gdb_write_memory(state);
            break;
          case 'X': /* Write binary memory */
            ret = gdb_write_bin_memory(state);
            break;
          case 'q': /* Query command */
            ret = gdb_query(state);
            break;
          case 'T': /* Query thread */
            ret = gdb_is_thread_active(state);
            break;
          case 'H': /* Thread related */
            ret = gdb_thread_context(state);
            break;
          case 'k': /* Kill request */
            ret = -ECONNRESET;
            break;
          case ASCII_ETX: /* Ctrl C */
            ret = gdb_send_ok_packet(state);
            break;
#ifdef CONFIG_ARCH_HAVE_DEBUG
          case 'Z': /* Insert breakpoint/watchpoint */
            ret = gdb_debugpoint(state, true);
            break;
          case 'z': /* Remove breakpoint/watchpoint */
            ret = gdb_debugpoint(state, false);
            break;
          case 's': /* Single step */
            ret = gdb_step(state);
            if (ret < 0)
              {
                break;
              }

            goto out;
          case 'c': /* Continue */
            ret = gdb_continue(state);
            if (ret < 0)
              {
                break;
              }

            goto out;
#endif
          default:
            ret = -EPROTONOSUPPORT;
      }

      if (ret == -EPROTONOSUPPORT)
        {
          state->pkt_len = 0;
          gdb_send_packet(state);
        }
      else if (ret == -ECONNRESET)
        {
          break;
        }
      else if (ret < 0)
        {
          gdb_send_error_packet(state, 0x00);
        }
    }

#ifdef CONFIG_ARCH_HAVE_DEBUG
out:
#endif
  state->last_stopreason = stopreason;
  state->last_stopaddr = stopaddr;
  return ret;
}
