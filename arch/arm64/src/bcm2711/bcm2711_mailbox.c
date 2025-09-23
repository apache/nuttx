/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_mailbox.c
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

/* NOTE: this module is written with the expectation that all requests are
 * coming from the ARM core, since that is where NuttX runs. As such, mailbox
 * 0 is used only to read responses from the VC, while mailbox 1 is used
 * only to write requests from the ARM core to the VC.
 *
 * TODO: The VideoCore mailbox API is capable of asynchronous
 * request/response. It is not guaranteed that requests will be responded to
 * in the same order they were went. In order to save myself the headache of
 * working around this, I've implemented each request/response interaction as
 * a single atomic transaction. All other callers must wait for the lock in
 * order to get their turn. This is not the most efficient, but it allows me
 * to move on to other features and the slow-down should be very negligible,
 * at least for the current use-case of getting property tags.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/cache.h>
#include <nuttx/config.h>

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "arm64_internal.h"
#include "bcm2711_mailbox.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Aliases for mailboxes so it is difficult to accidentally violate r/w
 * constraints.
 */

#define READ_MBOX BCM_VC_MBOX0
#define WRITE_MBOX BCM_VC_MBOX1

/* Power status get tag uses bit 1 to indicate if the device ID exists */

#define MBOX_DEVICE_DNE (1 << 1)

/* Read and write timeouts in seconds */

#define R_TIMEOUT (5)
#define W_TIMEOUT (3)

/* Constants to keep track of field count for determining buffer sizes */

#define TAG_FIELDS (3) /* See bcm2711_mbox_tag_s fields */
#define BUF_FIELDS (3) /* Size, request flags, taglist terminator */

/* Helper for special buffer alignment */

#define ALIGNED_MBOX __attribute__((aligned(16)))

/* Helpers */

#define mbox_full(box)                                                       \
  (getreg32(BCM_VC_MBOX_STATUS((box))) & BCM_VC_MBOX_STATUS_FULL)
#define mbox_empty(box)                                                      \
  (getreg32(BCM_VC_MBOX_STATUS((box))) & BCM_VC_MBOX_STATUS_EMPTY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Represents the format of a mailbox tag */

struct bcm2711_mbox_tag_s
{
  uint32_t tagid;   /* Tag identifier */
  uint32_t vbufsiz; /* Value buffer size in bytes */
  uint32_t code;    /* Request/response code */

  /* NOTE: Following this structure there should be the value buffer of size
   * `vbufsiz`
   */
};

/* Contains necessary state for operating interrupt-driven API */

struct bcm2711_mbox_s
{
  mutex_t lock;  /* Lock for atomic request/response interactions */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bcm2711_mbox_s g_mbox =
{
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcm2711_mbox_rword(uint8_t channel, uint32_t *result);
static int bcm2711_mbox_wword(uint8_t channel, uint32_t data);
static void bcm2711_mbox_makereq(uint32_t tag, FAR void *buf, uint32_t nval,
                                 uint32_t nbuf);
static int bcm2711_mbox_sendreq(FAR uint32_t *buf, uint8_t n);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_mbox_rword
 *
 * Description:
 *   Read a single word from the mailbox that matches `channel`
 *   WARNING: Data read not from the expected channel is discarded. For now
 *   this is okay, since the implementation _only_ considers reading the
 *   property tags.
 *
 * Input parameters:
 *   channel - The mailbox channel data should come from
 *   result - Where to store the value returned from the mailbox
 *
 * Returned Value:
 *    0 on success, -ETIMEDOUT if it takes over 3 seconds to see space in the
 *    mailbox
 ****************************************************************************/

static int bcm2711_mbox_rword(uint8_t channel, uint32_t *result)
{
  uint32_t data;
  int usecs_waited = 0;

  for (; ; )
    {
      /* Wait for mailbox to have data */

      while (mbox_empty(READ_MBOX))
        {
          up_udelay(10);
          usecs_waited += 10;
          if (usecs_waited >= R_TIMEOUT * 1000000)
            {
              ipcerr("Read timed out.");
              return -ETIMEDOUT;
            }
        }

      data = getreg32(BCM_VC_MBOX_RW(READ_MBOX));

      /* Channel matches, return word! */

      if ((data & BCM_VC_MBOX_RW_CHAN_MASK) == channel)
        {
          *result = data & ~BCM_VC_MBOX_RW_CHAN_MASK;
          return 0;
        }

      /* No channel match, so we continue until it does. */
    }
}

/****************************************************************************
 * Name: bcm2711_mbox_wword
 *
 * Description:
 *   Write a single word to the mailbox channel `channel`
 *
 * Input parameters:
 *   channel - The mailbox channel data will be written to
 *   data - The data to write to the mailbox
 *
 * Returned Value:
 *    0 on success, -ETIMEDOUT if it takes over 3 seconds to see space in the
 *    mailbox
 ****************************************************************************/

static int bcm2711_mbox_wword(uint8_t channel, uint32_t data)
{
  int usecs_waited = 0;

  /* Wait for space when the mailbox is full */

  while (mbox_full(WRITE_MBOX))
    {
      up_udelay(10);
      usecs_waited += 10;
      if (usecs_waited >= W_TIMEOUT * 1000000)
        {
          ipcerr("Read timed out.");
          return -ETIMEDOUT;
        }
    }

  /* Mask excess channel bits */

  channel &= BCM_VC_MBOX_RW_CHAN_MASK;

  /* Write the data */

  putreg32((data & (~0xf)) | channel, BCM_VC_MBOX_RW(WRITE_MBOX));
  return 0;
}

/****************************************************************************
 * Name: bcm2711_mbox_sendreq
 *
 * Description:
 *   Send a request to the VideoCore through the mailbox. This is atomic
 *   (other requests must wait for this one to complete).
 *   WARNING: The `buf` parameter must be 16-byte aligned and not NULL.
 *
 * Input parameters:
 *   buf  - A buffer containing enough space for the tag (12 bytes), buffer
 *          size indicator (4 bytes), buffer request code (4 bytes), end tag
 *          (4 bytes) and the maximum size of the response length or the tag
 *          value length (tag dependent).
 *
 * Returned Value:
 *   0 on success, -EIO on failure. Results are inside the buffer.
 ****************************************************************************/

static int bcm2711_mbox_sendreq(FAR uint32_t *buf, uint8_t n)
{
  int res;
  uint32_t retbuf;
  uint32_t bufptr = (uint32_t)(uintptr_t)(buf);

  /* Write the buffer to the VideoCore */

  up_flush_dcache(bufptr, bufptr + n);
  nxmutex_lock(&g_mbox.lock);
  res = bcm2711_mbox_wword(MBOX_CHAN_TAGS, bufptr);
  if (res < 0)
    {
      nxmutex_unlock(&g_mbox.lock);
      return res;
    }

  /* Get the response from the VideoCore.
   *
   * Before we check the buffer contents, we must invalidate the data cache
   * on this buffer's contents or we'll just read back what we sent without
   * modification. This is done before the 'write' to provide time for the
   * cache to flush.
   */

  res = bcm2711_mbox_rword(MBOX_CHAN_TAGS, &retbuf);
  nxmutex_unlock(&g_mbox.lock);
  if (res < 0)
    {
      return res;
    }

  /* The mailbox _MUST_ return the the same buffer address, it is
   * not possible for it to do otherwise (according to raspberrypi/firmware
   * docs).
   */

  if (retbuf != bufptr)
    {
      ipcerr("Expected %08x, got %08x", bufptr, res);
      return -EIO; /* Wrong buffer responded */
    }

  /* Make sure we have a response */

  if (buf[1] == BCM_VC_MBOX_RESP_OK)
    {
      return 0; /* Successful response */
    }
  else if (buf[1] == BCM_VC_MBOX_RESP_ERR)
    {
      /* Had a response error that was reported, partial response */

      ipcerr("Unsuccessful response: %08x", buf[1]);
      return -EIO;
    }

  /* Unknown response code, typically due to unknown tag */

  ipcerr("Unknown response code: %08x", buf[1]);
  return -EINVAL;
}

/****************************************************************************
 * Name: bcm2711_mbox_makereq
 *
 * Description:
 *   Construct a buffer to send to the video core.
 *   WARNING: The `buf` parameter must be 16-byte aligned.
 *
 * Input parameters:
 *   tag  - The tag identifier (MBOX_TAG_*) for the request
 *   buf  - A buffer containing enough space for the tag (12 bytes), buffer
 *          size indicator (4 bytes), buffer request code (4 bytes), end tag
 *          (4 bytes) and the maximum size of the response length or the tag
 *          value length (tag dependent).
 *   nval - The max size of either the value sent or the value expected in
 *          response
 *   nbuf - The total size of the buffer
 *
 ****************************************************************************/

static void bcm2711_mbox_makereq(uint32_t tag, FAR void *buf, uint32_t nval,
                                uint32_t nbuf)
{
  void *bufpos = buf;
  struct bcm2711_mbox_tag_s mtag = {
      .tagid = tag,
      .vbufsiz = nval, /* The value size already in the buffer */
      .code = BCM_VC_MBOX_REQUEST,
  };

  /* Buffer must be large enough to store the buffer size (u32), req/resp
   * code (u32), a series of tags (in this case one), the tag's value size,
   * and the tag terminator (u32).
   */

  DEBUGASSERT(nbuf >= sizeof(mtag) + nval + 3 * sizeof(uint32_t));

  /* Move the value in the buffer back so it starts right after where the
   * buffer fields (2 u32s) and tag will go.
   */

  if (nval != 0)
    {
      memcpy(buf + sizeof(mtag) + 2 * sizeof(uint32_t), buf, nval);
    }

  /* Populate buffer fields */

  ((uint32_t *)(bufpos))[0] = nbuf;                /* The total buf size */
  ((uint32_t *)(bufpos))[1] = BCM_VC_MBOX_REQUEST; /* The req/resp flag */
  bufpos += 2 * sizeof(uint32_t);                  /* Move past buf fields */

  /* The tag is inserted next, following the first two fields */

  memcpy(bufpos, &mtag, sizeof(mtag));
  bufpos += sizeof(mtag); /* Move past tag */

  /* After the tag is `nval` bytes worth of the tag value being passed.
   * Once we skip over those bytes, we can finally add the tag list
   * terminator.
   */

  bufpos += nval; /* Skips tag value */
  *((uint32_t *)(bufpos)) = BCM_VC_MBOX_TAGLIST_END;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_mbox_getcputemp
 *
 * Description:
 *   Gets the temperature of the SoC in millidegrees Celsius.
 *
 * Input parameters:
 *   temp - A pointer to where to store the temperature
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getcputemp(uint32_t *temp)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(temp != NULL);

  buf[0] = 0; /* ID should always be 0 for temp */

  bcm2711_mbox_makereq(MBOX_TAG_GETTEMP, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *temp = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getrev
 *
 * Description:
 *   Gets the board revision.
 *
 * Input parameters:
 *   rev - A pointer to where to store the revision number
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getrev(uint32_t *rev)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  DEBUGASSERT(rev != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_BREV, buf, sizeof(uint32_t), sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *rev = buf[5];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getmodel
 *
 * Description:
 *   Gets the board model number.
 *
 * Input parameters:
 *   model - A pointer to where to store the model number
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getmodel(uint32_t *model)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  DEBUGASSERT(model != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_BMODEL, buf, sizeof(uint32_t), sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *model = buf[5];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getmac
 *
 * Description:
 *   Gets the board MAC address in network byte order
 *
 * Input parameters:
 *   mac - A pointer to where to store the MAC address
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getmac(uint64_t *mac)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(mac != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_MAC, buf, 2 * sizeof(uint32_t), sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *mac = *((uint64_t *)(buf + 5));
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getpwr
 *
 * Description:
 *   Gets the power state of `id`.
 *
 * Input parameters:
 *   id - The device ID to know the power state of
 *   state - A pointer to where to store the state
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getpwr(uint8_t id, bool *state)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;
  buf[0] = id; /* First argument is the device ID */

  DEBUGASSERT(state != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETPWR, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  /* Check if mailbox recognized device */

  if (buf[6] & MBOX_DEVICE_DNE)
    {
      return -EINVAL;
    }

  *state = buf[6] & 0x1;
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_ledset
 *
 * Description:
 *   Sets the state of the LED specified by `pin`
 *
 * Input parameters:
 *   pin - The pin number of the LED to modify the state of
 *   on - True to turn on the LED, false to turn it off
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_ledset(uint8_t pin, bool on)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  buf[0] = pin;
  buf[1] = on;

  bcm2711_mbox_makereq(MBOX_TAG_SETLED, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  return err;
}
