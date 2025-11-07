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
  mutex_t lock; /* Lock for atomic request/response interactions */
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
  return -EAGAIN;
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
 * Name: bcm2711_mbox_setpwr
 *
 * Description:
 *   Sets the power state of `id`.
 *
 * Input parameters:
 *   id - The device ID to know the power state of
 *   on - True for power on, false for power off
 *   wait - True to block until power stable, false otherwise
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setpwr(uint8_t id, bool on, bool wait)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;
  buf[0] = id; /* First argument is the device ID */

  /* Second argument is power state. Bit 0 indicates desired state, bit 1
   * indicates desire to wait for stable power.
   */

  buf[1] = (on ? (1 << 0) : 0x0) | (wait ? (1 << 1) : 0x0);

  bcm2711_mbox_makereq(MBOX_TAG_SETPWR, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  /* Check if mailbox recognized device ID */

  if (buf[6] & MBOX_DEVICE_DNE)
    {
      return -EINVAL;
    }

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

/****************************************************************************
 * Name: bcm2711_mbox_getclken
 *
 * Description:
 *   Get the state of the clock (enabled/disabled) corresponding to `id`
 *
 * Input parameters:
 *   id - The ID of the clock to check
 *   state - Where to store the state of the clock
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getclken(uint8_t id, bool *state)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(state != NULL);

  buf[0] = id; /* Argument is clock ID */
  bcm2711_mbox_makereq(MBOX_TAG_GETCLKS, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  /* Check if clock ID was recognized */

  if (buf[6] & MBOX_DEVICE_DNE)
    {
      return -EINVAL;
    }

  *state = buf[6] & 0x1;
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_setclken
 *
 * Description:
 *   Set the state of the clock (enabled/disabled) corresponding to `id`
 *
 * Input parameters:
 *   id - The ID of the clock
 *   en - True to enable the clock, false otherwise
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setclken(uint8_t id, bool en)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;
  buf[0] = id;         /* First argument is clock ID */
  buf[1] = en ? 1 : 0; /* Second arg is clock state */

  bcm2711_mbox_makereq(MBOX_TAG_SETCLKS, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  /* Check if clock ID was recognized */

  if (buf[6] & MBOX_DEVICE_DNE)
    {
      return -EINVAL;
    }

  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getclkrate
 *
 * Description:
 *   Get the rate of the clock corresponding to `id` in Hz.
 *
 * Input parameters:
 *   id - The ID of the clock
 *   rate - Where to store the rate of the clock
 *   measured - True to return clock rate as a measured value (true rate),
 *              false for configured rate.
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getclkrate(uint8_t id, uint32_t *rate, bool measured)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;
  buf[0] = id; /* Argument is clock ID */

  DEBUGASSERT(rate != NULL);

  bcm2711_mbox_makereq(measured ? MBOX_TAG_GETCLKRM : MBOX_TAG_GETCLKR, buf,
                       2 * sizeof(uint32_t), sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  /* Returned clock rate is 0 when the clock does not exist (unless
   * measured). If the clock is not enabled, the return value for
   * unmeasured clock rate is the rate the clock will have when enabled.
   */

  if (buf[6] == 0 && !measured)
    {
      return -EINVAL;
    }

  *rate = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_setclkrate
 *
 * Description:
 *   Set the rate of the clock corresponding to `id` in Hz.
 *
 * Input parameters:
 *   id - The ID of the clock
 *   rate - The desired clock rate in Hz. The set rate will be returned in
 *          this variable, even if the clock is not enabled
 *   turbo - True to allow turbo settings (voltage, sdram and gpu), false
 *           otherwise
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setclkrate(uint8_t id, uint32_t *rate, bool turbo)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 3] ALIGNED_MBOX;

  DEBUGASSERT(rate != NULL);

  buf[0] = id;            /* First arg is clock ID */
  buf[1] = *rate;         /* Second arg is clock rate */
  buf[2] = turbo ? 0 : 1; /* Third arg is 1 to skip turbo */

  bcm2711_mbox_makereq(MBOX_TAG_SETCLKR, buf, 3 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  /* Returned clock rate is 0 when the clock does not exist. */

  if (buf[6] == 0)
    {
      return -EINVAL;
    }

  *rate = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getfb
 *
 * Description:
 *   Allocates a frame buffer and returns it.
 *
 * Input parameters:
 *   fb - A place to store the frame buffer address
 *   size - The frame buffer size in bytes
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getfb(void **fb, uint32_t *size)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(fb != NULL);

  buf[0] = 16; /* Alignment in bytes TODO: should this be variable? */
  buf[1] = 0;

  bcm2711_mbox_makereq(MBOX_TAG_FALLOC, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  *fb = (void *)(unsigned long)VCADDR_TO_ARM(buf[5]);
  *size = buf[6];

  /* Docs say that if size is returned as 0, the alignment was
   * invalid because the frame buffer wasn't allocated.
   */

  if (buf[6] == 0 && !err)
    {
      ipcerr("Frame buffer alignment of 16 bytes invalid.");
      return -EINVAL; /* Invalid alignment */
    }

  /* Frame buffer address needs to be modified since the address is what's
   * seen from the VideoCore memory.
   */

  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_releasefb
 *
 * Description:
 *   Releases the previously allocated frame buffer.
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_releasefb(void)
{
  uint32_t buf[BUF_FIELDS + TAG_FIELDS] ALIGNED_MBOX;
  bcm2711_mbox_makereq(MBOX_TAG_FREL, buf, 0, sizeof(buf));
  return bcm2711_mbox_sendreq(buf, sizeof(buf));
}

/****************************************************************************
 * Name: bcm2711_mbox_getdisp
 *
 * Description:
 *   Get physical display width and height.
 *
 * Input parameters:
 *   x - Width in pixels
 *   y - Height in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getdisp(uint32_t *x, uint32_t *y)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(x != NULL);
  DEBUGASSERT(y != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETDISP, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  *x = buf[5];
  *y = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getdepth
 *
 * Description:
 *   Get the bits per pixel used for the display.
 *
 * Input parameters:
 *   bpp - Bits per pixel
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getdepth(uint32_t *bpp)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  DEBUGASSERT(bpp != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETDEPTH, buf, sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *bpp = buf[5];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getalpha
 *
 * Description:
 *   Get the alpha mode.
 *
 * Input parameters:
 *   alpha - Returned alpha state: 0 enabled, 1 reversed, 2 ignored
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getalpha(uint32_t *alpha)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  DEBUGASSERT(alpha != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETALPHA, buf, sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *alpha = buf[5];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getpitch
 *
 * Description:
 *   Get the number of bytes per line.
 *
 * Input parameters:
 *   bpl - Bytes per line
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getpitch(uint32_t *bpl)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  DEBUGASSERT(bpl != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETPITCH, buf, sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *bpl = buf[5];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getvirtres
 *
 * Description:
 *   Get the virtual resolution of the display.
 *
 * Input parameters:
 *   x - Width in pixels
 *   y - Height in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getvirtres(uint32_t *x, uint32_t *y)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(x != NULL);
  DEBUGASSERT(y != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETVBUF, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *x = buf[5];
  *y = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_setvirtres
 *
 * Description:
 *   Set the virtual resolution of the display. The new virtual resolution is
 *   returned in the input parameters.
 *
 * Input parameters:
 *   x - Width in pixels
 *   y - Height in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setvirtres(uint32_t *x, uint32_t *y)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(x != NULL);
  DEBUGASSERT(y != NULL);

  buf[0] = *x;
  buf[1] = *y;

  bcm2711_mbox_makereq(MBOX_TAG_SETVBUF, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *x = buf[5];
  *y = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getvirtoff
 *
 * Description:
 *   Get the virtual buffer offset in pixels
 *
 * Input parameters:
 *   x - X in pixels
 *   y - Y in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getvirtoff(uint32_t *x, uint32_t *y)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 2] ALIGNED_MBOX;

  DEBUGASSERT(x != NULL);
  DEBUGASSERT(y != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETVIRTOFF, buf, 2 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *x = buf[5];
  *y = buf[6];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getpixord
 *
 * Description:
 *   Get the pixel order of the frame buffer.
 *
 * Input parameters:
 *   rgb - True if RGB order, false for BGR order
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getpixord(bool *rgb)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  DEBUGASSERT(rgb != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETPIXORD, buf, sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *rgb = buf[5] ? true : false;
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_setpixord
 *
 * Description:
 *   Set the pixel order of the frame buffer.
 *
 * Input parameters:
 *   rgb - True if RGB order, false for BGR order
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_setpixord(bool rgb)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 1] ALIGNED_MBOX;

  buf[0] = rgb ? 1 : 0;

  bcm2711_mbox_makereq(MBOX_TAG_SETPIXORD, buf, sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));

  if (buf[5] != rgb)
    {
      ipcinfo("Pixel order refused.");
      return -EAGAIN;
    }

  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_getoscan
 *
 * Description:
 *   Get the overscan values of the frame buffer in pixels.
 *
 * Input parameters:
 *   top - The amount of top overscan in pixels
 *   bot - The amount of bottom overscan in pixels
 *   left - The amount of left overscan in pixels
 *   right - The amount of right overscan in pixels
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_getoscan(uint32_t *top, uint32_t *bot, uint32_t *left,
                          uint32_t *right)
{
  int err;
  uint32_t buf[BUF_FIELDS + TAG_FIELDS + 4] ALIGNED_MBOX;

  DEBUGASSERT(top != NULL);
  DEBUGASSERT(bot != NULL);
  DEBUGASSERT(left != NULL);
  DEBUGASSERT(right != NULL);

  bcm2711_mbox_makereq(MBOX_TAG_GETOVSCAN, buf, 4 * sizeof(uint32_t),
                       sizeof(buf));
  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  *top = buf[5];
  *bot = buf[6];
  *left = buf[7];
  *right = buf[8];
  return err;
}

/****************************************************************************
 * Name: bcm2711_mbox_fbinit
 *
 * Description:
 *   Initializes the frame buffer video interface with the desired settings.
 *   NOTE: performing each of these actions individually does not appear to
 *   work correctly. I experienced 2x2 virtual resolution when I was trying
 *   to do this piece-wise. Other examples online take this bulk approach,
 *   and it seems to work without issue.
 *
 * Input parameters:
 *   x - The resolution (width) in pixels (physical display width returned
 *   here)
 *   y - The resolution (height) in pixels (physical display height returned
 *   here)
 *   bpp - The bits per pixel (depth) (real value returned her)
 *   fb - The returned frame buffer address
 *   fblen - The length of the frame buffer in bytes
 *
 * Returned Value:
 *   0 on success, negated error code on failure.
 ****************************************************************************/

int bcm2711_mbox_fbinit(uint32_t *x, uint32_t *y, uint32_t *bpp, void **fb,
                        uint32_t *fblen)
{
  int err;
  uint32_t buf[35] ALIGNED_MBOX;

  DEBUGASSERT(x != NULL);
  DEBUGASSERT(y != NULL);
  DEBUGASSERT(fb != NULL);

  buf[0] = sizeof(buf);
  buf[1] = BCM_VC_MBOX_REQUEST;

  /* First, set the physical display height and width */

  buf[2] = MBOX_TAG_SETDISP;
  buf[3] = 8;
  buf[4] = 0;
  buf[5] = *x;
  buf[6] = *y;

  /* Set the virtual resolution */

  buf[7] = MBOX_TAG_SETVBUF;
  buf[8] = 8;
  buf[9] = 8;
  buf[10] = *x;
  buf[11] = *y;

  /* Set offset into virtual display as 0 */

  buf[12] = MBOX_TAG_SETVIRTOFF;
  buf[13] = 8;
  buf[14] = 8;
  buf[15] = 0;
  buf[16] = 0;

  /* Set bits per pixel */

  buf[17] = MBOX_TAG_SETDEPTH;
  buf[18] = 4;
  buf[19] = 4;
  buf[20] = *bpp;

  /* Set the pixel order to RGB order */

  buf[21] = MBOX_TAG_SETPIXORD;
  buf[22] = 4;
  buf[23] = 4;
  buf[24] = 0;

  /* Allocate a frame buffer */

  buf[25] = MBOX_TAG_FALLOC;
  buf[26] = 8;
  buf[27] = 8;
  buf[28] = *bpp; /* Alignment in bits */
  buf[29] = 0;

  /* Get the stride */

  buf[30] = MBOX_TAG_GETPITCH;
  buf[31] = 4;
  buf[32] = 4;
  buf[33] = 0;

  buf[34] = BCM_VC_MBOX_TAGLIST_END;

  err = bcm2711_mbox_sendreq(buf, sizeof(buf));
  if (err < 0)
    {
      ipcerr("Couldn't send frame buffer init request: %d.", err);
      return err;
    }

  *x = buf[10];
  *y = buf[11];
  *bpp = buf[20];
  *fb = (void *)(uintptr_t)VCADDR_TO_ARM(buf[28]);
  *fblen = buf[29];
  return 0;
}
