/****************************************************************************
 * include/nuttx/audio/audio.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_AUDIO_AUDIO_H
#define __INCLUDE_NUTTX_AUDIO_AUDIO_H

/* For the purposes of this driver, an Audio device is any device that
 * generates, records, mixes, or otherwise modifies audio data in any format,
 * such as PCM, MP3, AAC, etc.
 *
 * The Audio driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the comman Audio interface
 *    to application level code, and
 * 2) A "lower half", platform-specific driver that implements the low-level
 *    controls to configure and communicate with the audio device(s).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <queue.h>
#include <semaphore.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_AUDIO - Enables Audio driver support
 * CONFIG_DEBUG_AUDIO - If enabled (with CONFIG_DEBUG_FEATURES and, optionally,
 *   CONFIG_DEBUG_INFO), this will generate output that can be used to
 *   debug Audio drivers.
 */

/* IOCTL Commands ***********************************************************/
/* The Audio module uses a standard character driver framework.  However, a
 * lot of the Audio driver functionality is configured via a device control
 * interface, such as sampling rate, volume, data format, etc.
 * The Audio ioctl commands are listed below:
 *
 * AUDIOIOC_GETCAPS - Get the Audio Device Capabilities
 *
 *   ioctl argument:  Pointer to the audio_caps_s structure to receive the
 *                    capabilities info.  The "len" and "type" fields should
 *                    be filled in prior to calling this ioctl.  To get
 *                    overall capabilities, specify the type as
 *                    AUDIO_TYPE_QUERY, otherwise specify any type that was
 *                    reported by the device during the QUERY.
 *
 * AUDIOIOC_CONFIGURE - Configure device for the specified mode
 *
 *   ioctl argument:  Pointer to the audio_caps_s structure which identifies
 *                    the capabilities to configure for.
 *
 * AUDIOIOC_SHUTDOWN - Shutdown the device.
 *
 *   ioctl argument:  None
 *
 * AUDIOIOC_START - Start Audio streaming
 *
 *   ioctl argument:  None
 *
 * AUDIOIOC_STOP - Stop Audio streaming
 *
 *   ioctl argument:  None
 */

#define AUDIOIOC_GETCAPS            _AUDIOIOC(1)
#define AUDIOIOC_RESERVE            _AUDIOIOC(2)
#define AUDIOIOC_RELEASE            _AUDIOIOC(3)
#define AUDIOIOC_CONFIGURE          _AUDIOIOC(4)
#define AUDIOIOC_SHUTDOWN           _AUDIOIOC(5)
#define AUDIOIOC_START              _AUDIOIOC(6)
#define AUDIOIOC_STOP               _AUDIOIOC(7)
#define AUDIOIOC_PAUSE              _AUDIOIOC(8)
#define AUDIOIOC_RESUME             _AUDIOIOC(9)
#define AUDIOIOC_GETBUFFERINFO      _AUDIOIOC(10)
#define AUDIOIOC_ALLOCBUFFER        _AUDIOIOC(11)
#define AUDIOIOC_FREEBUFFER         _AUDIOIOC(12)
#define AUDIOIOC_ENQUEUEBUFFER      _AUDIOIOC(13)
#define AUDIOIOC_REGISTERMQ         _AUDIOIOC(14)
#define AUDIOIOC_UNREGISTERMQ       _AUDIOIOC(15)
#define AUDIOIOC_HWRESET            _AUDIOIOC(16)

/* Audio Device Types *******************************************************/
/* The NuttX audio interface support different types of audio devices for
 * input, output, synthesis, and manipulation of audio data.  A given driver/
 * device could support a combination of these device type.  The following
 * is a list of bit-field definitions for defining the device type.
 */

#define AUDIO_TYPE_QUERY            0x00
#define AUDIO_TYPE_INPUT            0x01
#define AUDIO_TYPE_OUTPUT           0x02
#define AUDIO_TYPE_MIXER            0x04
#define AUDIO_TYPE_SELECTOR         0x08
#define AUDIO_TYPE_FEATURE          0x10
#define AUDIO_TYPE_EFFECT           0x20
#define AUDIO_TYPE_PROCESSING       0x40
#define AUDIO_TYPE_EXTENSION        0x80

/* Audio Format Types *******************************************************/
/* The following defines the audio data format types in NuttX.  During a
 * format query, these will be converted to bit positions within the
 * ac_format field, meaning we currently only support up to 16 formats. To
 * support more than that, we will use the FMT_OTHER entry, and the
 * interfacing software can perform a second query to get the other formats.
 */

#define AUDIO_FMT_UNDEF             0x00
#define AUDIO_FMT_OTHER             0x01
#define AUDIO_FMT_MPEG              0x02
#define AUDIO_FMT_AC3               0x03
#define AUDIO_FMT_WMA               0x04
#define AUDIO_FMT_DTS               0x05
#define AUDIO_FMT_PCM               0x06
#define AUDIO_FMT_WAV               0x07
#define AUDIO_FMT_MP3               0x08
#define AUDIO_FMT_MIDI              0x09
#define AUDIO_FMT_OGG_VORBIS        0x0a
#define AUDIO_FMT_FLAC              0x0b

/* Audio Sub-Format Types ***************************************************/

#define AUDIO_SUBFMT_END            0x00
#define AUDIO_SUBFMT_PCM_MP1        0x01
#define AUDIO_SUBFMT_PCM_MP2        0x02
#define AUDIO_SUBFMT_PCM_MP3        0x03
#define AUDIO_SUBFMT_PCM_MU_LAW     0x04
#define AUDIO_SUBFMT_PCM_A_LAW      0x05
#define AUDIO_SUBFMT_PCM_U8         0x06
#define AUDIO_SUBFMT_PCM_S8         0x07
#define AUDIO_SUBFMT_PCM_U16_LE     0x08
#define AUDIO_SUBFMT_PCM_S16_BE     0x09
#define AUDIO_SUBFMT_PCM_S16_LE     0x0a
#define AUDIO_SUBFMT_PCM_U16_BE     0x0b
#define AUDIO_SUBFMT_MIDI_0         0x0c
#define AUDIO_SUBFMT_MIDI_1         0x0d
#define AUDIO_SUBFMT_MIDI_2         0x0e

/* Supported Sampling Rates *************************************************/

#define AUDIO_SAMP_RATE_8K          0x0001
#define AUDIO_SAMP_RATE_11K         0x0002
#define AUDIO_SAMP_RATE_16K         0x0004
#define AUDIO_SAMP_RATE_22K         0x0008
#define AUDIO_SAMP_RATE_32K         0x0010
#define AUDIO_SAMP_RATE_44K         0x0020
#define AUDIO_SAMP_RATE_48K         0x0040
#define AUDIO_SAMP_RATE_96K         0x0080
#define AUDIO_SAMP_RATE_128K        0x0100
#define AUDIO_SAMP_RATE_160K        0x0200
#define AUDIO_SAMP_RATE_172K        0x0400
#define AUDIO_SAMP_RATE_192K        0x0800

/* Audio Sub-sampling Ratios  ***********************************************/

#define AUDIO_SUBSAMPLE_NONE        0
#define AUDIO_SUBSAMPLE_2X          2
#define AUDIO_SUBSAMPLE_4X          4
#define AUDIO_SUBSAMPLE_8X          8
#define AUDIO_SUBSAMPLE_16X         16

#define AUDIO_SUBSAMPLE_MIN         AUDIO_SUBSAMPLE_2X
#define AUDIO_SUBSAMPLE_MAX         AUDIO_SUBSAMPLE_16X

/* Supported Bit Rates *************************************************/

#define AUDIO_BIT_RATE_22K          0x01
#define AUDIO_BIT_RATE_44K          0x02
#define AUDIO_BIT_RATE_48K          0x04
#define AUDIO_BIT_RATE_96K          0x08
#define AUDIO_BIT_RATE_128K         0x10
#define AUDIO_BIT_RATE_160K         0x20
#define AUDIO_BIT_RATE_172K         0x40
#define AUDIO_BIT_RATE_192K         0x80

/* Supported Feature Units controls *****************************************/

#define AUDIO_FU_UNDEF              0x0000
#define AUDIO_FU_MUTE               0x0001
#define AUDIO_FU_VOLUME             0x0002
#define AUDIO_FU_BASS               0x0004
#define AUDIO_FU_MID                0x0008
#define AUDIO_FU_TREBLE             0x0010
#define AUDIO_FU_EQUALIZER          0x0020
#define AUDIO_FU_AGC                0x0040
#define AUDIO_FU_DELAY              0x0080
#define AUDIO_FU_BASS_BOOST         0x0100
#define AUDIO_FU_LOUDNESS           0x0200
#define AUDIO_FU_INP_GAIN           0x0400
#define AUDIO_FU_BALANCE            0x0800
#define AUDIO_FU_PHASE_INVERT       0x1000
#define AUDIO_FU_UNDERFLOW          0x2000
#define AUDIO_FU_OVERFLOW           0x4000
#define AUDIO_FU_LATENCY            0x8000

/* Processing Unit controls *************************************************/

#define AUDIO_PU_UNDEF              0x00
#define AUDIO_PU_UPDOWNMIX          0x01
#define AUDIO_PU_DOLBY_PROLOGIC     0x02
#define AUDIO_PU_STEREO_EXTENDER    0x03
#define AUDIO_PU_SUBSAMPLE_FORWARD  0x04
#define AUDIO_PU_SUBSAMPLE_REWIND   0x05

/* Stereo Extender PU Controls **********************************************/

#define AUDIO_STEXT_UNDEF           0x00
#define AUDIO_STEXT_ENABLE          0x01
#define AUDIO_STEXT_WIDTH           0x02
#define AUDIO_STEXT_UNDERFLOW       0x03
#define AUDIO_STEXT_OVERFLOW        0x04
#define AUDIO_STEXT_LATENCY         0x05

/* Audio Callback Reasons ***************************************************/

#define AUDIO_CALLBACK_UNDEF        0x00
#define AUDIO_CALLBACK_DEQUEUE      0x01
#define AUDIO_CALLBACK_IOERR        0x02
#define AUDIO_CALLBACK_COMPLETE     0x03

/* Audio Pipeline Buffer (AP Buffer) flags **********************************/

#define AUDIO_ABP_ALIGNMENT         0x000f  /* Mask to define buffer alignment */
#define AUDIO_ABP_CANDMA            0x0010  /* Set if the data is DMA'able */
#define AUDIO_ABP_STATIC            0x0020  /* Set if statically allocated */
#define AUDIO_ABP_ACTIVE            0x0040  /* Set if this buffer is still active.
                                             *   A buffer could become inactive
                                             *   if it is processed by an output
                                             *   device or a processing device
                                             *   that replaces it with an alternate
                                             *   buffer as a result of some DSP
                                             *   operation, etc.
                                             */

/* Standard Audio Message Queue message IDs */

#define AUDIO_MSG_NONE              0
#define AUDIO_MSG_DEQUEUE           1
#define AUDIO_MSG_START             2
#define AUDIO_MSG_STOP              3
#define AUDIO_MSG_PAUSE             4
#define AUDIO_MSG_RESUME            5
#define AUDIO_MSG_DATA_REQUEST      6
#define AUDIO_MSG_ENQUEUE           7
#define AUDIO_MSG_COMPLETE          8
#define AUDIO_MSG_USER             64

/* Audio Pipeline Buffer flags */

#define AUDIO_APB_OUTPUT_ENQUEUED   (1 << 0)
#define AUDIO_APB_OUTPUT_PROCESS    (1 << 1)
#define AUDIO_APB_DEQUEUED          (1 << 2)
#define AUDIO_APB_FINAL             (1 << 3) /* Last buffer in the stream */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Define the size of AP Buffer sample count base on CONFIG */

#ifdef CONFIG_AUDIO_LARGE_BUFFERS
typedef uint32_t apb_samp_t;
#else
typedef uint16_t apb_samp_t;
#endif

/* This structure is used to describe the audio device capabilities */

struct audio_caps_s
{
  uint8_t ac_len;           /* Length of the structure */
  uint8_t ac_type;          /* Capabilities (device) type */
  uint8_t ac_subtype;       /* Capabilities sub-type, if needed */
  uint8_t ac_channels;      /* Number of channels (1, 2, 5, 7) */

  union                     /* Audio data format(s) for this device */
  {
    uint8_t  b[2];
    uint16_t hw;
  } ac_format;

  union                     /* Device specific controls. For AUDIO_DEVICE_QUERY, */
  {                         /*   this field reports the device type supported */
    uint8_t  b[4];          /*   by this lower-half driver. */
    uint16_t hw[2];
    uint32_t w;
  } ac_controls;
};

struct audio_caps_desc_s
{
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void              *session;   /* Associated session */
#endif
  struct audio_caps_s   caps;       /* The capabilities struct */
};

/* This structure describes the characteristics of the Audio samples */

struct audio_info_s
{
  uint8_t samplerate;   /* Sample Rate of the audio data */
  uint8_t channels;     /* Number of channels (1, 2, 5, 7) */
  uint8_t format;       /* Audio data format */
  uint8_t subformat;    /* Audio subformat (maybe should be combined with format? */
};

/* This structure describes the preferred number and size of
 * audio pipeline buffers for the audio device.  Each device
 * may have unique needs regarding size and qty of buffers,
 * so this info is queried from the lower-half driver.
 */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
struct ap_buffer_info_s
{
  apb_samp_t  nbuffers;     /* Preferred qty of buffers */
  apb_samp_t  buffer_size;  /* Preferred size of the buffers */
};
#endif

/* This structure describes an Audio Pipeline Buffer */

begin_packed_struct struct ap_buffer_s
{
  struct dq_entry_s     dq_entry;   /* Double linked queue entry */
  struct audio_info_s   i;          /* The info for samples in this buffer */
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void              *session;   /* Associated session */
#endif
  apb_samp_t            nmaxbytes;  /* The maximum number of bytes */
  apb_samp_t            nbytes;     /* The number of bytes used */
  apb_samp_t            curbyte;    /* Next byte to be processed */
  sem_t                 sem;        /* Reference locking semaphore */
  uint16_t              flags;      /* Buffer flags */
  uint16_t              crefs;      /* Number of reference counts */
  uint8_t               samp[0];    /* Offset of the first sample */
} end_packed_struct;

/* Structure defining the messages passed to a listening audio thread
 * for dequeuing buffers and other operations.  Also used to allocate
 * and enqueue buffers via the AUDIOIOC_ALLOCBUFFER, AUDIOIOC_FREEBUFFER,
 * and AUDIOIOC_ENQUEUEBUFFER ioctls.
 */

struct audio_msg_s
{
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void           *session;      /* Associated channel */
#endif
  uint16_t            msgId;        /* Message ID */
  union
  {
    FAR void         *pPtr;         /* Buffer being dequeued */
    uint32_t          data;         /* Message data */
  } u;
};

/* Structure defining the built-in sounds */

#ifdef CONFIG_AUDIO_BUILTIN_SOUNDS
struct audio_sound_s
{
  const char         *name;         /* Name of the sound */
  uint32_t            id;           /* ID of the sound */
  uint32_t            type;         /* Type of sound */
  uint32_t            size;         /* Number of bytes in the sound */
  const uint8_t      *data;         /* Pointer to the data */
};

#endif

/* Structure for allocating, freeing and enqueueing audio pipeline
 * buffers via the AUDIOIOC_ALLOCBUFFER, AUDIOIOC_FREEBUFFER,
 * and AUDIOIOC_ENQUEUEBUFFER ioctls.
 */

struct audio_buf_desc_s
{
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void            *session;           /* Associated channel */
#endif
  uint16_t            numbytes;           /* Number of bytes to allocate */
  union
  {
    FAR struct ap_buffer_s  *pBuffer;     /* Buffer to free / enqueue */
    FAR struct ap_buffer_s  **ppBuffer;   /* Pointer to receive allocated buffer */
  } u;
};

/* Typedef for lower-level to upper-level callback for buffer dequeuing */

#ifdef CONFIG_AUDIO_MULTI_SESSION
typedef CODE void (*audio_callback_t)(FAR void *priv, uint16_t reason,
        FAR struct ap_buffer_s *apb, uint16_t status, FAR void *session);
#else
typedef CODE void (*audio_callback_t)(FAR void *priv, uint16_t reason,
        FAR struct ap_buffer_s *apb, uint16_t status);
#endif

/* This structure is a set a callback functions used to call from the upper-
 * half, generic Audo driver into lower-half, platform-specific logic that
 * supports the low-level functionality.
 */

struct audio_lowerhalf_s;
struct audio_ops_s
{
  /* This method is called to retrieve the lower-half device capabilities.
   * It will be called with device type AUDIO_TYPE_QUERY to request the
   * overall capabilities, such as to determine the types of devices supported
   * audio formats supported, etc.  Then it may be called once or more with
   * reported supported device types to determine the specific capabilities
   * of that device type (such as MP3 encoder, WMA encoder, PCM output, etc.).
   */

  CODE int (*getcaps)(FAR struct audio_lowerhalf_s *dev, int type,
      FAR struct audio_caps_s *pCaps);

  /* This method is called to bind the lower-level driver to the upper-level
   * driver and to configure the driver for a specific mode of
   * operation defined by the parameters selected in supplied device caps
   * structure.  The lower-level device should perform any initialization
   * needed to prepare for operations in the specified mode.  It should not,
   * however, process any audio data until the start method is called.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*configure)(FAR struct audio_lowerhalf_s *dev,
      FAR void *session, FAR const struct audio_caps_s *pCaps);
#else
  CODE int (*configure)(FAR struct audio_lowerhalf_s *dev,
      FAR const struct audio_caps_s *pCaps);
#endif

  /* This method is called when the driver is closed.  The lower half driver
   * should stop processing audio data, including terminating any active
   * output generation.  It should also disable the audio hardware and put
   * it into the lowest possible power usage state.
   *
   * Any enqueued Audio Pipeline Buffers that have not been processed / dequeued
   * should be dequeued by this function.
   */

  CODE int (*shutdown)(FAR struct audio_lowerhalf_s *dev);

  /* Start audio streaming in the configured mode.  For input and synthesis
   * devices, this means it should begin sending streaming audio data.  For output
   * or processing type device, it means it should begin processing of any enqueued
   * Audio Pipeline Buffers.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*start)(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
  CODE int (*start)(FAR struct audio_lowerhalf_s *dev);
#endif

  /* Stop audio streaming and/or processing of enqueued Audio Pipeline Buffers */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*stop)(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
  CODE int (*stop)(FAR struct audio_lowerhalf_s *dev);
#endif
#endif

  /* Pause the audio stream.  Should keep current playback context active
   * in case a resume is issued.  Could be called and then followed by a stop.
   */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*pause)(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
  CODE int (*pause)(FAR struct audio_lowerhalf_s *dev);
#endif

  /* Resumes audio streaming after a pause */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*resume)(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
  CODE int (*resume)(FAR struct audio_lowerhalf_s *dev);
#endif
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

  /* Allocate an audio pipeline buffer.  This routine provides the
   * lower-half driver with the opportunity to perform special buffer
   * allocation if needed, such as allocating from a specific memory
   * region (DMA-able, etc.).  If not supplied, then the top-half
   * driver will perform a standard kumm_malloc using normal user-space
   * memory region.
   */

  CODE int (*allocbuffer)(FAR struct audio_lowerhalf_s *dev,
          FAR struct audio_buf_desc_s *apb);

  /* Free an audio pipeline buffer.  If the lower-level driver
   * provides an allocbuffer routine, it should also provide the
   * freebuffer routine to perform the free operation.
   */

  CODE int (*freebuffer)(FAR struct audio_lowerhalf_s *dev,
         FAR struct audio_buf_desc_s *apb);

  /* Enqueue a buffer for processing.  This is a non-blocking enqueue operation.
   * If the lower-half driver's buffer queue is full, then it should return an
   * error code of -ENOMEM, and the upper-half driver can decide to either block
   * the calling thread or deal with it in a non-blocking manner.

   * For each call to enqueuebuffer, the lower-half driver must call
   * audio_dequeuebuffer when it is finished processing the bufferr, passing the
   * previously enqueued apb and a dequeue status so that the upper-half driver
   * can decide if a waiting thread needs to be release, if the dequeued buffer
   * should be passed to the next block in the Audio Pipeline, etc.
   */

  CODE int (*enqueuebuffer)(FAR struct audio_lowerhalf_s *dev,
          FAR struct ap_buffer_s *apb);

  /* Cancel a previously enqueued buffer. */

  CODE int (*cancelbuffer)(FAR struct audio_lowerhalf_s *dev,
          FAR struct ap_buffer_s *apb);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct audio_lowerhalf_s *dev,
                    int cmd, unsigned long arg);

  /* Lower-half logic may support platform-specific read commands */

  CODE int (*read)(FAR struct audio_lowerhalf_s *dev,
                    FAR char *buffer, size_t buflen);

  /* Lower-half logic may support platform-specific write commands */

  CODE int (*write)(FAR struct audio_lowerhalf_s *dev,
                    FAR const char *buffer, size_t buflen);

  /* Reserve a session (may only be one per device or may be multiple) for
   * use by a client.  Client software can open audio devices and issue
   * AUDIOIOC_GETCAPS calls freely, but other operations require a
   * reservation.  A session reservation will assign a context that must
   * be passed with
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*reserve)(FAR struct audio_lowerhalf_s *dev, FAR void **psession);
#else
  CODE int (*reserve)(FAR struct audio_lowerhalf_s *dev);
#endif

  /* Release a session.  */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  CODE int (*release)(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
  CODE int (*release)(FAR struct audio_lowerhalf_s *dev);
#endif
};

/* This structure is the generic form of state structure used by lower half
 * Audio driver.  This state structure is passed to the Audio driver when the
 * driver is initialized.  Then, on subsequent callbacks into the lower half
 * Audio logic, this structure is provided so that the Audio logic can
 * maintain state information.
 *
 * Normally that Audio logic will have its own, custom state structure
 * that is simply cast to struct audio_lowerhalf_s.  In order to perform such
 * casts, the initial fields of the custom state structure match the initial
 * fields of the following generic Audio state structure.
 */

struct audio_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the Audio
   * callback structure:
   */

  FAR const struct audio_ops_s *ops;

  /* The bind data to the upper-half driver used for callbacks of dequeuing
   * buffer, reporting asynchronous event, reporting errors, etc.
   */

  FAR audio_callback_t  upper;

  /* The private opaque pointer to be passed to upper-layer during callbacks */

  FAR void *priv;

  /* The custom Audio device state structure may include additional fields
   * after the pointer to the Audio callback structure.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * "Upper-Half" Audio Driver Interfaces
 ****************************************************************************/
/****************************************************************************
 * Name: audio_register
 *
 * Description:
 *   This function binds an instance of a "lower half" Audio driver with the
 *   "upper half" Audio device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input parameters:
 *   name - The name of the audio device.  This name will be used to generate
 *     a full path to the driver in the format "/dev/audio/[name]" in the NuttX
 *     filesystem (i.e. the path "/dev/audio" will be prepended to the supplied
 *     device name.  The recommended convention is to name Audio drivers
 *     based on the type of functionality they provide, such as "/dev/audio/pcm0",
 *     "/dev/audio/midi0", "/dev/audio/mp30, etc.
 *   dev - A pointer to an instance of lower half audio driver.  This instance
 *     is bound to the Audio driver and must persists as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int audio_register(FAR const char *name, FAR struct audio_lowerhalf_s *dev);

/****************************************************************************
 * Name: abp_alloc
 *
 * Description:
 *   Allocated an AP Buffer and prepares it for use.  This allocates a dynamically
 *   allocated buffer that has no special DMA capabilities.
 *
 * Input parameters:
 *   bufdesc:   Pointer to a buffer descriptor
 *
 * Returned Value:
 *   Pointer to the allocated buffer or NULL if no memory.
 *
 ****************************************************************************/

int apb_alloc(FAR struct audio_buf_desc_s *bufdesc);

/****************************************************************************
 * Name: apb_free
 *
 * Free's a previously allocated or referenced Audio Pipeline Buffer
 *
 ****************************************************************************/

void apb_free(FAR struct ap_buffer_s *apb);

/****************************************************************************
 * Name: apb_reference
 *
 * Claim a reference to an Audio Pipeline Buffer.  Each call to apb_reference
 * will increment the reference count and must have a matching apb_free
 * call.  When the refcount decrements to zero, the buffer will be freed.
 *
 ****************************************************************************/

void apb_reference(FAR struct ap_buffer_s *apb);

/****************************************************************************
 * Platform-Dependent "Lower-Half" Audio Driver Interfaces
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO */
#endif /* __INCLUDE_NUTTX_AUDIO_AUDIO_H */
