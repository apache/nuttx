===============
Audio Subsystem
===============

This page discusses the audio subsystem support for NuttX which is only built if
``CONFIG_AUDIO`` is defined in the NuttX configuration file.

Files in nuttx/audio
====================

This directory holds the NuttX audio subsystem upper-half.  The upper-half provides
a common interface for applications to interface with and also defines a bind
layer for specific lower-half audio device drivers.

* ``audio.c`` - The upper-half driver that binds to a lower-half driver from the
  drivers/audio subdirectory.  For each attached audio device, there
  will be an instance of this upper-half driver bound to the
  instance of the lower half driver context.
* ``pcm_decode.c`` - Routines to decode PCM / WAV type data.

Portions of the audio system interface have application interfaces.  Those
portions reside in the ``nuttx/libc/audio`` directory where the will be built for
access by both OS driver logic and user application logic.  Those relevant
files in ``nuttx/libc/audio`` include:

* ``buffer.c``  - Routines to manage creattion and destruction of audio pipeline buffers
  (apb) used in the audio subsystem.  Audio pipeline buffers are passed
  between user applications and the audio drivers to deliver audio
  content for playback (or possibly recording in the future).

Related Header Files
====================

``include/nuttx/audio/audio.h`` - Top level include file defining the audio interface
``include/nuttx/audio/vs1053.h`` - Specific driver initialization prototypes

Playback and Capture Flow
=========================

The upper half tracks a device-level state, held in ``struct audio_status_s``
and readable with ``AUDIOIOC_GETSTATUS``.  ``AUDIOIOC_START`` is only accepted
once the device has left ``AUDIO_STATE_OPEN``:

.. code-block:: none

   AUDIO_STATE_OPEN --configure--> AUDIO_STATE_PREPARED --start--> AUDIO_STATE_RUNNING
                                            ^                            |
                                            |                       pause | resume
                                            |                            v
                                       stop |                  AUDIO_STATE_PAUSED
                                            |
             AUDIO_STATE_DRAINING <----------+          (buffers under-run:
                     |                                  AUDIO_STATE_XRUN)
                     v
             AUDIO_STATE_OPEN  (once the lower half reports completion)

The states are defined in ``include/nuttx/audio/audio.h`` in ascending order of
priority, which is how the upper half aggregates the state of several
applications sharing one device:

============================ ===== ===========================================
State                        Value Meaning
============================ ===== ===========================================
``AUDIO_STATE_OPEN``         0     Opened, no format established yet
``AUDIO_STATE_PREPARED``     1     Format established, ready to be started
``AUDIO_STATE_PAUSED``       2     Started, then paused
``AUDIO_STATE_XRUN``         3     Started, but the application fell behind
``AUDIO_STATE_DRAINING``     4     Stop requested, waiting for queued buffers
``AUDIO_STATE_RUNNING``      5     Transferring
============================ ===== ===========================================

A normal playback sequence is:

#. ``open()`` the device
#. ``AUDIOIOC_RESERVE`` to obtain a session (with ``CONFIG_AUDIO_MULTI_SESSION``)
#. ``AUDIOIOC_GETCAPS`` to query what the device supports
#. ``AUDIOIOC_CONFIGURE`` with ``ac_type`` set to ``AUDIO_TYPE_OUTPUT`` (or
   ``AUDIO_TYPE_INPUT``) to establish the stream format
#. ``AUDIOIOC_GETBUFFERINFO`` to learn the preferred buffer size and count
#. ``AUDIOIOC_ALLOCBUFFER`` for each buffer
#. ``AUDIOIOC_REGISTERMQ`` so the driver can report buffer completion
#. ``AUDIOIOC_ENQUEUEBUFFER`` to hand filled buffers to the driver
#. ``AUDIOIOC_START`` to begin transferring
#. ``AUDIOIOC_STOP`` when finished, then ``AUDIOIOC_FREEBUFFER`` and
   ``AUDIOIOC_RELEASE``

.. important::
   Step 4 is mandatory.  ``AUDIOIOC_CONFIGURE`` is the only thing that moves the
   device from ``AUDIO_STATE_OPEN`` to ``AUDIO_STATE_PREPARED``, so omitting it
   makes ``AUDIOIOC_START`` fail with ``-EPERM``.

   Note that a lower half configuring itself does not satisfy this: only a
   configuration arriving through the ioctl is observed by the upper half, so
   the state can still be ``AUDIO_STATE_OPEN`` while the hardware is already
   programmed.  The application must issue ``AUDIOIOC_CONFIGURE`` itself, with
   plausible non-zero values if it cannot determine the real ones.  Passing
   zeros is not a workaround - drivers reject a zero channel count or sample
   rate with ``-ERANGE`` or ``-EINVAL``, and some assert on it.

Buffer Ownership
================

Audio pipeline buffers (``struct ap_buffer_s``, "apb") carry sample data
between the application and the lower-half driver.  ``AUDIOIOC_ALLOCBUFFER``
takes a ``struct audio_buf_desc_s`` and has two distinct modes, selected by the
``u.pbuffer`` field:

**Private buffers** (``u.pbuffer != NULL``)

  The caller supplies the address of a pointer, and receives a buffer that it
  owns for its lifetime.  The upper half only forwards the request - to
  ``lower->ops->allocbuffer()`` if the driver implements it, otherwise to
  ``apb_alloc()``.  These buffers are enqueued individually by passing
  ``u.buffer`` to ``AUDIOIOC_ENQUEUEBUFFER``, and must be released with
  ``AUDIOIOC_FREEBUFFER``.  On success the ioctl returns
  ``sizeof(struct audio_buf_desc_s)``.

  This is the mode used by ``nxplayer``, ``nxrecorder`` and ``nxlooper``.

**Shared ring buffers** (``u.pbuffer == NULL``)

  The upper half allocates the buffer and keeps it in an internal ring, whose
  depth is bounded by the buffer count the lower half reported through
  ``AUDIOIOC_GETBUFFERINFO``.  Buffers are not handed back to the caller;
  instead the application enqueues by passing a descriptor with
  ``u.buffer == NULL``, and the upper half picks the next ring slot itself.
  This is what allows several applications to feed one device concurrently, and
  it is also the form that ``mmap()`` exposes.

  When the ring has already been populated - typically because another
  application attached first - ``AUDIOIOC_ALLOCBUFFER`` returns ``0`` rather
  than an error.  A second application is expected to treat that as success and
  proceed directly to ``AUDIOIOC_ENQUEUEBUFFER``.

  No in-tree application uses this mode yet; the upper half provides it so that
  a client library needing concurrent access or zero-copy submission can be
  built on top of it.

The two modes are independent of the stream format.  Buffer size and count are
a property of the device (its DMA capabilities, FIFO depth or memory pool),
expressed in bytes; the format is a property of the stream, expressed as a rate.
Their ratio only determines how much audio one buffer represents, and the audio
subsystem does not constrain it.

.. note::
   ``AUDIOIOC_GETBUFFERINFO`` is a query, but in the upper half it also records
   the reported buffer count for use as the shared ring depth.  A lower half
   that does not implement it leaves that count at zero, which disables shared
   ring allocation.  Implementing it is recommended for any driver that has a
   meaningful preference; it is normally guarded by
   ``CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS``.

Polling and Memory Mapping
==========================

``poll()`` reports ``POLLIN | POLLOUT`` while the shared ring has room for more
data, and adds ``POLLERR`` when the application has fallen behind the hardware
read pointer, that is, on an under-run.

``mmap()`` is available when ``CONFIG_FS_FILEMAP`` is enabled, and the
requested length selects what gets mapped:

* a length equal to one buffer's ``nmaxbytes`` maps the shared ring buffer
  selected by ``offset / nmaxbytes``, allowing an application to fill samples
  without copying;
* a length equal to ``sizeof(struct audio_status_s)`` maps the device status,
  letting an application observe ``state``, ``head`` and ``tail`` without an
  ioctl.

Any other length is rejected with ``-EINVAL``.

IOCTL Commands
==============

All commands are defined in ``include/nuttx/audio/audio.h``.  Unrecognised
commands are forwarded to the lower-half driver, so a driver may add its own.

Session and Capability
----------------------

* ``AUDIOIOC_GETCAPS``
  Query device capabilities.  Argument is a ``struct audio_caps_s`` with
  ``ac_type`` and ``ac_subtype`` set to describe what is being asked.  Returns
  ``ac_len`` on success.
* ``AUDIOIOC_RESERVE`` / ``AUDIOIOC_RELEASE``
  Acquire and release a session.  With ``CONFIG_AUDIO_MULTI_SESSION`` the
  argument receives, respectively supplies, the session handle.
* ``AUDIOIOC_SHUTDOWN``
  Ask the lower half to release its hardware resources.
* ``AUDIOIOC_HWRESET``
  Reset the hardware, for error recovery.

Configuration
-------------

* ``AUDIOIOC_CONFIGURE``
  Set a parameter.  Argument is a ``struct audio_caps_desc_s``, and ``ac_type``
  selects what is being configured: ``AUDIO_TYPE_OUTPUT`` or
  ``AUDIO_TYPE_INPUT`` for the stream format, ``AUDIO_TYPE_FEATURE`` for
  volume, balance, bass and treble, ``AUDIO_TYPE_PROCESSING`` for the
  processing chain.  Only the format variants advance the state machine.
* ``AUDIOIOC_GETAUDIOINFO``
  Retrieve the current format as a ``struct audio_info_s``.  The lower half is
  asked first; if it does not answer, the format last set through
  ``AUDIOIOC_CONFIGURE`` is returned instead.
* ``AUDIOIOC_GETLATENCY``, ``AUDIOIOC_GETPOSITION``
  Query the current latency, respectively playback position.

Buffer Management
-----------------

* ``AUDIOIOC_GETBUFFERINFO`` / ``AUDIOIOC_SETBUFFERINFO``
  Query, respectively override, the preferred buffer size and count as a
  ``struct ap_buffer_info_s``.  See `Buffer Ownership`_ for the side effect of
  the query on the shared ring depth.
* ``AUDIOIOC_ALLOCBUFFER`` / ``AUDIOIOC_FREEBUFFER``
  Allocate and release audio pipeline buffers.  See `Buffer Ownership`_ for the
  two allocation modes and their return values.
* ``AUDIOIOC_ENQUEUEBUFFER``
  Hand a buffer to the driver.  ``u.buffer`` names a private buffer, or is
  ``NULL`` to submit the next slot of the shared ring.
* ``AUDIOIOC_FLUSH``
  Discard queued buffers.

Transfer Control
----------------

* ``AUDIOIOC_START``
  Begin transferring.  Requires the device to have left ``AUDIO_STATE_OPEN``,
  otherwise it fails with ``-EPERM``.  If the device is paused this resumes it.
* ``AUDIOIOC_STOP``
  Stop transferring.  The device enters ``AUDIO_STATE_DRAINING`` until the
  lower half has returned every queued buffer.  Excluded by
  ``CONFIG_AUDIO_EXCLUDE_STOP``.
* ``AUDIOIOC_PAUSE`` / ``AUDIOIOC_RESUME``
  Suspend and resume transferring.  Excluded by
  ``CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME``.

Status and Notification
-----------------------

* ``AUDIOIOC_GETSTATUS``
  Copy out the device status as a ``struct audio_status_s``, containing the
  state and the ``head`` and ``tail`` indices of the shared ring.
* ``AUDIOIOC_RESETSTATUS``
  Move this application's write position to the current ring tail, discarding
  the backlog.  Used to recover from an under-run.
* ``AUDIOIOC_REGISTERMQ`` / ``AUDIOIOC_UNREGISTERMQ``
  Register, respectively remove, the message queue on which the driver reports
  ``AUDIO_MSG_DEQUEUE``, ``AUDIO_MSG_COMPLETE`` and error events.

Multiple Applications
=====================

A single audio device may be opened by several applications at once.  Each
``open()`` gets its own private state, while the device-level state exposed
through ``AUDIOIOC_GETSTATUS`` is the aggregate: the upper half takes the
highest-priority state among the open instances, which is why the
``AUDIO_STATE_*`` values are ordered.  In practice this means one application
pausing does not stop a device another application is still using, and the
lower half is only stopped once every instance has finished.

Concurrent use relies on the shared ring described in `Buffer Ownership`_,
since that is the only mode in which the upper half, rather than a single
application, owns the buffers.  The upper half implements this, but no in-tree
application exercises it yet.

Configuration Settings
======================

General Audio Settings
----------------------

* ``CONFIG_AUDIO``
  Enables overall support for audio subsystem
* ``CONFIG_AUDIO_MULTI_SESSION``
  Enables support for the audio subsystem to track multiple open sessions
  with lower-level audio devices.
* ``CONFIG_AUDIO_LARGE_BUFFERS``
  Specifies that buffer size variables should be 32-bit vs. the normal 16-bit
  size.  This allows buffers to be larger than 64K bytes on systems with
  an abundance of RAM.
* ``CONFIG_AUDIO_NUM_BUFFERS``
  Sets the number of audio buffers to use for audio operations.  If the
  configuration has set ``CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS``, and an audio
  device does not support the operation, then this becomes the default number
  of buffers to use.
* ``CONFIG_AUDIO_BUFFER_SIZE``
  Sets the size of the audio buffers to use for audio operations.  If the
  configuration has set ``CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS``, and an audio
  device does not support the operation, then this becomes the default size
  of buffers to use.
* ``CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS``
  Enables support for lower-level audio drivers to specify the number and size
  of buffers that should be allocated for best performance while interacting
  with that driver.
* ``CONFIG_AUDIO_CUSTOM_DEV_PATH``
  Specifies that all audio devices should be registered in the filesystem at
  a location other than the standard ``/dev/audio`` directory.
* ``CONFIG_AUDIO_DEV_ROOT``
  Specifies that all audio devices should be registered in the ``/dev`` directory.
  Saves a tiny bit of code and RAM space since an additional directory isn't needed,
  but at the expense of execution speed when searching for audio devices since all
  entries in ``/dev`` must be opened and tested if they provide audio support.
  Available only if ``CONFIG_AUDIO_CUSTOM_DEV_PATH`` is selected.
* ``CONFIG_AUDIO_DEV_PATH``
  Specifies a custom directory where audio devices will be registered.
  Available if ``CONFIG_AUDIO_CUSTOM_DEV_PATH`` is selected and ``CONFIG_AUDIO_DEV_ROOT``
  is not selected.

Audio Format Support Selections
-------------------------------

* ``CONFIG_AUDIO_FORMAT_AC3``
  Specifies that AC3 support should be enabled if available by a lower-half driver.
* ``CONFIG_AUDIO_FORMAT_DTS``
  Specifies that DTS support should be enabled if available by a lower-half driver.
* ``CONFIG_AUDIO_FORMAT_PCM``
  Specifies that PCM support should be enabled if available by a lower-half driver.
* ``CONFIG_AUDIO_FORMAT_MP3``
  Specifies that MP3 support should be enabled if available by a lower-half driver.
* ``CONFIG_AUDIO_FORMAT_MIDI``
  Specifies that MIDI support should be enabled if available by a lower-half driver.
* ``CONFIG_AUDIO_FORMAT_WMA``
  Specifies that WMA support should be enabled if available by a lower-half driver.
* ``CONFIG_AUDIO_FORMAT_OGG_VORBIS``
  Specifies that Ogg Vorbis support should be enabled if available by a lower-half driver.

Audio feature exclusion Selections
----------------------------------

* ``CONFIG_AUDIO_EXCLUDE_VOLUME``
  Disables support in all libraries and drivers for setting the playback volume.  In
  this case, the device volume will depend on the default level defined by the
  lower-level driver, typically via a config setting.
* ``CONFIG_AUDIO_EXCLUDE_BALANCE``
  Disables support in all libraries and drivers for setting the playback balance.
  Also, the volume support must not be excluded for balance to work or make sense.
* ``CONFIG_AUDIO_EXCLUDE_TONE``
  Disables support for setting bass and treble.
* ``CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME``
  Disables support in all libraries and drivers for pausing and resuming playback.
* ``CONFIG_AUDIO_EXCLUDE_STOP``
  Disables support in all libraries and drivers for stopping an audio playback
  once it has started.  Typically selected if only short notification audio sounds
  are needed (vs. media playing type applications).

Related Subdirectories
======================

* ``drivers/audio`` - Contains the lower-level device specific drivers.
* ``apps/system/nxplayer`` - User-mode audio subsystem interface library.

Related Pages
=============

* :doc:`/components/drivers/special/audio`
* :doc:`/applications/audioutils/index`
