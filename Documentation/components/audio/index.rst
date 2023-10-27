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
