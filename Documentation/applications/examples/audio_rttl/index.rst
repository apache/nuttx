===================================
``audio_rttl`` Audio tone generator
===================================

This application is an example which uses the NuttX audio subsystem to play RTTL
audio. RTTL playing is made possible with
:doc:`/applications/audioutils/rtttl-c/index`.

You can run ``audio_rttl -h`` to display help information.

Supported Devices
-----------------

.. note::

   This application does not currently support all audio devices available under
   NuttX. Currently, only PWM audio devices are supported.

In order to add support for a new audio device, you must submit a patch defining
the interface for the following ``player_s`` structure:

.. code:: c

   struct player_s
   {
     void (*play)(struct rtttl_tone); /* Play RTTTL sound function */
     void (*teardown)(void);          /* Clean-up function */
     void *arg;                       /* Private data for functions */
   };

The ``play`` function is used by the ``rtttl-c`` library to play an audio tone.
You can implement the appropriate driver operations (i.e. ``ioctl`` calls) in
this function. See the PWM implementation for an example.

The ``teardown`` function is used to cleanup the player. For most audio devices,
this involves closing the file descriptor.

In the ``main`` function, the path to the audio device is used to identify the
player to use. For instance, PWM devices are under ``/dev/pwm<n>``, so device
paths containing "pwm" cause the PWM driver to be selected. If no player can be
associated with the device, the default player is selected which just prints an
error.


Usage
-----

The following will play the default RTTL song (a scale) over a PWM device (i.e.
buzzer). This application by default plays on all channels associated with the
PWM device.

.. code:: console

   nsh> audio_rttl /dev/pwm0

If you wish to play your own RTTL song, you may pass it as a string argument
with the ``-s`` flag:

.. code:: console

   nsh> audio_rttl /dev/pwm0 -s "Test:d=4,o=6,b=90:c,c#,d,d#,e,f,f#,g,a,a#,b,p,b,a#,a,g,f#,f,e,d#,d,c#,c"

This can be cumbersome, especially for long songs. You may also wish to provide
a file path where the file contains the RTTL string using the ``-f`` flag:

.. code:: console

   nsh> audio_rttl /dev/pwm0 -f /data/mysong.rttl

This application limits the size of the RTTL string that can be read from a file
using ``CONFIG_EXAMPLES_AUDIO_SOUND_MAXLEN``.
