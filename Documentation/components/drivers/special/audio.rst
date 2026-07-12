====================
Audio Device Drivers
====================

See ``include/nuttx/audio/audio.h`` for interface definitions.
See also the audio subsystem at ``nuttx/audio/``.

Audio Tone
----------

This driver has an interfaces which accepts a PWM lower-half and a oneshot timer
lower-half. Its features are enabled by ``CONFIG_AUDIO_TONE=y`` You can register
a tone driver as follows:

.. code:: c

   #include <nuttx/audio/tone.h>
   #include <nuttx/timers/pwm.h>

   /* Somewhere later in the code */

   int err;
   struct pwm_lowerhalf_s *pwm;         /* Assume initialized */
   struct oneshot_lowerhalf_s *oneshot; /* Assume initialized */

   /* Here, the argument '1' is the PWM channel to use */

   err = tone_register("/dev/tone0", pwm, 1, oneshot);
   if (err < 0)
     {
       syslog(LOG_ERR, "Couldn't register /dev/tone0: %d\n", err);
     }

The PWM interface generates the tone output, and the one-shot driver is used to
get the timing of musical notes correct.

If you want to try playing sound after the device is registered, you can write
`Music Macro Language (MML)
<https://en.wikipedia.org/wiki/Music_Macro_Language>`_ to the character driver.
For example, here is how you would play Ode to Joy:

.. code:: console

   nsh> echo "L8eefggfedccdeL6eL16dL4d" > /dev/tone0
