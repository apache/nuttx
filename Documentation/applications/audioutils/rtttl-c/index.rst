==========================================
``rtttl-c`` A simple RTTTL parsing library
==========================================

``rtttl-c`` is a simple library for parsing `Ring Tone Text Transfer Language`_
(RTTTL).

.. _Ring Tone Text Transfer Language: https://en.wikipedia.org/wiki/Ring_Tone_Text_Transfer_Language

To use the ``rtttl-c``, first include it from the audioutils::

        #include <audioutils/rtttl.h>

then define what to do with a tone::

        void
        play_tone(struct rtttl_tone tone)
        {
                /* TODO */
        }

and finally play a RTTTL string::

        rtttl_play("Jingle Bells:o=5,d=4,b=170,b=170:b,b,b,p,b,b,b,p,b,d6,g.,8a,2b.,8p,c6,c6,c6.,8c6,c6,b,b,8b,8b,b,a,a,b,2a,2d6", play_tone);
