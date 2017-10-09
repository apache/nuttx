README
======

Basic Setup
===========

    +------------+
    |            |
    |         +--+
    |     USB |  | <----------------------------> USB Power Source
    |         +--+           +------------+
    |            |           |         +--+
    |         +--+           |         |  | <---> Keyboard
    |     USB |  | <-------->|   USB   +--+
    |         +--+           |   HUB      |
    |            |           |         +--+
    |            |           |         |  | <---> Mouse
    |            |           |         +--+
    |            |           +------------+
    |    Mini +--+
    |    HDMI |  | <----------------------------> Monitor/TV
    |         +--+
    |            |
    +------------+

  You might be able to use the hub to power the Pi Zero, but I was not
  able to do that; my hub would not switch on power until it was enumerated
  by the host (the Raspberry Pi Zero), but then I could not power the Pi
  with the hub because it was not providing power.  Chicken'n'Egg.
