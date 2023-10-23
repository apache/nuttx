============================
``mcuboot`` MCUboot examples
============================

``swap_test``
-------------

Description
~~~~~~~~~~~

MCUboot Swap Image is an application to demostrate firmware upgrade using
internal flash memory. It simulate MCUboot API steps to switch between two
valid images.

This application add 3 Builtin Apps to NuttX NSH: version, set_img and confirm.
After application is build and ``nuttx.bin`` be generated, the binary must be
signed. Consult your board README file to get instructions how to do it.

How to build and flash
......................

First step is build your board configuraton using ``mcuboot-loader`` as target.
That create the bootloader itself. The ``nuttx.bin`` must be flash as usual.

After that, clean up environment and set ``mcuboot-swap-test`` as target. The
build output will generate the ``nuttx.bin`` file. You should execute the MCUboot
script called ``imgtool.py`` and sign the binary file two times.

The first time you will use ``--version 1.0.0`` and ``signedv1.bin`` as output file.
Then, the second sign you need change to ``--version 2.0.0`` and ``signedv2.bin``
as output file.

The ``signedv1.bin`` file must be at MCUboot Slot-0 partition and ``signedv2.bin``
at Slot-1.

More instructions about how to sign and flash can be found at board README file.

Running swap image test
.......................

Open you terminal and reboot your board. You can see a similar output as below.
You can check builtin apps using command ``?``::

  *** Booting MCUboot build 7c890f4b075aed73e4c825ccf875b2fb9ebf2ded ***
  NuttShell (NSH) NuttX-10.2.0
  nsh> ?
  help usage:  help [-v] [<cmd>]

    .         cd        echo      hexdump   mv        rmdir     true      xd
    [         cp        exec      kill      printf    set       truncate
    ?         cmp       exit      ls        ps        sleep     uname
    basename  dirname   false     mkdir     pwd       source    umount
    break     dd        free      mkrd      reboot    test      unset
    cat       df        help      mount     rm        time      usleep

  Builtin Apps:
    mcuboot_set_img  mcuboot_confirm  sh
    mcuboot_version  ramtest          nsh
  nsh>

First step (check version)::

  nsh> mcuboot_version
  Image version 1.0.0.0
  nsh>

Second step (mark image as good because it is running). This is an optional
step that must be executed if you ran ``imgtool.py`` without optional parameter
``--confirm``::

  nsh> mcuboot_confirm
  Application Image successfully confirmed!
  nsh>

Third step (let's reboot and see whats happen)::

  nsh> reboot
  *** Booting MCUboot build 7c890f4b075aed73e4c825ccf875b2fb9ebf2ded ***
  NuttShell (NSH) NuttX-10.2.0
  nsh> mcuboot_version
  Image version 1.0.0.0
  nsh>

Fourth step (let's switch image)::

  nsh> mcuboot_set_img
  Requested update for next boot. Restarting...
  *** Booting MCUboot build 7c890f4b075aed73e4c825ccf875b2fb9ebf2ded ***
  NuttShell (NSH) NuttX-10.2.0
  nsh> mcuboot_version
  Image version 2.0.0.0
  nsh>

Now, we switched from image version 1.0.0 to image 2.0.0. However, we intentionaly
will not run ``mcuboot_confirm`` app::

  nsh> reboot
  *** Booting MCUboot build 7c890f4b075aed73e4c825ccf875b2fb9ebf2ded ***
  NuttShell (NSH) NuttX-10.2.0
  nsh> mcuboot_version
  Image version 1.0.0.0
  nsh>

This means that if for any reason App reboot, have a malfunctioning or not boot,
MCUboot will switch back to old ``good`` image! Remember that we executed
``mcuboot_confirm`` at step two.

Fifth step (switch to image version 2 and mark as permanent)::

  nsh> mcuboot_set_img
  Requested update for next boot. Restarting...
  *** Booting MCUboot build 7c890f4b075aed73e4c825ccf875b2fb9ebf2ded ***
  NuttShell (NSH) NuttX-10.2.0
  nsh> mcuboot_confirm
  Application Image successfully confirmed!
  nsh> mcuboot_version
  Image version 2.0.0.0
  nsh>

Sixth step (Reboot and confirm V2 image)::

  nsh> reboot
  *** Booting MCUboot build 7c890f4b075aed73e4c825ccf875b2fb9ebf2ded ***
  NuttShell (NSH) NuttX-10.2.0
  nsh> mcuboot_version
  Image version 2.0.0.0
  nsh>

Conclusion, once we boot a newer image and confirm it MCUboot always run that
image, unless you instruct it to swap again!
