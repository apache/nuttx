============================
``mcuboot`` MCUboot examples
============================

``swap_test``
=============

Description
-----------

MCUboot Swap Image is an application to demonstrate firmware upgrade using
internal flash memory. It simulate MCUboot API steps to switch between two
valid images.

This application add 3 Builtin Apps to NuttX NSH: version, set_img and confirm.
After application is build and ``nuttx.bin`` be generated, the binary must be
signed. Consult your board documentation page to get instructions how to do it.

How to build and flash
----------------------

First step is build your board configuration using ``mcuboot-loader`` as target.
That create the bootloader itself. The ``nuttx.bin`` must be flash as usual.

After that, clean up environment and set ``mcuboot-swap-test`` as target. The
build output will generate the ``nuttx.bin`` file. You should execute the MCUboot
script called ``imgtool.py`` and sign the binary file two times.

The first time you will use ``--version 1.0.0`` and ``signedv1.bin`` as output file.
Then, the second sign you need change to ``--version 2.0.0`` and ``signedv2.bin``
as output file.

The ``signedv1.bin`` file must be at MCUboot Slot-0 partition and ``signedv2.bin``
at Slot-1.

More instructions about how to sign and flash can be found at board documentation
page.

Running swap image test
-----------------------

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

Third step (let's reboot and see what's happen)::

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

Now, we switched from image version 1.0.0 to image 2.0.0. However, we intentionally
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

``mcuboot_local_agent``
=======================

MCUBoot Local Update Agent is an application to demonstrate firmware upgrade
using a binary file from local storage. Unlike remote update mechanisms, this
example copies the firmware binary directly from local storage (such as SD Card,
USB drive, or any mounted filesystem) to the secondary flash slot for MCUBoot
to process during the next boot.

This application provides a simple and reliable way to update firmware without
requiring network connectivity or complex remote update infrastructure.

Features:

* Copy firmware binary from local storage to secondary flash slot
* Progress indication during firmware copy
* Automatic size validation (ensures firmware fits in secondary slot)
* Flash area erasing and writing
* Automatic boot marking and system restart

Build and flash
----------------------

First, build your board configuration with MCUBoot, SD Card support and enable this tool under
`Application Configuration → Examples → MCUboot Examples → MCUBoot Local Update Agent`.

Flash the board normally.

Generating the update binary
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To create a firmware binary for update:

1. Configure the build to target the secondary slot::

     make menuconfig
     # Configure MCUBoot to build for secondary slot
     # (specific configuration varies by architecture)

2. Optionally modify the application (e.g., change MOTD) to visually identify
   the updated image

3. Build the update binary::

     make

4. The resulting binary should be copied to your local storage device (i.e. SD Card).
   Consult your board documentation for the specific binary filename and
   any required post-processing steps.

Running the local update agent
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Mount your storage device containing the firmware binary::

     nsh> mount -t vfat /dev/mmcsd0 /mnt

2. Verify the firmware file is accessible::

     nsh> ls /mnt
     /mnt:
      nuttx.bin
      readme.txt

3. Run the update agent with the firmware file path::

     nsh> mcuboot_local_agent /mnt/nuttx.bin

   If no path is specified, it defaults to ``/mnt/sdcard/nuttx.bin``

Example output
~~~~~~~~~~~~~~

When running the update agent, you should see output similar to::

  nsh> mcuboot_local_agent /mnt/nuttx.bin
  MCUBoot Local Update Agent
  Firmware file: /mnt/nuttx.bin
  Firmware file size: 1048576 bytes
  Erasing secondary flash slot...
  Copying firmware to secondary slot...
  Progress: 4096/1048576 bytes [0%]
  Progress: 8192/1048576 bytes [0%]
  Progress: 12288/1048576 bytes [1%]
  ...
  Progress: 1044480/1048576 bytes [99%]
  Progress: 1048576/1048576 bytes [100%]
  Firmware copy completed successfully!
  Firmware successfully copied to secondary slot!
  Update scheduled for next boot. Restarting...
  reboot status=0

After reboot, MCUBoot will detect the new firmware in the secondary slot and
perform the update. You should see MCUBoot messages indicating the swap process::

  *** Booting MCUboot build v2.2.0-rc1 ***
  ...
  Primary image: magic=good, swap_type=0x1, copy_done=0x3, image_ok=0x1
  Scratch: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
  Boot source: primary slot
  Image index: 0, Swap type: test
  Starting swap using scratch algorithm.
  ...
  This is MCUBoot Updated Image!!
  nsh>
