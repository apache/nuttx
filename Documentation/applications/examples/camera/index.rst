``camera`` Camera Snapshot
==========================

This sample is implemented as ``camera`` command on NuttX Shell. The synopsis of
the command is as below.::

  nsh> camera ([-jpg]) ([capture num])

  -jpg        : this option is set for storing JPEG file into a strage.
              : If this option isn't set capturing raw RGB565 data in a file.
              : raw RGB565 is default.

  capture num : this option instructs number of taking pictures.
              : 10 is default.

Storage will be selected automatically based on the available storage option.

Execution example::

  nsh> camera
  nximage_listener: Connected
  nximage_initialize: Screen resolution (320,240)
  Take 10 pictures as RGB file in /mnt/sd0 after 5 seconds.
  After finishing taking pictures, this app will be finished after 10 seconds.
  Expier time is pasted.
  Start capturing...
  FILENAME:/mnt/sd0/VIDEO001.RGB
  FILENAME:/mnt/sd0/VIDEO002.RGB
  FILENAME:/mnt/sd0/VIDEO003.RGB
  FILENAME:/mnt/sd0/VIDEO004.RGB
  FILENAME:/mnt/sd0/VIDEO005.RGB
  FILENAME:/mnt/sd0/VIDEO006.RGB
  FILENAME:/mnt/sd0/VIDEO007.RGB
  FILENAME:/mnt/sd0/VIDEO008.RGB
  FILENAME:/mnt/sd0/VIDEO009.RGB
  FILENAME:/mnt/sd0/VIDEO010.RGB
  Finished capturing...
  Expier time is pasted.
  nximage_listener: Lost server connection: 117

