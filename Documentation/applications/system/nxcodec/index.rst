================================================
``nxcodec`` NxCodec video codec test application
================================================

This application is a command-line tool that verifies the functionality of
NuttX codecs. Specifically, it serves as a verification tool for v4l2m2m.

Usage
-----

This page shows nxcodec options, For a complete list of nxcodec options
just run ``nxcodec -h``::

 ap> nxcodec -h
 NxCodec Version: 1.00
 Usage: nxcodec -d devname -s [wxh] -f [informt] -i infile -f [outformat] -o outfile
 Default settings for decoder parameters

 [-d | --device]  Video device name
 [-s | --size]    Size of stream
 [-h | --help]    Print this message
 [-f | --format]  Format of stream
 [-i | --infile]  Input filename for M2M devices
 [-o | --outfile] Outputs stream to filename

By default, it is the decodeing mode, with a default parameter size of
640x480, input format of H264, and output format of YUV420. The ``-i``
parameter corresponds to the input parameter before it, and the ``-i``
parameter corresponds to the output parameter after it.

Examples
--------

Decode an H264 stream file into a yuv420 file::

 mount -t hostfs -o fs=/path/from/ /stream
 nxcodec -d /dev/video1 -s 256x144 -i /stream/256x144.h264 -o /stream/256x144-yuv420p.yuv

Encode a yuv420 file as an h264 stream file::

 mount -t hostfs -o fs=/path/from/ /stream
 nxcodec -d /dev/video2 -s 256x144 -f YU12 -i /stream/256x144-yuv420p.yuv -f H264 -o /stream/256x144.h264

Author: ZhengHui SHI
Date: 9 July 2024
