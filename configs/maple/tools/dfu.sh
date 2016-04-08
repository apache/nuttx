#!/bin/bash

dfu-util -a1 -d 1eaf:0003 -D nuttx.bin -R
