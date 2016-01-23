#!/bin/bash

cd tools
./configure.sh spark/$1
cd - > /dev/null
