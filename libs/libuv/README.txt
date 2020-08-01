README
=====

This README discusses how to setup libuv for NuttX.
Current port is based on libuv v1.38.1.

Install
=======

From NuttX root folder:

# Fetch libuv source code
git clone https://github.com/libuv/libuv.git -b v1.38.1 --depth=1 libs/libuv/libuv

# Apply patchs
cd libs/libuv/libuv
git am ../000*.patch

# Import headers in NuttX
cd ../../../
cp -r libs/libuv/libuv/include/* include/

