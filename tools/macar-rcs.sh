#! /usr/bin/env bash

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This is an "ar rcs" equivalent without "no symbols" warnings.

# Background:
#
# NuttX assumes that it's ok to create
#
#  - An object without any symbols ("has no symbols")
#  - A library without any symbols ("the table of contents is empty")
#
# While macOS's ranlib/libtool can handle those cases,
# it produces warnings cited in the parentheses.
# NuttX developers are not happy with those warnings.
# NuttX developers are not happy with providing per-library dummy
# objects either.
#
# The "has no symbols" warning can be suppressed with
# the -no_warning_for_no_symbols option if you are using
# a recent enough version of ranlib/libtool.
# (Unfortunately, ar doesn't have a way to pass the option to ranlib.)
# However, there seems to be no way to suppress the
# "the table of contents is empty" warning. (thus the grep below)
#
# Reference:
#
# https://opensource.apple.com/source/cctools/cctools-949.0.1/misc/

set -e
ar rcS "$@"
# Note: the following line is using bash process substitution
ranlib -no_warning_for_no_symbols "$1" 2> >(grep -F -v "the table of contents is empty")
