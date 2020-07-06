#!/bin/bash
#############################################################################
# tools/checkrelease.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
#############################################################################

set -e

BASE_URL="https://dist.apache.org/repos/dist/dev/incubator/nuttx"
TEMPDIR="dist.apache.org"
ORIGINAL_DIR="$(pwd)"
trap "{ cd $ORIGINAL_DIR; rm -rf $TEMPDIR; }" EXIT

function download_release() {
    wget -r -np -R "index.html*" -P . --cut-dirs 7 "$URL"
    cd "$TEMPDIR"
}

function check_sha512() {
    # check release sha512
    RELEASE_FILE=$1
    echo "Checking $RELEASE_FILE sha512..."
    sha512sum -c "$RELEASE_FILE.sha512"
}

function check_gpg() {
    # check nuttx sha512 and gpg
    RELEASE_FILE=$1
    echo "Checking $RELEASE_FILE GPG signature:"
    gpg --verify "$RELEASE_FILE.asc" "$RELEASE_FILE"
    echo
}

function check_required_files() {
    # check nuttx for required files
    RELEASE_FILE=$1
    RELEASE_DIR=$2 
    rm -rf "$RELEASE_DIR"
    tar xf "$RELEASE_FILE"
    ERROR=0
    if [ ! -f "$RELEASE_DIR/LICENSE" ]; then
        echo "LICENSE file not present."
        ERROR=1
    fi
    if [ ! -f "$RELEASE_DIR/NOTICE" ]; then
        echo "NOTICE file not present."
        ERROR=1
    fi
    if [ ! -f "$RELEASE_DIR/README.txt" ]; then
        echo "README.txt file not present."
        ERROR=1
    fi
    if [ ! -f "$RELEASE_DIR/DISCLAIMER-WIP" ]; then
        echo "DISCLAIMER-WIP file not present."
        ERROR=1
    fi
    if [ 0 -eq $ERROR ]; then
        echo "OK: All required files exist."
    fi
}

function check_nuttx() {
    # check nuttx sha512 and gpg
    RELEASE_FILE="$(ls *.tar.gz|head -1)"
    check_sha512 "$RELEASE_FILE" 
    check_gpg "$RELEASE_FILE"
    check_required_files "$RELEASE_FILE" "nuttx"
    mv "$RELEASE_FILE" ..
}

function check_nuttx_apps() {
    # check nuttx-apps sha512 and gpg
    RELEASE_FILE="$(ls *.tar.gz|head -2| tail -1)"
    check_sha512 "$RELEASE_FILE"
    check_gpg "$RELEASE_FILE"
    check_required_files "$RELEASE_FILE" "apps"
    mv "$RELEASE_FILE" ..
}

function usage() {
    echo "Usage: $0 <URL-of-release-candidate-directory-or-release-name>"
    echo "   Given release name or release full URL, downloads all files in"
    echo "   in that directory (which should include nuttx and nuttx-apps"
    echo "   sha512, asc, and tar.gz files), checks the release SHA512 and GPG "
    echo "   signatures, and checks the unpacked directories for required "
    echo "   files. Creates a temporary directory to do its work in."
    echo
    echo "   nuttx and nuttx-apps tar.gz files are left in the current"
    echo "   directory."
    echo 
    echo "Examples:"
    echo
    echo "  $0 9.1.0-RC1"
    echo "  $0 https://dist.apache.org/repos/dist/dev/incubator/nuttx/9.1.0-RC1/"
    echo
}

if [ "-h" == "$1" ]; then
    usage
    exit 0
fi

if [ -z "$1" ]; then
    usage
    exit 0
fi

ARG=$1
if [[ "$ARG" =~ ^"http".* ]]; then
  URL="$1/"
else
  URL="$BASE_URL/$1/"
fi

download_release
check_nuttx 
check_nuttx_apps 
