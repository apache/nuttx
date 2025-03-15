#!/usr/bin/env pswd
############################################################################
# tools/ci/cibuild.ps1
# PowerShell script for CI on Windows Native
#
# SPDX-License-Identifier: Apache-2.0
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
############################################################################

# Set-PSDebug -Trace 0

Write-Host "===================================================================================="
$timestamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
Write-Host "Start CI build: $timestamp" -ForegroundColor Yellow
Write-Host "------------------------------------------------------------------------------------"


$myname = $($MyInvocation.MyCommand.Name)

$CID = Get-Location
$CIWORKSPACE = Resolve-Path("$CID\..\..\..")
$CIPLAT = "$CIWORKSPACE\nuttx\tools\ci\platforms"
$nuttx = "$CIWORKSPACE\nuttx"
$apps = "$CIWORKSPACE\apps"

if ($IsWindows -or ($Env:OS -match '^($|(Windows )?Win)')) {
    Write-Host "$ENV:OS"
}
else {
    Write-Host "Not Windows"
}

function install_tools {
    $NUTTXTOOLS = "$CIWORKSPACE\tools"
    $env:NUTTXTOOLS = "$NUTTXTOOLS"
    if (-not (Test-Path -Path $NUTTXTOOLS)) {
        New-Item -ItemType Directory -Path $NUTTXTOOLS -Force > $null
    }
    $Pathps1 = "$CIPLAT\windows.ps1"
    # Check if the file exists
    if (Test-Path $Pathps1) {
        try {
            # Run the script
            & $Pathps1
        }
        catch {
            # Handle errors
            Write-Host "An error occurred while executing the script: $_" -ForegroundColor Red
        }
    }
    else {
        Write-Host "The specified script does not exist: $Pathps1" -ForegroundColor Red
        exit 1
    }
}

# Function to display the help
function usage {
    Write-Host ""
    Write-Host "USAGE: $myname [-h] [-i] [-s] [-c] [-*] <testlist>"
    Write-Host ""
    Write-Host "Where:"
    Write-Host "  -i install tools"
    Write-Host "  -s setup repos"
    Write-Host "  -c enable ccache"
    Write-Host "  -* support all options in testbuild.ps1"
    Write-Host "  -h will show this help text and terminate"
    Write-Host "  <testlist> select testlist file"
    Write-Host ""
    exit 1
}

function enable_ccache {
    # Currently for windows not needed
    Write-Host "enable_ccache: to-do"
}

function setup_repos {
    $oldpath = Get-Location
    
    if (Test-Path $nuttx) {
        Set-Location "$nuttx"
        git pull
        
    }
    else {
        git clone https://github.com/apache/nuttx.git "$nuttx"
        Set-Location "$nuttx"
    }
    git log -1

    if (Test-Path $apps) {
        Set-Location "$apps"
        git pull
    }
    else {
        git clone https://github.com/apache/nuttx-apps.git "$apps"
        Set-Location "$apps"
    }
    git log -1

    Set-Location "$oldpath"
}

function run_builds {
    if ($null -eq $builds) {
        Write-Host "ERROR: Missing test list file" -ForegroundColor Yellow
        usage
    }
  
    foreach ( $build in $builds ) {
        & $nuttx\tools\testbuild.ps1 $options $build
    }

}

$builds = @()

if (!$args[0]) {
    usage
}
for ( $i = 0; $i -lt $args.count; $i++ ) {
    switch -regex -casesensitive ($($args[$i])) {
        '-h' {
            usage
        }
        '-i' {
            install_tools
            continue
        }
        '-c' {
            enable_ccache
            continue
        }
        '-s' {
            setup_repos
            continue
        }
        { $_ -like '-*' } {
            $options += "$($args[$i]) "
            continue
        }
        default {
            $builds += $($args[$i])
        }
    }
}

# Main script execution
run_builds
