############################################################################
# boards/arm/ht32f491x3/esk32/tools/flash.ps1
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

param(
  [Parameter(ValueFromRemainingArguments = $true)]
  [string[]]$RemainingArgs
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 2

$ScriptDir = $PSScriptRoot
$TopDir = [System.IO.Path]::GetFullPath((Join-Path $ScriptDir "..\..\..\..\.."))

$DefaultBin = Join-Path $TopDir "nuttx.bin"
$WindowsSetup = "Windows 10 Pro"
$PowerShellSetup = "Windows PowerShell 5.1 or newer"
$HT32IDEVersion = "HT32-IDE 1.0.6 (Build Date: 2025/12/04)"
$HT32IDERoot = "C:\Program Files (x86)\Holtek HT32 Series\HT32-IDE"
$OpenOCDPackage = "xPack OpenOCD 0.11.0-4"
$OpenOCDRoot = Join-Path $HT32IDERoot "xPack\xpack-openocd-0.11.0-4"
$OpenOCDExe = Join-Path $OpenOCDRoot "bin\openocd.exe"
$ScriptsDir = Join-Path $OpenOCDRoot "scripts"
$FlashLoader = Join-Path $OpenOCDRoot "FlashLoader\HT32F491x3_256.HLM"
$DeviceName = "HT32F49163_100LQFP"
$FlashBase = "0x08000000"
$FlashEnd = "0x0803FFFF"
$SRAMBase = "0x20000000"
$WorkAreaSize = "0xC000"
$BinPath = $DefaultBin
$DryRun = $false
$ProgName = $MyInvocation.MyCommand.Name

function Get-AbsolutePath {
  param(
    [string]$Path
  )

  if ([System.IO.Path]::IsPathRooted($Path)) {
    return [System.IO.Path]::GetFullPath($Path)
  }

  return [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Path))
}

function Format-CommandArgument {
  param(
    [string]$Argument
  )

  if ($Argument -match '[\s"]') {
    return '"' + ($Argument -replace '"', '\"') + '"'
  }

  return $Argument
}

function Show-Assumptions {
  Write-Host "############################################################################"
  Write-Host "# Assumptions:"
  Write-Host "#"
  Write-Host "#   - $WindowsSetup"
  Write-Host "#   - $PowerShellSetup"
  Write-Host "#   - This is the native Windows backend; use flash.py from the same"
  Write-Host "#     directory for automatic backend selection, or run this script directly"
  Write-Host "#   - $HT32IDEVersion installed at:"
  Write-Host "#       $HT32IDERoot"
  Write-Host "#   - $OpenOCDPackage available at:"
  Write-Host "#       $OpenOCDRoot"
  Write-Host "#   - Holtek HT-Link probe using interface/htlink.cfg"
  Write-Host "#   - ESK32 board with $DeviceName and FlashLoader\HT32F491x3_256.HLM"
  Write-Host "#"
  Write-Host "# Update this script if any of the above are not true."
  Write-Host "#"
  Write-Host "############################################################################"
  Write-Host ""
}

function Show-Usage {
  Write-Host "Usage: $ProgName [options]"
  Write-Host ""
  Write-Host "Options:"
  Write-Host "  --bin PATH         Binary to flash. Default: $DefaultBin"
  Write-Host "  --device NAME      Holtek expected device name. Default: $DeviceName"
  Write-Host "  --openocd-root DIR Holtek xPack OpenOCD root."
  Write-Host "  --dry-run          Print the OpenOCD command without executing it."
  Write-Host "  --help             Show this help."
  Write-Host ""
  Write-Host "Examples:"
  Write-Host "  .\$ProgName"
  Write-Host "  .\$ProgName --dry-run"
  Write-Host "  .\$ProgName --device HT32F49163_100LQFP"
}

function Fail {
  param(
    [string]$Message,
    [switch]$ShowUsage
  )

  [Console]::Error.WriteLine($Message)

  if ($ShowUsage) {
    Show-Usage
  }

  exit 1
}

Show-Assumptions

for ($i = 0; $i -lt $RemainingArgs.Count; $i++) {
  switch ($RemainingArgs[$i]) {
    "--bin" {
      if ($i + 1 -ge $RemainingArgs.Count) {
        Fail "Missing value for --bin" -ShowUsage
      }

      $i++
      $BinPath = Get-AbsolutePath $RemainingArgs[$i]
    }
    "--device" {
      if ($i + 1 -ge $RemainingArgs.Count) {
        Fail "Missing value for --device" -ShowUsage
      }

      $i++
      $DeviceName = $RemainingArgs[$i]
    }
    "--openocd-root" {
      if ($i + 1 -ge $RemainingArgs.Count) {
        Fail "Missing value for --openocd-root" -ShowUsage
      }

      $i++
      $OpenOCDRoot = Get-AbsolutePath $RemainingArgs[$i]
      $OpenOCDExe = Join-Path $OpenOCDRoot "bin\openocd.exe"
      $ScriptsDir = Join-Path $OpenOCDRoot "scripts"
      $FlashLoader = Join-Path $OpenOCDRoot "FlashLoader\HT32F491x3_256.HLM"
    }
    "--dry-run" {
      $DryRun = $true
    }
    "--help" {
      Show-Usage
      exit 0
    }
    "-h" {
      Show-Usage
      exit 0
    }
    default {
      Fail "Unknown argument: $($RemainingArgs[$i])" -ShowUsage
    }
  }
}

if (-not $DryRun) {
  if (-not (Test-Path -Path $BinPath -PathType Leaf)) {
    Fail "Binary not found: $BinPath"
  }

  if (-not (Test-Path -Path $OpenOCDExe -PathType Leaf)) {
    Fail "OpenOCD executable not found: $OpenOCDExe"
  }

  if (-not (Test-Path -Path $FlashLoader -PathType Leaf)) {
    Fail "Flash loader not found: $FlashLoader"
  }
}

$OpenOCDArgs = @(
  "-s"
  $ScriptsDir
  "-c"
  "hlm_SRAM $SRAMBase $WorkAreaSize"
  "-c"
  "hlm_loader {$FlashLoader} $FlashBase $FlashEnd"
  "-c"
  "ht_flags erase_sector"
  "-c"
  "set WORKAREASIZE $WorkAreaSize"
  "-f"
  "interface/htlink.cfg"
  "-f"
  "target/HLM491x3.cfg"
  "-c"
  "set_expected_name $DeviceName"
  "-c"
  "program $BinPath verify reset exit $FlashBase"
)

$CommandParts = @($OpenOCDExe) + $OpenOCDArgs
$CommandText = ($CommandParts | ForEach-Object { Format-CommandArgument $_ }) -join " "

Write-Host ("TOPDIR      : {0}" -f $TopDir)
Write-Host ("Binary      : {0}" -f $BinPath)
Write-Host ("Device      : {0}" -f $DeviceName)
Write-Host ("OpenOCD     : {0}" -f $OpenOCDExe)
Write-Host ("Flash loader: {0}" -f $FlashLoader)

if ($DryRun) {
  if (-not (Test-Path -Path $BinPath -PathType Leaf)) {
    Write-Host "Warning     : binary not found yet"
  }

  if (-not (Test-Path -Path $OpenOCDExe -PathType Leaf)) {
    Write-Host "Warning     : OpenOCD executable not found"
  }

  if (-not (Test-Path -Path $FlashLoader -PathType Leaf)) {
    Write-Host "Warning     : flash loader not found"
  }

  Write-Host ("Command     : {0}" -f $CommandText)
  exit 0
}

& $OpenOCDExe @OpenOCDArgs
exit $LASTEXITCODE
