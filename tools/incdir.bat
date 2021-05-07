@echo off

rem tools/incdir.sh
rem
rem Licensed to the Apache Software Foundation (ASF) under one or more
rem contributor license agreements.  See the NOTICE file distributed with
rem this work for additional information regarding copyright ownership.  The
rem ASF licenses this file to you under the Apache License, Version 2.0 (the
rem "License"); you may not use this file except in compliance with the
rem License.  You may obtain a copy of the License at
rem
rem   http://www.apache.org/licenses/LICENSE-2.0
rem
rem Unless required by applicable law or agreed to in writing, software
rem distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
rem WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
rem License for the specific language governing permissions and limitations
rem under the License.
rem

rem Handle command line options

set progname=%0
set pathtype=user

:ArgLoop

rem [-d] [-w] [-s] [-h]. [-w] and [-d] Ignored for compatibility with incdir.sh

if "%1"=="-d" goto :NextArg
if "%1"=="-w" goto :NextArg
if "%1"=="-h" goto :Usage

if "%1"=="-s" (
  set pathtype=system
  goto :NextArg
)

goto :CheckCompiler

:NextArg
shift
goto :ArgLoop

:CheckCompiler
if "%1"=="" (
  echo ERROR: Missing compiler name
  goto :Usage
)

set ccpath=%1
shift

set compiler=
for /F %%i in ("%ccpath%") do set compiler=%%~ni

if "%1"=="" (
  echo ERROR: Missing directory paths
  goto :Usage
)

rem Check for some well known, non-GCC Windows native tools that require
rem a special output format as well as special paths

:GetFormat
set fmt=std
if "%compiler%"=="ez8cc" goto :SetZdsFormt
if "%compiler%"=="zneocc" goto :SetZdsFormt
if "%compiler%"=="ez80cc" goto :SetZdsFormt
goto :GeneratePaths

:SetZdsFormt
set fmt=zds

rem Generate the compiler include path directives.

:GeneratePaths
set response=

:DirLoop
if "%1" == "" (
  echo %response%
  goto :End
)

if "%fmt%"=="zds" goto :GenerateZdsPath
if "%response%"=="" goto :FirstStdPath
if "%pathtype%"=="system" goto :NextStdSystemPath

set response=%response% -I "%1"
goto :EndOfDirLoop

:NextStdSystemPath

set response=%response% -isystem "%1"
goto :EndOfDirLoop

:FirstStdPath

if "%pathtype%"=="system" goto :FirstStdSystemPath
set response=-I "%1"
goto :EndOfDirLoop

:FirstStdSystemPath

set response=-isystem "%1"
goto :EndOfDirLoop

:GenerateZdsPath

if "%response%"=="" goto :FirstZdsPath
set response=%response%;%1
goto :EndOfDirLoop

:FirstZdsPath

if "%pathtype%"=="system" goto :FirstZdsSystemPath
set response=-usrinc:%1
goto :EndOfDirLoop

:FirstZdsSystemPath

set response=-stdinc:%1

:EndOfDirLoop
shift
goto :DirLoop

:Usage
echo %progname% is a tool for flexible generation of include path arguments for a
echo variety of different compilers in a variety of compilation environments
echo USAGE: %progname% [-w] [-d] [-s] [-h] ^<compiler-path^> ^<dir1^> [^<dir2^> [^<dir3^> ...]]
echo Where:
echo  ^<compiler-path^>
echo    The full path to your compiler
echo  ^<dir1^> [^<dir2^> [^<dir3^> ...]]
echo    A list of include directories
echo  -w, -d
echo    For compatibility with incdir.sh (ignored)
echo  -s
echo    Generate standard, system header file paths instead of normal user
echo    header file paths.
echo  -h
echo    Shows this help text and exits.
:End
