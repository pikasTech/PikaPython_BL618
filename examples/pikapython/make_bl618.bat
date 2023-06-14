@echo off
cd pikapython && rust-msc-latest-win10.exe && cd ..

set "MAKE_PATH=%~dp0..\..\tools\make"
set "NINJA_PATH=%~dp0..\..\tools\ninja"
set "GCC_PATH=%~dp0..\..\tools\toolchain_gcc_t-head_windows\bin"

set "NEW_PATH=%MAKE_PATH%;%NINJA_PATH%;%GCC_PATH%"

echo Checking if paths are already in the PATH variable...

if not defined NEW_PATH_ADDED (
    echo Adding NEW_PATH to the PATH variable...
    set "PATH=%PATH%;%NEW_PATH%"
    set "NEW_PATH_ADDED=1"
) else (
    echo NEW_PATH is already in the PATH variable.
)

IF NOT EXIST "%~dp0..\..\tools\toolchain_gcc_t-head_windows" (
    git clone https://gitee.com/bouffalolab/toolchain_gcc_t-head_windows "%~dp0..\..\tools\toolchain_gcc_t-head_windows"
)

make ninja CHIP=bl616 BOARD=bl616g0
