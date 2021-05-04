@echo off
rem This script uploads .bin file into ArduinoMkrZero
rem Usage: UploadMkrZero ProjectName
rem NOTE: For this script to work from AtmelStudio change auto-generated Linker settings for the project:
rem Change "-Tflash_without_bootloader.ld" to "-Tflash_with_bootloader.ld"
rem SL (04.07.2020)

set COM_PORT=COM7
set PROJECT_NAME=%1%
set BIN_DIR=Release
set DEFAULT_BIN=..ino.bin

rem When MkrZero is stuck in a bootloader state, perform a direct bossac upload to bring it to normal state.
rem set BOSSAC=C:\Users\SL\AppData\Local\Arduino15\packages\arduino\tools\bossac\1.7.0-arduino3/bossac.exe 
rem %BOSSAC% -i -d --port=COM5 -U true -i -e -w -v Release\MkrTimer.bin -R 
rem Exit 0

rem Copy project binary to default name so arduino-cli uploader can find it
copy %BIN_DIR%\%PROJECT_NAME%.bin %BIN_DIR%\%DEFAULT_BIN%
IF %ERRORLEVEL% EQU 0 goto DoUpload
  Echo Failed to copy "%BIN_DIR%\%PROJECT_NAME%.BIN" into "%BIN_DIR%\%DEFAULT_BIN%". 
  Exit 1

:DoUpload
rem --log-level trace --verbose
c:\home\apps\arduino-cli\arduino-cli.exe upload --verbose --port %COM_PORT% --input-dir %BIN_DIR% --fqbn arduino:samd:mkrzero NotUsed
IF %ERRORLEVEL% EQU 0 goto DoExit 
echo Upload FAILED: errorlevel=%ERRORLEVEL%. 
Exit 1

:DoExit
del %BIN_DIR%\%DEFAULT_BIN%
echo ========== Upload: succeeded ==========
Exit 0
