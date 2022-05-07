rem When MkrZero was programmed with some unrunnable code
rem and does't seem to be working:
rem Press it's button quickly twice, the led starts slowly
rem waving between on and off state, then run this command.
set BOSSAC="%USERPROFILE%\AppData\Local\arduino15\packages\arduino\tools\bossac\1.7.0-arduino3\bossac.exe"
%BOSSAC% -i -d --port=COM3 -U true -i -e -w -v ./MkrZeroBlink.bin -R


