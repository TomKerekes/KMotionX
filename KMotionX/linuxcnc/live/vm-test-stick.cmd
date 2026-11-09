@echo off
rem vm-test-stick.cmd - try the live image as a USB STICK in a throwaway VirtualBox VM on the Windows
rem host: the ISO is copied onto a virtual disk the way a stick is written (the image, then unused
rem space), and the VM starts from that disk. For "Keep Changes on USB" (desktop/usb-persistence):
rem a VM started from the ISO itself (vm-test-iso.cmd) has nothing to keep changes on. Usage:
rem     vm-test-stick.cmd C:\ShareVM\kmotion\kmotion-linuxcnc-20261109.iso [size in GB, default 16] ["host adapter to the Kogna"]
rem The disk is made next to the ISO as <iso name>-stick.vdi (the ISO's size at first; it grows with
rem what is kept). Remove both afterwards:  VBoxManage unregistervm KMotion-Stick-Test --delete
setlocal
set VBM="%ProgramFiles%\Oracle\VirtualBox\VBoxManage.exe"
set NAME=KMotion-Stick-Test
if "%~1"=="" ( echo usage: vm-test-stick.cmd ^<iso^> [GB] ["host adapter"] & exit /b 2 )
if not exist "%~1" ( echo no such file: %~1 & exit /b 2 )
set GB=%~2
if "%GB%"=="" set GB=16
set /a MB=%GB%*1024
set DISK=%~dpn1-stick.vdi
%VBM% showvminfo "%NAME%" >nul 2>&1 && ( echo a VM named "%NAME%" exists: %VBM% unregistervm "%NAME%" --delete & exit /b 2 )
if exist "%DISK%" ( echo "%DISK%" exists: delete it, or remove the old test VM with --delete & exit /b 2 )
echo writing the image onto %DISK% ...
%VBM% convertfromraw "%~1" "%DISK%" --format VDI || exit /b 1
%VBM% modifymedium disk "%DISK%" --resize %MB% || exit /b 1
%VBM% createvm --name "%NAME%" --ostype Debian_64 --register || exit /b 1
%VBM% modifyvm "%NAME%" --memory 4096 --cpus 2 --vram 128 --graphicscontroller vboxsvga --mouse usbtablet --usb on --usbohci on --clipboard-mode bidirectional --audio-enabled off --boot1 disk --boot2 none
if not "%~3"=="" ( %VBM% modifyvm "%NAME%" --nic2 bridged --bridgeadapter2 "%~3" --nicpromisc2 allow-all || exit /b 1 )
%VBM% storagectl "%NAME%" --name SATA --add sata --controller IntelAhci --portcount 2 || exit /b 1
%VBM% storageattach "%NAME%" --storagectl SATA --port 0 --device 0 --type hdd --medium "%DISK%" || exit /b 1
%VBM% usbfilter add 0 --target "%NAME%" --name KFLOP --vendorid 0403 --productid f231
%VBM% usbfilter add 1 --target "%NAME%" --name "KFLOP (FTDI id) / Kogna COM port" --vendorid 0403 --productid 6001
%VBM% usbfilter add 2 --target "%NAME%" --name "Logitech F310 game pad" --vendorid 046d --productid c21d
%VBM% startvm "%NAME%"
