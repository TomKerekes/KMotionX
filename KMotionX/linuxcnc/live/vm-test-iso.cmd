@echo off
rem vm-test-iso.cmd - boot the live ISO in a throwaway VirtualBox VM on the Windows host, to try it
rem before writing a USB stick: 4 GB, 2 CPUs, VBoxSVGA, OHCI USB with the KFLOP / Kogna COM port /
rem F310 filters, no disk (a live test; add one to try "Install"). Usage:
rem     vm-test-iso.cmd C:\ShareVM\kmotion\kmotion-linuxcnc-20261108.iso
rem Remove it afterwards:  VBoxManage unregistervm KMotion-Live-Test --delete
setlocal
set VBM="%ProgramFiles%\Oracle\VirtualBox\VBoxManage.exe"
set NAME=KMotion-Live-Test
if "%~1"=="" ( echo usage: vm-test-iso.cmd ^<iso^> & exit /b 2 )
if not exist "%~1" ( echo no such file: %~1 & exit /b 2 )
%VBM% showvminfo "%NAME%" >nul 2>&1 && ( echo a VM named "%NAME%" exists: %VBM% unregistervm "%NAME%" --delete & exit /b 2 )
%VBM% createvm --name "%NAME%" --ostype Debian_64 --register || exit /b 1
%VBM% modifyvm "%NAME%" --memory 4096 --cpus 2 --vram 128 --graphicscontroller vboxsvga --usb on --usbohci on --clipboard-mode bidirectional --audio-enabled off --boot1 dvd --boot2 none || exit /b 1
%VBM% storagectl "%NAME%" --name SATA --add sata --controller IntelAhci --portcount 2 || exit /b 1
%VBM% storageattach "%NAME%" --storagectl SATA --port 1 --device 0 --type dvddrive --medium "%~1" || exit /b 1
%VBM% usbfilter add 0 --target "%NAME%" --name KFLOP --vendorid 0403 --productid f231
%VBM% usbfilter add 1 --target "%NAME%" --name "KFLOP (FTDI id) / Kogna COM port" --vendorid 0403 --productid 6001
%VBM% usbfilter add 2 --target "%NAME%" --name "Logitech F310 game pad" --vendorid 046d --productid c21d
%VBM% startvm "%NAME%"
