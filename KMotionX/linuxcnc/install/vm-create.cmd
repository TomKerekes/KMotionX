@echo off
rem vm-create.cmd - makes the VirtualBox VM for LinuxCNC on KMotion, on a Windows host
rem (vm-create.sh is the same for Linux/macOS). Needs VirtualBox 7 and the Debian 13 netinst ISO.
rem Usage:   vm-create.cmd <debian-13 iso> [bridged host adapter] [shared folder]
rem    e.g.  vm-create.cmd C:\InstallLinuxCNC\debian-13.7.0-amd64-netinst.iso "Intel(R) Ethernet Controller I226-V #2" C:\kmotion-share
rem The bridged adapter is the host's network connection to the Kogna, by the name VirtualBox
rem shows ("VBoxManage list bridgedifs"); the Kogna is its own DHCP server on that link.
rem Then start the VM and install Debian (INSTALL.md, step 2).
setlocal
set VBM="%ProgramFiles%\Oracle\VirtualBox\VBoxManage.exe"
set NAME=KMotion-LinuxCNC
set RAM=4096
set CPUS=2
set DISK=40000
if "%~1"=="" ( echo usage: vm-create.cmd ^<debian-13 iso^> [bridged host adapter] [shared folder] & exit /b 2 )
if not exist "%~1" ( echo no such ISO: %~1 & exit /b 2 )
if "%~2"=="" ( echo host adapters for the bridge to the Kogna ^(give one as the 2nd argument, or none for now^): & %VBM% list bridgedifs | findstr /b Name: )
rem the adapter must be one VirtualBox lists under "Name:" (not the VBoxNetworkName line): checked before anything is made
if not "%~2"=="" (
    %VBM% list bridgedifs | findstr /b /c:"Name:" | findstr /e /c:"%~2" >nul || (
        echo no host adapter named "%~2". VirtualBox knows these ^(use the name after Name: exactly^):
        %VBM% list bridgedifs | findstr /b Name:
        exit /b 2
    )
)

%VBM% showvminfo "%NAME%" >nul 2>&1 && (
    echo a VM named "%NAME%" exists already. Remove it first ^(this deletes its disk too^):
    echo     %VBM% unregistervm "%NAME%" --delete
    exit /b 2
)
%VBM% createvm --name "%NAME%" --ostype Debian_64 --register || exit /b 1
rem where VirtualBox put it: read from a file (a command inside for /f loses the quotes around a path with a space)
set INFO=%TEMP%\kmotion-vm-info.txt
%VBM% showvminfo "%NAME%" --machinereadable > "%INFO%"
set CFG=
for /f "usebackq tokens=1,* delims==" %%a in ("%INFO%") do if "%%a"=="CfgFile" set CFG=%%~b
del "%INFO%"
if "%CFG%"=="" ( echo could not find where VirtualBox put the VM ^(showvminfo gave no CfgFile^) & exit /b 1 )
for %%f in ("%CFG%") do set VMDIR=%%~dpf
%VBM% modifyvm "%NAME%" --memory %RAM% --cpus %CPUS% --vram 64 --graphicscontroller vmsvga --usb on --usbohci on --clipboard-mode bidirectional --draganddrop bidirectional --audio-enabled off --nic1 nat || exit /b 1
if not "%~2"=="" %VBM% modifyvm "%NAME%" --nic2 bridged --bridgeadapter2 "%~2" --cableconnected2 on || exit /b 1
%VBM% createmedium disk --filename "%VMDIR%%NAME%.vdi" --size %DISK% || exit /b 1
%VBM% storagectl "%NAME%" --name SATA --add sata --controller IntelAhci || exit /b 1
%VBM% storageattach "%NAME%" --storagectl SATA --port 0 --device 0 --type hdd --medium "%VMDIR%%NAME%.vdi" || exit /b 1
%VBM% storageattach "%NAME%" --storagectl SATA --port 1 --device 0 --type dvddrive --medium "%~1" || exit /b 1
rem a KFLOP on USB (FTDI 0403:6001, a full-speed device: the OHCI controller serves it) goes to the VM when it is plugged in
%VBM% usbfilter add 0 --target "%NAME%" --name KFLOP --vendorid 0403 --productid 6001 || exit /b 1
if not "%~3"=="" (
    if not exist "%~3" mkdir "%~3"
    %VBM% sharedfolder add "%NAME%" --name kmotion --hostpath "%~3" --automount --auto-mount-point /mnt/kmotion || exit /b 1
)
echo VM "%NAME%" made. Start it and install Debian 13 (INSTALL.md, step 2):
echo     %VBM% startvm "%NAME%"
endlocal
