# KMotion tool for Linux

The setup and troubleshooting screens of Dynomotion's KMotion.exe, for Linux: Digital I/O,
Axis and C Programs, in Python with Qt (PySide6) over KMotionX's library. It is one more
client of KMotionServer, so it works beside LinuxCNC's kmotion-motion or kmxWeb on the
same board; the board token serializes the commands.

    sudo apt install python3-pyside6.qtwidgets python3-pyside6.qtgui   # once
    make -C shim                                                      # builds ../../bin/libkmx_c.so
    python3 kmotion_tool.py

- `shim/kmx_c.cpp` is the only C++: an `extern "C"` layer over `CKMotionDLL` (open, console
  commands, the status block, compile/load, the console and error callbacks as queues).
- `kmx.py` loads it with ctypes and mirrors `MAIN_STATUS` field by field; the size is
  checked against the library's at load time, so a header change shows up immediately.
  `bit_state` / `bit_direction` know where every I/O bit lives in the status block, as
  `CDigitalPage` in KMotion.exe does.
- `io_screen.py`: the Digital I/O screen. Tabs as KMotion.exe's pages: bits 0-47, the Kogna's
  200-289 (Aux2/EX_IO with direction, differential inputs and outputs, opto outputs, RS485
  and ±15 V enables, HRPWM and SPI pins with their function/GPIO/I2C mux), virtual bits,
  Kanalog, SnapAmp. A click on State toggles the bit (`SetBit`/`ClearBit`), on Output flips
  the direction (`SetBitDirection`), the mux radios send `HRPWMSetMode`/`SPISetMode`. The
  status is read ten times a second in a thread.
- The Linux port's `MessageBox()` prints to the terminal and waits for Enter, which would
  freeze a GUI; the shim points the port's `mb_callback` at a handler that queues the text
  (it shows in the console dock as "error: ...") and answers OK, or No/Cancel to questions.
- `kmotion_tool.py`: the window, the status poller, the console dock. The board's `printf`
  output goes to the one client that last asked for it, so capturing it here takes it away
  from kmotion-motion: the checkbox in the console dock does that only on request.

- `axis_screen.py`: the Axis screen, 16 rows on a Kogna, 8 on a KFLOP: destination and
  position formatted as KMotion.exe does, Enable (`EnableAxis`/`DisableAxis`, parameters
  untouched), input and output modes from the status words, Done. The current gauges of the
  Windows screen (SnapAmp current sensing) are not there yet.
- `program_screen.py`: the C Programs screen. One file per thread 1-7, remembered between
  runs (QSettings); the editor with C highlighting; Compile (tcc67 for the connected board,
  the compiler's text below the editor, the cursor on the first error line), Download (Kill
  the thread, then LoadCoff), Run (Execute), All (the three in a row), Halt (Kill). The thread
  buttons turn green while their thread runs (ThreadActive). What KMotion.exe has beyond this:
  the TI compiler option, Find in Files, Go to Definition, reload on external modification.

- `config_screen.py`: the Config & Flash screen. One channel's settings as KMotion.exe's
  Config, Step Response and Filter screens hold them, in one form: modes (with the Step/Dir
  pulse mode and drive type packed into OutputChan0 the way `ConfigDlg` does it), input and
  output channels with gains and offsets, the tuning values, limit switch options (watch,
  polarity, bit numbers, action), soft limits, master/slave, backlash, motor values and the
  three IIR filters. Upload Channel reads them all from the board (the queries `Upload` in
  `StepDlg.cpp` makes), Download Channel writes them in the same order as
  `DownloadServoParams`, disabling an enabled axis first and re-enabling it at its
  destination afterwards; C Code -> Clipboard produces the `ch0->...=...;` lines of the
  Windows export, define names and number formats included. Flash: User Memory sends
  `FLASH` and waits for the board's Ready (up to 80 s), the Launch-on-Power-Up boxes send
  `SetStartupThread` and follow the status.
  The firmware buttons re-flash the board, the way KMotion.exe's Config screen does:
  New Version clears and re-loads the whole internal flash from a `.out` image (LoadCoff
  with pack-to-flash, then `ProgFlashImage`, up to 80 s under the token; on a Kogna it then
  offers to `reboot!`); Recovery on a KFLOP loads firmware into RAM over the USB boot loader
  for an un-bootable board (on a Kogna it shows the boot-jumper procedure instead, which is
  all KMotion.exe does there); Flash COM re-writes a Kogna's primary boot loader with TI's
  serial flash writer `sfh_OMAP-L138.exe` (a .NET program, so it needs `mono`:
  `sudo apt install mono-runtime`) over `/dev/ttyUSB*` with the board in UART boot mode.
  Status polling pauses while a flash owns the board.
- `console_screen.py`: the Console screen. A command goes out with echo and the board's
  reply lines are read back until its Ready, the way `CLogDlg::DoSend` works, with the
  token held so no other client interleaves; the board's own prompt line shows as well.
  Up/Down recall the history, Tab completes command names from `commands.txt` (169 names
  from the help's alphabetical list). Board printf output appears here too once the console
  is captured with the checkbox in the dock.

Status: all five screens built and exercised against the Kogna (status, bit toggles,
compile, channel upload/download round trip, console sends). The specification is the
Windows source in `~/KMotionSrcKogna/KMotion/` (`Digital*Page.cpp`, `DigitalPage.cpp`,
`AxisDlg.cpp`, `ProgramDlg.cpp`, `ConfigDlg.cpp`, `StepDlg.cpp`, `FilterDlg.cpp`,
`logdlg.cpp`, `KMotion.rc`).
