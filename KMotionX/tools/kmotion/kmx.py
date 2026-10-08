"""kmx - KMotionX's library for Python, through libkmx_c.so (see shim/).

One KMotion object is one client of KMotionServer, which serves it beside other clients
such as LinuxCNC's kmotion-motion; the board token serializes the commands. Status comes
as the library's MAIN_STATUS block, mirrored here field by field (the size is checked
against the library's at load time).
"""
import ctypes
import os
from pathlib import Path

# sizes from PC-DSP.h
N_ADCS, N_ADCS_SNAP, N_DACS, N_PWMS, N_PWMS_SNAP = 8, 8, 8, 8, 4
N_CHANNELS_KOGNA, N_ADCS_KOGNA, N_DACS_KOGNA, N_IO_PWMS, N_KOGNA_HRPWM, N_PC_COMM_PERSIST = 16, 4, 8, 8, 4, 8
BOARD_TYPE_KMOTION, BOARD_TYPE_KFLOP, BOARD_TYPE_KOGNA = 1, 2, 3
BOARD_NAMES = {BOARD_TYPE_KMOTION: "KMotion", BOARD_TYPE_KFLOP: "KFLOP", BOARD_TYPE_KOGNA: "Kogna"}
MAX_LINE = 2560

# I/O bit numbering (PC-DSP.h)
N_BITS, N_VIRTUAL_BITS = 48, 16                 # 0-47 board bits, 48-63 virtual
SNAP0_BIT0, SNAP1_BIT0, N_SNAPAMP_BITS = 64, 96, 32
KANALOG_IN0, N_KANALOG_IN, KANALOG_OUT0, N_KANALOG_OUT = 128, 16, 144, 24
VIRTUAL_HI0 = 168                               # 168-183: the second 16 virtual bits
KOGNA_AUX2_EXIO_0, N_KOGNA_AUX2_EXIO = 200, 24  # 200-209 Aux2, 210-223 EX_IO
KOGNA_DIFF_IN0, N_KOGNA_DIFF_IN = 225, 24       # 225-248 differential inputs
KOGNA_DIFF_OUT0, N_KOGNA_DIFF_OUT = 250, 24     # 250-273 differential outputs
KOGNA_OPTO_OUT0, N_KOGNA_OPTO_OUT = 274, 4      # 274-277 opto outputs
KOGNA_RS485_MODE, KOGNA_EN_15V = 278, 279
KOGNA_HRPWM0, N_KOGNA_HRPWM_BITS = 280, 4       # 280-283, GPIO when the pin mux says so
KOGNA_SPI0, N_KOGNA_SPI = 284, 6                # 284-289, GPIO when the pin mux says so
VIRTUAL_BITS_EX = 1024                          # 1024-2047; the status carries 1024-1055


class MAIN_STATUS(ctypes.Structure):
    _fields_ = [
        ("VersionAndSize", ctypes.c_int),
        ("ADC", ctypes.c_int * (N_ADCS + 2 * N_ADCS_SNAP)),
        ("DAC", ctypes.c_int * N_DACS),
        ("PWM", ctypes.c_int * (N_PWMS + 2 * N_PWMS_SNAP)),
        ("Position", ctypes.c_double * N_CHANNELS_KOGNA),
        ("Dest", ctypes.c_double * N_CHANNELS_KOGNA),
        ("OutputChan0", ctypes.c_ubyte * N_CHANNELS_KOGNA),
        ("InputModes", ctypes.c_int), ("InputModes2", ctypes.c_int), ("InputModes3", ctypes.c_int), ("InputModes4", ctypes.c_int),
        ("OutputModes", ctypes.c_int), ("OutputModes2", ctypes.c_int), ("OutputModes3", ctypes.c_int), ("OutputModes4", ctypes.c_int),
        ("Enables", ctypes.c_int),
        ("AxisDone", ctypes.c_int),
        ("BitsDirection", ctypes.c_int * 2),
        ("BitsDirection200", ctypes.c_int),
        ("BitsDirection280", ctypes.c_int),
        ("BitsState", ctypes.c_int * 2),
        ("BitsState200", ctypes.c_int * 3),
        ("PinMuxModes", ctypes.c_int),
        ("Kogna_ADC", ctypes.c_short * N_ADCS_KOGNA),
        ("Kogna_DAC", ctypes.c_short * N_DACS_KOGNA),
        ("Kogna_PWM_Prescale", ctypes.c_ubyte),
        ("Kogna_PWM", ctypes.c_ubyte * N_IO_PWMS),
        ("Kogna_PWM_Enables", ctypes.c_ubyte * N_IO_PWMS),
        ("HRPWMPeriod01", ctypes.c_ushort), ("HRPWMPeriod2", ctypes.c_ushort), ("HRPWMPeriod3", ctypes.c_ushort),
        ("HRPWM", ctypes.c_ushort * N_KOGNA_HRPWM),
        ("SnapBitsDirection0", ctypes.c_int), ("SnapBitsDirection1", ctypes.c_int),
        ("SnapBitsState0", ctypes.c_int), ("SnapBitsState1", ctypes.c_int),
        ("KanalogBitsStateInputs", ctypes.c_int), ("KanalogBitsStateOutputs", ctypes.c_int),
        ("RunOnStartUp", ctypes.c_int),
        ("ThreadActive", ctypes.c_int),
        ("StopImmediateState", ctypes.c_int),
        ("TimeStamp", ctypes.c_double),
        ("PC_comm", ctypes.c_int * N_PC_COMM_PERSIST),
        ("VirtualBits", ctypes.c_int),
        ("VirtualBitsEx0", ctypes.c_int),
    ]

    def copy(self):
        c = MAIN_STATUS()
        ctypes.memmove(ctypes.byref(c), ctypes.byref(self), ctypes.sizeof(MAIN_STATUS))
        return c


def _bit(word, n):
    return (word >> n) & 1


def bit_state(st, b):
    """the state of I/O bit b as MAIN_STATUS reports it (None when it is not in the status)"""
    if 0 <= b < N_BITS + N_VIRTUAL_BITS:
        return _bit(st.BitsState[b // 32], b % 32)
    if SNAP0_BIT0 <= b < SNAP0_BIT0 + N_SNAPAMP_BITS:
        return _bit(st.SnapBitsState0, b - SNAP0_BIT0)
    if SNAP1_BIT0 <= b < SNAP1_BIT0 + N_SNAPAMP_BITS:
        return _bit(st.SnapBitsState1, b - SNAP1_BIT0)
    if KANALOG_IN0 <= b < KANALOG_IN0 + N_KANALOG_IN:
        return _bit(st.KanalogBitsStateInputs, b - KANALOG_IN0)
    if KANALOG_OUT0 <= b < KANALOG_OUT0 + N_KANALOG_OUT:
        return _bit(st.KanalogBitsStateOutputs, b - KANALOG_OUT0)
    if VIRTUAL_HI0 <= b < VIRTUAL_HI0 + 16:
        return _bit(st.VirtualBits, b - VIRTUAL_HI0 + 16)
    if KOGNA_AUX2_EXIO_0 <= b < KOGNA_SPI0 + N_KOGNA_SPI:
        return _bit(st.BitsState200[(b - KOGNA_AUX2_EXIO_0) // 32], (b - KOGNA_AUX2_EXIO_0) % 32)
    if VIRTUAL_BITS_EX <= b < VIRTUAL_BITS_EX + 32:
        return _bit(st.VirtualBitsEx0, b - VIRTUAL_BITS_EX)
    return None


def bit_direction(st, b):
    """1 = output, 0 = input, None when the bit has no direction in the status"""
    if 0 <= b < N_BITS + N_VIRTUAL_BITS:
        return _bit(st.BitsDirection[b // 32], b % 32)
    if SNAP0_BIT0 <= b < SNAP0_BIT0 + N_SNAPAMP_BITS:
        return _bit(st.SnapBitsDirection0, b - SNAP0_BIT0)
    if SNAP1_BIT0 <= b < SNAP1_BIT0 + N_SNAPAMP_BITS:
        return _bit(st.SnapBitsDirection1, b - SNAP1_BIT0)
    if KOGNA_AUX2_EXIO_0 <= b < KOGNA_AUX2_EXIO_0 + N_KOGNA_AUX2_EXIO:
        return _bit(st.BitsDirection200, b - KOGNA_AUX2_EXIO_0)
    if KOGNA_HRPWM0 <= b < KOGNA_SPI0 + N_KOGNA_SPI:
        return _bit(st.BitsDirection280, b - KOGNA_HRPWM0)
    return None


def pin_mux_mode(st, pin):
    """Kogna pin mux: pins 0-3 are the HRPWM bits 280-283, 4-9 the SPI bits 284-289;
    0 = the pin's function (HRPWM / SPI), 1 = GPIO, 2 = I2C (SPI pins 4 and 5 only)"""
    return (st.PinMuxModes >> (2 * pin)) & 3


class KMotionError(Exception):
    pass


def board_string(board, locations=()):
    """KMotion.exe's name for a board location (CMainFrame::BoardString, its title bar): a
    KFLOP's USB location ID in hex, a Kogna's IP address and its serial number when the
    server's board list has it (ListLocations gives each Kogna's IP followed by
    0xFF000000 | serial), else the number itself"""
    if -15 <= board <= 0:
        return str(board)
    u = board & 0xFFFFFFFF
    if u < 0x00FFFFFF:
        return f"KFLOP 0x{u:X}"
    sn = ""
    locs = [x & 0xFFFFFFFF for x in locations]
    for i in range(len(locs) - 1):
        if locs[i] == u and (locs[i + 1] & 0xFF000000) == 0xFF000000:
            sn = f" - SN{locs[i + 1] & 0xFFF}"
            break
    return f"Kogna {u >> 24 & 0xFF}.{u >> 16 & 0xFF}.{u >> 8 & 0xFF}.{u & 0xFF}{sn}"


def firmware_dir(board_type):
    """the DSP_KOGNA / DSP_KFLOP directory: this source tree's if the tool runs from it, else the
    installed one under ~/.kmotionx"""
    name = "DSP_KOGNA" if board_type == BOARD_TYPE_KOGNA else "DSP_KFLOP"
    here = Path(__file__).resolve()
    for base in (here.parents[3], Path.home() / ".kmotionx"):
        if (base / name).is_dir():
            return base / name
    return Path.home() / ".kmotionx" / name


def serial_ports():
    import glob
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


# the library's status codes, read from it at load time
KMOTION_LOCKED = KMOTION_IN_USE = KMOTION_READY = KMOTION_NOT_CONNECTED = KMOTION_TIMEOUT = None

# a channel's settings, the ones KMotion.exe's Config, Step Response and Filter screens hold,
# in the order its "C Code -> Clipboard" export writes them
CHANNEL_FLOATS = ["Vel", "Accel", "Jerk", "P", "I", "D", "FFAccel", "FFVel", "MaxI", "MaxErr", "MaxOutput",
                  "DeadBandGain", "DeadBandRange", "SoftLimitPos", "SoftLimitNeg", "InputGain0", "InputGain1",
                  "InputOffset0", "InputOffset1", "OutputGain", "OutputOffset", "BacklashAmount", "BacklashRate",
                  "Lead", "StepperAmplitude"]
CHANNEL_DOUBLES = ["SlaveGain", "InvDistPerCycle", "MaxFollowingError"]
CHANNEL_INTS = ["InputMode", "OutputMode", "BacklashMode", "InputChan0", "InputChan1", "OutputChan0", "OutputChan1",
                "MasterAxis", "LimitSwitchNegBit", "LimitSwitchPosBit"]
INPUT_MODE_DEFINES = ["NO_INPUT_MODE", "ENCODER_MODE", "ADC_MODE", "RESOLVER_MODE", "USER_INPUT_MODE", "SERIAL_SERVO_INPUT_MODE"]
OUTPUT_MODE_DEFINES = ["NO_OUTPUT_MODE", "MICROSTEP_MODE", "DC_SERVO_MODE", "BRUSHLESS_3PH_MODE", "BRUSHLESS_4PH_MODE",
                       "DAC_SERVO_MODE", "STEP_DIR_MODE", "CL_STEP_DIR_MODE", "CL_MICROSTEP_MODE", "SERIAL_SERVO_MODE",
                       "CL_SERIAL_SERVO_MODE"]
BACKLASH_MODE_DEFINES = ["BACKLASH_OFF", "BACKLASH_LINEAR"]
N_IIR_FILTERS = 3


class ChannelParams:
    """one axis channel's settings; floats/ints by name, the limit switch options decoded,
    iir[filter] = (B0, B1, B2, A1, A2)"""

    def __init__(self):
        self.values = {name: 0.0 for name in CHANNEL_FLOATS + CHANNEL_DOUBLES}
        self.values.update({name: 0 for name in CHANNEL_INTS})
        self.watch_neg = self.watch_pos = False
        self.polarity_neg = self.polarity_pos = False     # True: stop when low
        self.action = 0                                   # 0 kill drive, 1 disallow drive into the limit, 2 stop movement
        self.iir = [(1.0, 0.0, 0.0, 0.0, 0.0) for _ in range(N_IIR_FILTERS)]

    def limit_switch_options(self):
        """the LimitSwitchOptions word, with bit 8 (extended bit numbers) as KMotion.exe writes it"""
        return ((1 if self.watch_neg else 0) | (2 if self.watch_pos else 0) | (4 if self.polarity_neg else 0) |
                (8 if self.polarity_pos else 0) | ((self.action & 0xf) << 4) | (1 << 8))

    def set_limit_switch_options(self, h):
        self.watch_neg, self.watch_pos = bool(h & 1), bool(h & 2)
        self.polarity_neg, self.polarity_pos = bool(h & 4), bool(h & 8)
        self.action = (h >> 4) & 0xf
        return bool(h & 0x100)                            # extended bit numbers in use

    def to_c(self, ch):
        """the channel as C statements, as KMotion.exe's "C Code -> Clipboard" writes them"""
        v = self.values
        g6 = lambda x: f"{x:.6g}"
        lines = [("InputMode", INPUT_MODE_DEFINES[v["InputMode"]] if 0 <= v["InputMode"] < len(INPUT_MODE_DEFINES) else str(v["InputMode"])),
                 ("OutputMode", OUTPUT_MODE_DEFINES[v["OutputMode"]] if 0 <= v["OutputMode"] < len(OUTPUT_MODE_DEFINES) else str(v["OutputMode"]))]
        lines += [(n, g6(v[n])) for n in ("Vel", "Accel", "Jerk", "P", "I", "D", "FFAccel", "FFVel", "MaxI", "MaxErr", "MaxOutput", "DeadBandGain", "DeadBandRange")]
        lines += [(n, str(int(v[n]))) for n in ("InputChan0", "InputChan1", "OutputChan0", "OutputChan1", "MasterAxis")]
        lines += [("LimitSwitchOptions", f"0x{self.limit_switch_options():x}"),
                  ("LimitSwitchNegBit", str(int(v["LimitSwitchNegBit"]))), ("LimitSwitchPosBit", str(int(v["LimitSwitchPosBit"])))]
        lines += [(n, g6(v[n])) for n in ("SoftLimitPos", "SoftLimitNeg", "InputGain0", "InputGain1", "InputOffset0", "InputOffset1", "OutputGain", "OutputOffset")]
        lines += [("SlaveGain", f"{v['SlaveGain']:.13g}"),
                  ("BacklashMode", BACKLASH_MODE_DEFINES[v["BacklashMode"]] if 0 <= v["BacklashMode"] < 2 else str(v["BacklashMode"])),
                  ("BacklashAmount", g6(v["BacklashAmount"])), ("BacklashRate", g6(v["BacklashRate"])),
                  ("invDistPerCycle", f"{v['InvDistPerCycle']:.13g}"), ("Lead", g6(v["Lead"])),
                  ("MaxFollowingError", f"{v['MaxFollowingError']:.13g}"), ("StepperAmplitude", g6(v["StepperAmplitude"]))]
        out = [f"\tch{ch}->{m}={val};" for m, val in lines]
        for f in range(N_IIR_FILTERS):
            out.append("")
            b0, b1, b2, a1, a2 = self.iir[f]
            for coeff, val in (("B0", b0), ("B1", b1), ("B2", b2), ("A1", a1), ("A2", a2)):
                out.append(f"\tch{ch}->iir[{f}].{coeff}={val:.7g};")
        return "\n".join(out) + "\n"


def _find_library():
    env = os.environ.get("KMX_C_LIB")
    candidates = [env] if env else []
    here = Path(__file__).resolve()
    candidates += [here.parents[3] / "bin" / "libkmx_c.so",            # KMotionX/bin in this tree
                   Path.home() / ".kmotionx" / "bin" / "libkmx_c.so",   # the installed tree
                   Path("libkmx_c.so")]
    for c in candidates:
        if c and Path(c).exists():
            return str(c)
    raise KMotionError("libkmx_c.so not found (build tools/kmotion/shim, or set KMX_C_LIB)")


_lib = None


def lib():
    global _lib
    if _lib is None:
        l = ctypes.CDLL(_find_library())
        l.kmx_open.restype = ctypes.c_void_p
        l.kmx_open.argtypes = [ctypes.c_int]
        l.kmx_close.argtypes = [ctypes.c_void_p]
        l.kmx_set_console.argtypes = [ctypes.c_void_p]
        l.kmx_service_console.argtypes = [ctypes.c_void_p]
        l.kmx_console_read.argtypes = [ctypes.c_char_p, ctypes.c_int]
        l.kmx_error_read.argtypes = [ctypes.c_char_p, ctypes.c_int]
        l.kmx_write_line.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        l.kmx_write_read.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        l.kmx_check_version.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
        l.kmx_board_type.argtypes = [ctypes.c_void_p]
        l.kmx_status.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        l.kmx_compile_load.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        l.kmx_load.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        l.kmx_compile.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        l.kmx_wait_token.argtypes = [ctypes.c_void_p, ctypes.c_int]
        l.kmx_release_token.argtypes = [ctypes.c_void_p]
        l.kmx_write_line_echo.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        l.kmx_read_line_timeout.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
        l.kmx_check_ready.argtypes = [ctypes.c_void_p]
        l.kmx_disconnect.argtypes = [ctypes.c_void_p]
        l.kmx_const.argtypes = [ctypes.c_int]
        l.kmx_load_pack.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        l.kmx_lock_recovery.argtypes = [ctypes.c_void_p]
        l.kmx_flash_new_version.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        l.kmx_out_name.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        l.kmx_list_locations.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int]
        l.kmx_usb_location.argtypes = [ctypes.c_void_p]
        global KMOTION_LOCKED, KMOTION_IN_USE, KMOTION_READY, KMOTION_NOT_CONNECTED, KMOTION_TIMEOUT
        KMOTION_LOCKED, KMOTION_IN_USE, KMOTION_READY, KMOTION_NOT_CONNECTED, KMOTION_TIMEOUT = (l.kmx_const(i) for i in range(5))
        size = l.kmx_status_size()
        if size != ctypes.sizeof(MAIN_STATUS):
            raise KMotionError(f"MAIN_STATUS is {size} bytes in the library but {ctypes.sizeof(MAIN_STATUS)} here: update kmx.py")
        _lib = l
    return _lib


class KMotion:
    """one connection to KMotionServer and the board it serves"""

    def __init__(self, board_id=0):
        self._l = lib()
        self._h = self._l.kmx_open(board_id)
        if not self._h:
            raise KMotionError("kmx_open failed")

    def close(self):
        if self._h:
            self._l.kmx_close(self._h)
            self._h = None

    # ---- console commands ----
    def write(self, cmd):
        """send one console command; raises on a transport failure"""
        rc = self._l.kmx_write_line(self._h, cmd.encode())
        if rc:
            raise KMotionError(f"{cmd}: write failed ({rc})")

    def query(self, cmd):
        """send one console command and return its one-line reply"""
        buf = ctypes.create_string_buffer(MAX_LINE + 1)
        rc = self._l.kmx_write_read(self._h, cmd.encode(), buf, MAX_LINE)
        if rc:
            raise KMotionError(f"{cmd}: no reply ({rc})")
        return buf.value.decode(errors="replace").strip()

    def board_type(self):
        """BOARD_TYPE_* or -1 when no board answers"""
        return self._l.kmx_board_type(self._h)

    def check_version(self):
        """the library's firmware/version check; (rc, board type)"""
        t = ctypes.c_int(0)
        rc = self._l.kmx_check_version(self._h, ctypes.byref(t))
        return rc, t.value

    def status(self):
        st = MAIN_STATUS()
        rc = self._l.kmx_status(self._h, ctypes.byref(st), ctypes.sizeof(st))
        if rc:
            raise KMotionError(f"GetStatus failed ({rc})")
        return st

    def usb_location(self):
        """the board this connection is on, as the server knows it: a KFLOP's USB location ID or
        a Kogna's IP address (a.b.c.d from the high byte down); -1 while the server has no
        connection to it. Asks only the server, never the board"""
        return self._l.kmx_usb_location(self._h)

    def locations(self):
        arr = (ctypes.c_int * 64)()
        n = self._l.kmx_list_locations(self._h, arr, 64)
        return [arr[i] for i in range(max(n, 0))]

    # ---- C programs ----
    def compile_load(self, path, thread):
        """compile with tcc67 for the connected board and download to the thread; '' or the error text"""
        err = ctypes.create_string_buffer(4096)
        rc = self._l.kmx_compile_load(self._h, str(path).encode(), thread, err, 4096)
        if rc:
            return err.value.decode(errors="replace") or f"compile/load failed ({rc})"
        return ""

    def load(self, path, thread):
        return self._l.kmx_load(self._h, str(path).encode(), thread)

    def out_name(self, path, thread):
        """the .out file the library compiles a source into for a thread"""
        buf = ctypes.create_string_buffer(1024)
        self._l.kmx_out_name(self._h, thread, str(path).encode(), buf, 1024)
        return buf.value.decode(errors="replace")

    def compile(self, path, thread, board_type):
        """compile only, with tcc67 for the board type; '' or the compiler's output"""
        err = ctypes.create_string_buffer(16384)
        rc = self._l.kmx_compile(self._h, str(path).encode(), self.out_name(path, thread).encode(), board_type, thread, err, 16384)
        if rc:
            return err.value.decode(errors="replace") or f"compile failed ({rc})"
        return ""

    def execute(self, thread):
        self.write(f"Execute{thread}")

    def kill(self, thread):
        self.write(f"Kill{thread}")

    # ---- the console's way of sending: echo and read the reply until "Ready" ----
    def send_console(self, cmd, timeout_ms=3000):
        """as KMotion.exe's console: the command with echo, then every reply line until the
        board's "Ready" (or the timeout); returns the lines. "flash" waits a minute,
        "reboot!" disconnects instead of waiting."""
        lines = []
        if self._l.kmx_wait_token(self._h, 5000) != KMOTION_LOCKED:
            raise KMotionError("the board is busy (no token)")
        try:
            if self._l.kmx_write_line_echo(self._h, cmd.encode()):
                raise KMotionError(f"{cmd}: write failed")
            low = cmd.strip().lower()
            if low == "reboot!":
                self._l.kmx_disconnect(self._h)
                return lines
            if low == "flash":
                timeout_ms = 60000
            buf = ctypes.create_string_buffer(MAX_LINE + 1)
            while True:
                rc = self._l.kmx_read_line_timeout(self._h, buf, MAX_LINE, timeout_ms)
                if rc:
                    lines.append("(no Ready within %.1f s)" % (timeout_ms / 1000.0))
                    break
                line = buf.value.decode(errors="replace")
                if line.strip() == "Ready":
                    break
                lines.append(line.rstrip("\r\n"))
        finally:
            self._l.kmx_release_token(self._h)
        return lines

    def flash_user_memory(self, timeout_s=80.0):
        """FLASH: the user programs and settings into the board's flash; waits for the board"""
        import time
        if self._l.kmx_wait_token(self._h, 5000) != KMOTION_LOCKED:
            raise KMotionError("the board is busy (no token)")
        try:
            if self._l.kmx_write_line_echo(self._h, b"FLASH"):
                raise KMotionError("FLASH: write failed")
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                time.sleep(0.5)
                r = self._l.kmx_check_ready(self._h)
                if r == KMOTION_READY:
                    return True
            return False
        finally:
            self._l.kmx_release_token(self._h)

    # ---- firmware ----
    def flash_new_version(self, image, timeout_s=80):
        """re-flash the board's firmware from a .out image, as KMotion.exe's New Version:
        0 done (reboot the board), 1 timeout, 2 the board reported an error, 3 download failed"""
        return self._l.kmx_flash_new_version(self._h, str(image).encode(), timeout_s)

    def recover_kflop(self, image, keep_waiting):
        """KMotion.exe's Flash Recovery for a KFLOP whose flash is corrupt: with the board
        powered on into its USB boot loader, lock it, hand-shake, load the firmware into RAM
        and jump to it. keep_waiting() is polled while waiting for the boot loader; returns
        '' when the firmware runs, otherwise the failure."""
        while True:
            if self._l.kmx_lock_recovery(self._h) == KMOTION_LOCKED:
                break
            if not keep_waiting():
                return "no boot loader connection"
        self._l.kmx_release_token(self._h)
        if self._l.kmx_write_line_echo(self._h, b"\x1bB"):
            return "boot loader did not take the command"
        buf = ctypes.create_string_buffer(MAX_LINE + 1)
        if self._l.kmx_read_line_timeout(self._h, buf, MAX_LINE, 5000) or buf.value != b"\x03\r\n":
            return "no acknowledge from the boot loader"
        if self._l.kmx_load_pack(self._h, str(image).encode(), 2):
            return "loading the firmware into RAM failed"
        self.write("JUMP 10000010")
        self._l.kmx_disconnect(self._h)
        return ""

    def set_startup_thread(self, thread, on):
        self.write(f"SetStartupThread {thread} {1 if on else 0}")

    # ---- a channel's settings, as the Config / Step / Filter screens move them ----
    def upload_channel(self, ch):
        p = ChannelParams()
        for name in CHANNEL_FLOATS + CHANNEL_DOUBLES:
            p.values[name] = float(self.query(f"{name}{ch}"))
        for name in CHANNEL_INTS:
            if name in ("LimitSwitchNegBit", "LimitSwitchPosBit"):
                continue
            p.values[name] = int(float(self.query(f"{name}{ch}")))
        h = int(self.query(f"LimitSwitch{ch}"), 16)
        extended = p.set_limit_switch_options(h)
        if extended:
            p.values["LimitSwitchNegBit"] = int(float(self.query(f"LimitSwitchNegBit{ch}")))
            p.values["LimitSwitchPosBit"] = int(float(self.query(f"LimitSwitchPosBit{ch}")))
        else:                                             # legacy: packed in the options word
            p.values["LimitSwitchNegBit"] = (h >> 16) & 0xff
            p.values["LimitSwitchPosBit"] = (h >> 24) & 0xff
        for f in range(N_IIR_FILTERS):
            parts = self.query(f"IIR{ch} {f}").split()
            if len(parts) >= 5:
                a1, a2, b0, b1, b2 = (float(x) for x in parts[:5])
                p.iir[f] = (b0, b1, b2, a1, a2)
        return p

    def download_channel(self, ch, p):
        """as DownloadServoParams: a disabled axis takes the settings, an enabled one is
        disabled first and re-enabled at its destination afterwards"""
        enabled = self.query(f"Enabled{ch}").strip() == "1"
        dest = float(self.query(f"Dest{ch}")) if enabled else 0.0
        if enabled:
            self.write(f"DisableAxis{ch}")
        v = p.values
        for name in ("P", "I", "D", "Vel", "Accel", "Jerk", "FFVel", "FFAccel", "MaxI", "MaxErr", "MaxOutput", "DeadBandGain", "DeadBandRange"):
            self.write(f"{name}{ch}={v[name]:.8g}")
        for name in ("InputMode", "OutputMode", "InputChan0", "InputChan1", "OutputChan0", "OutputChan1", "MasterAxis"):
            self.write(f"{name}{ch}={int(v[name])}")
        self.write(f"LimitSwitch{ch}={p.limit_switch_options():X}")
        self.write(f"LimitSwitchNegBit{ch}={int(v['LimitSwitchNegBit'])}")
        self.write(f"LimitSwitchPosBit{ch}={int(v['LimitSwitchPosBit'])}")
        for name in ("SoftLimitPos", "SoftLimitNeg", "InputGain0", "InputGain1", "InputOffset0", "InputOffset1", "OutputGain", "OutputOffset"):
            self.write(f"{name}{ch}={v[name]:.8g}")
        self.write(f"SlaveGain{ch}={v['SlaveGain']:.13g}")
        self.write(f"BacklashMode{ch}={int(v['BacklashMode'])}")
        self.write(f"BacklashAmount{ch}={v['BacklashAmount']:.8g}")
        self.write(f"BacklashRate{ch}={v['BacklashRate']:.8g}")
        self.write(f"InvDistPerCycle{ch}={v['InvDistPerCycle']:.13g}")
        self.write(f"Lead{ch}={v['Lead']:.8g}")
        self.write(f"MaxFollowingError{ch}={v['MaxFollowingError']:.13g}")
        self.write(f"StepperAmplitude{ch}={v['StepperAmplitude']:.8g}")
        for f in range(N_IIR_FILTERS):
            b0, b1, b2, a1, a2 = p.iir[f]
            self.write(f"IIR{ch} {f}={a1:f} {a2:f} {b0:f} {b1:f} {b2:f}")
        if enabled:
            self.write(f"EnableAxisDest{ch} {dest:.8g}")

    # ---- console output (the server sends it to the last client that asked) ----
    def capture_console(self):
        return self._l.kmx_set_console(self._h)

    def service_console(self):
        return self._l.kmx_service_console(self._h)

    def console_lines(self):
        return _drain(self._l.kmx_console_read)

    def errors(self):
        return _drain(self._l.kmx_error_read)

    # ---- I/O bits ----
    def set_bit(self, b, on):
        self.write(f"{'SetBit' if on else 'ClearBit'} {b}")

    def set_bit_direction(self, b, output):
        self.write(f"SetBitDirection {b} {1 if output else 0}")

    def set_hrpwm_mode(self, pin, mode):
        self.write(f"HRPWMSetMode{pin}={mode}")

    def set_spi_mode(self, pin, mode):
        self.write(f"SPISetMode{pin}={mode}")


def _drain(fn):
    out = []
    buf = ctypes.create_string_buffer(MAX_LINE + 1)
    while True:
        n = fn(buf, MAX_LINE)
        if n <= 0:
            break
        out.append(buf.value.decode(errors="replace").rstrip("\r\n"))
    return out


if __name__ == "__main__":
    # a smoke test: connect, identify the board, read the status a few times
    import time
    km = KMotion()
    t = km.board_type()
    print("board:", BOARD_NAMES.get(t, t), "| version query:", km.query("Version"))
    for i in range(3):
        st = km.status()
        print(f"status {i}: TimeStamp {st.TimeStamp:.3f}, Enables 0x{st.Enables & 0xffff:04x}, ThreadActive 0x{st.ThreadActive:02x}, "
              f"Dest[0..2] {st.Dest[0]:.0f} {st.Dest[1]:.0f} {st.Dest[2]:.0f}, BitsState 0x{st.BitsState[0] & 0xffffffff:08x} 0x{st.BitsState[1] & 0xffffffff:08x}, "
              f"Dir 0x{st.BitsDirection[0] & 0xffffffff:08x} 0x{st.BitsDirection[1] & 0xffffffff:08x}, LEDs 46/47 state {bit_state(st, 46)}{bit_state(st, 47)} dir {bit_direction(st, 46)}{bit_direction(st, 47)}, "
              f"PinMux 0x{st.PinMuxModes:05x}, Virtual 0x{st.VirtualBits & 0xffffffff:08x}")
        time.sleep(0.1)
    print("errors:", km.errors())
    km.close()
