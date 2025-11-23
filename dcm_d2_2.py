"""
DCM — Deliverable 2
==========================================================

HOW TO RUN
----------
RUN THROUGH IDLE, F5.
NEED PYQT5

MAIN FEATURES
-------------------
- Register/Login with a limit of 10 users (stored locally in JSON; passwords hashed)
- Dashboard  shows comms/device/telemetry state and lets you pick a pacing mode
- Opens a Mode Editor (AOO/VOO/AAI/VVI) with customizable parameters within provided ranges and steps:
    * URL, LRL, A/V Amplitude (+On/Off), A/V Pulse Width, Refractory Period, Hysteresis and Rate Smoothing Up/Down
- Summary tab that mirrors the current parameter set
- Egram (D2) tab with a live simulated 3-trace viewer and option to save PNG 
- Exports: 
  • File → Reports → Bradycardia Parameters Report…  (PDF)
  • File → Reports → Temporary Parameters Report…    (PDF)
  • File → Export Saved Params JSON…                 (JSON)
- Simulator menu to change states: Comms Connected, Device Changed, Device ID, Telemetry (OK/Out of Range/Noise)
- Utilities → About… and Utilities → Set Clock… (change device time shown in status bar)

DATA IN LOCAL JSON FILE: dcm_d1_file.json
{
  "users":  [ {"username": "<name>", "password_hash": "<sha256>"}, ... ],
  "params": { "<user>:<mode>": { ... parameter dict ... }, ... }
}

Saved parameters are per-(user, mode). Unsaved (current) values are taken directly from the widgets.

WIDGETS emit signals, METHODS connected to them button.clicked.connect(self.on_click), button.clicked signal

Navigation works by passing CALLBACKS between pages
+ asking MainWindow for global state (active user, simulator flags).


MAP TO FOLLOW (Objects & Responsibilities)
------------------------------------------
MainWindow (QMainWindow)
  ├─ Database                   → JSON read/write for users & params
  ├─ QStackedWidget (self.stack)
  │   ├─ LoginPage              → Register/Login. On success calls MainWindow._on_login_ok(name)
  │   ├─ DashboardPage          → Shows status labels + 4 mode buttons; calls MainWindow._open_mode_editor(mode)
  │   └─ ModeEditorPage         → Parameter widgets, Summary, Egram
  │        ├─ Uses Database     → save_params()/load_params() for (user, mode)
  │        └─ D1EgramView       → Live simulated traces; Start/Stop/Save PNG
  ├─ Menus
  │   ├─ File → Reports         → _export_brady_params_report(), _export_temp_params_report()
  │   ├─ File → Export JSON     → _export_all_json()
  │   ├─ Simulator              → _toggle_comms(), _toggle_device_changed(),
  │   │                           _set_device_id(), _set_telemetry("ok|out_of_range|noise")
  │   └─ Utilities              → _show_about(), _set_clock_dialog()
  └─ QStatusBar                 → Shows Comms | Device ID | Change state | Telemetry | Clock

D2 Ref:
https://github.com/god233012yamil/Creating-a-Serial-Port-Communication-GUI-Using-PyQt5/blob/main/pyqt5-serial-communication.py
"""

# =========================
# 1) Standard library imports
# =========================
import json # read/write JSON file (database)
import os # file paths and existence checks
import hashlib # SHA256 to hash passwords (security)
import serial # pySerial for serial communcation
from typing import Dict, Any, List, Optional  # type hints

# =========================
# 2) PyQt5 GUI imports and serial imports
# =========================
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt, QSize, QDateTime, QTimer # QtCore has core types; Qt namespace
from PyQt5.QtGui import QIcon, QValidator, QPainter, QTextDocument # QtGui has visual stuff (icons, validators)
from PyQt5.QtPrintSupport import QPrinter# printsupport for pdfs, 3.2.4
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo # serial communication GUI
from queue import Queue # for queuing serial information packets


from PyQt5.QtGui import QIcon, QValidator 

from PyQt5.QtWidgets import (
    QApplication, # the global GUI application object (event loop here)
    QWidget, # base class for most things shown on screen
    QMainWindow, # main window with menu/status/central widget
    QStackedWidget, # "deck" to switch between multiple pages
    QVBoxLayout, # vertical layout manager
    QHBoxLayout, # horizontal layout manager
    QLabel, # shows text or rich text (HTML)
    QLineEdit, # single-line text input
    QPushButton, # clickable button
    QMessageBox, # modal dialogs (info/warning/errors)
    QFormLayout, # 2-column "Label : Field" layout
    QSpinBox, # integer input with arrows + typing
    QDoubleSpinBox, # floating-point input with arrows + typing
    QComboBox, # drop-down selection list
    QGroupBox, # box with a title around a group of widgets
    QTabWidget, # tabs (parameters / summary / egram)
    QFileDialog, # file picker dialog
    QAction, # menu item (can be clickable, checkable)
    QActionGroup, # group of actions (useful for radio-button behavior)
    QStatusBar,  # one-line bar at bottom for status text
    QDialog, # popup
    QDialogButtonBox, #buttons
    QDateTimeEdit, #change time 3.2.3
    QInputDialog # input popup
)

from serial.serialutil import SerialException

# =========================
# 3) FILE STORAGE!!!!!!!!!!!
# =========================
DB_FILE = "dcm_d1_file.json" # JSON file path to store users + saved parameters
MAX_USERS = 10 # allow at most 10 users locally

class Database:
    """
    We store JSON like:
    {
      "users": [{"username": "alice", "password_hash": "..."}, ...],
      "params": {"alice:VVI": {...}, "alice:AAI": {...}, ...}
    }
    """

    def __init__(self, path: str): #string, path for location of json file, will be in current working directory
        self.path = path # saving incoming string path on the object itself as self.path so it can be used later

        # true if file exists, if not flips it
        if not os.path.exists(self.path):  # if file doesn't exist yet
            self._write({"users": [], "params": {}}) # create with empty structure

    # private read helper _ internal
    def _read(self) -> Dict[str, Any]:
        with open(self.path, "r", encoding="utf-8") as f: # open file for reading
            return json.load(f)  # parse JSON -> Python dict

    # private write helper
    def _write(self, data: Dict[str, Any]) -> None:
        with open(self.path, "w", encoding="utf-8") as f: # open for writing (overwrite)
            json.dump(data, f, indent=2) # dump dict -> json

    # public: count how many users are registered
    def user_count(self) -> int:
        return len(self._read()["users"])

    # public: add a new user (case-insensitive unique usernames; enforce MAX_USERS)
    def add_user(self, username: str, password_hash: str) -> bool:
        data = self._read()
        # check duplicate ignoring case (use .lower() on both sides)
        if any(u["username"].lower() == username.lower() for u in data["users"]):
            return False
        # enforce maximum user count
        if len(data["users"]) >= MAX_USERS:
            return False
        # append new record and save to file
        data["users"].append({"username": username, "password_hash": password_hash})
        self._write(data)
        return True

    # public: verify login (username+hash must match a stored record)
    def verify_user(self, username: str, password_hash: str) -> bool:
        data = self._read()
        user_lc = username.lower()
        for user in data["users"]:
            if user["username"].lower() == user_lc and user["password_hash"] == password_hash:
                return True
        return False

    # public: save parameter dict under key "<user>:<mode>"
    def save_params(self, username: str, mode: str, params: Dict[str, Any]) -> None:
        data = self._read()
        key = f"{username}:{mode}" # composite key simple lookup
        data["params"][key] = params
        self._write(data)

    # public: load parameter dict; if none saved, return empty dict
    def load_params(self, username: str, mode: str) -> Dict[str, Any]:
        data = self._read()
        return data["params"].get(f"{username}:{mode}", {})

# =========================
# 4) Parameters
# =========================

# LRL (Lower Rate Limit) has piecewise steps depending on the range
LRL_SEGMENTS = [
    (30, 50, 5),   # 30, 35, 40, 45, 50
    (50, 90, 1),   # 50..90 (1 step)
    (90, 175, 5),  # 90, 95, ..., 175
]
# URL (Upper Rate Limit): 50–175 step 5
URL_MIN, URL_MAX, URL_STEP = 50, 175, 5

# D2: Maximum Sensor Rate, same as URL
MSR_MIN, MSR_MAX, MSR_STEP = 50, 175, 5

# D2: AV delay
AV_MIN, AV_MAX, AV_STEP = 70, 300, 10

# D2 Amplitude choices: "Off", 0.1–5.0V with 0.1V increments
AMP_ATR_MIN,  AMP_ATR_MAX,  AMP_ATR_STEP  = 0.1, 5.0, 0.1 # ATRIAL range
AMP_VENT_MIN, AMP_VENT_MAX, AMP_VENT_STEP = 0.1, 5.0, 0.1 # VENTRICULAR range


# D2 Pulse width: from 1 to 30ms in 1ms steps
PW_SEGMENTS = [
    (1, 30, 1)
]

# Refractory periods: 150–500 ms in 10 ms steps
REF_MIN, REF_MAX, REF_STEP = 150, 500, 10

# D2: Atrial/ventricular sensitivity: 0-5V in 0.1V steps
SENSE_MIN, SENSE_MAX, SENSE_STEP = 0.0, 5.0, 0.1


# D2 Modes
MODES = ["AOO", "VOO", "AAI", "VVI", "AOOR", "VOOR", "AAIR", "VVIR"]

# Hysteresis:
# - enabled only for inhibiting modes (AAI, VVI)
# - when enabled, the Hysteresis Rate Limit (HRL) uses the same choices as LRL
HYSTERESIS_STATES = ["Off", "On"]

# Rate Smoothing:
# - two programmable parameters Up and Down
# - options Off, 3, 6, 9, 12, 15, 18, 21, 25 %
RATE_SMOOTH_CHOICES = ["Off", "3%", "6%", "9%", "12%", "15%", "18%", "21%", "25%"]

# D2 Additional Parameters

# Response Factor:
# 1-16 in integer steps
RESPONSE_FACTOR_MIN, RESPONSE_FACTOR_MAX, RESPONSE_FACTOR_STEP = 1, 16, 1

# Reaction time:
# 30 to 60ms in 10ms increments
REACTION_TIME_MIN, REACTION_TIME_MAX, REACTION_TIME_STEP = 30, 60, 10

# Activity threshold options:
ACTIVITY_THRESHOLD = ["V-Low", "Low", "Med-Low", "Med", "Med-High", "High", "V-High"]

# Recovery time:
# 2 to 16 minutes in 1 minute increments
RECOVERY_TIME_MIN, RECOVERY_TIME_MAX, RECOVERY_TIME_STEP = 2, 16, 1


# D2: Serial Communication-Related Parameters:
# - baud rate of 57600
# - streaming period of 4 to 5 ms
BAUD_RATE = 57600
STREAM_PERIOD = [4.0, 5.0]


# -------------------------
# About / Utility constants
# -------------------------
APP_MODEL_NUMBER   = "DCM D2"
APP_SOFTWARE_REV   = "D1"
APP_SERIAL_NUMBER  = "SN"
APP_INSTITUTION    = "McMaster University"

# =========================
# 5) helpers
# =========================
def hash_password(plain: str) -> str: # hash password safety
    """
    Turn a plaintext password into a SHA256 hex string.
    - .encode("utf-8") converts Python str to bytes
    - hashlib.sha256(...).hexdigest() returns a hex string like 'a9f...'
    """
    return hashlib.sha256(plain.encode("utf-8")).hexdigest()

def build_allowed_lrl(segments) -> List[int]: # doesnt need float range bc whole numbers
    """
    build allowed integer LRL values by merging each (lo,hi,step) segment
    """
    vals = set() # set prevents duplicates around edges 
    for lo, hi, st in segments:
        vals.update(range(lo, hi + 1, st))  # +1 makes hi inclusive
    return sorted(vals)


def build_allowed_url(min_v: int, max_v: int, step: int) -> List[int]: # doesnt need float range bc whole numbers
    """simple inclusive range for URL values"""
    return list(range(min_v, max_v + 1, step))

def build_allowed_sense(min_v: int, max_v: int, step: int) -> List[float]:
    vals = set()
    x = min_v
    while x < max_v + 1e-12:
        vals.add(round(x, 2))
        x += step
    return sorted(vals)

def build_pw_values() -> List[float]:
    """
    build pulse-width allowed values from the PW_SEGMENTS spec.
    uses set to avoid duplicates; sorted to make stepping nice.
    """
    vals = set()
    for lo, hi, st in PW_SEGMENTS:
        x = lo
        while x <= hi + 1e-12: # include hi
            vals.add(round(x, 2)) # keep two decimals for the 0.05
            x += st
    return sorted(vals)

def _percent_to_int(s: str) -> int:
    """'Off' -> 0, '12%' -> 12"""
    return 0 if s == "Off" else int(s.rstrip("%"))

def _int_to_percent(v: int) -> str:
    """0 -> 'Off', 12 -> '12%'"""
    return "Off" if v == 0 else f"{int(v)}%"

def _hexify(i: int) -> str:
    # convert int to hexadecimal and strip 0x
    return hex(i)[2:]


# all allowed lists to be used by Widgets
ALLOWED_LRL = build_allowed_lrl(LRL_SEGMENTS)
ALLOWED_URL = build_allowed_url(URL_MIN, URL_MAX, URL_STEP)
ALLOWED_SENSE = build_allowed_sense(SENSE_MIN, SENSE_MAX, SENSE_STEP)
PW_VALUES   = build_pw_values()
REF_VALUES  = list(range(REF_MIN, REF_MAX + 1, REF_STEP))


# D2: thread to handle all serial communications

class SerialThread(QThread):

    serial_received = pyqtSignal(bytes)
    serial_error = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port: QSerialPort = QSerialPort()
        self.isRunning: bool = False
        self.isReading: bool = False
        self.writeQueue: Queue = Queue()
        self._rx_lock = False # added for a buffer to prevent data overlap


    def port_setup(self, port_name: str, baud_rate: int, data_bits: int)-> None:
        '''
        parity, stop bits, flow control omitted
        '''
        self.port.setPortName(port_name)
        self.port.setBaudRate(baud_rate)
        self.port.setDataBits(data_bits)

    def run(self)-> None:
        '''
        reads and writes data from serial communication
        '''
        if not self.port.open(QSerialPort.ReadWrite):
            self.serial_error.emit("ERROR: Could not open port.")
            return
        self.is_running = True
        try:
            while self.is_running:
                while not self.writeQueue.empty():
                    #write as long as there is still information in the queue
                    preData = self.writeQueue.get()
                    if isinstance(preData, str):
                        data = bytes.fromhex(preData)
                    elif isinstance(preData, (bytes, bytearray)):
                        data =bytes(preData)
                    else:
                        try:
                            data = bytes(preData)
                        except Exception:
                            self.serial_error.emit("ERROR: Invalid data type in write queue.")
                            continue
                    self.port.write(data)
                    ok = self.port.waitForBytesWritten(int(STREAM_PERIOD[1]*1000)) # wait max 5ms
                    if not ok:
                        self.serial_error.emit("ERROR: Timeout during write.")
                    self.port.flush()

                if self.isReading:
                    if self.port.waitForReadyRead(int(STREAM_PERIOD[1]*1000)): # wait max 5ms
                        qba = self.port.readAll()
                        data_bytes = bytes(qba.data()) if hasattr(qba, 'data') else bytes(qba)
                        if data_bytes:
                            self.serial_received.emit(data_bytes)
                self.msleep(1) # prevent 100% CPU
        finally:
            try:
                self.port.close()
            except Exception:
                pass

    def stop(self)-> None:
        self.is_running = False
        self.quit()
        self.wait()
    
    def writeToQueue(self, data: str)-> None:
        self.isReading = False
        if self.port.isOpen():
            self.writeQueue.put(data.encode())
        else:
            self.serial_error.emit("Failed")

    def beginEgram(self, func_code: int = 0x32)-> None: #defaultt 0x32, func_code is for egram reading
        self.isReading = True
        # Request data to be sent from pacemaker
        sync = 0x16
        payload = [sync, func_code, 0x01]
        chk = sum(payload[2:]) % 256
        frame = bytes(payload + [chk])       
        self.writeToQueue(frame)
        self.isReading = True

    def stop_egram(self, func_code: int = 0x32)-> None:
        # Request data to be sent from pacemaker
        sync = 0x16
        payload = [sync, func_code, 0x00]
        chk = sum(payload[2:]) % 256
        frame = bytes(payload + [chk])       
        self.writeToQueue(frame)
        #self.isReading = False

    def request_params(self)-> None:
        sync = 0x16
        FUNC_READ_PARAMS = 0x10 #adjust to actual, ask what it is
        payload = [sync, FUNC_READ_PARAMS]
        chk = sum(payload[2:]) % 256
        frame = bytes(payload + [chk])
        self.writeToQueue(frame)
        self.isReading = True

    def readFromPacemaker(self)-> None:
        self.isReading = True
        # Request data to be sent from pacemaker
        self.writeToQueue({hex(16)[2:], hex(22)[2:]}) #change to readparams from pacemaker
        #add function for readEgramfrompacemaker - safety critical

        



# =========================
# 6) CUSTOM WIDGETS FOR PARAMETERS!!!!!!
# =========================
class LRLSpinBox(QSpinBox):
    """Integer spinbox for LRL; enforces allowed piecewise values"""

    def __init__(self, parent=None):
        super().__init__(parent) # call base class constructor
        self.allowed = ALLOWED_LRL # creating self.allowed to use ALLOWED_LRL in this class
        self.setRange(min(self.allowed), max(self.allowed)) # min/max bounds keep in list
        self.setSuffix(" ppm") # add " ppm" after number
        self.setValue(60) # default value 60, this is also the nominal value

    def stepBy(self, steps: int) -> None:
        """
        called when user presses the up/down arrows; override it to jump within self.allowed
        so it won't just go +1/-1
        """
        current = self.value()
        try:
            idx = self.allowed.index(current) # find current position
        except ValueError:
            # if user typed in-between value, find the nearest index
            idx = min(range(len(self.allowed)), key=lambda i: abs(self.allowed[i] - current))
        idx = max(0, min(len(self.allowed) - 1, idx + steps))  # clamp to ends of list
        self.setValue(self.allowed[idx]) # set the new value

    def validate(self, text: str, pos: int):
        """
        controls whats accepted when you type in the box
        validator states!
        intermediate = allow typing to continue, so like 6 towards 60
        invalid = stops typing
        acceptable = what we want, in range
        """
        # strip the suffix to check number
        raw = text.replace(" ppm", "").strip()
        if raw == "":
            return (QValidator.Intermediate, text, pos) #space/empty, allow

        # if it's not an integer yet, let the user keep typing
        if not raw.lstrip("-").isdigit():
            return (QValidator.Intermediate, text, pos)

        val = int(raw)

        # while typing up to the minimum (ex 6 on the way to 60), allow it
        if val < self.minimum():
            return (QValidator.Intermediate, text, pos)

        # above maximum is invalid
        if val > self.maximum():
            return (QValidator.Invalid, text, pos)

        # inside bounds: only exact allowed values are acceptable
        if val in self.allowed:
            return (QValidator.Acceptable, text, pos)

        # within numeric bounds but not an allowed discrete value
        return (QValidator.Invalid, text, pos)

class URLSpinBox(QSpinBox):
    """SAME CONCEPT AS LRL!! except using allowed url"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.allowed = ALLOWED_URL # multiples of 5
        self.setRange(min(self.allowed), max(self.allowed))
        self.setSuffix(" ppm")
        self.setValue(120)

    def stepBy(self, steps: int) -> None:
        current = self.value()
        try:
            idx = self.allowed.index(current)
        except ValueError:
            # if user typed in-between value, find the nearest legal index
            idx = min(range(len(self.allowed)), key=lambda i: abs(self.allowed[i] - current)) # clamp to [0, last]
        idx = max(0, min(len(self.allowed) - 1, idx + steps)) # new value
        self.setValue(self.allowed[idx])

    def validate(self, text: str, pos: int):
        # strip the suffix the widget appends
        raw = text.replace(" ppm", "").strip()
        if raw == "":
            return (QValidator.Intermediate, text, pos)

        # if it's not an integer yet, let the user keep typing
        if not raw.lstrip("-").isdigit():
            return (QValidator.Intermediate, text, pos)

        val = int(raw)

        # while typing up to the minimum (ex 6 on the way to 60), allow it
        if val < self.minimum():
            return (QValidator.Intermediate, text, pos)

        # above maximum is invalid 
        if val > self.maximum():
            return (QValidator.Invalid, text, pos)

        # inside bounds: only exact allowed values are acceptable
        if val in self.allowed:
            return (QValidator.Acceptable, text, pos)

        # within numeric bounds but not an allowed discrete value
        return (QValidator.Invalid, text, pos)

class PWSpinBox(QDoubleSpinBox):
    """double spinbox for pulse width, 0.05 then 0.1–   1.9 stepping"""
    # used by ModeEditorPage -> self.a/v_pw
    # values retrieved and saved in _collect_params()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.allowed = PW_VALUES
        self.setRange(min(self.allowed), max(self.allowed))
        self.setDecimals(2) # show two decimals (0.05)
        self.setSingleStep(0.05) 
        self.setSuffix(" ms")
        self.setValue(1) # nominal value

    def stepBy(self, steps: int) -> None: # same pattern
        """jump along our allowed values list rather than raw 0.05 steps"""
        cur = self.value()
        try:
            idx = self.allowed.index(cur)
        except ValueError:
            idx = min(range(len(self.allowed)), key=lambda i: abs(self.allowed[i] - cur))
        idx = max(0, min(len(self.allowed) - 1, idx + steps))
        self.setValue(self.allowed[idx])

    def validate(self, text: str, pos: int): # same pattern
        """accept only exact members of PW_VALUES""" 
        try:
            val = float(text.replace(" ms", "").strip())
        except ValueError:
            return (QValidator.Intermediate, text, pos)
        if any(abs(val - a) < 1e-9 for a in self.allowed):
            return (QValidator.Acceptable, text, pos)
        if self.minimum() <= val <= self.maximum():
            return (QValidator.Invalid, text, pos)
        return (QValidator.Invalid, text, pos)

class RefPeriodSpinBox(QSpinBox):
    """ppinbox for ARP/VRP"""
    # no floats no validate . saved and loaded like other stuff
    # used by ModeEditorPage -> self.a/vrp
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(REF_MIN, REF_MAX)
        self.setSingleStep(REF_STEP)
        self.setSuffix(" ms")
        self.setValue(250)

class SensitivitySpinBox(QDoubleSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.allowed = ALLOWED_SENSE
        self.setRange(min(self.allowed), max(self.allowed))
        self.setDecimals(1) # show 1 decimal
        self.setSingleStep(SENSE_STEP) 
        self.setSuffix(" mV")
        self.setValue(2.5) # nominal value

    def stepBy(self, steps: int) -> None: # same pattern
        """jump along allowed values list rather than raw steps"""
        cur = self.value()
        try:
            idx = self.allowed.index(cur)
        except ValueError:
            # Find closest value
            idx = min(range(len(self.allowed)), key=lambda i: abs(self.allowed[i] - cur))
        idx = max(0, min(len(self.allowed) - 1, idx + steps))
        self.setValue(self.allowed[idx])

    def validate(self, text: str, pos: int): # same pattern
        """accept only exact members of ALLOWED_SENSE""" 
        try:
            val = float(text.replace(" mV", "").strip())
        except ValueError:
            return (QValidator.Intermediate, text, pos)
        if any(abs(val - a) < 1e-9 for a in self.allowed):
            return (QValidator.Acceptable, text, pos)
        if self.minimum() <= val <= self.maximum():
            return (QValidator.Invalid, text, pos)
        return (QValidator.Invalid, text, pos)

class ReactionTimeSpinBox(QSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(REACTION_TIME_MIN, REACTION_TIME_MAX)
        self.setSingleStep(REACTION_TIME_STEP)
        self.setSuffix(" sec")
        self.setValue(10)

class ResponseFactorSpinBox(QSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(RESPONSE_FACTOR_MIN, RESPONSE_FACTOR_MAX)
        self.setSingleStep(RESPONSE_FACTOR_STEP)
        self.setValue(8)

class RecoveryTimeSpinBox(QSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(RECOVERY_TIME_MIN, RECOVERY_TIME_MAX)
        self.setSingleStep(RECOVERY_TIME_STEP)
        self.setSuffix(" min")
        self.setValue(5)

class AmplitudeWidget(QWidget):
    """
    widget w/ [Label] [Off/On dropdown] [Voltage spinbox]
    - If "Off" selected -> .value() returns "Off" (string)
    - If "On" selected  -> .value() returns a float snapped to valid step range

    A and V different ranges
    """
    # ModeEditorPage creates self.a_amp, self.v_amp = AmplitudeWidget("")
    # _collect_params() calls .value() on each
    # _apply_params_to_widgets() calls .setValue(...) on each

    def __init__(self, label_text: str,
                 min_v: float, max_v: float, step_v: float, default_v: float, # range and number shown when on
                 parent=None): # defining values
        super().__init__(parent)

        # remember range so snapping knows what to do
        self._min_v = float(min_v)
        self._max_v = float(max_v)
        self._step_v = float(step_v)

        # build a horizontal row: label | combo | spin
        row = QHBoxLayout(self) # 'self' is the container widget
        row.setContentsMargins(0, 0, 0, 0) # no extra padding inside

        self.label = QLabel(label_text) # ex "Atrial Amplitude:" 
        self.state_combobox = QComboBox() # dropdown with "Off"/"On"
        self.state_combobox.addItems(["Off", "On"]) # add the two choices

        self.volt_spinbox = QDoubleSpinBox() # voltage input
        self.volt_spinbox.setDecimals(1) # show 1 decimal place
        self.volt_spinbox.setRange(self._min_v, self._max_v)  # chamber-specific range
        self.volt_spinbox.setSingleStep(self._step_v) # chamber-specific step
        self.volt_spinbox.setSuffix(" V") # add suffix
        self.volt_spinbox.setValue(default_v) # starting value

        # react when combo changes (disable/enable the spinbox)
        self.state_combobox.currentIndexChanged.connect(self._update_enabled_state) # ref to _update_enabled_state
        self._update_enabled_state() # set initial enabled state

        # add all three widgets into the row
        row.addWidget(self.label)
        row.addWidget(self.state_combobox)
        row.addWidget(self.volt_spinbox)

    def _update_enabled_state(self) -> None:
        """enable the number field only if 'On' is selected"""
        self.volt_spinbox.setEnabled(self.state_combobox.currentText() == "On")

    def value(self):
        """return either 'Off' (str) or a float rounded to grid"""
        if self.state_combobox.currentText() == "Off":
            return "Off"

        v = float(self.volt_spinbox.value()) # whats in the box rn, can be anything, need to snap/modify
        # snap to the nearest limit defined by (min, step)
        steps = round((v - self._min_v) / self._step_v)
        snapped = round(self._min_v + steps * self._step_v, 1)
        # clamp 
        snapped = max(self._min_v, min(self._max_v, snapped))
        return snapped

    ## ^^ basically whats happening is taking the current value - minimum and dividing by the step
    ## ex. current (2.87 - min 0.5)/0.1 step = 23.7 round to 24 steps
    ## then add to min and multiply by step to get actual value of 2.9

    def setValue(self, val) -> None:
        """set widget value from saved data: 'Off' or number volt"""
        if val == "Off":
            self.state_combobox.setCurrentText("Off")
        else:
            self.state_combobox.setCurrentText("On")
            self.volt_spinbox.setValue(float(val))

# =========================
# 7) screens/pages inside the stacked widget
# =========================
class LoginPage(QWidget):
    """
    registration + login page.
    - receive 'db' (Database) and 'on_login_ok' (callback) from MainWindow*
    - when login succeeds, we call on_login_ok(username) to tell MainWindow
    """

    def __init__(self, db: Database, on_login_ok, parent=None): # depends on db
        super().__init__(parent) # build the QWidget base
        self.db = db # remember the database so we can query it
        self.on_login_ok = on_login_ok  # remember callback to MainWindow *

        page = QVBoxLayout(self) # vertical page layout
        page.addWidget(QLabel("<h2>DCM — Welcome!</h2>"))  # simple title

        # registration group (top)
        reg_group = QGroupBox("New user registration")
        reg_form = QFormLayout(reg_group) # label : field pairs inside the group

        self.reg_name = QLineEdit() # name input
        self.reg_pass = QLineEdit() # password input
        self.reg_pass.setEchoMode(QLineEdit.Password) # hide characters

        RegisterButton = QPushButton("Register")# button the user clicks to register
        RegisterButton.clicked.connect(self._handle_register)  # connect signal -> slot (method below)

        reg_form.addRow("Name:", self.reg_name)
        reg_form.addRow("Password:", self.reg_pass)
        reg_form.addRow(RegisterButton)

        # login group (bottom) 
        login_group = QGroupBox("Login")
        login_form = QFormLayout(login_group)

        self.login_name = QLineEdit()
        self.login_pass = QLineEdit()
        self.login_pass.setEchoMode(QLineEdit.Password)

        LoginButton = QPushButton("Login")
        LoginButton.clicked.connect(self._handle_login)

        login_form.addRow("Name:", self.login_name)
        login_form.addRow("Password:", self.login_pass)
        login_form.addRow(LoginButton)

        # add both groups to the page layout
        page.addWidget(reg_group)
        page.addWidget(login_group)
        page.addStretch(1) # spacing

    # slots (handlers) for the two buttons 
    def _handle_register(self) -> None:
        """validate inputs, enforce limits, then add user to db"""
        name = self.reg_name.text().strip()  # .text() gets the text; .strip() trims spaces
        pw   = self.reg_pass.text()
        if not name or not pw:
            QMessageBox.warning(self, "Missing info", "Please enter a name and password!")
            return
        if self.db.user_count() >= MAX_USERS:
            QMessageBox.critical(self, "Max user capacity reached", f"Maximum of {MAX_USERS} users stored.")
            return
        if not self.db.add_user(name, hash_password(pw)):  # returns False if duplicate or full
            QMessageBox.warning(self, "Registration failed!", "User exists or capacity reached.")
            return
        QMessageBox.information(self, "Success!", "Registration complete. Please log in.")
        self.reg_pass.clear()   # clear password field for safety

    def _handle_login(self) -> None:
        """check user and pass against db; if success, notify MainWindow."""
        name = self.login_name.text().strip()
        pw   = self.login_pass.text()
        if self.db.verify_user(name, hash_password(pw)):
            self.on_login_ok(name)# <- calls the callback passed from MainWindow
        else:
            QMessageBox.critical(self, "Login failed", "Invalid username or password.")


class DashboardPage(QWidget):
    """
    dashboard shows current device/comms state and mode buttons
    receive 'on_mode_click(mode)' callback to inform MainWindow which mode to open.
    """

    def __init__(self, on_mode_click, parent=None): # we call on_mode_click later in MainWindow._open_mode_editor
        super().__init__(parent)
        self.on_mode_click = on_mode_click  # remember the callback

        page = QVBoxLayout(self) # vertical page layout
        page.addWidget(QLabel("<h2>Device Controller–Monitor</h2>"))

        # row of status labels
        status = QHBoxLayout()
        self.label_comms = QLabel("Comms: <b>Not Connected</b>")
        self.label_device = QLabel("Device: <i>None</i>")
        self.label_changed = QLabel("Status: <b>Last Device OK</b>")
        self.label_telemetry = QLabel("Telemetry: <b>OK</b>")
        for w in (self.label_comms, self.label_device, self.label_changed, self.label_telemetry):
            status.addWidget(w)
            status.addSpacing(20) # small space between labels
        status.addStretch(1)
        page.addLayout(status)

        # row of modes (AOO/VOO/AAI/VVI/AOOR/VOOR/AAIR/VVIR)
        row = QHBoxLayout()
        for mode in MODES:
            ModeButton = QPushButton(mode) # text/label is the mode
            ModeButton.setMinimumWidth(120) # make them wide enough
            # lambda captures 'mode' into 'm' so each button calls with its own value
            ModeButton.clicked.connect(lambda _, m=mode: self.on_mode_click(m))
            row.addWidget(ModeButton)
        row.addStretch(1)
        page.addLayout(row)
        page.addStretch(1)

    # public methods called by MainWindow to update the labels
    def show_comms(self, connected: bool) -> None:
        self.label_comms.setText(f"Comms: <b>{'Connected' if connected else 'Not Connected'}</b>")

    def show_device(self, device_id: str) -> None:
        self.label_device.setText(f"Device: <b>{device_id or 'None'}</b>")

    def show_changed(self, changed: bool) -> None:
        self.label_changed.setText("Status: <b>Device Changed</b>" if changed else "Status: <b>Last Device OK</b>")

    def show_telemetry(self, state: str) -> None:
        mapping = {
            "ok": "Telemetry: <b>OK</b>",
            "out_of_range": "Telemetry: <b>Lost – Out of Range</b>",
            "noise": "Telemetry: <b>Lost – Noise</b>",
        }
        self.label_telemetry.setText(mapping.get(state, "Telemetry: <b>OK</b>"))

class D2EgramView(QWidget):
    """
    Updated D2EgramView: no longer opens its own QSerialPort.
    Instead attach a SerialThread instance with attach_serial_thread(thread).
    Parses framed binary egram packets and appends samples.
    """

    FUNC_EGRAM = 0x32  # device-specific — replace if different

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)

        # --- local buffers ---
        self.buffer_len = 400
        self.atrialList = []
        self.ventricularList = []
        self.surfaceList = []

        # --- show/hide ---
        self.show_atrial = True
        self.show_ventricular = True
        self.show_surface = True

        # --- no local QSerialPort anymore ---
        self.serial_thread = None

        # --- QTimer for GUI update (still used to force repaint) ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)

        # receive buffer for framing (bytes)
        self.rx_buffer = bytearray()

        # rendering timer: don't auto-start — ModeEditorPage controls start/stop
        self._running = False

    # allow MainWindow / ModeEditorPage to attach the running SerialThread
    def attach_serial_thread(self, thread: SerialThread) -> None:
        """
        Connect to a SerialThread to receive bytes.
        """
        if self.serial_thread is not None:
            try:
                self.serial_thread.serial_received.disconnect(self._on_serial_bytes)
            except Exception:
                pass

        self.serial_thread = thread
        if thread is not None:
            thread.serial_received.connect(self._on_serial_bytes)
            # don't change thread.isReading here — ModeEditorPage or user commands control it

    # CONTROL
    def start(self):
        """
        Start GUI update timer and send begin_egram request via serial_thread.
        """
        if self._running:
            return
        if not self.serial_thread:
            print("D2EgramView.start: No SerialThread attached.")
            return

        # request the device begin streaming (thread will enable reading)
        try:
            self.serial_thread.begin_egram(self.FUNC_EGRAM)
        except Exception as e:
            print("Failed to request egram start:", e)

        self.timer.start(20)  # 50 Hz refresh
        self._running = True

    def stop(self):
        """
        Stop timer and request stop from device.
        """
        if not self._running:
            return
        if self.serial_thread:
            try:
                self.serial_thread.stop_egram(self.FUNC_EGRAM)
            except Exception as e:
                print("Failed to request egram stop:", e)

        if self.timer.isActive():
            self.timer.stop()
        self._running = False

    # called when SerialThread emits raw bytes
    def _on_serial_bytes(self, data: bytes) -> None:
        """
        Append bytes to rx_buffer and extract frames.
        Frame (example): [0x16][FUNC_EGRAM=0x32][A_hi][A_lo][V_hi][V_lo][CHK]
        A/V are signed 16-bit integers representing mV * 100.
        """
        if not data:
            return
        self.rx_buffer.extend(data)

        # parse frames repeatedly
        while True:
            # need at least 7 bytes for a frame in this schema
            if len(self.rx_buffer) < 7:
                break

            # find sync byte (0x16)
            try:
                sync_idx = self.rx_buffer.index(0x16)
            except ValueError:
                # no sync -> drop buffer
                self.rx_buffer.clear()
                break

            if sync_idx > 0:
                # drop leading bytes up to sync
                del self.rx_buffer[:sync_idx]

            # now check we have at least full frame
            if len(self.rx_buffer) < 7:
                break

            # peek frame
            sync = self.rx_buffer[0]
            func = self.rx_buffer[1]
            if func != self.FUNC_EGRAM:
                # not egram frame — drop sync byte and continue searching
                del self.rx_buffer[0]
                continue

            # extract bytes (big-endian)
            a_hi = self.rx_buffer[2]
            a_lo = self.rx_buffer[3]
            v_hi = self.rx_buffer[4]
            v_lo = self.rx_buffer[5]
            chk = self.rx_buffer[6]

            # compute checksum (device-specific; here we sum func..lastdata)
            calc = (func + a_hi + a_lo + v_hi + v_lo) % 256
            if calc != chk:
                # checksum mismatch — drop sync and try again
                del self.rx_buffer[0]
                continue

            # parse signed 16-bit values (big-endian)
            atr_raw = (a_hi << 8) | a_lo
            ven_raw = (v_hi << 8) | v_lo
            # interpret as signed:
            if atr_raw & 0x8000:
                atr_raw = atr_raw - 0x10000
            if ven_raw & 0x8000:
                ven_raw = ven_raw - 0x10000

            # convert to mV (example scaling: device sends mV*100)
            atr_mv = atr_raw / 100.0
            ven_mv = ven_raw / 100.0

            # append samples
            self._append_samples(atr_mv, ven_mv)

            # remove parsed frame
            del self.rx_buffer[:7]

    # DATA HANDLING (same as before but accepts mV floats now)
    def _append_samples(self, atrial_mv: float, ventricular_mv: float):
        # scale values down to a drawable −1…1 range (keep same scale as previously used)
        a = atrial_mv * 0.25
        v = ventricular_mv * 0.25
        surf = (a + v) / 2.0

        self.atrialList.append(a)
        self.ventricularList.append(v)
        self.surfaceList.append(surf)

        # trim buffers
        if len(self.atrialList) > self.buffer_len:
            self.atrialList.pop(0)
            self.ventricularList.pop(0)
            self.surfaceList.pop(0)

    # RENDER (unchanged)
    def paintEvent(self, ev):
        from PyQt5.QtGui import QPainter, QPen, QColor
        from PyQt5.QtCore import QPointF, Qt

        p = QPainter(self)
        try:
            p.fillRect(self.rect(), self.palette().base())

            w = self.width()
            h = self.height()
            row_h = h / 3.0

            bases = [row_h * 0.5, row_h * 1.5, row_h * 2.5]
            names = ["Atrial", "Ventricular", "Surface ECG"]
            colors = [QColor("red"), QColor("blue"), QColor("green")]
            series = [self.atrialList, self.ventricularList, self.surfaceList]
            visible = [self.show_atrial, self.show_ventricular, self.show_surface]

            for i in range(3):
                base = bases[i]

                # baseline
                p.setPen(QPen(Qt.gray))
                p.drawLine(0, int(base), w, int(base))
                p.drawText(5, int(base - 5), names[i])

                if not visible[i] or len(series[i]) < 2:
                    continue

                s = series[i]
                step = w / max(1, len(s) - 1)
                scale = row_h * 0.35

                pen = QPen(colors[i])
                pen.setWidth(2)
                p.setPen(pen)

                last = QPointF(0, base - scale * s[0])
                for j in range(1, len(s)):
                    pt = QPointF(j * step, base - scale * s[j])
                    p.drawLine(last, pt)
                    last = pt
        finally:
            p.end()

#qthread and signals

# ABOUT !!!
class AboutDialog(QDialog): #QDialog popup windows, vs QWidget for normal page
    """
    'About' panel listing model, software rev, serial, institution. 3.2.3
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("About DCM")
        lay = QFormLayout(self)
        lay.addRow("Application model number:", QLabel(APP_MODEL_NUMBER))
        lay.addRow("Software revision:",       QLabel(APP_SOFTWARE_REV))
        lay.addRow("DCM serial number:",       QLabel(APP_SERIAL_NUMBER))
        lay.addRow("Institution name:",        QLabel(APP_INSTITUTION))
        buttons = QDialogButtonBox(QDialogButtonBox.Ok, parent=self)# ok button
        buttons.accepted.connect(self.accept)
        lay.addRow(buttons)


class SetClockDialog(QDialog): # popup
    """
    "The Set Clock function shall set the date and time of the device" 3.2.3
    """
    def __init__(self, current_DeviceTime: QDateTime, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Set Clock")
        lay = QFormLayout(self)

        self.dt_edit = QDateTimeEdit(current_DeviceTime) # picker widget internalized w current date/time
        self.dt_edit.setDisplayFormat("yyyy-MM-dd HH:mm:ss") # how it appears to user
        self.dt_edit.setCalendarPopup(True)

        lay.addRow("Device date/time:", self.dt_edit)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self) # ok and cancel, Q handles layout
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay.addRow(buttons)

    def selected_datetime(self) -> QDateTime:
        return self.dt_edit.dateTime()


class ModeEditorPage(QWidget):
    """
    mode editor:
    - parameters tab: edit LRL, URL, A/V amplitude + width, ARP/VRP
    - summary tab: read-only summary of current values
    - egram (D2): placeholder text for next deliverable

    receives:
      db: Database     (for load/save)
      get_active_user: callable returning the username (or none)
    """

    def __init__(self, db: Database, get_active_user, parent=None, serial_thread=None):
        super().__init__(parent)
        self.serial_thread: Optional[SerialThread] = None
        self.db = db
        self.get_active_user = get_active_user
        #self.serial_thread = serial_thread
        self.current_mode: str = MODES[0] # default mode until changed

        # outer layout abd tavs
        page = QVBoxLayout(self)
        self.tabs = QTabWidget() # creates tabs along the top
        page.addWidget(self.tabs)

        # parameters tab
        self.tab_params = QWidget()
        self.tabs.addTab(self.tab_params, "Parameters")
        form = QFormLayout(self.tab_params) # two-column "Label : Widget"

        # choose mode at the top of the form
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(MODES)
        self.mode_combo.currentTextChanged.connect(self.set_mode)  # when user changes, call set_mode()
        form.addRow("Pacing Mode:", self.mode_combo)

        # widgets for each parameter
        self.lrl   = LRLSpinBox()
        self.url   = URLSpinBox()
        self.msr   = URLSpinBox()

        # atrial: 0.5–3.2 V, step 0.1  (default 5 V)
        self.atrial_amp = AmplitudeWidget("Atrial Amplitude:",
                                     AMP_ATR_MIN, AMP_ATR_MAX, AMP_ATR_STEP, 5.0)

        # ventricular: 0.1 to 5 V, step 0.5 (default 5 V)
        self.ventricular_amp = AmplitudeWidget("Ventricular Amplitude:",
                                     AMP_VENT_MIN, AMP_VENT_MAX, AMP_VENT_STEP, 5.0)

        self.atrial_pw  = PWSpinBox()
        self.ventricular_pw  = PWSpinBox()
        self.atrial_sens = SensitivitySpinBox()
        self.ventricular_sens = SensitivitySpinBox()
        self.arp   = RefPeriodSpinBox()
        self.vrp   = RefPeriodSpinBox()

        # add them to the form with labels where appropriate
        form.addRow("Lower Rate Limit (LRL):", self.lrl)
        form.addRow("Upper Rate Limit (URL):", self.url)
        form.addRow("Maximum Sensor Rate (MSR):", self.msr)
        form.addRow(self.atrial_amp)   # AmplitudeWidget includes its own label
        form.addRow("Atrial Pulse Width:", self.atrial_pw)
        form.addRow(self.ventricular_amp)
        form.addRow("Ventricular Pulse Width:", self.ventricular_pw)
        form.addRow("Atrial Sensitivity:", self.atrial_sens)
        form.addRow("Ventricular Sensitivity:", self.ventricular_sens)
        form.addRow("Atrial Refractory Period (ARP):", self.arp)
        form.addRow("Ventricular Refractory Period (VRP):", self.vrp)

        # hysteresis AAI/VVI only
        self.hysteresis_state = QComboBox()
        self.hysteresis_state.addItems(HYSTERESIS_STATES) # "Off" | "On"
        self.HysRateLimit = LRLSpinBox() # use same choices as LRL
        self.HysRateLimit.setEnabled(False) # only enabled when "On"

        # when user flips Off/On, enable/disable HRL field
        self.hysteresis_state.currentTextChanged.connect(
            lambda s: self.HysRateLimit.setEnabled(s == "On")
        )

        form.addRow("Hysteresis:", self.hysteresis_state)
        form.addRow("Hysteresis Rate Limit (HRL):", self.HysRateLimit)

        # rate Smoothing AAI/VVI only
        self.smooth_up = QComboBox()
        self.smooth_up.addItems(RATE_SMOOTH_CHOICES)
        self.smooth_down = QComboBox()
        self.smooth_down.addItems(RATE_SMOOTH_CHOICES)
        form.addRow("Rate Smoothing Up:", self.smooth_up)
        form.addRow("Rate Smoothing Down:", self.smooth_down)

        # D2 (XXXR only)
        self.response_factor = ResponseFactorSpinBox()
        self.reaction_time = ReactionTimeSpinBox()

        self.activity_threshold = QComboBox()
        self.activity_threshold.addItems(ACTIVITY_THRESHOLD)
        self.activity_threshold.currentTextChanged.connect(self.set_threshold)

        self.recovery_time = RecoveryTimeSpinBox()
        form.addRow("Activity Threshold:", self.activity_threshold)
        form.addRow("Reaction Time:", self.reaction_time)
        form.addRow("Response Factor:", self.response_factor)
        form.addRow("Recovery Time:", self.recovery_time)

        # buttons: save / revert
        buttons = QHBoxLayout()
        self.SaveButton = QPushButton("Save Parameters")
        self.RevertButton = QPushButton("Revert to Saved")
        self.LoadButton = QPushButton("Load Parameters to Pacemaker")
        self.GetButton = QPushButton("Get Pacemaker Parameters")
        self.SaveButton.clicked.connect(self._handle_save) # connect click -> save
        self.RevertButton.clicked.connect(self._handle_revert) # connect click -> revert
        self.LoadButton.clicked.connect(self._handle_load)
        self.GetButton.clicked.connect(self._handle_get)
        buttons.addWidget(self.SaveButton)
        buttons.addWidget(self.RevertButton)
        buttons.addWidget(self.LoadButton)
        buttons.addWidget(self.GetButton)
        buttons.addStretch(1)
        form.addRow(buttons)

        # SUMMARY
        self.tab_summary = QWidget()
        self.tabs.addTab(self.tab_summary, "Summary")
        summary_layout = QVBoxLayout(self.tab_summary)
        self.label_summary = QLabel("")  # will show HTML text with values
        summary_layout.addWidget(self.label_summary)
        summary_layout.addStretch(1)

        # EGRAM D2 TAB
        self.tab_egram = QWidget()
        self.tabs.addTab(self.tab_egram, "Egram (D2)")
        egram_layout = QVBoxLayout(self.tab_egram)

        # the viewer
        self.egram_view = D2EgramView()
        egram_layout.addWidget(self.egram_view)

        # controls
        EgramButtons = QHBoxLayout()
        self.StartEgramButton = QPushButton("Start")
        self.StopEgramButton  = QPushButton("Stop")
        self.SaveEgramButton  = QPushButton("Save Strip (PNG)")
        # Instead of directly calling self.egram_view.start/stop, call serial-thread-backed operations:
       # self.StartEgramButton.clicked.disconnect() if hasattr(self.StartEgramButton.clicked, 'disconnect') else None
       # self.StopEgramButton.clicked.disconnect()  if hasattr(self.StopEgramButton.clicked, 'disconnect') else None

        def _on_start_clicked():
            if self.serial_thread:
            # this will request pacemaker to start and egram_view.start will start timer
                self.egram_view.start()
            else:
                QMessageBox.warning(self, "No serial thread", "Serial connection not initialized.")

        def _on_stop_clicked():
            if self.serial_thread:
                self.egram_view.stop()
            else:
                QMessageBox.warning(self, "No serial thread", "Serial connection not initialized.")

        self.StartEgramButton.clicked.connect(_on_start_clicked)
        self.StopEgramButton.clicked.connect(_on_stop_clicked)

        self.SaveEgramButton.clicked.connect(self._save_egram_png)
        EgramButtons.addWidget(self.StartEgramButton)
        EgramButtons.addWidget(self.StopEgramButton)
        EgramButtons.addWidget(self.SaveEgramButton)
        EgramButtons.addStretch(1)
        egram_layout.addLayout(EgramButtons)

        # initialize the page to the default mode (enable/disable fields + load saved if any)
        self.set_mode(self.current_mode)

        # after: self.tabs.addTab(self.tab_egram, "Egram (D2)")
        self.tabs.currentChanged.connect(self._on_tab_changed)
    def attach_serial_thread(self, thread: SerialThread) -> None:
        self.serial_thread = thread
        if self.serial_thread:
            # ensure egram view listens to the thread
            self.egram_view.attach_serial_thread(self.serial_thread)

    def _on_tab_changed(self, idx: int) -> None:
        # auto start when the Egram tab is visible; stop when leaving
        if self.tabs.widget(idx) is self.tab_egram:
            self.egram_view.start()
        else:
            self.egram_view.stop()


    # public method: MainWindow calls this before showing the page
    def set_mode(self, mode: str) -> None: # remember newly selected mode
        """switch editor to a given mode; enable correct chamber fields and load saved data"""
        self.current_mode = mode
        # keep the combo box synchronized without re-triggering this method recursively, dont trigger currentTextChanged
        if self.mode_combo.currentText() != mode:
            old = self.mode_combo.blockSignals(True)  # temporarily silence signals
            self.mode_combo.setCurrentText(mode) # set the visible value
            self.mode_combo.blockSignals(old) # restore signal behavior

        # enable only the chamber widgets that make sense for this mode, booleans true
        
        atrial_on = mode in ("AOO", "AAI", "AAIR", "VVIR")
        ventricular_on = mode in ("VOO", "VVI", "VOOR", "VVIR")
        inhibiting  = mode in ("AAI", "VVI", "AAIR", "VVIR")   # hysteresis + smoothing apply here
        rate_adaptive = mode in ("AOOR", "VOOR", "AAIR", "VVIR")
        
        #enabling amp depending on the modes
        self.atrial_amp.setEnabled(atrial_on); self.atrial_pw.setEnabled(atrial_on)
        self.arp.setEnabled(atrial_on and inhibiting); self.atrial_sens.setEnabled(atrial_on and inhibiting)

        self.ventricular_amp.setEnabled(ventricular_on); self.ventricular_pw.setEnabled(ventricular_on)
        self.vrp.setEnabled(ventricular_on and inhibiting); self.ventricular_sens.setEnabled(ventricular_on and inhibiting)

        # hysteresis + smoothing
        self.hysteresis_state.setEnabled(inhibiting) # on/enabled when inhibiting
        self.HysRateLimit.setEnabled(inhibiting and self.hysteresis_state.currentText() == "On")
        self.smooth_up.setEnabled(inhibiting)
        self.smooth_down.setEnabled(inhibiting)

        # rate adaptive
        self.msr.setEnabled(rate_adaptive)
        self.activity_threshold.setEnabled(rate_adaptive)
        self.reaction_time.setEnabled(rate_adaptive)
        self.response_factor.setEnabled(rate_adaptive)
        self.recovery_time.setEnabled(rate_adaptive)

        # load any saved params for the active user+mode; if none, keep current values
        self._handle_revert()
    
    # D2: Set activity threshold
    def set_threshold(self, activity_threshold: str):
        self.threshold = activity_threshold

    # pulls current values from widgets into a plain dict ready for saving/export. at modes
    def _collect_params(self) -> Dict[str, Any]:
        return {
            "mode": self.current_mode,
            "LRL_ppm": self.lrl.value(),
            "URL_ppm": self.url.value(),
            "MSR_ppm": self.msr.value(),
            "AtrialAmplitude_V": self.atrial_amp.value(),
            "AtrialPulseWidth_ms": round(self.atrial_pw.value(), 2),
            "VentricularAmplitude_V": self.ventricular_amp.value(),
            "VentricularPulseWidth_ms": round(self.ventricular_pw.value(), 2),
            "AtrialSensitivity_mV": round(self.atrial_sens.value(), 2),
            "VentricularSensitivity_mV": round(self.ventricular_sens.value(), 2),
            "ARP_ms": self.arp.value(),
            "VRP_ms": self.vrp.value(),
            
            "Hysteresis": self.hysteresis_state.currentText(), # "Off"/"On"
            "HRL_ppm": self.HysRateLimit.value(), # used when Hysteresis == "On"
            "RateSmoothingUp_percent": _percent_to_int(self.smooth_up.currentText()),
            "RateSmoothingDown_percent": _percent_to_int(self.smooth_down.currentText()),
            "ActivityThreshold": self.activity_threshold.currentText(),
            "ReactionTime_s": self.reaction_time.value(),
            "ResponseFactor": self.response_factor.value(),
            "RecoveryTime_min": self.recovery_time.value(),

        }
    
    # D2: prepares parameters to be written to the serial connection - "WRITE" and packaging FUNCTION
    def _package_params(self) -> str:
        p = self._collect_params()
        header = [_hexify(22), _hexify(85)] #sync and pparams function codes
        params = [
            _hexify(MODES.index(p['mode'])+1),
            _hexify(int(p['Hysteresis'] == 'On')),
            _hexify(p['HRL_ppm']),
            _hexify(p['LRL_ppm']),
            _hexify(p['URL_ppm'])
        ]
        if self.current_mode in ("AOO", "AAI", "AOOR", "AAIR"):
            params.extend([
                _hexify(30) if p['AtrialAmplitude_V'] == "Off" else _hexify(int(p['AtrialAmplitude_V']*10)),
                _hexify(int(p["AtrialPulseWidth_ms"]*10)),
                _hexify(int(p['ARP_ms']/10))
                ])
        else:
            params.extend([
                _hexify(35) if p['VentricularAmplitude_V'] == "Off" else _hexify(int(p['VentricularAmplitude_V']*10)),
                _hexify(int(p["VentricularPulseWidth_ms"]*10)),
                _hexify(int(p['VRP_ms']/10))
                ])
        # Hysteresis values
        if self.current_mode in ("AAI", "AAIR", "VVI", "VVIR"):
            params.append(_hexify(int(p['AtrialSensitivity_mV']*10)) if self.current_mode in ("AAI", "AAIR") else _hexify(int(p["VentricularSensitivity_mV"]*10)))
            params.extend([_hexify(p['RateSmoothingUp_percent']), _hexify(p['RateSmoothingDown_percent'])])
        else:
            # If not hysteresis, append 0s
            params.append(_hexify(0))
            params.extend([_hexify(0), _hexify(0)])
        # Rate adaptive values
        if self.current_mode in ("AOOR", "AAIR", "VOOR", "VVIR"):
            params.extend([_hexify(ACTIVITY_THRESHOLD.index(p["ActivityThreshold"])+1), _hexify(p["ReactionTime_s"])])
            params.extend([_hexify(p["ResponseFactor"]), _hexify(p["RecoveryTime_min"])])
        # If not rate adaptive, append 0s
        else:
            params.extend([_hexify(0), _hexify(0), _hexify(0), _hexify(0)])
        # Check sum (sum of all hex values)

        # MSR
        params.append(_hexify(p['MSR_ppm']) if self.current_mode in ("AOOR", "AAIR", "VOOR", "VVIR") else _hexify(0))

        chksum = 0
        for i in params:
            chksum += int(i, 16)
        # Join to end of array
        chksum = chksum%256 # make sure it fits in 1 byte
        params.append(_hexify(chksum))

        data = " ".join((header+params))
        return data
    
    #D2: process recieved serial data into parameters
    def _process_to_params(self, recieved: str) -> Dict[str, Any]: #"read" annd processing function
        # Check sum first
        splitParams = recieved.split(" ")
        # Check sync byte and recieved code (in hex)
        if int(splitParams[0]) == 16 and int(splitParams[1]) == 49:
            # Check sum
            chksum = 0
            for i in splitParams[2:-1]:
                chksum += int(i, 16)
            chksum = chksum%256
            if chksum == int(splitParams[-1], 16):
                mode_ind = int(splitParams[2], 16)-1
                # create a dict of params
                return {
                "mode": MODES[mode_ind],
                "Hysteresis": "On" if int(splitParams[3], 16) else "Off",
                "HRL_ppm": self.HysRateLimit.value() if int(splitParams[4], 16) == 0 else int(splitParams[4], 16), # do not change if 0 written in
                
                "LRL_ppm": self.lrl.value() if int(splitParams[5], 16) == 0 else int(splitParams[5], 16),
                "URL_ppm": self.url.value() if int(splitParams[6], 16) == 0 else int(splitParams[6], 16),
                
                "AtrialAmplitude_V": self.atrial_amp.value() if mode_ind%2 == 1 else round(int(splitParams[7], 16)/10, 2),
                "AtrialPulseWidth_ms": round(self.atrial_pw.value(), 2) if mode_ind%2 == 1 else round(int(splitParams[8], 16)/10, 2),
                "VentricularAmplitude_V": self.ventricular_amp.value() if mode_ind%2 == 0 else round(int(splitParams[7], 16)/10, 2),
                "VentricularPulseWidth_ms": round(self.ventricular_pw.value(), 2) if mode_ind%2 == 0 else round(int(splitParams[8], 16)/10, 2),
                "AtrialSensitivity_mV": round(self.atrial_sens.value(), 2) if mode_ind%2 == 1 else round(int(splitParams[9], 16)/10, 2),
                "VentricularSensitivity_mV": round(self.ventricular_sens.value(), 2) if mode_ind%2 == 0 else round(int(splitParams[9], 16)/10, 2),
                "ARP_ms": self.arp.value() if mode_ind%2 == 1 else round(int(splitParams[10], 16)*10, 2),
                "VRP_ms": self.vrp.value() if mode_ind%2 == 0 else round(int(splitParams[10], 16)*10, 2),
            
                "RateSmoothingUp_percent": _percent_to_int(self.smooth_up.currentText()) if int(splitParams[11], 16) == 0 else int(splitParams[11], 16),
                "RateSmoothingDown_percent": _percent_to_int(self.smooth_down.currentText()) if int(splitParams[12], 16) == 0 else int(splitParams[12], 16),
                "ActivityThreshold": self.activity_threshold.currentText() if int(splitParams[13], 16) == 0 else ACTIVITY_THRESHOLD[int(splitParams[13], 16)],
                "ReactionTime_s": self.reaction_time.value() if int(splitParams[14], 16) == 0 else int(splitParams[14], 16),
                "ResponseFactor": self.response_factor.value() if int(splitParams[15], 16) == 0 else int(splitParams[15], 16),
                "RecoveryTime_min": self.recovery_time.value() if int(splitParams[16], 16) == 0 else int(splitParams[16], 16),
                "MSR_ppm": self.msr.value() if int(splitParams[17], 16) == 0 else int(splitParams[17], 16)
                }
            else:
                QMessageBox.critical(self, "Errror", "Check Sum Failed")
                return
  
        else:
            QMessageBox.critical(self, "Error", "Data Sent Not Correct")
            return


    # apply a dict of params back onto the widgets, use for revert
    def _apply_params_to_widgets(self, p: Dict[str, Any]) -> None:
        self.lrl.setValue(p.get("LRL_ppm", self.lrl.value()))
        self.url.setValue(p.get("URL_ppm", self.url.value()))
        self.msr.setValue(p.get("MSR_ppm", self.msr.value()))
        self.atrial_amp.setValue(p.get("AtrialAmplitude_V", self.atrial_amp.value()))
        self.atrial_pw.setValue(p.get("AtrialPulseWidth_ms", self.atrial_pw.value()))
        self.ventricular_amp.setValue(p.get("VentricularAmplitude_V", self.ventricular_amp.value()))
        self.ventricular_pw.setValue(p.get("VentricularPulseWidth_ms", self.ventricular_pw.value()))
        self.arp.setValue(p.get("ARP_ms", self.arp.value()))
        self.vrp.setValue(p.get("VRP_ms", self.vrp.value()))

        # NEW fields
        self.atrial_sens.setValue(p.get("AtrialSensitivity_mV", self.atrial_sens.value()))
        self.ventricular_sens.setValue(p.get("VentricularSensitivity_mV", self.ventricular_sens.value()))
        self.hysteresis_state.setCurrentText(p.get("Hysteresis", self.hysteresis_state.currentText()))
        # enable/disable HRL based on state
        self.HysRateLimit.setEnabled(self.hysteresis_state.currentText() == "On")
        self.HysRateLimit.setValue(p.get("HRL_ppm", self.HysRateLimit.value()))
        self.smooth_up.setCurrentText(_int_to_percent(p.get("RateSmoothingUp_percent", _percent_to_int(self.smooth_up.currentText()))))
        self.smooth_down.setCurrentText(_int_to_percent(p.get("RateSmoothingDown_percent", _percent_to_int(self.smooth_down.currentText()))))

        # D2 Parameters
        self.response_factor.setValue(p.get("ResponseFactor", self.response_factor.value()))
        self.reaction_time.setValue(p.get("ReactionTime_s", self.reaction_time.value()))
        self.activity_threshold.setCurrentIndex(self.activity_threshold.findText(p.get("ActivityThreshold", self.activity_threshold.currentText())))
        self.recovery_time.setValue(p.get("RecoveryTime_min", self.recovery_time.value()))

    # EGRAM GRAPH PHOTOS 3.2.5
    def _save_egram_png(self) -> None:
        # self parent, dialog title, suggested default file name, file type filter only shows png returns selected path, type
        path, _ = QFileDialog.getSaveFileName(self, "Save Egram Snapshot", "egram.png", "PNG Files (*.png)")
        if not path:
            return
        screenshot = self.egram_view.grab()
        photoTaken = screenshot.save(path, "PNG")
        if photoTaken:
            QMessageBox.information(self, "Saved!", f"Egram snapshot saved to:\n{path}")
        else:
            QMessageBox.warning(self, "Error!", "Could not save the image.")
    def _print_egram_pdf(self) -> None:
        #    Render the egram_view into a PDF using QPrinter.
    
        suggested = "egram_strip.pdf"
        dest, _ = QFileDialog.getSaveFileName(self, "Save Egram PDF", suggested, "PDF Files (*.pdf)")
        if not dest:
            return
        printer = QPrinter(QPrinter.HighResolution)
        printer.setOutputFormat(QPrinter.PdfFormat)
        printer.setOutputFileName(dest)
        printer.setPageSize(QPrinter.A4)

        # render the widget to the printer
        painter = QPainter()
        if not painter.begin(printer):
            QMessageBox.warning(self, "Print Error", "Could not initialize printer.")
            return
            # scale widget to printer page while preserving aspect
        rect = painter.viewport()
        size = self.egram_view.size()
        size.scale(rect.size(), Qt.KeepAspectRatio)
        painter.setViewport(rect.x(), rect.y(), size.width(), size.height())
        painter.setWindow(self.egram_view.rect())
        self.egram_view.render(painter)
        painter.end()
        QMessageBox.information(self, "Saved", f"Egram PDF saved to:\n{dest}")


    # rebuild the HTML text for summary tab
    def _refresh_summary(self) -> None:
        p = self._collect_params()
        lines = [
            f"<b>Mode:</b> {p['mode']}",
            f"<b>LRL:</b> {p['LRL_ppm']} ppm",
            f"<b>URL:</b> {p['URL_ppm']} ppm",
            f"<b>MSR:</b> {p['MSR_ppm']} ppm",
            f"<b>Atrial:</b> Amp {p['AtrialAmplitude_V']} V, PW {p['AtrialPulseWidth_ms']} ms",
            f"<b>Ventricular:</b> Amp {p['VentricularAmplitude_V']} V, PW {p['VentricularPulseWidth_ms']} ms",
            f"<b>ARP:</b> {p['ARP_ms']} ms, <b>VRP:</b> {p['VRP_ms']} ms",

        ]
        # hysteresis params for AAI, VVI, AAIR, VVIR
        if self.current_mode in ("AAI", "VVI", "AAIR", "VVIR"):
            lines.append(f"<b>Sensitivity:</b> {p['AtrialSensitivity_mV']}" if self.current_mode in ("AAI", "AAIR") else f"<b>Sensitivity:</b> {p['VentricularSensitivity_mV']}")
            lines.append(f"<b>Hysteresis:</b> {p['Hysteresis']}"
                         + (f", HRL {p['HRL_ppm']} ppm" if p['Hysteresis']=='On' else ""))
            lines.append(f"<b>Rate Smoothing:</b> Up {_int_to_percent(p['RateSmoothingUp_percent'])}, "
                         f"Down {_int_to_percent(p['RateSmoothingDown_percent'])}")
        if self.current_mode in ("AOOR", "VOOR", "AAIR", "VVIR"):
            lines.extend((f"<b>Activity Threshold:</b> {p['ActivityThreshold']}",
            f"<b>Reaction Time:</b> {p['ReactionTime_s']} s",
            f"<b>Response Factor:</b> {p['ResponseFactor']}",
            f"<b>Recovery Time:</b> {p['RecoveryTime_min']} min"))
        self.label_summary.setText("<br>".join(lines))

    def _defaults(self, mode: str) -> Dict[str, Any]:
        """general defaults based on nominal values, to refer back to"""
        base = {
            "LRL_ppm": 60,
            "URL_ppm": 120,
            "MSR_ppm": 120,
            "AtrialPulseWidth_ms": 0.40,
            "VentricularPulseWidth_ms": 0.40,
            "ARP_ms": 250,
            "VRP_ms": 320,
            "AtrialSensitivity_mV": 2.5,
            "VentricularSensitivity_mV": 2.5,
            "Hysteresis": "Off",
            "HRL_ppm": 60,
            "RateSmoothingUp_percent": 0,
            "RateSmoothingDown_percent": 0,
            "ActivityThreshold": "Med",
            "ReactionTime_s": 10,
            "ResponseFactor": 8,
            "RecoveryTime_min": 5
        }
        # only amplitudes depend on the mode, rest general
        base["AtrialAmplitude_V"]      = 3.0 if mode in ("AOO", "AAI", "AOOR", "AAIR") else "Off"
        base["VentricularAmplitude_V"] = 3.5 if mode in ("VOO", "VVI", "VOOR", "VVIR") else "Off"
        return base

    def _apply_defaults_for_mode(self, mode: str) -> None:
        """write defaults into the widgets (go back to when nothing saved)"""
        d = self._defaults(mode)
        self.lrl.setValue(d["LRL_ppm"])
        self.url.setValue(d["URL_ppm"])
        self.msr.setValue(d["MSR_ppm"])
        self.atrial_amp.setValue(d["AtrialAmplitude_V"])
        self.atrial_pw.setValue(d["AtrialPulseWidth_ms"])
        self.ventricular_amp.setValue(d["VentricularAmplitude_V"])
        self.ventricular_pw.setValue(d["VentricularPulseWidth_ms"])
        self.arp.setValue(d["ARP_ms"])
        self.vrp.setValue(d["VRP_ms"])
        self.atrial_sens.setValue(d["AtrialSensitivity_mV"])
        self.ventricular_sens.setValue(d["VentricularSensitivity_mV"])
        self.hysteresis_state.setCurrentText(d["Hysteresis"])
        self.HysRateLimit.setEnabled(self.hysteresis_state.currentText() == "On")
        self.HysRateLimit.setValue(d["HRL_ppm"])
        self.smooth_up.setCurrentText(_int_to_percent(d["RateSmoothingUp_percent"]))
        self.smooth_down.setCurrentText(_int_to_percent(d["RateSmoothingDown_percent"]))
        self.activity_threshold.setCurrentIndex(self.activity_threshold.findText(d["ActivityThreshold"]))
        self.reaction_time.setValue(d["ReactionTime_s"])
        self.response_factor.setValue(d["ResponseFactor"])
        self.reaction_time.setValue(d["RecoveryTime_min"])


    # SAVE!!!!!!!!!!
    def _handle_save(self) -> None:
        user = self.get_active_user()
        if not user:
            QMessageBox.warning(self, "No user", "Please log in first.")
            return
        # safety: LRL should not exceed URL
        if self.lrl.value() > self.url.value():
            QMessageBox.warning(self, "Check Parameters!", "LRL must be <= URL.")
            return
        params = self._collect_params()
        self.db.save_params(user, self.current_mode, params)
        QMessageBox.information(self, "Saved", f"Parameters saved for {user} [{self.current_mode}].")
        self._refresh_summary()


    # REVERT!!!! (also called by set_mode)
    def _handle_revert(self) -> None:
        user = self.get_active_user()
        saved = self.db.load_params(user, self.current_mode) if user else {}
        if saved:
            self._apply_params_to_widgets(saved)
        else:
            # no saved params, go to default
            self._apply_defaults_for_mode(self.current_mode)
        self._refresh_summary()

    # D2: Load data to pacemaker -WRITE
    def _handle_load(self) -> None:
        user = self.get_active_user()
        if not user:
            QMessageBox.warning(self, "No user", "Please log in first.")
            return
        # safety: LRL should not exceed URL
        if self.lrl.value() > self.url.value():
            QMessageBox.warning(self, "Check Parameters!", "LRL must be <= URL.")
            return


        data = self._package_params()
        MainWindow._write_to_serial(win, data)
        QMessageBox.information(self, "Loaded", f"Parameters loaded for {user} [{self.current_mode}].")
        
    # D2: Get data from pacemaker and write to DCM
    def _handle_get(self) -> None:
        # TODO: get data from serial
        # data_test = "16 49 1 0 ff 3c 78 1e a 19 0 0 0 0 0 0 0 0 f5" #HARDCODED  -> change to d= self.serial....
        # d = self._process_to_params(data_test)
        self.serial_thread.readFromPacemaker()
        d = self.serial_thread.serial_recieved.connect(self._process_to_params) #<-
        
        user = self.get_active_user()
        try:
            self.set_mode(d['mode'])
            self._apply_params_to_widgets(d)
            self._refresh_summary()
            QMessageBox.information(self, "Data Retrieved", f"Parameters for {user} [{self.current_mode}] retrieved successfully")
        except TypeError:
            QMessageBox.critical(self, "Error", "Serial message format incorrect, parameters not changed")
        
    def _handle_get(self) -> None:
        # TODO: get data from serial
        data_test = MainWindow._read_from_serial(win)
        d = self._process_to_params(data_test)

        user = self.get_active_user()
        try:
            self.set_mode(d['mode'])
            self._apply_params_to_widgets(d)
            self._refresh_summary()
            QMessageBox.information(self, "Data Retrieved", f"Parameters for {user} [{self.current_mode}] retrieved successfully")
        except TypeError:
            QMessageBox.critical(self, "Error", "Serial message format incorrect, parameters not changed") 



# =========================
# 8) Egram container (for D2)
# =========================
class EgramData:
    """Egram page placeholder for streaming data in D2"""
    def __init__(self, time_ms: List[int], atrial_mv: List[float], ventricular_mv: List[float]):
        self.time_ms = time_ms # time in ms
        self.atrial_mv = atrial_mv # in mv
        self.ventricular_mv = ventricular_mv

    def __repr__(self) -> str: # printing in debug contexts
        return f"EgramData(n_samples={len(self.time_ms)})"


# =========================
# 9) Main application window (owns pages + menus + status bar)
# =========================
class MainWindow(QMainWindow):
    """
    MainWindow is the top-level frame:
      - creates a QStackedWidget to hold three pages
      - wires callbacks between pages (login -> dashboard -> editor)
      - hosts "Simulator" menu to flip comms/device/telemetry flags
      - shows status bar text based on those flags
    """

    def __init__(self):
        super().__init__()  # QMainWindow init
        self.setWindowTitle("DCM — Deliverable 2")
        self.resize(900, 600)

        # global app state
        self.db = Database(DB_FILE) # database instance pointing at json shared to children
        self.active_user: Optional[str] = None  # None until someone logs in

        # telemetry and serial connection states (D2)
        self.comms_connected = False
        self.available_devices = {} # Available serial ports & devices
        self.device_id = ["", ""] # tuple, index 0 is COM port, index 1 is nickname
        self.device_changed = False
        self.telemetry_state = "ok"  # one of "ok" | "out_of_range" | "noise"
        self.serial_thread = None  # ensure the attribute exists before connecting -> MAKE IT NOT A REFERENCE (directly an instance)


        # device clock 3.2.3 #2
        self.device_clock = QDateTime.currentDateTime()
        
        # creates a QStackedWidget (a deck where exactly one “page” is shown).
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # create the pages and pass callbacks/DB as needed
        self.page_login = LoginPage(self.db, self._on_login_ok)
        self.page_dash  = DashboardPage(self._open_mode_editor)
        self.page_edit  = ModeEditorPage(self.db, self._get_active_user, self.serial_thread) # active user?

        # add all pages to the stack 
        for p in (self.page_login, self.page_dash, self.page_edit):
            self.stack.addWidget(p)

        self.stack.setCurrentWidget(self.page_login)  # start on login page

        # status bar at the bottom 
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._refresh_status_bar() # fills w initial status text

        # build top menu bar ("File", "Simulator")
        self._build_menus()
    
    # D2: override closeEvent function to terminate any running threads
    def closeEvent(self, event):
        if self.comms_connected and self.serial_thread is not None: #added additional error trap
            self.serial_thread.stop() 
        event.accept()

    # build menus and wire actions to methods
    def _build_menus(self) -> None:
        MenuBar = self.menuBar() # QMainWindow gives us a menu bar

        # file menu (export / reports / quit)
        menu_file = MenuBar.addMenu("File")

        # reports submenu as per 3.2.4 1,2
        menu_reports = menu_file.addMenu("Reports")
        action_r1 = QAction("Bradycardia Parameters Report…", self)
        action_r2 = QAction("Temporary Parameters Report…", self)
        action_r1.triggered.connect(self._export_brady_params_report) # when clicked generate pdf
        action_r2.triggered.connect(self._export_temp_params_report)
        menu_reports.addAction(action_r1)
        menu_reports.addAction(action_r2)

        action_export = QAction("Export Saved Params JSON…", self) # export json
        action_export.triggered.connect(self._export_all_json)
        menu_file.addAction(action_export)

        menu_file.addSeparator() # visual separator line in menu
        action_quit = QAction("Quit", self)
        action_quit.triggered.connect(self.close)
        menu_file.addAction(action_quit)

        # simulator menu (toggles and radio options for telemetry)
        menu_sim = MenuBar.addMenu("Simulator")

        # D2: Nested menu of available serial ports
        self.serialPorts = menu_sim.addMenu("Available Ports:")
        self.available_devices = QSerialPortInfo.availablePorts()
        for port in self.available_devices:
            self.action_port = QAction(str(port.portName()), self, checkable=True)
            self.action_port.triggered.connect(self._toggle_comms)
            self.serialPorts.addAction(self.action_port)

        # D2: Refresh list of serial ports
        self.action_refresh = QAction("Refresh Ports", self)
        self.action_refresh.triggered.connect(self._refresh_ports)
        menu_sim.addAction(self.action_refresh)

        # D2: Opens dialog to set a device alias
        self.action_set_device = QAction("Set Device ID", self)
        self.action_set_device.triggered.connect(self._set_device_id)
        menu_sim.addAction(self.action_set_device)

        # radio group for telemetry  ok default
        self.telemetry_group = QActionGroup(self)
        self.telemetry_group.setExclusive(True)  # makes them mutual-exclusive

        self.action_tel_ok = QAction("Telemetry OK", self, checkable=True)
        self.action_tel_oor = QAction("Loss: Out of Range", self, checkable=True)
        self.action_tel_noise = QAction("Loss: Noise", self, checkable=True)
        self.action_tel_ok.setChecked(True) # default selection

        # utilities menu (about/set clock)
        menu_utilities = MenuBar.addMenu("Utilities")

        action_about = QAction("About…", self)
        action_about.triggered.connect(self._show_about)
        menu_utilities.addAction(action_about)

        action_clock = QAction("Set Clock…", self)
        action_clock.triggered.connect(self._set_clock_dialog)
        menu_utilities.addAction(action_clock)

        # put actions in the group + menu
        for a in (self.action_tel_ok, self.action_tel_oor, self.action_tel_noise):
            self.telemetry_group.addAction(a)
            menu_sim.addAction(a)

        # connect each radio action to a lambda that sets the string state
        self.action_tel_ok.triggered.connect(lambda: self._set_telemetry("ok"))
        self.action_tel_oor.triggered.connect(lambda: self._set_telemetry("out_of_range"))
        self.action_tel_noise.triggered.connect(lambda: self._set_telemetry("noise"))

    #file menu handler: export our JSON DB to a chosen file path
    def _export_all_json(self) -> None: # opens save file dialog, dest is full chosen path or empty if canceled
        dest, _ = QFileDialog.getSaveFileName(self, "Export database JSON", "dcm_params.json")
        if not dest:  # user hit cancel
            return # exit
        with open(DB_FILE, "r", encoding="utf-8") as source, open(dest, "w", encoding="utf-8") as out:
            out.write(source.read())
        QMessageBox.information(self, "Exported", f"Saved to {dest}")
        # ^^ source: current database file on disk, out is file user picked in dialog

    # PNGS 3.2.4
    def _current_params(self) -> Optional[Dict[str, Any]]:
        """get currently shown editor parameters, or None if no user"""
        user = self._get_active_user()
        if not user:
            return None
        return self.page_edit._collect_params() # ask modeeditorpage to collect whats currently in widgets and return dict

    def _report_header_html(self, report_name: str) -> str:
        # spec header fields
        now = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss") # get current time as string
        return f"""
        <h2 style="margin-bottom:2px;">{report_name}</h2> 
        <hr>
        <table cellspacing="4">
          <tr><td><b>Institution:</b></td><td>{APP_INSTITUTION}</td></tr>
          <tr><td><b>Printed:</b></td><td>{now}</td></tr>
          <tr><td><b>DCM Model/Version:</b></td><td>{APP_MODEL_NUMBER} / {APP_SOFTWARE_REV}</td></tr>
          <tr><td><b>DCM Serial:</b></td><td>{APP_SERIAL_NUMBER}</td></tr>
          <tr><td><b>Device ID:</b></td><td>{self.device_id[1] or 'None'}</td></tr>
          <tr><td><b>Report Name:</b></td><td>{report_name}</td></tr>
        </table>
        <br>
        """
    #^^ html string

    def _params_table_html(self, p: Dict[str, Any]) -> str:
        rows = []
        def row(k, v): rows.append(f"<tr><td><b>{k}</b></td><td>{v}</td></tr>")

        row("Mode", p["mode"])
        row("LRL", f'{p["LRL_ppm"]} ppm')
        row("URL", f'{p["URL_ppm"]} ppm')
        row("MSR", f'{p["MSR_ppm"]} ppm')
        row("Atrial Amplitude", f'{p["AtrialAmplitude_V"]} V')
        row("Atrial Pulse Width", f'{p["AtrialPulseWidth_ms"]} ms')
        row("Ventricular Amplitude", f'{p["VentricularAmplitude_V"]} V')
        row("Ventricular Pulse Width", f'{p["VentricularPulseWidth_ms"]} ms')
        row("ARP", f'{p["ARP_ms"]} ms')
        row("VRP", f'{p["VRP_ms"]} ms')
        if p["mode"] in ("AAI", "VVI", "AAIR", "VVIR"):
            row("Atrial Sensitivity", f'{p["AtrialSensitivity_mV"]} mV')
            row("Ventricular Sensitivity", f'{p["VentricularSensitivity_mV"]} mV')
            row("Hysteresis", p["Hysteresis"] + (f' (HRL {p["HRL_ppm"]} ppm)' if p["Hysteresis"]=="On" else ""))
            row("Rate Smoothing Up", f'{p["RateSmoothingUp_percent"]}%')
            row("Rate Smoothing Down", f'{p["RateSmoothingDown_percent"]}%')
        if p["mode"] in ("AOOR", "VOOR", "VVIR", "AAIR"):
            row("Activity Threshold", f'{p["ActivityThreshold"]}')
            row("Reaction Time", f'{p["ReactionTime_s"]}')
            row("Response Factor", f'{p["ResponseFactor"]}')
            row("Recovery Time", f'{p["RecoveryTime_min"]}')

        return f"""
        <table border="1" cellspacing="0" cellpadding="4">
            {''.join(rows)}
        </table>
        """

    def _save_pdf(self, html: str, suggested: str) -> None:
        dest, _ = QFileDialog.getSaveFileName(self, "Save PDF", suggested, "PDF Files (*.pdf)")
        if not dest:
            return
        # print HTML to PDF
        doc = QTextDocument()
        doc.setHtml(html)
        printer = QPrinter(QPrinter.HighResolution)
        printer.setOutputFormat(QPrinter.PdfFormat)
        printer.setOutputFileName(dest)
        doc.print_(printer)
        QMessageBox.information(self, "Saved", f"PDF saved to:\n{dest}")

    def _export_brady_params_report(self) -> None:
        p = self._current_params()
        if not p:
            QMessageBox.warning(self, "No user", "Please log in and open the Mode Editor first.")
            return
        html = self._report_header_html("Bradycardia Parameters Report") + self._params_table_html(p)
        self._save_pdf(html, "Bradycardia Parameters Report.pdf")

    def _export_temp_params_report(self) -> None:
        p = self._current_params()
        if not p:
            QMessageBox.warning(self, "No user", "Please log in and open the Mode Editor first.")
            return
        html = self._report_header_html("Temporary Parameters Report") + self._params_table_html(p)
        self._save_pdf(html, "Temporary Parameters Report.pdf")


    # NAVIGATION CALLED BY LoginPage when login OK
    def _on_login_ok(self, username: str) -> None:
        self.active_user = username
        self.stack.setCurrentWidget(self.page_dash)  # switch to dashboard
        self.status_bar.showMessage(f"Logged in as {username}", 4000)

    # navigation: called by DashboardPage when a mode button clicked 
    def _open_mode_editor(self, mode: str) -> None:
        self.page_edit.set_mode(mode) # tell editor which mode
        self.stack.setCurrentWidget(self.page_edit) # switch to editor page

    # helper passed to ModeEditorPage so it can ask who is logged in
    def _get_active_user(self) -> Optional[str]:
        return self.active_user

    # simulator handlers: flip internal flags and update labels/status 
    def _toggle_comms(self) -> None:
        self.comms_connected = self.action_port.isChecked()

        if self.comms_connected:
            try:
                # ping the port
                self.serial_thread = SerialThread(self)
                # after creating serial_thread and starting it:
                self.serial_thread.port_setup(self.action_port.text() if hasattr(self, 'COM4') else "COM3", BAUD_RATE)
                self.serial_thread.start()
                # attach to the ModeEditorPage so the EgramView can access it
                self.page_edit.attach_serial_thread(self.serial_thread)

                self.serial_thread.port_setup(self.device_id[0], BAUD_RATE, 8)
                self.serial_thread.start()
                self.page_edit.serial_thread = self.serial_thread

                if self.action_port.text() != self.device_id[0]:
                    self.device_changed = True
                    self.device_id[0] = self.action_port.text()
                    self.device_id[1] = self.action_port.text() # set default nickname to port
                    QMessageBox.information(self, "Device changed", f"Current device: {self.device_id[1]}.")
                    self.page_dash.show_device(self.device_id[1])
                    
            except SerialException:
                self.serial_thread.stop()
                QMessageBox.critical(self, "Error", "Failed to connect to serial port")
                self.comms_connected = False
        else:
            if self.serial_thread is not None: #conditional stop
                self.serial_thread.stop()
                self.serial_thread = None #proper stop
            QMessageBox.information(self, "Device disconnected", f"Disconnected from {self.device_id[1]}.")
            #not properly displaying port name
            self.comms_connected = False
            self.device_id = ["", ""]
            self.page_dash.show_device(self.device_id[1])

        self.page_dash.show_comms(self.comms_connected)
        
        self._refresh_status_bar()

    def _refresh_ports(self) -> None:
        #Refesh the status bar and re-check for serial ports
        if QSerialPortInfo.availablePorts() != self.available_devices:
            if self.comms_connected:
                QMessageBox.warning(self, "Warning", "Cannot refresh, there is a device currently connected.")
            else:
                self.available_devices = QSerialPortInfo.availablePorts()
                self.serialPorts.clear()
                self.available_devices = QSerialPortInfo.availablePorts()
                for port in self.available_devices:
                    self.action_port = QAction(str(port.portName()), self, checkable=True)
                    self.action_port.triggered.connect(self._toggle_comms)
                    self.serialPorts.addAction(self.action_port)
        
        self._refresh_status_bar()
        
    def _set_device_id(self) -> None:
        # reuse a save dialog as a crude "enter a string" prompt.
        if not self.comms_connected:
            QMessageBox.warning(self, "Warning", "No device is connected!")
        else:
            device_name = QInputDialog.getText(self, 'Input Dialog', 'Enter a nickname for this device:')
            self.device_id[1] = device_name[0]
        self.page_dash.show_device(self.device_id[1])
        self._refresh_status_bar()

    def _set_telemetry(self, state: str) -> None:
        self.telemetry_state = state
        self.page_dash.show_telemetry(state)
        self._refresh_status_bar()

    # status bar
    def _refresh_status_bar(self) -> None:
        comms = "Connected" if self.comms_connected else "Not Connected"
        changed = "Device Changed" if self.device_changed else "Last Device OK"
        tel = {
            "ok": "Telemetry: OK",
            "out_of_range": "Telemetry: Lost – Out of Range",
            "noise": "Telemetry: Lost – Noise"
        }[self.telemetry_state]
        clock_str = self.device_clock.toString("yyyy-MM-dd HH:mm:ss")
        text = f"{comms} | Device: {self.device_id[1] or 'None'} | {changed} | {tel} | Clock: {clock_str}"

        self.status_bar.showMessage(text)

    def _show_about(self) -> None:
        AboutDialog(self).exec_()

    def _set_clock_dialog(self) -> None:
        dlg = SetClockDialog(self.device_clock, self)
        if dlg.exec_() == QDialog.Accepted:
            self.device_clock = dlg.selected_datetime()
            self._refresh_status_bar()
    def _write_to_serial(self, data: str):
        print(data) #this is for testing purposes, below is the actual implementation
        #size = len(data) #CHANGE THIS! not actual byte size
        #reinitialize for correct size
        #self.serial_thread.portSetup(self.self.device_id[0], BAUD_RATE, size)
        #self.serial_thread.writeToQueue(data)
    def _read_from_serial(self) -> str:
        data_test = "16 49 1 0 ff 3c 78 1e a 19 0 0 0 0 0 0 0 0 f5" #again just for testing
        return data_test
        #self.serial_thread.readFromPacemaker()
        #d = self.serial_thread.serial_recieved.connect(self._process_to_params)
        #return d


# =========================
# 10) RUNNING!!!!
# =========================
if __name__ == "__main__":
    app = QApplication([]) # create the app object (one per process)

    win = MainWindow() # create main window
    win.show() # make it visible
    app.exec_() # enter Qt event loop (blocks until window closes)

