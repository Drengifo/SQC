#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import threading
import os
import subprocess
import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
from datetime import datetime
import queue


import serial
import serial.tools.list_ports

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    MATPLOTLIB_OK = True
except Exception:
    MATPLOTLIB_OK = False

# ─────────────────────────────────────────────────────────────
# Parámetros base
# ─────────────────────────────────────────────────────────────
TOL_MM = 0.05
SER_BAUD = 115200
SER_PORT = None
SER_TIMEOUT = 1.0
PHOTO_MONITOR_MS = 5000
SESSION_FOLDER_FORMAT = "%H-%M-%S_%d-%m-%Y"
SENSOR_IGNORE_SECONDS = 1.5
LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S = 0.45
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
CONFIG_PATH = os.path.join(SCRIPT_DIR, "config.txt")

# Calibración LVDT
CAL_A = 0.5054
CAL_RAW0 = 344.57
cal_sign = -1

def raw_to_mm(raw):
    return cal_sign * CAL_A * (raw - CAL_RAW0)

# ─────────────────────────────────────────────────────────────
# Ventana fija optimizada para Raspberry Pi
# ─────────────────────────────────────────────────────────────
WINDOW_CLIENT_W = 1024
WINDOW_CLIENT_H = 520

HEADER_H = 96
LEFT_W = 238
MAIN_X = LEFT_W + 6
MAIN_Y = HEADER_H + 6
MAIN_W = WINDOW_CLIENT_W - MAIN_X - 6
MAIN_H = WINDOW_CLIENT_H - MAIN_Y - 6

TOP_SECTION_H = 116

SHARED_AREA_W = MAIN_W - 16
SHARED_AREA_H = MAIN_H - TOP_SECTION_H - 14
SHARED_FOOTER_H = 34
SHARED_CONTENT_H = SHARED_AREA_H - SHARED_FOOTER_H - 4

# ─────────────────────────────────────────────────────────────
# Estilo
# ─────────────────────────────────────────────────────────────
APP_BG = "#eef2f6"
HEADER_BG = "#16395d"
TAB_ACTIVE_BG = "#2a75b3"
TAB_BG = "#edf1f5"
TAB_FG = "#1f2d3d"

CARD_BG = "#ffffff"
CARD_BORDER = "#cfd8e3"

TEXT_MAIN = "#1f2d3d"
TEXT_MUTED = "#5a6b7b"
STATUS_BG = "#e7f2ec"

BTN_BLUE = "#2a75b3"
BTN_BLUE_DARK = "#1f5c8e"
BTN_LIGHT = "#dce6f0"
BTN_RED = "#cc4b4b"
BTN_GREEN = "#2c8b49"
BTN_ORANGE = "#d08a00"

FONT_TITLE = ("Helvetica", 18, "bold")
FONT_MODE = ("Helvetica", 11)
FONT_CARD = ("Helvetica", 10, "bold")
FONT_SMALL = ("Helvetica", 8)
FONT_MED = ("Helvetica", 9)
FONT_BIGVAL = ("Helvetica", 13, "bold")
FONT_BTN = ("Helvetica", 10, "bold")
FONT_BTN2 = ("Helvetica", 10)
FONT_CONSOLE = ("Courier", 10)

# ─────────────────────────────────────────────────────────────
# GPIO relés
# ─────────────────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
rele_pin = [17, 27, 22, 23]
for pin in rele_pin:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

PULSE_SECONDS = 1.0
CH_INICIO = 1
CH_DETENER = 2
CH_IZQUIERDA = 3
CH_DERECHA = 4

# ─────────────────────────────────────────────────────────────
# GPIO LEDs de estado
# ─────────────────────────────────────────────────────────────
LED_PIN_IZQUIERDA = 24
LED_PIN_DETENIDO = 25
LED_PIN_DERECHA = 26
LED_PINS = [LED_PIN_IZQUIERDA, LED_PIN_DETENIDO, LED_PIN_DERECHA]

for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# ─────────────────────────────────────────────────────────────
# Estado global
# ─────────────────────────────────────────────────────────────
msg = "Sistema detenido"
sistema_iniciado = False
dir_actual = None
selected_direction = None
manual_override = False
user_zero_applied = False
lvdt_offset = 0.0

_latest_lvdt_raw = None
_latest_pres1_raw = None
_latest_pres2_raw = None
_latest_temp1_raw = None
_latest_temp2_raw = None

sensor_ignore_until = 0.0
lvdt_filter_lock = threading.Lock()
_lvdt_effective_mm = None
_lvdt_effective_time = None

stop_move = threading.Event()
motion_thread = None
auto_running = False
auto_paused = threading.Event()
auto_start_time = None
auto_run_epoch = None
auto_pause_started = None
auto_paused_accumulated = 0.0
disp_list = []
loaded_txt_path = ""

log_handle = None
console_log_handle = None
console_log_lock = threading.Lock()
auto_log_thread = None

_pos_buffer = []
_time_buffer = []

# Consolas
console_widgets = []

# Vista actual
current_view = "manual"
shared_center_mode = "messages"

# Sesiones / carpetas
session_root_folder = None
session_data_folder = None
session_log_folder = None
session_photo_folder = None

# Foto
photo_scheduler_thread = None
photo_scheduler_stop = threading.Event()
photo_capture_queue = queue.Queue()
auto_photo_enabled_default = True
photo_capture_active = False
photo_status_message = "Estado foto: Lista."
photo_status_color = TEXT_MUTED


def get_elapsed_automatic_time_for_file():
    if auto_start_time is None:
        return None

    if auto_paused.is_set() and auto_pause_started is not None:
        elapsed = auto_pause_started - auto_start_time - auto_paused_accumulated
    else:
        elapsed = time.time() - auto_start_time - auto_paused_accumulated

    if elapsed < 0:
        elapsed = 0.0
    return elapsed


def format_elapsed_photo_filename(elapsed_seconds):
    total_ms = int(round(elapsed_seconds * 1000.0))
    if total_ms < 0:
        total_ms = 0
    secs = total_ms // 1000
    ms = total_ms % 1000
    return f"{secs:04d}.{ms:03d}.jpg"


def get_default_base_folder():
    desktop = os.path.join(os.path.expanduser("~"), "Desktop")
    if os.path.isdir(desktop):
        return desktop
    return os.path.expanduser("~")


def get_default_session_preview_folder():
    folder = os.path.join(get_default_base_folder(), "Actuador-PUCV")
    os.makedirs(folder, exist_ok=True)
    return folder


DEFAULT_CONFIG = {
    "SENSOR_IGNORE_SECONDS": "1.5",
    "LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S": "0.45",
    "AUTO_RIGHT_INCREASES_LVDT": "0",
    "LVDT_ZERO_OFFSET_MM": "0.0",
    "PHOTO_FOLDER": "",
    "PHOTO_AUTO_ENABLED": "1",
    "PHOTO_INTERVAL_MIN": "0.5",
    "KEYBOARD_ENABLED": "0",
    "MANUAL_GRAPH_WINDOW": "15 s",
    "AUTO_GRAPH_WINDOW": "2 min",
    "MANUAL_DISPLACEMENT_MM": "1",
    "AUTO_GUARDAR": "0",
    "CFG_STOP_ALERT": "0",
    "P1_MAX": "250.0",
    "P2_MAX": "250.0",
    "LVDT_MIN": "-25.0",
    "LVDT_MAX": "25.0",
    "T1_MAX": "80.0",
    "T2_MAX": "80.0",
    "PRES1_PUNTO1_RAW": "0.0",
    "PRES1_PUNTO2_RAW": "4095.0",
    "PRES1_PUNTO1_REAL": "0.0",
    "PRES1_PUNTO2_REAL": "250.0",
    "PRES2_PUNTO1_RAW": "0.0",
    "PRES2_PUNTO2_RAW": "4095.0",
    "PRES2_PUNTO1_REAL": "0.0",
    "PRES2_PUNTO2_REAL": "250.0",
    "TEMP1_PUNTO1_RAW": "0.0",
    "TEMP1_PUNTO2_RAW": "4095.0",
    "TEMP1_PUNTO1_REAL": "0.0",
    "TEMP1_PUNTO2_REAL": "100.0",
    "TEMP2_PUNTO1_RAW": "0.0",
    "TEMP2_PUNTO2_RAW": "4095.0",
    "TEMP2_PUNTO1_REAL": "0.0",
    "TEMP2_PUNTO2_REAL": "100.0",
}


def config_get_str(data, key, default=""):
    value = data.get(key, default)
    return str(value).strip()


def config_get_float(data, key, default=0.0):
    try:
        return float(str(data.get(key, default)).replace(",", ".").strip())
    except Exception:
        return float(default)


def config_get_bool(data, key, default=False):
    value = str(data.get(key, "1" if default else "0")).strip().lower()
    return value in ("1", "true", "yes", "si", "on")


def write_config_file(data):
    lines = [
        "# Configuración Actuador-PUCV",
        "# Edita los valores según sea necesario.",
        "",
        "# Parámetros principales",
        f"SENSOR_IGNORE_SECONDS = {data.get('SENSOR_IGNORE_SECONDS', DEFAULT_CONFIG['SENSOR_IGNORE_SECONDS'])}",
        f"LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S = {data.get('LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S', DEFAULT_CONFIG['LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S'])}",
        f"AUTO_RIGHT_INCREASES_LVDT = {data.get('AUTO_RIGHT_INCREASES_LVDT', DEFAULT_CONFIG['AUTO_RIGHT_INCREASES_LVDT'])}",
        f"LVDT_ZERO_OFFSET_MM = {data.get('LVDT_ZERO_OFFSET_MM', DEFAULT_CONFIG['LVDT_ZERO_OFFSET_MM'])}",
        "",
        "# Estado general de la app",
        f"PHOTO_FOLDER = {data.get('PHOTO_FOLDER', '')}",
        f"PHOTO_AUTO_ENABLED = {data.get('PHOTO_AUTO_ENABLED', DEFAULT_CONFIG['PHOTO_AUTO_ENABLED'])}",
        f"PHOTO_INTERVAL_MIN = {data.get('PHOTO_INTERVAL_MIN', DEFAULT_CONFIG['PHOTO_INTERVAL_MIN'])}",
        f"KEYBOARD_ENABLED = {data.get('KEYBOARD_ENABLED', DEFAULT_CONFIG['KEYBOARD_ENABLED'])}",
        f"MANUAL_GRAPH_WINDOW = {data.get('MANUAL_GRAPH_WINDOW', DEFAULT_CONFIG['MANUAL_GRAPH_WINDOW'])}",
        f"AUTO_GRAPH_WINDOW = {data.get('AUTO_GRAPH_WINDOW', DEFAULT_CONFIG['AUTO_GRAPH_WINDOW'])}",
        f"MANUAL_DISPLACEMENT_MM = {data.get('MANUAL_DISPLACEMENT_MM', DEFAULT_CONFIG['MANUAL_DISPLACEMENT_MM'])}",
        f"AUTO_GUARDAR = {data.get('AUTO_GUARDAR', DEFAULT_CONFIG['AUTO_GUARDAR'])}",
        f"CFG_STOP_ALERT = {data.get('CFG_STOP_ALERT', DEFAULT_CONFIG['CFG_STOP_ALERT'])}",
        "",
        "# Límites / alertas",
        f"P1_MAX = {data.get('P1_MAX', DEFAULT_CONFIG['P1_MAX'])}",
        f"P2_MAX = {data.get('P2_MAX', DEFAULT_CONFIG['P2_MAX'])}",
        f"LVDT_MIN = {data.get('LVDT_MIN', DEFAULT_CONFIG['LVDT_MIN'])}",
        f"LVDT_MAX = {data.get('LVDT_MAX', DEFAULT_CONFIG['LVDT_MAX'])}",
        f"T1_MAX = {data.get('T1_MAX', DEFAULT_CONFIG['T1_MAX'])}",
        f"T2_MAX = {data.get('T2_MAX', DEFAULT_CONFIG['T2_MAX'])}",
        "",
        "# Calibración sensores auxiliares (conversión lineal RAW -> REAL)",
        "# Fórmula: REAL = REAL1 + (RAW - RAW1) * (REAL2 - REAL1) / (RAW2 - RAW1)",
        f"PRES1_PUNTO1_RAW = {data.get('PRES1_PUNTO1_RAW', DEFAULT_CONFIG['PRES1_PUNTO1_RAW'])}",
        f"PRES1_PUNTO2_RAW = {data.get('PRES1_PUNTO2_RAW', DEFAULT_CONFIG['PRES1_PUNTO2_RAW'])}",
        f"PRES1_PUNTO1_REAL = {data.get('PRES1_PUNTO1_REAL', DEFAULT_CONFIG['PRES1_PUNTO1_REAL'])}",
        f"PRES1_PUNTO2_REAL = {data.get('PRES1_PUNTO2_REAL', DEFAULT_CONFIG['PRES1_PUNTO2_REAL'])}",
        "",
        f"PRES2_PUNTO1_RAW = {data.get('PRES2_PUNTO1_RAW', DEFAULT_CONFIG['PRES2_PUNTO1_RAW'])}",
        f"PRES2_PUNTO2_RAW = {data.get('PRES2_PUNTO2_RAW', DEFAULT_CONFIG['PRES2_PUNTO2_RAW'])}",
        f"PRES2_PUNTO1_REAL = {data.get('PRES2_PUNTO1_REAL', DEFAULT_CONFIG['PRES2_PUNTO1_REAL'])}",
        f"PRES2_PUNTO2_REAL = {data.get('PRES2_PUNTO2_REAL', DEFAULT_CONFIG['PRES2_PUNTO2_REAL'])}",
        "",
        f"TEMP1_PUNTO1_RAW = {data.get('TEMP1_PUNTO1_RAW', DEFAULT_CONFIG['TEMP1_PUNTO1_RAW'])}",
        f"TEMP1_PUNTO2_RAW = {data.get('TEMP1_PUNTO2_RAW', DEFAULT_CONFIG['TEMP1_PUNTO2_RAW'])}",
        f"TEMP1_PUNTO1_REAL = {data.get('TEMP1_PUNTO1_REAL', DEFAULT_CONFIG['TEMP1_PUNTO1_REAL'])}",
        f"TEMP1_PUNTO2_REAL = {data.get('TEMP1_PUNTO2_REAL', DEFAULT_CONFIG['TEMP1_PUNTO2_REAL'])}",
        "",
        f"TEMP2_PUNTO1_RAW = {data.get('TEMP2_PUNTO1_RAW', DEFAULT_CONFIG['TEMP2_PUNTO1_RAW'])}",
        f"TEMP2_PUNTO2_RAW = {data.get('TEMP2_PUNTO2_RAW', DEFAULT_CONFIG['TEMP2_PUNTO2_RAW'])}",
        f"TEMP2_PUNTO1_REAL = {data.get('TEMP2_PUNTO1_REAL', DEFAULT_CONFIG['TEMP2_PUNTO1_REAL'])}",
        f"TEMP2_PUNTO2_REAL = {data.get('TEMP2_PUNTO2_REAL', DEFAULT_CONFIG['TEMP2_PUNTO2_REAL'])}",
        "",
    ]
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def load_config_file():
    data = DEFAULT_CONFIG.copy()

    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    s = line.strip()
                    if not s or s.startswith("#") or "=" not in s:
                        continue
                    key, value = s.split("=", 1)
                    data[key.strip()] = value.strip()
        except Exception:
            data = DEFAULT_CONFIG.copy()

    if not data.get("PHOTO_FOLDER"):
        data["PHOTO_FOLDER"] = get_default_session_preview_folder()

    try:
        os.makedirs(data["PHOTO_FOLDER"], exist_ok=True)
    except Exception:
        data["PHOTO_FOLDER"] = get_default_session_preview_folder()

    write_config_file(data)
    return data


APP_CONFIG = load_config_file()

SENSOR_IGNORE_SECONDS = config_get_float(APP_CONFIG, "SENSOR_IGNORE_SECONDS", SENSOR_IGNORE_SECONDS)
LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S = config_get_float(
    APP_CONFIG,
    "LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S",
    LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S
)
AUTO_RIGHT_INCREASES_LVDT = config_get_bool(APP_CONFIG, "AUTO_RIGHT_INCREASES_LVDT", False)
lvdt_offset = config_get_float(APP_CONFIG, "LVDT_ZERO_OFFSET_MM", lvdt_offset)
user_zero_applied = abs(lvdt_offset) > 1e-12

SENSOR_CALIBRATION = {
    "PRES1": {
        "P1_RAW": config_get_float(APP_CONFIG, "PRES1_PUNTO1_RAW", 0.0),
        "P2_RAW": config_get_float(APP_CONFIG, "PRES1_PUNTO2_RAW", 4095.0),
        "P1_REAL": config_get_float(APP_CONFIG, "PRES1_PUNTO1_REAL", 0.0),
        "P2_REAL": config_get_float(APP_CONFIG, "PRES1_PUNTO2_REAL", 250.0),
    },
    "PRES2": {
        "P1_RAW": config_get_float(APP_CONFIG, "PRES2_PUNTO1_RAW", 0.0),
        "P2_RAW": config_get_float(APP_CONFIG, "PRES2_PUNTO2_RAW", 4095.0),
        "P1_REAL": config_get_float(APP_CONFIG, "PRES2_PUNTO1_REAL", 0.0),
        "P2_REAL": config_get_float(APP_CONFIG, "PRES2_PUNTO2_REAL", 250.0),
    },
    "TEMP1": {
        "P1_RAW": config_get_float(APP_CONFIG, "TEMP1_PUNTO1_RAW", 0.0),
        "P2_RAW": config_get_float(APP_CONFIG, "TEMP1_PUNTO2_RAW", 4095.0),
        "P1_REAL": config_get_float(APP_CONFIG, "TEMP1_PUNTO1_REAL", 0.0),
        "P2_REAL": config_get_float(APP_CONFIG, "TEMP1_PUNTO2_REAL", 100.0),
    },
    "TEMP2": {
        "P1_RAW": config_get_float(APP_CONFIG, "TEMP2_PUNTO1_RAW", 0.0),
        "P2_RAW": config_get_float(APP_CONFIG, "TEMP2_PUNTO2_RAW", 4095.0),
        "P1_REAL": config_get_float(APP_CONFIG, "TEMP2_PUNTO1_REAL", 0.0),
        "P2_REAL": config_get_float(APP_CONFIG, "TEMP2_PUNTO2_REAL", 100.0),
    },
}

photo_folder = config_get_str(APP_CONFIG, "PHOTO_FOLDER", get_default_session_preview_folder())
try:
    os.makedirs(photo_folder, exist_ok=True)
except Exception:
    photo_folder = get_default_session_preview_folder()


def apply_photo_folder(folder, log_message=False):
    global photo_folder, last_photo_path
    if not folder:
        return
    photo_folder = folder
    try:
        photo_folder_var.set(photo_folder)
    except Exception:
        pass
    last_photo_path = None
    if log_message:
        console_print(f"Carpeta de fotos actualizada: {photo_folder}")
    if current_view == "foto":
        refresh_photo_preview(force=True)


def create_session_structure(base_folder=None):
    if not base_folder:
        base_folder = get_default_base_folder()
    session_name = datetime.now().strftime(SESSION_FOLDER_FORMAT)
    root_folder = os.path.join(base_folder, session_name)
    photos_folder = os.path.join(root_folder, "Fotos")
    data_folder = os.path.join(root_folder, "Datos")
    log_folder = os.path.join(root_folder, "Log")
    os.makedirs(photos_folder, exist_ok=True)
    os.makedirs(data_folder, exist_ok=True)
    os.makedirs(log_folder, exist_ok=True)
    return root_folder, photos_folder, data_folder, log_folder


def close_session_files():
    global log_handle, console_log_handle
    global session_root_folder, session_data_folder, session_log_folder, session_photo_folder
    if log_handle:
        try:
            log_handle.flush()
            log_handle.close()
        except Exception:
            pass
        log_handle = None
    if console_log_handle:
        try:
            console_log_handle.flush()
            console_log_handle.close()
        except Exception:
            pass
        console_log_handle = None

    session_root_folder = None
    session_data_folder = None
    session_log_folder = None
    session_photo_folder = None


def prepare_automatic_session(base_folder):
    global session_root_folder, session_data_folder, session_log_folder, session_photo_folder
    global log_handle, console_log_handle

    close_session_files()

    root_folder, photos_folder, data_folder, log_folder = create_session_structure(base_folder)
    session_root_folder = root_folder
    session_photo_folder = photos_folder
    session_data_folder = data_folder
    session_log_folder = log_folder

    data_path = os.path.join(data_folder, datetime.now().strftime("registro_%Y%m%d_%H%M%S.txt"))
    console_path = os.path.join(log_folder, "log.txt")

    console_log_handle = open(console_path, "w", encoding="utf-8")
    log_handle = open(data_path, "w", encoding="utf-8")

    create_graph_custom_file(data_folder)
    apply_photo_folder(photos_folder, log_message=False)

    return {
        "root": root_folder,
        "photos": photos_folder,
        "data": data_folder,
        "log": log_folder,
        "data_file": data_path,
        "console_file": console_path,
    }


def set_motion_led(state):
    state_map = {
        "izquierda": LED_PIN_IZQUIERDA,
        "detenido": LED_PIN_DETENIDO,
        "derecha": LED_PIN_DERECHA,
    }
    active_pin = state_map.get(state)
    for pin in LED_PINS:
        GPIO.output(pin, GPIO.HIGH if pin == active_pin else GPIO.LOW)


last_photo_path = None
camera_imgtk = None

# Gráfico compartido por manual/automático usando archivos
GRAPH_TEMP_FILE = os.path.join(os.getcwd(), "_grafico_temporal.csv")
graph_custom_file = None
graph_source_mode = "temp"   # temp | custom
graph_write_lock = threading.Lock()
last_graph_write = 0.0
graph_display_paused = False

# ─────────────────────────────────────────────────────────────
# Utilidades base
# ─────────────────────────────────────────────────────────────
def register_console_widget(widget):
    console_widgets.append(widget)


def console_print(message):
    for widget in list(console_widgets):
        try:
            widget.config(state=tk.NORMAL)
            widget.insert(tk.END, message + "\n")
            widget.see(tk.END)
            widget.config(state=tk.DISABLED)
        except Exception:
            pass

    if console_log_handle:
        with console_log_lock:
            try:
                console_log_handle.write(message + "\n")
                console_log_handle.flush()
            except Exception:
                pass


def pick_serial_port():
    if SER_PORT:
        return SER_PORT
    by_id = "/dev/serial/by-id"
    try:
        if os.path.isdir(by_id):
            for name in os.listdir(by_id):
                if "usb" in name.lower() or "arduino" in name.lower() or "sam" in name.lower():
                    return os.path.join(by_id, name)
    except Exception:
        pass
    for p in serial.tools.list_ports.comports():
        if ("ACM" in p.device) or ("USB" in p.device):
            return p.device
    return "/dev/ttyACM0"


def serial_reader():
    global _latest_lvdt_raw, _latest_pres1_raw, _latest_pres2_raw, _latest_temp1_raw, _latest_temp2_raw
    while True:
        port = pick_serial_port()
        try:
            with serial.Serial(port, SER_BAUD, timeout=SER_TIMEOUT) as ser:
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                while True:
                    line = ser.readline()
                    if not line:
                        continue
                    try:
                        s = line.decode("utf-8", errors="replace").strip()
                        if not s:
                            continue
                        parts = [p.strip() for p in s.split(",")]

                        if len(parts) == 1:
                            _latest_lvdt_raw = float(parts[0].replace(",", "."))
                            continue

                        if len(parts) >= 5:
                            _latest_lvdt_raw = float(parts[0].replace(",", "."))
                            _latest_pres1_raw = int(float(parts[1]))
                            _latest_pres2_raw = int(float(parts[2]))
                            _latest_temp1_raw = int(float(parts[3]))
                            _latest_temp2_raw = int(float(parts[4]))
                            continue
                    except ValueError:
                        continue
        except (serial.SerialException, FileNotFoundError):
            time.sleep(1.0)


def sensors_in_ignore_window():
    return time.time() < sensor_ignore_until


def begin_sensor_ignore_window(duration=None):
    global sensor_ignore_until
    ignore_seconds = SENSOR_IGNORE_SECONDS if duration is None else duration
    sensor_ignore_until = max(sensor_ignore_until, time.time() + ignore_seconds)


def get_effective_sensor_snapshot():
    return _latest_lvdt_raw, _latest_pres1_raw, _latest_pres2_raw, _latest_temp1_raw, _latest_temp2_raw


def get_lvdt_effective_mm():
    global _lvdt_effective_mm, _lvdt_effective_time

    raw_mm = None if _latest_lvdt_raw is None else raw_to_mm(_latest_lvdt_raw)
    now = time.time()

    with lvdt_filter_lock:
        if raw_mm is None:
            if _lvdt_effective_mm is None:
                return 0.0
            return _lvdt_effective_mm

        if _lvdt_effective_mm is None or _lvdt_effective_time is None:
            _lvdt_effective_mm = raw_mm
            _lvdt_effective_time = now
            return _lvdt_effective_mm

        dt = now - _lvdt_effective_time
        if dt < 0:
            dt = 0.0

        if sensors_in_ignore_window():
            max_delta = LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S * dt
            if max_delta <= 0:
                filtered_mm = _lvdt_effective_mm
            else:
                delta = raw_mm - _lvdt_effective_mm
                if delta > max_delta:
                    delta = max_delta
                elif delta < -max_delta:
                    delta = -max_delta
                filtered_mm = _lvdt_effective_mm + delta
        else:
            filtered_mm = raw_mm

        _lvdt_effective_mm = filtered_mm
        _lvdt_effective_time = now
        return _lvdt_effective_mm


def get_lvdt_display(offset_ref=None):
    base_mm = get_lvdt_effective_mm()
    off = lvdt_offset if offset_ref is None else offset_ref
    return base_mm - off


def get_auto_desired_direction(current, target):
    if current < (target - TOL_MM):
        return "derecha" if AUTO_RIGHT_INCREASES_LVDT else "izquierda"
    return "izquierda" if AUTO_RIGHT_INCREASES_LVDT else "derecha"


def calculate_velocity(pos):
    t = time.time()
    _pos_buffer.append(pos)
    _time_buffer.append(t)

    if len(_pos_buffer) > 5:
        _pos_buffer.pop(0)
        _time_buffer.pop(0)

    if len(_pos_buffer) >= 2:
        dt = _time_buffer[-1] - _time_buffer[0]
        vel = (_pos_buffer[-1] - _pos_buffer[0]) / dt if dt > 0 else 0
    else:
        vel = 0
    return -vel


def apply_linear_calibration(raw_value, cfg):
    if raw_value is None:
        return None
    try:
        raw1 = float(cfg["P1_RAW"])
        raw2 = float(cfg["P2_RAW"])
        real1 = float(cfg["P1_REAL"])
        real2 = float(cfg["P2_REAL"])
    except Exception:
        return float(raw_value)

    if abs(raw2 - raw1) < 1e-12:
        return float(raw_value)

    return real1 + (float(raw_value) - raw1) * (real2 - real1) / (raw2 - raw1)


def get_calibrated_sensor_value(sensor_key, raw_value):
    cfg = SENSOR_CALIBRATION.get(sensor_key)
    if cfg is None:
        return None if raw_value is None else float(raw_value)
    return apply_linear_calibration(raw_value, cfg)


def format_sensor_value(value):
    if value is None:
        return "—"
    return f"{value:.2f}"


def get_entry_value(entry, default=""):
    try:
        return entry.get().strip()
    except Exception:
        return default


def set_entry_value(entry, value):
    try:
        entry.delete(0, tk.END)
        entry.insert(0, str(value))
    except Exception:
        pass


def collect_current_config():
    data = DEFAULT_CONFIG.copy()

    data["SENSOR_IGNORE_SECONDS"] = f"{SENSOR_IGNORE_SECONDS}"
    data["LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S"] = f"{LVDT_RELAY_FILTER_MAX_RATE_MM_PER_S}"
    data["AUTO_RIGHT_INCREASES_LVDT"] = "1" if AUTO_RIGHT_INCREASES_LVDT else "0"
    data["LVDT_ZERO_OFFSET_MM"] = f"{lvdt_offset:.6f}"
    data["PHOTO_FOLDER"] = photo_folder

    if "photo_auto_var" in globals():
        data["PHOTO_AUTO_ENABLED"] = "1" if photo_auto_var.get() else "0"
    if "keyboard_enabled_var" in globals():
        data["KEYBOARD_ENABLED"] = "1" if keyboard_enabled_var.get() else "0"
    if "manual_graph_window_var" in globals():
        data["MANUAL_GRAPH_WINDOW"] = manual_graph_window_var.get()
    if "auto_graph_window_var" in globals():
        data["AUTO_GRAPH_WINDOW"] = auto_graph_window_var.get()
    if "var_guardar" in globals():
        data["AUTO_GUARDAR"] = "1" if var_guardar.get() else "0"
    if "cfg_stop_alert_var" in globals():
        data["CFG_STOP_ALERT"] = "1" if cfg_stop_alert_var.get() else "0"

    if "entry_photo_interval" in globals():
        data["PHOTO_INTERVAL_MIN"] = get_entry_value(entry_photo_interval, DEFAULT_CONFIG["PHOTO_INTERVAL_MIN"])
    if "entry_despl" in globals():
        data["MANUAL_DISPLACEMENT_MM"] = get_entry_value(entry_despl, DEFAULT_CONFIG["MANUAL_DISPLACEMENT_MM"])
    if "entry_p1max" in globals():
        data["P1_MAX"] = get_entry_value(entry_p1max, DEFAULT_CONFIG["P1_MAX"])
    if "entry_p2max" in globals():
        data["P2_MAX"] = get_entry_value(entry_p2max, DEFAULT_CONFIG["P2_MAX"])
    if "entry_lvdtmin" in globals():
        data["LVDT_MIN"] = get_entry_value(entry_lvdtmin, DEFAULT_CONFIG["LVDT_MIN"])
    if "entry_lvdtmax" in globals():
        data["LVDT_MAX"] = get_entry_value(entry_lvdtmax, DEFAULT_CONFIG["LVDT_MAX"])
    if "entry_t1max" in globals():
        data["T1_MAX"] = get_entry_value(entry_t1max, DEFAULT_CONFIG["T1_MAX"])
    if "entry_t2max" in globals():
        data["T2_MAX"] = get_entry_value(entry_t2max, DEFAULT_CONFIG["T2_MAX"])

    data["PRES1_PUNTO1_RAW"] = f"{SENSOR_CALIBRATION['PRES1']['P1_RAW']}"
    data["PRES1_PUNTO2_RAW"] = f"{SENSOR_CALIBRATION['PRES1']['P2_RAW']}"
    data["PRES1_PUNTO1_REAL"] = f"{SENSOR_CALIBRATION['PRES1']['P1_REAL']}"
    data["PRES1_PUNTO2_REAL"] = f"{SENSOR_CALIBRATION['PRES1']['P2_REAL']}"

    data["PRES2_PUNTO1_RAW"] = f"{SENSOR_CALIBRATION['PRES2']['P1_RAW']}"
    data["PRES2_PUNTO2_RAW"] = f"{SENSOR_CALIBRATION['PRES2']['P2_RAW']}"
    data["PRES2_PUNTO1_REAL"] = f"{SENSOR_CALIBRATION['PRES2']['P1_REAL']}"
    data["PRES2_PUNTO2_REAL"] = f"{SENSOR_CALIBRATION['PRES2']['P2_REAL']}"

    data["TEMP1_PUNTO1_RAW"] = f"{SENSOR_CALIBRATION['TEMP1']['P1_RAW']}"
    data["TEMP1_PUNTO2_RAW"] = f"{SENSOR_CALIBRATION['TEMP1']['P2_RAW']}"
    data["TEMP1_PUNTO1_REAL"] = f"{SENSOR_CALIBRATION['TEMP1']['P1_REAL']}"
    data["TEMP1_PUNTO2_REAL"] = f"{SENSOR_CALIBRATION['TEMP1']['P2_REAL']}"

    data["TEMP2_PUNTO1_RAW"] = f"{SENSOR_CALIBRATION['TEMP2']['P1_RAW']}"
    data["TEMP2_PUNTO2_RAW"] = f"{SENSOR_CALIBRATION['TEMP2']['P2_RAW']}"
    data["TEMP2_PUNTO1_REAL"] = f"{SENSOR_CALIBRATION['TEMP2']['P1_REAL']}"
    data["TEMP2_PUNTO2_REAL"] = f"{SENSOR_CALIBRATION['TEMP2']['P2_REAL']}"

    return data


def save_runtime_config():
    try:
        write_config_file(collect_current_config())
    except Exception:
        pass


def periodic_save_config():
    save_runtime_config()
    root.after(2000, periodic_save_config)


def on_app_close():
    save_runtime_config()
    stop_auto_photo_scheduler()
    close_session_files()
    try:
        GPIO.cleanup()
    except Exception:
        pass
    root.destroy()


def get_ip_for_interface(interface_name):
    try:
        proc = subprocess.run(
            ["ip", "-4", "addr", "show", "dev", interface_name],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if proc.returncode != 0:
            return "No disponible"
        for line in proc.stdout.splitlines():
            line = line.strip()
            if line.startswith("inet "):
                return line.split()[1].split("/")[0]
    except Exception:
        pass
    return "No disponible"


def refresh_system_network_info():
    try:
        lbl_wifi_ip_value.config(text=get_ip_for_interface("wlan0"))
        lbl_eth_ip_value.config(text=get_ip_for_interface("eth0"))
    except Exception:
        pass


def apply_squeekboard_state(enabled):
    command = [
        "sudo",
        "raspi-config",
        "nonint",
        "do_squeekboard",
        "S1" if enabled else "S3",
    ]

    def worker():
        try:
            proc = subprocess.run(command, capture_output=True, text=True, timeout=90)
            if proc.returncode == 0:
                msg_text = "Teclado en pantalla activado." if enabled else "Teclado en pantalla desactivado."
                root.after(0, lambda m=msg_text: console_print(m))
            else:
                prev = not enabled
                root.after(0, lambda v=prev: keyboard_enabled_var.set(v))
                stderr = (proc.stderr or proc.stdout or "").strip()
                root.after(0, lambda e=stderr: console_print(f"No se pudo cambiar el teclado en pantalla: {e or 'sin detalle'}"))
        except Exception as e:
            prev = not enabled
            root.after(0, lambda v=prev: keyboard_enabled_var.set(v))
            root.after(0, lambda err=str(e): console_print(f"Error al cambiar teclado en pantalla: {err}"))

    threading.Thread(target=worker, daemon=True).start()


def on_toggle_keyboard():
    save_runtime_config()
    apply_squeekboard_state(bool(keyboard_enabled_var.get()))

# ─────────────────────────────────────────────────────────────
# Archivos de gráfico
# ─────────────────────────────────────────────────────────────
def ensure_file_exists(path):
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder, exist_ok=True)
    if not os.path.exists(path):
        with open(path, "w", encoding="utf-8"):
            pass


def reset_temp_graph_file():
    with graph_write_lock:
        with open(GRAPH_TEMP_FILE, "w", encoding="utf-8"):
            pass


def set_graph_source_temp(clear_temp=False):
    global graph_source_mode, graph_custom_file
    graph_source_mode = "temp"
    graph_custom_file = None
    if clear_temp:
        reset_temp_graph_file()


def create_graph_custom_file(base_folder=None):
    global graph_source_mode, graph_custom_file
    if not base_folder:
        base_folder = get_default_base_folder()
    os.makedirs(base_folder, exist_ok=True)
    graph_custom_file = os.path.join(
        base_folder,
        datetime.now().strftime("grafico_%Y%m%d_%H%M%S.csv")
    )
    with open(graph_custom_file, "w", encoding="utf-8"):
        pass
    graph_source_mode = "custom"
    return graph_custom_file


def write_graph_sample(path, t_epoch, pos_mm):
    ensure_file_exists(path)
    with graph_write_lock:
        with open(path, "a", encoding="utf-8") as f:
            f.write(f"{t_epoch:.6f},{pos_mm:.6f}\n")


def append_plot_sample_to_files(pos):
    global last_graph_write
    now = time.time()
    if (now - last_graph_write) < 0.15:
        return

    write_graph_sample(GRAPH_TEMP_FILE, now, pos)
    if graph_source_mode == "custom" and graph_custom_file:
        write_graph_sample(graph_custom_file, now, pos)

    last_graph_write = now


def get_graph_source_path():
    if graph_source_mode == "custom" and graph_custom_file and os.path.exists(graph_custom_file):
        return graph_custom_file
    return GRAPH_TEMP_FILE


def reset_graph_action():
    set_graph_source_temp(clear_temp=True)
    console_print("Gráfica reiniciada. Fuente temporal restablecida.")
    if not graph_display_paused:
        redraw_shared_graph()


def tail_lines(path, n_lines=2000, block_size=4096):
    if not os.path.exists(path):
        return []
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        file_size = f.tell()
        data = b""
        lines_found = 0
        pos = file_size

        while pos > 0 and lines_found <= n_lines:
            read_size = block_size if pos >= block_size else pos
            pos -= read_size
            f.seek(pos)
            chunk = f.read(read_size)
            data = chunk + data
            lines_found = data.count(b"\n")

        text = data.decode("utf-8", errors="replace")
        lines = text.splitlines()
        if len(lines) > n_lines:
            lines = lines[-n_lines:]
        return lines


def read_all_lines_locked(path):
    if not os.path.exists(path):
        return []
    with graph_write_lock:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().splitlines()


def get_seconds_from_label(label, default=30):
    mapping = {
        "15 s": 15,
        "30 s": 30,
        "1 min": 60,
        "2 min": 120,
        "5 min": 300,
        "10 min": 600,
        "Todo": None,
    }
    return mapping.get(label, default)


def read_graph_series(window_label="30 s", since_auto=False):
    path = get_graph_source_path()
    if not os.path.exists(path):
        return [], []

    window_seconds = get_seconds_from_label(window_label, 30)

    if window_seconds is None:
        lines = read_all_lines_locked(path)
    else:
        if window_seconds <= 15:
            max_lines = 220
        elif window_seconds <= 30:
            max_lines = 420
        elif window_seconds <= 60:
            max_lines = 800
        elif window_seconds <= 120:
            max_lines = 1600
        elif window_seconds <= 300:
            max_lines = 3500
        else:
            max_lines = 7000
        lines = tail_lines(path, n_lines=max_lines)

    raw_t = []
    raw_y = []
    for line in lines:
        parts = line.strip().split(",")
        if len(parts) != 2:
            continue
        try:
            t_epoch = float(parts[0])
            y = float(parts[1])
        except ValueError:
            continue
        raw_t.append(t_epoch)
        raw_y.append(y)

    if not raw_t:
        return [], []

    end_t = raw_t[-1]
    start_t = raw_t[0]

    if since_auto and auto_run_epoch is not None:
        start_t = max(start_t, auto_run_epoch)

    if window_seconds is not None:
        start_t = max(start_t, end_t - window_seconds)

    xs = []
    ys = []
    for t_epoch, y in zip(raw_t, raw_y):
        if t_epoch >= start_t:
            xs.append(t_epoch - start_t)
            ys.append(y)
    return xs, ys

# ─────────────────────────────────────────────────────────────
# Relés / pulsos
# ─────────────────────────────────────────────────────────────
def cambiar_estado_rele(num_rele, estado):
    if num_rele < 1 or num_rele > 4:
        return
    pin = rele_pin[num_rele - 1]
    GPIO.output(pin, GPIO.LOW if estado.lower() == "on" else GPIO.HIGH)


def _pulse_worker(num_rele, seconds, btn_to_disable=None):
    try:
        if btn_to_disable is not None:
            btn_to_disable.config(state=tk.DISABLED)
        cambiar_estado_rele(num_rele, "on")
        time.sleep(seconds)
    finally:
        cambiar_estado_rele(num_rele, "off")
        if btn_to_disable is not None:
            btn_to_disable.config(state=tk.NORMAL)


def pulse_channel(num_rele, seconds=PULSE_SECONDS, btn_to_disable=None):
    begin_sensor_ignore_window()
    t = threading.Thread(target=_pulse_worker, args=(num_rele, seconds, btn_to_disable), daemon=True)
    t.start()


def pulse_start_and_dir(direction):
    begin_sensor_ignore_window()
    def worker():
        cambiar_estado_rele(CH_INICIO, "on")
        if direction == "derecha":
            cambiar_estado_rele(CH_DERECHA, "on")
        else:
            cambiar_estado_rele(CH_IZQUIERDA, "on")
        time.sleep(PULSE_SECONDS)
        cambiar_estado_rele(CH_INICIO, "off")
        if direction == "derecha":
            cambiar_estado_rele(CH_DERECHA, "off")
        else:
            cambiar_estado_rele(CH_IZQUIERDA, "off")
    threading.Thread(target=worker, daemon=True).start()


def pulse_direction(direction):
    if direction == "derecha":
        pulse_channel(CH_DERECHA, PULSE_SECONDS)
    else:
        pulse_channel(CH_IZQUIERDA, PULSE_SECONDS)

# ─────────────────────────────────────────────────────────────
# Helpers UI
# ─────────────────────────────────────────────────────────────
def set_dir_buttons_color(active=None):
    bg_default = BTN_LIGHT
    if active == "izquierda":
        btn_izq.config(bg=BTN_GREEN, fg="white", activebackground=BTN_GREEN)
        btn_der.config(bg=bg_default, fg=TEXT_MAIN, activebackground=bg_default)
    elif active == "derecha":
        btn_der.config(bg=BTN_GREEN, fg="white", activebackground=BTN_GREEN)
        btn_izq.config(bg=bg_default, fg=TEXT_MAIN, activebackground=bg_default)
    else:
        btn_izq.config(bg=bg_default, fg=TEXT_MAIN, activebackground=bg_default)
        btn_der.config(bg=bg_default, fg=TEXT_MAIN, activebackground=bg_default)


def set_manual_buttons_enabled(enabled: bool):
    state = tk.NORMAL if enabled else tk.DISABLED
    btn_izq.config(state=state)
    btn_der.config(state=state)
    btn_det.config(state=state)
    entry_despl.config(state=state)


def do_stop():
    global sistema_iniciado, msg
    if sistema_iniciado:
        pulse_channel(CH_DETENER, PULSE_SECONDS, btn_to_disable=btn_det)
        sistema_iniciado = False
    set_motion_led("detenido")
    msg = "Detenido"
    console_print(msg)
    lbl_estado_grande.config(text=msg)
    set_dir_buttons_color(None)


def update_graph_pause_buttons():
    txt = "Reanudar" if graph_display_paused else "Pausar"
    try:
        btn_manual_graph_pause.config(text=txt)
    except Exception:
        pass
    try:
        btn_auto_graph_pause.config(text=txt)
    except Exception:
        pass


def set_graph_display_pause_state(paused):
    global graph_display_paused
    graph_display_paused = paused
    update_graph_pause_buttons()
    if not graph_display_paused and shared_center_mode == "graph":
        redraw_shared_graph()


def toggle_graph_display_pause():
    if graph_display_paused:
        set_graph_display_pause_state(False)
        console_print("Visualización del gráfico reanudada.")
    else:
        set_graph_display_pause_state(True)
        console_print("Visualización del gráfico pausada.")

# ─────────────────────────────────────────────────────────────
# Lógica de movimiento
# ─────────────────────────────────────────────────────────────
def move_relative_blocking(delta_mm, part_of_auto=False, offset_ref=None, forced_direction=None):
    global msg, sistema_iniciado, dir_actual, manual_override

    delta_abs = abs(delta_mm)
    if delta_abs < 1e-9:
        console_print("Desplazamiento = 0 mm (nada que mover)")
        return

    if offset_ref is None:
        offset_ref = lvdt_offset

    if forced_direction is not None:
        direction = forced_direction
    else:
        direction = "derecha" if delta_mm > 0 else "izquierda"

    set_dir_buttons_color(direction)
    dir_actual = direction
    manual_override = False

    pulse_start_and_dir(direction)
    set_motion_led(direction)
    sistema_iniciado = True
    msg = f"Iniciado (mov. {direction})"
    console_print(msg)
    lbl_estado_grande.config(text=msg)

    start_display = get_lvdt_display(offset_ref)
    console_print(f"Moviendo a la {direction} {delta_abs:.2f} mm (distancia absoluta)")

    last_log_t = 0.0
    while not stop_move.is_set():
        pos_disp = get_lvdt_display(offset_ref)
        dist_abs = abs(pos_disp - start_display)
        if (not manual_override) and (dist_abs >= (delta_abs - TOL_MM)):
            break
        if log_handle:
            t = time.time()
            if t - last_log_t >= 0.1:
                vel = calculate_velocity(pos_disp)
                log_handle.write(f"#DBG {t:.3f} {pos_disp:.3f} {vel:.3f}\n")
                last_log_t = t
        time.sleep(0.02)

    do_stop()


def auto_logger_100hz(routine_offset_ref):
    global log_handle
    if log_handle is None:
        return
    start = time.perf_counter()
    paused_accum = 0.0
    pause_started = None
    next_t = start
    flush_count = 0

    while auto_running and not stop_move.is_set():
        now = time.perf_counter()
        if now < next_t:
            time.sleep(min(0.001, next_t - now))
            continue

        if auto_paused.is_set():
            if pause_started is None:
                pause_started = now
            next_t += 0.01
            continue
        else:
            if pause_started is not None:
                paused_accum += (now - pause_started)
                pause_started = None

        elapsed = now - start - paused_accum
        mins = int(elapsed // 60)
        secs = int(elapsed % 60)
        ms = int((elapsed - int(elapsed)) * 1000)
        pos = get_lvdt_display(routine_offset_ref)
        log_handle.write(f"{mins:02d}:{secs:02d}.{ms:03d} {pos:.3f}\n")
        flush_count += 1
        if flush_count >= 100:
            try:
                log_handle.flush()
            except Exception:
                pass
            flush_count = 0
        next_t += 0.01

    try:
        log_handle.flush()
    except Exception:
        pass


def auto_streaming_run(routine_offset_ref):
    global sistema_iniciado, dir_actual, auto_running

    idx = 0
    n = len(disp_list)
    if n == 0:
        return

    while idx < n and not stop_move.is_set():
        if auto_paused.is_set():
            if sistema_iniciado:
                pulse_channel(CH_DETENER, PULSE_SECONDS)
                sistema_iniciado = False
                set_motion_led("detenido")
            while auto_paused.is_set() and not stop_move.is_set():
                time.sleep(0.05)

        current = get_lvdt_display(routine_offset_ref)
        target = disp_list[idx]

        if abs(current - target) <= TOL_MM:
            idx += 1
            continue

        desired_dir = get_auto_desired_direction(current, target)

        if not sistema_iniciado:
            pulse_start_and_dir(desired_dir)
            sistema_iniciado = True
            set_motion_led(desired_dir)
        elif dir_actual != desired_dir:
            pulse_direction(desired_dir)
            set_motion_led(desired_dir)

        dir_actual = desired_dir
        set_dir_buttons_color(desired_dir)
        msg_text = f"Automático: yendo a {target:.2f} mm ({desired_dir})"
        root.after(0, lambda m=msg_text: lbl_estado_grande.config(text=m))

        while not stop_move.is_set():
            if auto_paused.is_set():
                break
            pos = get_lvdt_display(routine_offset_ref)
            if abs(pos - target) <= TOL_MM:
                idx += 1
                break
            desired_dir2 = get_auto_desired_direction(pos, target)
            if desired_dir2 != dir_actual:
                pulse_direction(desired_dir2)
                dir_actual = desired_dir2
                set_dir_buttons_color(dir_actual)
                set_motion_led(dir_actual)
            time.sleep(0.01)

    do_stop()

# ─────────────────────────────────────────────────────────────
# Foto
# ─────────────────────────────────────────────────────────────
def get_latest_photo(folder):
    if not folder or not os.path.isdir(folder):
        return None
    files = []
    for name in os.listdir(folder):
        path = os.path.join(folder, name)
        if os.path.isfile(path) and name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp")):
            files.append(path)
    if not files:
        return None
    files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return files[0]


def render_photo(path):
    global camera_imgtk
    if path is None:
        lbl_photo_preview.config(image="", text="Sin imagen", compound="center", bg="#f2f4f7")
        lbl_photo_name.config(text="Última foto: —")
        lbl_photo_time.config(text="Hora: —")
        return
    try:
        img = Image.open(path)
        img.thumbnail((470, 230), Image.LANCZOS)
        camera_imgtk = ImageTk.PhotoImage(img)
        lbl_photo_preview.config(image=camera_imgtk, text="", bg="#f2f4f7")
        lbl_photo_name.config(text=f"Última foto: {os.path.basename(path)}")
        ts = datetime.fromtimestamp(os.path.getmtime(path)).strftime("%d/%m/%Y %H:%M:%S")
        lbl_photo_time.config(text=f"Hora: {ts}")
    except Exception:
        lbl_photo_preview.config(image="", text="Error al cargar imagen", compound="center", bg="#f2f4f7")
        lbl_photo_name.config(text=f"Última foto: {os.path.basename(path)}")
        lbl_photo_time.config(text="Hora: —")


def refresh_photo_preview(force=False):
    global last_photo_path
    latest = get_latest_photo(photo_folder)
    if force or latest != last_photo_path:
        last_photo_path = latest
        render_photo(latest)


def monitor_latest_photo():
    try:
        if current_view == "foto":
            refresh_photo_preview(force=False)
    except Exception:
        pass
    root.after(PHOTO_MONITOR_MS, monitor_latest_photo)


def choose_photo_folder():
    folder = filedialog.askdirectory(initialdir=photo_folder if os.path.isdir(photo_folder) else get_default_base_folder())
    if not folder:
        return
    apply_photo_folder(folder, log_message=True)
    save_runtime_config()
    refresh_photo_preview(force=True)


def get_photo_interval_seconds():
    try:
        val = float(entry_photo_interval.get().replace(",", "."))
        if val <= 0:
            raise ValueError
        return val * 60.0
    except Exception:
        return 30.0


def set_photo_status(message, color=TEXT_MUTED, busy=False):
    global photo_capture_active, photo_status_message, photo_status_color
    photo_capture_active = busy
    photo_status_message = message
    photo_status_color = color

    try:
        lbl_photo_status.config(text=message, fg=color)
    except Exception:
        pass

    try:
        if busy:
            btn_photo_manual.config(
                text="Tomando...",
                bg=BTN_ORANGE,
                fg="white",
                activebackground=BTN_ORANGE,
                state=tk.DISABLED
            )
        else:
            btn_photo_manual.config(
                text="FOTO",
                bg=BTN_LIGHT,
                fg=TEXT_MAIN,
                activebackground=BTN_LIGHT,
                state=tk.NORMAL
            )
    except Exception:
        pass


def enqueue_photo_capture(source="manual"):
    folder = photo_folder if photo_folder else get_default_session_preview_folder()
    os.makedirs(folder, exist_ok=True)
    photo_capture_queue.put({"source": source, "folder": folder})
    root.after(0, lambda s=source: set_photo_status(f"Captura {s} solicitada…", BTN_BLUE, busy=False))
    if source == "manual":
        console_print(f"Captura manual solicitada. Carpeta destino: {folder}")


def build_photo_filepath(folder):
    elapsed = get_elapsed_automatic_time_for_file()
    if auto_running and elapsed is not None:
        filename = format_elapsed_photo_filename(elapsed)
        return os.path.join(folder, filename), None

    ts = datetime.now()
    filename = ts.strftime("%Y-%m-%d_%H-%M-%S") + f"-{int(ts.microsecond / 1000):03d}.jpg"
    return os.path.join(folder, filename), ts


def capture_photo_now(folder, source="manual"):
    os.makedirs(folder, exist_ok=True)
    filepath, ts = build_photo_filepath(folder)
    root.after(0, lambda s=source: set_photo_status(f"Tomando foto {s}…", BTN_ORANGE, busy=True))

    try:
        subprocess.run(
            ["sudo", "pkill", "-f", "gvfs"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
            check=False,
        )
    except Exception:
        pass

    try:
        proc = subprocess.run(
            [
                "gphoto2",
                "--capture-image-and-download",
                "--force-overwrite",
                "--filename",
                filepath,
            ],
            capture_output=True,
            text=True,
            timeout=180,
        )

        if proc.returncode == 0 and os.path.exists(filepath):
            msg_text = f"Foto {source} guardada: {filepath}"
            root.after(0, lambda m=msg_text: console_print(m))
            root.after(0, lambda p=filepath: set_photo_status(f"Nueva foto: {p}", BTN_GREEN, busy=False))
            if current_view == "foto":
                root.after(0, lambda: refresh_photo_preview(force=True))
            return True

        stderr = (proc.stderr or proc.stdout or "").strip()
        root.after(0, lambda e=stderr: console_print(f"Error al tomar foto ({source}): {e or 'sin detalle'}"))
        root.after(0, lambda e=stderr: set_photo_status(f"Error al tomar foto: {e or 'sin detalle'}", BTN_RED, busy=False))
        return False
    except Exception as e:
        root.after(0, lambda err=str(e): console_print(f"Error al tomar foto ({source}): {err}"))
        root.after(0, lambda err=str(e): set_photo_status(f"Error al tomar foto: {err}", BTN_RED, busy=False))
        return False


def photo_capture_worker():
    while True:
        request = photo_capture_queue.get()
        if request is None:
            photo_capture_queue.task_done()
            break
        try:
            capture_photo_now(request.get("folder") or photo_folder, request.get("source", "manual"))
        finally:
            photo_capture_queue.task_done()


def manual_photo_capture():
    enqueue_photo_capture("manual")


def photo_scheduler_worker():
    while not photo_scheduler_stop.is_set():
        interval_s = get_photo_interval_seconds()
        if photo_scheduler_stop.wait(interval_s):
            break
        enqueue_photo_capture("automática")


def start_auto_photo_scheduler_if_needed():
    global photo_scheduler_thread
    stop_auto_photo_scheduler()
    if not photo_auto_var.get():
        return
    photo_scheduler_stop.clear()
    photo_scheduler_thread = threading.Thread(target=photo_scheduler_worker, daemon=True)
    photo_scheduler_thread.start()
    console_print("Programador de fotos automáticas iniciado.")


def stop_auto_photo_scheduler():
    global photo_scheduler_thread
    photo_scheduler_stop.set()
    photo_scheduler_thread = None

# ─────────────────────────────────────────────────────────────
# Gráfico compartido
# ─────────────────────────────────────────────────────────────
def redraw_shared_graph():
    if not MATPLOTLIB_OK or shared_ax is None:
        return

    if current_view == "manual":
        window_label = manual_graph_window_var.get()
        title = "LVDT en vivo"
        since_auto = False
    else:
        window_label = auto_graph_window_var.get()
        title = "LVDT vs tiempo"
        since_auto = True

    xs, ys = read_graph_series(window_label=window_label, since_auto=since_auto)

    shared_ax.clear()
    shared_ax.set_title(title, fontsize=9)
    shared_ax.set_xlabel("Tiempo [s]", fontsize=8)
    shared_ax.set_ylabel("LVDT [mm]", fontsize=8)
    shared_ax.grid(True, alpha=0.35)

    if xs:
        shared_ax.plot(xs, ys, linewidth=1.3)
        shared_ax.set_xlim(left=0, right=max(xs[-1], 1))
    else:
        shared_ax.text(0.5, 0.5, "Sin datos", ha="center", va="center", transform=shared_ax.transAxes)

    shared_fig.tight_layout(pad=0.8)
    shared_canvas.draw_idle()


def periodic_graph_refresh():
    try:
        if current_view in ("manual", "automatico") and shared_center_mode == "graph" and not graph_display_paused:
            redraw_shared_graph()
    except Exception:
        pass
    root.after(650, periodic_graph_refresh)

# ─────────────────────────────────────────────────────────────
# Navegación
# ─────────────────────────────────────────────────────────────
def toggle_shared_center_mode():
    if shared_center_mode == "graph":
        set_shared_center_mode("messages")
    else:
        set_shared_center_mode("graph")


def show_view(view_name):
    global current_view
    current_view = view_name

    for frame in view_frames.values():
        frame.place_forget()

    if view_name in ("manual", "automatico", "foto", "configuracion", "sistema"):
        view_frames[view_name].place(x=MAIN_X, y=MAIN_Y, width=MAIN_W, height=MAIN_H)

    if view_name in ("manual", "automatico"):
        shared_center_frame.place(
            x=MAIN_X + 8,
            y=MAIN_Y + TOP_SECTION_H + 6,
            width=SHARED_AREA_W,
            height=SHARED_AREA_H
        )
    else:
        shared_center_frame.place_forget()

    lbl_mode_header.config(text=view_titles[view_name])

    for key, btn in nav_buttons.items():
        if key == view_name:
            btn.config(bg=TAB_ACTIVE_BG, fg="white", activebackground=BTN_BLUE_DARK)
        else:
            btn.config(bg=TAB_BG, fg=TAB_FG, activebackground=TAB_BG)

    if view_name == "manual":
        shared_footer_manual.lift()
    elif view_name == "automatico":
        shared_footer_auto.lift()
    elif view_name == "foto":
        refresh_photo_preview(force=True)
    elif view_name == "sistema":
        refresh_system_network_info()

    if view_name in ("manual", "automatico"):
        set_shared_center_mode(shared_center_mode)


def set_shared_center_mode(mode):
    global shared_center_mode
    shared_center_mode = mode

    if mode == "graph":
        shared_graph_frame.lift()
        btn_manual_toggle.config(text="Mensajes")
        btn_auto_toggle.config(text="Mensajes")
    else:
        shared_console_frame.lift()
        btn_manual_toggle.config(text="Gráfica")
        btn_auto_toggle.config(text="Gráfica")

# ─────────────────────────────────────────────────────────────
# Acciones GUI
# ─────────────────────────────────────────────────────────────
def on_btn_zero():
    global lvdt_offset, user_zero_applied
    lvdt_offset = 0.0 if _latest_lvdt_raw is None else raw_to_mm(_latest_lvdt_raw)
    user_zero_applied = True
    save_runtime_config()
    console_print(f"Zero aplicado (offset = {lvdt_offset:.2f} mm)")
    lbl_lvdt_value.config(text="0.00 mm")


def on_btn_izq():
    global selected_direction, dir_actual, manual_override
    selected_direction = "izquierda"
    set_dir_buttons_color(selected_direction)
    if sistema_iniciado:
        manual_override = True
        dir_actual = "izquierda"
        pulse_channel(CH_IZQUIERDA, PULSE_SECONDS, btn_to_disable=btn_izq)
        set_motion_led("izquierda")


def on_btn_der():
    global selected_direction, dir_actual, manual_override
    selected_direction = "derecha"
    set_dir_buttons_color(selected_direction)
    if sistema_iniciado:
        manual_override = True
        dir_actual = "derecha"
        pulse_channel(CH_DERECHA, PULSE_SECONDS, btn_to_disable=btn_der)
        set_motion_led("derecha")


def on_btn_iniciar_det():
    global motion_thread, selected_direction

    if btn_det.cget("text") == "Iniciar":
        try:
            delta_user = float(entry_despl.get().replace(",", "."))
        except ValueError:
            console_print("Valor inválido en 'Desplazamiento'. Ej: 1 o -1")
            return

        delta_abs = abs(delta_user)
        if delta_abs < 1e-9:
            console_print("Desplazamiento = 0 mm (nada que mover)")
            return

        if selected_direction not in ("derecha", "izquierda"):
            selected_direction = "derecha"
            set_dir_buttons_color(selected_direction)

        if motion_thread and motion_thread.is_alive():
            console_print("Ya hay un movimiento en curso.")
            return

        btn_det.config(text="Detener", bg=BTN_RED, fg="white")
        stop_move.clear()
        offset_ref = lvdt_offset

        def _run():
            try:
                signed_delta = delta_abs if selected_direction == "derecha" else -delta_abs
                move_relative_blocking(
                    signed_delta,
                    part_of_auto=False,
                    offset_ref=offset_ref,
                    forced_direction=selected_direction
                )
            finally:
                btn_det.config(text="Iniciar", bg=BTN_BLUE, fg="white")

        motion_thread = threading.Thread(target=_run, daemon=True)
        motion_thread.start()
    else:
        stop_move.set()
        do_stop()
        btn_det.config(text="Iniciar", bg=BTN_BLUE, fg="white")


def update_points_listbox():
    points_listbox.delete(0, tk.END)
    for i, val in enumerate(disp_list, start=1):
        points_listbox.insert(tk.END, f"{i:03d}: {val:.3f}")


def on_btn_cargar():
    global disp_list, loaded_txt_path
    filename = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")])
    if not filename:
        return

    loaded_txt_path = filename
    disp_list = []

    with open(filename, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip().replace(" ", "")
            if not s:
                continue
            try:
                disp_list.append(float(s))
            except ValueError:
                pass

    set_graph_source_temp(clear_temp=True)
    update_points_listbox()
    console_print(f"Cargados {len(disp_list)} setpoints desde {os.path.basename(filename)}.")
    lbl_points_title.config(text=f"Lista de puntos ({len(disp_list)})")


def on_btn_auto():
    global auto_running, auto_start_time, auto_run_epoch, log_handle, motion_thread, auto_log_thread
    global auto_pause_started, auto_paused_accumulated

    if btn_auto.cget("text") == "Automático":
        if not disp_list:
            console_print("No hay setpoints cargados (usa 'Cargar TXT').")
            return

        if not messagebox.askyesno("Confirmar inicio", "¿Seguro que deseas iniciar el ensayo automático?"):
            console_print("Inicio de rutina automática cancelado por usuario.")
            return

        session_needed = (var_guardar.get() == 1) or bool(photo_auto_var.get())
        session_info = None

        if session_needed:
            if var_guardar.get() == 1:
                base_folder = filedialog.askdirectory(
                    title="Selecciona la carpeta base para guardar el ensayo",
                    initialdir=get_default_base_folder()
                )
                if not base_folder:
                    console_print("Inicio cancelado: no se seleccionó carpeta para guardar el ensayo.")
                    return
            else:
                base_folder = get_default_base_folder()

            try:
                session_info = prepare_automatic_session(base_folder)
            except Exception as e:
                close_session_files()
                set_graph_source_temp(clear_temp=True)
                messagebox.showerror("Error", f"No se pudo preparar el guardado del ensayo:\n{e}")
                console_print(f"No se pudo preparar el guardado del ensayo: {e}")
                return
        else:
            close_session_files()
            set_graph_source_temp(clear_temp=True)

        if session_info:
            console_print(f"Sesión de ensayo: {session_info['root']}")
            console_print(f"Datos: {session_info['data']}")
            console_print(f"Fotos: {session_info['photos']}")
            console_print(f"Log: {session_info['log']}")
            console_print(f"Archivo de datos: {os.path.basename(session_info['data_file'])}")
            console_print(f"Archivo de consola: {os.path.basename(session_info['console_file'])}")
            console_print(f"Gráfica usando archivo personalizado: {os.path.basename(graph_custom_file)}")
            if photo_auto_var.get() and var_guardar.get() == 0:
                console_print("Fotos automáticas activas: se usará guardado por defecto en el Escritorio.")

        on_btn_zero()
        console_print("Zero aplicado automáticamente al iniciar la rutina.")

        routine_offset_ref = lvdt_offset

        set_manual_buttons_enabled(False)
        btn_auto.config(text="Detener", bg=BTN_RED, fg="white")
        btn_pause.config(text="Pausar", state=tk.NORMAL, bg=BTN_LIGHT, fg=TEXT_MAIN)
        auto_paused.clear()

        auto_running = True
        stop_move.clear()
        auto_start_time = time.time()
        auto_run_epoch = auto_start_time
        auto_pause_started = None
        auto_paused_accumulated = 0.0
        lbl_runtime_value.config(text="0.0 min")

        start_auto_photo_scheduler_if_needed()

        if log_handle:
            auto_log_thread = threading.Thread(target=auto_logger_100hz, args=(routine_offset_ref,), daemon=True)
            auto_log_thread.start()
        else:
            auto_log_thread = None

        def _auto_run():
            global auto_running, auto_pause_started, auto_paused_accumulated
            try:
                auto_streaming_run(routine_offset_ref)
                if stop_move.is_set():
                    console_print("Rutina automática cancelada por usuario.")
                else:
                    console_print("Rutina automática finalizada.")
            finally:
                auto_running = False
                auto_pause_started = None
                auto_paused_accumulated = 0.0
                stop_auto_photo_scheduler()
                set_graph_source_temp(clear_temp=False)
                if session_root_folder:
                    console_print(f"Sesión cerrada: {session_root_folder}")
                close_session_files()
                btn_auto.config(text="Automático", bg=BTN_BLUE, fg="white")
                btn_pause.config(text="Pausar", state=tk.DISABLED, bg=BTN_LIGHT, fg=TEXT_MAIN)
                set_manual_buttons_enabled(True)

        motion_thread = threading.Thread(target=_auto_run, daemon=True)
        motion_thread.start()
    else:
        stop_move.set()
        stop_auto_photo_scheduler()
        set_graph_source_temp(clear_temp=False)
        console_print("Deteniendo rutina automática…")


def on_btn_pause():
    global sistema_iniciado, auto_pause_started, auto_paused_accumulated
    if not auto_running:
        return

    if btn_pause.cget("text") == "Pausar":
        auto_paused.set()
        auto_pause_started = time.time()
        if sistema_iniciado:
            pulse_channel(CH_DETENER, PULSE_SECONDS)
            sistema_iniciado = False
        set_motion_led("detenido")
        btn_pause.config(text="Continuar", bg=BTN_BLUE, fg="white")
        lbl_estado_grande.config(text="Pausado")
        console_print("Rutina automática en pausa.")
    else:
        auto_paused.clear()
        if auto_pause_started is not None:
            auto_paused_accumulated += (time.time() - auto_pause_started)
            auto_pause_started = None
        btn_pause.config(text="Pausar", bg=BTN_LIGHT, fg=TEXT_MAIN)
        lbl_estado_grande.config(text="Reanudando…")
        console_print("Rutina automática reanudada.")


def confirm_shutdown():
    if messagebox.askyesno("Confirmar apagado", "¿Seguro que deseas apagar el equipo?"):
        console_print("Apagando equipo…")
        try:
            subprocess.Popen(["sudo", "shutdown", "-h", "now"])
        except Exception as e:
            console_print(f"No se pudo ejecutar apagado: {e}")


def confirm_reboot():
    if messagebox.askyesno("Confirmar reinicio", "¿Seguro que deseas reiniciar el equipo?"):
        console_print("Reiniciando equipo…")
        try:
            subprocess.Popen(["sudo", "reboot"])
        except Exception as e:
            console_print(f"No se pudo ejecutar reinicio: {e}")

# ─────────────────────────────────────────────────────────────
# UI updater
# ─────────────────────────────────────────────────────────────
def update_live_labels(P, V, P1, P2, T1, T2, EA, EAC, status_main_text):
    lbl_lvdt_value.config(text=f"{P:.2f} mm")
    lbl_vel_value.config(text=f"{V:.2f} mm/s")
    lbl_temp1_value.config(text=T1)
    lbl_temp2_value.config(text=T2)
    lbl_pres1_value.config(text=P1)
    lbl_pres2_value.config(text=P2)
    lbl_auto_status_value.config(text=EA, fg=EAC)
    if status_main_text is not None:
        lbl_estado_grande.config(text=status_main_text)


def ui_updater():
    while True:
        time.sleep(0.1)
        pos = get_lvdt_display()
        vel = calculate_velocity(pos)

        # Durante pausa automática no se agregan nuevos puntos al gráfico
        if not (auto_running and auto_paused.is_set()):
            append_plot_sample_to_files(pos)

        eff_p1 = get_calibrated_sensor_value("PRES1", _latest_pres1_raw)
        eff_p2 = get_calibrated_sensor_value("PRES2", _latest_pres2_raw)
        eff_t1 = get_calibrated_sensor_value("TEMP1", _latest_temp1_raw)
        eff_t2 = get_calibrated_sensor_value("TEMP2", _latest_temp2_raw)
        p1 = format_sensor_value(eff_p1)
        p2 = format_sensor_value(eff_p2)
        t1 = format_sensor_value(eff_t1)
        t2 = format_sensor_value(eff_t2)

        if auto_running and auto_paused.is_set():
            estado_auto_texto = "En Pausa"
            estado_auto_color = BTN_ORANGE
            status_main_text = "Pausado"
        elif auto_running:
            estado_auto_texto = "Activa"
            estado_auto_color = BTN_GREEN
            status_main_text = None
        else:
            estado_auto_texto = "Desactivada"
            estado_auto_color = BTN_RED
            status_main_text = msg

        root.after(
            0,
            lambda P=pos, V=vel, P1=p1, P2=p2, T1=t1, T2=t2, EA=estado_auto_texto, EAC=estado_auto_color, SMT=status_main_text:
                update_live_labels(P, V, P1, P2, T1, T2, EA, EAC, SMT)
        )

        if auto_running and auto_start_time is not None:
            if auto_paused.is_set() and auto_pause_started is not None:
                elapsed = auto_pause_started - auto_start_time - auto_paused_accumulated
            else:
                elapsed = time.time() - auto_start_time - auto_paused_accumulated
            mins = max(0.0, elapsed / 60.0)
            root.after(0, lambda m=mins: lbl_runtime_value.config(text=f"{m:.1f} min"))
        else:
            root.after(0, lambda: lbl_runtime_value.config(text="0.0 min"))

# ─────────────────────────────────────────────────────────────
# Construcción GUI
# ─────────────────────────────────────────────────────────────
root = tk.Tk()
root.title("Control Actuador Hidráulico")
root.geometry(f"{WINDOW_CLIENT_W}x{WINDOW_CLIENT_H}+0+0")
root.resizable(False, False)
root.configure(bg=APP_BG)

manual_graph_window_var = tk.StringVar(value=config_get_str(APP_CONFIG, "MANUAL_GRAPH_WINDOW", "15 s"))
auto_graph_window_var = tk.StringVar(value=config_get_str(APP_CONFIG, "AUTO_GRAPH_WINDOW", "2 min"))
photo_folder_var = tk.StringVar(value=photo_folder)
photo_auto_var = tk.BooleanVar(value=config_get_bool(APP_CONFIG, "PHOTO_AUTO_ENABLED", auto_photo_enabled_default))
keyboard_enabled_var = tk.BooleanVar(value=config_get_bool(APP_CONFIG, "KEYBOARD_ENABLED", False))

reset_temp_graph_file()
set_motion_led("detenido")

# Header
frame_header = tk.Frame(root, bg=HEADER_BG)
frame_header.place(x=0, y=0, width=WINDOW_CLIENT_W, height=HEADER_H)

lbl_title = tk.Label(frame_header, text="Control Actuador Hidráulico", bg=HEADER_BG, fg="white", font=FONT_TITLE)
lbl_title.place(x=18, y=8)

lbl_mode_header = tk.Label(frame_header, text="Modo Manual", bg=HEADER_BG, fg="#dce8f5", font=FONT_MODE)
lbl_mode_header.place(x=382, y=14)

# Tabs superiores
tab_y = 52
tab_w = 145
tab_h = 34
tab_gap = 10
tab_xs = [20 + i * (tab_w + tab_gap) for i in range(5)]

nav_buttons = {}
view_titles = {
    "manual": "Modo Manual",
    "automatico": "Modo Automático",
    "foto": "Foto",
    "configuracion": "Configuración",
    "sistema": "Sistema",
}


def make_tab(name, text, x):
    btn = tk.Button(
        frame_header,
        text=text,
        command=lambda: show_view(name),
        relief="flat",
        bg=TAB_BG,
        fg=TAB_FG,
        activebackground=TAB_BG,
        font=FONT_BTN
    )
    btn.place(x=x, y=tab_y, width=tab_w, height=tab_h)
    nav_buttons[name] = btn


make_tab("manual", "Manual", tab_xs[0])
make_tab("automatico", "Automático", tab_xs[1])
make_tab("foto", "Foto", tab_xs[2])
make_tab("configuracion", "Configuración", tab_xs[3])
make_tab("sistema", "Sistema", tab_xs[4])

# Panel izquierdo fijo
left_panel = tk.Frame(root, bg=APP_BG)
left_panel.place(x=6, y=MAIN_Y, width=LEFT_W, height=MAIN_H)


def make_card(parent, x, y, w, h, title):
    card = tk.Frame(parent, bg=CARD_BG, highlightbackground=CARD_BORDER, highlightthickness=1)
    card.place(x=x, y=y, width=w, height=h)
    lbl = tk.Label(card, text=title, bg=CARD_BG, fg=TEXT_MAIN, font=FONT_CARD)
    lbl.place(x=12, y=8)
    return card

# Card estado
card_state = make_card(left_panel, 0, 0, LEFT_W, 126, "Estado del equipo")
tk.Label(card_state, text="Estado actual", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=16, y=30)
lbl_estado_grande = tk.Label(card_state, text=msg, bg=STATUS_BG, fg=TEXT_MAIN, font=("Helvetica", 13, "bold"))
lbl_estado_grande.place(x=16, y=46, width=204, height=30)
tk.Label(card_state, text="Referencia", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=16, y=86)
btn_zero = tk.Button(card_state, text="Zero", command=on_btn_zero, relief="flat",
                     bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=FONT_BTN2)
btn_zero.place(x=16, y=100, width=70, height=22)

# Card lecturas
card_live = make_card(left_panel, 0, 136, LEFT_W, 170, "Lecturas en vivo")


def make_live_value(parent, title, x, y, varname):
    tk.Label(parent, text=title, bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=x, y=y)
    lbl = tk.Label(parent, text="—", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_BIGVAL)
    lbl.place(x=x, y=y + 14)
    globals()[varname] = lbl


make_live_value(card_live, "LVDT", 16, 30, "lbl_lvdt_value")
make_live_value(card_live, "Velocidad", 122, 30, "lbl_vel_value")
make_live_value(card_live, "Temp 1", 16, 76, "lbl_temp1_value")
make_live_value(card_live, "Temp 2", 122, 76, "lbl_temp2_value")
make_live_value(card_live, "Presión 1", 16, 122, "lbl_pres1_value")
make_live_value(card_live, "Presión 2", 122, 122, "lbl_pres2_value")

# Card runtime ajustada
card_runtime = make_card(left_panel, 0, 316, LEFT_W, 72, "Rutina automática")
tk.Label(card_runtime, text="Estado:", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=16, y=24)
lbl_auto_status_value = tk.Label(card_runtime, text="Desactivada", bg=CARD_BG, fg=BTN_RED, font=("Helvetica", 11, "bold"))
lbl_auto_status_value.place(x=62, y=20)

tk.Label(card_runtime, text="Tiempo transcurrido", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=16, y=46)
lbl_runtime_value = tk.Label(card_runtime, text="0.0 min", bg=CARD_BG, fg=TEXT_MAIN, font=("Helvetica", 11, "bold"))
lbl_runtime_value.place(x=116, y=42)

# Vistas superiores
view_frames = {}
for key in ["manual", "automatico", "foto", "configuracion", "sistema"]:
    frame = tk.Frame(root, bg=APP_BG, highlightbackground=CARD_BORDER, highlightthickness=1)
    view_frames[key] = frame

# ───────────────────── MANUAL ─────────────────────
manual = view_frames["manual"]

card_manual_controls = make_card(manual, 8, 8, 330, 100, "Control manual")
btn_izq = tk.Button(card_manual_controls, text="Izquierda", command=on_btn_izq, relief="flat",
                    bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=FONT_BTN)
btn_izq.place(x=14, y=30, width=82, height=28)

btn_det = tk.Button(card_manual_controls, text="Iniciar", command=on_btn_iniciar_det, relief="flat",
                    bg=BTN_BLUE, fg="white", activebackground=BTN_BLUE_DARK, font=FONT_BTN)
btn_det.place(x=104, y=30, width=82, height=28)

btn_der = tk.Button(card_manual_controls, text="Derecha", command=on_btn_der, relief="flat",
                    bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=FONT_BTN)
btn_der.place(x=194, y=30, width=82, height=28)

tk.Label(card_manual_controls, text="Desplazamiento [mm]", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=14, y=62)
entry_despl = tk.Entry(card_manual_controls, font=FONT_MED, justify="center")
entry_despl.place(x=14, y=76, width=76, height=20)
entry_despl.insert(0, config_get_str(APP_CONFIG, "MANUAL_DISPLACEMENT_MM", "1"))

# ───────────────────── AUTOMÁTICO ─────────────────────
automatico = view_frames["automatico"]

card_auto_controls = make_card(automatico, 8, 8, 448, 100, "Rutina automática")
btn_cargar = tk.Button(card_auto_controls, text="Cargar TXT", command=on_btn_cargar, relief="flat",
                       bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=FONT_BTN)
btn_cargar.place(x=12, y=30, width=88, height=28)

btn_auto = tk.Button(card_auto_controls, text="Automático", command=on_btn_auto, relief="flat",
                     bg=BTN_BLUE, fg="white", activebackground=BTN_BLUE_DARK, font=FONT_BTN)
btn_auto.place(x=108, y=30, width=92, height=28)

btn_pause = tk.Button(card_auto_controls, text="Pausar", command=on_btn_pause, state=tk.DISABLED,
                      relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=FONT_BTN)
btn_pause.place(x=208, y=30, width=78, height=28)

var_guardar = tk.IntVar(value=1 if config_get_bool(APP_CONFIG, "AUTO_GUARDAR", False) else 0)
chk_guardar = tk.Checkbutton(card_auto_controls, text="Guardar", variable=var_guardar,
                             bg=CARD_BG, fg=TEXT_MAIN, activebackground=CARD_BG, font=FONT_MED)
chk_guardar.place(x=12, y=60)

card_points = make_card(automatico, 464, 8, MAIN_W - 472, 100, "Lista de puntos")
lbl_points_title = tk.Label(card_points, text="Lista de puntos (0)", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_CARD)
lbl_points_title.place(x=12, y=8)

points_scroll = tk.Scrollbar(card_points, orient="vertical", width=24)
points_scroll.place(x=(MAIN_W - 472) - 32, y=30, width=24, height=58)

points_listbox = tk.Listbox(card_points, font=("Courier", 10), yscrollcommand=points_scroll.set)
points_listbox.place(x=12, y=30, width=(MAIN_W - 472) - 48, height=58)
points_scroll.config(command=points_listbox.yview)

# ───────────────────── FOTO ─────────────────────
foto = view_frames["foto"]

card_photo_main = make_card(foto, 8, 8, 560, MAIN_H - 16, "Última foto tomada")
lbl_photo_preview = tk.Label(card_photo_main, text="Sin imagen", bg="#f2f4f7", fg=TEXT_MUTED)
lbl_photo_preview.place(x=24, y=34, width=505, height=220)

lbl_photo_name = tk.Label(card_photo_main, text="Última foto: —", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_SMALL, anchor="w")
lbl_photo_name.place(x=24, y=262, width=505)
lbl_photo_time = tk.Label(card_photo_main, text="Hora: —", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL, anchor="w")
lbl_photo_time.place(x=24, y=278, width=505)

btn_photo_manual = tk.Button(card_photo_main, text="FOTO", command=manual_photo_capture, relief="flat",
                             bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 12, "bold"))
btn_photo_manual.place(x=220, y=304, width=110, height=28)

lbl_photo_status = tk.Label(
    card_photo_main,
    text=photo_status_message,
    bg=CARD_BG,
    fg=photo_status_color,
    font=("Helvetica", 9, "bold"),
    justify="left",
    anchor="nw",
    wraplength=505
)
lbl_photo_status.place(x=24, y=338, width=505, height=40)

card_photo_cfg = make_card(foto, 576, 8, MAIN_W - 584, 178, "Configuración")
chk_photo_auto = tk.Checkbutton(card_photo_cfg, text="Fotos automáticas", variable=photo_auto_var,
                                bg=CARD_BG, fg=TEXT_MAIN, activebackground=CARD_BG, font=FONT_MED)
chk_photo_auto.place(x=14, y=32)

tk.Label(card_photo_cfg, text="Intervalo:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=14, y=66)
entry_photo_interval = tk.Entry(card_photo_cfg, font=FONT_MED, justify="center")
entry_photo_interval.place(x=76, y=62, width=46, height=24)
entry_photo_interval.insert(0, config_get_str(APP_CONFIG, "PHOTO_INTERVAL_MIN", "0.5"))
tk.Label(card_photo_cfg, text="min", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_SMALL).place(x=126, y=66)

btn_photo_folder = tk.Button(card_photo_cfg, text="Carpeta", command=choose_photo_folder, relief="flat",
                             bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=FONT_BTN2)
btn_photo_folder.place(x=14, y=98, width=90, height=26)

entry_photo_folder = tk.Entry(card_photo_cfg, textvariable=photo_folder_var, font=("Helvetica", 8))
entry_photo_folder.place(x=14, y=132, width=(MAIN_W - 584) - 28, height=34)

# ───────────────────── CONFIGURACIÓN ─────────────────────
configuracion = view_frames["configuracion"]
card_cfg = make_card(configuracion, 8, 8, MAIN_W - 16, MAIN_H - 16, "Alertas activas")

tk.Label(card_cfg, text="Presión 1 Max:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=40, y=40)
entry_p1max = tk.Entry(card_cfg, width=10, font=FONT_MED)
entry_p1max.place(x=152, y=38)
entry_p1max.insert(0, config_get_str(APP_CONFIG, "P1_MAX", "250.0"))

tk.Label(card_cfg, text="Presión 2 Max:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=40, y=68)
entry_p2max = tk.Entry(card_cfg, width=10, font=FONT_MED)
entry_p2max.place(x=152, y=66)
entry_p2max.insert(0, config_get_str(APP_CONFIG, "P2_MAX", "250.0"))

tk.Label(card_cfg, text="LVDT Min:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=40, y=96)
entry_lvdtmin = tk.Entry(card_cfg, width=10, font=FONT_MED)
entry_lvdtmin.place(x=152, y=94)
entry_lvdtmin.insert(0, config_get_str(APP_CONFIG, "LVDT_MIN", "-25.0"))

tk.Label(card_cfg, text="Temperatura 1 Max:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=300, y=40)
entry_t1max = tk.Entry(card_cfg, width=10, font=FONT_MED)
entry_t1max.place(x=448, y=38)
entry_t1max.insert(0, config_get_str(APP_CONFIG, "T1_MAX", "80.0"))

tk.Label(card_cfg, text="Temperatura 2 Max:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=300, y=68)
entry_t2max = tk.Entry(card_cfg, width=10, font=FONT_MED)
entry_t2max.place(x=448, y=66)
entry_t2max.insert(0, config_get_str(APP_CONFIG, "T2_MAX", "80.0"))

tk.Label(card_cfg, text="LVDT Max:", bg=CARD_BG, fg=TEXT_MAIN, font=FONT_MED).place(x=300, y=96)
entry_lvdtmax = tk.Entry(card_cfg, width=10, font=FONT_MED)
entry_lvdtmax.place(x=448, y=94)
entry_lvdtmax.insert(0, config_get_str(APP_CONFIG, "LVDT_MAX", "25.0"))

cfg_stop_alert_var = tk.IntVar(value=1 if config_get_bool(APP_CONFIG, "CFG_STOP_ALERT", False) else 0)
chk_stop_alert = tk.Checkbutton(card_cfg, text="Detener rutina si ocurre una alerta", variable=cfg_stop_alert_var,
                                bg=CARD_BG, fg=TEXT_MAIN, activebackground=CARD_BG, font=("Helvetica", 11))
chk_stop_alert.place(x=40, y=140)

# ───────────────────── SISTEMA ─────────────────────
sistema = view_frames["sistema"]

card_sys = make_card(sistema, 8, 8, 280, 152, "Control")
btn_shutdown = tk.Button(card_sys, text="Apagar", command=confirm_shutdown, relief="flat",
                         bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 14))
btn_shutdown.place(x=42, y=42, width=130, height=38)

btn_reboot = tk.Button(card_sys, text="Reiniciar", command=confirm_reboot, relief="flat",
                       bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 14))
btn_reboot.place(x=42, y=90, width=130, height=38)

card_sys_cfg = make_card(sistema, 296, 8, MAIN_W - 304, 152, "Configuración")
chk_keyboard = tk.Checkbutton(
    card_sys_cfg,
    text="Teclado en pantalla",
    variable=keyboard_enabled_var,
    command=on_toggle_keyboard,
    bg=CARD_BG,
    fg=TEXT_MAIN,
    activebackground=CARD_BG,
    font=("Helvetica", 10)
)
chk_keyboard.place(x=18, y=34)

tk.Label(card_sys_cfg, text="IP WiFi:", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=18, y=72)
lbl_wifi_ip_value = tk.Label(card_sys_cfg, text="—", bg=CARD_BG, fg=TEXT_MAIN, font=("Helvetica", 11, "bold"), anchor="w")
lbl_wifi_ip_value.place(x=84, y=66, width=220)

tk.Label(card_sys_cfg, text="IP Ethernet:", bg=CARD_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=18, y=106)
lbl_eth_ip_value = tk.Label(card_sys_cfg, text="—", bg=CARD_BG, fg=TEXT_MAIN, font=("Helvetica", 11, "bold"), anchor="w")
lbl_eth_ip_value.place(x=84, y=100, width=220)

card_about = make_card(sistema, 8, 168, MAIN_W - 16, MAIN_H - 176, "Acerca de")

about_text = tk.Label(
    card_about,
    text="Sistema de automatizacion actuador lineal PUCV, diseñado y fabricado por LoRe Ingenieria.\ncontacto@lore-ingenieria.cl\nwww.lore-ingenieria.cl",
    justify="left",
    anchor="nw",
    bg=CARD_BG,
    fg=TEXT_MAIN,
    font=("Helvetica", 12),
    wraplength=430
)
about_text.place(x=18, y=36, width=450, height=100)

logo_paths = [
    os.path.join(os.getcwd(), "logo-LoRe.png"),
    "/mnt/data/logo-LoRe.png",
]
logo_lore_imgtk = None
for lp in logo_paths:
    if os.path.exists(lp):
        try:
            img = Image.open(lp)
            img.thumbnail((250, 130), Image.LANCZOS)
            logo_lore_imgtk = ImageTk.PhotoImage(img)
            break
        except Exception:
            logo_lore_imgtk = None

if logo_lore_imgtk is not None:
    lbl_logo_about = tk.Label(card_about, image=logo_lore_imgtk, bg=CARD_BG)
    lbl_logo_about.image = logo_lore_imgtk
    lbl_logo_about.place(x=500, y=18)
else:
    tk.Label(card_about, text="Logo LoRe", bg=CARD_BG, fg=TEXT_MUTED, font=("Helvetica", 11, "italic")).place(x=540, y=54)

# ───────────────────── Panel compartido manual/automático ─────────────────────
shared_center_frame = tk.Frame(root, bg=APP_BG)

shared_graph_frame = tk.Frame(shared_center_frame, bg=APP_BG)
shared_console_frame = tk.Frame(shared_center_frame, bg=APP_BG)

shared_graph_frame.place(x=0, y=0, width=SHARED_AREA_W, height=SHARED_CONTENT_H)
shared_console_frame.place(x=0, y=0, width=SHARED_AREA_W, height=SHARED_CONTENT_H)

shared_card_graph = make_card(shared_graph_frame, 0, 0, SHARED_AREA_W, SHARED_CONTENT_H, "Gráfico LVDT")
if MATPLOTLIB_OK:
    shared_fig = Figure(figsize=(6.2, 2.2), dpi=100)
    shared_ax = shared_fig.add_subplot(111)
    shared_canvas = FigureCanvasTkAgg(shared_fig, master=shared_card_graph)
    shared_canvas.get_tk_widget().place(x=8, y=30, width=SHARED_AREA_W - 16, height=SHARED_CONTENT_H - 38)
else:
    shared_fig = shared_ax = shared_canvas = None
    tk.Label(shared_card_graph, text="matplotlib no disponible", bg=CARD_BG, fg=TEXT_MUTED).place(relx=0.5, rely=0.5, anchor="center")

shared_card_console = make_card(shared_console_frame, 0, 0, SHARED_AREA_W, SHARED_CONTENT_H, "Mensajes / consola")
txt_shared_console = tk.Text(shared_card_console, wrap="word", font=FONT_CONSOLE, state=tk.DISABLED)
txt_shared_console.place(x=8, y=30, width=SHARED_AREA_W - 16, height=SHARED_CONTENT_H - 38)
register_console_widget(txt_shared_console)

# Footer compartido con controles inferiores
shared_footer_container = tk.Frame(shared_center_frame, bg=APP_BG)
shared_footer_container.place(x=0, y=SHARED_CONTENT_H + 4, width=SHARED_AREA_W, height=SHARED_FOOTER_H)

shared_footer_manual = tk.Frame(shared_footer_container, bg=APP_BG)
shared_footer_auto = tk.Frame(shared_footer_container, bg=APP_BG)

shared_footer_manual.place(x=0, y=0, width=SHARED_AREA_W, height=SHARED_FOOTER_H)
shared_footer_auto.place(x=0, y=0, width=SHARED_AREA_W, height=SHARED_FOOTER_H)

# Footer manual
tk.Label(shared_footer_manual, text="Ventana", bg=APP_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=10, y=9)

opt_manual = tk.OptionMenu(shared_footer_manual, manual_graph_window_var, "15 s", "30 s", "1 min", "2 min", "5 min", "Todo")
opt_manual.config(relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, highlightthickness=0, font=FONT_SMALL)
opt_manual.place(x=54, y=5, width=82, height=24)

btn_manual_toggle = tk.Button(shared_footer_manual, text="Gráfica", command=toggle_shared_center_mode,
                              relief="flat", bg=BTN_BLUE, fg="white", activebackground=BTN_BLUE_DARK, font=FONT_BTN2)
btn_manual_toggle.place(x=150, y=5, width=80, height=24)

btn_manual_reset = tk.Button(shared_footer_manual, text="Borrar gráfica", command=reset_graph_action,
                             relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 9))
btn_manual_reset.place(x=240, y=5, width=102, height=24)

btn_manual_graph_pause = tk.Button(shared_footer_manual, text="Pausar", command=toggle_graph_display_pause,
                                   relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 9))
btn_manual_graph_pause.place(x=350, y=5, width=92, height=24)

# Footer automático
tk.Label(shared_footer_auto, text="Ventana", bg=APP_BG, fg=TEXT_MUTED, font=FONT_SMALL).place(x=10, y=9)

opt_auto = tk.OptionMenu(shared_footer_auto, auto_graph_window_var, "30 s", "1 min", "2 min", "5 min", "10 min", "Todo")
opt_auto.config(relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, highlightthickness=0, font=FONT_SMALL)
opt_auto.place(x=54, y=5, width=82, height=24)

btn_auto_toggle = tk.Button(shared_footer_auto, text="Gráfica", command=toggle_shared_center_mode,
                            relief="flat", bg=BTN_BLUE, fg="white", activebackground=BTN_BLUE_DARK, font=FONT_BTN2)
btn_auto_toggle.place(x=150, y=5, width=80, height=24)

btn_auto_reset = tk.Button(shared_footer_auto, text="Borrar gráfica", command=reset_graph_action,
                           relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 9))
btn_auto_reset.place(x=240, y=5, width=102, height=24)

btn_auto_graph_pause = tk.Button(shared_footer_auto, text="Pausar", command=toggle_graph_display_pause,
                                 relief="flat", bg=BTN_LIGHT, activebackground=BTN_LIGHT, font=("Helvetica", 9))
btn_auto_graph_pause.place(x=350, y=5, width=92, height=24)

# ─────────────────────────────────────────────────────────────
# Estado inicial
# ─────────────────────────────────────────────────────────────
set_dir_buttons_color(None)
set_graph_display_pause_state(False)
set_shared_center_mode("messages")
show_view("manual")

console_print("Actuador-PUCV --- LoRe Ingenieria 2026")
set_photo_status(photo_status_message, photo_status_color, busy=False)
save_runtime_config()
root.protocol("WM_DELETE_WINDOW", on_app_close)

# ─────────────────────────────────────────────────────────────
# Tareas
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=serial_reader, daemon=True).start()
    threading.Thread(target=ui_updater, daemon=True).start()
    threading.Thread(target=photo_capture_worker, daemon=True).start()
    root.after(650, periodic_graph_refresh)
    root.after(2000, periodic_save_config)
    root.after(PHOTO_MONITOR_MS, monitor_latest_photo)
    root.mainloop()
