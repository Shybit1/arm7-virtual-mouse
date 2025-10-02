import ctypes
import serial
import threading
import time
import keyboard  # pip install keyboard

# --- Windows API setup ---
PUL = ctypes.POINTER(ctypes.c_ulong)

class MOUSEINPUT(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class _INPUT_union(ctypes.Union):
    _fields_ = [("mi", MOUSEINPUT)]

class INPUT(ctypes.Structure):
    _anonymous_ = ("union",)
    _fields_ = [("type", ctypes.c_ulong),
                ("union", _INPUT_union)]

MOUSEEVENTF_MOVE = 0x0001
MOUSEEVENTF_LEFTDOWN = 0x0002
MOUSEEVENTF_LEFTUP = 0x0004
MOUSEEVENTF_RIGHTDOWN = 0x0008
MOUSEEVENTF_RIGHTUP = 0x0010
MOUSEEVENTF_WHEEL = 0x0800
INPUT_MOUSE = 0
SCROLL_MULTIPLIER = 1  # Increase this to scroll faster (e.g., 2 or 3)
last_scroll_time = 0
SCROLL_DEBOUNCE_MS = 200

def send_mouse_move(dx, dy):
    mi = MOUSEINPUT(dx, dy, 0, MOUSEEVENTF_MOVE, 0, None)
    inp = INPUT(INPUT_MOUSE, _INPUT_union(mi))
    ctypes.windll.user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))

def send_mouse_click(button, down):
    flag = {
        ('left', True): MOUSEEVENTF_LEFTDOWN,
        ('left', False): MOUSEEVENTF_LEFTUP,
        ('right', True): MOUSEEVENTF_RIGHTDOWN,
        ('right', False): MOUSEEVENTF_RIGHTUP,
    }.get((button, down), 0)
    if flag:
        mi = MOUSEINPUT(0, 0, 0, flag, 0, None)
        inp = INPUT(INPUT_MOUSE, _INPUT_union(mi))
        ctypes.windll.user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))

def send_mouse_scroll(amount):
    mi = MOUSEINPUT(0, 0, amount, MOUSEEVENTF_WHEEL, 0, None)
    inp = INPUT(INPUT_MOUSE, _INPUT_union(mi))
    ctypes.windll.user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))

# --- Main Serial Mouse Function ---
def serial_mouse(port='COM5', baud=115200):
    ser = serial.Serial(port, baud, timeout=1)
    prev_left = False
    prev_right = False

    # Tunables
    base_gain = 0.2           # Movement gain
    deadzone = 8              # Ignore micro jitter
    filter_alpha = 0.6        # Low-pass filter weight
    # Dynamic gain adjustment (debug hotkeys)
    if keyboard.is_pressed('up'):
        gain += 0.01
    if keyboard.is_pressed('down'):
        gain = max(0.01, gain - 0.01)
    # Previous filtered values
    filtered_dx = 0
    filtered_dy = 0

    def velocity_curve(x):
        return int(base_gain * x * abs(x) / 10)  # Quadratic gain

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 5:
                continue

            dx_raw, dy_raw, leftClick, rightClick, scroll = map(int, parts)

            # Deadzone filtering
            if abs(dx_raw) < deadzone:
                dx_raw = 0
            if abs(dy_raw) < deadzone:
                dy_raw = 0

            # Apply low-pass filter
            filtered_dx = filter_alpha * dx_raw + (1 - filter_alpha) * filtered_dx
            filtered_dy = filter_alpha * dy_raw + (1 - filter_alpha) * filtered_dy

            # Apply nonlinear velocity scaling
            move_x = velocity_curve(filtered_dx)
            move_y = velocity_curve(filtered_dy)

            if move_x != 0 or move_y != 0:
                send_mouse_move(move_x, -move_y)  # invert Y

            # Left click edge-trigger
            if leftClick and not prev_left:
                send_mouse_click('left', True)
            elif not leftClick and prev_left:
                send_mouse_click('left', False)
            prev_left = bool(leftClick)

            # Right click edge-trigger
            if rightClick and not prev_right:
                send_mouse_click('right', True)
            elif not rightClick and prev_right:
                send_mouse_click('right', False)
            prev_right = bool(rightClick)

            current_time = time.time() * 1000  # Convert to milliseconds

            if scroll != 0 and (current_time - last_scroll_time) > SCROLL_DEBOUNCE_MS:
                send_mouse_scroll(scroll * 120 * SCROLL_MULTIPLIER)
                last_scroll_time = current_time


        except Exception as e:
            print("Error:", e)

# --- Start Serial Thread ---
if __name__ == "__main__":
    t = threading.Thread(target=serial_mouse, args=('COM5', 115200), daemon=True)
    t.start()
    while True:
        time.sleep(1)