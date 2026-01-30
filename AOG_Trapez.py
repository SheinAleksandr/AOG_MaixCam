# MaixCAM Pro / MaixPy
# YOLOv5 obstacle detection + symmetric trapezoid corridor with TWO LEFT handles:
#   Handle A (left-bottom = D): X -> AD (bottom width), Y -> AB + vertical position of AD
#   Handle B (left-top = C):    X -> BC (top width),    Y -> AB + vertical position of BC
# AB and CD are symmetric automatically.

from maix import camera, display, image, nn, app, sys, time, network
from maix.touchscreen import TouchScreen
import socket

device_id = sys.device_id()

# =========================
# TouchScreen init
# =========================
try:
    touchscreen = TouchScreen()
    print("✅ TouchScreen инициализирован")
except Exception as e:
    print(f"❌ Ошибка инициализации TouchScreen: {e}")
    touchscreen = None

# =========================
# Wi-Fi manager
# =========================
class WiFiManager:
    def __init__(self):
        self.wifi = network.wifi.Wifi()
        self.udp_socket = None
        self.esp32_ip = "192.168.4.1"
        self.esp32_port = 8888
        self.connected = False

    def connect(self, ssid, password, timeout=30):
        print(f"📡 Подключение к Wi-Fi: {ssid}")
        try:
            e = self.wifi.connect(ssid, password, wait=True, timeout=timeout)
            if e == 0:
                self.connected = True
                new_ip = self.wifi.get_ip()
                print(f"✅ Wi-Fi подключен! IP: {new_ip}")
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print(f"✅ UDP сокет создан для отправки на {self.esp32_ip}:{self.esp32_port}")
                return True
            print(f"❌ Ошибка подключения Wi-Fi: {e}")
            return False
        except Exception as e:
            print(f"❌ Ошибка настройки Wi-Fi: {e}")
            return False

    def send_obstacle_data(self, has_obstacle, obstacle_count, steering_angle):
        if (not self.connected) or (self.udp_socket is None):
            return False
        try:
            msg = f"OBSTACLE:{1 if has_obstacle else 0}:COUNT:{obstacle_count}:ANGLE:{steering_angle:.1f}"
            self.udp_socket.sendto(msg.encode("utf-8"), (self.esp32_ip, self.esp32_port))
            return True
        except Exception as e:
            print(f"❌ Ошибка отправки UDP: {e}")
            return False

# =========================
# Angle receiver
# =========================
class AngleReceiver:
    def __init__(self, listen_port=8889):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(0.02)
        self.udp_socket.bind(("0.0.0.0", listen_port))
        self.current_angle = 0.0

    def receive_angle(self):
        try:
            data, _ = self.udp_socket.recvfrom(64)
            msg = data.decode("utf-8").strip()
            if msg.startswith("ANGLE:"):
                s = msg.replace("ANGLE:", "").strip()
                try:
                    self.current_angle = float(s)
                    return True
                except ValueError:
                    return False
        except socket.timeout:
            return False
        except Exception as e:
            print(f"❌ Ошибка приема угла: {e}")
            return False

# =========================
# Touch calibration (simple scaling)
# =========================
class TouchCalibrator:
    def __init__(self, display_width, display_height):
        self.display_width = display_width
        self.display_height = display_height
        print(f"📐 Калибратор: {display_width}x{display_height}")

    def transform_coordinates(self, x, y):
        display_x = int(x * (self.display_width / 640.0))
        display_y = int(y * (self.display_height / 480.0))
        display_x = max(0, min(display_x, self.display_width - 1))
        display_y = max(0, min(display_y, self.display_height - 1))
        return display_x, display_y

# =========================
# Geometry: point in convex quad
# =========================
def point_in_quad(px, py, quad):
    def cross(ax, ay, bx, by, cx, cy):
        return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)

    x1,y1 = quad[0]
    x2,y2 = quad[1]
    x3,y3 = quad[2]
    x4,y4 = quad[3]

    c1 = cross(x1,y1, x2,y2, px,py)
    c2 = cross(x2,y2, x3,y3, px,py)
    c3 = cross(x3,y3, x4,y4, px,py)
    c4 = cross(x4,y4, x1,y1, px,py)

    return (c1 >= 0 and c2 >= 0 and c3 >= 0 and c4 >= 0) or \
           (c1 <= 0 and c2 <= 0 and c3 <= 0 and c4 <= 0)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# =========================
# ZoneConfig: symmetric trapezoid with two LEFT handles (B=top-left, A=bottom-left)
# =========================
class ZoneConfig:
    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.shift_far_k = 1.0 # коэффициэнт движения вершины

        # base geometry in pixels derived from ratios (init)
        self.yA_ratio = 0.90  # bottom line y (AD)
        self.yB_ratio = 0.28  # top line y (BC)

        self.near_half_ratio = 0.30  # half width of bottom AD
        self.far_half_ratio  = 0.14  # half width of top BC

        # steering shift behavior
        self.max_shift_ratio = 0.4
        self.max_steer_for_max = 30.0  # +/-30° -> max shift

        # UI / edit
        self.edit_mode = False
        self.selected = None   # 'A' or 'B'
        self.touch_threshold = 60
        self.last_touch_time = 0
        self.touch_cooldown = 70

        self.obstacle_detection_enabled = True

        # constraints
        self.min_half_ratio = 0.06
        self.max_half_ratio = 0.49
        self.min_height_px = 40

    def toggle_obstacle_detection(self):
        self.obstacle_detection_enabled = not self.obstacle_detection_enabled
        status = "ВКЛЮЧЕНО" if self.obstacle_detection_enabled else "ВЫКЛЮЧЕНО"
        print(f"🎯 Обнаружение препятствий: {status}")
        return self.obstacle_detection_enabled

    def _steer_norm(self, steering_angle):
        return clamp(steering_angle / self.max_steer_for_max, -1.0, 1.0)

    def _get_params_px(self):
        W, H = self.width, self.height
        cx = W // 2

        yA = int(H * self.yA_ratio)
        yB = int(H * self.yB_ratio)

        # ensure valid ordering & min height
        yA = clamp(yA, int(H * 0.55), H - 1)        # bottom must be lower half
        yB = clamp(yB, 0, int(H * 0.80))            # top must be upper-ish
        if yA - yB < self.min_height_px:
            yB = max(0, yA - self.min_height_px)

        near_half = int(W * self.near_half_ratio)
        far_half  = int(W * self.far_half_ratio)

        near_half = clamp(near_half, int(W * self.min_half_ratio), int(W * self.max_half_ratio))
        far_half  = clamp(far_half,  int(W * self.min_half_ratio), int(W * self.max_half_ratio))

        return cx, yA, yB, near_half, far_half

    def get_trapezoid(self, steering_angle=0.0):
        """
        Returns A,B,C,D in your naming:
          A = right-bottom
          B = right-top
          C = left-top
          D = left-bottom
        Symmetric about centerline, then shifted by steering.
        """
        W = self.width
        cx, yA, yB, near_half, far_half = self._get_params_px()

        max_shift = int(W * self.max_shift_ratio)
        s = self._steer_norm(steering_angle)

        shift_far = int(s * max_shift * self.shift_far_k)     # ВЕРХ ДВИГАЕМ ПО КОЭФФ
        shift_near = int(0.1 * shift_far)                     # НИЗ  ДВИГАЕМ МЕНЬШЕ
        
        # bottom (AD) — без сдвига
        D = (cx - near_half + shift_near, yA)
        A = (cx + near_half + shift_near, yA)

        # top (BC) — со сдвигом
        C = (cx - far_half + shift_far, yB)
        B = (cx + far_half + shift_far, yB)

        # clamp bounds
        def clamp_pt(p):
            return (int(clamp(p[0], 0, self.width - 1)), int(clamp(p[1], 0, self.height - 1)))

        return clamp_pt(A), clamp_pt(B), clamp_pt(C), clamp_pt(D)

    def get_quad_for_tests(self, steering_angle=0.0):
        A, B, C, D = self.get_trapezoid(steering_angle)
        # polygon for point_in_quad: D->C->B->A
        return [D, C, B, A], (A, B, C, D)

    # ---- handles: two points on LEFT (B=top-left (C), A=bottom-left (D)) ----
    def get_left_handles(self, steering_angle=0.0):
        A, B, C, D = self.get_trapezoid(steering_angle)
        # handle A = left-bottom corner D
        # handle B = left-top corner C
        return {
            "A": (D[0], D[1]),
            "B": (C[0], C[1]),
        }

    # ---- update rules per your requirements ----
    def _set_near_from_x(self, x):
        W = self.width
        cx = W // 2
        half = abs(cx - x)
        half = clamp(half, int(W * self.min_half_ratio), int(W * self.max_half_ratio))
        self.near_half_ratio = half / W
        print(f"📐 AD (низ) half={half}px  ratio={self.near_half_ratio:.3f}")

    def _set_far_from_x(self, x):
        W = self.width
        cx = W // 2
        half = abs(cx - x)
        half = clamp(half, int(W * self.min_half_ratio), int(W * self.max_half_ratio))
        self.far_half_ratio = half / W
        print(f"📐 BC (верх) half={half}px  ratio={self.far_half_ratio:.3f}")

    def _set_yA_from_y(self, y):
        H = self.height
        y = clamp(y, int(H * 0.55), H - 1)
        self.yA_ratio = y / H
        print(f"📐 yA (AD по вертикали) = {y}px  ratio={self.yA_ratio:.3f}")

    def _set_yB_from_y(self, y):
        H = self.height
        y = clamp(y, 0, int(H * 0.80))
        self.yB_ratio = y / H
        print(f"📐 yB (BC по вертикали) = {y}px  ratio={self.yB_ratio:.3f}")

    def handle_touch(self, x, y, pressed, steering_angle=0.0):
        now = time.ticks_ms()
        if now - self.last_touch_time < self.touch_cooldown:
            return False
        self.last_touch_time = now

        # Buttons
        btn_w, btn_h = 120, 32
        btn_y = 0
        edit_x = 0
        det_x = self.width - btn_w

        if not self.edit_mode:
            if pressed == 1:
                if edit_x <= x <= edit_x + btn_w and btn_y <= y <= btn_y + btn_h:
                    self.edit_mode = True
                    self.selected = None
                    print("📐 EDIT ON: две точки слева (A=низ, B=верх)")
                    return True
                if det_x <= x <= det_x + btn_w and btn_y <= y <= btn_y + btn_h:
                    self.toggle_obstacle_detection()
                    return True
            return False

        # editing
        if pressed == 1:
            if edit_x <= x <= edit_x + btn_w and btn_y <= y <= btn_y + btn_h:
                self.edit_mode = False
                self.selected = None
                print("💾 EDIT OFF")
                return True
            if det_x <= x <= det_x + btn_w and btn_y <= y <= btn_y + btn_h:
                self.toggle_obstacle_detection()
                return True

        handles = self.get_left_handles(steering_angle)

        # pick nearest handle
        nearest = None
        best_d = 1e9
        for k, (hx, hy) in handles.items():
            d = ((x - hx) ** 2 + (y - hy) ** 2) ** 0.5
            if d < best_d:
                best_d = d
                nearest = k

        if pressed == 1:
            if best_d < self.touch_threshold:
                self.selected = nearest
                print(f"🎯 Выбрана точка {self.selected}")
            else:
                self.selected = None
                return False

        # apply drag logic even if pressed==0 (на некоторых прошивках так идет движение)
        if self.selected == "A":
            # X -> AD, Y -> AB and vertical position of AD
            self._set_near_from_x(x)
            self._set_yA_from_y(y)
            return True

        if self.selected == "B":
            # X -> BC, Y -> AB and vertical position of BC
            self._set_far_from_x(x)
            self._set_yB_from_y(y)
            return True

        return False

# =========================
# Model / Camera / Display
# =========================
detector = nn.YOLOv5(model="/root/models/yolov5s.mud", dual_buff=True)
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()

class_names = {
    0: "Person", 1: "Bicycle", 2: "Car", 3: "Motorbike",
    7: "Truck", 17: "Horse", 18: "Sheep", 19: "Cow"
}

# =========================
# Wi-Fi
# =========================
wifi_manager = WiFiManager()
SSID = "AOG4"
PASSWORD = "12345678"
wifi_connected = wifi_manager.connect(SSID, PASSWORD)

# =========================
# Angle receiver
# =========================
angle_receiver = AngleReceiver()

# =========================
# Zone + touch calibrator
# =========================
zone_config = ZoneConfig(cam.width(), cam.height())
touch_calibrator = TouchCalibrator(cam.width(), cam.height())

print(f"📱 Разрешение: {cam.width()}x{cam.height()}")
print("✅ Запуск: детекция + симметричная трапеция + две точки слева")

touch_count = 0
last_obstacle_print = 0
last_angle_print = 0
steering_angle = 0.0

# =========================
# Main loop
# =========================
while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
    target_objects = [o for o in objs if o.class_id in class_names]

    # angle
    if angle_receiver.receive_angle():
        steering_angle = angle_receiver.current_angle
        now = time.ticks_ms()
        if now - last_angle_print > 1000:
            print(f"📥 Угол от ESP32: {steering_angle:.1f}°")
            last_angle_print = now

    # touch
    if touchscreen and touchscreen.available():
        try:
            td = touchscreen.read()
            if td and len(td) >= 3:
                raw_x, raw_y, pressed = td
                touch_count += 1
                x, y = touch_calibrator.transform_coordinates(raw_x, raw_y)

                if touch_count <= 6:
                    print(f"👆 Touch#{touch_count}: raw({raw_x},{raw_y}) -> ({x},{y}) pressed={pressed}")

                zone_config.handle_touch(x, y, pressed, steering_angle)
        except Exception as e:
            print(f"❌ Ошибка TouchScreen: {e}")

    quad, (A, B, C, D) = zone_config.get_quad_for_tests(steering_angle)

    # objects in zone
    objects_in_zone = []
    for o in target_objects:
        cx = o.x + o.w // 2
        cy = o.y + o.h // 2
        if point_in_quad(cx, cy, quad):
            objects_in_zone.append(o)

    has_obstacle = (len(objects_in_zone) > 0) and zone_config.obstacle_detection_enabled

    # send UDP
    if wifi_connected and zone_config.obstacle_detection_enabled:
        wifi_manager.send_obstacle_data(has_obstacle, len(objects_in_zone), steering_angle)

    # print throttled
    now = time.ticks_ms()
    if has_obstacle and now - last_obstacle_print > 2000:
        detected = []
        for cid in class_names:
            cnt = 0
            for o in objects_in_zone:
                if o.class_id == cid:
                    cnt += 1
            if cnt > 0:
                detected.append(f"{class_names[cid]}:{cnt}")
        status = "📡 UDP OK" if wifi_connected else "❌ Wi-Fi OFF"
        print(f"🚨 ПРЕПЯТСТВИЕ: {', '.join(detected)} | angle={steering_angle:.1f}° | {status}")
        last_obstacle_print = now

    # draw detections
    for o in target_objects:
        if o.class_id == 0:
            color = image.COLOR_GREEN
        elif o.class_id in [1, 2, 3, 7]:
            color = image.COLOR_BLUE
        else:
            color = image.COLOR_YELLOW
        img.draw_rect(o.x, o.y, o.w, o.h, color=color, thickness=2)
        img.draw_string(o.x, max(0, o.y - 15), f"{class_names[o.class_id]}:{o.score:.2f}", color=color, scale=1.2)

    # zone color
    if zone_config.obstacle_detection_enabled:
        zone_color = image.COLOR_RED if has_obstacle else image.COLOR_GREEN
        if zone_config.edit_mode:
            zone_color = image.COLOR_GREEN
    else:
        zone_color = image.COLOR_GRAY

    # draw trapezoid edges: AB, BC, CD, DA
    img.draw_line(A[0], A[1], B[0], B[1], color=zone_color, thickness=3)  # AB (right)
    img.draw_line(B[0], B[1], C[0], C[1], color=zone_color, thickness=3)  # BC (top)
    img.draw_line(C[0], C[1], D[0], D[1], color=zone_color, thickness=3)  # CD (left)
    img.draw_line(D[0], D[1], A[0], A[1], color=zone_color, thickness=3)  # DA (bottom)

    # draw LEFT handles in edit mode
    if zone_config.edit_mode:
        handles = zone_config.get_left_handles(steering_angle)
        for k, (hx, hy) in handles.items():
            is_sel = (zone_config.selected == k)
            c = image.COLOR_RED if is_sel else image.COLOR_YELLOW
            img.draw_rect(hx - 20, hy - 20, 40, 40, color=c, thickness=2)
            img.draw_rect(hx - 16, hy - 16, 32, 32, color=c, thickness=-1)
            img.draw_string(hx + 22, hy - 18, k, color=image.COLOR_WHITE, scale=1.1)

    # buttons
    btn_w, btn_h = 120, 32
    btn_y = 0

    edit_text = "EDIT" if not zone_config.edit_mode else "SAVE"
    edit_color = image.COLOR_BLUE if not zone_config.edit_mode else image.COLOR_GREEN
    img.draw_rect(0, btn_y, btn_w, btn_h, color=edit_color, thickness=3)
    img.draw_string(12, btn_y + 9, edit_text, color=edit_color, scale=0.9)

    det_text = "DETECT ON" if zone_config.obstacle_detection_enabled else "DETECT OFF"
    det_color = image.COLOR_GREEN if zone_config.obstacle_detection_enabled else image.COLOR_RED
    det_x = zone_config.width - btn_w
    img.draw_rect(det_x, btn_y, btn_w, btn_h, color=det_color, thickness=3)
    img.draw_string(det_x + 6, btn_y + 9, det_text, color=det_color, scale=0.7)

    # stats
    wifi_status = "Wi-Fi:ON" if wifi_connected else "Wi-Fi:OFF"
    det_status = "DET:ON" if zone_config.obstacle_detection_enabled else "DET:OFF"
    stats = f"Obj:{len(target_objects)} In:{len(objects_in_zone)} Ang:{steering_angle:.1f} {wifi_status} {det_status}"
    y_pos = zone_config.height - 14
    img.draw_rect(0, y_pos - 2, len(stats) * 6 + 14, 18, color=image.COLOR_BLACK, thickness=-1)
    img.draw_string(4, y_pos, stats, color=image.COLOR_WHITE, scale=0.7)

    # obstacle banner
    if zone_config.obstacle_detection_enabled and has_obstacle:
        img.draw_rect(cam.width() // 2 - 110, 45, 220, 28, color=image.COLOR_RED, thickness=-1)
        img.draw_string(cam.width() // 2 - 100, 52, "OBSTACLE!", color=image.COLOR_WHITE, scale=0.9)
        send_txt = "SENT" if wifi_connected else "Wi-Fi ERR"
        send_color = image.COLOR_GREEN if wifi_connected else image.COLOR_RED
        img.draw_string(cam.width() // 2 - 35, 76, send_txt, color=send_color, scale=0.8)

    if not zone_config.obstacle_detection_enabled:
        img.draw_rect(cam.width() // 2 - 150, 45, 300, 28, color=image.COLOR_GRAY, thickness=-1)
        img.draw_string(cam.width() // 2 - 140, 52, "DETECTION DISABLED", color=image.COLOR_WHITE, scale=0.8)

    disp.show(img)
