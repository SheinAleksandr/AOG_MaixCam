from maix import camera, display, image, nn, app, gpio, pinmap, sys, err, uart, time, network
from maix.touchscreen import TouchScreen
import socket

device_id = sys.device_id()

# Инициализация TouchScreen
try:
    touchscreen = TouchScreen()
    print("✅ TouchScreen инициализирован")
except Exception as e:
    print(f"Ошибка инициализации TouchScreen: {e}")
    touchscreen = None

# Настройка Wi-Fi
class WiFiManager:
    def __init__(self):
        self.wifi = network.wifi.Wifi()
        self.udp_socket = None
        self.esp32_ip = "192.168.4.1"  # IP ESP32 в режиме AP
        self.esp32_port = 8888
        self.connected = False
        
    def connect(self, ssid, password, timeout=30):
        print(f"📡 Подключение к Wi-Fi: {ssid}")
        try:
            # Пытаемся подключиться
            e = self.wifi.connect(ssid, password, wait=True, timeout=timeout)
            if e == 0:
                self.connected = True
                new_ip = self.wifi.get_ip()
                print(f"✅ Wi-Fi подключен! IP: {new_ip}")
                
                # Создаем UDP сокет для отправки на ESP32
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print(f"✅ UDP сокет создан для отправки на {self.esp32_ip}:{self.esp32_port}")
                return True
            else:
                print(f"❌ Ошибка подключения Wi-Fi: {e}")
                return False
                
        except Exception as e:
            print(f"❌ Ошибка настройки Wi-Fi: {e}")
            return False
    
    def send_obstacle_data(self, has_obstacle, obstacle_count, steering_angle):
        """Отправка данных о препятствии на ESP32 по UDP"""
        if not self.connected or self.udp_socket is None:
            return False
            
        try:
            # Формат: "OBSTACLE:1:COUNT:2:ANGLE:12.5"
            message = f"OBSTACLE:{1 if has_obstacle else 0}:COUNT:{obstacle_count}:ANGLE:{steering_angle:.1f}"
            self.udp_socket.sendto(message.encode('utf-8'), (self.esp32_ip, self.esp32_port))
            return True
        except Exception as e:
            print(f"❌ Ошибка отправки UDP: {e}")
            return False

# Класс для приема угла от ESP32
class AngleReceiver:
    def __init__(self):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(0.1)  # Таймаут 100мс
        self.udp_socket.bind(("0.0.0.0", 8889))  # Слушаем порт 8889 для приема угла
        self.current_angle = 0.0
        
    def receive_angle(self):
        try:
            data, addr = self.udp_socket.recvfrom(64)
            message = data.decode('utf-8').strip()
            
            if message.startswith("ANGLE:"):
                angle_str = message.replace("ANGLE:", "").strip()
                try:
                    self.current_angle = float(angle_str)
                    return True
                except ValueError:
                    pass
        except socket.timeout:
            pass
        except Exception as e:
            print(f"❌ Ошибка приема угла: {e}")
            
        return False

# Класс для калибровки TouchScreen
class TouchCalibrator:
    def __init__(self, display_width, display_height):
        self.display_width = display_width
        self.display_height = display_height
        print(f"📐 Калибратор инициализирован для дисплея {display_width}x{display_height}")
        
    def transform_coordinates(self, x, y):
        """Преобразование координат touchscreen в координаты дисплея"""
        display_x = int(x * (self.display_width / 640.0))
        display_y = int(y * (self.display_height / 480.0))
        
        display_x = max(0, min(display_x, self.display_width - 1))
        display_y = max(0, min(display_y, self.display_height - 1))
        
        return display_x, display_y

# Класс для настройки зоны
class ZoneConfig:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.x1_ratio = 0.25
        self.x2_ratio = 0.75
        self.y1_ratio = 0.2
        self.y2_ratio = 0.9
        self.max_shift_ratio = 0.3
        self.edit_mode = False
        self.selected_corner = None
        self.touch_threshold = 50
        self.last_touch_time = 0
        self.touch_cooldown = 100
        self.obstacle_detection_enabled = True  # Флаг включения обнаружения препятствий
        
    def get_zone(self, steering_angle=0):
        base_x1 = int(self.width * self.x1_ratio)
        base_x2 = int(self.width * self.x2_ratio)
        base_y1 = int(self.height * self.y1_ratio)
        base_y2 = int(self.height * self.y2_ratio)
        
        max_shift = int(self.width * self.max_shift_ratio)
        max_angle = 45.0
        clamped_angle = max(-max_angle, min(steering_angle, max_angle))
        shift = int((clamped_angle / max_angle) * max_shift)  # ДИНАМИЧЕСКИЙ СДВИГ
        
        x1 = max(0, base_x1 + shift)  # ЗОНА СДВИГАЕТСЯ В ЗАВИСИМОСТИ ОТ УГЛА
        x2 = min(self.width, base_x2 + shift)
        if x2 <= x1:
            x2 = x1 + int(self.width * 0.3)
            
        return x1, base_y1, x2, base_y2
    
    def get_corner_coords(self, corner_idx):
        x1 = int(self.width * self.x1_ratio)
        x2 = int(self.width * self.x2_ratio)
        y1 = int(self.height * self.y1_ratio)
        y2 = int(self.height * self.y2_ratio)
        
        corners = [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]
        return corners[corner_idx]
    
    def move_corner_to_position(self, corner_idx, x, y):
        new_x_ratio = x / self.width
        new_y_ratio = y / self.height
        
        min_size = 0.1
        
        if corner_idx == 0:
            self.x1_ratio = max(0.0, min(new_x_ratio, self.x2_ratio - min_size))
            self.y1_ratio = max(0.0, min(new_y_ratio, self.y2_ratio - min_size))
        elif corner_idx == 1:
            self.x2_ratio = max(self.x1_ratio + min_size, min(new_x_ratio, 1.0))
            self.y1_ratio = max(0.0, min(new_y_ratio, self.y2_ratio - min_size))
        elif corner_idx == 2:
            self.x1_ratio = max(0.0, min(new_x_ratio, self.x2_ratio - min_size))
            self.y2_ratio = max(self.y1_ratio + min_size, min(new_y_ratio, 1.0))
        elif corner_idx == 3:
            self.x2_ratio = max(self.x1_ratio + min_size, min(new_x_ratio, 1.0))
            self.y2_ratio = max(self.y1_ratio + min_size, min(new_y_ratio, 1.0))
        
        print(f"📐 Угол {corner_idx} перемещен в X={x}, Y={y}")
    
    def toggle_obstacle_detection(self):
        """Переключение состояния обнаружения препятствий"""
        self.obstacle_detection_enabled = not self.obstacle_detection_enabled
        status = "ВКЛЮЧЕНО" if self.obstacle_detection_enabled else "ВЫКЛЮЧЕНО"
        print(f"🎯 Обнаружение препятствий: {status}")
        return self.obstacle_detection_enabled
    
    def handle_touch(self, x, y, pressed):
        current_time = time.ticks_ms()
        if current_time - self.last_touch_time < self.touch_cooldown:
            return False
            
        self.last_touch_time = current_time
        
        if not self.edit_mode:
            # Кнопка редактирования зоны (СЛЕВА)
            edit_button_width = 100
            edit_button_height = 30
            edit_button_x = 10  # Слева
            edit_button_y = 10  # Вверху
            
            # Кнопка включения/выключения обнаружения препятствий (СПРАВА)
            obstacle_button_width = 100
            obstacle_button_height = 30
            obstacle_button_x = self.width - obstacle_button_width - 10  # Справа
            obstacle_button_y = 10  # Вверху
            
            # Проверка касания кнопки редактирования
            if (edit_button_x <= x <= edit_button_x + edit_button_width and 
                edit_button_y <= y <= edit_button_y + edit_button_height):
                self.edit_mode = True
                self.selected_corner = None
                print("📐 Режим редактирования зоны включен")
                return True
            
            # Проверка касания кнопки обнаружения препятствий
            if (obstacle_button_x <= x <= obstacle_button_x + obstacle_button_width and 
                obstacle_button_y <= y <= obstacle_button_y + obstacle_button_height):
                self.toggle_obstacle_detection()
                return True
            return False
        
        if pressed == 1:
            # Кнопка сохранения зоны (СЛЕВА)
            save_button_width = 100
            save_button_height = 30
            save_button_x = 0  # Слева
            save_button_y = 0  # Вверху
            
            # Кнопка включения/выключения обнаружения препятствий (СПРАВА)
            obstacle_button_width = 100
            obstacle_button_height = 30
            obstacle_button_x = self.width - obstacle_button_width - 0  # Справа
            obstacle_button_y = 0  # Вверху
            
            # Проверка касания кнопки сохранения
            if (save_button_x <= x <= save_button_x + save_button_width and 
                save_button_y <= y <= save_button_y + save_button_height):
                self.edit_mode = False
                self.selected_corner = None
                print("💾 Режим редактирования зоны выключен")
                return True
            
            # Проверка касания кнопки обнаружения препятствий
            if (obstacle_button_x <= x <= obstacle_button_x + obstacle_button_width and 
                obstacle_button_y <= y <= obstacle_button_y + obstacle_button_height):
                self.toggle_obstacle_detection()
                return True
            
            for i in range(4):
                corner_x, corner_y = self.get_corner_coords(i)
                distance = ((x - corner_x) ** 2 + (y - corner_y) ** 2) ** 0.5
                if distance < self.touch_threshold:
                    self.selected_corner = i
                    print(f"🎯 Выбран угол {i}")
                    self.move_corner_to_position(i, x, y)
                    return True
            
            if self.selected_corner is not None:
                self.move_corner_to_position(self.selected_corner, x, y)
                return True
            
            self.selected_corner = None
            return False
        
        return False

# === Загрузка модели ===
detector = nn.YOLOv5(model="/root/models/yolov5s.mud", dual_buff=True)
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()

class_names = {
    0: "Person", 1: "Bicycle", 2: "Car", 3: "Motorbike", 
    7: "Truck", 17: "Horse", 18: "Sheep", 19: "Cow"
}

# Инициализация Wi-Fi
wifi_manager = WiFiManager()

# Настройки Wi-Fi
SSID = "AOG4"
PASSWORD = "12345678"

# Подключаемся к Wi-Fi
wifi_connected = wifi_manager.connect(SSID, PASSWORD)

# Инициализация приемника угла
angle_receiver = AngleReceiver()

# Инициализация конфигуратора зоны и калибратора
zone_config = ZoneConfig(cam.width(), cam.height())
touch_calibrator = TouchCalibrator(cam.width(), cam.height())

print(f"📱 Разрешение дисплея: {cam.width()}x{cam.height()}")
print("✅ Система запущена! Детекция объектов и передача по Wi-Fi...")

# Статистика
touch_count = 0
last_obstacle_print = 0
last_angle_print = 0
steering_angle = 0.0  # Угол по умолчанию

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
    
    target_objects = [obj for obj in objs if obj.class_id in class_names]
    
    # --- ПРИЕМ УГЛА ОТ ESP32 ---
    if angle_receiver.receive_angle():
        steering_angle = angle_receiver.current_angle
        # Для отладки - выводим угол раз в секунду
        current_time = time.ticks_ms()
        if current_time - last_angle_print > 1000:
            print(f"📥 Получен угол от ESP32: {steering_angle}°")
            last_angle_print = current_time
    
    # --- Обработка касаний TouchScreen ---
    if touchscreen and touchscreen.available():
        try:
            touch_data = touchscreen.read()
            if touch_data and len(touch_data) >= 3:
                raw_x, raw_y, pressed = touch_data
                touch_count += 1
                
                display_x, display_y = touch_calibrator.transform_coordinates(raw_x, raw_y)
                
                if touch_count <= 5:
                    print(f"👆 Касание #{touch_count}: raw({raw_x}, {raw_y}) -> display({display_x}, {display_y})")
                
                if pressed == 1:
                    zone_config.handle_touch(display_x, display_y, pressed)
                    
        except Exception as e:
            print(f"❌ Ошибка чтения TouchScreen: {e}")

    # --- Получение ДИНАМИЧЕСКОЙ зоны с учетом угла ---
    x1, y1, x2, y2 = zone_config.get_zone(steering_angle)

    # --- Объекты в зоне ---
    objects_in_zone = []
    for obj in target_objects:
        cx = obj.x + obj.w // 2
        cy = obj.y + obj.h // 2
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            objects_in_zone.append(obj)

    # --- Отправка сигнала о препятствии по Wi-Fi ---
    has_obstacle = len(objects_in_zone) > 0 and zone_config.obstacle_detection_enabled
    
    if wifi_connected and zone_config.obstacle_detection_enabled:
        wifi_manager.send_obstacle_data(has_obstacle, len(objects_in_zone), steering_angle)
    
    # Вывод в консоль при обнаружении препятствия
    current_time = time.ticks_ms()
    if has_obstacle and current_time - last_obstacle_print > 2000:
        detected = [f"{class_names[oid]}: {len([o for o in objects_in_zone if o.class_id == oid])}"
                   for oid in class_names if any(o.class_id == oid for o in objects_in_zone)]
        status = "📡 Отправлено по Wi-Fi" if wifi_connected else "❌ Wi-Fi не подключен"
        print(f"🚨 ПРЕПЯТСТВИЕ: {', '.join(detected)} | Угол: {steering_angle}° | {status}")
        last_obstacle_print = current_time

    # --- Визуализация ---
    for obj in target_objects:
        color = image.COLOR_GREEN if obj.class_id == 0 else \
                image.COLOR_BLUE if obj.class_id in [1,2,3,7] else \
                image.COLOR_YELLOW
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color, thickness=2)
        msg = f"{class_names[obj.class_id]}: {obj.score:.2f}"
        img.draw_string(obj.x, obj.y - 15, msg, color=color, scale=1.2)

    # --- Отрисовка ДИНАМИЧЕСКОЙ зоны ---
    if zone_config.obstacle_detection_enabled:
        zone_color = image.COLOR_RED if has_obstacle else image.COLOR_GREEN
        if zone_config.edit_mode:
            zone_color = image.COLOR_GREEN
    else:
        zone_color = image.COLOR_GRAY  # Серый цвет, когда обнаружение выключено
        
    img.draw_rect(x1, y1, x2 - x1, y2 - y1, color=zone_color, thickness=3)
    
    # --- Отрисовка углов для редактирования ---
    if zone_config.edit_mode:
        for i in range(4):
            corner_x, corner_y = zone_config.get_corner_coords(i)
            color = image.COLOR_RED if i == zone_config.selected_corner else image.COLOR_YELLOW
            img.draw_rect(corner_x - 25, corner_y - 25, 50, 50, color=color, thickness=2)
            img.draw_rect(corner_x - 20, corner_y - 20, 40, 40, color=color, thickness=-1)
            img.draw_string(corner_x + 26, corner_y - 23, str(i), color=image.COLOR_WHITE, scale=1.2)

    # --- Кнопка редактирования зоны (СЛЕВА ВВЕРХУ) ---
    button_text = "EDIT ZONE" if not zone_config.edit_mode else "SAVE ZONE"
    button_color = image.COLOR_BLUE if not zone_config.edit_mode else image.COLOR_GREEN
    
    button_width = 100
    button_height = 30
    button_x = 0  # Слева
    button_y = 0  # Вверху
    
    img.draw_rect(button_x, button_y, button_width, button_height, color=button_color, thickness=3)
    
    text_x = button_x + (button_width - len(button_text) * 8) // 2
    text_y = button_y + 8
    img.draw_string(text_x, text_y, button_text, color=button_color, scale=0.8)

    # --- Кнопка включения/выключения обнаружения препятствий (СПРАВА ВВЕРХУ) ---
    obstacle_button_text = "DETECT ON" if zone_config.obstacle_detection_enabled else "DETECT OFF"
    obstacle_button_color = image.COLOR_GREEN if zone_config.obstacle_detection_enabled else image.COLOR_RED
    
    obstacle_button_width = 100
    obstacle_button_height = 30
    obstacle_button_x = zone_config.width - obstacle_button_width - 0  # Справа
    obstacle_button_y = 0  # Вверху
    
    img.draw_rect(obstacle_button_x, obstacle_button_y, obstacle_button_width, obstacle_button_height, color=obstacle_button_color, thickness=3)
    
    obstacle_text_x = obstacle_button_x + (obstacle_button_width - len(obstacle_button_text) * 8) // 2
    obstacle_text_y = obstacle_button_y + 8
    img.draw_string(obstacle_text_x, obstacle_text_y, obstacle_button_text, color=obstacle_button_color, scale=0.8)

    # --- Статистика на экране (ВНИЗУ) ---
    wifi_status = "Wi-Fi: ON" if wifi_connected else "Wi-Fi: OFF"
    detect_status = "DETECT: ON" if zone_config.obstacle_detection_enabled else "DETECT: OFF"
    stats_text = f"Objects: {len(target_objects)} | Zone: {len(objects_in_zone)} | Angle: {steering_angle:.1f} | {wifi_status}"
    
    y_pos = zone_config.height - 10  # Внизу экрана
    img.draw_rect(5, y_pos - 2, len(stats_text) * 6 + 10, 18, color=image.COLOR_BLACK, thickness=-1)
    img.draw_string(0, y_pos, stats_text, color=image.COLOR_WHITE, scale=0.7)
    
    # Добавляем индикатор препятствия (только если обнаружение включено)
    if has_obstacle and zone_config.obstacle_detection_enabled:
        warning_text = "OBSTACLE DETECTED!"
        img.draw_rect(cam.width()//2 - 100, 50, 200, 25, color=image.COLOR_RED, thickness=-1)
        img.draw_string(cam.width()//2 - 90, 53, warning_text, color=image.COLOR_WHITE, scale=0.8)
        # Добавляем статус отправки
        send_status = "SENT TO ESP32" if wifi_connected else "Wi-Fi ERROR"
        status_color = image.COLOR_GREEN if wifi_connected else image.COLOR_RED
        img.draw_string(cam.width()//2 - 70, 75, send_status, color=status_color, scale=0.7)
    
    # Показываем статус, если обнаружение выключено
    if not zone_config.obstacle_detection_enabled:
        status_text = "OBSTACLE DETECTION DISABLED"
        img.draw_rect(cam.width()//2 - 120, 50, 240, 25, color=image.COLOR_GRAY, thickness=-1)
        img.draw_string(cam.width()//2 - 110, 53, status_text, color=image.COLOR_WHITE, scale=0.7)
       
    disp.show(img)