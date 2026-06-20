# MaixCAM2 -> ВКонтакте: Wi-Fi (хотспот телефона) + RTMP + звук, всё в одном.
#
# 1) Включи на телефоне хотспот И мобильный интернет.
# 2) Впиши ниже SSID/PASSWORD хотспота и RTMP_URL из ВК (постоянный ключ).
# 3) Запусти. Камера сама подключится к телефону и начнёт пуш на ВК.

import sys
from maix import camera, time, rtmp, image, app, audio, network

# ===== Wi-Fi: точка доступа телефона (с мобильным интернетом) =====
SSID     = "TractorAi"      # <-- имя хотспота телефона
PASSWORD = "12345678"       # <-- пароль хотспота

# ===== RTMP-ссылка ВК (сервер + ПОЛНЫЙ постоянный ключ одной строкой) =====
RTMP_URL = "rtmp://............/................"

WIDTH, HEIGHT = 640, 480
BITRATE = 800000               # при обрывах снижай: 600000 / 400000

# ---------- подключение к хотспоту ----------
def connect_wifi(ssid, password):
    w = network.wifi.Wifi()
    print(f"Wi-Fi: подключаюсь к '{ssid}' ...")
    for i in range(1, 6):
        try:
            if w.connect(ssid, password, wait=True, timeout=30) == 0:
                ip = w.get_ip()
                print(f"Wi-Fi: OK, ip = {ip}")
                return ip
        except Exception as e:
            print(f"Wi-Fi: попытка {i} не удалась: {e}")
        time.sleep(2)
    print("Wi-Fi: НЕ подключился. Проверь хотспот/пароль.")
    return None

ip = connect_wifi(SSID, PASSWORD)
if not ip:
    print("Без сети дальше смысла нет — выходим.")
    sys.exit(1)

# ---------- разбор RTMP-ссылки ----------
_arg1 = sys.argv[1].strip() if len(sys.argv) > 1 else ""
url = _arg1 if _arg1.startswith("rtmp://") else RTMP_URL
if len(sys.argv) > 2 and sys.argv[2].isdigit():
    BITRATE = int(sys.argv[2])
if not url.startswith("rtmp://") or "ВСТАВЬ_ПОЛНЫЙ_КЛЮЧ" in url:
    print("Впиши настоящую RTMP-ссылку ВК в RTMP_URL.")
    sys.exit(1)

rest = url[len("rtmp://"):]
hostport, _, path = rest.partition("/")
if ":" in hostport:
    host, p = hostport.split(":", 1)
    port = int(p)
else:
    host, port = hostport, 1935
app_name, _, stream = path.partition("/")
print("host=", host, "port=", port, "app=", app_name, "bitrate=", BITRATE)

# ---------- камера + RTMP + звук ----------
cam = camera.Camera(WIDTH, HEIGHT, image.Format.FMT_YVU420SP)   # RTMP требует NV21
r = rtmp.Rtmp(host, port, app_name, stream, BITRATE)
r.bind_camera(cam)
try:
    rec = audio.Recorder()
    r.bind_audio_recorder(rec)
    print("audio = ON")
except Exception as e:
    print("audio = OFF (", e, ")")

ret = r.start()
print("rtmp start ret =", ret, "(0 = ok)")
print("Пуш запущен. В ВК дождись превью и нажми 'Запустить трансляцию'.")

while not app.need_exit():
    time.sleep(1)