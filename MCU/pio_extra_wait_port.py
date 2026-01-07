# pio_extra_wait_port.py
from SCons.Script import Import
Import("env")
import os, time

# ใส่ flag เฉพาะกับ C++ (ไม่ลงกับ C)
env.Append(CXXFLAGS=["-fno-rtti"])
# ถูกเรียกโดย SCons ด้วยคีย์เวิร์ด: source=..., target=..., env=...
def wait_port(source, target, env):
    # อ่านพอร์ตจาก platformio.ini (upload_port)
    port = env.GetProjectOption("upload_port")
    deadline = time.time() + 15.0
    last_err = ""

    while time.time() < deadline:
        if os.path.exists(port):
            try:
                # ลอง open/close แบบ low-level ให้ชัวร์ว่าไม่ได้ถูกยึด
                fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK)
                os.close(fd)
                print("PORT READY:", port)
                return None
            except Exception as e:
                last_err = str(e)
        time.sleep(0.2)

    print("WARN: port not ready:", port, last_err)
    return None

# hook ก่อน target "upload"
env.AddPreAction("upload", wait_port)
