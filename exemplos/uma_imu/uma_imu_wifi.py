# minimal_tcp_client_save.py
import socket, time

HOST = "192.168.0.123"   # IP do ESP32
PORT = 8080

fname = f"imu_{int(time.time())}.csv"
with socket.create_connection((HOST, PORT)) as s, open(fname, "w", encoding="utf-8") as out:
    out.write("pitch,roll,yaw,ax,ay,az,gx,gy,gz,t_ms\n")  # opcional
    f = s.makefile("r", encoding="utf-8", newline="\n")
    for line in f:
        line = line.strip()
        print(line)
        out.write(line + "\n")
        out.flush()
