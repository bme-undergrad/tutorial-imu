import serial
import time
from datetime import datetime

# --- Ajuste aqui ---
PORT = "COM7"      # Windows: "COMx" (confirme no Gerenciador de Dispositivos)
# PORT = "/dev/rfcomm0"  # Linux
BAUD = 115200      # Irrelevante p/ SPP, mas precisa de um valor

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Conectado a {PORT}")

fname = f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
with open(fname, "w", encoding="utf-8") as f:
    # Cabeçalho (opcional)
    f.write("pitch,roll,yaw,ax,ay,az,gx,gy,gz,t_ms\n")
    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                # Visualiza no console
                print(line)
                # Salva bruto (já vem no formato CSV do ESP32)
                f.write(line + "\n")
                f.flush()
    except KeyboardInterrupt:
        pass

ser.close()
print("Encerrado.")
print("Arquivo salvo em:", fname)
