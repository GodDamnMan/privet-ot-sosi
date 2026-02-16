import serial
import time

# Настройка последовательного порта
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Замените COM3 на ваш порт
time.sleep(2)  # Ждем инициализации порта

try:
    while True:
        if ser.in_waiting > 0:
            # Читаем строку и декодируем
            line = ser.readline().decode('utf-8').strip()
            print(f"Получено число: {line}")
            if not line:
                continue
            try:
                v = int(line)
            except ValueError:
                # игнорируем мусор
                pass
            # Если нужно преобразовать в число:
            # number = int(data)
            # print(f"Число: {number}")
            
except KeyboardInterrupt:
    print("Программа остановлена")
finally:
    ser.close()