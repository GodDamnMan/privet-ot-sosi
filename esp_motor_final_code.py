#!/usr/bin/env python3
from time import perf_counter, sleep
import serial

from can import CAN_Bus
from motors.gyems import GyemsDRC


# ---------------------------
# Настройки
# ---------------------------
MOTOR_INTERFACE = "can0"
MOTOR_ID = 0x141
CURRENT_LIMIT = 200            # лимит тока (как в вашем коде)
CONTROL_HZ = 200               # частота цикла

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600

SIGNAL_MIN = 0
SIGNAL_MAX = 5000
MAX_OFFSET_DEG = 180.0         # 0..5000 -> 0..180 градусов

# Фильтрация и мягкость
SENSOR_ALPHA = 0.2             # 0..1, больше = быстрее реагирует, меньше = плавнее
TARGET_SLEW_DEG_S = 120.0      # ограничение скорости изменения цели (град/сек)
SERIAL_STALE_SEC = 0.7         # если данных нет дольше этого — считаем сигнал = 0

# PD-регулятор по углу (подбирается)
KP = 3.0
KD = 0.9


# ---------------------------
# Вспомогательные функции
# ---------------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def angle_err_deg(target, current):
    """
    Ошибка угла в диапазоне [-180, 180)
    """
    e = (target - current + 180.0) % 360.0 - 180.0
    return e

def try_update_motor_state(motor):
    """
    На разных версиях библиотек названия методов обновления состояния могут отличаться.
    Пытаемся аккуратно дернуть то, что есть, иначе живём с motor.state как есть.
    """
    for name in ("update", "read_state", "get_state", "refresh", "request_state"):
        if hasattr(motor, name):
            fn = getattr(motor, name)
            try:
                fn()
                return
            except TypeError:
                # если метод требует аргументы — пропускаем
                pass
            except Exception:
                pass

def safe_get_state(motor):
    st = getattr(motor, "state", {}) or {}
    q = float(st.get("angle", 0.0))
    dq = float(st.get("speed", 0.0))
    return q, dq

class ForceSerialReader:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0)  # неблокирующее чтение
        self.last_value = 0
        self.last_ts = perf_counter()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def read_latest(self):
        """
        Возвращает (value, age_sec).
        value — последнее валидное число.
        age_sec — сколько времени прошло с момента последнего валидного числа.
        """
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            print(line)
            if not line:
                continue
            try:
                v = int(line)
                self.last_value = v
                self.last_ts = perf_counter()
            except ValueError:
                # игнорируем мусор
                pass

        age = perf_counter() - self.last_ts
        return self.last_value, age


# ---------------------------
# Главная логика
# ---------------------------
def main():
    # CAN + мотор
    bus = CAN_Bus(interface=MOTOR_INTERFACE)
    motor = GyemsDRC(can_bus=bus, device_id=MOTOR_ID)
    motor.set_degrees()
    motor.current_limit = CURRENT_LIMIT
    motor.enable()

    # Serial
    reader = ForceSerialReader(SERIAL_PORT, SERIAL_BAUD)
    sleep(2.0)  # дать ESP/Serial прогреться

    try:
        # Считать стартовую позицию q0 (что и будет "позиция при 0 сигнале")
        for _ in range(10):
            try_update_motor_state(motor)
            sleep(0.02)
        q0, _ = safe_get_state(motor)

        filtered_signal = 0.0
        target_deg = q0
        last_t = perf_counter()
        dt_target = 1.0 / CONTROL_HZ

        while True:
            t = perf_counter()
            dt = t - last_t
            if dt <= 0:
                dt = dt_target
            last_t = t

            # 1) читаем сигнал
            raw, age = reader.read_latest()
            if age > SERIAL_STALE_SEC:
                raw = 0  # если ESP молчит — уходим в "удержание базовой" позиции
            
            raw = clamp(raw, SIGNAL_MIN, SIGNAL_MAX)


            # 2) фильтруем сигнал
            filtered_signal = (1.0 - SENSOR_ALPHA) * filtered_signal + SENSOR_ALPHA * float(raw)

            # 3) считаем желаемый угол
            s = filtered_signal / float(SIGNAL_MAX)  # 0..1
            desired = q0 + MAX_OFFSET_DEG * s

            # 4) ограничиваем скорость изменения цели
            max_step = TARGET_SLEW_DEG_S * dt
            step = clamp(desired - target_deg, -max_step, +max_step)
            target_deg += step

            # 5) обновляем состояние мотора
            try_update_motor_state(motor)
            q, dq = safe_get_state(motor)

            # 6) PD по углу
            e = angle_err_deg(target_deg, q)
            u = KP * e - KD * dq

            # ограничение по току
            u = clamp(u, -CURRENT_LIMIT, +CURRENT_LIMIT)
            motor.set_current(u)

            # 7) частота цикла
            sleep(max(0.0, dt_target - (perf_counter() - t)))

    except KeyboardInterrupt:
        pass
    finally:
        try:
            motor.set_current(0)
        except Exception:
            pass
        try:
            motor.disable()
        except Exception:
            pass
        reader.close()


if __name__ == "__main__":
    main()