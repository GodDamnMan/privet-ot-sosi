from time import perf_counter
from can import CAN_Bus
from motors.gyems import GyemsDRC
import numpy as np

def force_compensation_control(motor, external_force, stiffness=30):
    """
    Простой контроллер компенсации внешней силы
    Мотор сопротивляется внешнему воздействию
    """
    q, dq = motor.state['angle'], motor.state['speed']
    
    # Базовая компенсация внешней силы
    force_compensation = external_force
    
    # Добавляем небольшое демпфирование для устойчивости
    damping = -5 * dq
    
    # Возвращаем в положение равновесия (мягкая пружина)
    spring = stiffness * (180 - q)  # возвращаем к 180 градусам
    
    # Итоговое управление
    u = force_compensation + damping + spring
    
    return u

# Параметры
motor_param = {
    'interface': 'can0',
    'id_motor': 0x141,
    'current_limit': 200
}

# Инициализация
bus = CAN_Bus(interface=motor_param['interface'])
motor = GyemsDRC(can_bus=bus, device_id=motor_param['id_motor'])
motor.set_degrees()
motor.current_limit = motor_param['current_limit']
motor.enable()

# Главный цикл
t0 = perf_counter()
external_force = 50  # пример внешней силы

try:
    while perf_counter() - t0 < 10:
        # Здесь должна быть функция чтения реальной силы
        # Например: external_force = read_force_sensor()
        
        u = force_compensation_control(motor, external_force, stiffness=40)
        motor.set_current(u)

except KeyboardInterrupt:
    motor.set_current(0)

finally:
    motor.disable()