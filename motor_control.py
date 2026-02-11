from time import perf_counter
from can import CAN_Bus
from motors.gyems import GyemsDRC

def pd_control(motor, control_param):
    q, dq = motor.state['angle'], motor.state['speed']
    q_des, dq_des = control_param['q_des'], control_param['dq_des']
    Kp, Kd = control_param['Kp'], control_param['Kd']
    
    u = 50 # put your code here
    
    return u

control_param = {'Kp': 10,
                 'Kd': 15,
                 'q_des': 360,
                 'dq_des': 0}

motor_param = { 'interface': 'can0',
                'id_motor': 0x141,
                'current_limit': 200}

bus = CAN_Bus(interface=motor_param['interface'])

print('CAN BUS connected successfully')

motor = GyemsDRC(can_bus=bus, device_id=motor_param['id_motor'])
motor.set_degrees()
motor.current_limit = motor_param['current_limit']
motor.enable()

t0 = perf_counter()
t = []
t_max = 3

print('Motor control starts')
try:
    while perf_counter() - t0 < t_max:
        u = pd_control(motor, control_param)
        motor.set_current(u)

except KeyboardInterrupt:
    motor.set_current(0)
    print('Something happends :(')

finally:
    motor.disable()
