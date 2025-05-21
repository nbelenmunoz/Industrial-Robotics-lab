import time
from DynamixerControl import DynamixelController

if __name__ == "__main__":
    motor = DynamixelController(device_name='COM7', dxl_id=1)

motor.open_gripper(0, 50)         # uses default: 0°
time.sleep(2)
motor.close_gripper()        # uses default: 100°
time.sleep(2)
motor.open_gripper(0, 50)


