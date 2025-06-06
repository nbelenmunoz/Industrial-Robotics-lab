import sys
import time
from dynamixel_sdk import *  # Library DynamixelSDK

# Added open and close function for the gripper
# open_gripper(Angle, Speed)  
# close_gripper(Angle, Speed) 
# If you don't put any specific angle of speed, there are already default ones: 
# The default angle in open function is 0 and in the close one is 100
# The default speed in both is 100


# Example of how to used it in the main code: 

# if __name__ == "__main__":
#     motor = DynamixelController(device_name='COM7', dxl_id=1)
    # motor.open_gripper(0, 50)         # uses default: 0°
    # time.sleep(2)
    # motor.close_gripper()        # uses default: 100°
    # time.sleep(2)
    # motor.open_gripper(0, 50)


class DynamixelController:
    def __init__(self, device_name='COM7', baudrate=1000000, dxl_id=1, protocol_version=1.0):
        self.DEVICENAME = device_name  # Ex. 'COM3' for Windows, '/dev/ttyUSB0' for Linux/Mac
        self.BAUDRATE = baudrate
        self.DXL_ID = dxl_id
        self.PROTOCOL_VERSION = protocol_version

        # Memory addresses Dynamixel AX-18A
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_PRESENT_POSITION = 36
        self.ADDR_MOVING_SPEED = 32
        self.ADDR_TORQUE_LIMIT = 34

        # Control variables
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.MIN_POSITION = 0  # 0° (minimum position value)
        self.MAX_POSITION = 1023  # 300° (maximum position value)
        self.MIN_SPEED = 0  # Minimum speed
        self.MAX_SPEED = 1023  # Maximum speed

        # Initialize communication
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            print("Impossible to open the serial port.")
            sys.exit()

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Impossible to set baudrate.")
            sys.exit()
        # ────── INSERT HERE ──────
        # Set runtime torque limit to maximum (1023 → 100%)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler,
            self.DXL_ID,
            self.ADDR_TORQUE_LIMIT,
            1023
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Error setting torque limit: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            sys.exit()

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler,
            self.DXL_ID,
            self.ADDR_TORQUE_ENABLE,
            1
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Error enabling torque: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            sys.exit()

        print("Motor successfully initialized with 100% torque!")

    def move_motor(self, angle, speed=100):

        if not (0 <= angle <= 300):
            print("Out of range (0-300°) angle")
            return

        if not (self.MIN_SPEED <= speed <= self.MAX_SPEED):
            print("Out of range (0-1023) speed")
            return

        # Convert angle in register value (0-1023)
        goal_position = int((angle / 300) * 1023)

        # Set velocity
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID,
                                                                       self.ADDR_MOVING_SPEED, speed)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Error setting velocity: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return

        # Set target position
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID,
                                                                       self.ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Error setting position: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return

        print(f"Motor moving to {angle}° at speed {speed}")

    def open_gripper(self, angle=0, speed=100):
        self.move_motor(angle, speed)

    def close_gripper(self, angle=100, speed=100):
        self.move_motor(angle, speed)


    def close(self):

        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.portHandler.closePort()
        print("Motor disabled and serial port closed.")


if __name__ == "__main__":
    motor = DynamixelController(device_name='COM3', dxl_id=1)

    try:
        motor.move_motor(150, 200)
        time.sleep(2)  # Wait 2 seconds
        motor.move_motor(90, 300)
        time.sleep(2)
        motor.move_motor(0, 100)
    except KeyboardInterrupt:
        print("\n User interrupt")
    finally:
        motor.close()