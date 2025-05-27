import tm_packet
import tm_motion_functions_V1_80
from pymodbus.client import ModbusTcpClient
from rich_logging_format import rich_logger
#from ColorSensor import run_detection_loop
from DynamixerControl import DynamixelController


log = rich_logger()

class TM12X:

    def __init__(self, ip, table_name="Default"):
        self.TMSCT = None
        self.ip = ip
        self.modbus = ModbusTcpClient(host=ip, port=502)
        self.modbus.connect()
        self.TMSVR = tm_packet.TMSVR(ip, table_name)
        log.info("Successfully connected to robot Ethernet Slave and Modbus.")
        self.motion_functions = tm_motion_functions_V1_80.TM_Motion_Functions()
        self._tcp_coord = [0.0] * 6
        self._joints = [0.0] * 6
        self.home = [663.90, -156.30, 688.15, 180.00, 90.00, 90.00]

    def connect_listen_node(self, ip=None):
        if ip:
            self.ip = ip
        self.TMSCT = tm_packet.TMSCT(self.ip)
        log.info("Connected to Listen Node")
        return self.TMSCT is not None

    def close_connection(self):
        self.modbus.close()
        self.TMSVR.close()
        if self.TMSCT is not None:
            self.TMSCT.close()

    @property
    def tcp_coord(self):
        try:
            x_response = self.modbus.read_input_registers(7025, count=12)
            if not x_response or not hasattr(x_response, 'registers'):
                raise ValueError("Invalid Modbus response.")
            self._tcp_coord = self.modbus.convert_from_registers(
                x_response.registers, self.modbus.DATATYPE.FLOAT32, "big")
        except Exception as e:
            log.warning(f"Failed to read TCP coordinates: {e}")
        return self._tcp_coord

    @property
    def joints(self):
        x_response = self.modbus.read_input_registers(7013, count=12)
        self._joints = self.modbus.convert_from_registers(x_response.registers, self.modbus.DATATYPE.FLOAT32, "big")
        return self._joints

    def svr_write(self, item, value):
        self.TMSVR.send(item, value)

    def listen_svr_write(self, item, value):
        self.TMSCT.send(f"svr_write({item},{value})")

    def path_from_csv(self, file_path, speed):
        poses = []
        with open(file_path) as path:
            for pose in path.readlines():
                pose = pose.strip("\n")
                pose = pose.split(",")
                if len(pose) == 1:
                    pose = pose[0].split(" ")
                poses.append(pose)
            # print(poses)
        self.path(poses, speed)

    def path(self, poses, speed):
        self.line(poses, speed)
        print("Path in execution")

    def go_home(self, speed):
        self.ptp(self.home, speed)
        print("Going home")

    def wait(self, milli_sec):
        self.TMSCT.send([f"WaitFor({milli_sec})"])
        # not supported by the TMFlow 1.80

    def wait_queue_tag(self, mode=''):
        self.TMSCT.send(f"WaitQueueTag({mode})", queue=False)
        print("Waiting for queue tag")

    def queue_tag(self, tag_number):
        self.TMSCT.send(f"QueueTag({tag_number}, 1)", queue=False)
        print("Waiting for queue tag")

    def stop(self, mode=''):
        self.TMSCT.send(f"StopAndClearBuffer({mode})", queue=False)
        print("Stopped")

    def exit(self, mode=''):
        # print(f"ScriptExit({mode})")
        self.TMSCT.send(f"ScriptExit({mode})", queue=False)
        print("Exiting")

    # **kwargs allows passing extra named arguments as a dictionary.
    def ptp(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.ptp(poses, speed, **kwargs), queue=queue)

    def move_ptp(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.move_ptp(poses, speed, **kwargs), queue=queue)

    def line(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.line(poses, speed, **kwargs), queue=queue)

    def pline(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.pline(poses, speed, **kwargs), queue=queue)

    def move_line(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.move_ptp(poses, speed, **kwargs), queue=queue)

    def circle(self, mid_point, end_point, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.circle(mid_point, end_point, speed, **kwargs), queue=queue)

    def helmet_operation_1(self, speed=20):
        try:
            if not self.TMSCT or not self.modbus.connected:
                raise ConnectionError("Robot connection not ready")

            print("\n==== Helmet Operation Completed ====")

        except Exception as e:
            log.error(f"Operation interrupted: {e}")
            self.stop('0')

        finally:
            self.close_connection()

    def points(self):

        # self.p0 = [189, 580.75, 520.00, -162.35, 10.88, 171.42]  # JPP
        # self.p1 = [-372.69, 728.20, 237.84, 47.88, -5.07, 87.62] #CPP
        # self.p2 = [-127.45, 712.48, 808.93, -177.36, 0.86, 87.60]
        # self.p3 = [178.63, 679.19, 631.38, -129.72, 1.15, 90.34]

        # self.p1 = [-372.69, 728.20, 237.84, 90, 0, 90]
        # self.p2 = [-127.45, 712.48, 808.93, 180, 0, 90]
        # self.p3 = [178.63, 679.19, 631.38, 270, 0, 90]

        # self.p1 = [-304.8, 730.39, 278.52, 56.73, -3.45, 86.19]
        # self.p2 = [-182.30, 698.92, 759.17, 156, -1.42, 84.48]
        # self.p3 = [263.84, 667.39, 621.60, -126.79, -1.10, 82.24]

        # self.p1 = [-318.69, 723.00, 282.80, 53.28, -0.78, 81.83]
        # self.p2 = [-80.77, 717.28, 843.75, 165.91, 3.16, 86.11]
        # self.p3 = [303.53, 705.47, 612.09, -114.94, 4.62, 89.88]

        self.p1 = [-321.49, 716.60, 277.92, 61.96, -2.95, 88.55]

        self.p2 = [-387.34, 736.46, 459.56, 89.87, 1.88, 82.07]

        self.p3 = [-71.12, 711.48, 845.19, 178.54, -0.92, 86.17]

        self.p4 = [229.00, 685.49, 736.52, -127.95, -0.71, 81.21]

        self.p5 = [332.04, 685.69, 556.29, -96.24, 2.77, 85.24]

        self.p6 = [-357.784, 703.569, 507.028, 107.094, 4.221, 95.405]
        self.p7 = [-350.845, 709.014, 563.964, 114.408, -1.597, 91.433]
        self.p8 = [-341.781, 699.429, 598.227, 117.873, -5.413, 91.224]
        self.p9 = [-300.434, 708.608, 663.088, 133.460, -0.056, 89.656]
        self.p10 = [-255.223, 723.291, 718.376, 144.948, 1.437, 79.308]
        self.p11 = [-214.225, 736.380, 742.524, 152.465, 5.622, 80.697]

        # self.ph1 = [-439.065, 709.343, 301.720, 178.089, 0.760, 86.154]  # vertical down
        # self.ph2 = [-349.524, 693.717, 291.047, 117.121, -10251, 102.648]  # Horizontal
        # self.ph3 = [-345.345, 716.598, 252.019, 61.946, -2.952, 88.559]  # Angle
        # self.ph4 = [-311.544, 716.604, 255.895, 61.965, -2.955, 88.559]  # Up
        self.ph5 = [-311.545, 716.600, 277.942, 61.965, -2.956,88.558] # hook

    # JOINTS!!!!
        self.ph1 = [132.673, 24.584, 100.989, -33.516, 89.194, 136.482]  # vertical down
        self.ph2 = [134.910, 24.583, 100.997, 10.266, 40.712, 144.977]  # Horizontal
        self.ph3 = [132.286, 44.951, 81.496, 90.416, 51.709, 208.224]  # Angle
        self.ph4 = [130.524, 43.612, 83.793, 90.318, 50.309, 209.598]  # Up
       # self.ph5 = [130.524, 42.419, 83.320, 91.984, 50.309, 209.597] # hook


if __name__ == "__main__":
    robot = TM12X("192.168.1.2") # Real robot
    # robot = TM12X("127.0.0.1") # Simulation

    if robot.connect_listen_node():

       # robot.go_home(20)
       robot.points()
       motor = DynamixelController(device_name='COM7', dxl_id=1)
       
        # Initial Routine
       motor.open_gripper(65, 20)
    #    input("press enter")
    #    robot.ptp(robot.ph2, speed=20, data_format="JPP")
    #    robot.wait_queue_tag()
       input("press enter")
       robot.ptp(robot.ph3, speed=20, data_format="JPP")  # Try the poses
       robot.wait_queue_tag()
       input("press enter")
       robot.ptp(robot.ph4, speed=5, data_format="JPP")
       robot.wait_queue_tag()
       input("press enter")
       robot.ptp(robot.ph5, speed=5, data_format="CPP")
       robot.wait_queue_tag()
       input("press enter")
       robot.ptp(robot.p1, speed=5, data_format="CPP")
       robot.wait_queue_tag()
       input("press enter")
       motor.close_gripper(180, 300)
       robot.circle(robot.p2, robot.p3, speed=20)
       input("press enter")
       robot.ptp(robot.p3, speed=20)
       robot.circle(robot.p4, robot.p5, speed=20)
