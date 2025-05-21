import tm_packet
import tm_motion_functions_V1_80
from pymodbus.client import ModbusTcpClient
from rich_logging_format import rich_logger
from color_sensor import run_detection_loop
import time
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
        # self.home = [663.90, -156.3, 688.15, 180.00, 0.00, 90.00]
        self.home = [189, 580.75, 520.00, -162.35, 10.88, 171.42]
        #self.p1 = [-372.69, 728.20, 237.84, 47.88, -5.07, 87.62]
        #self.p2 = [-127.45, 712.48, 808.93, -177.36, 0.86, 87.60]
        #self.p3 = [178.63, 679.19, 631.38, -129.72, 1.15, 90.34]

        self.p1 = [-372.69, 728.20, 237.84, 47.88, 0, 90]
        self.p2 = [-127.45, 712.48, 808.93, 179.0, 0, 90]
        self.p3 = [178.63, 679.19, 631.38, -129.72, 0, 90]
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

    def pline(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.pline(poses, speed, **kwargs), queue=queue)

    def line(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.line(poses, speed, **kwargs), queue=queue)

    def circle(self, mid_point, end_point, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.circle(mid_point, end_point, speed, **kwargs), queue=queue)

    def move_ptp(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.move_ptp(poses, speed, **kwargs), queue=queue)

    def move_line(self, poses, speed, queue=False, **kwargs):
        self.TMSCT.send(self.motion_functions.move_ptp(poses, speed, **kwargs), queue=queue)

    def helmet_operation_1(self, speed=20):
        try:
            if not self.TMSCT or not self.modbus.connected:
                raise ConnectionError("Robot connection not ready")

            self.go_home(speed)
            self.wait_queue_tag()

            # # Open visor switch
            # # Big adjustment
            # robot.ptp(p, speed=20)
            # self.wait_queue_tag()
            # # Slight adjustment
            # robot.ptp(p, speed=20)
            # self.wait_queue_tag()
            # # Small circle
            # robot.circle(p, p, speed=20)
            # self.wait_queue_tag()
            #
            # # Open Chin guard switch
            # # Big adjustment
            # robot.ptp(p, speed=20)
            # self.wait_queue_tag()
            # # Slight adjustment
            # robot.ptp(p, speed=20)
            # self.wait_queue_tag()
            # # Motor
            # # Big circle
            # robot.circle(p, p, speed=20)
            # self.wait_queue_tag()

            # p1 = [399.32, -455.44, 441.06, -168.48, -59.52, 172.18]
            # p2 = [275.75, -437.00, 717.88, -179.05, -20.03, 179.79]
            # p3 = [-94.15, -370.80, 529.62, 154.39, 1.61, 81.80]
            # robot.ptp(p1, speed=20)
            # robot.circle(p2, p3, speed=20)

            self.go_home(speed)
            self.wait_queue_tag()
            print("\n==== Helmet Operation Completed ====")

        except Exception as e:
            log.error(f"Operation interrupted: {e}")
            self.stop('0')
        finally:
            self.close_connection()



    def execute_demo_path(self):
        try:
            # print(f"Current TCP coordinates: {self.tcp_coord}")
            # print(f"Current joint angles: {self.joints}")

            print("Moving to home...")
            self.go_home(speed=20)
            self.wait_queue_tag()

            # Define demonstration path (helmet operation positions)
            demo_path = [
                [399.32, -455.44, 441.06, -168.48, -59.52, 172.18],
                [275.75, -437.00, 717.88, -179.05, -20.03, 179.79],
                [-94.15, -370.80, 529.62, 154.39, 1.61, 81.80],
            ]

            # Execute point-to-point motion
            print("Executing PTP demonstration path...")
            for i, pos in enumerate(demo_path):
                print(f"Moving to position {i + 1}: {pos}")
                self.ptp(pos, speed=20)
                self.wait_queue_tag()

            print("Demonstration path completed successfully!")

        except Exception as e:
            print(f"Error during demonstration: {str(e)}")

        finally:
            self.close_connection()

if __name__ == "__main__":
    #robot = TM12X("192.168.1.2") # Real robot
    robot = TM12X("127.0.0.1") # Simulation
    if robot.connect_listen_node():
       # robot.helmet_operation_1()
       # robot.execute_demo_path()
       # robot.go_home(speed=20)
       # robot.wait_queue_tag()

       robot.ptp(robot.p1, speed=20)
       # robot.wait_queue_tag()
       # robot.ptp(robot.p2, speed=20)
       # robot.wait_queue_tag()
       # robot.ptp(robot.p3, speed=20)
       # robot.wait_queue_tag()
       robot.circle(robot.p2, robot.p3, speed=10)
       # robot.path_from_csv("arc_trajectory.csv", 20)

       #def circle(self, mid_point, end_point, speed, queue=False, **kwargs):
         # self.TMSCT.send(self.motion_functions.circle(mid_point, end_point, speed, **kwargs), queue=queue)

       # robot.wait_queue_tag()
       # robot.go_home(speed=20)

