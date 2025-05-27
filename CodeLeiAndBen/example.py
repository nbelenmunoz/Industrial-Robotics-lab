import techman as tm
import time

def main():
    robot = TM12X("127.0.0.1")

    # Run the TMFlow program with the listen node before running the connect_listen_node function
    robot.connect_listen_node()

    p1 = [-318.69, 723.00, 282.80, 53.28, -0.78, 81.83]
    p2 = [-80.77, 717.28, 843.75, 165.91, 3.16, 86.11]
    p3 = [303.53, 705.47, 612.09, -114.94, 4.62, 89.88]

    robot.ptp(robot.p1, speed=20)

    TM5.close_connection()




if __name__ == "__main__":
    main()