import techman as tm
import time

def main():
    robot_ip = ("127.0.0.1")
    TM5 = tm.TM_Robot(robot_ip)

    TM5.connect_listen_node()

    P1 = [370, 300, 60, 150, 0, 90]
    P2 = [370, 600, 60, 150, 0, 90]
    path = input("Press enter to start")

    for i in range(3):
        TM5.ptp(P1, 10)

        input("press enter to scan")
        TM5.line(P2, 100)
        time.sleep(0.5)

        P1[3] += 30
        P2[3] += 30

    input("press enter to close connection")

    TM5.close_connection()

if __name__ == "__main__":
    main()