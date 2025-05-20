import serial
import serial.tools.list_ports
import time

def run_detection_loop(detection_keyword="White", 
                       detections_needed=15, 
                       window_duration=2, 
                       cooldown_duration=10, 
                       baudrate=115200, 
                       timeout=1):
    
    # Arduino Control
    ports = list(serial.tools.list_ports.comports())
    arduino_port = None
    for port in ports:
        if any(x in port.description for x in ["Arduino", "CH340", "USB Serial Device", "USB-SERIAL"]):
            arduino_port = port.device
            break

    if arduino_port is None:
        print("Arduino not found. Please check the connection.")
        return

    try:
        arduino = serial.Serial(port=arduino_port, baudrate=baudrate, timeout=timeout)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    time.sleep(2)
    print(f"Connected to {arduino_port}. Monitoring status...\n")

    # Status monitor!
    status = "UNKNOWN"
    white_count = 0
    window_start = time.time()
    cooldown = False
    last_toggle_time = 0

    try:
        while True:
            line = arduino.readline().decode('utf-8').strip()
            if not line:
                continue

            now = time.time()

            if cooldown:
                if now - last_toggle_time >= cooldown_duration:
                    cooldown = False
                    window_start = now
                    white_count = 0
                    # print("Cooldown over—resuming detection.")  
                else:
                    continue

            if detection_keyword in line:
                white_count += 1

            if now - window_start >= window_duration:
                if white_count >= detections_needed:
                    new_status = "WORK IN PROGRESS"
                else:
                    new_status = "ERROR"

                if new_status != status:
                    status = new_status
                    print(f"*** STATUS CHANGED: {status} ***")
                    cooldown = True
                    last_toggle_time = now

                window_start = now
                white_count = 0

    except KeyboardInterrupt:
        print("Detection loop stopped by user.")
    finally:
        arduino.close()
        print("Serial connection closed.")
