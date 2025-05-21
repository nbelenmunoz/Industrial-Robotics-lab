from color_sensor import run_detection_loop

if __name__ == "__main__":
    run_detection_loop(
        detection_keyword="White",
        detections_needed=10,
        window_duration=2,
        cooldown_duration=5
    )
