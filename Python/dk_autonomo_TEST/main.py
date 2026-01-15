import cv2
import time
import numpy as np
import ArducamDepthCamera as ac
from dronekit import connect, VehicleMode

from config import *
from dk_passo1 import passo_1_step


def arm_and_takeoff(vehicle, alt):
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)

    vehicle.simple_takeoff(alt)
    while vehicle.location.global_relative_frame.alt < alt * 0.95:
        time.sleep(0.5)


def main():
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
    arm_and_takeoff(vehicle, TARGET_ALTITUDE_M)

    cam = ac.ArducamCamera()
    cam.open(ac.Connection.CSI, 0)
    cam.start(ac.FrameType.DEPTH)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

    cv2.namedWindow("preview")

    token = 0  # 0=manuale, 1=passo1

    while True:
        blank = np.zeros((480, 640, 3), dtype=np.uint8)

        if token == 1:
            done = passo_1_step(vehicle, cam, blank)
            if done:
                token = 0

        cv2.putText(blank, f"TOKEN: {token}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 255), 2)

        cv2.imshow("preview", blank)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("y"):
            token = 1
        elif key == ord("l"):
            vehicle.mode = VehicleMode("LAND")
        elif key == ord("q"):
            break

    vehicle.close()
    cam.stop()
    cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
