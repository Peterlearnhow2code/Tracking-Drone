import sys, time
import argparse

sys.path.append(r"D:\Python Project\ai-drone\modules")
# import cv2
# import collections

# from modules import detector_mobilenet as detector
# from modules import vision
# from modules import control
from modules import drone

def main():
    print("Connect through UART")
    drone.connect_drone('COM14')
    print("Get version")
    drone.get_version()
    print("Get EKF status")
    drone.get_EKF_status()
    print("Get home location")
    drone.get_home_location()
    print("Get heading")
    drone.get_heading()

if __name__ == "__main__":
    main()