import tkinter as tk
from tkinter import ttk
import threading
import time
import sys, time
sys.path.insert(1, 'modules')
sys.path.append("/home/jetson/peter/Autonomous-Ai-drone-scripts")
sys.path.append(r"D:\Python Project\ai-drone")
from modules import drone

# ====================
# Tham số mặc định
DEFAULT_SPEED = 1.0
DEFAULT_YAW = 10
TAKEOFF_HEIGHT = 5
# ====================

class DroneController:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Manual Controller")

        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED)
        self.yaw_var = tk.DoubleVar(value=DEFAULT_YAW)

        self.build_ui()

        # Kết nối drone
        threading.Thread(target=self.connect_and_arm, daemon=True).start()

    def build_ui(self):
        frame = ttk.Frame(self.root, padding=20)
        frame.grid()

        ttk.Label(frame, text="Speed (m/s):").grid(row=0, column=0)
        ttk.Entry(frame, textvariable=self.speed_var, width=10).grid(row=0, column=1)

        ttk.Label(frame, text="Yaw (deg):").grid(row=0, column=2)
        ttk.Entry(frame, textvariable=self.yaw_var, width=10).grid(row=0, column=3)

        # Điều khiển hướng
        control_frame = ttk.Frame(frame, padding=10)
        control_frame.grid(row=1, column=0, columnspan=4)

        ttk.Button(control_frame, text="↑", command=self.forward).grid(row=0, column=1)
        ttk.Button(control_frame, text="←", command=self.left).grid(row=1, column=0)
        ttk.Button(control_frame, text="STOP", command=self.stop).grid(row=1, column=1)
        ttk.Button(control_frame, text="→", command=self.right).grid(row=1, column=2)
        ttk.Button(control_frame, text="↓", command=self.backward).grid(row=2, column=1)
        ttk.Button(control_frame, text="←↑", command=self.forward_yaw).grid(row=3, column=3)

        # Yaw điều khiển
        ttk.Button(control_frame, text="↺ Yaw Left", command=self.yaw_left).grid(row=3, column=0)
        ttk.Button(control_frame, text="↻ Yaw Right", command=self.yaw_right).grid(row=3, column=2)

        # Takeoff/Land
        ttk.Button(frame, text="Takeoff", command=self.takeoff).grid(row=4, column=0)
        ttk.Button(frame, text="Land", command=self.land).grid(row=4, column=1)

    def connect_and_arm(self):
        try:
            print("Connecting...")
            drone.connect_drone('0.0.0.0:14550')
            print("Connected")
        except Exception as e:
            print(f"Lỗi kết nối: {e}")

    def takeoff(self):
        try:
            print(f"Taking off {TAKEOFF_HEIGHT}m...")
            drone.arm_and_takeoff(TAKEOFF_HEIGHT)
        except Exception as e:
            print(f"Lỗi cất cánh: {e}")

    def land(self):
        print("Landing...")
        drone.land()

    def forward(self):
        drone.set_velocity_xyH(self.speed_var.get(), 0, 0, 0)
    
    def forward_yaw(self):
        drone.set_velocity_xyH(self.speed_var.get(), self.speed_var.get(), 0, 0)

    def backward(self):
        drone.set_velocity_xyH(-self.speed_var.get(), 0, 0, 0)

    def left(self):
        drone.set_velocity_xyH(0, -self.speed_var.get(), 0, 0)

    def right(self):
        drone.set_velocity_xyH(0, self.speed_var.get(), 0, 0)

    def stop(self):
        drone.set_velocity_xyz(0, 0, 0)

    def yaw_left(self):
        drone.set_velocity_xyH(0, 0, -self.yaw_var.get(), 10)

    def yaw_right(self):
        drone.set_velocity_xyH(0, 0, self.yaw_var.get(), 10)

# ====================
if __name__ == "__main__":
    root = tk.Tk()
    app = DroneController(root)
    root.mainloop()
