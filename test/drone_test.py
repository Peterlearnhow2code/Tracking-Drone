import sys, time
sys.path.insert(1, 'modules')
sys.path.append("/home/jetson/peter/Autonomous-Ai-drone-scripts")
sys.path.append(r"D:\Python Project\ai-drone")
from modules import drone

# === CONFIG ===
height = 2
speed = 3       # m/s
size = 40       # meters (kích thước cạnh chính xác)
yaw_angle = 90
yaw_rate = 10
# ==============

def stop_drone():
    for _ in range(20):
        drone.set_velocity_xyz( 0, 0, 0)
        time.sleep(0.05)

def main():
    try:
        # Connect to drone
        print("Connect to drone...")
        drone.connect_drone('0.0.0.0:14550')

        # # Guided mode
        # print("Setting mode: GUIDED...")
        # drone.set_flight_mode("GUIDED")
        # time.sleep(2)  # Đợi chế độ cập nhật

        # Cất cánh
        print(f"Take off {height}m...")
        drone.arm_and_takeoff(height)
        time.sleep(1)

        print(f"Mode: {drone.get_mode()}")
       

        # Bay hình chữ nhật bằng XYA
        duration = size / speed  # Thời gian mỗi cạnh (10m / 3m/s = 3.33s)
        print(f"Flight Regtangle {size}m x {size}m XYA (Eachtime: {duration:.2f}s)...")
        drone.set_velocity_xyYaw(0, 0, yaw_angle, 0)   # Trước
        time.sleep(duration)
        stop_drone()
        
        # drone.set_velocity_xyYaw(speed, 0, yaw_angle, yaw_rate)   # Phải
        # time.sleep(duration)
        # stop_drone()
        
        # drone.set_velocity_xyYaw(-speed, 0, yaw_angle, 0)  # Sau
        # time.sleep(duration)
        # stop_drone()
        
        # drone.set_velocity_xyYaw(-speed, 0, yaw_angle, yaw_rate)  # Trái
        # time.sleep(duration)
        # stop_drone()
        

        # Bay hình chữ nhật bằng YAW rotation
        # print(f"Bay hình chữ nhật {size}m x {size}m bằng YAW rotation...")
        # for i in range(4):
        #     if i > 0:
        #         print(f"Xoay 90° (lần {i})...")
        #         drone.set_yaw(90, 10)
        #         time.sleep(10)  # Đợi xoay xong (90° / 30 deg/s = 3s)
        #         drone.set_velocity_xyz(speed, 0, 0)  # Tiến về phía trước
        #         time.sleep(duration)
        #     stop_drone()
        #     time.sleep(0.5)

        # Hạ cánh
        print("Hạ cánh...")
        drone.land()
        time.sleep(1)
        drone.disconnect_drone()

    except Exception as e:
        print(f"Lỗi: {e}")
        drone.land()
        drone.disconnect_drone()

if __name__ == "__main__":
    main()