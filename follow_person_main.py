import sys, time
import argparse

sys.path.append("/home/jetson/peter/ai-drone0/modules")
import cv2
import collections
import numpy as np

from modules import mjpeg
from modules import detector_mobilenet as detector
from modules import vision
from modules import control
from modules import drone


# Args parser
parser = argparse.ArgumentParser(description='Drive autonomous')
parser.add_argument('--debug_path', type=str, default="debug/run1", help='debug message name')
parser.add_argument('--mode', type=str, default='flight', help='Switches between flight record and flight visualisation')
parser.add_argument('--control', type=str, default='PID', help='Use PID or P controller' )
args = parser.parse_args()

# config
MAX_ALT =  5                                  #m
MAX_MA_X_LEN = 5
MAX_MA_Z_LEN = 5
MA_X = collections.deque(maxlen=MAX_MA_X_LEN)   #Moving Average X
MA_Z = collections.deque(maxlen=MAX_MA_Z_LEN)   #Moving Average Z
STATE = "takeoff"                               # takeoff land track search
mjpeg_server = None
# end config

def setup():
    global mjpeg_server

    print("setting up detector")
    detector.initialize_detector()

    image_width, image_height = detector.get_image_size()
    init_frame = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    try:
        # Sử dụng '0.0.0.0' để server có thể được truy cập từ các máy khác trong cùng mạng
        # Port 8080 là port phổ biến, bạn có thể đổi nếu cần
        mjpeg_server = mjpeg.MjpegServer(init_img=init_frame, ip='0.0.0.0', port=8080)
        print(f"MJPEG server started. Access stream at http://100.68.218.111:8080/mjpg")
    except Exception as e:
        print(f"Failed to start MJPEG server: {e}")
        sys.exit(1)
        # Bạn có thể quyết định có tiếp tục chạy chương trình không nếu server không khởi động được

    print("connecting to drone")
    if args.mode == "flight":
        print("MODE = flight")
        control.connect_drone('/dev/ttyTHS1')
    else:
        print("MODE = test")
        control.connect_drone('0.0.0.0:14550')
    
setup()

image_width, image_height = detector.get_image_size()
image_center = (image_width / 2, image_height / 2)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# debug_image_writer = cv2.VideoWriter(args.debug_path + ".avi", fourcc, 29.0, (image_width, image_height))


control.configure_PID(args.control)
control.initialize_debug_logs(args.debug_path)

def track():
    global mjpeg_server
    print("State is TRACKING -> " + STATE)
    start = time.time()
    flag_movement = 1
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Closing due to manual interruption")
            land() # Closes the loop and program
        while time.time() - start >= 0.5 :
            start = time.time()
            if flag_movement == 0:
                flag_movement = 1
            else:
                flag_movement = 0
            detections, fps, image = detector.get_detections()

            if len(detections) > 0:
                person_to_track = detections[0] # only track 1 person
                
                print(person_to_track)

                person_center = person_to_track.Center # get center of person to track

                x_delta = vision.get_single_axis_delta(image_center[0],person_center[0]) # get x delta frame picture theo tiêu chuẩn quốc tế
                y_delta = vision.get_single_axis_delta(image_center[1],person_center[1]) # get y delta frame picture theo tiêu chuẩn quốc tế
                y_delta = - y_delta # đổi dấu y_delta
                x_real = x_delta

                #debug later
                # if x_delta < 90 or x_delta > -90:
                #     x_delta = 0
                 
                MA_Z.append(y_delta)
                MA_X.append(x_delta)
                
                #yaw command > PID and moving average
                yaw_command = 0
                if len(MA_X) > 0:
                    x_delta_MA = calculate_ma(MA_X)
                    control.setXdelta(x_delta_MA)
                    yaw_command = control.getMovementYawAngle()

                #depth x command > PID and moving average
                velocity_z_command = 0
                if  len(MA_Z) > 0: #only if valid value is given change the forward velocity. Otherwise keep previos velocity (done by arducopter itself)
                    z_delta_MA = calculate_ma(MA_Z) 
                    control.setZDelta(z_delta_MA)
                    velocity_z_command = control.getMovementVelocityXCommand()

                control.control_drone(flag_movement)
            
                prepare_visualisation(x_real, person_center, person_to_track, image, yaw_command, x_delta, y_delta, fps, velocity_z_command)
                if mjpeg_server:
                    mjpeg_server.send_img(image)
                # visualize(image)
                # debug_image_writer.write(image)

            else:
                if mjpeg_server:
                    mjpeg_server.send_img(image)
                # debug_image_writer.release()
                return "search"

def search():
    wait_time = 60
    global mjpeg_server
    print("State is SEARCH -> " + STATE)
    start_search_time = time.time()
    control.stop_drone()
    while time.time() - start_search_time < wait_time:
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     print("Closing due to manual interruption")
        #     land() # Closes the loop and program

        elapsed_time = time.time() - start_search_time

        # Giai đoạn đứng im tìm kiếm (20 giây đầu)
        if elapsed_time < 20:
            # Drone đứng im nhờ control.stop_drone() ở đầu hàm
            search_text = "Searching (Hovering)... Time left: {:.0f}s".format(wait_time - elapsed_time)
        # Giai đoạn quay tìm kiếm (sau 20 giây)
        else:
            # Quay từ từ sang phải (tốc độ 10 độ/s, lệnh kéo dài 1s)
            drone.set_velocity_xyYaw(0, 0, 10, 1)
            search_text = "Searching (Rotating)... Time left: {:.0f}s".format(wait_time - elapsed_time)

        detections, fps, image = detector.get_detections()
        cv2.putText(image, search_text, (50, image_height - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        if mjpeg_server:
            mjpeg_server.send_img(image)
        
        print("searching: " + str(len(detections)))

        if len(detections) > 0:
            return "track"
        # if "test" == args.mode:
        #     cv2.putText(image, search_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA) 
        #     #visualize(image)

    return "land"

def takeoff():
    #control.print_drone_report()
    print("State = TAKEOFF -> " + STATE)
    control.arm_and_takeoff(MAX_ALT) #start control when drone is ready
    return "search"

def land():
    global mjpeg_server
    print("State = LAND -> " + STATE)
    control.land()
    detector.close_camera()
    #cv2.destroyAllWindows()  
    if mjpeg_server:
        print("Shutting down MJPEG server...")
        mjpeg_server.shutdown()
        mjpeg_server = None # Xóa tham chiếu global
    sys.exit(0)
    

# def visualize(img):
#     display_img = cv2.resize(img, (1280, 720))
#     if "flight" == args.mode:
#         debug_image_writer.write(display_img)
#     else:
#         cv2.imshow("out", display_img)
#         cv2.waitKey(1)
#     return

def prepare_visualisation(x_real, person_center, person_to_track, image, yaw_command, x_delta, y_delta, fps, velocity_x_command):
    #draw path
    cv2.line(image, (int(image_center[0]), int(image_center[1])), (int(person_center[0]), int(person_center[1])), (255, 0, 0), thickness=10, lineType=8, shift=0)

    #draw bbox around target
    cv2.rectangle(image,(int(person_to_track.Left),int(person_to_track.Bottom)), (int(person_to_track.Right),int(person_to_track.Top)), (0,0,255), thickness=10)

    #show drone center
    cv2.circle(image, (int(image_center[0]), int(image_center[1])), 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    #show trackable center
    cv2.circle(image, (int(person_center[0]), int(person_center[1])), 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)

    #show stats
    cv2.putText(image, "fps: " + str(round(fps,2)) + " yaw: " + str(round(yaw_command,2)) + " forward: " + str(round(velocity_x_command,2)) , (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA) 
    cv2.putText(image, "x_delta: " + str(round(x_delta,2)) + " y_delta: " + str(round(y_delta,2)), (50, 150), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA) 
    cv2.putText(image, "x_real: " + str(round(x_real,2)), (50, 200), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 255), 3, cv2.LINE_AA) 
    #visualize(image)

#tích phân
def calculate_ma(ma_array):
    sum_ma = 0
    for i in ma_array:
        sum_ma = sum_ma + i

    return sum_ma / len(ma_array)

# main program loop
try:
    while True:
        # main program loop
        """" True or False values depend whether or not
            a PID controller or a P controller will be used  """

        if STATE == "track":
            control.set_system_state("track")
            STATE = track()

        elif STATE == "search":
            control.set_system_state("search")
            STATE = search()
        
        elif STATE == "takeoff":
            STATE = takeoff()

        elif STATE == "land":
            STATE = land()
except KeyboardInterrupt:
    print("\nKeyboard interrupt received, landing drone...")
    land()
