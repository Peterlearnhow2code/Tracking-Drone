import jetson.inference
import jetson.utils


net = None
camera = None
output_stream = None
font = None

# Các tham số cho videoSource, giữ nguyên từ code gốc của bạn
VIDEO_SOURCE_ARGS = [
    '--input-width=1920',
    '--input-height=1080',
    '--input-flip=rotate-180',
]

# Các tham số cho videoOutput, ví dụ sử dụng codec H.264 và bitrate
# Bạn có thể điều chỉnh bitrate để cân bằng chất lượng và độ trễ
VIDEO_OUTPUT_ARGS = [
    '--output-codec=h264',
    '--output-bitrate=4000000' # 4Mbps
]

def initialize_detector(laptop_ip_address, model_name="ssd-inception-v2"):
    global net, camera, output_stream, font
    print(f"Initializing detector with model: {model_name}")
    net = jetson.inference.detectNet(model_name, threshold=0.5) # Thêm threshold nếu cần
    
    print(f"Initializing camera: csi://0 with args: {VIDEO_SOURCE_ARGS}")
    camera = jetson.utils.videoSource("csi://0", argv=VIDEO_SOURCE_ARGS)
    
    stream_uri = f"rtp://{laptop_ip_address}:1234"
    print(f"Initializing RTP output stream to: {stream_uri} with args: {VIDEO_OUTPUT_ARGS}")
    output_stream = jetson.utils.videoOutput(stream_uri, argv=VIDEO_OUTPUT_ARGS)
    
    print("Initializing CUDA font")
    font = jetson.utils.cudaFont()
    print("Detector initialization complete.")

def get_image_size():
    if camera:
        return camera.GetWidth(), camera.GetHeight()
    return None, None

def close_resources():
    global camera, output_stream
    print("Closing camera...")
    if camera:
        camera.Close()
        camera = None # Đặt lại để tránh sử dụng lại
    print("Closing output stream...")
    if output_stream:
        output_stream.Close()
        output_stream = None # Đặt lại
    print("Resources closed.")

def capture_and_detect():
    if not camera or not net:
        print("Error: Camera or network not initialized.")
        return None, [], 0.0

    img = camera.Capture()
    if img is None:
        print("Error: Failed to capture image from camera.")
        return None, [], 0.0
        
    detections = net.Detect(img)
    fps = net.GetNetworkFPS()
    return img, detections, fps

def draw_overlays_and_stream(cuda_img, detections, person_to_track, image_center_tuple,
                             x_real, yaw_command, x_delta, y_delta, fps_val, velocity_x_command,
                             current_state_text="N/A", search_time_left=None):
    if cuda_img is None or output_stream is None or font is None:
        # print("Error: CUDA image, output stream, or font not available for drawing/streaming.")
        return None # Trả về None nếu không có ảnh để xử lý

    # Vẽ bounding box cho tất cả detections (chỉ ClassID 1 - person)
    for detection in detections:
        if detection.ClassID == 1: # Person class
            # Màu xanh lá cho các đối tượng người khác
            color = (0, 255, 0, 150) # RGBA
            if person_to_track and detection.Instance == person_to_track.Instance:
                 # Màu đỏ cho đối tượng đang theo dõi
                color = (255, 0, 0, 200)
            
            jetson.utils.cudaDrawRect(cuda_img, (detection.Left, detection.Top, detection.Right, detection.Bottom), color)
            label = f"{net.GetClassDesc(detection.ClassID)} {detection.Confidence*100:.1f}%"
            font.OverlayText(cuda_img, cuda_img.width, cuda_img.height, label, int(detection.Left + 5), int(detection.Top + 5), font.White, font.Gray40)


    if person_to_track:
        # Vẽ đường từ tâm ảnh đến tâm đối tượng theo dõi
        person_center_x, person_center_y = person_to_track.Center
        jetson.utils.cudaDrawLine(cuda_img, (int(image_center_tuple[0]), int(image_center_tuple[1])),
                                  (int(person_center_x), int(person_center_y)), (255, 0, 0, 255), 10) # Màu xanh dương, dày

        # Vẽ tâm đối tượng theo dõi (màu đỏ)
        jetson.utils.cudaDrawCircle(cuda_img, (int(person_center_x), int(person_center_y)), 20, (255, 0, 0, 200)) # Đỏ

    # Vẽ tâm camera (màu xanh lá)
    jetson.utils.cudaDrawCircle(cuda_img, (int(image_center_tuple[0]), int(image_center_tuple[1])), 20, (0, 255, 0, 200)) # Xanh lá

    # Hiển thị thông số
    line_height = 35 # Chiều cao ước tính của một dòng text
    start_y_offset = 50

    text_color = font.White
    bg_color = font.Gray40

    font.OverlayText(cuda_img, cuda_img.width, cuda_img.height,
                     f"FPS: {fps_val:.2f} Yaw: {yaw_command:.2f} Fwd: {velocity_x_command:.2f}",
                     10, start_y_offset, text_color, bg_color)
    
    font.OverlayText(cuda_img, cuda_img.width, cuda_img.height,
                     f"X_delta: {x_delta:.2f} Y_delta: {y_delta:.2f}",
                     10, start_y_offset + line_height, text_color, bg_color)
    
    font.OverlayText(cuda_img, cuda_img.width, cuda_img.height,
                     f"X_real: {x_real:.2f}",  # X_real thường giống x_delta nếu không có xử lý đặc biệt
                     10, start_y_offset + 2 * line_height, text_color, bg_color)

    font.OverlayText(cuda_img, cuda_img.width, cuda_img.height,
                     f"State: {current_state_text}",
                     10, start_y_offset + 3 * line_height, text_color, bg_color)
    
    if search_time_left is not None:
         font.OverlayText(cuda_img, cuda_img.width, cuda_img.height,
                     f"Search time left: {search_time_left:.0f}s",
                     10, start_y_offset + 4 * line_height, text_color, bg_color)


    output_stream.Render(cuda_img)
    
    # Trả về ảnh NumPy để ghi file AVI nếu cần
    return jetson.utils.cudaToNumpy(cuda_img)