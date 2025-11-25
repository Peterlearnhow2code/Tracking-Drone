from pymavlink import mavutil
import time
import math
vehicle = None

# Connect to drone
def connect_drone(connection_string, baudrate=115200):
    global vehicle
    """
    Kết nối đến drone qua giao thức MAVLink.

    Args:
        connection_string (str): Chuỗi kết nối (ví dụ: 'udp:127.0.0.1:14550' hoặc '/dev/ttyTHS1').
        baudrate (int, optional): Tốc độ baud cho kết nối nối tiếp. Mặc định là 57600.

    Returns:
        mavutil.mavlink_connection: Đối tượng kết nối MAVLink nếu kết nối thành công.

    Raises:
        ConnectionError: Nếu không thể kết nối hoặc không nhận được heartbeat trong thời gian chờ.
    """
    if vehicle == None:
        
        try:
        # Kiểm tra loại kết nối và khởi tạo đối tượng vehicle
            if connection_string.startswith("udp:") or connection_string.startswith("tcp:"):
                vehicle = mavutil.mavlink_connection(connection_string)
            else:
                vehicle = mavutil.mavlink_connection(connection_string, baud=baudrate)
        
        # Chờ heartbeat với timeout để kiểm tra trạng thái kết nối
            vehicle.wait_heartbeat(timeout=10)  # Timeout sau 10 giây
            print(f"Heartbeat detected (system {vehicle.target_system} component {vehicle.target_component})")
        
        # Trả về đối tượng vehicle nếu kết nối thành công
        except Exception as e:
            raise ConnectionError(f"Error Connect to drone: {e}")
    else:
        print(f"Connect drone as {connection_string} with baudrate {baudrate}...")
        return vehicle
# Disconnect from drone
def disconnect_drone():
    global vehicle
    """
    Disconnects from the drone and closes the MAVLink connection.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object to disconnect.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        Exception: If an error occurs while closing the connection.
    """
    if vehicle is None:
        raise ValueError("No drone connection provided to disconnect")    
    try:
        vehicle.close()
        print("Drone disconnected successfully")
    except Exception as e:
        raise Exception(f"Failed to disconnect drone: {e}")
# get version
def get_version():
    global vehicle
    """
    Retrieves the autopilot version information from the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        dict: A dictionary containing version information:
              - "flight_sw_version": Flight software version.
              - "middleware_sw_version": Middleware software version.
              - "os_sw_version": Operating system software version.
              - "board_version": Board hardware version.
              Returns None if the version cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no version response is received within the timeout period.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    # Ensure the connection is established by waiting for a heartbeat
    print("Waiting for heartbeat from drone...")
    try:
        vehicle.wait_heartbeat(timeout=10)
        print(f"Heartbeat received from system {vehicle.target_system}, component {vehicle.target_component}")
    except Exception as e:
        raise TimeoutError(f"Failed to receive heartbeat within 10 seconds: {e}")

    # Set target system and component (default to autopilot)
    target_system = vehicle.target_system
    target_component = vehicle.target_component

    try:
        # Send request for autopilot version
        print("Requesting autopilot version...")
        vehicle.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            0,  # Confirmation
            1,  # Param1: Request version
            0, 0, 0, 0, 0, 0  # Unused parameters
        )

        # Wait for AUTOPILOT_VERSION response with a retry mechanism
        start_time = time.time()
        timeout = 30  # Timeout in seconds
        while time.time() - start_time < timeout:
            msg = vehicle.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=1)
            if msg:
                print("Received AUTOPILOT_VERSION message")
                # Extract version information
                version_info = {
                    "flight_sw_version": msg.flight_sw_version,
                    "middleware_sw_version": msg.middleware_sw_version,
                    "os_sw_version": msg.os_sw_version,
                    "board_version": msg.board_version
                }
                return version_info
            else:
                print("Still waiting for AUTOPILOT_VERSION message...")

        raise TimeoutError("Cannot receive version info within 30 seconds")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving version: {e}")
        raise
# get mission
def get_mission():
    global vehicle
    """
    Retrieves the mission items from the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        list: A list of MISSION_ITEM messages representing the mission waypoints.
              Returns an empty list if no mission is received or an error occurs.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no mission data is received within the timeout period.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    mission_items = []

    try:
        # Request the total number of mission items
        vehicle.mav.mission_request_list_send(
            vehicle.target_system,
            vehicle.target_component
        )

        # Wait for MISSION_COUNT response
        msg = vehicle.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM'], blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Timeout waiting for mission count")

        if msg.get_type() == 'MISSION_COUNT':
            count = msg.count
            print(f"Number of mission items: {count}")

            # Request each mission item
            for i in range(count):
                vehicle.mav.mission_request_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    i
                )
                item_msg = vehicle.recv_match(type='MISSION_ITEM', blocking=True, timeout=2)
                if item_msg:
                    mission_items.append(item_msg)
                else:
                    print(f"Cannot receive mission item {i}")
                    raise TimeoutError(f"Failed to retrieve mission item {i}")

    except TimeoutError as e:
        print(f"Error: {e}")
        return mission_items  # Return partial mission items if any
    except Exception as e:
        print(f"Unexpected error while retrieving mission: {e}")
        raise

    return mission_items
# get location
def get_location():
    global vehicle
    """
    Retrieves the current GPS location of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        dict: A dictionary containing GPS coordinates:
              - "lat": Latitude in degrees.
              - "lon": Longitude in degrees.
              - "alt": Altitude in meters (above sea level).
              Returns None if the location cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no GPS data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for GLOBAL_POSITION_INT message with a shorter timeout
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("No GPS signal received within 2 seconds")

        # Extract and convert GPS data
        location = {
            "lat": msg.lat / 1e7,  # Convert from 1E7 to degrees
            "lon": msg.lon / 1e7,  # Convert from 1E7 to degrees
            "alt": msg.alt / 1000.0  # Convert from millimeters to meters
        }
        return location

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving location: {e}")
        raise
# get altitude
def get_altitude():
    global vehicle
    """
    Retrieves the current altitude of the drone relative to the takeoff point.
    Returns:
        float: Altitude in meters relative to the takeoff point.
               Returns None if altitude data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no altitude data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for GLOBAL_POSITION_INT message with a shorter timeout
        msg = vehicle.recv_match(type='ALTITUDE', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive altitude data within 2 seconds")

        # Extract relative altitude and convert from millimeters to meters
        alt_meters = msg.altitude_relative / 1000.0
        alt_terrain = msg.altitude_terrain / 1000.0
        alt_bottom = msg.bottom_clearance / 1000.0
        return alt_meters, alt_terrain, alt_bottom

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving altitude: {e}")
        raise
# get velocity
def get_velocity():
    global vehicle
    """
    Retrieves the current velocity of the drone in the NED (North-East-Down) frame.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        tuple: A tuple (vx, vy, vz) representing velocity in meters per second:
               - vx: Velocity in the X-axis (North).
               - vy: Velocity in the Y-axis (East).
               - vz: Velocity in the Z-axis (Down).
               Returns None if velocity data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no velocity data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for GLOBAL_POSITION_INT message with a shorter timeout
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive velocity data within 2 seconds")

        # Extract velocity and convert from cm/s to m/s
        vx = msg.vx / 100.0  # X-axis velocity (North)
        vy = msg.vy / 100.0  # Y-axis velocity (East)
        vz = msg.vz / 100.0  # Z-axis velocity (Down)
        return (vx, vy, vz)

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving velocity: {e}")
        raise
# get battery info
def get_battery_info():
    global vehicle
    """
    Retrieves the current battery status of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        dict: A dictionary containing battery information:
              - "voltage_V": Voltage in volts.
              - "current_A": Current in amperes.
              - "level_percent": Battery level in percentage (0-100).
              Returns None if battery data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no battery data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for SYS_STATUS message with a shorter timeout
        msg = vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive battery info within 2 seconds")

        # Extract and convert battery data
        battery_info = {
            "voltage_V": msg.voltage_battery / 1000.0,  # Convert from mV to V
            "current_A": msg.current_battery / 100.0,   # Convert from cA to A
            "level_percent": msg.battery_remaining      # Battery level in percentage
        }
        return battery_info

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving battery info: {e}")
        raise
# get mode
def get_mode():
    global vehicle
    """
    Retrieves the current flight mode of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        str: The current flight mode (e.g., "STABILIZE", "GUIDED") or "UNKNOWN_MODE_<id>" if not recognized.
             Returns None if mode data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no mode data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for HEARTBEAT message with a shorter timeout
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive mode info within 2 seconds")

        # Extract mode ID from HEARTBEAT message
        mode_id = msg.custom_mode

        # Mapping of mode IDs to names (specific to ArduCopter)
        mode_mapping = {
            0: "STABILIZE",
            1: "ACRO",
            2: "ALT_HOLD",
            3: "AUTO",
            4: "GUIDED",
            5: "LOITER",
            6: "RTL",
            7: "CIRCLE",
            9: "LAND",
            11: "DRIFT",
            13: "SPORT",
            14: "FLIP",
            15: "AUTOTUNE",
            16: "POSHOLD",
            17: "BRAKE",
            18: "THROW",
            19: "AVOID_ADSB",
            20: "GUIDED_NOGPS",
            21: "SMART_RTL"
        }
        return mode_mapping.get(mode_id, f"UNKNOWN_MODE_{mode_id}")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving mode: {e}")
        raise
# get home location
def get_home_location():
    global vehicle
    """
    Retrieves the home location of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        dict: A dictionary containing home position coordinates:
              - "lat": Latitude in degrees.
              - "lon": Longitude in degrees.
              - "alt": Altitude in meters (above sea level).
              Returns None if home position cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no home position data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Send request for home position
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Unused parameters
        )

        # Wait for HOME_POSITION response with a shorter timeout
        msg = vehicle.recv_match(type='HOME_POSITION', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive home position within 2 seconds")

        # Extract and convert home position data
        home_location = {
            "lat": msg.latitude / 1e7,    # Convert from 1E7 to degrees
            "lon": msg.longitude / 1e7,   # Convert from 1E7 to degrees
            "alt": msg.altitude / 1000.0  # Convert from millimeters to meters
        }
        return home_location

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving home location: {e}")
        raise
# get heading
def get_heading():
    global vehicle
    """
    Retrieves the current heading (yaw) of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        float: Heading in degrees (0-359), where 0 is North.
               Returns None if heading data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no heading data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for VFR_HUD message containing heading with a timeout
        msg = vehicle.recv_match(type='VFR_HUD', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive heading data within 2 seconds")

        # Extract heading in degrees
        return msg.heading

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving heading: {e}")
        raise
# get EKF status
def get_EKF_status():
    global vehicle
    """
    Retrieves the Extended Kalman Filter (EKF) status of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        bool: True if the EKF status is healthy (attitude, velocity, and position are OK),
              False otherwise. Returns None if EKF data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no EKF status data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for EKF_STATUS_REPORT message with a timeout
        msg = vehicle.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=30)
        if not msg:
            raise TimeoutError("Cannot receive EKF status within 30 seconds")

        # Check EKF flags for health status
        # Bit positions: 0=attitude, 1=velocity_horiz, 2=velocity_vert, 3=pos_horiz_rel
        required_flags = (
            msg.flags & (1 << 0) and  # Attitude OK
            msg.flags & (1 << 1) and  # Horizontal velocity OK
            msg.flags & (1 << 2) and  # Vertical velocity OK
            msg.flags & (1 << 3)      # Relative horizontal position OK
        )
        return bool(required_flags)

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving EKF status: {e}")
        raise
# get ground speed
def get_ground_speed():
    global vehicle
    """
    Retrieves the current ground speed of the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.

    Returns:
        float: Ground speed in meters per second (m/s).
               Returns None if ground speed data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or invalid.
        TimeoutError: If no ground speed data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Wait for VFR_HUD message containing ground speed with a timeout
        msg = vehicle.recv_match(type='VFR_HUD', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive ground speed data within 2 seconds")

        # Extract ground speed in m/s
        return msg.groundspeed

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving ground speed: {e}")
        raise
# set gimbal angle
def set_gimbal_angle(angle):
    """
    Sets the gimbal tilt angle on the drone.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.
        angle (float): The desired tilt angle in degrees (typically -90 to 90).

    Raises:
        ValueError: If the vehicle object is None or the angle is invalid.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(angle, (int, float)) or angle < -90 or angle > 90:
        raise ValueError(f"Invalid gimbal angle: {angle}. Must be between -90 and 90 degrees")

    try:
        print(f"Setting gimbal angle to: {angle} degrees")
        # Send command to control gimbal
        vehicle.mav.command_long_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,  # Command ID
            0,                        # Confirmation
            0,                        # Param1: Pitch (not used here)
            angle,                    # Param2: Roll (tilt angle in degrees)
            0,                        # Param3: Yaw (not used here)
            0, 0, 0,                  # Param4-6: Reserved
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING  # Param7: Mount mode
        )
    except Exception as e:
        print(f"Unexpected error while setting gimbal angle: {e}")
        raise

# set ground speed
def set_groundspeed(speed):
    """
    Sets the ground speed of the drone in the local NED (North-East-Down) frame.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.
        speed (float): Desired ground speed in meters per second (m/s).

    Raises:
        ValueError: If the vehicle object is None or the speed is invalid.
        RuntimeError: If the current heading cannot be retrieved.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(speed, (int, float)) or speed < 0:
        raise ValueError(f"Invalid speed: {speed}. Must be a non-negative number")

    try:
        # Get current heading to compute velocity components
        heading = get_heading(vehicle)
        if heading is None:
            raise RuntimeError("Cannot retrieve heading to set ground speed")

        # Calculate velocity components based on heading and speed
        vx = speed * math.cos(math.radians(heading))  # North velocity (m/s)
        vy = speed * math.sin(math.radians(heading))  # East velocity (m/s)

        # Send velocity command in local NED frame
        vehicle.mav.set_position_target_local_ned_send(
            0,                        # Time boot (not used)
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame
            0b0000111111000111,       # Type mask: velocity only (ignore position, accel, yaw)
            0, 0, 0,                  # X, Y, Z position (ignored)
            vx, vy, 0,                # X, Y, Z velocity (m/s)
            0, 0, 0,                  # X, Y, Z acceleration (ignored)
            0, 0                      # Yaw, yaw rate (ignored)
        )
        print(f"Ground speed set to: {speed} m/s (vx={vx:.2f}, vy={vy:.2f})")

    except RuntimeError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while setting ground speed: {e}")
        raise
# set_flight_mode
def set_flight_mode(f_mode):
    """
    Sets the flight mode of the drone.

    Args:
        f_mode (str): The desired flight mode (e.g., "GUIDED", "STABILIZE").

    Raises:
        ValueError: If the vehicle object is None or the flight mode is invalid/unsupported.
        Exception: For unexpected errors during mode setting.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(f_mode, str):
        raise ValueError(f"Invalid flight mode: {f_mode}. Must be a string")

    try:
        # Get mode ID from mode mapping
        mode_mapping = vehicle.mode_mapping()
        mode_id = mode_mapping.get(f_mode)
        if mode_id is None:
            raise ValueError(f"Flight mode '{f_mode}' not supported by this vehicle")

                # Set the flight mode
        vehicle.set_mode(mode_id)
        print(f"Flight mode set to: {f_mode}")

    except ValueError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while setting flight mode: {e}")
        raise
# set param
def set_param(param, value):
    """
    Sets a parameter value on the drone.

    Args:
        param (str): The parameter name (e.g., "ALT_HOLD_P").
        value (float): The value to set for the parameter.

    Raises:
        ValueError: If the vehicle object is None or the parameter/value is invalid.
        Exception: For unexpected errors during parameter setting.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(param, str) or not param:
        raise ValueError(f"Invalid parameter name: {param}. Must be a non-empty string")
    try:
        value = float(value)  # Ensure value can be converted to float
    except (TypeError, ValueError):
        raise ValueError(f"Invalid parameter value: {value}. Must be convertible to float")

    try:
        print(f"Setting parameter '{param}' to {value}")
        vehicle.param_set_send(param.encode(), value)
    except Exception as e:
        print(f"Unexpected error while setting parameter '{param}': {e}")
        raise
# get param
def get_param(param):
    """
    Retrieves the value of a specific parameter from the drone.

    Args:
        param (str): The parameter name to read (e.g., "ALT_HOLD_P").

    Returns:
        float: The parameter value if retrieved successfully, None if not found.

    Raises:
        ValueError: If the vehicle object is None or the parameter name is invalid.
        TimeoutError: If no parameter value is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(param, str) or not param:
        raise ValueError(f"Invalid parameter name: {param}. Must be a non-empty string")

    try:
        # Request the parameter value
        vehicle.mav.param_request_read_send(
            vehicle.target_system,
            vehicle.target_component,
            param.encode(),
            -1  # Parameter index (-1 means read by name)
        )

        # Wait for PARAM_VALUE response
        msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if msg is None:
            raise TimeoutError(f"Timeout reading parameter '{param}' after 2 seconds")

        # Check if the received parameter matches the requested one
        received_param = msg.param_id.decode('utf-8').strip('\x00')
        if received_param == param:
            return msg.param_value
        else:
            raise TimeoutError(f"Received unexpected parameter '{received_param}' instead of '{param}'")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving parameter '{param}': {e}")
        raise

# read channel
def read_channel(channel):
    """
    Retrieves the raw value of a specific RC channel from the drone.

    Args:
        channel (int): The RC channel number to read (1-18).

    Returns:
        int: The raw PWM value of the specified RC channel (typically 1000-2000).
             Returns None if the channel data cannot be retrieved.

    Raises:
        ValueError: If the vehicle object is None or the channel number is invalid.
        TimeoutError: If no RC channel data is received within the timeout period.
        Exception: For other unexpected errors during retrieval.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(channel, int) or channel < 1 or channel > 18:
        raise ValueError(f"Invalid channel number: {channel}. Must be between 1 and 18")

    try:
        # Wait for RC_CHANNELS message with a timeout
        msg = vehicle.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
        if not msg:
            raise TimeoutError("Cannot receive RC channel data within 2 seconds")

        # Construct attribute name for the specified channel
        chan_attr = f"chan{channel}_raw"
        if hasattr(msg, chan_attr):
            return getattr(msg, chan_attr)
        else:
            raise ValueError(f"Channel {channel} not available in RC_CHANNELS message")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except ValueError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while retrieving RC channel {channel}: {e}")
        raise
# set channel
def set_channel(channel, value):
    """
    Sets the value of a specific RC channel on the drone using RC override.

    Args:
        channel (int): The RC channel number to set (1-18).
        value (int): The PWM value to set (typically 1000-2000, or 65535 to ignore).

    Raises:
        ValueError: If the vehicle object is None, or channel/value is invalid.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not isinstance(channel, int) or channel < 1 or channel > 18:
        raise ValueError(f"Invalid channel number: {channel}. Must be between 1 and 18")
    if not isinstance(value, int) or value < 0 or value > 65535:
        raise ValueError(f"Invalid PWM value: {value}. Must be between 0 and 65535")

    try:
        # Initialize RC values with 65535 (ignore) for all 18 channels
        rc_values = [65535] * 18
        rc_values[channel - 1] = value  # Set the specified channel value

        # Send RC override command
        vehicle.mav.rc_channels_override_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            *rc_values                # Unpacked RC channel values
        )
        print(f"RC channel {channel} set to: {value}")

    except Exception as e:
        print(f"Unexpected error while setting RC channel {channel}: {e}")
        raise
# clear channel
def clear_channel():
    """
    Clears all RC channel overrides on the drone by setting them to ignore.

    Args:

    Raises:
        ValueError: If the vehicle object is None.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Initialize RC values with 65535 (ignore) for all 18 channels
        rc_values = [65535] * 18

        # Send RC override command to clear all channels
        vehicle.mav.rc_channels_override_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            *rc_values                # Unpacked RC channel values (all set to 65535)
        )
        print("All RC channel overrides cleared")

    except Exception as e:
        print(f"Unexpected error while clearing RC channels: {e}")
        raise

#arm
def arm():
    """
    arms the drone.

    Args:

    Raises:
        ValueError: If the vehicle object is None.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Send disarm command
        vehicle.mav.command_long_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID
            0,                        # Confirmation
            1,                        # Param1: 0 = disarm, 1 = arm
            0, 0, 0, 0, 0, 0         # Param2-7: Unused
        )
        print("Arm command sent successfully")

    except Exception as e:
        print(f"Unexpected error while disarming drone: {e}")
        raise

#disarm
def disarm():
    """
    Disarms the drone, stopping all motors.

    Args:

    Raises:
        ValueError: If the vehicle object is None.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Send disarm command
        vehicle.mav.command_long_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID
            0,                        # Confirmation
            0,                        # Param1: 0 = disarm
            0, 0, 0, 0, 0, 0         # Param2-7: Unused
        )
        print("Disarm command sent successfully")

    except Exception as e:
        print(f"Unexpected error while disarming drone: {e}")
        raise

#arm with stabelize mode
def arm_and_stabilize():
    """
    Arms the drone and sets it to STABILIZE mode after performing pre-arm checks.

    Args:

    Raises:
        ValueError: If the vehicle object is None.
        RuntimeError: If STABILIZE mode is not supported or arming fails.
        TimeoutError: If connection or arming confirmation times out.
        Exception: For other unexpected errors during execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        print("Performing basic pre-arm checks...")

        # Wait for heartbeat to confirm connection
        vehicle.wait_heartbeat(timeout=2)
        print(f"Heartbeat received from system (system {vehicle.target_system}, component {vehicle.target_component})")

        # Request autopilot capabilities to ensure communication
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            0,  # Confirmation
            1,  # Param1: Request capabilities
            0, 0, 0, 0, 0, 0  # Param2-7: Unused
        )

        # Set flight mode to STABILIZE
        mode = "STABILIZE"
        mode_mapping = vehicle.mode_mapping()
        mode_id = mode_mapping.get(mode)
        if mode_id is None:
            raise RuntimeError(f"Flight mode '{mode}' not supported by this vehicle")
        
        vehicle.set_mode(mode_id)
        print(f"Mode set to {mode}, waiting for stabilization...")
        time.sleep(1)  # Wait for mode to take effect

        # Arm the drone
        print("Arming motors...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Param1: 1 = arm
            0, 0, 0, 0, 0, 0  # Param2-7: Unused
        )

        # Wait until armed with timeout
        start_time = time.time()
        timeout = 5  # Maximum wait time in seconds
        while time.time() - start_time < timeout:
            msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Drone is ARMED!")
                return
            print("Waiting for arming...")
            time.sleep(1)
        
        raise TimeoutError("Failed to confirm arming within 5 seconds")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except RuntimeError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while arming and stabilizing: {e}")
        raise

#arm with guided mode
def arm_and_takeoff(target_altitude):
    """
    Arms the drone and commands it to take off to a target altitude in GUIDED mode.

    Args:
        target_altitude (float): Target altitude in meters above takeoff point.

    Raises:
        ValueError: If vehicle not connected or invalid altitude.
        RuntimeError: If GUIDED mode not supported or arming fails.
        TimeoutError: If connection or operation times out.
    """
    if vehicle is None or not isinstance(target_altitude, (int, float)) or target_altitude <= 0:
        raise ValueError("Drone not connected or invalid target altitude")

    try:
        vehicle.wait_heartbeat(timeout=2)
        print(f"Heartbeat received from system {vehicle.target_system}")

        mode_id = vehicle.mode_mapping().get("GUIDED")
        if mode_id is None:
            raise RuntimeError("GUIDED mode not supported")
        vehicle.set_mode(mode_id)
        time.sleep(1)

        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Param1: 1 = arm
            21196,  # Param2: 21196 = arm without pre-arm checks
            0, 0, 0, 0, 0  # Param3-7: Unused
        )

        start_time = time.time()
        while time.time() - start_time < 10:
            msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Drone ARMED")
                break
            time.sleep(1)
        else:
            raise TimeoutError("Arming timed out")

        print(f"Taking off to {target_altitude}m...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0,  # Param1-6: Unused
            target_altitude    # Param7: Target altitude in meters
        )

        start_time = time.time()
        while time.time() - start_time < 50:
            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                print(f"Current altitude: {current_alt:.2f}m")
                if current_alt >= target_altitude * 0.95:
                    print("Reached target altitude")
                    return
            time.sleep(1)
        raise TimeoutError(f"Failed to reach target altitude within 50 seconds")

    except Exception as e:
        print(f"Error during takeoff: {e}")
        raise

#land
def land():
    """
    Commands the drone to land and monitors the landing process.

    Args:

    Raises:
        ValueError: If the vehicle object is None.
        TimeoutError: If landing confirmation times out or position data is unavailable.
        Exception: For unexpected errors during execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Send land command
        print("Sending LAND command...")
        vehicle.mav.command_long_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            mavutil.mavlink.MAV_CMD_NAV_LAND,  # Command ID
            0,                        # Confirmation
            0, 0, 0, 0,               # Param1-4: Unused
            0, 0, 0                   # Param5-7: Latitude, Longitude, Altitude (unused)
        )

        # Monitor landing with timeout
        landing_timeout = 30  # Maximum wait time in seconds
        start_time = time.time()
        while time.time() - start_time < landing_timeout:
            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if not msg:
                print("No position data received, retrying...")
                time.sleep(1)
                continue
            alt = msg.relative_alt / 1000.0  # Convert from mm to m
            print(f"Altitude during landing: {alt:.2f} m")
            if alt <= 0.1:  # Consider landed if altitude <= 0.1 m
                print("Landed successfully")
                return
            time.sleep(1)
        raise TimeoutError(f"Failed to land within {landing_timeout} seconds")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while landing: {e}")
        raise

#return to launch location
def return_to_launch():
    """
    Commands the drone to return to its launch location (RTL mode).

    Args:
    
    Raises:
        ValueError: If the vehicle object is None.
        TimeoutError: If RTL mode confirmation times out or heartbeat data is unavailable.
        Exception: For unexpected errors during execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")

    try:
        # Send RTL command
        print("Sending RTL (Return to Launch) command...")
        vehicle.mav.command_long_send(
            vehicle.target_system,    # Target system
            vehicle.target_component, # Target component
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # Command ID
            0,                        # Confirmation
            0, 0, 0, 0, 0, 0, 0       # Param1-7: Unused
        )

        # Monitor mode change to RTL with timeout
        rtl_timeout = 10  # Maximum wait time in seconds
        start_time = time.time()
        while time.time() - start_time < rtl_timeout:
            msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if not msg:
                print("No heartbeat data received, retrying...")
                time.sleep(1)
                continue
            mode = mavutil.mode_string_v10(msg)
            print(f"Current mode: {mode}")
            if mode == "RTL":
                print("Vehicle is returning to launch")
                return
            time.sleep(1)
        raise TimeoutError(f"Failed to switch to RTL mode within {rtl_timeout} seconds")

    except TimeoutError as e:
        print(f"Error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error while initiating RTL: {e}")
        raise

#send movement YAW
# def set_yaw(yaw_angle, speed):
#     global vehicle
#     if vehicle is None:
#         raise ValueError("Drone not connected")
#     if not isinstance(yaw_angle, (int, float)):
#         raise ValueError(f"Invalid yaw angle: {yaw_angle}. Must be a number")
#     if not isinstance(speed, (int, float)) or speed <= 0:
#         raise ValueError(f"Invalid yaw speed: {speed}. Must be a positive number")

#     try:
#         # Determine direction and absolute yaw angle
#         direction = 1 if yaw_angle >= 0 else -1
#         absolute_yaw = abs(yaw_angle)

#         print(f"Sending YAW command to heading: {yaw_angle}° at {speed}°/s (direction: {'clockwise' if direction == 1 else 'counterclockwise'})")

#         # Encode and send the yaw command
#         msg = vehicle.mav.command_long_encode(
#             vehicle.target_system,    # Target system
#             vehicle.target_component, # Target component
#             mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command ID
#             0,                        # Confirmation
#             absolute_yaw,             # Param1: Target yaw angle (degrees)
#             speed,                    # Param2: Yaw speed (degrees/second)
#             direction,                # Param3: Direction (1 = clockwise, -1 = counterclockwise)
#             1,                        # Param4: Relative (0 = absolute angle, 1 = relative)
#             0, 0, 0                   # Param5-7: Unused
#         )
#         vehicle.mav.send(msg)

#     except Exception as e:
#         print(f"Unexpected error while setting yaw to {yaw_angle}°: {e}")
#         raise

#send movement velocity
def set_velocity_xyz(velocity_x, velocity_y, velocity_z):
    global vehicle
    """
    Commands the drone to move with specified velocities in the X, Y, and Z directions.

    Args:
        vehicle (mavutil.mavlink_connection): The MAVLink connection object.
        velocity_x (float): Velocity in the X direction (m/s, forward/backward).
        velocity_y (float): Velocity in the Y direction (m/s, right/left).
        velocity_z (float): Velocity in the Z direction (m/s, up/down, positive = down).

    Raises:
        ValueError: If the vehicle object is None or velocities are invalid.
        Exception: For unexpected errors during command execution.
    """
    if vehicle is None:
        raise ValueError("Drone not connected")
    if not all(isinstance(v, (int, float)) for v in [velocity_x, velocity_y, velocity_z]):
        raise ValueError(f"Invalid velocities: vx={velocity_x}, vy={velocity_y}, vz={velocity_z}. Must be numbers")

    try:
        print(f"Sending velocity command: vx={velocity_x} m/s, vy={velocity_y} m/s, vz={velocity_z} m/s")

        # Encode and send velocity command in BODY_NED frame
        msg = vehicle.mav.set_position_target_local_ned_encode(
            0,                        # Time boot (not used)
            0,                        # Target system
            0,                        # Target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Frame: Body NED (relative to drone orientation)
            3527,                       # Type mask: Enable only velocity (ignore position, accel, yaw)
            0, 0, 0,                  # X, Y, Z position (ignored)
            velocity_x, velocity_y, velocity_z,  # X, Y, Z velocity (m/s)
            0, 0, 0,                  # X, Y, Z acceleration (ignored)
            0, 0                      # Yaw, yaw rate (ignored)
        )
        vehicle.mav.send(msg)

    except Exception as e:
        print(f"Unexpected error while setting velocity (vx={velocity_x}, vy={velocity_y}, vz={velocity_z}): {e}")
        raise



def set_velocity_xyYaw(x,y,a,yaw_rate):
    try:
        print(f"Sending velocity command: vx={x} m/s, vy={y} m/s, yaw_angle={a} deg, yaw_rate={yaw_rate} deg/s")
        a = a * 3.14159 / 180
        yaw_rate = yaw_rate * 3.14159 / 180
        # This command sets the drone's velocity in the body-fixed NED (North-East-Down) frame
        # Body-fixed means the velocities are relative to the drone's current orientation:
        # - X: Forward(+)/Backward(-) relative to drone's nose
        # - Y: Right(+)/Left(-) relative to drone's sides
        # - Z: Down(+)/Up(-) relative to drone's vertical axis
        msg = vehicle.mav.set_position_target_local_ned_encode(
            0,                        # Time boot ms (0 = use system time)
            0,                        # Target system ID (0 = broadcast)
            0,                        # Target component ID (0 = broadcast)
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Reference frame: relative to drone's current orientation
            2439,                     # Type mask: 0b0000100110000111
            0, 0, 0,                 # Position X,Y,Z in meters (ignored due to mask)
            x, y, 0,                 # Velocity X,Y,Z in m/s (forward, right, down)
            0, 0, 0,                 # Acceleration X,Y,Z in m/s^2 (ignored due to mask)
            a, yaw_rate              # Yaw angle in degrees, yaw rate in deg/s
        )
        vehicle.mav.send(msg)

    except Exception as e:
        print(f"Unexpected error while setting velocity (vx={x}, vy={y}, yaw_angle/rate={a}): {e}")
        raise

def set_position_xyYaw(x,y,a,yaw_rate):
    try:
        print(f"Sending velocity command: vx={x} m/s, vy={y} m/s, yaw_angle/rate={a} deg")
        a = a * 3.14159 / 180
        yaw_rate = yaw_rate * 3.14159 / 180
        # This command sets the drone's velocity in the body-fixed NED (North-East-Down) frame
        # Body-fixed means the velocities are relative to the drone's current orientation:
        # - X: Forward(+)/Backward(-) relative to drone's nose
        # - Y: Right(+)/Left(-) relative to drone's sides
        # - Z: Down(+)/Up(-) relative to drone's vertical axis
        msg = vehicle.mav.set_position_target_local_ned_encode(
            0,                        # Time boot ms (0 = use system time)
            0,                        # Target system ID (0 = broadcast)
            0,                        # Target component ID (0 = broadcast)
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Reference frame: relative to drone's current orientation
            0b0000100110000000,                     # Type mask: 0b0000100110000111
            x, y, 0,                 # Position X,Y,Z in meters (ignored due to mask)
            0, 0, 0,                 # Velocity X,Y,Z in m/s (forward, right, down)
            0, 0, 0,                 # Acceleration X,Y,Z in m/s^2 (ignored due to mask)
            a, yaw_rate              # Yaw angle in degrees, yaw rate in deg/s
        )
        vehicle.mav.send(msg)

    except Exception as e:
        print(f"Unexpected error while setting velocity (vx={x}, vy={y}, yaw_angle/rate={a}): {e}")
        raise