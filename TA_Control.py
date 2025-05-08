import collections
from collections import abc
collections.MutableMapping = abc.MutableMapping

from dronekit import connect, VehicleMode
from pymavlink import mavutil
from time import sleep
from math import radians
import global_variable
from threading import Thread
import time


def set_servo(vehicle, servo_number, pwm_value):
    to_send = vehicle.message_factory.command_long_encode(
                    0, 0,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
                    0, 
                    servo_number, pwm_value, 
                    0, 0, 0, 0, 0)
    
    vehicle.send_mavlink(to_send)

def dropload_init_state(vehicle):
    set_servo(vehicle, 9, 1704) # Lock Fireball
    set_servo(vehicle, 10, 1000) # Camera Up 45 derajat
   

def drop_object_baru(vehicle): # Drop Torus
    print("Dropping Object")
    set_servo(vehicle, 9, 989)

def release_blok_left(vehicle): # Drop Left Payload
    print("Dropping Blok Left")
    set_servo(vehicle, 12, 2011)   

def release_blok_right(vehicle): # Drop Right Payload
    print("Dropping Blok Right")
    set_servo(vehicle, 11, 2011) 

def camera_up(vehicle): # Camera Up for Exit
    print("Camera Up")
    set_servo(vehicle, 10, 2000) #1900

def camera_tilt_up(vehicle): # Camera Up to Avoid Misread Torus - Basket
    print("Camera Tilt Up")
    set_servo(vehicle, 10, 1300)

def camera_tilt_basket(vehicle): # Camera Tilt for Basket
    print("Camera Tilt Basket")
    set_servo(vehicle, 10, 1000)

def camera_tilt_flat(vehicle): #  Camera Flat
    print("Camera Tilt Flat")
    set_servo(vehicle, 10, 910)

def camera_tilt_down(vehicle): # Camera Down for Torus
    print("Camera Tilt Down")
    set_servo(vehicle, 10, 750)   

def camera_tilt_outdoor(vehicle): # Camera Up for Outdoor Target
    print("Camera Tilt Outdoor")
    set_servo(vehicle, 10, 1300)

def camera_tilt_rotate(vehicle): # Camera Up for Searching
    print("Camera Tilt Rotate")
    set_servo(vehicle, 10, 1000)


def change_mode(vehicle, flight_mode: str):
    vehicle.mode = VehicleMode(flight_mode)
    # Wait until mode changed
    vehicle.wait_for_mode(flight_mode)

    if vehicle.mode.name == flight_mode:
        print("Mode changed to", flight_mode)
    else:
        print("Failed to Change Mode")


def arm(vehicle):
    print("Arming motors")
    vehicle.arm(wait=True) # Wait until armed
    print("Vehicle Armed")


def disarm(vehicle):
    print("Disarming motors")
    vehicle.disarm(wait=True) # Wait until disarmed
    print("Vehicle Disarmed")


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    to_send = vehicle.message_factory.set_position_target_local_ned_encode(
                    10, # time_boot_ms
                    0, # DroneKit will automatically update the value with the correct ID for the connected vehicle
                    0, # Not updated by DroneKit, but should be set to 0 (broadcast) unless the message is really intended for a specific component
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # Frame
                    0b0000011111000111,  # Typemask
                    0, 0, 0,  # XYZ Position (m)
                    velocity_x, velocity_y, velocity_z,
                    # XYZ Velocity (m/s)
                    0, 0, 0,  # XYZ Acceleration (m/s/s)
                    0,  # Yaw setpoint (rad)
                    0  # Yaw rate (rad/s)
                    )
    # Send command to vehicle
    vehicle.send_mavlink(to_send)


def send_global_position(vehicle, code):
    latitude = global_variable.waypoint_item_left[code]["latitude"]
    longitude = global_variable.waypoint_item_left[code]["longitude"]
    altitude = global_variable.waypoint_item_left[code]["altitude"]
    confirmation = 0
    global_variable.api_counter = 0

    while True:
        if global_variable.fireball_carried == False: # Basket
            to_send = vehicle.message_factory.set_position_target_global_int_encode(10, 0, 0, 
                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame
                              0b110111111000,  # Typemask
                              int(latitude * 10**7), 
                              int(longitude * 10**7), 
                              altitude,
                              0, 0, 0, # Velocity
                              0, 0, 0, # Acceleration
                              0, 0) # Yaw
            vehicle.send_mavlink(to_send)
            object_counter = global_variable.api_counter
            confirmation = 37
        
        print(f"OBJECT: {object_counter}")

        # Stop if object detected for certain counts
        if object_counter >= confirmation:
            global_variable.center_object_last_grid = -1 # reset
            print("======================================= STOP =======================================")
            break
    
    


def override_global_position(vehicle):
    to_send = vehicle.message_factory.command_long_encode(0, 0, 
                              mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO, 
                              0, 
                              mavutil.mavlink.MAV_GOTO_DO_HOLD, 
                              mavutil.mavlink.MAV_GOTO_HOLD_AT_CURRENT_POSITION, 
                              0, 0, 0, 0, 0) 
    vehicle.send_mavlink(to_send)


def condition_yaw(vehicle, target_heading, direction, yaw_speed=30):
    to_send = vehicle.message_factory.command_long_encode(0, 0,
                                  mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                                  0,  # Confirmation
                                  target_heading,  # Yaw in (deg)
                                  yaw_speed,  # Yaw speed (deg/s)
                                  direction,  # -1=ccw, 1=cw
                                  1,  # 0=absolute angle, 1=relative angle
                                  0, 0, 0)
    # Send command to vehicle
    vehicle.send_mavlink(to_send)
    
    # Wait to ensure yaw is completed
    sleep(target_heading / yaw_speed)


def rangefinder_listener(self, name, message):
    # message: DISTANCE_SENSOR {time_boot_ms : 435245, min_distance : 4, max_distance : 2500, current_distance : 40, type : 0, id : 10, orientation : 0, covariance : 0, horizontal_fov : 0.0, vertical_fov : 0.0, quaternion : [0.0, 0.0, 0.0, 0.0], signal_quality : 0}
    orientation = message.orientation

    global_variable.rngfnd[str(orientation)]={}
    global_variable.rngfnd[str(orientation)]["id"] = message.id
    global_variable.rngfnd[str(orientation)]["current_distance"] = message.current_distance


def wp_dist_listener(self, name, message):
    if global_variable.init_wp_dist != 0:
        global_variable.current_wp_dist = message.wp_dist
    else:
        if message.wp_dist != 0:
            global_variable.current_wp_dist = message.wp_dist
        global_variable.init_wp_dist = message.wp_dist


def takeoff(vehicle: object, target_altitude: float) -> object:
    change_mode(vehicle, "GUIDED")
    arm(vehicle)

    while not vehicle.armed:      
        print("Waiting for arming...")
        sleep(0.25)

    print("Taking off..")
    vehicle.simple_takeoff(target_altitude)


def cek_altitude(vehicle, target_altitude):
    altitude_counter = 0
    while True:
        try:
            current_altitude = global_variable.rngfnd["25"]["current_distance"] / 100
        except KeyError:
            current_altitude = target_altitude - 0.2

        print(f"Altitude: {current_altitude} m of {target_altitude} m")      

        if vehicle.system_status == "ACTIVE":
            if current_altitude < 0.95 * target_altitude: 
                send_ned_velocity(vehicle, 0, 0, -0.25)
                print("Naik to reach altitude")
            
            if current_altitude > 1.1 * target_altitude: 
                send_ned_velocity(vehicle, 0, 0, 0.3)
                print("Turun to reach altitude")

            if 0.95 * target_altitude <= current_altitude <= 1.1 * target_altitude:
                altitude_counter += 1
                print(f"ALT: {altitude_counter}")
                # Stop if altitude reached for 3 counts
                if altitude_counter >= 3:
                    print("Target Altitude Reached")
                    break


def cek_heading(vehicle):
    heading_reference = global_variable.init_heading
    current_heading = vehicle.heading # degree (int)

    if current_heading < 0.98 * heading_reference:
        target_heading = heading_reference - current_heading
        direction = 1 # CW
        if target_heading > 180:
            target_heading -= 180
            direction = -1
        condition_yaw(vehicle, target_heading, direction)
        sleep(2)

    elif current_heading > 1.02 * heading_reference:
        target_heading = current_heading - heading_reference
        direction = -1 # CCW
        if target_heading > 180:
            target_heading -= 180
            direction = 1
        condition_yaw(vehicle, target_heading, direction)
        sleep(2)

    if 0.98 * heading_reference <= current_heading <= 1.02 * heading_reference:
        print("------------------------- HEADING OK -------------------------")


def land(vehicle):
    print("Landing..")
    change_mode(vehicle, "LAND")

    while True:
        try:
            current_altitude = global_variable.rngfnd["25"]["current_distance"] / 100
        except KeyError:
            current_altitude = 0.2

        current_status = vehicle.system_status
        print(f"Altitude: {current_altitude} m of 0 m")

        if current_status == "STANDBY":
            print("Vehicle landed")
            break

def navigate_waypoints_stop_on_api(vehicle, waypoint_codes, start_index=0):
    """
    Navigates vehicle through a list of waypoints, stops if api (fire) is detected.
    Returns the index of the last waypoint visited or -1 if stopped due to api detection.
    """
    # Reset api detection counter 
    global_variable.api_counter = 0  # This is actually counting api detections
    api_detection_threshold = 3  # Number of detections to confirm api presence
    
    for i in range(start_index, len(waypoint_codes)):
        code = waypoint_codes[i]
        print(f"Navigating to waypoint {i+1}/{len(waypoint_codes)}: Code {code}")
        
        # Get waypoint coordinates
        latitude = global_variable.waypoint_item_left[code]["latitude"]
        longitude = global_variable.waypoint_item_left[code]["longitude"]
        altitude = global_variable.waypoint_item_left[code]["altitude"]
        
        # Set waypoint reached flag
        waypoint_reached = False
        
        # Navigate to waypoint while checking for api
        start_time = time.time()
        
        while not waypoint_reached:
            # Send position command to move toward waypoint
            to_send = vehicle.message_factory.set_position_target_global_int_encode(
                10,                 # time_boot_ms
                0, 0,               # target system, target component
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
                0b110111111000,     # Typemask
                int(latitude * 10**7),  # lat
                int(longitude * 10**7), # lon
                altitude,           # alt
                0, 0, 0,            # Velocity
                0, 0, 0,            # Acceleration
                0, 0)               # Yaw
            vehicle.send_mavlink(to_send)
            
            # Check if api detected
            if global_variable.api_counter >= api_detection_threshold:
                print("======================================= API (FIRE) DETECTED - STOPPING MISSION =======================================")
                global_variable.center_object_last_grid = -1  # reset
                global_variable.last_waypoint_index = i  # Store the current waypoint index to resume later
                return -1  # Indicate stopped due to api detection
            
            # Check if waypoint reached (either by distance or timeout)
            current_location = vehicle.location.global_relative_frame
            distance_to_waypoint = get_distance_metres(
                current_location.lat, 
                current_location.lon, 
                latitude, 
                longitude
            )
            
            if distance_to_waypoint < 2.0:  # Within 2 meters considered reached
                waypoint_reached = True
                print(f"Waypoint {i+1} reached")
            
            # Alternative: timeout after certain time
            if time.time() - start_time > 30:  # 30 second timeout per waypoint
                print(f"Timeout reaching waypoint {i+1}, continuing to next")
                waypoint_reached = True
            
            # Small delay to prevent flooding the vehicle with commands
            time.sleep(0.1)
    
    print("All waypoints navigated successfully")
    return len(waypoint_codes) - 1  # Return index of last waypoint

def get_distance_metres(lat1, lon1, lat2, lon2):
    """
    Calculate distance between two sets of coordinates
    """
    from math import sin, cos, sqrt, atan2, radians
    
    # Approximate radius of earth in meters
    R = 6371000.0
    
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)
    
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    
    distance = R * c
    return distance
    

def find_api(vehicle):
    while True:
        sleep(1.0/30)
        if global_variable.object_detected == False:
            if global_variable.center_object_last_grid==1:
                send_ned_velocity(vehicle, 0.07, -0.3, 0)
            elif global_variable.center_object_last_grid==2:
                send_ned_velocity(vehicle, 0.07, 0, 0)
            elif global_variable.center_object_last_grid==3:
                send_ned_velocity(vehicle, 0.07, 0.3, 0)
            elif global_variable.center_object_last_grid==4:
                send_ned_velocity(vehicle, 0, -0.07, 0)
            elif global_variable.center_object_last_grid==5:
                send_ned_velocity(vehicle, 0, 0.07, 0)
            elif global_variable.center_object_last_grid==6:
                send_ned_velocity(vehicle, -0.08, -0.07, 0)
            elif global_variable.center_object_last_grid==7:
                send_ned_velocity(vehicle, -0.08, 0 ,0)
            elif global_variable.center_object_last_grid==8:
                send_ned_velocity(vehicle, -0.08, 0.07 ,0)
            else:
                send_ned_velocity(vehicle, 0.2, 0, 0)
            print(f"LAST GRID: {global_variable.center_object_last_grid}")
        else:
            pass
        sleep(0)

        if global_variable.object_found == True:
            global_variable.object_detected = False 
            break
    global_variable.object_found = False 




def drop_fireball(vehicle):
    print("Dropping torus")
    drop_object_baru(vehicle) # Servo
    print("Torus Dropped")
    sleep(2)
    global_variable.fireball_dropped = True


def change_param(vehicle, value):
    # Change EK3_SRC1_POSZ parameter value
    vehicle.parameters["EK3_SRC1_POSZ"] = value # 1: Baro, 2: RangeFinder, 3: GPS 

    # Parameter Check
    while True:
        param_value = vehicle.parameters["EK3_SRC1_POSZ"]
        if param_value == value:
            print(f"Name: EK3_SRC1_POSZ\tValue: {param_value}")
            print("============================= PARAMETER CHANGED =============================")
            break      

