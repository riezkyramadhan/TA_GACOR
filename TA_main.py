from TA_Yolo import tracking
from TA_Control import *
import config


def init_vehicle():
    """Initialize and connect to the vehicle"""
    print("Connecting to vehicle...")
    vehicle = connect(config.controller_address, wait_ready=True)

    if not vehicle:
        print("Connection failed, please check the connection")
        exit(1)

    # Check the arming status
    if vehicle.is_armable:
        print("Flight controller is armable.")
    else:
        print("Flight controller is not armable.")
        vehicle.wait_for_armable()

    return vehicle        

def init_program(vehicle):
    """Initialize all program components"""
    # Add message listeners
    vehicle.add_message_listener('DISTANCE_SENSOR', rangefinder_listener)
    vehicle.add_message_listener('NAV_CONTROLLER_OUTPUT', wp_dist_listener)
    
    # Set parameters
    change_param(vehicle, 2)  # Change to RangeFinder
    
    # Initialize payload systems
    dropload_init_state(vehicle)
    
    # Store initial heading
    global_variable.init_heading = vehicle.heading

def init_object_detection(vehicle):
    """Start the computer vision thread"""
    cv = Thread(target=tracking, args=(vehicle, config.video_resolution, ))
    cv.setDaemon(True)
    cv.start()
    sleep(5)

def main():
    """Main program execution"""
    print("Starting drone program...")
    
    
    # Initialize vehicle connection
    vehicle = init_vehicle()
    
    # Initialize program components
    init_program(vehicle)
    init_object_detection(vehicle)
    
    # Set camera angle
    camera_tilt_flat(vehicle)
    
    # Wait for user confirmation
    input("Press enter to start flying after tracking program is loaded...")
    print("================================= PROCEEDING OUTDOOR MISSION =================================")
    
    # Takeoff to specified height
    takeoff(vehicle, 0.7)
    
    # Verify altitude is correct
    cek_altitude(vehicle, 1.55)
    sleep(1)
    
    # Initialize current waypoint index
    current_waypoint_index = 0
    
    # First navigation until api detection
    navigation_result = navigate_waypoints_stop_on_api(vehicle, global_variable.waypoint_codes, current_waypoint_index)
    
    if navigation_result == -1:
        print("API (fire) detected during waypoint navigation")
        # Store the waypoint to resume from
        current_waypoint_index = global_variable.last_waypoint_index
        
        # Enable centering for api approach
        global_variable.do_centering = True
        
        # Find and center on the api
        find_api(vehicle)
        
        # Drop the torus once centered
        while True:
            if global_variable.do_drop:
                drop_fireball(vehicle)
                sleep(2)  # Wait for torus to fully drop
                break
        
        # Continue with the remaining waypoints
        print("Continuing with remaining waypoints")
        navigation_result = navigate_waypoints_stop_on_api(
            vehicle, global_variable.waypoint_codes, current_waypoint_index + 1
        )
    
    # Set landing flag
    global_variable.do_land = True
    
    # Land when mission complete
    land(vehicle)
    
    print("===================================== OUTDOOR MISSION COMPLETE =====================================")

if __name__ == "__main__":
    main()