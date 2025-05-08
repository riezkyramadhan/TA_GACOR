# Description: Configuration file for the robot

# Pinout configuration
magnet_pin = 8
servo_1_pin = 7
servo_2_pin = 8

# PWM values configuration
magnet_release_pwm = 1807
magnet_hold_pwm = 1039
magnet_drop_pwm = 988
servo_1_hold_pwm = 2012
servo_2_hold_pwm = 1039
servo_1_drop_pwm = 988
servo_2_drop_pwm = 1807

# Video properties
video_resolution = (640, 480)

# Connection properties
controller_address ="udp:127.0.0.1:14550"
baud_rate = 115200

# Servo properties
servo_1 = 7
servo_2 = 8
servo_pwm = 2012

# Dropload properties
dropload_pwm = 2012
dropload_pwm_drop = 988
dropload_pwm_hold = 1039
dropload_pwm_release = 1807

# Tracking properties
tracking = True
tracking_delay = 10

# Takeoff properties
takeoff_height = 0.7

# Velocity properties
velocity_x = 0.05
velocity_y = 0.05
velocity_z = 0.0
velocity_delay = 0

# Grid properties
grid_1 = (0.05, -0.05, 0)
grid_2 = (0.05, 0, 0)
grid_3 = (0.05, 0.1, 0)
grid_4 = (0, -0.05, 0)
grid_5 = (0, 0.05, 0)
grid_6 = (-0.05, -0.05, 0)
grid_7 = (-0.05, 0, 0)
grid_8 = (-0.05, 0.05, 0)
grid_default = (0.2, 0, 0)

# Sleep properties
sleep_time = 1.0 / 30
sleep_time_tracking = 0
sleep_time_takeoff = 0
sleep_time_grid = 0
sleep_time_default = 0