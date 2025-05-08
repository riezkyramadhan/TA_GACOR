object_detected = False
object_found = False


do_centering = False
do_drop = False
do_land = False

center_object_last_grid = -1


  # Using this to count api (fire) detections
api_counter = 0
fireball_dropped = False
fireball_carried = False
last_waypoint_index = 0
xy_ok = 0
rngfnd = {}
current_wp_dist = 0
init_wp_dist = 0

xy_ok = 0
yz_ok = 0




waypoint_codes = {1: {"latitude" : -7.0531691,
                          "longitude": 110.43957053,
                          "altitude" : 2.5},
                      2: {"latitude" : -7.0533274,
                          "longitude": 110.4395096,
                          "altitude" : 2.5},
                      3: {"latitude" : -7.0532514,
                          "longitude": 110.4394875,
                          "altitude" : 2.5}}
