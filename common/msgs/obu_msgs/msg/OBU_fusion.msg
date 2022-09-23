std_msgs/Header header

# Add light information here.
int32 light_state
int32 light_remain_time
float64 light_jd
float64 light_wd
float64 light_x
float64 light_y

# Add ego vehicle information here.
float64 obu_jd
float64 obu_wd
float64 obu_angle
float64 obu_x
float64 obu_y
float64 obu_angle_rad

# Add obstacle information here.
float64 event_jd
float64 event_wd
float64 event_x
float64 event_y
float64 event_length
float64 event_radius

# Add park information here.
float64 park_jd
float64 park_wd
float64 park_x
float64 park_y

# Add emergency car here.
float64 emergency_car_jd
float64 emergency_car_wd
float64 emergency_car_x
float64 emergency_car_y
int32 emergency_car_is_near

# Add terminal point here.
float64 terminal_jd
float64 terminal_wd
float64 terminal_x
float64 terminal_y
