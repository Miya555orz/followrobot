# Dynamic cinematic reference consumed by servo_manager.
std_msgs/Header header

uint8 STATIC_TRACK=0
uint8 DOLLY_IN_OUT=1
uint8 TRUCK_LEFT_RIGHT=2
uint8 ORBIT_ARC=3

uint8 mode
int32 tracking_id
float32 desired_depth
float32 max_linear_speed

# Feed-forward chassis velocity in base_link. PBVS adds its distance/bearing
# feedback before applying the final safety limits.
geometry_msgs/Twist chassis_feedforward

float32 progress
bool allow_translation
bool valid
