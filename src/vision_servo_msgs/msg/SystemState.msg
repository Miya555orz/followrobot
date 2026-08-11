std_msgs/Header header

uint8 MODE_STANDBY=0
uint8 MODE_FOLLOW=1
uint8 MODE_CINEMATIC=2

uint8 CINEMATIC_INACTIVE=0
uint8 CINEMATIC_READY=1
uint8 CINEMATIC_EXECUTING=2

uint8 mode
uint8 cinematic_state
uint64 state_version
bool emergency_stop
string mode_name
string detail

