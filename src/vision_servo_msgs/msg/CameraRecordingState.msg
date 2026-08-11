std_msgs/Header header

uint8 UNKNOWN=0
uint8 STOPPED=1
uint8 STARTING=2
uint8 RECORDING=3
uint8 STOPPING=4
uint8 ERROR=5

uint8 state
string state_name
bool connected
bool command_pending
uint32 sdk_code
string detail

