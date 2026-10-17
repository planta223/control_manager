uint8 IDLE=0
uint8 MANUAL=1
uint8 AUTO=2
uint8 INIT=0
uint8 READY=1
uint8 DRIVING=2
uint8 STOPPED=3
uint8 FAULT=4
# Selector mode / selected-command FSM; does not report Vehicle Interface protection or measured motion.
std_msgs/Header header
uint8 mode
uint8 fsm_state
string reason
