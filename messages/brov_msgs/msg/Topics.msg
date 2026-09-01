# This file defines the topics used by the BlueROV2 Heavy (Mr. Pinchy) stack.
# The goal is to have a single location that all cpp/py scripts can easily
# access, instead of repeating topic strings in every node.
#
# Unlike most vehicles in smarc2, the BlueROV2 talks to the rest of the system
# through mavros, so these names mirror the mavros topic layout.

####################
# Flight controller I/O (mavros)
#
# Relative names: they resolve inside whatever namespace the node is pushed
# into, the same way mavros' own topics do.
####################
string RC_OVERRIDE_TOPIC = 'mavros/rc/override'    # Type: mavros_msgs/OverrideRCIn
string MAVROS_STATE_TOPIC = 'mavros/state'         # Type: mavros_msgs/State

####################
# Vehicle-namespaced topics.
#
# TODO: the namespace here needs to come from the sim vehicle name
####################
string RC_IN_TOPIC = '/BlueROV2_Heavy/mavros/rc/in'         # Type: mavros_msgs/RCIn
string IMU_TOPIC = '/BlueROV2_Heavy/mavros/imu/data_raw'    # Type: sensor_msgs/Imu
