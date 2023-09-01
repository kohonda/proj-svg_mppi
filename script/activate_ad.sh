#!/bin/bash

# get location of suzlab workspace
SUZ_WS=$(cd $(dirname $0) && cd .. && pwd) # recommended to use "~/suzlab_ws"

# start autonomous driving
rostopic pub -1 /is_active_ad std_msgs/Bool "data: True"&
mpv $SUZ_WS/data/sound/activate_ad.mp3&
