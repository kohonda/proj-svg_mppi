#!/bin/bash

# get location of suzlab workspace
SUZ_WS=$(cd $(dirname $0) && cd .. && pwd) # recommended to use "~/suzlab_ws"

# start autonomous driving
rostopic pub -1 /is_active_ad std_msgs/Bool "data: False"&
mpv $SUZ_WS/data/sound/deactivate_ad.mp3&
