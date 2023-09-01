#!/bin/bash

# close window hosting gym rviz
echo "[INFO] close processes from agent_template.launch";
wmctrl -lix |grep 'agent_template.launch'|cut -d ' ' -f 1 |xargs -i% wmctrl -i -c %

# close window hosting gym environment
echo "[INFO] close processes from gym_bridge.launch";
wmctrl -lix |grep 'gym_bridge.launch'|cut -d ' ' -f 1 |xargs -i% wmctrl -i -c %
