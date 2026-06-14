#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Set vehicle name fallback
export VEHICLE_NAME=${VEHICLE_NAME:-duckiebot}

# Launch the full maze navigation stack
dt-exec roslaunch maze_path_planner maze_navigation.launch \
  veh:=${VEHICLE_NAME} \
  start_node:=${START_NODE:-S} \
  goal_node:=${GOAL_NODE:-T}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
