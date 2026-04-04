#!/bin/bash

# Kill Gazebo and RViz processes
echo "Killing Gazebo and RViz..."

# Kill gazebo processes
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient

# Kill RViz2
pkill -f rviz2
pkill -f rviz

# Kill spawn_entity if running
pkill -f spawn_entity

echo "Done! Gazebo and RViz processes terminated."
