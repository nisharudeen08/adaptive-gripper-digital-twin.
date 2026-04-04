FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive

# Minimal deps to build + run rosbridge for this workspace.
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-rosbridge-suite \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

# Build only the rosbridge launch package for Render. The rest of the repo
# includes Gazebo/RViz dependencies that don't run well in a PaaS container.
RUN source /opt/ros/humble/setup.bash \
  && colcon build --packages-select robotic_control

COPY render_entrypoint.sh /render_entrypoint.sh
RUN chmod +x /render_entrypoint.sh

ENV ROS_LOG_DIR=/tmp/ros_logs

CMD ["/render_entrypoint.sh"]

