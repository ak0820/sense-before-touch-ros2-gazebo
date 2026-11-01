# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:99 \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    LIBGL_DRI3_DISABLE=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Gazebo Harmonic is provided through ROS 2 Jazzy vendor packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    xvfb x11vnc novnc websockify openbox x11-utils \
    mesa-utils libgl1-mesa-dri libglu1-mesa libegl1 \
    libxrender1 libxext6 libsm6 fonts-dejavu-core \
    bash ca-certificates procps && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/pai_ws
COPY sense_before_touch_ros2 /opt/pai_ws/src/sense_before_touch_ros2
COPY tests /opt/pai_ws/tests
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install --event-handlers console_direct+'

COPY scripts/container_entrypoint.sh /opt/pai_ws/container_entrypoint.sh
COPY scripts/ai_dashboard.py /opt/pai_ws/ai_dashboard.py
COPY scripts/dashboard /opt/pai_ws/dashboard
RUN chmod +x /opt/pai_ws/container_entrypoint.sh && \
    mkdir -p /recordings /tmp/runtime-root && chmod 700 /tmp/runtime-root

EXPOSE 6080 6081
ENTRYPOINT ["/opt/pai_ws/container_entrypoint.sh"]
