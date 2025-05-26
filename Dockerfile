FROM videobase:latest

 ENV DEBIAN_FRONTEND noninteractive

COPY src /opt/ws/src
# As a temporal solution, delete the venv. Later it should be placed outside ros2 ws
RUN rm -rf /opt/ws/venv

# Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd /opt/ws && \
    colcon build

# Entrypoint
COPY docker-entrypoint.sh /usr/bin
RUN chmod +x /usr/bin/docker-entrypoint.sh

ENTRYPOINT [ "bash", "/usr/bin/docker-entrypoint.sh" ]