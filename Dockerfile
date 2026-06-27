ARG ISAAC_ROS_IMAGE=nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907
FROM ${ISAAC_ROS_IMAGE}

WORKDIR /workspace/superarm_ws

COPY . .

RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/superarm_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
