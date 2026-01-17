FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pytest \
    python3-numpy  \
    ros-jazzy-py-trees \
    ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-launch-testing-ros \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

