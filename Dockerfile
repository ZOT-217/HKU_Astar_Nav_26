ARG ROS_BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

# ── System dependencies ─────────────────────────────────────────────
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    # PCL / Eigen / OpenCV
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    # g2o (graph optimization for hdl_graph_slam)
    ros-noetic-libg2o \
    # serial (dm_imu)
    ros-noetic-serial \
    # apr (livox_ros_driver2)
    libapr1-dev \
    # libdw (elfutils, bot_sim linker dependency)
    libdw-dev \
    # geodesy (hdl_graph_slam)
    ros-noetic-geodesy \
    # hdl_graph_slam extras
    ros-noetic-nmea-msgs \
    ros-noetic-interactive-markers \
    # bot_sim extras
    ros-noetic-costmap-converter \
    ros-noetic-teb-local-planner \
    # OpenMP (Point-LIO, hdl_graph_slam)
    libomp-dev \
    # DecisionNode dependencies
    libyaml-cpp-dev \
    libzmq3-dev \
    libsqlite3-dev \
    # Python deps (hdl_graph_slam scripts)
    python3-scipy \
    python3-progressbar \
    python3-pip \
    # Misc
    wget \
    git \
    cmake \
    build-essential \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ── Livox SDK2 (required by livox_ros_driver2) ───────────────────────
WORKDIR /tmp
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir -p build && cd build && \
    cmake .. \
      -DCMAKE_CXX_FLAGS="-w" \
      -DCMAKE_BUILD_TYPE=Release && \
    make -j1 && make install && \
    rm -rf /tmp/Livox-SDK2

# ── BehaviorTree.CPP v3 (DecisionNode dependency) ────────────────────
WORKDIR /tmp
RUN git clone --depth 1 --branch v3.8 https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    cd BehaviorTree.CPP && mkdir -p build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_UNIT_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TOOLS=OFF && \
    make -j4 && make install && ldconfig && \
    rm -rf /tmp/BehaviorTree.CPP

# ── ROS workspace skeleton ───────────────────────────────────────────
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Source ROS and workspaces for convenience
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/livox_ws/devel/setup.bash --extend 2>/dev/null" >> /root/.bashrc && \
    echo "source /workspace/sim_nav/devel/setup.bash --extend 2>/dev/null" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

# Entrypoint: start a bash shell that sources ROS automatically
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
